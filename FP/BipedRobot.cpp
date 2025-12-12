#include "BipedRobot.h"
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

BipedRobot::BipedRobot(mjModel* model, mjData* data) : m(model), d(data) {
    // 缓存关节 ID
    id_hip_left = mj_name2id(m, mjOBJ_JOINT, "left_hip");
    id_knee_left = mj_name2id(m, mjOBJ_JOINT, "left_knee");
    id_hip_right = mj_name2id(m, mjOBJ_JOINT, "right_hip");
    id_knee_right = mj_name2id(m, mjOBJ_JOINT, "right_knee");

    // 缓存几何体和Site ID
    id_floor_geom_ = mj_name2id(m, mjOBJ_GEOM, "floor");
    id_left_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "left_shin_geom");
    id_right_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "right_shin_geom");
    id_left_foot_site_ = mj_name2id(m, mjOBJ_SITE, "left_foot");
    id_right_foot_site_ = mj_name2id(m, mjOBJ_SITE, "right_foot");

    warning_msg_ = "";
    is_violated_ = false;

    // 初始化状态机
    current_state_ = RobotState::IDLE;
    walk_start_time_ = 0.0;
}

void BipedRobot::enforceTorqueLimits(std::vector<double>& torques) {
    const double HIP_TORQUE_LIMIT = 30.0;
    const double KNEE_TORQUE_LIMIT = 60.0;

    if (torques.size() >= 4) {
        torques[0] = std::max(-HIP_TORQUE_LIMIT, std::min(torques[0], HIP_TORQUE_LIMIT));
        torques[1] = std::max(-KNEE_TORQUE_LIMIT, std::min(torques[1], KNEE_TORQUE_LIMIT));
        torques[2] = std::max(-HIP_TORQUE_LIMIT, std::min(torques[2], HIP_TORQUE_LIMIT));
        torques[3] = std::max(-KNEE_TORQUE_LIMIT, std::min(torques[3], KNEE_TORQUE_LIMIT));
    }
}

void BipedRobot::step() {
    if (is_violated_) return;
    mj_step(m, d);
    checkConstraints();
}

void BipedRobot::freeFall() {
    if (current_state_ != RobotState::IDLE) {
        current_state_ = RobotState::IDLE;
    }
    mju_zero(d->ctrl, m->nu);
    step();
}

void BipedRobot::stand(double target_x, double target_z, double target_pitch, double duration) {
    if (current_state_ != RobotState::STAND) {
        current_state_ = RobotState::STAND;
        std::cout << "[FSM] Switch to STAND at t=" << d->time << std::endl;
    }

    bool left_grounded = false;
    bool right_grounded = false;

    for (int i = 0; i < d->ncon; ++i) {
        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;
        if ((g1 == id_left_shin_geom_ && g2 == id_floor_geom_) || (g2 == id_left_shin_geom_ && g1 == id_floor_geom_)) left_grounded = true;
        if ((g1 == id_right_shin_geom_ && g2 == id_floor_geom_) || (g2 == id_right_shin_geom_ && g1 == id_floor_geom_)) right_grounded = true;
    }

    if (left_grounded && right_grounded) {
        std::vector<double> torques = controller_.computeStandControl(m, d, target_x, target_z, target_pitch, duration);
        enforceTorqueLimits(torques);
        for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];
    } else {
        mju_zero(d->ctrl, m->nu);
    }
    step();
}

// [修改] 迈步任务实现：应用动力学分析后的修正
void BipedRobot::walk() {
    double t_now = d->time;

    // 状态机初始化与切换逻辑
    if (current_state_ != RobotState::WALK_WEIGHT_SHIFT &&
        current_state_ != RobotState::WALK_SWING &&
        current_state_ != RobotState::WALK_LAND) {

        current_state_ = RobotState::WALK_WEIGHT_SHIFT;
        walk_start_time_ = t_now;
        std::cout << "[FSM] Start WALK task. Phase: WEIGHT SHIFT at t=" << t_now << std::endl;
    }

    RobotController::WalkCommand cmd;
    // 默认保持参数 (与 Stand 一致)
    cmd.trunk_z_des = 0.48;
    cmd.trunk_pitch_des = 0.0;
    cmd.trunk_x_vel_des = 0.0;

    // 阶段参数
    // [修正] 增加摆动时间以减小加速度峰值
    double T_shift = 0.5; // 重心转移时间
    double T_swing = 0.8; // 摆动时间 (原 0.6，增加到 0.8 减缓冲击)

    double t_task = t_now - walk_start_time_;

    // FSM Logic
    if (current_state_ == RobotState::WALK_WEIGHT_SHIFT) {
        // Phase 1: 重心转移
        cmd.left_contact = true;
        cmd.right_contact = true;

        // 关键：将躯干 X 向右(前)推一点，使左脚变轻
        cmd.trunk_x_vel_des = 0.1;

        // [修正] 允许 Pitch 前倾，利用重力辅助起步
        cmd.trunk_pitch_des = 0.1; // 约 5.7度前倾

        // 权重配置：双脚硬接触，摆动权重为0
        cmd.w_trunk_z = 300;
        cmd.w_trunk_pitch = 300;
        // [修正] 增加 X 权重，确保重心真的移过去
        cmd.w_trunk_x = 100.0;
        cmd.w_swing_pos = 0.0;

        if (t_task > T_shift) {
            current_state_ = RobotState::WALK_SWING;
            // 记录摆动初始点
            swing_init_pos_ << d->site_xpos[3*id_left_foot_site_],
                               d->site_xpos[3*id_left_foot_site_+1],
                               d->site_xpos[3*id_left_foot_site_+2];
            // 设定目标点：向前迈 0.2m
            swing_target_pos_ = swing_init_pos_;
            swing_target_pos_(0) += 0.2; // 向前 0.2m
            std::cout << "[FSM] Switch to SWING at t=" << t_now << std::endl;
        }

    } else if (current_state_ == RobotState::WALK_SWING) {
        // Phase 2: 左脚摆动
        cmd.left_contact = false;
        cmd.right_contact = true;

        double t_swing_curr = t_task - T_shift;
        double s = std::min(t_swing_curr / T_swing, 1.0);

        // 贝塞尔曲线规划
        cmd.swing_pos_des = getBezierPos(s, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, swing_init_pos_, swing_target_pos_);

        // 权重配置
        cmd.w_trunk_z = 300;
        cmd.w_trunk_pitch = 200; // [修正] 降低 Pitch 权重，允许身体为了平衡 X 动量而微调
        cmd.w_trunk_x = 100.0;   // [修正] 保持 X 权重，对抗后坐力
        cmd.w_swing_pos = 300;   // 强追踪摆动轨迹

        // [修正] 维持前倾姿态和前向速度
        cmd.trunk_pitch_des = 0.1;
        cmd.trunk_x_vel_des = 0.2;

        if (s >= 1.0) {
            current_state_ = RobotState::WALK_LAND;
            std::cout << "[FSM] Switch to LAND at t=" << t_now << std::endl;
        }

    } else if (current_state_ == RobotState::WALK_LAND) {
        // Phase 3: 落地平衡
        cmd.left_contact = true;
        cmd.right_contact = true;

        // 恢复直立，停止向前
        cmd.trunk_pitch_des = 0.0;
        cmd.trunk_x_vel_des = 0.0;

        cmd.w_trunk_z = 300;
        cmd.w_trunk_pitch = 300;
        cmd.w_trunk_x = 100.0; // 保持阻尼
        cmd.w_swing_pos = 0.0;
    }

    // 调用新的控制器
    std::vector<double> torques = controller_.computeWalkStepControl(m, d, cmd);
    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];

    step();
}

// 贝塞尔曲线辅助函数
Eigen::Vector3d BipedRobot::getBezierPos(double s, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    // 简单的3阶贝塞尔，中间点抬高 Z
    Eigen::Vector3d p1 = p0; p1(2) += 0.1; // 抬高 10cm
    Eigen::Vector3d p2 = p3; p2(2) += 0.1;

    double u = 1 - s;
    return u*u*u*p0 + 3*u*u*s*p1 + 3*u*s*s*p2 + s*s*s*p3;
}

Eigen::Vector3d BipedRobot::getBezierVel(double s, double T, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1 = p0; p1(2) += 0.1;
    Eigen::Vector3d p2 = p3; p2(2) += 0.1;

    double u = 1 - s;
    // dP/ds
    Eigen::Vector3d dPds = 3*u*u*(p1-p0) + 6*u*s*(p2-p1) + 3*s*s*(p3-p2);
    return dPds / T;
}

Eigen::Vector3d BipedRobot::getBezierAcc(double s, double T, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1 = p0; p1(2) += 0.1;
    Eigen::Vector3d p2 = p3; p2(2) += 0.1;

    double u = 1 - s;
    // d2P/ds2
    Eigen::Vector3d d2Pds2 = 6*u*(p2 - 2*p1 + p0) + 6*s*(p3 - 2*p2 + p1);
    return d2Pds2 / (T*T);
}

void BipedRobot::resetToKeyframe() {
    is_violated_ = false;
    warning_msg_ = "";
    current_state_ = RobotState::IDLE;
    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
        mj_forward(m, d);
    }
}

void BipedRobot::resetToState(const std::vector<double>& qpos) {
    is_violated_ = false;
    warning_msg_ = "";
    current_state_ = RobotState::IDLE;
    mj_resetData(m, d);
    for (size_t i = 0; i < qpos.size() && i < (size_t)m->nq; ++i) {
        d->qpos[i] = qpos[i];
    }
    mj_forward(m, d);
}

void BipedRobot::checkConstraints() {
    warning_msg_ = "";
    const double HIP_ANGLE_MIN = deg2rad(-120);
    const double HIP_ANGLE_MAX = deg2rad(30);
    const double KNEE_ANGLE_MIN = deg2rad(0);
    const double KNEE_ANGLE_MAX = deg2rad(160);
    const double VEL_LIMIT = 30.0;

    auto check = [&](int id, std::string n, double min_q, double max_q) {
        int q_adr = m->jnt_qposadr[id];
        int v_adr = m->jnt_dofadr[id];
        double q = -d->qpos[q_adr]; // 模型定义反向
        double v = d->qvel[v_adr];
        if (q < min_q || q > max_q || std::abs(v) > VEL_LIMIT) {
            // warning_msg_ += "Limit " + n + "\n";
            // is_violated_ = true;
        }
    };
    check(id_hip_left, "L_Hip", HIP_ANGLE_MIN, HIP_ANGLE_MAX);
    check(id_knee_left, "L_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX);
    check(id_hip_right, "R_Hip", HIP_ANGLE_MIN, HIP_ANGLE_MAX);
    check(id_knee_right, "R_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX);
}