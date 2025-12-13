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

    // [新增] 初始化状态日志
    state_log_file_.open("../robot_state.csv");
    if (state_log_file_.is_open()) {
        state_log_file_ << "Time";
        // 7 DoF Pos (qpos)
        state_log_file_ << ",q_x,q_z,q_pitch,q_lh,q_lk,q_rh,q_rk";
        // 7 DoF Vel (qvel)
        state_log_file_ << ",v_x,v_z,v_pitch,v_lh,v_lk,v_rh,v_rk";
        // Left Foot (Pos + Vel)
        state_log_file_ << ",lf_x,lf_y,lf_z,lf_vx,lf_vy,lf_vz";
        // Right Foot (Pos + Vel)
        state_log_file_ << ",rf_x,rf_y,rf_z,rf_vx,rf_vy,rf_vz";
        state_log_file_ << "\n";
    } else {
        std::cerr << "Failed to open robot_state.csv" << std::endl;
    }
}

// [新增] 析构函数
BipedRobot::~BipedRobot() {
    if (state_log_file_.is_open()) {
        state_log_file_.close();
    }
}

// [新增] 埋点函数：记录详细状态
void BipedRobot::logState() {
    if (!state_log_file_.is_open()) return;

    state_log_file_ << d->time;

    // 1. 记录 7 DoF 关节位置
    for (int i = 0; i < m->nq; ++i) {
        state_log_file_ << "," << d->qpos[i];
    }
    // 2. 记录 7 DoF 关节速度
    for (int i = 0; i < m->nv; ++i) {
        state_log_file_ << "," << d->qvel[i];
    }

    // 3. 计算并记录脚部状态
    auto log_foot = [&](int site_id) {
        // 位置直接从 site_xpos 获取
        double* pos = d->site_xpos + 3 * site_id;
        state_log_file_ << "," << pos[0] << "," << pos[1] << "," << pos[2];

        // 速度需要通过雅可比矩阵计算: v = J * qvel
        // 分配雅可比矩阵内存 (3 x nv)
        std::vector<mjtNum> J(3 * m->nv);
        mj_jacSite(m, d, J.data(), nullptr, site_id);

        double vx = 0, vy = 0, vz = 0;
        for (int i = 0; i < m->nv; ++i) {
            vx += J[0 * m->nv + i] * d->qvel[i];
            vy += J[1 * m->nv + i] * d->qvel[i];
            vz += J[2 * m->nv + i] * d->qvel[i];
        }
        state_log_file_ << "," << vx << "," << vy << "," << vz;
    };

    log_foot(id_left_foot_site_);
    log_foot(id_right_foot_site_);

    state_log_file_ << "\n";
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

    // [新增] 每次物理步进后记录状态
    logState();

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

// [修改] 迈步任务实现：应用"蓄力爆发 + Raibert Heuristic"策略
void BipedRobot::walk(double target_x_vel) {
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
    // 默认保持参数
    cmd.trunk_x_des = d->qpos[0];  // 默认跟随当前 X 位置 (位置误差=0)
    cmd.trunk_z_des = d->qpos[1];  // 当前 Z 高度

    // 阶段参数
    // Phase 1 稍微缩短，Phase 2 保持平滑
    double T_shift = 0.4;
    double T_swing = 0.8;

    double t_task = t_now - walk_start_time_;

    // FSM Logic
    if (current_state_ == RobotState::WALK_WEIGHT_SHIFT) {
        // Phase 1: 蓄力爆发 (双脚支撑)
        cmd.left_contact = true;
        cmd.right_contact = true;

        // [策略核心]
        // 利用后脚还在地上的机会，疯狂加速！
        // 只有双脚都在，才能产生巨大的向前推力而不翻车
        cmd.trunk_x_vel_des = target_x_vel; // 使用入参目标速度

        // 权重配置：X 轴权重拉满
        cmd.w_trunk_z = 200;
        cmd.w_trunk_pitch = 200;
        cmd.w_trunk_x = 500.0; // 这里的优先级必须最高！
        cmd.w_swing_pos = 0.0;

        // 稍微前倾，辅助加速
        cmd.trunk_pitch_des = 0.05;

        if (t_task > T_shift) {
            current_state_ = RobotState::WALK_SWING;
            // 记录摆动初始点
            swing_init_pos_ << d->site_xpos[3*id_left_foot_site_],
                               d->site_xpos[3*id_left_foot_site_+1],
                               d->site_xpos[3*id_left_foot_site_+2];

            // [关键修正] 计算落脚点时必须包含身体的预估位移 (Raibert Heuristic)
            // 目标位置 = 初始位置 + 身体位移 + 实际步长
            // 身体预估位移 = 目标速度 * 摆动时间
            swing_target_pos_ = swing_init_pos_;
            double estimated_body_dist = cmd.trunk_x_vel_des * T_swing; // 0.6 * 0.8 = 0.48m
            double step_length = 0.25; // 物理步长
            swing_target_pos_(0) += estimated_body_dist + step_length;

            std::cout << "[FSM] Switch to SWING at t=" << t_now
                      << ". Target Step (Relative): " << (estimated_body_dist + step_length)
                      << " [Body Est: " << estimated_body_dist << " + Step: " << step_length << "]" << std::endl;
        }

    } else if (current_state_ == RobotState::WALK_SWING) {
        // Phase 2: 惯性滑行 (单脚支撑)
        cmd.left_contact = false;
        cmd.right_contact = true;

        double t_swing_curr = t_task - T_shift;
        double s = std::min(t_swing_curr / T_swing, 1.0);
        double trunk_vel = d->qvel[0];  // 使用真实躯干速度

        // 贝塞尔曲线规划 - 传入 T_swing 和真实躯干速度
        cmd.swing_pos_des = getBezierPos(s, T_swing, trunk_vel, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, trunk_vel, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, trunk_vel, swing_init_pos_, swing_target_pos_);

        // 权重配置：此时姿态最重要
        cmd.w_trunk_z = 300;
        cmd.w_trunk_pitch = 500.0; // 锁死 Pitch，防止扑街
        cmd.w_trunk_x = 10.0;      // [策略核心] 此时放弃 X 加速，允许自然滑行/减速
        cmd.w_swing_pos = 200;     // 柔顺摆腿

        // 目标速度稍微降低，允许减速
        cmd.trunk_x_vel_des = 0.0;
        cmd.trunk_pitch_des = 0.05; // 保持微前倾

        if (s >= 1.0) {
            current_state_ = RobotState::WALK_LAND;
            land_x_pos_ = d->qpos[0];  // 记录落地时的 X 位置
            std::cout << "[FSM] Switch to LAND at t=" << t_now << ", lock X=" << land_x_pos_ << std::endl;
        }

    } else if (current_state_ == RobotState::WALK_LAND) {
        // Phase 3: 落地刹车
        cmd.left_contact = true;
        cmd.right_contact = true;

        // 锁定位置 + 刹车
        cmd.trunk_x_des = land_x_pos_;  // 使用锁定的 X 位置
        cmd.trunk_x_vel_des = 0.0;
        cmd.trunk_pitch_des = 0.0;

        cmd.w_trunk_z = 300;
        cmd.w_trunk_pitch = 300;
        cmd.w_trunk_x = 200.0; // 位置锁定
        cmd.w_swing_pos = 0.0;
    }

    // 调用控制器
    std::vector<double> torques = controller_.computeWalkStepControl(m, d, cmd);
    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];

    step();
}

// 在 BipedRobot.cpp 中找到这三个函数

// 辅助函数：计算适应躯干速度的控制点
void getBezierControlPoints(const Eigen::Vector3d& p0, const Eigen::Vector3d& p3, double T,
                            double trunk_vel, Eigen::Vector3d& p1, Eigen::Vector3d& p2) {
    // 1. 基础提升高度
    double clearance_h = 0.10; // [加强] 火烈鸟结构需要更高的抬腿高度，防止脚尖蹭地

    p1 = p0;
    p2 = p3;
    p1(2) += clearance_h;
    p2(2) += clearance_h*0.35;

    // 2. [关键修复] 初始速度前馈 (Velocity Feedforward)
    // 贝塞尔起点速度 V_start = 3 * (p1 - p0) / T
    // 我们希望 V_start.x ≈ 躯干速度
    // 所以 (p1.x - p0.x) = V_trunk * T / 3
    double forward_offset = trunk_vel * T / 3.0;

    p1(0) += forward_offset;

    // 3. 终点减速缓冲
    // 我们希望落地时速度不要太快，但也不能为0（否则会顿挫），保持轻微前向速度
    p2(0) -= forward_offset * 0.5;
}

// 修改 getBezierPos - 增加 T 和 trunk_vel 参数
Eigen::Vector3d BipedRobot::getBezierPos(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2);

    double u = 1 - s;
    return u*u*u*p0 + 3*u*u*s*p1 + 3*u*s*s*p2 + s*s*s*p3;
}

// 同样修改 getBezierVel 和 getBezierAcc，使用相同的 p1, p2 计算逻辑
Eigen::Vector3d BipedRobot::getBezierVel(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2); // 确保 p1, p2 一致

    double u = 1 - s;
    Eigen::Vector3d dPds = 3*u*u*(p1-p0) + 6*u*s*(p2-p1) + 3*s*s*(p3-p2);
    return dPds / T;
}

Eigen::Vector3d BipedRobot::getBezierAcc(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2);

    double u = 1 - s;
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
    constexpr double VEL_LIMIT = 30.0;

    auto check = [&](int id, const std::string& name, double min_q, double max_q) {
        int q_adr = m->jnt_qposadr[id];
        int v_adr = m->jnt_dofadr[id];
        double q = -d->qpos[q_adr];
        double v = d->qvel[v_adr];
        double q_deg = q * 180.0 / M_PI;
        double min_deg = min_q * 180.0 / M_PI;
        double max_deg = max_q * 180.0 / M_PI;

        if (q < min_q) {
            warning_msg_ += "[" + name + "] Angle below min: " +
                           std::to_string(q_deg) + " deg < " + std::to_string(min_deg) + " deg\n";
            is_violated_ = true;
        }
        if (q > max_q) {
            warning_msg_ += "[" + name + "] Angle above max: " +
                           std::to_string(q_deg) + " deg > " + std::to_string(max_deg) + " deg\n";
            is_violated_ = true;
        }
        if (std::abs(v) > VEL_LIMIT) {
            warning_msg_ += "[" + name + "] Velocity exceeded: " +
                           std::to_string(v) + " rad/s > " + std::to_string(VEL_LIMIT) + " rad/s\n";
            is_violated_ = true;
        }
    };
    // check(id_hip_left, "L_Hip", HIP_ANGLE_MIN, HIP_ANGLE_MAX);
    // check(id_knee_left, "L_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX);
    // check(id_hip_right, "R_Hip", HIP_ANGLE_MIN, HIP_ANGLE_MAX);
    // check(id_knee_right, "R_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX);
}