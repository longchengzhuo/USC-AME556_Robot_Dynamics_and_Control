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

    // 缓存几何体 ID
    id_floor_geom_ = mj_name2id(m, mjOBJ_GEOM, "floor");
    id_left_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "left_shin_geom");
    id_right_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "right_shin_geom");

    warning_msg_ = "";
    is_violated_ = false;

    // 初始化落足点估计
    left_foot_stance_x_ = 0.0;
    right_foot_stance_x_ = 0.0;

    // [Init] 初始化状态机
    current_state_ = RobotState::IDLE;
    gait_start_time_ = 0.0;
}

// 力矩限幅函数实现
void BipedRobot::enforceTorqueLimits(std::vector<double>& torques) {
    // 题目严格限制：
    // Hip (Joint 1, 3): 30 Nm
    // Knee (Joint 2, 4): 60 Nm
    const double HIP_TORQUE_LIMIT = 30.0;
    const double KNEE_TORQUE_LIMIT = 60.0;

    if (torques.size() >= 4) {
        // Left Hip (0)
        torques[0] = std::max(-HIP_TORQUE_LIMIT, std::min(torques[0], HIP_TORQUE_LIMIT));
        // Left Knee (1)
        torques[1] = std::max(-KNEE_TORQUE_LIMIT, std::min(torques[1], KNEE_TORQUE_LIMIT));
        // Right Hip (2)
        torques[2] = std::max(-HIP_TORQUE_LIMIT, std::min(torques[2], HIP_TORQUE_LIMIT));
        // Right Knee (3)
        torques[3] = std::max(-KNEE_TORQUE_LIMIT, std::min(torques[3], KNEE_TORQUE_LIMIT));
    }
}

void BipedRobot::step() {
    if (is_violated_) return;

    mj_step(m, d);
    // checkConstraints();
}

void BipedRobot::freeFall() {
    // 简单重置状态，如果是自由落体
    if (current_state_ != RobotState::IDLE) {
        current_state_ = RobotState::IDLE;
    }
    mju_zero(d->ctrl, m->nu);
    step();
}

void BipedRobot::stand(double target_x, double target_z, double target_pitch, double duration) {
    // [State Update] 切换到 STAND 状态
    if (current_state_ != RobotState::STAND) {
        current_state_ = RobotState::STAND;
        // stand 不需要重置 gait_start_time，因为它不依赖相位
        std::cout << "[FSM] Switch to STAND at t=" << d->time << std::endl;
    }

    // 1. 接触检测
    bool left_grounded = false;
    bool right_grounded = false;

    for (int i = 0; i < d->ncon; ++i) {
        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;

        if ((g1 == id_left_shin_geom_ && g2 == id_floor_geom_) ||
            (g2 == id_left_shin_geom_ && g1 == id_floor_geom_)) left_grounded = true;

        if ((g1 == id_right_shin_geom_ && g2 == id_floor_geom_) ||
            (g2 == id_right_shin_geom_ && g1 == id_floor_geom_)) right_grounded = true;
    }

    // 2. 控制逻辑
    if (left_grounded && right_grounded) {
        std::vector<double> torques = controller_.computeStandControl(
            m, d, target_x, target_z, target_pitch, duration
        );

        enforceTorqueLimits(torques);

        for (int i = 0; i < m->nu && i < (int)torques.size(); ++i) {
            d->ctrl[i] = torques[i];
        }
    } else {
        mju_zero(d->ctrl, m->nu);
    }

    step();
}

// === [核心逻辑] Walk ===
void BipedRobot::walk(double target_vel_x) {
    const double T_cycle = 0.8;      // 完整步态周期
    const double T_swing = 0.3;      // 单脚摆动时长
    const double swing_height = 0.05; // 抬腿高度 (5cm)
    const double T_step = 0.4;       // 单步时长 (T_cycle / 2)

    // [State Check] 状态机检查：如果当前不是 WALK 状态，说明刚切换过来
    if (current_state_ != RobotState::WALK) {
        current_state_ = RobotState::WALK;
        gait_start_time_ = d->time; // 记录开始时间

        // 重置落足点估计为当前真实位置，防止坐标跳变
        int id_site_l = mj_name2id(m, mjOBJ_SITE, "left_foot");
        int id_site_r = mj_name2id(m, mjOBJ_SITE, "right_foot");
        left_foot_stance_x_ = d->site_xpos[id_site_l * 3];
        right_foot_stance_x_ = d->site_xpos[id_site_r * 3];

        std::cout << "[FSM] Switch to WALK at t=" << d->time << " (Reset Stance)" << std::endl;
    }

    // 使用相对时间计算相位，确保从 0 开始
    double t = d->time - gait_start_time_;
    double t_local = fmod(t, T_cycle);

    bool contact_l = true;
    bool contact_r = true;
    FootTarget task_l, task_r;

    // 获取当前脚的位置（用于更新支撑点）
    int id_site_l = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int id_site_r = mj_name2id(m, mjOBJ_SITE, "right_foot");
    double curr_l_x = d->site_xpos[id_site_l * 3];
    double curr_r_x = d->site_xpos[id_site_r * 3];

    // 步长计算
    double current_vel_x = d->qvel[0];
    double step_len = target_vel_x * T_step + 0.1 * (current_vel_x - target_vel_x);

    // --- 简易状态机 ---

    // Phase 1: Left Swing (0.0 ~ 0.3s)
    if (t_local < T_swing) {
        contact_l = false; // 左脚摆动
        contact_r = true;  // 右脚支撑

        right_foot_stance_x_ = curr_r_x; // 更新支撑脚位置

        double start_x = left_foot_stance_x_;
        double end_x = right_foot_stance_x_ + step_len;
        double dist = end_x - start_x;

        double phase = t_local / T_swing;
        // [修改] 传入 T_swing
        task_l = generateSwingTrajectory(phase, start_x, dist, swing_height, T_swing);
    }
    // Phase 2: Double Support (0.3 ~ 0.4s)
    else if (t_local < 0.4) {
        contact_l = true;
        contact_r = true;
        // 刚落地，更新左脚位置
        left_foot_stance_x_ = curr_l_x;
    }
    // Phase 3: Right Swing (0.4 ~ 0.7s)
    else if (t_local < 0.4 + T_swing) {
        contact_l = true;
        contact_r = false; // 右脚摆动

        left_foot_stance_x_ = curr_l_x;

        double start_x = right_foot_stance_x_;
        double end_x = left_foot_stance_x_ + step_len;
        double dist = end_x - start_x;

        double phase = (t_local - 0.4) / T_swing;
        // [修改] 传入 T_swing
        task_r = generateSwingTrajectory(phase, start_x, dist, swing_height, T_swing);
    }
    // Phase 4: Double Support (0.7 ~ 0.8s)
    else {
        contact_l = true;
        contact_r = true;
        right_foot_stance_x_ = curr_r_x;
    }

    // CoM 规划
    // [Fix] 降低 Lookahead，避免起步时躯干加速度过大 (0.1 -> 0.06)
    const double T_lookahead = 0.06;
    double target_CoM_x = d->qpos[0] + target_vel_x * T_lookahead;

    double target_CoM_z = 0.48; // 行走时稍微降低重心
    double target_pitch = 0.0;

    // 调用混合控制器
    std::vector<double> torques = controller_.computeHybridControl(
        m, d, target_CoM_x, target_CoM_z, target_pitch,
        task_l, task_r, contact_l, contact_r
    );

    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];
    step();
}

// === [核心逻辑] Run ===
void BipedRobot::run(double target_vel_x) {
    const double T_cycle = 0.6;
    const double T_stance = 0.15;
    const double swing_height = 0.08;
    const double T_swing_run = 0.3;

    // [State Check] 状态机检查：如果当前不是 RUN 状态，说明刚切换过来
    if (current_state_ != RobotState::RUN) {
        current_state_ = RobotState::RUN;
        gait_start_time_ = d->time; // 记录开始时间

        int id_site_l = mj_name2id(m, mjOBJ_SITE, "left_foot");
        int id_site_r = mj_name2id(m, mjOBJ_SITE, "right_foot");
        left_foot_stance_x_ = d->site_xpos[id_site_l * 3];
        right_foot_stance_x_ = d->site_xpos[id_site_r * 3];

        std::cout << "[FSM] Switch to RUN at t=" << d->time << " (Reset Stance)" << std::endl;
    }

    double t = d->time - gait_start_time_; // 使用相对时间
    double t_local = fmod(t, T_cycle);

    bool contact_l = false;
    bool contact_r = false;
    FootTarget task_l, task_r;

    // 获取当前信息
    int id_site_l = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int id_site_r = mj_name2id(m, mjOBJ_SITE, "right_foot");
    double curr_l_x = d->site_xpos[id_site_l * 3];
    double curr_r_x = d->site_xpos[id_site_r * 3];

    // Raibert Heuristic
    double stance_step = target_vel_x * T_stance / 2.0 + 0.05 * (d->qvel[0] - target_vel_x);
    double flight_dist = target_vel_x * 0.15;

    // Phase 1: Left Stance (0 ~ 0.15)
    if (t_local < T_stance) {
        contact_l = true;
        contact_r = false; // 右脚摆动
        left_foot_stance_x_ = curr_l_x;

        double start_x = right_foot_stance_x_;
        double end_x = d->qpos[0] + flight_dist + stance_step + 0.1;
        double dist = end_x - start_x;

        double phase = t_local / T_swing_run;
        task_r = generateSwingTrajectory(phase, start_x, dist, swing_height, T_swing_run);
    }
    // Phase 2: Flight (0.15 ~ 0.3)
    else if (t_local < 0.3) {
        contact_l = false;
        contact_r = false;
        double start_x = right_foot_stance_x_;
        double end_x = d->qpos[0] + stance_step + 0.1;
        double dist = end_x - start_x;
        double phase = t_local / T_swing_run;
        task_r = generateSwingTrajectory(phase, start_x, dist, swing_height, T_swing_run);
    }
    // Phase 3: Right Stance (0.3 ~ 0.45)
    else if (t_local < 0.3 + T_stance) {
        contact_l = false;
        contact_r = true;
        right_foot_stance_x_ = curr_r_x;

        double start_x = left_foot_stance_x_;
        double end_x = d->qpos[0] + flight_dist + stance_step + 0.1;
        double dist = end_x - start_x;
        double phase = (t_local - 0.3) / T_swing_run;
        task_l = generateSwingTrajectory(phase, start_x, dist, swing_height, T_swing_run);
    }
    // Phase 4: Flight (0.45 ~ 0.6)
    else {
        contact_l = false;
        contact_r = false;
        double start_x = left_foot_stance_x_;
        double end_x = d->qpos[0] + stance_step + 0.1;
        double dist = end_x - start_x;
        double phase = (t_local - 0.3) / T_swing_run;
        task_l = generateSwingTrajectory(phase, start_x, dist, swing_height, T_swing_run);
    }

    // Run parameters
    double target_CoM_x = d->qpos[0] + target_vel_x * 0.15;
    double target_CoM_z = 0.50;
    double target_pitch = 0.15;

    std::vector<double> torques = controller_.computeHybridControl(
        m, d, target_CoM_x, target_CoM_z, target_pitch,
        task_l, task_r, contact_l, contact_r
    );

    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];
    step();
}

// [修改] 升级为五次多项式插值 (Quintic Spline)，确保起步 0 加速度
FootTarget BipedRobot::generateSwingTrajectory(double phase, double start_x, double dist, double height, double swing_time) {
    FootTarget t;
    // s in [0, 1]
    double s = std::max(0.0, std::min(1.0, phase));

    // 使用五次多项式 (Quintic Polynomial) 进行相位重映射
    // p(s) = 10s^3 - 15s^4 + 6s^5
    // 这保证了位置、速度、加速度在 s=0 和 s=1 处均为 0 (Minimum Jerk)
    double s3 = s * s * s;
    double s4 = s3 * s;
    double s5 = s4 * s;

    double p = 10*s3 - 15*s4 + 6*s5;
    double dp = 30*s*s - 60*s3 + 30*s4;
    double ddp = 60*s - 180*s*s + 120*s3;

    // 时间缩放因子
    double dt = 1.0 / swing_time;
    double dt2 = dt * dt;

    // === X轴 ===
    t.x = start_x + dist * p;
    t.vx = dist * dp * dt;
    t.ax = dist * ddp * dt2;

    // === Z轴 ===
    // 使用重映射后的相位 p 代入正弦函数
    // z = R + H * sin(pi * p)
    const double FOOT_RADIUS = 0.03;
    double sin_pi_p = std::sin(M_PI * p);
    double cos_pi_p = std::cos(M_PI * p);

    t.z = FOOT_RADIUS + height * sin_pi_p;

    // Chain rule: dz/dt = dz/dp * dp/ds * ds/dt
    // dz/dp = H * pi * cos(pi * p)
    t.vz = height * M_PI * cos_pi_p * dp * dt;

    // Chain rule: d2z/dt2
    // d(v)/dt = d(v)/ds * ds/dt
    // v = C * cos(pi*p) * dp   (C constants)
    // dv/ds = C * [ -pi*sin(pi*p)*dp*dp + cos(pi*p)*ddp ]
    t.az = height * M_PI * ( -M_PI * sin_pi_p * dp * dp + cos_pi_p * ddp ) * dt2;

    return t;
}

void BipedRobot::resetToKeyframe() {
    is_violated_ = false;
    warning_msg_ = "";

    // 重置状态机
    current_state_ = RobotState::IDLE;
    gait_start_time_ = 0.0;

    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
        mj_forward(m, d);
    }
}

void BipedRobot::resetToState(const std::vector<double>& qpos) {
    is_violated_ = false;
    warning_msg_ = "";

    // 重置状态机
    current_state_ = RobotState::IDLE;
    gait_start_time_ = 0.0;

    mj_resetData(m, d);
    for (size_t i = 0; i < qpos.size() && i < (size_t)m->nq; ++i) {
        d->qpos[i] = qpos[i];
    }
    mj_forward(m, d);
}

void BipedRobot::checkConstraints() {
    warning_msg_ = "";

    // === 题目物理约束 ===
    const double HIP_ANGLE_MIN = deg2rad(-120);
    const double HIP_ANGLE_MAX = deg2rad(30);
    const double KNEE_ANGLE_MIN = deg2rad(0);
    const double KNEE_ANGLE_MAX = deg2rad(160);

    const double HIP_VELOCITY_LIMIT = 30.0;
    const double KNEE_VELOCITY_LIMIT = 15.0;

    auto check_joint = [&](int id, std::string name, double min_q, double max_q, double vel_lim) {
        int qpos_adr = m->jnt_qposadr[id];
        int dof_adr = m->jnt_dofadr[id];

        double q = -d->qpos[qpos_adr];
        double v = d->qvel[dof_adr];

        if (q < min_q || q > max_q) {
            warning_msg_ += "VIOLATION: " + name + " ANGLE (" + std::to_string(q) + ")\n";
            is_violated_ = true;
        }

        if (std::abs(v) > vel_lim) {
            warning_msg_ += "VIOLATION: " + name + " VELOCITY (" + std::to_string(v) + ")\n";
            is_violated_ = true;
        }
    };

    check_joint(id_hip_left,  "L_Hip",  HIP_ANGLE_MIN,  HIP_ANGLE_MAX,  HIP_VELOCITY_LIMIT);
    check_joint(id_knee_left, "L_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX, KNEE_VELOCITY_LIMIT);
    check_joint(id_hip_right, "R_Hip",  HIP_ANGLE_MIN,  HIP_ANGLE_MAX,  HIP_VELOCITY_LIMIT);
    check_joint(id_knee_right,"R_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX, KNEE_VELOCITY_LIMIT);
}