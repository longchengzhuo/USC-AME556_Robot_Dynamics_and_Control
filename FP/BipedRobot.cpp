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

    // 初始化状态日志
    state_log_file_.open("../robot_state.csv");
    if (state_log_file_.is_open()) {
        state_log_file_ << "Time";
        state_log_file_ << ",q_x,q_z,q_pitch,q_lh,q_lk,q_rh,q_rk";
        state_log_file_ << ",v_x,v_z,v_pitch,v_lh,v_lk,v_rh,v_rk";
        state_log_file_ << ",lf_x,lf_y,lf_z,lf_vx,lf_vy,lf_vz";
        state_log_file_ << ",rf_x,rf_y,rf_z,rf_vx,rf_vy,rf_vz";
        state_log_file_ << "\n";
    } else {
        std::cerr << "Failed to open robot_state.csv" << std::endl;
    }
}

BipedRobot::~BipedRobot() {
    if (state_log_file_.is_open()) {
        state_log_file_.close();
    }
}

void BipedRobot::logState() {
    if (!state_log_file_.is_open()) return;

    state_log_file_ << d->time;

    for (int i = 0; i < m->nq; ++i) {
        state_log_file_ << "," << d->qpos[i];
    }
    for (int i = 0; i < m->nv; ++i) {
        state_log_file_ << "," << d->qvel[i];
    }

    auto log_foot = [&](int site_id) {
        double* pos = d->site_xpos + 3 * site_id;
        state_log_file_ << "," << pos[0] << "," << pos[1] << "," << pos[2];

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
        // std::cout << "[FSM] Switch to STAND at t=" << d->time << std::endl;
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

// 任务三：单步迈步 (Old Task)
void BipedRobot::one_step(double target_x_vel) {
    double t_now = d->time;
    if (current_state_ != RobotState::WALK_WEIGHT_SHIFT &&
        current_state_ != RobotState::WALK_SWING &&
        current_state_ != RobotState::WALK_LAND) {
        current_state_ = RobotState::WALK_WEIGHT_SHIFT;
        walk_start_time_ = t_now;
        std::cout << "[FSM] Start WALK task. Phase: WEIGHT SHIFT at t=" << t_now << std::endl;
    }

    RobotController::WalkCommand cmd;
    cmd.trunk_x_des = d->qpos[0];
    cmd.trunk_z_des = d->qpos[1];
    double T_shift = 0.4;
    double T_swing = 0.8;
    double t_task = t_now - walk_start_time_;

    if (current_state_ == RobotState::WALK_WEIGHT_SHIFT) {
        cmd.left_contact = true; cmd.right_contact = true;
        cmd.trunk_x_vel_des = target_x_vel;
        cmd.w_trunk_z = 200; cmd.w_trunk_pitch = 200; cmd.w_trunk_x = 500.0; cmd.w_swing_pos = 0.0;
        cmd.trunk_pitch_des = 0.05;

        if (t_task > T_shift) {
            current_state_ = RobotState::WALK_SWING;
            swing_init_pos_ << d->site_xpos[3*id_left_foot_site_], d->site_xpos[3*id_left_foot_site_+1], d->site_xpos[3*id_left_foot_site_+2];
            swing_target_pos_ = swing_init_pos_;
            double estimated_body_dist = cmd.trunk_x_vel_des * T_swing;
            double step_length = 0.25;
            swing_target_pos_(0) += estimated_body_dist + step_length;
        }

    } else if (current_state_ == RobotState::WALK_SWING) {
        cmd.left_contact = false; cmd.right_contact = true;
        double t_swing_curr = t_task - T_shift;
        double s = std::min(t_swing_curr / T_swing, 1.0);
        double trunk_vel = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, trunk_vel, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, trunk_vel, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, trunk_vel, swing_init_pos_, swing_target_pos_);

        cmd.w_trunk_z = 300; cmd.w_trunk_pitch = 500.0; cmd.w_trunk_x = 10.0; cmd.w_swing_pos = 200;
        cmd.trunk_x_vel_des = 0.0; cmd.trunk_pitch_des = 0.05;

        if (s >= 1.0) {
            current_state_ = RobotState::WALK_LAND;
            land_x_pos_ = d->qpos[0];
        }

    } else if (current_state_ == RobotState::WALK_LAND) {
        cmd.left_contact = true; cmd.right_contact = true;
        cmd.trunk_x_des = land_x_pos_;
        cmd.trunk_x_vel_des = 0.0; cmd.trunk_pitch_des = 0.0;
        cmd.w_trunk_z = 300; cmd.w_trunk_pitch = 300; cmd.w_trunk_x = 200.0; cmd.w_swing_pos = 0.0;
    }

    std::vector<double> torques = controller_.computeWalkStepControl(m, d, cmd);
    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];
    step();
}

// =====================================================================
// [Task 4] 持续行走 (real_walk)
// 逻辑序列：
// 1. Shift (双脚加速)
// 2. Swing Left (抬左脚)
// 3. Land (双脚缓冲/停顿) + Shift (双脚加速) -> 在 DS_L2R 状态中实现
// 4. Swing Right (抬右脚)
// 5. Land (双脚缓冲/停顿) + Shift (双脚加速) -> 在 DS_R2L 状态中实现
// =====================================================================
void BipedRobot::forward_walk(double target_x_vel) {
    double t_now = d->time;

    // --- 0. 参数配置 ---
    const double T_init_shift = 0.4; // 初始启动时间
    const double T_swing = 0.6;      // 摆动时间

    // 将 0.68 的双支撑时间拆分为 Land 和 Shift 两个阶段
    const double T_land  = 0.2;     // 落地缓冲时间 (Vel -> 0)
    const double T_shift = 0.51;     // 重心转移/加速时间 (Vel -> Target)
    const double T_ds_total = T_land + T_shift; // 0.68

    // 目标高度与姿态
    const double Z_height = 0.48;
    const double Pitch_des = 0.05;

    // Raibert Heuristic Gains
    const double K_vel = 0.0; // 保持 0.0，严格按照 walk 的逻辑

    // --- 1. 初始化进入 Weight Shift 阶段 ---
    if (current_state_ != RobotState::REAL_WALK_INIT_DS &&
        current_state_ != RobotState::REAL_WALK_LEFT_SWING &&
        current_state_ != RobotState::REAL_WALK_RIGHT_SWING &&
        current_state_ != RobotState::REAL_WALK_DS_L2R &&
        current_state_ != RobotState::REAL_WALK_DS_R2L) {

        current_state_ = RobotState::REAL_WALK_INIT_DS;
        cw_phase_start_time_ = t_now;
        std::cout << "[RealWalk] Start! Phase 1: INIT_DS (Shift)" << std::endl;
    }

    double t_phase = t_now - cw_phase_start_time_;
    RobotController::WalkCommand cmd;

    // 默认高度和姿态
    cmd.trunk_z_des = Z_height;
    cmd.trunk_pitch_des = Pitch_des;

    // -----------------------------------------------------
    // 阶段 1: 初始加速 (Shift) - 对应序列 1
    // -----------------------------------------------------
    if (current_state_ == RobotState::REAL_WALK_INIT_DS) {
        cmd.left_contact = true;
        cmd.right_contact = true;

        // Shift 逻辑：高权重加速
        cmd.trunk_x_des = d->qpos[0]; // 跟随当前位置
        cmd.trunk_x_vel_des = target_x_vel;
        cmd.w_trunk_x = 500.0;
        cmd.w_trunk_z = 200.0;
        cmd.w_trunk_pitch = 200.0;
        cmd.w_swing_pos = 0.0;

        if (t_phase > T_init_shift) {
            current_state_ = RobotState::REAL_WALK_LEFT_SWING;
            cw_phase_start_time_ = t_now;

            // 准备抬左脚
            swing_init_pos_ << d->site_xpos[3*id_left_foot_site_], d->site_xpos[3*id_left_foot_site_+1], d->site_xpos[3*id_left_foot_site_+2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (target_x_vel * T_swing) + K_vel * (v_current - target_x_vel) + 0.25;
            swing_target_pos_(0) += step_dist;
            // double predicted_travel = v_current * T_swing;
            // double raibert_offset = 0.0 * (v_current - target_x_vel); // K_vel = 0
            //
            // // 目标是绝对世界坐标 X
            // // 必须基于 d->qpos[0] (当前躯干位置) 计算，而不是后脚位置
            // swing_target_pos_(0) = d->qpos[0] + predicted_travel * 0.5 + 0.15 + raibert_offset;
            //
            // // 保持 Y 和 Z 不变 (或者 Z 落地为 0)
            // swing_target_pos_(2) = 0.0;
            std::cout << "[RealWalk] Phase 2: LEFT SWING" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 2: 抬左脚 (Swing Left) - 对应序列 2
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_LEFT_SWING) {
        cmd.left_contact = false;
        cmd.right_contact = true;

        // 摆动相逻辑：弱化 X 轴约束，允许滑行
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = target_x_vel; // 期望保持速度
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_L2R;
            cw_phase_start_time_ = t_now;

            // [修改] land_x_pos_ 设定为双脚中点
            land_x_pos_ = (d->site_xpos[3*id_left_foot_site_] + d->site_xpos[3*id_right_foot_site_]) / 2.0;

            std::cout << "[RealWalk] Phase 3: DS (Land -> Shift)" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 3 & 4: 双脚支撑 (Land + Shift) - 对应序列 3, 4
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_DS_L2R) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero(); cmd.swing_vel_des.setZero(); cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            // === 序列 3: Land (落地缓冲) ===
            // 参考 walk::WALK_LAND: 锁定位置，速度归零
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0; // 较低的权重，主要为了稳住
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            // === 序列 4: Shift (再次加速) ===
            // 参考 walk::WALK_WEIGHT_SHIFT: 追逐速度
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = target_x_vel;
            cmd.w_trunk_x = 500.0; // 高权重，爆发加速
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_RIGHT_SWING;
            cw_phase_start_time_ = t_now;

            // 准备抬右脚
            swing_init_pos_ << d->site_xpos[3*id_right_foot_site_], d->site_xpos[3*id_right_foot_site_+1], d->site_xpos[3*id_right_foot_site_+2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (target_x_vel * T_swing) + K_vel * (v_current - target_x_vel) + 0.3;
            swing_target_pos_(0) += step_dist;
            // double predicted_travel = v_current * T_swing;
            // double raibert_offset = 0.0 * (v_current - target_x_vel); // K_vel = 0
            //
            // // 目标是绝对世界坐标 X
            // // 必须基于 d->qpos[0] (当前躯干位置) 计算，而不是后脚位置
            // swing_target_pos_(0) = d->qpos[0] + predicted_travel * 0.5 + 0.15 + raibert_offset;
            //
            // // 保持 Y 和 Z 不变 (或者 Z 落地为 0)
            // swing_target_pos_(2) = 0.0;
            std::cout << "[RealWalk] Phase 5: RIGHT SWING" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 5: 抬右脚 (Swing Right) - 对应序列 5
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_RIGHT_SWING) {
        cmd.left_contact = true;
        cmd.right_contact = false;

        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = target_x_vel;
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_R2L;
            cw_phase_start_time_ = t_now;

            // [修改] land_x_pos_ 设定为双脚中点
            land_x_pos_ = (d->site_xpos[3*id_left_foot_site_] + d->site_xpos[3*id_right_foot_site_]) / 2.0;

            std::cout << "[RealWalk] Phase 6: DS (Land -> Shift)" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 6 & 4: 双脚支撑 (Land + Shift) - 对应序列 6, 4 (Loop)
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_DS_R2L) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero(); cmd.swing_vel_des.setZero(); cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            // === 序列 6: Land ===
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0;
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            // === 序列 4: Shift (Loop) ===
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = target_x_vel;
            cmd.w_trunk_x = 500.0;
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_LEFT_SWING;
            cw_phase_start_time_ = t_now;

            // 准备抬左脚 (Loop)
            swing_init_pos_ << d->site_xpos[3*id_left_foot_site_], d->site_xpos[3*id_left_foot_site_+1], d->site_xpos[3*id_left_foot_site_+2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (target_x_vel * T_swing) + K_vel * (v_current - target_x_vel) + 0.3;
            swing_target_pos_(0) += step_dist;
            // double predicted_travel = v_current * T_swing;
            // double raibert_offset = 0.0 * (v_current - target_x_vel); // K_vel = 0
            //
            // // 目标是绝对世界坐标 X
            // // 必须基于 d->qpos[0] (当前躯干位置) 计算，而不是后脚位置
            // swing_target_pos_(0) = d->qpos[0] + predicted_travel * 0.5 + 0.15 + raibert_offset;
            //
            // // 保持 Y 和 Z 不变 (或者 Z 落地为 0)
            // swing_target_pos_(2) = 0.0;
            std::cout << "[RealWalk] Loop -> Phase 2: LEFT SWING" << std::endl;
        }
    }

    // 保存 trunk_x_des 用于显示
    trunk_x_des_ = cmd.trunk_x_des;

    // 调用控制器
    std::vector<double> torques = controller_.computeWalkControl(m, d, cmd);
    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];

    step();
}

// =====================================================================
// Task 5: Backward Walk (持续后退行走)
// 逻辑与forward_walk相同，只是方向相反
// =====================================================================
void BipedRobot::backward_walk(double target_x_vel) {
    double t_now = d->time;

    // --- 0. 参数配置 ---
    const double T_init_shift = 0.5; // 初始启动时间
    const double T_swing = 0.53;      // 摆动时间

    // 将双支撑时间拆分为 Land 和 Shift 两个阶段
    const double T_land  = 0.2;     // 落地缓冲时间 (Vel -> 0)
    const double T_shift = 0.51;     // 重心转移/加速时间 (Vel -> Target)
    const double T_ds_total = T_land + T_shift;

    // 目标高度与姿态 - 后退时微微后倾
    const double Z_height = 0.48;
    const double Pitch_des = -0.01;  // 负值表示后倾

    // Raibert Heuristic Gains
    const double K_vel = 0.0;

    // 后退速度（负值）
    const double back_vel = -std::abs(target_x_vel);

    // --- 1. 初始化进入 Weight Shift 阶段 ---
    if (current_state_ != RobotState::REAL_WALK_INIT_DS &&
        current_state_ != RobotState::REAL_WALK_LEFT_SWING &&
        current_state_ != RobotState::REAL_WALK_RIGHT_SWING &&
        current_state_ != RobotState::REAL_WALK_DS_L2R &&
        current_state_ != RobotState::REAL_WALK_DS_R2L) {

        current_state_ = RobotState::REAL_WALK_INIT_DS;
        cw_phase_start_time_ = t_now;
        std::cout << "[BackWalk] Start! Phase 1: INIT_DS (Shift)" << std::endl;
    }

    double t_phase = t_now - cw_phase_start_time_;
    RobotController::WalkCommand cmd;

    // 默认高度和姿态
    cmd.trunk_z_des = Z_height;
    cmd.trunk_pitch_des = Pitch_des;

    // -----------------------------------------------------
    // 阶段 1: 初始加速 (Shift) - 向后加速
    // -----------------------------------------------------
    if (current_state_ == RobotState::REAL_WALK_INIT_DS) {
        cmd.left_contact = true;
        cmd.right_contact = true;

        // Shift 逻辑：高权重加速（向后）
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = back_vel;  // 负速度后退
        cmd.w_trunk_x = 500.0;
        cmd.w_trunk_z = 200.0;
        cmd.w_trunk_pitch = 200.0;
        cmd.w_swing_pos = 0.0;

        if (t_phase > T_init_shift) {
            // 后退时先抬右脚（因为站立时右脚在前）
            current_state_ = RobotState::REAL_WALK_RIGHT_SWING;
            cw_phase_start_time_ = t_now;

            // 准备抬右脚（向后迈）
            swing_init_pos_ << d->site_xpos[3*id_right_foot_site_], d->site_xpos[3*id_right_foot_site_+1], d->site_xpos[3*id_right_foot_site_+2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            // 步长计算：向后迈步
            double step_dist = (back_vel * T_swing) + K_vel * (v_current - back_vel) -0.3;
            swing_target_pos_(0) += step_dist;
            std::cout << "[BackWalk] Phase 2: RIGHT SWING (first)" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 2: 抬右脚 (Swing Right) - 后退时先抬右脚
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_RIGHT_SWING) {
        cmd.left_contact = true;
        cmd.right_contact = false;

        // 摆动相逻辑：弱化 X 轴约束，允许滑行
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = back_vel;
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_R2L;
            cw_phase_start_time_ = t_now;

            land_x_pos_ = (d->site_xpos[3*id_left_foot_site_] + d->site_xpos[3*id_right_foot_site_]) / 2.0;

            std::cout << "[BackWalk] Phase 3: DS_R2L (Land -> Shift)" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 3: 双脚支撑 DS_R2L (Land + Shift)
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_DS_R2L) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero(); cmd.swing_vel_des.setZero(); cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            // Land: 锁定位置，速度归零
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0;
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            // Shift: 再次向后加速
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = back_vel;
            cmd.w_trunk_x = 500.0;
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_LEFT_SWING;
            cw_phase_start_time_ = t_now;

            // 准备抬左脚（向后迈）
            swing_init_pos_ << d->site_xpos[3*id_left_foot_site_], d->site_xpos[3*id_left_foot_site_+1], d->site_xpos[3*id_left_foot_site_+2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (back_vel * T_swing) + K_vel * (v_current - back_vel) -0.34;
            swing_target_pos_(0) += step_dist;
            std::cout << "[BackWalk] Phase 4: LEFT SWING" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 4: 抬左脚 (Swing Left) - 向后摆动
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_LEFT_SWING) {
        cmd.left_contact = false;
        cmd.right_contact = true;

        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = back_vel;
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_L2R;
            cw_phase_start_time_ = t_now;

            land_x_pos_ = (d->site_xpos[3*id_left_foot_site_] + d->site_xpos[3*id_right_foot_site_]) / 2.0;

            std::cout << "[BackWalk] Phase 5: DS_L2R (Land -> Shift)" << std::endl;
        }
    }
    // -----------------------------------------------------
    // 阶段 5: 双脚支撑 DS_L2R (Land + Shift) - Loop back to RIGHT_SWING
    // -----------------------------------------------------
    else if (current_state_ == RobotState::REAL_WALK_DS_L2R) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero(); cmd.swing_vel_des.setZero(); cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            // Land
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0;
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            // Shift (Loop)
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = back_vel;
            cmd.w_trunk_x = 500.0;
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_RIGHT_SWING;
            cw_phase_start_time_ = t_now;

            // 准备抬右脚 (Loop)
            swing_init_pos_ << d->site_xpos[3*id_right_foot_site_], d->site_xpos[3*id_right_foot_site_+1], d->site_xpos[3*id_right_foot_site_+2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (back_vel * T_swing) + K_vel * (v_current - back_vel) -0.34;
            swing_target_pos_(0) += step_dist;
            std::cout << "[BackWalk] Loop -> Phase 2: RIGHT SWING" << std::endl;
        }
    }

    // 保存 trunk_x_des 用于显示
    trunk_x_des_ = cmd.trunk_x_des;

    // 调用控制器
    std::vector<double> torques = controller_.computeWalkControl(m, d, cmd);
    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];

    step();
}

void BipedRobot::getBezierControlPoints(const Eigen::Vector3d& p0, const Eigen::Vector3d& p3, double T,
                            double trunk_vel, Eigen::Vector3d& p1, Eigen::Vector3d& p2) {
    double clearance_h = 0.10;

    p1 = p0;
    p2 = p3;
    p1(2) += clearance_h;
    p2(2) += clearance_h*0.1;

    double forward_offset = trunk_vel * T / 3.0;

    p1(0) += forward_offset;
    p2(0) -= forward_offset * 0.5;
}

Eigen::Vector3d BipedRobot::getBezierPos(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2);

    double u = 1 - s;
    return u*u*u*p0 + 3*u*u*s*p1 + 3*u*s*s*p2 + s*s*s*p3;
}

Eigen::Vector3d BipedRobot::getBezierVel(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2);

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