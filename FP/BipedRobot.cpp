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

    // 初始化状态机
    current_state_ = RobotState::IDLE;
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