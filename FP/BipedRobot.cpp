#include "BipedRobot.h"
#include <iostream>
#include <algorithm>

BipedRobot::BipedRobot(mjModel* model, mjData* data) : m(model), d(data) {
    // 缓存关节 ID
    id_hip_left = mj_name2id(m, mjOBJ_JOINT, "left_hip");
    id_knee_left = mj_name2id(m, mjOBJ_JOINT, "left_knee");
    id_hip_right = mj_name2id(m, mjOBJ_JOINT, "right_hip");
    id_knee_right = mj_name2id(m, mjOBJ_JOINT, "right_knee");

    // 缓存几何体 ID 用于接触检测
    id_floor_geom_ = mj_name2id(m, mjOBJ_GEOM, "floor");
    id_left_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "left_shin_geom");
    id_right_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "right_shin_geom");

    warning_msg_ = "";
    is_violated_ = false;
}

void BipedRobot::step() {
    if (is_violated_) return; // 保护机制：若已违规则冻结

    mj_step(m, d);
    checkConstraints();
}

void BipedRobot::freeFall() {
    // 显式清零控制输入
    mju_zero(d->ctrl, m->nu);
    step();
}

void BipedRobot::stand(double target_x, double target_z, double target_pitch, double duration) {
    // 1. 优雅的接触检测 (Smart Contact Check)
    // 遍历当前的所有接触对，检查是否双脚都触碰了地板
    bool left_grounded = false;
    bool right_grounded = false;

    for (int i = 0; i < d->ncon; ++i) {
        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;

        // 检查左脚-地面
        if ((g1 == id_left_shin_geom_ && g2 == id_floor_geom_) ||
            (g2 == id_left_shin_geom_ && g1 == id_floor_geom_)) {
            left_grounded = true;
        }
        // 检查右脚-地面
        if ((g1 == id_right_shin_geom_ && g2 == id_floor_geom_) ||
            (g2 == id_right_shin_geom_ && g1 == id_floor_geom_)) {
            right_grounded = true;
        }
    }

    // 2. 状态分发
    if (left_grounded && right_grounded) {
        // 双脚着地：激活控制器
        std::vector<double> torques = controller_.computeStandControl(
            m, d, target_x, target_z, target_pitch, duration
        );
        for (int i = 0; i < m->nu && i < (int)torques.size(); ++i) {
            d->ctrl[i] = torques[i];
        }
    } else {
        // 任何一只脚悬空：进入松弛状态 (Relax)
        // 这完美处理了空中初始化、跳跃离地、或侧翻倒地的情况
        mju_zero(d->ctrl, m->nu);
    }

    step();
}

void BipedRobot::resetToKeyframe() {
    is_violated_ = false;
    warning_msg_ = "";

    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
        mj_forward(m, d);
    }
}

void BipedRobot::resetToState(const std::vector<double>& qpos) {
    is_violated_ = false;
    warning_msg_ = "";

    mj_resetData(m, d);
    for (size_t i = 0; i < qpos.size() && i < (size_t)m->nq; ++i) {
        d->qpos[i] = qpos[i];
    }
    mj_forward(m, d);
}

void BipedRobot::checkConstraints() {
    warning_msg_ = "";

    // 简化的约束检查，只保留最核心的物理限制
    const double HIP_MIN = deg2rad(-120);
    const double HIP_MAX = deg2rad(30);
    const double KNEE_MIN = deg2rad(0);
    const double KNEE_MAX = deg2rad(160);
    // 放宽一点速度限制，避免误触发
    const double VEL_LIMIT = 40.0;
    const double TORQUE_LIMIT = 100.0; // 这里的限制仅用于检测爆炸，控制器内部有更严格的 QP 限制

    auto check_joint = [&](int id, std::string name, double min_q, double max_q) {
        int qpos_adr = m->jnt_qposadr[id];
        int dof_adr = m->jnt_dofadr[id];

        double q = -d->qpos[qpos_adr];
        double v = d->qvel[dof_adr];
        double tau = d->qfrc_actuator[dof_adr];

        if (q < min_q || q > max_q) {
            warning_msg_ += "VIOLATION: " + name + " LIMIT\n";
            is_violated_ = true;
        }
        if (std::abs(v) > VEL_LIMIT || std::abs(tau) > TORQUE_LIMIT) {
            warning_msg_ += "VIOLATION: UNSTABLE\n";
            is_violated_ = true;
        }
    };

    check_joint(id_hip_left, "L_Hip", HIP_MIN, HIP_MAX);
    check_joint(id_knee_left, "L_Knee", KNEE_MIN, KNEE_MAX);
    check_joint(id_hip_right, "R_Hip", HIP_MIN, HIP_MAX);
    check_joint(id_knee_right, "R_Knee", KNEE_MIN, KNEE_MAX);
}