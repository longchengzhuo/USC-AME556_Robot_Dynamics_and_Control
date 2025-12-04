#include "BipedRobot.h"
#include <iostream>
#include <algorithm>

BipedRobot::BipedRobot(mjModel* model, mjData* data) : m(model), d(data) {
    // 缓存关节 ID，避免在循环中频繁查找字符串
    id_hip_left = mj_name2id(m, mjOBJ_JOINT, "left_hip");
    id_knee_left = mj_name2id(m, mjOBJ_JOINT, "left_knee");
    id_hip_right = mj_name2id(m, mjOBJ_JOINT, "right_hip");
    id_knee_right = mj_name2id(m, mjOBJ_JOINT, "right_knee");
    
    warning_msg_ = "";
    is_violated_ = false; // 初始化标志
}

void BipedRobot::step() {
    // === 核心逻辑修改 ===
    // 如果已经发生过违规，直接返回。
    // 这将导致 mj_step 不被调用，d->time 停止增加，画面和时间被“冻结”。
    if (is_violated_) return;

    // 1. 运行物理步进
    mj_step(m, d);

    // 2. 步进后立即检查约束
    checkConstraints();
}

void BipedRobot::freeFall() {
    // 自由落体模式：不施加任何控制力
    // 确保所有电机关停
    for(int i=0; i<m->nu; i++) {
        d->ctrl[i] = 0.0;
    }
    step();
}

void BipedRobot::stand(double target_x, double target_z, double target_pitch, double duration) {
    // 调用通用控制器的站立计算方法
    std::vector<double> torques = controller_.computeStandControl(m, d, target_x, target_z, target_pitch, duration);

    // 应用力矩
    for (int i = 0; i < m->nu && i < (int)torques.size(); ++i) {
        d->ctrl[i] = torques[i];
    }

    step();
}

void BipedRobot::walk() {
    // TODO: Implement walking state machine
    step();
}

void BipedRobot::resetToKeyframe() {
    // 重置时务必清除违规状态，否则机器人会一直处于冻结状态
    is_violated_ = false;
    warning_msg_ = "";

    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
        mj_forward(m, d);
    }
}

void BipedRobot::resetToState(const std::vector<double>& qpos) {
    // 重置时务必清除违规状态
    is_violated_ = false;
    warning_msg_ = "";

    mj_resetData(m, d);
    // 假设 qpos 长度匹配模型
    for (size_t i = 0; i < qpos.size() && i < (size_t)m->nq; ++i) {
        d->qpos[i] = qpos[i];
    }
    mj_forward(m, d);
}

void BipedRobot::checkConstraints() {
    warning_msg_ = ""; // 清空上一帧警告

    // === 阈值定义 ===
    const double HIP_MIN = deg2rad(-120);
    const double HIP_MAX = deg2rad(30);
    const double KNEE_MIN = deg2rad(0);
    const double KNEE_MAX = deg2rad(160);

    const double VEL_HIP_LIMIT = 30.0;
    const double VEL_KNEE_LIMIT = 15.0;

    const double TORQUE_HIP_LIMIT = 30.0;
    const double TORQUE_KNEE_LIMIT = 60.0;

    // 获取 qpos 和 qvel 的地址偏移
    auto check_joint = [&](int id, std::string name, double min_q, double max_q, double vel_lim, double torque_lim) {
        int qpos_adr = m->jnt_qposadr[id];
        int dof_adr = m->jnt_dofadr[id]; // 速度和力矩使用 dof 地址

        // === 坐标系转换 ===
        // 题目定义的 Z 轴垂直纸面向外（逆时针为正）
        // MuJoCo 的 Y 轴若与此相反，则所有测量值需要取反
        double q = -d->qpos[qpos_adr];
        double v = -d->qvel[dof_adr];
        double torque = -d->qfrc_actuator[dof_adr];

        // 1. 角度限制
        // 注意：现在 q 已经是取反后的值，可以直接跟题目的 min/max 比较
        if (q < min_q || q > max_q) {
            warning_msg_ += "VIOLATION: " + name + " ANGLE (" + std::to_string(q) + ")\n";
            is_violated_ = true; // 触发冻结
        }
        // 2. 速度限制
        if (std::abs(v) > vel_lim) {
            warning_msg_ += "VIOLATION: " + name + " VELOCITY (" + std::to_string(v) + ")\n";
            is_violated_ = true; // 触发冻结
        }
        // 3. 力矩限制
        if (std::abs(torque) > torque_lim) {
            warning_msg_ += "VIOLATION: " + name + " TORQUE (" + std::to_string(torque) + ")\n";
            is_violated_ = true; // 触发冻结
        }
    };

    check_joint(id_hip_left, "Left Hip", HIP_MIN, HIP_MAX, VEL_HIP_LIMIT, TORQUE_HIP_LIMIT);
    check_joint(id_knee_left, "Left Knee", KNEE_MIN, KNEE_MAX, VEL_KNEE_LIMIT, TORQUE_KNEE_LIMIT);
    // 右腿同理...
    check_joint(id_hip_right, "Right Hip", HIP_MIN, HIP_MAX, VEL_HIP_LIMIT, TORQUE_HIP_LIMIT);
    check_joint(id_knee_right, "Right Knee", KNEE_MIN, KNEE_MAX, VEL_KNEE_LIMIT, TORQUE_KNEE_LIMIT);
}