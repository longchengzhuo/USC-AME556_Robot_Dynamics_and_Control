#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <cmath>
#include "RobotController.h" // 修改引入

class BipedRobot {
public:
    BipedRobot(mjModel* model, mjData* data);

    // 核心仿真步进函数
    void step();

    // 动作模式
    void freeFall();

    // 站立模式接口
    void stand(double target_x, double target_z, double target_pitch, double duration);

    void walk();  // Placeholder

    // 状态管理
    void resetToKeyframe();
    void resetToState(const std::vector<double>& qpos);

    std::string getWarningMessage() const { return warning_msg_; }

private:
    mjModel* m;
    mjData* d;
    std::string warning_msg_;

    bool is_violated_;

    // 修改：使用通用的机器人控制器实例
    RobotController controller_;

    int id_hip_left, id_knee_left;
    int id_hip_right, id_knee_right;

    void checkConstraints();

    double deg2rad(double deg) { return deg * M_PI / 180.0; }
};