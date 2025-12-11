#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <cmath>
#include "RobotController.h"

// 定义机器人状态枚举
enum class RobotState {
    IDLE,
    STAND
};

class BipedRobot {
public:
    BipedRobot(mjModel* model, mjData* data);

    void step();
    void freeFall();

    // 静态站立
    void stand(double target_x, double target_z, double target_pitch, double duration);

    void resetToKeyframe();
    void resetToState(const std::vector<double>& qpos);

    std::string getWarningMessage() const { return warning_msg_; }

private:
    mjModel* m;
    mjData* d;
    std::string warning_msg_;
    bool is_violated_;
    RobotController controller_;

    int id_hip_left, id_knee_left;
    int id_hip_right, id_knee_right;
    int id_floor_geom_, id_left_shin_geom_, id_right_shin_geom_;

    // 状态变量
    RobotState current_state_;      // 当前所处的任务状态

    // 辅助函数
    void checkConstraints();
    void enforceTorqueLimits(std::vector<double>& torques);
    double deg2rad(double deg) { return deg * M_PI / 180.0; }
};