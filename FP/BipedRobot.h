#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <cmath>
#include "RobotController.h"

// 定义机器人状态枚举
enum class RobotState {
    IDLE,
    STAND,
    WALK_WEIGHT_SHIFT, // 重心转移
    WALK_SWING,        // 摆动相
    WALK_LAND          // 落地平衡
};

class BipedRobot {
public:
    BipedRobot(mjModel* model, mjData* data);

    void step();
    void freeFall();

    // 静态站立
    void stand(double target_x, double target_z, double target_pitch, double duration);

    // [新增] 迈步任务
    void walk();

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
    int id_left_foot_site_, id_right_foot_site_;

    // 状态变量
    RobotState current_state_;      // 当前所处的任务状态

    // 迈步任务变量
    double walk_start_time_;
    Eigen::Vector3d swing_init_pos_;
    Eigen::Vector3d swing_target_pos_;

    // 辅助函数
    void checkConstraints();
    void enforceTorqueLimits(std::vector<double>& torques);
    double deg2rad(double deg) { return deg * M_PI / 180.0; }

    // 贝塞尔曲线规划
    Eigen::Vector3d getBezierPos(double s, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
    Eigen::Vector3d getBezierVel(double s, double T, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
    Eigen::Vector3d getBezierAcc(double s, double T, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
};