#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <cmath>
#include "RobotController.h"

// [新增] 定义机器人状态枚举
enum class RobotState {
    IDLE,
    STAND,
    WALK,
    RUN
};

class BipedRobot {
public:
    BipedRobot(mjModel* model, mjData* data);

    void step();
    void freeFall();

    // 静态站立
    void stand(double target_x, double target_z, double target_pitch, double duration);

    // [新增] 动态行走
    // target_vel_x: 期望前进速度 (m/s)
    void walk(double target_vel_x);

    // [新增] 动态奔跑
    // target_vel_x: 期望前进速度 (m/s)
    void run(double target_vel_x);

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

    // === [新增] 内部步态状态变量 ===
    RobotState current_state_;      // 当前所处的任务状态
    double gait_start_time_ = 0.0;  // 记录步态开始的绝对时间

    // 用于记录落足点
    double left_foot_stance_x_ = 0.0;
    double right_foot_stance_x_ = 0.0;

    // 辅助函数
    void checkConstraints();
    void enforceTorqueLimits(std::vector<double>& torques);
    double deg2rad(double deg) { return deg * M_PI / 180.0; }

    // [修改] 增加 swing_time 参数用于正确计算速度
    FootTarget generateSwingTrajectory(double phase, double start_x, double step_len, double height, double swing_time);
};