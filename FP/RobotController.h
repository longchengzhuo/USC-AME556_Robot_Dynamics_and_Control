#pragma once
#include "mujoco/mujoco.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <fstream>

class RobotController {
public:
    RobotController();
    ~RobotController();

    // 站立控制接口
    std::vector<double> computeStandControl(const mjModel* m, const mjData* d,
                                            double target_x, double target_z,
                                            double target_pitch, double duration);

    // 任务三：单步迈步控制接口
    struct WalkCommand {
        double trunk_x_des;
        double trunk_z_des;
        double trunk_pitch_des;
        double trunk_x_vel_des;

        Eigen::Vector3d swing_pos_des;
        Eigen::Vector3d swing_vel_des;
        Eigen::Vector3d swing_acc_des;

        bool left_contact;
        bool right_contact;

        double w_trunk_z;
        double w_trunk_pitch;
        double w_trunk_x;
        double w_swing_pos;
    };

    std::vector<double> computeWalkStepControl(const mjModel* m, const mjData* d, const WalkCommand& cmd);

    // [新增] 任务四：持续行走控制接口 (复用 WalkCommand 结构体)
    // 逻辑与 computeWalkStepControl 相似，但可能参数微调
    std::vector<double> computeWalkControl(const mjModel* m, const mjData* d, const WalkCommand& cmd);

private:
    // PD Gains for Stand
    const double KP_X = 100.0, KD_X = 20.0;
    const double KP_Z = 200.0, KD_Z = 20.0;
    const double KP_PITCH = 200.0, KD_PITCH = 20.0;

    // PD Gains for Walk Task 3
    const double W_KP_TRUNK_Z = 300.0, W_KD_TRUNK_Z = 30.0;
    const double W_KP_TRUNK_PITCH = 300.0, W_KD_TRUNK_PITCH = 30.0;
    const double W_KP_TRUNK_X = 100.0, W_KD_TRUNK_X = 10.0;
    const double W_KP_SWING = 400.0, W_KD_SWING = 15.0;

    // [新增] PD Gains for Continuous Walk Task 4
    // 持续行走时，我们需要稍微柔顺一点的 X 轴跟踪，或者更强的速度阻尼
    const double CW_KP_TRUNK_X = 100.0;
    const double CW_KD_TRUNK_X = 15.0;
    // 高度保持要硬
    const double CW_KP_TRUNK_Z = 300.0;
    const double CW_KD_TRUNK_Z = 30.0;
    // 姿态保持要非常硬
    const double CW_KP_TRUNK_PITCH = 300.0;
    const double CW_KD_TRUNK_PITCH = 30.0;
    // 摆动脚追踪
    const double CW_KP_SWING = 450.0;
    const double CW_KD_SWING = 20.0;

    // 物理参数
    const double MU_CTRL = 0.6;
    const double FZ_MIN = 0.0;
    const double FZ_MAX = 500.0;

    std::ofstream log_file_;

    void check_and_fix_nan(std::vector<double>& torques);
};