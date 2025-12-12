#pragma once
#include "mujoco/mujoco.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <fstream> // 引入文件流

class RobotController {
public:
    RobotController();
    ~RobotController(); // 析构函数用于关闭日志文件

    // 站立控制接口 (任务二/三前段)
    std::vector<double> computeStandControl(const mjModel* m, const mjData* d,
                                            double target_x, double target_z,
                                            double target_pitch, double duration);

    // [新增] 迈步控制接口 (任务三后段)
    // 这是一个统一的 WBC 接口，通过 contact flags 和 task weights 区分不同阶段
    struct WalkCommand {
        // 期望躯干状态
        double trunk_z_des;
        double trunk_pitch_des;
        double trunk_x_vel_des; // 躯干向前速度 (弱约束)

        // 期望摆动脚状态 (仅在摆动相有效)
        Eigen::Vector3d swing_pos_des;
        Eigen::Vector3d swing_vel_des;
        Eigen::Vector3d swing_acc_des;

        // 接触状态标志
        bool left_contact;
        bool right_contact;

        // 任务权重
        double w_trunk_z;
        double w_trunk_pitch;
        double w_trunk_x;
        double w_swing_pos;
    };

    std::vector<double> computeWalkStepControl(const mjModel* m, const mjData* d, const WalkCommand& cmd);

private:
    // PD Gains for Stand
    const double KP_X = 100.0, KD_X = 20.0;
    const double KP_Z = 200.0, KD_Z = 20.0;
    const double KP_PITCH = 200.0, KD_PITCH = 20.0;

    // PD Gains for Walk
    // 躯干保持刚性
    const double W_KP_TRUNK_Z = 300.0, W_KD_TRUNK_Z = 30.0;
    const double W_KP_TRUNK_PITCH = 300.0, W_KD_TRUNK_PITCH = 30.0;
    // [调试修正] 躯干 X 增加一点比例增益，辅助推进，确保不为0，默认为 20
    const double W_KP_TRUNK_X = 20.0, W_KD_TRUNK_X = 10.0;
    // 摆动脚追踪
    const double W_KP_SWING = 400.0, W_KD_SWING = 15.0;

    // 物理参数
    const double MU_CTRL = 0.6;
    const double FZ_MIN = 0.0;
    const double FZ_MAX = 500.0;

    // 日志文件句柄
    std::ofstream log_file_;

    void check_and_fix_nan(std::vector<double>& torques);
};