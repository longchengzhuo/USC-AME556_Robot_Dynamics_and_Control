#pragma once
#include "mujoco/mujoco.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

class RobotController {
public:
    RobotController();

    /**
     * @brief 计算站立任务所需的关节力矩
     * @param m MuJoCo 模型
     * @param d MuJoCo 数据
     * @param target_x 期望躯干 X 位置
     * @param target_z 期望躯干 Z 高度 (对应题目的 y)
     * @param target_pitch 期望躯干俯仰角 (对应题目的 theta)
     * @param duration 期望维持该状态的持续时间
     * @return std::vector<double> 对应各个关节的控制力矩
     */
    std::vector<double> computeStandControl(const mjModel* m, const mjData* d,
                                            double target_x, double target_z,
                                            double target_pitch, double duration);

private:
    // 控制参数 (PD Gains)
    const double KP_X = 100.0;    // 原 500.0
    const double KD_X = 20.0;     // 原 50.0

    const double KP_Z = 200.0;    // 原 2000.0 (关键修改：允许约 5cm 的误差而不违反重力约束)
    const double KD_Z = 20.0;     // 原 100.0

    const double KP_PITCH = 200.0; // 原 1000.0
    const double KD_PITCH = 20.0;  // 原 80.0

    // 摩擦系数与力限制
    const double MU_CTRL = 0.5;
    const double FZ_MIN = 0.0;
    const double FZ_MAX = 500.0;

    // 调试辅助
    void check_and_fix_nan(std::vector<double>& torques);
};