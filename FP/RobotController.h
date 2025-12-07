#pragma once
#include "mujoco/mujoco.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

// [新增] 定义足端轨迹任务结构体
struct FootTarget {
    double x, z;        // 期望位置
    double vx, vz;      // 期望速度
    double ax, az;      // 期望加速度 (FF)

    // 默认构造函数：静止在地面
    FootTarget() : x(0), z(0), vx(0), vz(0), ax(0), az(0) {}
};

class RobotController {
public:
    RobotController();

    // 原有的站立接口 (保持兼容)
    std::vector<double> computeStandControl(const mjModel* m, const mjData* d,
                                            double target_x, double target_z,
                                            double target_pitch, double duration);

    // [新增] 混合动力学控制接口 (支持行走、奔跑)
    // 允许分别指定左右脚的接触状态和摆动轨迹
    std::vector<double> computeHybridControl(const mjModel* m, const mjData* d,
                                             double target_x, double target_z, double target_pitch,
                                             const FootTarget& left_foot, const FootTarget& right_foot,
                                             bool left_contact, bool right_contact);

private:
    // PD Gains
    const double KP_X = 100.0;
    const double KD_X = 20.0;
    const double KP_Z = 200.0;
    const double KD_Z = 20.0;
    const double KP_PITCH = 200.0;
    const double KD_PITCH = 20.0;

    // Swing Leg PD Gains (用于摆动腿追踪)
    const double KP_SWING = 600.0;
    const double KD_SWING = 40.0;

    // 物理参数
    const double MU_CTRL = 0.5;
    const double FZ_MIN = 0.0;
    const double FZ_MAX = 500.0;

    void check_and_fix_nan(std::vector<double>& torques);
};