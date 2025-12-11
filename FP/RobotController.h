#pragma once
#include "mujoco/mujoco.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

class RobotController {
public:
    RobotController();

    // 站立控制接口
    std::vector<double> computeStandControl(const mjModel* m, const mjData* d,
                                            double target_x, double target_z,
                                            double target_pitch, double duration);

private:
    // PD Gains
    const double KP_X = 100.0;
    const double KD_X = 20.0;
    const double KP_Z = 200.0;
    const double KD_Z = 20.0;
    const double KP_PITCH = 200.0;
    const double KD_PITCH = 20.0;

    // 物理参数
    const double MU_CTRL = 0.5;
    const double FZ_MIN = 0.0;
    const double FZ_MAX = 500.0;

    void check_and_fix_nan(std::vector<double>& torques);
};