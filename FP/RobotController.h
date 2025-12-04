#pragma once
#include "mujoco/mujoco.h"
#include <vector>

class RobotController {
public:
    RobotController();
    
    /**
     * @brief 计算站立任务所需的关节力矩
     * * @param m MuJoCo 模型
     * @param d MuJoCo 数据
     * @param target_x 期望躯干 X 位置
     * @param target_z 期望躯干 Z 高度
     * @param target_pitch 期望躯干俯仰角
     * @param duration 期望维持该状态的持续时间
     * @return std::vector<double> 对应各个关节的控制力矩
     */
    std::vector<double> computeStandControl(const mjModel* m, const mjData* d, 
                                            double target_x, double target_z, 
                                            double target_pitch, double duration);

    // 未来可以在这里添加行走控制接口，例如：
    // std::vector<double> computeWalkControl(...);

private:
    // 这里可以存放通用的控制参数，如 PID 增益、QP solver 实例等
};