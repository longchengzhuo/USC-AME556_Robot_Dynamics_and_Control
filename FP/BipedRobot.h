#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <cmath>

class BipedRobot {
public:
    BipedRobot(mjModel* model, mjData* data);
    
    // 核心仿真步进函数
    void step();

    // 动作模式
    void freeFall(); 
    void stand(); // Placeholder
    void walk();  // Placeholder

    // 状态管理
    void resetToKeyframe();
    void resetToState(const std::vector<double>& qpos);

    // 获取当前的警告信息用于 UI 显示
    std::string getWarningMessage() const { return warning_msg_; }

private:
    mjModel* m;
    mjData* d;
    std::string warning_msg_;

    // 新增：违规标志位
    bool is_violated_;

    // 缓存关节 ID 以提高效率
    int id_hip_left, id_knee_left;
    int id_hip_right, id_knee_right;

    // 内部函数：检查所有物理约束
    void checkConstraints();
    
    // 辅助函数：角度转弧度
    double deg2rad(double deg) { return deg * M_PI / 180.0; }
};