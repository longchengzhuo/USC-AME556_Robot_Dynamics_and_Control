#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include "RobotController.h"

// 定义机器人状态枚举
enum class RobotState {
    IDLE,
    STAND,

    // Task 3: Single Step
    WALK_WEIGHT_SHIFT,
    WALK_SWING,
    WALK_LAND,

    // [Task 4] Continuous Walk (Cyclic)
    REAL_WALK_INIT_DS,       // 初始双支撑加速
    REAL_WALK_LEFT_SWING,    // 左脚摆动
    REAL_WALK_DS_L2R,        // 双脚支撑 (Left -> Right)
    REAL_WALK_RIGHT_SWING,   // 右脚摆动
    REAL_WALK_DS_R2L         // 双脚支撑 (Right -> Left)
};

class BipedRobot {
public:
    BipedRobot(mjModel* model, mjData* data);
    ~BipedRobot();

    void step();
    void freeFall();

    // 静态站立
    void stand(double target_x, double target_z, double target_pitch, double duration);

    // 迈步任务 (Task 3)
    void walk(double target_x_vel = 0.2);

    // [Task 4] 持续行走任务
    void real_walk(double target_x_vel = 0.2);

    void resetToKeyframe();
    void resetToState(const std::vector<double>& qpos);

    std::string getWarningMessage() const { return warning_msg_; }

    // 获取当前状态用于显示
    RobotState getCurrentState() const { return current_state_; }

    // 获取脚的x位置 (用于显示)
    double getLeftFootX() const { return d->site_xpos[3*id_left_foot_site_]; }
    double getRightFootX() const { return d->site_xpos[3*id_right_foot_site_]; }

    // 判断是否在双脚支撑状态
    bool isDoubleSupport() const {
        return current_state_ == RobotState::REAL_WALK_INIT_DS ||
               current_state_ == RobotState::REAL_WALK_DS_L2R ||
               current_state_ == RobotState::REAL_WALK_DS_R2L;
    }

    // 获取双脚支撑时前脚的x位置
    double getFrontFootX() const {
        double lx = d->site_xpos[3*id_left_foot_site_];
        double rx = d->site_xpos[3*id_right_foot_site_];
        return (lx > rx) ? lx : rx;
    }

    // 获取当前 trunk_x_des (用于显示)
    double getTrunkXDes() const { return trunk_x_des_; }

private:
    double trunk_x_des_ = 0.0;  // 保存当前的 trunk_x_des 用于显示
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
    RobotState current_state_;

    // 迈步任务变量 (通用)
    double walk_start_time_;
    double land_x_pos_;
    Eigen::Vector3d swing_init_pos_;
    Eigen::Vector3d swing_target_pos_;

    // [新增] 持续行走专用变量
    double cw_phase_start_time_; // 当前相位开始时间
    // double cw_trunk_x_ref_;   // [已移除] 不再使用虚拟参考点，改用 flexible tracking

    // [DEBUG] 埋点数据缓存
    double dbg_cmd_x_des_;
    double dbg_cmd_v_des_;
    double dbg_weight_x_;
    double dbg_swing_tgt_x_;

    // 状态日志文件
    std::ofstream state_log_file_;
    // [New] 双支撑专用日志文件
    std::ofstream ds_log_file_;

    void logState();
    // [New] 记录双支撑数据函数
    void logDoubleSupport(double cmd_x, double cmd_w);

    // 辅助函数
    void checkConstraints();
    void enforceTorqueLimits(std::vector<double>& torques);
    double deg2rad(double deg) { return deg * M_PI / 180.0; }

    // 贝塞尔曲线规划
    Eigen::Vector3d getBezierPos(double s, double T, double trunk_vel, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
    Eigen::Vector3d getBezierVel(double s, double T, double trunk_vel, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
    Eigen::Vector3d getBezierAcc(double s, double T, double trunk_vel, const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);

    // 贝塞尔辅助
    void getBezierControlPoints(const Eigen::Vector3d& p0, const Eigen::Vector3d& p3, double T,
                            double trunk_vel, Eigen::Vector3d& p1, Eigen::Vector3d& p2);
};