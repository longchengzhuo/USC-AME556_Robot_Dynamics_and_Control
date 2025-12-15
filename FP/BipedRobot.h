/**
 * @file BipedRobot.h
 * @brief Biped robot state machine and locomotion controller interface.
 *
 * This class implements a finite state machine for biped robot walking,
 * including standing, forward walking, and backward walking with
 * Bezier curve foot trajectory planning.
 */

#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>
#include <cmath>
#include "RobotController.h"

/**
 * @brief Robot locomotion state enumeration.
 */
enum class RobotState {
    IDLE,
    STAND,
    REAL_WALK_INIT_DS,
    REAL_WALK_LEFT_SWING,
    REAL_WALK_DS_L2R,
    REAL_WALK_RIGHT_SWING,
    REAL_WALK_DS_R2L
};

class BipedRobot {
public:
    BipedRobot(mjModel* model, mjData* data);
    ~BipedRobot() = default;

    /**
     * @brief Execute one simulation step with current control.
     */
    void step();

    /**
     * @brief Free fall mode with zero control torques.
     */
    void freeFall();

    /**
     * @brief Static standing control at target pose.
     * @param target_x Desired trunk X position.
     * @param target_z Desired trunk Z position (height).
     * @param target_pitch Desired trunk pitch angle.
     */
    void stand(double target_x, double target_z, double target_pitch);

    /**
     * @brief Continuous forward walking controller.
     * @param target_x_vel Desired forward velocity (positive).
     * @param target_z Desired trunk height.
     * @param target_pitch Desired trunk pitch angle.
     */
    void forwardWalk(double target_x_vel, double target_z, double target_pitch);

    /**
     * @brief Backward walking with left foot stepping first.
     * @param target_x_vel Desired backward velocity magnitude (positive value).
     * @param target_z Desired trunk height.
     * @param target_pitch Desired trunk pitch angle.
     */
    void backwardWalkLeftFirst(double target_x_vel, double target_z, double target_pitch);

    /**
     * @brief Reset robot to initial keyframe pose.
     */
    void resetToKeyframe();

    /**
     * @brief Reset robot to specified joint configuration.
     * @param qpos Target joint positions.
     */
    void resetToState(const std::vector<double>& qpos);

    std::string getWarningMessage() const { return warning_msg_; }
    RobotState getCurrentState() const { return current_state_; }
    double getLeftFootX() const { return d->site_xpos[3 * id_left_foot_site_]; }
    double getRightFootX() const { return d->site_xpos[3 * id_right_foot_site_]; }

    bool isDoubleSupport() const {
        return current_state_ == RobotState::REAL_WALK_INIT_DS ||
               current_state_ == RobotState::REAL_WALK_DS_L2R ||
               current_state_ == RobotState::REAL_WALK_DS_R2L;
    }

    double getFrontFootX() const {
        double lx = d->site_xpos[3 * id_left_foot_site_];
        double rx = d->site_xpos[3 * id_right_foot_site_];
        return (lx > rx) ? lx : rx;
    }

    double getTrunkXDes() const { return trunk_x_des_; }

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

    RobotState current_state_;
    double trunk_x_des_ = 0.0;
    double cw_phase_start_time_;
    double land_x_pos_;
    Eigen::Vector3d swing_init_pos_;
    Eigen::Vector3d swing_target_pos_;

    void checkConstraints();
    void enforceTorqueLimits(std::vector<double>& torques);
    double deg2rad(double deg) { return deg * M_PI / 180.0; }

    Eigen::Vector3d getBezierPos(double s, double T, double trunk_vel,
                                  const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
    Eigen::Vector3d getBezierVel(double s, double T, double trunk_vel,
                                  const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
    Eigen::Vector3d getBezierAcc(double s, double T, double trunk_vel,
                                  const Eigen::Vector3d& p0, const Eigen::Vector3d& p3);
    void getBezierControlPoints(const Eigen::Vector3d& p0, const Eigen::Vector3d& p3, double T,
                                double trunk_vel, Eigen::Vector3d& p1, Eigen::Vector3d& p2);
};
