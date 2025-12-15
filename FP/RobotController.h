/**
 * @file RobotController.h
 * @brief QP-based whole-body controller for biped robot locomotion.
 *
 * This controller implements optimization-based control for standing and walking
 * using quadratic programming (QP) with contact constraints, friction cones,
 * and torque limits.
 */

#pragma once
#include "mujoco/mujoco.h"
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

class RobotController {
public:
    RobotController() = default;
    ~RobotController() = default;

    /**
     * @brief Compute joint torques for static standing control.
     * @param m MuJoCo model pointer.
     * @param d MuJoCo data pointer.
     * @param target_x Desired trunk X position.
     * @param target_z Desired trunk Z position (height).
     * @param target_pitch Desired trunk pitch angle.
     * @return Vector of 4 joint torques [left_hip, left_knee, right_hip, right_knee].
     */
    std::vector<double> computeStandControl(const mjModel* m, const mjData* d,
                                            double target_x, double target_z,
                                            double target_pitch);

    /**
     * @brief Walking command structure containing desired trajectory and contact state.
     */
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

    /**
     * @brief Compute joint torques for continuous walking control.
     * @param m MuJoCo model pointer.
     * @param d MuJoCo data pointer.
     * @param cmd Walking command with desired trajectory and contact state.
     * @return Vector of 4 joint torques.
     */
    std::vector<double> computeWalkControl(const mjModel* m, const mjData* d, const WalkCommand& cmd);

private:
    static constexpr double KP_X = 100.0, KD_X = 20.0;
    static constexpr double KP_Z = 200.0, KD_Z = 20.0;
    static constexpr double KP_PITCH = 200.0, KD_PITCH = 20.0;

    static constexpr double CW_KP_TRUNK_X = 100.0, CW_KD_TRUNK_X = 15.0;
    static constexpr double CW_KP_TRUNK_Z = 300.0, CW_KD_TRUNK_Z = 30.0;
    static constexpr double CW_KP_TRUNK_PITCH = 300.0, CW_KD_TRUNK_PITCH = 30.0;
    static constexpr double CW_KP_SWING = 450.0, CW_KD_SWING = 20.0;

    static constexpr double MU_CTRL = 0.6;
    static constexpr double FZ_MIN = 0.0;
    static constexpr double FZ_MAX = 500.0;

    void checkAndFixNan(std::vector<double>& torques);
};
