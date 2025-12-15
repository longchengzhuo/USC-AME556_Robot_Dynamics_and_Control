/**
 * @file BipedRobot.cpp
 * @brief Implementation of biped robot locomotion controller.
 */

#include "BipedRobot.h"
#include <iostream>
#include <algorithm>

BipedRobot::BipedRobot(mjModel* model, mjData* data) : m(model), d(data) {
    id_hip_left = mj_name2id(m, mjOBJ_JOINT, "left_hip");
    id_knee_left = mj_name2id(m, mjOBJ_JOINT, "left_knee");
    id_hip_right = mj_name2id(m, mjOBJ_JOINT, "right_hip");
    id_knee_right = mj_name2id(m, mjOBJ_JOINT, "right_knee");

    id_floor_geom_ = mj_name2id(m, mjOBJ_GEOM, "floor");
    id_left_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "left_shin_geom");
    id_right_shin_geom_ = mj_name2id(m, mjOBJ_GEOM, "right_shin_geom");
    id_left_foot_site_ = mj_name2id(m, mjOBJ_SITE, "left_foot");
    id_right_foot_site_ = mj_name2id(m, mjOBJ_SITE, "right_foot");

    warning_msg_ = "";
    is_violated_ = false;
    current_state_ = RobotState::IDLE;
}

void BipedRobot::enforceTorqueLimits(std::vector<double>& torques) {
    const double HIP_TORQUE_LIMIT = 30.0;
    const double KNEE_TORQUE_LIMIT = 60.0;

    if (torques.size() >= 4) {
        torques[0] = std::max(-HIP_TORQUE_LIMIT, std::min(torques[0], HIP_TORQUE_LIMIT));
        torques[1] = std::max(-KNEE_TORQUE_LIMIT, std::min(torques[1], KNEE_TORQUE_LIMIT));
        torques[2] = std::max(-HIP_TORQUE_LIMIT, std::min(torques[2], HIP_TORQUE_LIMIT));
        torques[3] = std::max(-KNEE_TORQUE_LIMIT, std::min(torques[3], KNEE_TORQUE_LIMIT));
    }
}

void BipedRobot::step() {
    if (is_violated_) return;
    mj_step(m, d);
    checkConstraints();
}

void BipedRobot::freeFall() {
    if (current_state_ != RobotState::IDLE) {
        current_state_ = RobotState::IDLE;
    }
    mju_zero(d->ctrl, m->nu);
    step();
}

void BipedRobot::stand(double target_x, double target_z, double target_pitch) {
    if (current_state_ != RobotState::STAND) {
        current_state_ = RobotState::STAND;
    }

    bool left_grounded = false;
    bool right_grounded = false;

    for (int i = 0; i < d->ncon; ++i) {
        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;
        if ((g1 == id_left_shin_geom_ && g2 == id_floor_geom_) ||
            (g2 == id_left_shin_geom_ && g1 == id_floor_geom_)) left_grounded = true;
        if ((g1 == id_right_shin_geom_ && g2 == id_floor_geom_) ||
            (g2 == id_right_shin_geom_ && g1 == id_floor_geom_)) right_grounded = true;
    }

    if (left_grounded && right_grounded) {
        std::vector<double> torques = controller_.computeStandControl(m, d, target_x, target_z, target_pitch);
        enforceTorqueLimits(torques);
        for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];
    } else {
        mju_zero(d->ctrl, m->nu);
    }
    step();
}

void BipedRobot::forwardWalk(double target_x_vel, double target_z, double target_pitch) {
    double t_now = d->time;

    const double T_init_shift = 0.4;
    const double T_swing = 0.6;
    const double T_land = 0.203;
    const double T_shift = 0.5061;
    const double T_ds_total = T_land + T_shift;
    const double K_vel = 0.0;

    if (current_state_ != RobotState::REAL_WALK_INIT_DS &&
        current_state_ != RobotState::REAL_WALK_LEFT_SWING &&
        current_state_ != RobotState::REAL_WALK_RIGHT_SWING &&
        current_state_ != RobotState::REAL_WALK_DS_L2R &&
        current_state_ != RobotState::REAL_WALK_DS_R2L) {

        current_state_ = RobotState::REAL_WALK_INIT_DS;
        cw_phase_start_time_ = t_now;
        std::cout << "[ForwardWalk] Start! Phase: INIT_DS" << std::endl;
    }

    double t_phase = t_now - cw_phase_start_time_;
    RobotController::WalkCommand cmd;

    cmd.trunk_z_des = target_z;
    cmd.trunk_pitch_des = target_pitch;

    if (current_state_ == RobotState::REAL_WALK_INIT_DS) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = target_x_vel;
        cmd.w_trunk_x = 500.0;
        cmd.w_trunk_z = 200.0;
        cmd.w_trunk_pitch = 200.0;
        cmd.w_swing_pos = 0.0;

        if (t_phase > T_init_shift) {
            current_state_ = RobotState::REAL_WALK_LEFT_SWING;
            cw_phase_start_time_ = t_now;

            swing_init_pos_ << d->site_xpos[3 * id_left_foot_site_],
                               d->site_xpos[3 * id_left_foot_site_ + 1],
                               d->site_xpos[3 * id_left_foot_site_ + 2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (target_x_vel * T_swing) + K_vel * (v_current - target_x_vel) + 0.25;
            swing_target_pos_(0) += step_dist;
            std::cout << "[ForwardWalk] Phase: LEFT_SWING" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_LEFT_SWING) {
        cmd.left_contact = false;
        cmd.right_contact = true;
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = target_x_vel;
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_L2R;
            cw_phase_start_time_ = t_now;
            land_x_pos_ = (d->site_xpos[3 * id_left_foot_site_] + d->site_xpos[3 * id_right_foot_site_]) / 2.0;
            std::cout << "[ForwardWalk] Phase: DS_L2R" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_DS_L2R) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero();
        cmd.swing_vel_des.setZero();
        cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0;
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = target_x_vel;
            cmd.w_trunk_x = 500.0;
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_RIGHT_SWING;
            cw_phase_start_time_ = t_now;

            swing_init_pos_ << d->site_xpos[3 * id_right_foot_site_],
                               d->site_xpos[3 * id_right_foot_site_ + 1],
                               d->site_xpos[3 * id_right_foot_site_ + 2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (target_x_vel * T_swing) + K_vel * (v_current - target_x_vel) + 0.3;
            swing_target_pos_(0) += step_dist;
            std::cout << "[ForwardWalk] Phase: RIGHT_SWING" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_RIGHT_SWING) {
        cmd.left_contact = true;
        cmd.right_contact = false;
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = target_x_vel;
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_R2L;
            cw_phase_start_time_ = t_now;
            land_x_pos_ = (d->site_xpos[3 * id_left_foot_site_] + d->site_xpos[3 * id_right_foot_site_]) / 2.0;
            std::cout << "[ForwardWalk] Phase: DS_R2L" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_DS_R2L) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero();
        cmd.swing_vel_des.setZero();
        cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0;
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = target_x_vel;
            cmd.w_trunk_x = 500.0;
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_LEFT_SWING;
            cw_phase_start_time_ = t_now;

            swing_init_pos_ << d->site_xpos[3 * id_left_foot_site_],
                               d->site_xpos[3 * id_left_foot_site_ + 1],
                               d->site_xpos[3 * id_left_foot_site_ + 2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (target_x_vel * T_swing) + K_vel * (v_current - target_x_vel) + 0.3;
            swing_target_pos_(0) += step_dist;
            std::cout << "[ForwardWalk] Loop -> LEFT_SWING" << std::endl;
        }
    }

    trunk_x_des_ = cmd.trunk_x_des;

    std::vector<double> torques = controller_.computeWalkControl(m, d, cmd);
    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];

    step();
}

void BipedRobot::backwardWalkLeftFirst(double target_x_vel, double target_z, double target_pitch) {
    double t_now = d->time;

    const double T_init_shift = 0.39;
    const double T_swing = 0.541;
    const double T_land = 0.205;
    const double T_shift = 0.523;
    const double T_ds_total = T_land + T_shift;
    const double K_vel = 0.0;
    const double back_vel = -std::abs(target_x_vel);

    if (current_state_ != RobotState::REAL_WALK_INIT_DS &&
        current_state_ != RobotState::REAL_WALK_LEFT_SWING &&
        current_state_ != RobotState::REAL_WALK_RIGHT_SWING &&
        current_state_ != RobotState::REAL_WALK_DS_L2R &&
        current_state_ != RobotState::REAL_WALK_DS_R2L) {

        current_state_ = RobotState::REAL_WALK_INIT_DS;
        cw_phase_start_time_ = t_now;
        std::cout << "[BackwardWalkLeftFirst] Start! Phase: INIT_DS" << std::endl;
    }

    double t_phase = t_now - cw_phase_start_time_;
    RobotController::WalkCommand cmd;
    cmd.trunk_z_des = target_z;
    cmd.trunk_pitch_des = target_pitch;

    if (current_state_ == RobotState::REAL_WALK_INIT_DS) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = back_vel;
        cmd.w_trunk_x = 500.0;
        cmd.w_trunk_z = 200.0;
        cmd.w_trunk_pitch = 200.0;
        cmd.w_swing_pos = 0.0;

        if (t_phase > T_init_shift) {
            current_state_ = RobotState::REAL_WALK_LEFT_SWING;
            cw_phase_start_time_ = t_now;

            swing_init_pos_ << d->site_xpos[3 * id_left_foot_site_],
                               d->site_xpos[3 * id_left_foot_site_ + 1],
                               d->site_xpos[3 * id_left_foot_site_ + 2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (back_vel * T_swing) + K_vel * (v_current - back_vel) - 0.3;
            swing_target_pos_(0) += step_dist;
            std::cout << "[BackwardWalkLeftFirst] Phase: LEFT_SWING (first)" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_LEFT_SWING) {
        cmd.left_contact = false;
        cmd.right_contact = true;
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = back_vel;
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_L2R;
            cw_phase_start_time_ = t_now;
            land_x_pos_ = (d->site_xpos[3 * id_left_foot_site_] + d->site_xpos[3 * id_right_foot_site_]) / 2.0;
            std::cout << "[BackwardWalkLeftFirst] Phase: DS_L2R" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_DS_L2R) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero();
        cmd.swing_vel_des.setZero();
        cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0;
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = back_vel;
            cmd.w_trunk_x = 500.0;
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_RIGHT_SWING;
            cw_phase_start_time_ = t_now;

            swing_init_pos_ << d->site_xpos[3 * id_right_foot_site_],
                               d->site_xpos[3 * id_right_foot_site_ + 1],
                               d->site_xpos[3 * id_right_foot_site_ + 2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (back_vel * T_swing) + K_vel * (v_current - back_vel) - 0.34;
            swing_target_pos_(0) += step_dist;
            std::cout << "[BackwardWalkLeftFirst] Phase: RIGHT_SWING" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_RIGHT_SWING) {
        cmd.left_contact = true;
        cmd.right_contact = false;
        cmd.trunk_x_des = d->qpos[0];
        cmd.trunk_x_vel_des = back_vel;
        cmd.w_trunk_x = 50.0;
        cmd.w_trunk_z = 300.0;
        cmd.w_trunk_pitch = 500.0;
        cmd.w_swing_pos = 450.0;

        double s = std::min(t_phase / T_swing, 1.0);
        double v_current = d->qvel[0];
        cmd.swing_pos_des = getBezierPos(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_vel_des = getBezierVel(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);
        cmd.swing_acc_des = getBezierAcc(s, T_swing, v_current, swing_init_pos_, swing_target_pos_);

        if (s >= 1.0) {
            current_state_ = RobotState::REAL_WALK_DS_R2L;
            cw_phase_start_time_ = t_now;
            land_x_pos_ = (d->site_xpos[3 * id_left_foot_site_] + d->site_xpos[3 * id_right_foot_site_]) / 2.0;
            std::cout << "[BackwardWalkLeftFirst] Phase: DS_R2L" << std::endl;
        }
    }
    else if (current_state_ == RobotState::REAL_WALK_DS_R2L) {
        cmd.left_contact = true;
        cmd.right_contact = true;
        cmd.swing_pos_des.setZero();
        cmd.swing_vel_des.setZero();
        cmd.swing_acc_des.setZero();
        cmd.w_swing_pos = 0.0;

        if (t_phase < T_land) {
            cmd.trunk_x_des = land_x_pos_;
            cmd.trunk_x_vel_des = 0.0;
            cmd.w_trunk_x = 200.0;
            cmd.w_trunk_z = 300.0;
            cmd.w_trunk_pitch = 300.0;
        } else {
            cmd.trunk_x_des = d->qpos[0];
            cmd.trunk_x_vel_des = back_vel;
            cmd.w_trunk_x = 500.0;
            cmd.w_trunk_z = 200.0;
            cmd.w_trunk_pitch = 200.0;
        }

        if (t_phase > T_ds_total) {
            current_state_ = RobotState::REAL_WALK_LEFT_SWING;
            cw_phase_start_time_ = t_now;

            swing_init_pos_ << d->site_xpos[3 * id_left_foot_site_],
                               d->site_xpos[3 * id_left_foot_site_ + 1],
                               d->site_xpos[3 * id_left_foot_site_ + 2];
            swing_target_pos_ = swing_init_pos_;
            double v_current = d->qvel[0];
            double step_dist = (back_vel * T_swing) + K_vel * (v_current - back_vel) - 0.34;
            swing_target_pos_(0) += step_dist;
            std::cout << "[BackwardWalkLeftFirst] Loop -> LEFT_SWING" << std::endl;
        }
    }

    trunk_x_des_ = cmd.trunk_x_des;
    std::vector<double> torques = controller_.computeWalkControl(m, d, cmd);
    enforceTorqueLimits(torques);
    for (int i = 0; i < m->nu; ++i) d->ctrl[i] = torques[i];
    step();
}

void BipedRobot::getBezierControlPoints(const Eigen::Vector3d& p0, const Eigen::Vector3d& p3, double T,
                                        double trunk_vel, Eigen::Vector3d& p1, Eigen::Vector3d& p2) {
    double clearance_h = 0.09;

    p1 = p0;
    p2 = p3;
    p1(2) += clearance_h;
    p2(2) += clearance_h * 0.1;

    double forward_offset = trunk_vel * T / 3.0;

    p1(0) += forward_offset;
    p2(0) -= forward_offset * 0.5;
}

Eigen::Vector3d BipedRobot::getBezierPos(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2);

    double u = 1 - s;
    return u * u * u * p0 + 3 * u * u * s * p1 + 3 * u * s * s * p2 + s * s * s * p3;
}

Eigen::Vector3d BipedRobot::getBezierVel(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2);

    double u = 1 - s;
    Eigen::Vector3d dPds = 3 * u * u * (p1 - p0) + 6 * u * s * (p2 - p1) + 3 * s * s * (p3 - p2);
    return dPds / T;
}

Eigen::Vector3d BipedRobot::getBezierAcc(double s, double T, double trunk_vel,
                                         const Eigen::Vector3d& p0, const Eigen::Vector3d& p3) {
    Eigen::Vector3d p1, p2;
    getBezierControlPoints(p0, p3, T, trunk_vel, p1, p2);

    double u = 1 - s;
    Eigen::Vector3d d2Pds2 = 6 * u * (p2 - 2 * p1 + p0) + 6 * s * (p3 - 2 * p2 + p1);
    return d2Pds2 / (T * T);
}

void BipedRobot::resetToKeyframe() {
    is_violated_ = false;
    warning_msg_ = "";
    current_state_ = RobotState::IDLE;
    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
        mj_forward(m, d);
    }
}

void BipedRobot::resetToState(const std::vector<double>& qpos) {
    is_violated_ = false;
    warning_msg_ = "";
    current_state_ = RobotState::IDLE;
    mj_resetData(m, d);
    for (size_t i = 0; i < qpos.size() && i < (size_t)m->nq; ++i) {
        d->qpos[i] = qpos[i];
    }
    mj_forward(m, d);
}

void BipedRobot::checkConstraints() {
    warning_msg_ = "";
    const double HIP_ANGLE_MIN = deg2rad(-120);
    const double HIP_ANGLE_MAX = deg2rad(30);
    const double KNEE_ANGLE_MIN = deg2rad(0);
    const double KNEE_ANGLE_MAX = deg2rad(160);
    const double HIP_VEL_LIMIT = 30.0;
    const double KNEE_VEL_LIMIT = 15.0;

    auto check = [&](int id, const std::string& name, double min_q, double max_q, double vel_limit) {
        int q_adr = m->jnt_qposadr[id];
        int v_adr = m->jnt_dofadr[id];
        double q = -d->qpos[q_adr];
        double v = d->qvel[v_adr];
        double q_deg = q * 180.0 / M_PI;
        double min_deg = min_q * 180.0 / M_PI;
        double max_deg = max_q * 180.0 / M_PI;

        if (q < min_q) {
            warning_msg_ += "[" + name + "] Angle below min: " +
                            std::to_string(q_deg) + " deg < " + std::to_string(min_deg) + " deg\n";
            is_violated_ = true;
        }
        if (q > max_q) {
            warning_msg_ += "[" + name + "] Angle above max: " +
                            std::to_string(q_deg) + " deg > " + std::to_string(max_deg) + " deg\n";
            is_violated_ = true;
        }
        if (std::abs(v) > vel_limit) {
            warning_msg_ += "[" + name + "] Velocity exceeded: " +
                            std::to_string(v) + " rad/s > " + std::to_string(vel_limit) + " rad/s\n";
            is_violated_ = true;
        }
    };
    check(id_hip_left, "L_Hip", HIP_ANGLE_MIN, HIP_ANGLE_MAX, HIP_VEL_LIMIT);
    check(id_knee_left, "L_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX, KNEE_VEL_LIMIT);
    check(id_hip_right, "R_Hip", HIP_ANGLE_MIN, HIP_ANGLE_MAX, HIP_VEL_LIMIT);
    check(id_knee_right, "R_Knee", KNEE_ANGLE_MIN, KNEE_ANGLE_MAX, KNEE_VEL_LIMIT);
}
