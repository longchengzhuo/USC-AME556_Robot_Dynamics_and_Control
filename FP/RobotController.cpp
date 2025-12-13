#include "RobotController.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>

RobotController::RobotController() {
    log_file_.open("../mpc_log.csv");
    if (log_file_.is_open()) {
        log_file_ << "Time";
        for (int i = 0; i < 7; ++i) log_file_ << ",qacc_" << i;
        for (int i = 0; i < 4; ++i) log_file_ << ",tau_" << i;
        log_file_ << ",lambda_Lx,lambda_Lz,lambda_Rx,lambda_Rz\n";
    } else {
        std::cerr << "Failed to open log file!" << std::endl;
    }
}

RobotController::~RobotController() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

// ==========================================
// 站立控制 (复用之前的逻辑)
// ==========================================
std::vector<double> RobotController::computeStandControl(const mjModel* m, const mjData* d,
                                                         double target_x, double target_z,
                                                         double target_pitch, double duration) {
    int nv = m->nv;
    int nu = m->nu;
    std::vector<double> torques(nu, 0.0);
    mjData* d_diag = const_cast<mjData*>(d);

    double q_x = d->qpos[0];
    double q_z = d->qpos[1];
    double q_pitch = d->qpos[2];
    double v_x = d->qvel[0];
    double v_z = d->qvel[1];
    double v_pitch = d->qvel[2];

    Eigen::Vector3d qacc_base_des;
    qacc_base_des(0) = KP_X * (target_x - q_x) - KD_X * v_x;
    qacc_base_des(1) = KP_Z * (target_z - q_z) - KD_Z * v_z;
    qacc_base_des(2) = KP_PITCH * (target_pitch - q_pitch) - KD_PITCH * v_pitch;

    int left_foot_id = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int right_foot_id = mj_name2id(m, mjOBJ_SITE, "right_foot");

    std::vector<mjtNum> jac_l(3 * nv);
    std::vector<mjtNum> jac_r(3 * nv);
    mj_jacSite(m, d, jac_l.data(), nullptr, left_foot_id);
    mj_jacSite(m, d, jac_r.data(), nullptr, right_foot_id);

    auto build_J_task = [&](const std::vector<mjtNum>& jl, const std::vector<mjtNum>& jr) {
        Eigen::MatrixXd J(4, nv);
        for(int i=0; i<nv; ++i) {
            J(0, i) = jl[0*nv + i];
            J(1, i) = jl[2*nv + i];
            J(2, i) = jr[0*nv + i];
            J(3, i) = jr[2*nv + i];
        }
        return J;
    };

    Eigen::MatrixXd J_task = build_J_task(jac_l, jac_r);
    Eigen::VectorXd bias_acc = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd J_base = J_task.block(0, 0, 4, 3);
    Eigen::MatrixXd J_joint = J_task.block(0, 3, 4, 4);
    Eigen::VectorXd rhs = - (J_base * qacc_base_des + bias_acc);
    Eigen::VectorXd qacc_joint_des = J_joint.colPivHouseholderQr().solve(rhs);

    Eigen::VectorXd qacc_total_des(nv);
    qacc_total_des.segment(0, 3) = qacc_base_des;
    qacc_total_des.segment(3, 4) = qacc_joint_des;

    for(int i=0; i<nv; ++i) d_diag->qacc[i] = qacc_total_des(i);
    mj_rne(m, d_diag, 1, d_diag->qfrc_inverse);
    Eigen::VectorXd ID_target(nv);
    for(int i=0; i<nv; ++i) ID_target(i) = d->qfrc_inverse[i];

    int n_vars = 8;
    int n_constraints = nv + 4 + 2 + 4;
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    P.setIdentity();
    for(int i=4; i<8; ++i) P.coeffRef(i,i) = 1e-1;
    Eigen::VectorXd q_cost = Eigen::VectorXd::Zero(n_vars);

    std::vector<Eigen::Triplet<double>> triplets;
    Eigen::VectorXd l(n_constraints);
    Eigen::VectorXd u(n_constraints);

    int row = 0;
    for(int i=0; i<nv; ++i) {
        if (i >= 3) triplets.emplace_back(row, i-3, 1.0);
        for(int k=0; k<4; ++k) triplets.emplace_back(row, 4+k, J_task(k, i));
        l(row) = ID_target(i);
        u(row) = ID_target(i);
        row++;
    }
    // Friction and Normal force constraints...
    triplets.emplace_back(row, 4, 1.0); triplets.emplace_back(row, 5, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    triplets.emplace_back(row, 4, 1.0); triplets.emplace_back(row, 5, MU_CTRL);  l(row) = 0; u(row) = OsqpEigen::INFTY; row++;
    triplets.emplace_back(row, 6, 1.0); triplets.emplace_back(row, 7, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    triplets.emplace_back(row, 6, 1.0); triplets.emplace_back(row, 7, MU_CTRL);  l(row) = 0; u(row) = OsqpEigen::INFTY; row++;
    triplets.emplace_back(row, 5, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;
    triplets.emplace_back(row, 7, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;

    double lim_hip = 30.0; double lim_knee = 60.0;
    triplets.emplace_back(row, 0, 1.0); l(row) = -lim_hip; u(row) = lim_hip; row++;
    triplets.emplace_back(row, 1, 1.0); l(row) = -lim_knee; u(row) = lim_knee; row++;
    triplets.emplace_back(row, 2, 1.0); l(row) = -lim_hip; u(row) = lim_hip; row++;
    triplets.emplace_back(row, 3, 1.0); l(row) = -lim_knee; u(row) = lim_knee; row++;

    Eigen::SparseMatrix<double> A(n_constraints, n_vars);
    A.setFromTriplets(triplets.begin(), triplets.end());
    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q_cost);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    if(solver.initSolver() && solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd sol = solver.getSolution();
        for(int i=0; i<nu; ++i) torques[i] = sol(i);
    }
    check_and_fix_nan(torques);
    return torques;
}


// ... (computeWalkStepControl 保持不变，用于任务三) ...
std::vector<double> RobotController::computeWalkStepControl(const mjModel* m, const mjData* d, const WalkCommand& cmd) {
    int nv = m->nv;
    int nu = m->nu;
    int n_lambda = 4;
    int n_vars = nv + nu + n_lambda;

    Eigen::VectorXd q_vel(nv);
    for(int i=0; i<nv; ++i) q_vel(i) = d->qvel[i];

    double x_curr = d->qpos[0];
    double z_curr = d->qpos[1];
    double pitch_curr = d->qpos[2];
    double x_vel = d->qvel[0];
    double z_vel = d->qvel[1];
    double pitch_vel = d->qvel[2];

    double acc_z_des = W_KP_TRUNK_Z * (cmd.trunk_z_des - z_curr) - W_KD_TRUNK_Z * z_vel;
    double acc_pitch_des = W_KP_TRUNK_PITCH * (cmd.trunk_pitch_des - pitch_curr) - W_KD_TRUNK_PITCH * pitch_vel;
    double acc_x_des = W_KP_TRUNK_X * (cmd.trunk_x_des - x_curr) + W_KD_TRUNK_X * (cmd.trunk_x_vel_des - x_vel);

    int left_foot_id = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int right_foot_id = mj_name2id(m, mjOBJ_SITE, "right_foot");

    Eigen::Vector3d pos_l;
    pos_l << d->site_xpos[3*left_foot_id], d->site_xpos[3*left_foot_id+1], d->site_xpos[3*left_foot_id+2];

    std::vector<mjtNum> jac_l(3*nv), jac_r(3*nv);
    mj_jacSite(m, d, jac_l.data(), nullptr, left_foot_id);
    mj_jacSite(m, d, jac_r.data(), nullptr, right_foot_id);

    Eigen::MatrixXd J_L(3, nv), J_R(3, nv);
    for(int i=0; i<nv; ++i) {
        J_L(0,i) = jac_l[0*nv+i]; J_L(1,i) = jac_l[1*nv+i]; J_L(2,i) = jac_l[2*nv+i];
        J_R(0,i) = jac_r[0*nv+i]; J_R(1,i) = jac_r[1*nv+i]; J_R(2,i) = jac_r[2*nv+i];
    }
    Eigen::Vector3d vel_l = J_L * q_vel;

    Eigen::Vector3d acc_swing_des = cmd.swing_acc_des +
                                    W_KP_SWING * (cmd.swing_pos_des - pos_l) +
                                    W_KD_SWING * (cmd.swing_vel_des - vel_l);

    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;

    auto addTaskCost = [&](const Eigen::MatrixXd& J_sub, const Eigen::VectorXd& acc_sub, double w) {
        if (w < 1e-6) return;
        Eigen::MatrixXd Hessian = w * J_sub.transpose() * J_sub;
        Eigen::VectorXd Gradient = -w * J_sub.transpose() * acc_sub;

        for(int r=0; r<nv; ++r) {
            for(int c=0; c<nv; ++c) {
                p_triplets.emplace_back(r, c, Hessian(r, c));
            }
            q_vec(r) += Gradient(r);
        }
    };

    Eigen::MatrixXd J_tx = Eigen::MatrixXd::Zero(1, nv); J_tx(0, 0) = 1.0;
    Eigen::VectorXd acc_tx(1); acc_tx(0) = acc_x_des;
    addTaskCost(J_tx, acc_tx, cmd.w_trunk_x);

    Eigen::MatrixXd J_tz = Eigen::MatrixXd::Zero(1, nv); J_tz(0, 1) = 1.0;
    Eigen::VectorXd acc_tz(1); acc_tz(0) = acc_z_des;
    addTaskCost(J_tz, acc_tz, cmd.w_trunk_z);

    Eigen::MatrixXd J_tp = Eigen::MatrixXd::Zero(1, nv); J_tp(0, 2) = 1.0;
    Eigen::VectorXd acc_tp(1); acc_tp(0) = acc_pitch_des;
    addTaskCost(J_tp, acc_tp, cmd.w_trunk_pitch);

    if (!cmd.left_contact) {
        Eigen::MatrixXd J_sw(2, nv);
        J_sw.row(0) = J_L.row(0);
        J_sw.row(1) = J_L.row(2);
        Eigen::VectorXd acc_sw(2);
        acc_sw(0) = acc_swing_des(0);
        acc_sw(1) = acc_swing_des(2);
        addTaskCost(J_sw, acc_sw, cmd.w_swing_pos);
    }

    for(int i=0; i<nv; ++i) p_triplets.emplace_back(i, i, 1e-2);
    for(int i=nv; i<nv+nu; ++i) p_triplets.emplace_back(i, i, 1e-4);
    for(int i=nv+nu; i<n_vars; ++i) p_triplets.emplace_back(i, i, 1e-4);

    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    int n_cons_dyn = nv;
    int n_cons_contact_acc = 4;
    int n_cons_swing_force = 4;
    int n_cons_friction = 8;
    int n_cons_torque = nu;
    int n_constraints = n_cons_dyn + n_cons_contact_acc + n_cons_swing_force + n_cons_friction + n_cons_torque;

    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l = Eigen::VectorXd::Constant(n_constraints, -OsqpEigen::INFTY);
    Eigen::VectorXd u = Eigen::VectorXd::Constant(n_constraints, OsqpEigen::INFTY);

    int row = 0;
    mjData* d_diag = const_cast<mjData*>(d);
    Eigen::MatrixXd M(nv, nv);
    std::vector<double> M_dense(nv*nv);
    mj_fullM(m, M_dense.data(), d->qM);
    for(int r=0; r<nv; ++r)
        for(int c=0; c<nv; ++c)
            M(r, c) = M_dense[r*nv+c];

    Eigen::VectorXd h(nv);
    mj_rne(m, d_diag, 0, d_diag->qfrc_inverse);
    for(int i=0; i<nv; ++i) h(i) = d->qfrc_inverse[i];

    for(int i=0; i<nv; ++i) {
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, M(i,c));
        if (i >= 3) a_triplets.emplace_back(row, nv + (i-3), -1.0);
        a_triplets.emplace_back(row, nv+nu+0, -J_L(0, i));
        a_triplets.emplace_back(row, nv+nu+1, -J_L(2, i));
        a_triplets.emplace_back(row, nv+nu+2, -J_R(0, i));
        a_triplets.emplace_back(row, nv+nu+3, -J_R(2, i));
        l(row) = -h(i); u(row) = -h(i); row++;
    }

    if (cmd.left_contact) {
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_L(0, c)); l(row) = 0; u(row) = 0; row++;
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_L(2, c)); l(row) = 0; u(row) = 0; row++;
    } else { row += 2; }

    if (cmd.right_contact) {
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_R(0, c)); l(row) = 0; u(row) = 0; row++;
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_R(2, c)); l(row) = 0; u(row) = 0; row++;
    } else { row += 2; }

    if (!cmd.left_contact) {
        a_triplets.emplace_back(row, nv+nu+0, 1.0); l(row) = 0; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+1, 1.0); l(row) = 0; u(row) = 0; row++;
    } else {
        a_triplets.emplace_back(row, nv+nu+1, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;
        row++;
    }
    if (!cmd.right_contact) {
        a_triplets.emplace_back(row, nv+nu+2, 1.0); l(row) = 0; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+3, 1.0); l(row) = 0; u(row) = 0; row++;
    } else {
        a_triplets.emplace_back(row, nv+nu+3, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;
        row++;
    }

    if (cmd.left_contact) {
        a_triplets.emplace_back(row, nv+nu+0, 1.0); a_triplets.emplace_back(row, nv+nu+1, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+0, -1.0); a_triplets.emplace_back(row, nv+nu+1, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    } else { row += 2; }
    if (cmd.right_contact) {
        a_triplets.emplace_back(row, nv+nu+2, 1.0); a_triplets.emplace_back(row, nv+nu+3, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+2, -1.0); a_triplets.emplace_back(row, nv+nu+3, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    } else { row += 2; }

    double lim_hip = 30.0; double lim_knee = 60.0;
    a_triplets.emplace_back(row, nv+0, 1.0); l(row) = -lim_hip; u(row) = lim_hip; row++;
    a_triplets.emplace_back(row, nv+1, 1.0); l(row) = -lim_knee; u(row) = lim_knee; row++;
    a_triplets.emplace_back(row, nv+2, 1.0); l(row) = -lim_hip; u(row) = lim_hip; row++;
    a_triplets.emplace_back(row, nv+3, 1.0); l(row) = -lim_knee; u(row) = lim_knee; row++;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    Eigen::SparseMatrix<double> A_sparse(n_constraints, n_vars);
    A_sparse.setFromTriplets(a_triplets.begin(), a_triplets.end());

    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q_vec);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    std::vector<double> torques(nu, 0.0);
    if(solver.initSolver() && solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd sol = solver.getSolution();
        for(int i=0; i<nu; ++i) torques[i] = sol(nv+i);
        if (log_file_.is_open()) {
            log_file_ << d->time;
            for (int i=0; i<nv; ++i) log_file_ << "," << sol(i);
            for (int i=0; i<nu; ++i) log_file_ << "," << sol(nv+i);
            for (int i=0; i<n_lambda; ++i) log_file_ << "," << sol(nv+nu+i);
            log_file_ << "\n";
        }
    }
    check_and_fix_nan(torques);
    return torques;
}

// ==========================================================
// [Task 4] 持续行走控制器 (computeWalkControl)
// 大部分代码结构复用 computeWalkStepControl，但使用独立的 PD 增益
// ==========================================================
std::vector<double> RobotController::computeWalkControl(const mjModel* m, const mjData* d, const WalkCommand& cmd) {
    // 结构完全一致，只是计算 acc_des 时使用 CW_ 前缀的增益
    int nv = m->nv;
    int nu = m->nu;
    int n_lambda = 4;
    int n_vars = nv + nu + n_lambda;

    // --- 1. 获取状态 ---
    Eigen::VectorXd q_vel(nv);
    for(int i=0; i<nv; ++i) q_vel(i) = d->qvel[i];

    double x_curr = d->qpos[0];
    double z_curr = d->qpos[1];
    double pitch_curr = d->qpos[2];
    double x_vel = d->qvel[0];
    double z_vel = d->qvel[1];
    double pitch_vel = d->qvel[2];

    // --- 2. 构造任务 (使用 Continuous Walk 增益) ---
    // Trunk Control
    double acc_z_des = CW_KP_TRUNK_Z * (cmd.trunk_z_des - z_curr) - CW_KD_TRUNK_Z * z_vel;
    double acc_pitch_des = CW_KP_TRUNK_PITCH * (cmd.trunk_pitch_des - pitch_curr) - CW_KD_TRUNK_PITCH * pitch_vel;
    double acc_x_des = CW_KP_TRUNK_X * (cmd.trunk_x_des - x_curr) + CW_KD_TRUNK_X * (cmd.trunk_x_vel_des - x_vel);

    // Swing Foot Control
    int left_foot_id = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int right_foot_id = mj_name2id(m, mjOBJ_SITE, "right_foot");

    Eigen::Vector3d pos_l, pos_r;
    pos_l << d->site_xpos[3*left_foot_id], d->site_xpos[3*left_foot_id+1], d->site_xpos[3*left_foot_id+2];
    pos_r << d->site_xpos[3*right_foot_id], d->site_xpos[3*right_foot_id+1], d->site_xpos[3*right_foot_id+2];

    std::vector<mjtNum> jac_l(3*nv), jac_r(3*nv);
    mj_jacSite(m, d, jac_l.data(), nullptr, left_foot_id);
    mj_jacSite(m, d, jac_r.data(), nullptr, right_foot_id);

    Eigen::MatrixXd J_L(3, nv), J_R(3, nv);
    for(int i=0; i<nv; ++i) {
        J_L(0,i) = jac_l[0*nv+i]; J_L(1,i) = jac_l[1*nv+i]; J_L(2,i) = jac_l[2*nv+i];
        J_R(0,i) = jac_r[0*nv+i]; J_R(1,i) = jac_r[1*nv+i]; J_R(2,i) = jac_r[2*nv+i];
    }
    Eigen::Vector3d vel_l = J_L * q_vel;
    Eigen::Vector3d vel_r = J_R * q_vel;

    // --- 3. 构建 Cost Function (QP) ---
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(n_vars);
    std::vector<Eigen::Triplet<double>> p_triplets;

    auto addTaskCost = [&](const Eigen::MatrixXd& J_sub, const Eigen::VectorXd& acc_sub, double w) {
        if (w < 1e-6) return;
        Eigen::MatrixXd Hessian = w * J_sub.transpose() * J_sub;
        Eigen::VectorXd Gradient = -w * J_sub.transpose() * acc_sub;

        for(int r=0; r<nv; ++r) {
            for(int c=0; c<nv; ++c) {
                p_triplets.emplace_back(r, c, Hessian(r, c));
            }
            q_vec(r) += Gradient(r);
        }
    };

    // Trunk Tasks
    Eigen::MatrixXd J_tx = Eigen::MatrixXd::Zero(1, nv); J_tx(0, 0) = 1.0;
    Eigen::VectorXd acc_tx(1); acc_tx(0) = acc_x_des;
    addTaskCost(J_tx, acc_tx, cmd.w_trunk_x);

    Eigen::MatrixXd J_tz = Eigen::MatrixXd::Zero(1, nv); J_tz(0, 1) = 1.0;
    Eigen::VectorXd acc_tz(1); acc_tz(0) = acc_z_des;
    addTaskCost(J_tz, acc_tz, cmd.w_trunk_z);

    Eigen::MatrixXd J_tp = Eigen::MatrixXd::Zero(1, nv); J_tp(0, 2) = 1.0;
    Eigen::VectorXd acc_tp(1); acc_tp(0) = acc_pitch_des;
    addTaskCost(J_tp, acc_tp, cmd.w_trunk_pitch);

    // Swing Task (Left or Right)
    // 这里的逻辑稍微改进一下，支持左右脚
    if (!cmd.left_contact) {
        Eigen::MatrixXd J_sw(2, nv);
        J_sw.row(0) = J_L.row(0); J_sw.row(1) = J_L.row(2);
        Eigen::VectorXd acc_sw = cmd.swing_acc_des +
                                 CW_KP_SWING * (cmd.swing_pos_des - pos_l) +
                                 CW_KD_SWING * (cmd.swing_vel_des - vel_l);
        // 只取 X, Z
        Eigen::VectorXd acc_sw_2d(2); acc_sw_2d(0) = acc_sw(0); acc_sw_2d(1) = acc_sw(2);
        addTaskCost(J_sw, acc_sw_2d, cmd.w_swing_pos);
    }
    else if (!cmd.right_contact) {
        Eigen::MatrixXd J_sw(2, nv);
        J_sw.row(0) = J_R.row(0); J_sw.row(1) = J_R.row(2);
        Eigen::VectorXd acc_sw = cmd.swing_acc_des +
                                 CW_KP_SWING * (cmd.swing_pos_des - pos_r) +
                                 CW_KD_SWING * (cmd.swing_vel_des - vel_r);
        Eigen::VectorXd acc_sw_2d(2); acc_sw_2d(0) = acc_sw(0); acc_sw_2d(1) = acc_sw(2);
        addTaskCost(J_sw, acc_sw_2d, cmd.w_swing_pos);
    }

    // Regularization
    for(int i=0; i<nv; ++i) p_triplets.emplace_back(i, i, 1e-2);
    for(int i=nv; i<nv+nu; ++i) p_triplets.emplace_back(i, i, 1e-4);
    for(int i=nv+nu; i<n_vars; ++i) p_triplets.emplace_back(i, i, 1e-4);

    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    // --- 4. 约束 (Constraints) ---
    // (逻辑与 computeWalkStepControl 完全一致，直接照搬构造部分)
    int n_constraints = nv + 4 + 4 + 8 + nu;
    std::vector<Eigen::Triplet<double>> a_triplets;
    Eigen::VectorXd l = Eigen::VectorXd::Constant(n_constraints, -OsqpEigen::INFTY);
    Eigen::VectorXd u = Eigen::VectorXd::Constant(n_constraints, OsqpEigen::INFTY);
    int row = 0;

    mjData* d_diag = const_cast<mjData*>(d);
    Eigen::MatrixXd M(nv, nv);
    std::vector<double> M_dense(nv*nv);
    mj_fullM(m, M_dense.data(), d->qM);
    for(int r=0; r<nv; ++r) for(int c=0; c<nv; ++c) M(r, c) = M_dense[r*nv+c];

    Eigen::VectorXd h(nv);
    mj_rne(m, d_diag, 0, d_diag->qfrc_inverse);
    for(int i=0; i<nv; ++i) h(i) = d->qfrc_inverse[i];

    // Dynamics
    for(int i=0; i<nv; ++i) {
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, M(i,c));
        if (i >= 3) a_triplets.emplace_back(row, nv + (i-3), -1.0);
        a_triplets.emplace_back(row, nv+nu+0, -J_L(0, i));
        a_triplets.emplace_back(row, nv+nu+1, -J_L(2, i));
        a_triplets.emplace_back(row, nv+nu+2, -J_R(0, i));
        a_triplets.emplace_back(row, nv+nu+3, -J_R(2, i));
        l(row) = -h(i); u(row) = -h(i); row++;
    }

    // Contact Acceleration
    if (cmd.left_contact) {
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_L(0, c)); l(row) = 0; u(row) = 0; row++;
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_L(2, c)); l(row) = 0; u(row) = 0; row++;
    } else { row += 2; }
    if (cmd.right_contact) {
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_R(0, c)); l(row) = 0; u(row) = 0; row++;
        for(int c=0; c<nv; ++c) a_triplets.emplace_back(row, c, J_R(2, c)); l(row) = 0; u(row) = 0; row++;
    } else { row += 2; }

    // Contact Forces (Normal)
    if (!cmd.left_contact) {
        a_triplets.emplace_back(row, nv+nu+0, 1.0); l(row) = 0; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+1, 1.0); l(row) = 0; u(row) = 0; row++;
    } else {
        a_triplets.emplace_back(row, nv+nu+1, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;
        row++;
    }
    if (!cmd.right_contact) {
        a_triplets.emplace_back(row, nv+nu+2, 1.0); l(row) = 0; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+3, 1.0); l(row) = 0; u(row) = 0; row++;
    } else {
        a_triplets.emplace_back(row, nv+nu+3, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;
        row++;
    }

    // Friction
    if (cmd.left_contact) {
        a_triplets.emplace_back(row, nv+nu+0, 1.0); a_triplets.emplace_back(row, nv+nu+1, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+0, -1.0); a_triplets.emplace_back(row, nv+nu+1, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    } else { row += 2; }
    if (cmd.right_contact) {
        a_triplets.emplace_back(row, nv+nu+2, 1.0); a_triplets.emplace_back(row, nv+nu+3, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
        a_triplets.emplace_back(row, nv+nu+2, -1.0); a_triplets.emplace_back(row, nv+nu+3, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    } else { row += 2; }

    // Torque Limits
    double lim_hip = 30.0; double lim_knee = 60.0;
    a_triplets.emplace_back(row, nv+0, 1.0); l(row) = -lim_hip; u(row) = lim_hip; row++;
    a_triplets.emplace_back(row, nv+1, 1.0); l(row) = -lim_knee; u(row) = lim_knee; row++;
    a_triplets.emplace_back(row, nv+2, 1.0); l(row) = -lim_hip; u(row) = lim_hip; row++;
    a_triplets.emplace_back(row, nv+3, 1.0); l(row) = -lim_knee; u(row) = lim_knee; row++;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);
    Eigen::SparseMatrix<double> A_sparse(n_constraints, n_vars);
    A_sparse.setFromTriplets(a_triplets.begin(), a_triplets.end());
    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q_vec);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    std::vector<double> torques(nu, 0.0);
    if(solver.initSolver() && solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd sol = solver.getSolution();
        for(int i=0; i<nu; ++i) torques[i] = sol(nv+i);
    }
    check_and_fix_nan(torques);
    return torques;
}

void RobotController::check_and_fix_nan(std::vector<double>& torques) {
    bool replaced = false;
    for(double& val : torques) {
        if(std::isnan(val) || std::isinf(val)) {
            val = 0.0;
            replaced = true;
        }
    }
    if (replaced) std::cerr << "WARNING: NaN detected in torques, replaced with 0.0" << std::endl;
}