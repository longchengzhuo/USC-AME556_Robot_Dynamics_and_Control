#include "RobotController.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>

// ==========================================
// 辅助工具函数
// ==========================================

template<typename Derived>
std::string eigen_to_string(const Eigen::MatrixBase<Derived>& m) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << "[";
    for(int i=0; i<m.rows(); ++i) {
        for(int j=0; j<m.cols(); ++j) {
            ss << m(i,j) << (j == m.cols()-1 ? "" : ", ");
        }
        if (i < m.rows() - 1) ss << "; ";
    }
    ss << "]";
    return ss.str();
}

bool is_unstable(const std::vector<double>& v, double limit = 1e5) {
    for (double x : v) {
        if (std::isnan(x) || std::isinf(x) || std::abs(x) > limit) return true;
    }
    return false;
}
bool is_unstable(const Eigen::VectorXd& v, double limit = 1e5) {
    if (!v.allFinite()) return true;
    if (v.cwiseAbs().maxCoeff() > limit) return true;
    return false;
}

RobotController::RobotController() {
    std::ofstream log_file("../controller.txt", std::ios::out);
    log_file << "Log started.\n";
}

// ==========================================
// 1. 站立控制 (保持原样)
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
    // Dynamics
    for(int i=0; i<nv; ++i) {
        if (i >= 3) triplets.emplace_back(row, i-3, 1.0);
        for(int k=0; k<4; ++k) triplets.emplace_back(row, 4+k, J_task(k, i));
        l(row) = ID_target(i);
        u(row) = ID_target(i);
        row++;
    }
    // Friction
    triplets.emplace_back(row, 4, 1.0); triplets.emplace_back(row, 5, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    triplets.emplace_back(row, 4, 1.0); triplets.emplace_back(row, 5, MU_CTRL);  l(row) = 0; u(row) = OsqpEigen::INFTY; row++;
    triplets.emplace_back(row, 6, 1.0); triplets.emplace_back(row, 7, -MU_CTRL); l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;
    triplets.emplace_back(row, 6, 1.0); triplets.emplace_back(row, 7, MU_CTRL);  l(row) = 0; u(row) = OsqpEigen::INFTY; row++;
    // Normal Force
    triplets.emplace_back(row, 5, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;
    triplets.emplace_back(row, 7, 1.0); l(row) = FZ_MIN; u(row) = FZ_MAX; row++;
    // Torque Limits
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
        if (sol.norm() < 1e6) {
            for(int i=0; i<nu; ++i) torques[i] = sol(i);
        }
    }

    check_and_fix_nan(torques);
    return torques;
}

// ==========================================
// 2. 混合控制 (修复 Over-Constraint 问题)
// ==========================================
std::vector<double> RobotController::computeHybridControl(const mjModel* m, const mjData* d,
                                                          double target_x, double target_z, double target_pitch,
                                                          const FootTarget& task_l, const FootTarget& task_r,
                                                          bool contact_l, bool contact_r)
{
    int nv = m->nv; // 7
    int nu = m->nu; // 4
    std::vector<double> torques(nu, 0.0);

    // 调试开关
    bool enable_debug = (d->time > 0.98);
    std::ofstream debug_log;
    if (enable_debug) {
        debug_log.open("../controller.txt", std::ios::app);
        debug_log << "\n========================================\n";
        debug_log << "DEBUG FRAME Time: " << d->time << "\n";
    }

    mjData* d_diag = const_cast<mjData*>(d);

    // --- State ---
    double q_x = d->qpos[0]; double q_z = d->qpos[1]; double q_pitch = d->qpos[2];
    double v_x = d->qvel[0]; double v_z = d->qvel[1]; double v_pitch = d->qvel[2];

    // --- Step 1: Base Task PD ---
    Eigen::Vector3d acc_base_des;
    acc_base_des(0) = KP_X * (target_x - q_x) - KD_X * v_x;
    acc_base_des(1) = KP_Z * (target_z - q_z) - KD_Z * v_z;
    acc_base_des(2) = KP_PITCH * (target_pitch - q_pitch) - KD_PITCH * v_pitch;

    if (enable_debug) debug_log << "Plan -> Acc_Base_Des: " << acc_base_des.transpose() << "\n";

    // --- Step 2: Foot Tasks ---
    int id_site_l = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int id_site_r = mj_name2id(m, mjOBJ_SITE, "right_foot");

    std::vector<mjtNum> jac_l_full(3 * nv), jac_r_full(3 * nv);
    mj_jacSite(m, d, jac_l_full.data(), nullptr, id_site_l);
    mj_jacSite(m, d, jac_r_full.data(), nullptr, id_site_r);

    double pos_l[3] = {d->site_xpos[id_site_l*3], d->site_xpos[id_site_l*3+1], d->site_xpos[id_site_l*3+2]};
    double pos_r[3] = {d->site_xpos[id_site_r*3], d->site_xpos[id_site_r*3+1], d->site_xpos[id_site_r*3+2]};

    // Extract translational Jacobian (X, Z)
    Eigen::MatrixXd J_l(2, nv), J_r(2, nv);
    for(int i=0; i<nv; ++i) {
        J_l(0, i) = jac_l_full[0*nv + i]; // X
        J_l(1, i) = jac_l_full[2*nv + i]; // Z
        J_r(0, i) = jac_r_full[0*nv + i]; // X
        J_r(1, i) = jac_r_full[2*nv + i]; // Z
    }

    Eigen::VectorXd v_generalized(nv); for(int i=0;i<nv;++i) v_generalized(i)=d->qvel[i];
    Eigen::Vector2d v_curr_l = J_l * v_generalized;
    Eigen::Vector2d v_curr_r = J_r * v_generalized;

    Eigen::Vector2d acc_l_des, acc_r_des;
    if (contact_l) acc_l_des.setZero();
    else {
        acc_l_des(0) = task_l.ax + KP_SWING * (task_l.x - pos_l[0]) + KD_SWING * (task_l.vx - v_curr_l(0));
        acc_l_des(1) = task_l.az + KP_SWING * (task_l.z - pos_l[2]) + KD_SWING * (task_l.vz - v_curr_l(1));
    }

    if (contact_r) acc_r_des.setZero();
    else {
        acc_r_des(0) = task_r.ax + KP_SWING * (task_r.x - pos_r[0]) + KD_SWING * (task_r.vx - v_curr_r(0));
        acc_r_des(1) = task_r.az + KP_SWING * (task_r.z - pos_r[2]) + KD_SWING * (task_r.vz - v_curr_r(1));
    }

    if (enable_debug) {
        debug_log << "Plan -> Acc_Foot_L: " << acc_l_des.transpose() << "\n";
        debug_log << "Plan -> Acc_Foot_R: " << acc_r_des.transpose() << "\n";
    }

    // --- Step 3: Acceleration IK ---
    Eigen::MatrixXd J_task(4, nv);
    J_task.block(0,0,2,nv) = J_l;
    J_task.block(2,0,2,nv) = J_r;

    Eigen::VectorXd acc_feet_total(4);
    acc_feet_total.segment(0,2) = acc_l_des;
    acc_feet_total.segment(2,2) = acc_r_des;

    Eigen::MatrixXd J_base = J_task.block(0, 0, 4, 3);
    Eigen::MatrixXd J_joint = J_task.block(0, 3, 4, 4);
    Eigen::VectorXd rhs = acc_feet_total - J_base * acc_base_des;
    Eigen::VectorXd acc_joint_des = J_joint.colPivHouseholderQr().solve(rhs);

    Eigen::VectorXd qacc_total_des(nv);
    qacc_total_des.segment(0, 3) = acc_base_des;
    qacc_total_des.segment(3, 4) = acc_joint_des;

    // --- Step 4: Inverse Dynamics (ID) ---
    for(int i=0; i<nv; ++i) d_diag->qacc[i] = qacc_total_des(i);
    mj_rne(m, d_diag, 1, d_diag->qfrc_inverse);
    Eigen::VectorXd ID_target(nv);
    for(int i=0; i<nv; ++i) ID_target(i) = d_diag->qfrc_inverse[i];

    if (enable_debug) debug_log << "ID -> Computed Generalized Forces: " << ID_target.transpose() << "\n";

    // --- Step 5: QP Formulation (Soft Constraints for Base) ---
    // [FIX] Under-actuation handling
    // We cannot satisfy Base Dynamics (Rows 0-2) perfectly in Single Support.
    // Solution: Move Base Dynamics to Cost Function. Keep Joint Dynamics as Constraints.

    int n_vars = 8;
    // Constraints: Joint Dynamics(4) + Left Contact(4) + Right Contact(4) + Torque Limits(4) = 16
    // Note: We REMOVED 3 Base Dynamics rows from constraints.
    int n_constraints = 4 + 4 + 4 + 4;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(enable_debug);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    // === Hessian P & Gradient q ===
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    P.setIdentity();
    Eigen::VectorXd q_cost = Eigen::VectorXd::Zero(n_vars);

    // 1. Regularization
    for(int i=0; i<4; ++i) P.coeffRef(i,i) = 1.0;   // Min Torque
    for(int i=4; i<8; ++i) P.coeffRef(i,i) = 1e-3; // Min Force

    // 2. [NEW] Soft Constraint for Base Dynamics
    // Minimize || J_base^T * F - ID_base ||^2
    // J_base^T (3x4) maps Forces(4) to BaseGenForces(3)
    // We strictly use the first 3 rows of the transposed Jacobian
    // J_task is (4x7). J_task^T is (7x4). Top 3 rows are Base.
    Eigen::MatrixXd W_base = J_task.block(0,0,4,3).transpose(); // (3x4)
    Eigen::VectorXd ID_base = ID_target.segment(0, 3);          // (3x1)

    double weight_base = 1e5; // High weight for tracking

    // P += weight * W^T * W
    Eigen::MatrixXd P_add = weight_base * W_base.transpose() * W_base;
    for(int r=0; r<4; ++r) {
        for(int c=0; c<4; ++c) {
            // Forces are at indices 4..7 in QP vars
            P.coeffRef(4+r, 4+c) += P_add(r, c);
        }
    }

    // q += weight * -W^T * ID_base
    Eigen::VectorXd q_add = weight_base * (-W_base.transpose() * ID_base);
    for(int i=0; i<4; ++i) {
        q_cost(4+i) += q_add(i);
    }

    // === Constraints A ===
    std::vector<Eigen::Triplet<double>> triplets;
    Eigen::VectorXd l(n_constraints);
    Eigen::VectorXd u(n_constraints);
    int row = 0;

    // 5.1 Joint Dynamics Constraints (4 rows)
    // ID_joint = tau + J_joint^T * F
    // => tau + J_joint^T * F = ID_joint
    for(int i=3; i<nv; ++i) { // Skip 0,1,2 (Base)
        // Tau part
        triplets.emplace_back(row, i-3, 1.0);

        // Force part
        triplets.emplace_back(row, 4, J_task(0, i)); // Lx
        triplets.emplace_back(row, 5, J_task(1, i)); // Lz
        triplets.emplace_back(row, 6, J_task(2, i)); // Rx
        triplets.emplace_back(row, 7, J_task(3, i)); // Rz

        l(row) = ID_target(i);
        u(row) = ID_target(i);
        row++;
    }

    // 5.2 Left Foot Constraints
    double fz_min_l = 0, fz_max_l = (contact_l ? FZ_MAX : 0);
    triplets.emplace_back(row, 5, 1.0); l(row) = fz_min_l; u(row) = fz_max_l; row++;

    triplets.emplace_back(row, 4, 1.0); triplets.emplace_back(row, 5, -MU_CTRL);
    l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;

    triplets.emplace_back(row, 4, 1.0); triplets.emplace_back(row, 5, MU_CTRL);
    l(row) = 0; u(row) = OsqpEigen::INFTY; row++;

    if (!contact_l) {
        triplets.emplace_back(row, 4, 1.0); l(row) = -1e-5; u(row) = 1e-5; row++;
    } else {
        triplets.emplace_back(row, 4, 0.0); l(row) = -OsqpEigen::INFTY; u(row) = OsqpEigen::INFTY; row++;
    }

    // 5.3 Right Foot Constraints
    double fz_min_r = 0, fz_max_r = (contact_r ? FZ_MAX : 0);
    triplets.emplace_back(row, 7, 1.0); l(row) = fz_min_r; u(row) = fz_max_r; row++;

    triplets.emplace_back(row, 6, 1.0); triplets.emplace_back(row, 7, -MU_CTRL);
    l(row) = -OsqpEigen::INFTY; u(row) = 0; row++;

    triplets.emplace_back(row, 6, 1.0); triplets.emplace_back(row, 7, MU_CTRL);
    l(row) = 0; u(row) = OsqpEigen::INFTY; row++;

    if (!contact_r) {
        triplets.emplace_back(row, 6, 1.0); l(row) = -1e-5; u(row) = 1e-5; row++;
    } else {
        triplets.emplace_back(row, 6, 0.0); l(row) = -OsqpEigen::INFTY; u(row) = OsqpEigen::INFTY; row++;
    }

    // 5.4 Torque Limits
    double lim = 60.0;
    for(int i=0; i<4; ++i) {
        triplets.emplace_back(row, i, 1.0); l(row) = -lim; u(row) = lim; row++;
    }

    // Solve
    Eigen::SparseMatrix<double> A_sparse(n_constraints, n_vars);
    A_sparse.setFromTriplets(triplets.begin(), triplets.end());

    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q_cost);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    if(solver.initSolver() && solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd sol = solver.getSolution();
        if (enable_debug) debug_log << "QP -> Full Solution: " << sol.transpose() << "\n";

        if (!is_unstable(sol)) {
            for(int i=0; i<nu; ++i) torques[i] = sol(i);
        } else {
            if(enable_debug) debug_log << "[CRITICAL] QP Solution Unstable!\n";
        }
    } else {
        if (enable_debug) debug_log << "[CRITICAL] QP Solver FAILED (Status Error).\n";
        std::cerr << "QP Failed!" << std::endl;
    }

    if (enable_debug) {
        debug_log << "Final Torques: " << eigen_to_string(Eigen::Map<Eigen::VectorXd>(torques.data(), nu)) << "\n";
        debug_log.close();
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