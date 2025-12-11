#include "RobotController.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>

RobotController::RobotController() {
    std::ofstream log_file("../controller.txt", std::ios::out);
    log_file << "Log started.\n";
}

// ==========================================
// 站立控制
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