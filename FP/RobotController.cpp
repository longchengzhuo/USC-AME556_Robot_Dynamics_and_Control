#include "RobotController.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>

RobotController::RobotController() {
    std::ofstream log_file("controller.txt", std::ios::out);
}

std::vector<double> RobotController::computeStandControl(const mjModel* m, const mjData* d,
                                                         double target_x, double target_z,
                                                         double target_pitch, double duration) {
    int nv = m->nv;
    int nu = m->nu;
    std::vector<double> torques(nu, 0.0);

    // 打开日志
    std::ofstream debug_log("../controller.txt", std::ios::app);
    debug_log << std::fixed << std::setprecision(4);
    debug_log << "=== Time: " << d->time << " ===\n";

    // ==========================================
    // 0. 物理模型体检 (Physics Diagnostics)
    // ==========================================
    int trunk_id = mj_name2id(m, mjOBJ_BODY, "trunk");
    double trunk_mass = (trunk_id >= 0) ? m->body_mass[trunk_id] : -1.0;
    debug_log << "0. Diagnostics: Trunk_Mass=" << trunk_mass << " Total_Mass=" << m->body_subtreemass[0] << "\n";

    // 准备诊断用的 data 副本
    mjData* d_diag = const_cast<mjData*>(d);

    // 备份现场
    std::vector<mjtNum> qacc_backup(nv);
    std::vector<mjtNum> qvel_backup(nv); // [新增] 备份速度
    std::vector<mjtNum> qfrc_constraint_backup(nv);
    mju_copy(qacc_backup.data(), d->qacc, nv);
    mju_copy(qvel_backup.data(), d->qvel, nv); // [新增]
    mju_copy(qfrc_constraint_backup.data(), d->qfrc_constraint, nv);

    // [净化环境]
    mju_zero(d_diag->qfrc_constraint, nv);
    mju_zero(d_diag->qvel, nv); // [关键修复] 清除速度，消除阻尼/软接触的瞬态干扰
    mju_zero(d_diag->qacc, nv);

    mj_inverse(m, d_diag); // 计算纯静态重力补偿

    double gravity_comp_force_z = d_diag->qfrc_inverse[1];
    debug_log << "0. Diagnostics: Gravity_Comp_Force_Z (Expected ~88N) = " << gravity_comp_force_z << "\n";

    // [漏掉的关键步骤 - 必须修复]
    // 诊断结束后，必须立即恢复现场！
    // 否则下面的 "1. PD Control" 读到的 v_x, v_z 全是 0，导致 PD 的 D 项（阻尼）失效！
    mju_copy(d_diag->qacc, qacc_backup.data(), nv);
    mju_copy(d_diag->qvel, qvel_backup.data(), nv);
    mju_copy(d_diag->qfrc_constraint, qfrc_constraint_backup.data(), nv);

    // ==========================================
    // 1. PD Control
    // ==========================================
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

    debug_log << "1. PD_Base_Acc_Des: " << qacc_base_des.transpose() << "\n";

    // ==========================================
    // 2. Rigid Inverse Kinematics
    // ==========================================
    int left_foot_id = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int right_foot_id = mj_name2id(m, mjOBJ_SITE, "right_foot");

    std::vector<mjtNum> jac_l(3 * nv);
    std::vector<mjtNum> jac_r(3 * nv);
    mj_jacSite(m, d, jac_l.data(), nullptr, left_foot_id);
    mj_jacSite(m, d, jac_r.data(), nullptr, right_foot_id);

    auto build_J_task = [&](const std::vector<mjtNum>& jl, const std::vector<mjtNum>& jr) {
        Eigen::MatrixXd J(4, nv);
        for(int i=0; i<nv; ++i) {
            J(0, i) = jl[0*nv + i]; // Left X
            J(1, i) = jl[2*nv + i]; // Left Z
            J(2, i) = jr[0*nv + i]; // Right X
            J(3, i) = jr[2*nv + i]; // Right Z
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

    debug_log << "2. IK_Joint_Acc_Des: " << qacc_joint_des.transpose() << "\n";

    // ==========================================
    // 3. Inverse Dynamics
    // ==========================================
    // [关键修复] 保持环境纯净：无约束力，无速度 (Quasi-Static assumption for ID)
    // 这能保证 ID 输出正确的重力补偿项，不会被瞬态效应带偏
    // 因为前面我们在第 0 步后恢复了数据，所以这里必须再次清零
    mju_zero(d_diag->qfrc_constraint, nv);
    mju_zero(d_diag->qvel, nv);

    for(int i=0; i<nv; ++i) d_diag->qacc[i] = qacc_total_des(i);
    mj_inverse(m, d_diag);
    Eigen::VectorXd ID_target(nv);
    for(int i=0; i<nv; ++i) ID_target(i) = d->qfrc_inverse[i];

    // 恢复所有现场
    mju_copy(d_diag->qacc, qacc_backup.data(), nv);
    mju_copy(d_diag->qvel, qvel_backup.data(), nv); // 恢复速度
    mju_copy(d_diag->qfrc_constraint, qfrc_constraint_backup.data(), nv);

    debug_log << "3. ID_Target_Torque: " << ID_target.transpose() << "\n";

    // ==========================================
    // 4. QP Optimization
    // ==========================================
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

    // [Safety Check] 防止 QP 崩溃输出垃圾值
    if(solver.initSolver() && solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd sol = solver.getSolution();
        // 检查是否有极端值 (例如 > 1e6)
        if (sol.norm() < 1e6) {
            for(int i=0; i<nu; ++i) torques[i] = sol(i);
            debug_log << "4. QP_Solution: " << sol.transpose() << "\n";
        } else {
             debug_log << "4. QP_SOLUTION_GARBAGE! Ignored.\n";
        }
    } else {
        debug_log << "4. QP_FAILED. Defaulting to 0 torque.\n";
    }

    debug_log << "--------------------------------\n";
    debug_log.close();

    check_and_fix_nan(torques);
    return torques;
}

void RobotController::check_and_fix_nan(std::vector<double>& torques) {
    for(double& val : torques) {
        if(std::isnan(val) || std::isinf(val)) val = 0.0;
    }
}