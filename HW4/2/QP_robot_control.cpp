/*
 * Biped Balancing QP Controller for MuJoCo
 *
 * Stability Status: STABLE
 * Interaction: IMPROVED (Standard MuJoCo mouse/keyboard controls)
 *
 * Analysis of Success:
 * 1. Re-initializing solver every step avoids "Sparsity Pattern" crashes.
 * 2. NaNs are caught early, preventing physics explosions.
 */

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <iomanip>

// Eigen and QP Solver
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

// MuJoCo and Visualization
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

// --- Configuration ---
const char* MODEL_PATH = "../robot.xml";
const char* LOG_PATH = "../robot_data.csv";
const char* DEBUG_LOG_PATH = "../debug_log.txt";

const double SIM_DURATION = 50.0;
const double STARTUP_DELAY = 0.1;

// Controller Gains
const double KP_POS_X = 300.0;
const double KD_POS_X = 30.0;
const double KP_POS_Z = 200.0;
const double KD_POS_Z = 20.0;
const double KP_PITCH = 300.0;
const double KD_PITCH = 60.0;

// Safety constraints
const double MU_CTRL = 0.5;
const double FZ_MIN = 0.0;
const double FZ_MAX = 250.0;
const double DESIRED_Z = 0.5;
const double DESIRED_X = 0.0;
const double DESIRED_PITCH = 0.0;

// Global variables
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// Data logging globals
double global_grf[4] = {0.0, 0.0, 0.0, 0.0};
int global_qp_status = 0;

// Debug Stream
std::ofstream debug_file;

// --- Interaction State ---
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// --- Logging Helper Macros ---
template<typename T>
void log_val(const std::string& label, T val) {
    if (debug_file.is_open()) debug_file << label << "=" << val << " ";
}

bool has_nan(const Eigen::VectorXd& v, const std::string& name) {
    if (!v.allFinite()) {
        if(debug_file.is_open()) debug_file << " [ERROR: " << name << " contains NaN/Inf!] ";
        return true;
    }
    return false;
}

// --- Callbacks (Updated as requested) ---

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
        // Reset to keyframe 0 (initial)
        int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
        if (key_id >= 0) mj_resetDataKeyframe(m, d, key_id);
        else mj_resetData(m, d); // Fallback
        mj_forward(m, d);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// --- Contact Detection ---
bool check_double_support(mjModel* m, mjData* d) {
    bool left_contact = false;
    bool right_contact = false;

    int floor_id = mj_name2id(m, mjOBJ_GEOM, "floor");
    int left_shin_id = mj_name2id(m, mjOBJ_GEOM, "left_shin_geom");
    int right_shin_id = mj_name2id(m, mjOBJ_GEOM, "right_shin_geom");

    for (int i = 0; i < d->ncon; i++) {
        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;
        if ((g1 == floor_id && g2 == left_shin_id) || (g2 == floor_id && g1 == left_shin_id)) left_contact = true;
        if ((g1 == floor_id && g2 == right_shin_id) || (g2 == floor_id && g1 == right_shin_id)) right_contact = true;
    }
    return left_contact && right_contact;
}

void apply_damping(mjData* d) {
    for(int i=0; i<4; i++) d->ctrl[i] = -1.0 * d->qvel[3+i];
    for(int i=0; i<4; i++) global_grf[i] = 0.0;
}

// --- QP Controller ---
void run_qp_controller(mjModel* m, mjData* d) {
    if (debug_file.is_open()) debug_file << "\n[T=" << std::fixed << std::setprecision(4) << d->time << "] ";

    // 1. Startup / Safety
    bool contacts = check_double_support(m, d);
    if (d->time < STARTUP_DELAY || !contacts) {
        global_qp_status = 0;
        apply_damping(d);
        if(debug_file.is_open()) debug_file << "State: Passive/Damping";
        return;
    }

    int nv = m->nv;
    int n_vars = 8;

    // 2. PD Control
    double q_trunk[3] = {d->qpos[0], d->qpos[1], d->qpos[2]};
    double v_trunk[3] = {d->qvel[0], d->qvel[1], d->qvel[2]};

    Eigen::VectorXd qacc_des = Eigen::VectorXd::Zero(nv);
    qacc_des(0) = KP_POS_X * (DESIRED_X - q_trunk[0]) - KD_POS_X * v_trunk[0];
    qacc_des(1) = KP_POS_Z * (DESIRED_Z - q_trunk[1]) - KD_POS_Z * v_trunk[1];
    qacc_des(2) = KP_PITCH * (DESIRED_PITCH - q_trunk[2]) - KD_PITCH * v_trunk[2];

    if(debug_file.is_open()) {
        log_val("PD_Ax", qacc_des(0));
        log_val("PD_Az", qacc_des(1));
        log_val("PD_Apitch", qacc_des(2));
    }

    if (has_nan(qacc_des, "PD_Output")) { apply_damping(d); return; }

    // 3. Inverse Dynamics
    mjtNum* original_qacc = new mjtNum[nv];
    mju_copy(original_qacc, d->qacc, nv);
    for(int i=0; i<nv; i++) d->qacc[i] = qacc_des(i);

    mj_inverse(m, d);

    Eigen::VectorXd ID_target(nv);
    for(int i=0; i<nv; i++) ID_target(i) = d->qfrc_inverse[i];

    mju_copy(d->qacc, original_qacc, nv);
    delete[] original_qacc;

    if(debug_file.is_open()) {
        debug_file << "ID_Base=[" << ID_target(0) << "," << ID_target(1) << "," << ID_target(2) << "] ";
    }
    if (has_nan(ID_target, "ID_Target")) { apply_damping(d); return; }

    // 4. Jacobians
    int left_site_id = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int right_site_id = mj_name2id(m, mjOBJ_SITE, "right_foot");

    if (left_site_id == -1 || right_site_id == -1) return;

    std::vector<mjtNum> jacp_L(3 * nv);
    std::vector<mjtNum> jacp_R(3 * nv);
    mj_jacSite(m, d, jacp_L.data(), NULL, left_site_id);
    mj_jacSite(m, d, jacp_R.data(), NULL, right_site_id);

    Eigen::MatrixXd J_L_T_red(nv, 2);
    Eigen::MatrixXd J_R_T_red(nv, 2);

    for(int i=0; i<nv; i++) {
        J_L_T_red(i, 0) = jacp_L[0*nv + i];
        J_L_T_red(i, 1) = jacp_L[2*nv + i];
        J_R_T_red(i, 0) = jacp_R[0*nv + i];
        J_R_T_red(i, 1) = jacp_R[2*nv + i];
    }

    // Log Jacobian stats
    if (debug_file.is_open()) {
         debug_file << " JacL_00=" << J_L_T_red(0,0);
    }

    // 5. QP Setup (RE-INIT EVERY STEP for stability)
    OsqpEigen::Solver solver;

    int n_constraints = 3 + 4 + 4 + 2;
    Eigen::SparseMatrix<double> A(n_constraints, n_vars);
    Eigen::VectorXd l(n_constraints);
    Eigen::VectorXd u(n_constraints);

    std::vector<Eigen::Triplet<double>> triplets;

    // Dynamics
    for(int i=0; i<nv; i++) {
        if(i >= 3) triplets.push_back(Eigen::Triplet<double>(i, i-3, 1.0));
        triplets.push_back(Eigen::Triplet<double>(i, 4, J_L_T_red(i, 0)));
        triplets.push_back(Eigen::Triplet<double>(i, 5, J_L_T_red(i, 1)));
        triplets.push_back(Eigen::Triplet<double>(i, 6, J_R_T_red(i, 0)));
        triplets.push_back(Eigen::Triplet<double>(i, 7, J_R_T_red(i, 1)));
        l(i) = ID_target(i);
        u(i) = ID_target(i);
    }

    // Friction & Normal
    int r = 7;
    triplets.push_back(Eigen::Triplet<double>(r, 4, 1.0)); triplets.push_back(Eigen::Triplet<double>(r, 5, -MU_CTRL));
    l(r) = -OsqpEigen::INFTY; u(r) = 0; r++;
    triplets.push_back(Eigen::Triplet<double>(r, 4, 1.0)); triplets.push_back(Eigen::Triplet<double>(r, 5, MU_CTRL));
    l(r) = 0; u(r) = OsqpEigen::INFTY; r++;
    triplets.push_back(Eigen::Triplet<double>(r, 6, 1.0)); triplets.push_back(Eigen::Triplet<double>(r, 7, -MU_CTRL));
    l(r) = -OsqpEigen::INFTY; u(r) = 0; r++;
    triplets.push_back(Eigen::Triplet<double>(r, 6, 1.0)); triplets.push_back(Eigen::Triplet<double>(r, 7, MU_CTRL));
    l(r) = 0; u(r) = OsqpEigen::INFTY; r++;
    triplets.push_back(Eigen::Triplet<double>(r, 5, 1.0)); l(r) = FZ_MIN; u(r) = FZ_MAX; r++;
    triplets.push_back(Eigen::Triplet<double>(r, 7, 1.0)); l(r) = FZ_MIN; u(r) = FZ_MAX; r++;

    A.setFromTriplets(triplets.begin(), triplets.end());

    if (has_nan(l, "LowerBound") || has_nan(u, "UpperBound")) { apply_damping(d); return; }

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    Eigen::DiagonalMatrix<double, 8> P_diag;
    P_diag.diagonal() << 1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.1;
    Eigen::SparseMatrix<double> P = P_diag.toDenseMatrix().sparseView();
    Eigen::VectorXd q_cost = Eigen::VectorXd::Zero(n_vars);

    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q_cost);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    if (!solver.initSolver()) {
         if(debug_file.is_open()) debug_file << " ERROR: InitSolver Failed! ";
         apply_damping(d);
         return;
    }

    if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd sol = solver.getSolution();

        // SANITY CHECK for Garbage Values (NaNs/Inf or Huge Ints)
        if (std::abs(sol(0)) > 1e8 || std::isnan(sol(0))) {
             if(debug_file.is_open()) debug_file << " ERROR: Junk Value Detected (" << sol(0) << ") ";
             apply_damping(d);
             global_qp_status = -1;
             return;
        }

        if(debug_file.is_open()) {
            debug_file << " SUCCESS. Tau0=" << sol(0);
        }

        for(int k=0; k<4; k++) d->ctrl[k] = sol(k);
        global_grf[0] = sol(4); global_grf[1] = sol(5);
        global_grf[2] = sol(6); global_grf[3] = sol(7);
        global_qp_status = 1;
    } else {
        if(debug_file.is_open()) debug_file << " ERROR: QP Status Code: Infeasible";
        apply_damping(d);
        global_qp_status = -2;
    }
}

// --- Main ---
int main() {
    debug_file.open(DEBUG_LOG_PATH);

    char error[1000] = "Could not load binary model";
    m = mj_loadXML(MODEL_PATH, 0, error, 1000);
    if (!m) { std::cerr << error << std::endl; return 1; }
    d = mj_makeData(m);

    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "QP Balance", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjv_makeScene(m, &scn, 2000);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0.5;
    cam.distance = 2.5; cam.azimuth = 90; cam.elevation = -10;

    // Callbacks improved
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, keyboard);

    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id >= 0) mj_resetDataKeyframe(m, d, key_id);
    mj_forward(m, d);

    std::ofstream dataFile(LOG_PATH);
    dataFile << "time,x,z,theta,tau1,tau2,tau3,tau4,FLx,FLz,FRx,FRz,qp_status" << std::endl;

    while (!glfwWindowShouldClose(window) && d->time < SIM_DURATION) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            if (std::isnan(d->qpos[0])) {
                if(debug_file.is_open()) debug_file << "\nFATAL: NaN detected\n";
                break;
            }

            mj_step1(m, d);
            run_qp_controller(m, d);
            mj_step2(m, d);

            dataFile << d->time << "," << d->qpos[0] << "," << d->qpos[1] << "," << d->qpos[2] << ","
                     << d->ctrl[0] << "," << d->ctrl[1] << "," << d->ctrl[2] << "," << d->ctrl[3] << ","
                     << global_grf[0] << "," << global_grf[1] << "," << global_grf[2] << "," << global_grf[3] << ","
                     << global_qp_status << std::endl;
        }
        if (std::isnan(d->qpos[0])) break;

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    debug_file.close();
    dataFile.close();
    mj_deleteData(d); mj_deleteModel(m);
    mjr_freeContext(&con); mjv_freeScene(&scn);
    glfwTerminate();
    return 0;
}