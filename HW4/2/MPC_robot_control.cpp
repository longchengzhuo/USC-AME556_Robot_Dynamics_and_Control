#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <cstdio>

// Eigen and OsqpEigen
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

// MuJoCo and Visualization
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

// --- Configuration ---
const char* MODEL_PATH = "../robot.xml";
const char* LOG_PATH = "../robot_data_mpc.csv";
const char* DEBUG_LOG_PATH = "../debug_log_mpc.txt";
const char* VIDEO_PATH = "../mpc.mp4";

const double SIM_DURATION = 5.0;
const double STARTUP_DELAY = 0.01;

// MPC Parameters
const int MPC_HORIZON = 10;
const double MPC_DT = 0.02; // 50Hz Update

// MPC Weights
const double Q_POS_X = 1000.0;
const double Q_POS_Z = 5000.0;
const double Q_THETA = 1000.0;
const double Q_VEL_X = 10.0;
const double Q_VEL_Z = 10.0;
const double Q_OMEGA = 10.0;
const double R_FORCE = 0.01;

// Joint Damping
const double KD_JOINT = 2.0;

// Constraints
const double MU_CTRL = 0.5;
const double FZ_MIN = 0.0;
const double FZ_MAX = 300.0;

// Targets
const double TARGET_X = 0.0;
const double TARGET_Z = 0.5;
const double TARGET_THETA = 0.0;

// --- Global Variables ---
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// State
std::ofstream debug_file;
double global_mpc_grf[4] = {0.0, 0.0, 0.0, 0.0}; // [FLx, FLz, FRx, FRz]
int global_mpc_status = 0;
double last_mpc_time = -1.0;

// Model Params (Fetched at runtime)
double robot_mass = 0.0;
double robot_inertia = 0.0;

// Interaction
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

// --- Helper Functions ---

template<typename T>
void log_kv(const std::string& key, T val) {
    if (debug_file.is_open()) debug_file << key << "=" << val << " ";
}

void log_vec(const std::string& key, const Eigen::VectorXd& v) {
    if (debug_file.is_open()) {
        debug_file << key << "=[";
        for(int i=0; i<v.size(); i++) debug_file << v(i) << (i<v.size()-1?",":"");
        debug_file << "] ";
    }
}

bool has_nan(const Eigen::VectorXd& v) {
    return !v.allFinite();
}

// --- Interaction Callbacks ---
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
        int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
        if (key_id >= 0) mj_resetDataKeyframe(m, d, key_id);
        else mj_resetData(m, d);
        mj_forward(m, d);
        last_mpc_time = -1.0;
        if (debug_file.is_open()) debug_file << "\n[RESET]\n";
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
    lastx = xpos; lasty = ypos;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else action = mjMOUSE_ZOOM;
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// --- Physics Helpers ---
void apply_damping(mjData* d) {
    for(int i=0; i<4; i++) d->ctrl[i] = -10.0 * d->qvel[3+i];
}

bool check_double_support(mjModel* m, mjData* d) {
    bool left = false, right = false;
    int floor = mj_name2id(m, mjOBJ_GEOM, "floor");
    int l_shin = mj_name2id(m, mjOBJ_GEOM, "left_shin_geom");
    int r_shin = mj_name2id(m, mjOBJ_GEOM, "right_shin_geom");
    for (int i = 0; i < d->ncon; i++) {
        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;
        if ((g1 == floor && g2 == l_shin) || (g2 == floor && g1 == l_shin)) left = true;
        if ((g1 == floor && g2 == r_shin) || (g2 == floor && g1 == r_shin)) right = true;
    }
    return left && right;
}

// --- MPC Core ---
bool solve_mpc(mjModel* m, mjData* d, Eigen::VectorXd& optimal_force) {
    // State
    double cx = d->qpos[0];
    double cz = d->qpos[1];
    double ctheta = d->qpos[2];
    double vx = d->qvel[0];
    double vz = d->qvel[1];
    double vtheta = d->qvel[2];

    // Fail-safe check
    if (std::abs(ctheta) > 1.0) return false;

    if (debug_file.is_open()) {
        debug_file << "[MPC_IN] ";
        log_kv("z", cz); log_kv("th", ctheta);
    }

    // Foot locations
    int l_site = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int r_site = mj_name2id(m, mjOBJ_SITE, "right_foot");
    double l_pos[3] = {d->site_xpos[3*l_site], d->site_xpos[3*l_site+1], d->site_xpos[3*l_site+2]};
    double r_pos[3] = {d->site_xpos[3*r_site], d->site_xpos[3*r_site+1], d->site_xpos[3*r_site+2]};

    double rx_L = l_pos[0] - cx;
    double rz_L = l_pos[2] - cz;
    double rx_R = r_pos[0] - cx;
    double rz_R = r_pos[2] - cz;

    if (debug_file.is_open()) {
        log_kv("rx_L", rx_L); log_kv("rz_L", rz_L);
    }

    int nx = 6;
    int nu = 4;

    // Matrices
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx, nx);
    A(0, 3) = MPC_DT; A(1, 4) = MPC_DT; A(2, 5) = MPC_DT;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
    double m_inv = 1.0 / robot_mass;
    double I_inv = 1.0 / robot_inertia;

    B(3, 0) = m_inv * MPC_DT; B(3, 2) = m_inv * MPC_DT;
    B(4, 1) = m_inv * MPC_DT; B(4, 3) = m_inv * MPC_DT;
    B(5, 0) = ( rz_L * I_inv) * MPC_DT;
    B(5, 1) = (-rx_L * I_inv) * MPC_DT;
    B(5, 2) = ( rz_R * I_inv) * MPC_DT;
    B(5, 3) = (-rx_R * I_inv) * MPC_DT;

    Eigen::VectorXd g_vec = Eigen::VectorXd::Zero(nx);
    g_vec(4) = -9.81 * MPC_DT;

    // QP Setup
    int n_vars = nx * (MPC_HORIZON + 1) + nu * MPC_HORIZON;
    int n_constr = nx * (MPC_HORIZON + 1) + 6 * MPC_HORIZON;

    Eigen::DiagonalMatrix<double, 6> Q;
    Q.diagonal() << Q_POS_X, Q_POS_Z, Q_THETA, Q_VEL_X, Q_VEL_Z, Q_OMEGA;
    Eigen::DiagonalMatrix<double, 4> R;
    R.diagonal() << R_FORCE, R_FORCE, R_FORCE, R_FORCE;

    Eigen::VectorXd x0(nx); x0 << cx, cz, ctheta, vx, vz, vtheta;
    Eigen::VectorXd xref(nx); xref << TARGET_X, TARGET_Z, TARGET_THETA, 0, 0, 0;

    std::vector<Eigen::Triplet<double>> P_triplets;
    std::vector<Eigen::Triplet<double>> A_triplets;
    Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(n_vars);
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(n_constr);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(n_constr);

    for(int k=0; k<=MPC_HORIZON; ++k) {
        int idx_x = k * (nx + nu);
        for(int i=0; i<nx; ++i) {
            P_triplets.emplace_back(idx_x+i, idx_x+i, Q.diagonal()[i]);
            q_vec(idx_x+i) = -Q.diagonal()[i] * xref(i);
        }
        if(k < MPC_HORIZON) {
            int idx_u = idx_x + nx;
            for(int i=0; i<nu; ++i)
                P_triplets.emplace_back(idx_u+i, idx_u+i, R.diagonal()[i]);
        }
    }

    int constr_idx = 0;
    for(int i=0; i<nx; ++i) {
        A_triplets.emplace_back(constr_idx+i, i, 1.0);
        lb(constr_idx+i) = x0(i); ub(constr_idx+i) = x0(i);
    }
    constr_idx += nx;

    for(int k=0; k<MPC_HORIZON; ++k) {
        int idx_x_k = k * (nx + nu);
        int idx_u_k = idx_x_k + nx;
        int idx_x_next = (k+1) * (nx + nu);

        for(int i=0; i<nx; ++i) {
            A_triplets.emplace_back(constr_idx+i, idx_x_next+i, 1.0);
            for(int j=0; j<nx; ++j) if(std::abs(A(i,j))>1e-9) A_triplets.emplace_back(constr_idx+i, idx_x_k+j, -A(i,j));
            for(int j=0; j<nu; ++j) if(std::abs(B(i,j))>1e-9) A_triplets.emplace_back(constr_idx+i, idx_u_k+j, -B(i,j));
            lb(constr_idx+i) = g_vec(i); ub(constr_idx+i) = g_vec(i);
        }
        constr_idx += nx;

        for(int leg=0; leg<2; leg++) {
            int u_fx = idx_u_k + leg*2;
            int u_fz = idx_u_k + leg*2 + 1;

            A_triplets.emplace_back(constr_idx, u_fx, 1.0); A_triplets.emplace_back(constr_idx, u_fz, -MU_CTRL);
            lb(constr_idx) = -OsqpEigen::INFTY; ub(constr_idx) = 0.0; constr_idx++;
            A_triplets.emplace_back(constr_idx, u_fx, 1.0); A_triplets.emplace_back(constr_idx, u_fz, MU_CTRL);
            lb(constr_idx) = 0.0; ub(constr_idx) = OsqpEigen::INFTY; constr_idx++;
            A_triplets.emplace_back(constr_idx, u_fz, 1.0);
            lb(constr_idx) = FZ_MIN; ub(constr_idx) = FZ_MAX; constr_idx++;
        }
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constr);

    Eigen::SparseMatrix<double> P_sparse(n_vars, n_vars); P_sparse.setFromTriplets(P_triplets.begin(), P_triplets.end());
    Eigen::SparseMatrix<double> A_sparse(n_constr, n_vars); A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());

    if (!solver.data()->setHessianMatrix(P_sparse)) return false;
    if (!solver.data()->setGradient(q_vec)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) return false;
    if (!solver.data()->setLowerBound(lb)) return false;
    if (!solver.data()->setUpperBound(ub)) return false;

    if (!solver.initSolver()) return false;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

    Eigen::VectorXd solution = solver.getSolution();
    optimal_force = solution.segment(nx, nu);
    return true;
}

// --- Controller Wrapper ---
void run_mpc_controller(mjModel* m, mjData* d) {
    if (debug_file.is_open()) debug_file << "\n[T=" << std::fixed << std::setprecision(4) << d->time << "] ";

    // 1. Startup Check
    bool contacts = check_double_support(m, d);

    if (d->time < STARTUP_DELAY || !contacts) {
        apply_damping(d);
        global_mpc_status = 0;
        if (debug_file.is_open()) {
            debug_file << "Status: DAMPING";
            debug_file.flush();
        }
        return;
    }

    // 2. MPC Update
    if (d->time - last_mpc_time >= MPC_DT) {
        Eigen::VectorXd f_opt(4);
        if (solve_mpc(m, d, f_opt)) {
            if (has_nan(f_opt)) {
                if (debug_file.is_open()) debug_file << " [FATAL] MPC NaN! ";
                global_mpc_status = -1;
            } else {
                global_mpc_grf[0] = f_opt(0); global_mpc_grf[1] = f_opt(1);
                global_mpc_grf[2] = f_opt(2); global_mpc_grf[3] = f_opt(3);
                global_mpc_status = 1;
                if (debug_file.is_open()) {
                    debug_file << "[MPC_OUT] ";
                    log_vec("F_opt", f_opt);
                }
            }
        } else {
            global_mpc_status = -2;
            if (debug_file.is_open()) debug_file << " [ERROR] MPC Infeasible! ";
        }
        last_mpc_time = d->time;
    } else {
        if (debug_file.is_open()) debug_file << " [WBC_HOLD] ";
    }

    // 3. WBC
    int nv = m->nv;
    std::vector<mjtNum> jacp_L(3 * nv);
    std::vector<mjtNum> jacp_R(3 * nv);
    int l_site = mj_name2id(m, mjOBJ_SITE, "left_foot");
    int r_site = mj_name2id(m, mjOBJ_SITE, "right_foot");

    mj_jacSite(m, d, jacp_L.data(), NULL, l_site);
    mj_jacSite(m, d, jacp_R.data(), NULL, r_site);

    Eigen::MatrixXd J_L_T(nv, 2);
    Eigen::MatrixXd J_R_T(nv, 2);
    for(int i=0; i<nv; i++) {
        J_L_T(i, 0) = jacp_L[0*nv+i]; J_L_T(i, 1) = jacp_L[2*nv+i];
        J_R_T(i, 0) = jacp_R[0*nv+i]; J_R_T(i, 1) = jacp_R[2*nv+i];
    }

    Eigen::Vector2d F_L(global_mpc_grf[0], global_mpc_grf[1]);
    Eigen::Vector2d F_R(global_mpc_grf[2], global_mpc_grf[3]);

    Eigen::VectorXd tau_mpc = J_L_T * F_L + J_R_T * F_R;

    // Apply with Full Logging Breakdown
    for(int i=0; i<4; i++) {
        int qvel_idx = 3 + i;
        double tau_grav = d->qfrc_bias[qvel_idx];
        double tau_damp = -KD_JOINT * d->qvel[qvel_idx];
        double tau_task = tau_mpc(qvel_idx);

        // Raw sum (NO CLAMP)
        double tau_total = tau_task + tau_grav + tau_damp;
        d->ctrl[i] = tau_total;

        // Forensic Logging for Actuator 0 (Left Hip) and 1 (Left Knee)
        if ((i == 0 || i == 1) && debug_file.is_open()) {
            debug_file << " [Act" << i << "] ";
            log_kv("T_MPC", tau_task);
            log_kv("T_Grav", tau_grav);
            log_kv("T_Damp", tau_damp);
            log_kv("T_Total", tau_total);
        }
    }

    if (debug_file.is_open()) debug_file.flush(); // FORCE WRITE
}

// --- Main ---
int main() {
    debug_file.open(DEBUG_LOG_PATH);
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(MODEL_PATH, 0, error, 1000);
    if (!m) { std::cerr << error << std::endl; return 1; }
    d = mj_makeData(m);

    int trunk_id = mj_name2id(m, mjOBJ_BODY, "trunk");
    if (trunk_id >= 0) {
        robot_mass = m->body_subtreemass[trunk_id];
        robot_inertia = m->body_inertia[trunk_id*3 + 1] * 1.5;
    } else {
        robot_mass = 9.0; robot_inertia = 0.2;
    }

    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MPC Balance (Forensic Log)", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    unsigned char* rgb_buffer = new unsigned char[width * height * 3];
    std::string cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s " +
                      std::to_string(width) + "x" + std::to_string(height) +
                      " -r 60 -i - -vf vflip -an -c:v libx264 -pix_fmt yuv420p -preset fast " + std::string(VIDEO_PATH);
    FILE* ffmpeg_pipe = popen(cmd.c_str(), "w");

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjv_makeScene(m, &scn, 2000);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0.5;
    cam.distance = 2.5; cam.azimuth = 90; cam.elevation = -10;

    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetKeyCallback(window, keyboard);

    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id >= 0) mj_resetDataKeyframe(m, d, key_id);
    mj_forward(m, d);

    std::ofstream dataFile(LOG_PATH);
    dataFile << "time,x,z,theta,tau1,tau2,tau3,tau4,FLx,FLz,FRx,FRz,mpc_status" << std::endl;

    while (!glfwWindowShouldClose(window) && d->time < SIM_DURATION) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            if (std::isnan(d->qpos[0])) {
                if (debug_file.is_open()) debug_file << "\n[FATAL] NaN Detected.\n";
                break;
            }

            mj_step1(m, d);
            run_mpc_controller(m, d);
            mj_step2(m, d);

            dataFile << d->time << "," << d->qpos[0] << "," << d->qpos[1] << "," << d->qpos[2] << ","
                     << d->ctrl[0] << "," << d->ctrl[1] << "," << d->ctrl[2] << "," << d->ctrl[3] << ","
                     << global_mpc_grf[0] << "," << global_mpc_grf[1] << "," << global_mpc_grf[2] << "," << global_mpc_grf[3] << ","
                     << global_mpc_status << std::endl;
        }
        if (std::isnan(d->qpos[0])) break;

        mjrRect viewport = {0, 0, width, height};
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        if (ffmpeg_pipe) {
            mjr_readPixels(rgb_buffer, NULL, viewport, &con);
            fwrite(rgb_buffer, 1, width * height * 3, ffmpeg_pipe);
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    if (ffmpeg_pipe) pclose(ffmpeg_pipe);
    delete[] rgb_buffer;
    debug_file.close();
    dataFile.close();
    mj_deleteData(d); mj_deleteModel(m);
    mjr_freeContext(&con); mjv_freeScene(&scn);
    glfwTerminate();
    return 0;
}