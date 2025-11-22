#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdio> // popen

// Graphics & Physics
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"

// Math & Solver
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

// --- Constants ---
const double PI = 3.14159265358979323846;
const double M_val = 1.0;
const double m_val = 0.2;
const double l_val = 0.3;
const double g_val = 9.8;

// --- MuJoCo Globals ---
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// --- Interaction ---
bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

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
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (button_right) action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left) action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else action = mjMOUSE_ZOOM;
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
// --- MPC Solver Class (Fixed & Verified) ---
class MPCSolver {
public:
    OsqpEigen::Solver solver;
    bool initialized = false;

    // Dimensions
    static constexpr int nx = 4; // State dim
    static constexpr int nu = 1; // Input dim

    // Horizon & Time
    int N;
    double dt;
    int n_vars;
    int n_cons;

    // [新增] 本地存储边界向量，用于在 solve 中快速修改和回传
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    void setup(int horizon_N, double sample_dt) {
        N = horizon_N;
        dt = sample_dt;
        n_vars = (N + 1) * nx + N * nu;
        n_cons = (N + 1) * nx + n_vars;

        // 1. Discretize Model (Forward Euler)
        Eigen::Matrix4d Ac = Eigen::Matrix4d::Zero();
        Ac(0, 2) = 1.0; Ac(1, 3) = 1.0;
        Ac(2, 1) = (m_val * g_val) / M_val;
        Ac(3, 1) = (g_val * (M_val + m_val)) / (M_val * l_val);

        Eigen::Vector4d Bc_vec;
        Bc_vec << 0, 0, 1.0/M_val, 1.0/(M_val*l_val);

        Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(nx, nx) + Ac * dt;
        Eigen::MatrixXd Bd = Bc_vec * dt;

        // 2. Prepare Sparse Triplets
        std::vector<Eigen::Triplet<double>> P_triplets;
        std::vector<Eigen::Triplet<double>> A_triplets;
        P_triplets.reserve(n_vars);
        A_triplets.reserve(n_vars * 5);

        // Weights
        Eigen::Vector4d Q_diag; Q_diag << 100, 0.01, 100, 0.01;
        double R_val = 1.0;

        // 3. Build Matrices
        for (int k = 0; k <= N; ++k) {
            int x_idx = k * (nx + nu);
            int u_idx = x_idx + nx;

            // --- Hessian P ---
            for (int i = 0; i < nx; ++i)
                P_triplets.emplace_back(x_idx + i, x_idx + i, Q_diag(i));

            if (k < N)
                P_triplets.emplace_back(u_idx, u_idx, R_val);

            // --- Constraint Matrix A ---
            // Part 1: Variable Identity (I * z)
            for (int i = 0; i < nx; ++i)
                A_triplets.emplace_back((N + 1) * nx + x_idx + i, x_idx + i, 1.0);
            if (k < N)
                A_triplets.emplace_back((N + 1) * nx + u_idx, u_idx, 1.0);

            // Part 2: Dynamics
            if (k < N) {
                int row_dyn = (k + 1) * nx;
                int next_x_idx = (k + 1) * (nx + nu);

                for (int r = 0; r < nx; ++r)
                    for (int c = 0; c < nx; ++c)
                        if (std::abs(Ad(r, c)) > 1e-9)
                            A_triplets.emplace_back(row_dyn + r, x_idx + c, -Ad(r, c));

                for (int r = 0; r < nx; ++r)
                    if (std::abs(Bd(r, 0)) > 1e-9)
                        A_triplets.emplace_back(row_dyn + r, u_idx, -Bd(r, 0));

                for (int r = 0; r < nx; ++r)
                    A_triplets.emplace_back(row_dyn + r, next_x_idx + r, 1.0);
            }
        }

        // Part 3: Initial State Constraint
        for (int i = 0; i < nx; ++i)
            A_triplets.emplace_back(i, i, 1.0);

        Eigen::SparseMatrix<double> P(n_vars, n_vars);
        Eigen::SparseMatrix<double> A(n_cons, n_vars);
        P.setFromTriplets(P_triplets.begin(), P_triplets.end());
        A.setFromTriplets(A_triplets.begin(), A_triplets.end());

        // 4. Bounds Setup (Save to member variables)
        lowerBound = Eigen::VectorXd::Zero(n_cons);
        upperBound = Eigen::VectorXd::Zero(n_cons);

        // Box Constraints
        int box_start = (N + 1) * nx;
        for (int k = 0; k <= N; ++k) {
            int offset = box_start + k * (nx + nu);

            lowerBound.segment<2>(offset) << -0.8, -PI/4.0;
            upperBound.segment<2>(offset) <<  0.8,  PI/4.0;
            lowerBound.segment<2>(offset+2).setConstant(-OsqpEigen::INFTY);
            upperBound.segment<2>(offset+2).setConstant(OsqpEigen::INFTY);

            if (k < N) {
                lowerBound(offset + 4) = -10.0;
                upperBound(offset + 4) =  10.0;
            }
        }

        // 5. Init Solver
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(n_vars);
        solver.data()->setNumberOfConstraints(n_cons);
        solver.data()->setHessianMatrix(P);

        // [修复1] 创建显式变量 q，解决 rvalue 报错
        Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);
        solver.data()->setGradient(q);

        solver.data()->setLinearConstraintsMatrix(A);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        if(!solver.initSolver()) std::cerr << "MPC Init Failed" << std::endl;
        initialized = true;
    }

    double solve(const Eigen::Vector4d& x0) {
        if (!initialized) return 0.0;

        // [修复2] 直接修改成员变量 lowerBound/upperBound，不需要 getLowerBound
        // Update Initial State Constraint (Rows 0..3)
        lowerBound.head(nx) = x0;
        upperBound.head(nx) = x0;

        // 将更新后的完整边界传回求解器
        solver.updateBounds(lowerBound, upperBound);

        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 0.0;

        return solver.getSolution()(4);
    }
};

// --- Simulation Runner ---

void run_simulation() {
    // 1. Setup Window
    int w = 1200, h = 900;
    GLFWwindow* window = glfwCreateWindow(w, h, "MPC Cart-Pole (Closed-Loop)", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0;
    cam.distance = 2.5; cam.azimuth = 90; cam.elevation = -10;

    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // 2. Init MPC
    MPCSolver mpc;
    // Problem specs: N=20, dt=50ms=0.05s
    mpc.setup(20, 0.05);

    // 3. Reset Physics
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);

    int aid = mj_name2id(m, mjOBJ_ACTUATOR, "u");
    int sid = mj_name2id(m, mjOBJ_JOINT, "slide");
    int hid = mj_name2id(m, mjOBJ_JOINT, "hinge");

    std::ofstream csv("../MPC_result.csv");
    csv << "t,x,theta,u\n";

    // Video Pipe
    std::string cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s 1200x900 -r 60 -i - -c:v libx264 -pix_fmt yuv420p -vf vflip -loglevel error ../video_mpc.mp4";
    #ifdef _WIN32
    FILE* pipe = _popen(cmd.c_str(), "wb");
    #else
    FILE* pipe = popen(cmd.c_str(), "w");
    #endif
    std::vector<unsigned char> buffer(w * h * 3);

    // --- Main Loop with Time Logic ---
    double T = 15.0;
    double sim_dt = m->opt.timestep; // Likely 0.001 or 0.002 from XML
    double mpc_dt = 0.05;

    double last_mpc_time = -1.0; // Force run on first step
    double current_u = 0.0;

    while (!glfwWindowShouldClose(window) && d->time < T) {

        // Run physics until simulation time catches up to next frame or just do fixed steps
        // Here we simulate 1 frame worth of steps (approx 1/60s) to keep rendering smooth
        // But MPC check happens inside physics stepping

        for (int i = 0; i < 17; ++i) { // ~60fps visual
            if (d->time >= T) break;

            // --- MPC Logic ---
            // Check if enough time has passed since last MPC update
            if (d->time - last_mpc_time >= mpc_dt || last_mpc_time < 0) {

                // 1. Get Current State
                Eigen::Vector4d state;
                state(0) = d->qpos[m->jnt_qposadr[sid]];     // x
                state(1) = -d->qpos[m->jnt_qposadr[hid]];    // theta (Handle sign convention!)
                state(2) = d->qvel[m->jnt_dofadr[sid]];      // x_dot
                state(3) = -d->qvel[m->jnt_dofadr[hid]];     // theta_dot

                // 2. Solve MPC
                current_u = mpc.solve(state);

                // 3. Update timer
                last_mpc_time = d->time;

                // Debug output occasionally
                // std::cout << "t=" << d->time << " MPC u=" << current_u << std::endl;
            }

            // Apply Control (Zero-Order Hold: Keep same u until next MPC step)
            d->ctrl[aid] = current_u;

            mj_step(m, d);

            // Log
            csv << d->time << ","
                << d->qpos[m->jnt_qposadr[sid]] << ","
                << -d->qpos[m->jnt_qposadr[hid]] << ","
                << d->ctrl[aid] << "\n";
        }

        mjrRect viewport = {0, 0, w, h};
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        if (pipe) {
            mjr_readPixels(buffer.data(), nullptr, viewport, &con);
            fwrite(buffer.data(), 1, buffer.size(), pipe);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    if (pipe) {
        #ifdef _WIN32
        _pclose(pipe);
        #else
        pclose(pipe);
        #endif
        std::cout << "Video saved." << std::endl;
    }

    csv.close();
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
}

int main() {
    if (!glfwInit()) return 1;

    std::string src = __FILE__;
    std::string xml = src.substr(0, src.find_last_of("/\\")) + "/cartpole.xml";

    char err[1024] = {0};
    m = mj_loadXML(xml.c_str(), nullptr, err, 1024);
    if (!m) {
        std::cerr << "Load Error: " << err << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    run_simulation();

    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}