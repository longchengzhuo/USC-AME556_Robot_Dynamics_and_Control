#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <thread>
#include <chrono>
#include <iomanip>

// Graphics and Physics
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"

// Math
#include <Eigen/Dense>

// --- Constants ---
const double M_val = 1.0;
const double m_val = 0.2;
const double l_val = 0.3;
const double g_val = 9.8;
const double PI = 3.14159265358979323846;

// --- MuJoCo Globals ---
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// --- Interaction State ---
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// --- Callback Functions ---

// Keyboard: Space to reset to Keyframe
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
        // 重置到 XML 定义的 keyframe (30度倾斜)
        mj_resetDataKeyframe(m, d, 0);
        mj_forward(m, d);
    }
}

// Mouse Button
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

// Mouse Move
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

// Scroll
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// --- LQR Solver ---

Eigen::MatrixXd solveCARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                          const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd P_next;
    Eigen::MatrixXd R_inv = R.inverse();
    double dt = 0.001;
    double diff = 1.0;
    int max_iter = 10000000;

    for(int i=0; i<max_iter && diff > 1e-9; ++i) {
        Eigen::MatrixXd P_dot = A.transpose() * P + P * A - P * B * R_inv * B.transpose() * P + Q;
        P_next = P + P_dot * dt;
        diff = (P_next - P).norm();
        P = P_next;
    }
    return P;
}

Eigen::MatrixXd computeLQR(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    Eigen::Matrix4d A;
    A << 0, 0, 1, 0,
         0, 0, 0, 1,
         0, (m_val * g_val) / M_val, 0, 0,
         0, (g_val * (M_val + m_val)) / (M_val * l_val), 0, 0;

    Eigen::Vector4d B_vec;
    B_vec << 0, 0, 1.0 / M_val, 1.0 / (M_val * l_val);
    Eigen::MatrixXd B = B_vec;

    Eigen::MatrixXd P = solveCARE(A, B, Q, R);
    Eigen::MatrixXd K = R.inverse() * B.transpose() * P;

    std::cout << "K matrix: \n" << K << "\n\n";
    return K;
}

// --- Simulation Runner ---

struct SimConfig {
    Eigen::Matrix4d Q;
    double R_scalar;
    std::string name;
    std::string file_suffix;
};

void run_simulation(const SimConfig& config) {
    // 1. Calculate Control Gain
    Eigen::MatrixXd R(1,1);
    R(0,0) = config.R_scalar;
    Eigen::MatrixXd K = computeLQR(config.Q, R);

    // 2. Setup Window (Cleaned up)
    // 直接创建窗口，无需多余检查
    GLFWwindow* window = glfwCreateWindow(1200, 900, ("LQR Control: " + config.name).c_str(), NULL, NULL);
    if (!window) {
        mju_error("Could not create GLFW window");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 3. Init Visuals
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0;
    cam.distance = 2.5;
    cam.azimuth = 90;
    cam.elevation = -10;

    // 4. Register Callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // 5. Load Keyframe Initial State
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);

    int jid_slide = mj_name2id(m, mjOBJ_JOINT, "slide");
    int jid_hinge = mj_name2id(m, mjOBJ_JOINT, "hinge");
    int aid_u     = mj_name2id(m, mjOBJ_ACTUATOR, "u");

    std::ofstream file("../LQR_result_" + config.file_suffix + ".csv");
    file << "t,x,theta,u\n";

    std::cout << "Simulating " << config.name << "..." << std::endl;

    double T_total = 6.0;
    // 60Hz 刷新率下，每帧运行约 17ms 物理步长以匹配实时速度
    const int steps_per_frame = 17;

    while (!glfwWindowShouldClose(window) && d->time < T_total) {

        for (int i = 0; i < steps_per_frame; ++i) {
             if (d->time >= T_total) break;

            // --- 状态获取与符号修正 ---
            // MuJoCo (Right+) vs LQR (Left+) -> Theta 取反
            double x = d->qpos[m->jnt_qposadr[jid_slide]];
            double theta = -d->qpos[m->jnt_qposadr[jid_hinge]]; // 取反

            double x_dot = d->qvel[m->jnt_dofadr[jid_slide]];
            double theta_dot = -d->qvel[m->jnt_dofadr[jid_hinge]]; // 取反

            Eigen::Vector4d state;
            state << x, theta, x_dot, theta_dot;

            Eigen::VectorXd u_vec = -K * state;
            d->ctrl[aid_u] = u_vec(0);

            mj_step(m, d);

            // Log (保存 LQR 视角的 theta)
            file << d->time << "," << x << "," << theta << "," << d->ctrl[aid_u] << "\n";
        }

        // Render
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    std::cout << "Simulation finished at t=" << d->time << "s" << std::endl;

    file.close();
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
}

int main(int argc, char** argv) {
    if (!glfwInit()) mju_error("Could not initialize GLFW");

    std::string source_path = __FILE__;
    std::string dir_path = source_path.substr(0, source_path.find_last_of("/\\"));
    std::string xmlpath = dir_path + "/cartpole.xml";

    char error[1024] = {0};
    m = mj_loadXML(xmlpath.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Error loading XML: " << error << "\n";
        return 1;
    }
    d = mj_makeData(m);

    std::vector<SimConfig> configs;

    // Case 1
    Eigen::Matrix4d Q1 = Eigen::Matrix4d::Identity();
    Q1.diagonal() << 10, 10, 10, 10;
    configs.push_back({Q1, 1.0, "Case 1 (Q=10I, R=1)", "case1"});

    // Case 2
    Eigen::Matrix4d Q2 = Eigen::Matrix4d::Zero();
    Q2.diagonal() << 100, 0.01, 100, 0.01;
    configs.push_back({Q2, 10.0, "Case 2 (Q=Mixed, R=10)", "case2"});

    // Case 3
    Eigen::Matrix4d Q3 = Eigen::Matrix4d::Zero();
    Q3.diagonal() << 100, 0.01, 100, 0.01;
    configs.push_back({Q3, 1.0, "Case 3 (Q=Mixed, R=1)", "case3"});

    for (const auto& config : configs) {
        run_simulation(config);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "All simulations complete." << std::endl;

    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}