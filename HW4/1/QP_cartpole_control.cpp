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
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"

// --- MuJoCo Globals ---
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// --- Mouse Interaction (Minimal) ---
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

// --- Helper: Read LQR Data ---
std::vector<double> readLQRData(const std::string& filename) {
    std::vector<double> data;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Warning: Cannot open " << filename << std::endl;
        return data;
    }

    // Skip header: t,x,theta,u
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string seg;
        std::vector<std::string> row;

        while (std::getline(ss, seg, ',')) row.push_back(seg);

        // u is at index 3
        if (row.size() >= 4) data.push_back(std::stod(row[3]));
    }
    std::cout << "Loaded " << data.size() << " steps from " << filename << std::endl;
    return data;
}

// --- Solver: QP ---
std::vector<double> solveQP(const std::vector<double>& u_ref) {
    std::vector<double> u_star;
    if (u_ref.empty()) return u_star;

    std::cout << "Solving QP..." << std::endl;

    // Problem: min 0.5 * u^T * P * u + q^T * u
    // P = 2.0, q = -2.0 * u_ref
    // s.t. -10 <= u <= 10

    Eigen::SparseMatrix<double> P(1, 1); P.insert(0, 0) = 2.0;
    Eigen::SparseMatrix<double> A(1, 1); A.insert(0, 0) = 1.0;
    Eigen::VectorXd l(1); l << -10.0;
    Eigen::VectorXd u(1); u <<  10.0;
    Eigen::VectorXd q(1); q << 0;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(1);
    solver.data()->setNumberOfConstraints(1);

    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    solver.initSolver();

    for (double ref : u_ref) {
        q(0) = -2.0 * ref;
        solver.updateGradient(q);
        solver.solveProblem();
        u_star.push_back(solver.getSolution()(0));
    }

    return u_star;
}

// --- Main Simulation ---
void run_simulation(const std::vector<double>& u_seq) {
    int w = 1200, h = 900;
    GLFWwindow* window = glfwCreateWindow(w, h, "QP Cart-Pole", NULL, NULL);
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

    // Initial State
    mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);

    int aid = mj_name2id(m, mjOBJ_ACTUATOR, "u");
    int sid = mj_name2id(m, mjOBJ_JOINT, "slide");
    int hid = mj_name2id(m, mjOBJ_JOINT, "hinge");

    std::ofstream csv("../QP_result.csv");
    csv << "t,x,theta,u\n";

    // Video Pipe
    std::string cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s 1200x900 -r 60 -i - -c:v libx264 -pix_fmt yuv420p -vf vflip -loglevel error ../video_qp.mp4";
    #ifdef _WIN32
    FILE* pipe = _popen(cmd.c_str(), "wb");
    #else
    FILE* pipe = popen(cmd.c_str(), "w");
    #endif
    std::vector<unsigned char> buffer(w * h * 3);

    double T = 2.0;
    double dt = 0.001;
    int steps_per_frame = 17;

    while (!glfwWindowShouldClose(window) && d->time < T) {
        for (int i = 0; i < steps_per_frame; ++i) {
            if (d->time >= T) break;

            // Control
            int idx = (int)(d->time / dt);
            double ctrl = (idx < u_seq.size()) ? u_seq[idx] : 0.0;
            d->ctrl[aid] = ctrl;

            mj_step(m, d);

            // Log
            csv << d->time << ","
                << d->qpos[m->jnt_qposadr[sid]] << ","
                << -d->qpos[m->jnt_qposadr[hid]] << ","
                << ctrl << "\n";
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

    // Path trick to find xml
    std::string src = __FILE__;
    std::string xml = src.substr(0, src.find_last_of("/\\")) + "/cartpole.xml";

    char err[1024] = {0};
    m = mj_loadXML(xml.c_str(), nullptr, err, 1024);
    if (!m) return 1;
    d = mj_makeData(m);

    // 1. Load LQR Ref
    // 假设文件在 build 目录的上级 (根据之前的逻辑)
    auto u_lqr = readLQRData("../LQR_result_case1.csv");
    if (u_lqr.empty()) u_lqr = readLQRData("LQR_result_case1.csv"); // Fallback

    // 2. Solve QP
    auto u_qp = solveQP(u_lqr);

    // 3. Run
    if (!u_qp.empty()) run_simulation(u_qp);

    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}