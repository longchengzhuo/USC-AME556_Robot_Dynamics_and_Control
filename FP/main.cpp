#include <iostream>
#include <vector>
#include <fstream>
#include <cmath> // std::min, std::max
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "SimulationUI.h"
#include "BipedRobot.h"

// 其他目标保持不变
const double DESIRED_X = 0.0;
const double DESIRED_PITCH = 0.0;
const double STAND_DURATION = 1.0;

int main(int argc, const char** argv) {
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("../robot.xml", 0, error, 1000);
    if (!m) {
        std::cerr << "Load Error: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    // 初始化数据记录文件
    std::ofstream log_file("../robot_data.txt");
    if (log_file.is_open()) {
        log_file << "time";
        for (int i = 0; i < m->nq; ++i) log_file << " q_" << i;
        for (int i = 0; i < m->nv; ++i) log_file << " v_" << i;
        for (int i = 0; i < m->nu; ++i) log_file << " u_" << i;
        log_file << " target_z"; // [新增] 记录一下目标高度，方便画图对比
        log_file << "\n";
    }

    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Biped Trajectory", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    SimulationUI::Init();
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    glfwSetKeyCallback(window, SimulationUI::keyboard);
    glfwSetCursorPosCallback(window, SimulationUI::mouse_move);
    glfwSetMouseButtonCallback(window, SimulationUI::mouse_button);
    glfwSetScrollCallback(window, SimulationUI::scroll);

    BipedRobot robot(m, d);
    glfwSetWindowUserPointer(window, &robot);

    robot.resetToKeyframe();

    cam.azimuth = 90;
    cam.elevation = -5;
    cam.distance = 1.5;
    cam.lookat[2] = 0.5;

    // 初始化视频录制
    SimulationUI::VideoRecorder recorder;
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    if (!recorder.Start("../test.mp4", fbw, fbh, 60)) {
        std::cerr << "Warning: Video recording failed to start." << std::endl;
    }

    // === 轨迹参数定义 ===
    // 为了更稳，我们让它先在 0.45m 处稳定 1 秒钟 (Warmup)，再开始执行任务
    double warmup_time = 1.0;
    double current_target_z;

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            if (!robot.getWarningMessage().empty()) {
                break;
            }

            double t_now = d->time;

            if (t_now < warmup_time) {
                current_target_z = 0.45;
            }
            else {
                double t = t_now - warmup_time;

                if (t <= 0.5) {
                    double ratio = t / 0.5;
                    current_target_z = 0.45 + ratio * (0.55 - 0.45);
                }
                else if (t <= 1.5) { // 0.5 + 1.0 = 1.5
                    double t_phase2 = t - 0.5;
                    double ratio = t_phase2 / 1.0;
                    current_target_z = 0.55 + ratio * (0.40 - 0.55);
                }
                else {
                    current_target_z = 0.40;
                }
            }

            // 将动态计算出的 z 传给控制器
            robot.stand(DESIRED_X, current_target_z, DESIRED_PITCH, STAND_DURATION);

            // 记录数据
            if (log_file.is_open()) {
                log_file << d->time;
                for (int i = 0; i < m->nq; ++i) log_file << " " << d->qpos[i];
                for (int i = 0; i < m->nv; ++i) log_file << " " << d->qvel[i];
                for (int i = 0; i < m->nu; ++i) log_file << " " << d->ctrl[i];
                log_file << " " << current_target_z; // 记录目标值
                log_file << "\n";
            }
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // [修改顺序] 1. 先绘制 Overlay (时间/警告)
        char time_str[50];
        sprintf(time_str, "Time: %.2f s", d->time);
        mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, viewport, time_str, NULL, &con);

        std::string warning = robot.getWarningMessage();
        if (!warning.empty()) {
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, viewport, warning.c_str(), NULL, &con);
        }

        // [修改顺序] 2. 再录制帧 (此时 Framebuffer 里已经有文字了)
        recorder.RecordFrame(viewport, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    recorder.Stop();
    if (log_file.is_open()) log_file.close();

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}