#include <iostream>
#include <vector>
#include <fstream>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "SimulationUI.h"
#include "BipedRobot.h"

// 定义站立控制的目标参数
const double DESIRED_Z = 0.55; // 修正为合理的站立高度
const double DESIRED_X = 0.0;
const double DESIRED_PITCH = 0.0;
const double STAND_DURATION = 1.0; // 秒

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
        log_file << "\n";
    }

    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Biped Simulation", NULL, NULL);
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

    // === [新增] 初始化视频录制器 ===
    SimulationUI::VideoRecorder recorder;
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    // 录制到上级目录的 test.mp4，帧率设为 60
    if (!recorder.Start("../test.mp4", fbw, fbh, 60)) {
        std::cerr << "Warning: Video recording failed to start." << std::endl;
    }

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            if (!robot.getWarningMessage().empty()) {
                break;
            }

            // 执行控制与步进
            robot.stand(DESIRED_X, DESIRED_Z, DESIRED_PITCH, STAND_DURATION);

            // 记录数据
            if (log_file.is_open()) {
                log_file << d->time;
                for (int i = 0; i < m->nq; ++i) log_file << " " << d->qpos[i];
                for (int i = 0; i < m->nv; ++i) log_file << " " << d->qvel[i];
                for (int i = 0; i < m->nu; ++i) log_file << " " << d->ctrl[i];
                log_file << "\n";
            }
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // === [新增] 录制当前帧 ===
        // 在渲染(render)之后，交换缓冲区(SwapBuffers)之前调用
        recorder.RecordFrame(viewport, &con);

        char time_str[50];
        sprintf(time_str, "Time: %.2f s", d->time);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, time_str, NULL, &con);

        std::string warning = robot.getWarningMessage();
        if (!warning.empty()) {
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, viewport, warning.c_str(), NULL, &con);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // === [新增] 停止录制 ===
    recorder.Stop();

    if (log_file.is_open()) log_file.close();

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}