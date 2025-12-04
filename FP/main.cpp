#include <iostream>
#include <vector>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "SimulationUI.h"
#include "BipedRobot.h"

// 定义站立控制的目标参数
const double DESIRED_Z = 4.5;
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

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            if (!robot.getWarningMessage().empty()) {
                break;
            }
            robot.freeFall();
            // === 修改处：使用 stand 替代 freeFall ===
            // 每一帧都将目标参数传入控制器
            // robot.stand(DESIRED_X, DESIRED_Z, DESIRED_PITCH, STAND_DURATION);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

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

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}