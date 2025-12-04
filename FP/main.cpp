#include <iostream>
#include <vector>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "SimulationUI.h"
#include "BipedRobot.h"

int main(int argc, const char** argv) {
    // 1. 加载模型
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("../robot.xml", 0, error, 1000);
    if (!m) {
        std::cerr << "Load Error: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    // 2. 初始化图形界面
    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Biped Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    SimulationUI::Init(); // 初始化 mjv 结构
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 绑定回调
    glfwSetKeyCallback(window, SimulationUI::keyboard);
    glfwSetCursorPosCallback(window, SimulationUI::mouse_move);
    glfwSetMouseButtonCallback(window, SimulationUI::mouse_button);
    glfwSetScrollCallback(window, SimulationUI::scroll);

    // 3. 实例化机器人对象
    BipedRobot robot(m, d);

    // 初始化状态 (如果有命令行参数则解析，这里演示默认逻辑)
    // 如果想要自定义初始化： robot.resetToState({ ... });
    robot.resetToKeyframe();

    // 设置摄像机视角
    cam.azimuth = 90;
    cam.elevation = -5;
    cam.distance = 1.5;
    cam.lookat[2] = 0.5;

    // 4. 仿真主循环
    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            // 检查是否违规
            if (!robot.getWarningMessage().empty()) {
                break; // 退出物理循环，去渲染画面和警告
            }
            robot.freeFall();
        }

        // 获取视图
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // 更新场景
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // === 核心功能：在屏幕上显示时间 ===
        char time_str[50];
        sprintf(time_str, "Time: %.2f s", d->time);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, time_str, NULL, &con);

        // === 核心功能：显示约束警告 ===
        std::string warning = robot.getWarningMessage();
        if (!warning.empty()) {
            // 红色大字体显示警告
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, viewport, warning.c_str(), "1 0 0", &con);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 清理
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}