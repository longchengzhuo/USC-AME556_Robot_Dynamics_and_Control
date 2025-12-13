#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "SimulationUI.h"
#include "BipedRobot.h"

// === Task 类定义 ===
class Task {
public:
    Task(BipedRobot& robot, mjModel* m, mjData* d)
        : robot_(robot), m_(m), d_(d) {}

    // 任务一：高空自由落体测试
    void task_one() {
        if (d_->time == 0.0) {
            std::vector<double> drop_state = {0.0, 0.8, 0.2, 0.5, -1.0, 0.3, -1.0};
            // std::vector<double> drop_state = {0,0.465,0,1.0472,-1.5708,0.523598776,-1.5708};
            robot_.resetToState(drop_state);
        }
        robot_.freeFall();
    }

    // 任务二：动态下蹲起立轨迹
    double task_two() {
        const double warmup_time = 1.0;
        double current_target_z;
        double t_now = d_->time;

        if (t_now < warmup_time) {
            current_target_z = 0.45;
        } else {
            double t = t_now - warmup_time;

            if (t <= 0.5) {
                double ratio = t / 0.5;
                current_target_z = 0.45 + ratio * (0.55 - 0.45);
            } else if (t <= 1.5) {
                double t_phase2 = t - 0.5;
                double ratio = t_phase2 / 1.0;
                current_target_z = 0.55 + ratio * (0.40 - 0.55);
            } else {
                current_target_z = 0.40;
            }
        }

        // 使用类内部的参数
        robot_.stand(DESIRED_X, current_target_z, DESIRED_PITCH, STAND_DURATION);

        return current_target_z;
    }

    // [新增] 任务三：先稳定站立，后行走
    void task_three() {
        // 前 1.0 秒：稳定在 0.48 米高度
        // 使用与 task_two 类似的 stand 方法，但固定高度
        if (d_->time < 1.0) {
            robot_.stand(DESIRED_X, 0.48, DESIRED_PITCH, STAND_DURATION);
        }
        else {
            robot_.walk(0.211);
        }
    }

private:
    BipedRobot& robot_;
    mjModel* m_;
    mjData* d_;

    // [修改] 参数移入类内部
    const double DESIRED_X = 0.0;
    const double DESIRED_PITCH = 0.0;
    const double STAND_DURATION = 1.0;
};

// === Main Function ===
int main(int argc, const char** argv) {
    // 1. 加载模型
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("../robot.xml", 0, error, 1000);
    if (!m) return 1;
    d = mj_makeData(m);


    // 3. [修改] 使用 SimulationUI 一键初始化窗口和 Context
    GLFWwindow* window = SimulationUI::SetupWindow(m, "MuJoCo Biped Task");
    if (!window) return 1;

    // 4. 创建机器人和任务
    BipedRobot robot(m, d);
    Task task(robot, m, d);

    // [重要] 将 robot 指针传给窗口，供回调函数使用
    glfwSetWindowUserPointer(window, &robot);

    robot.resetToKeyframe();
    // 设置初始相机视角
    cam.azimuth = 90;
    cam.elevation = -5;
    cam.distance = 1.5;
    cam.lookat[2] = 0.5;

    // 视频录制
    SimulationUI::VideoRecorder recorder;
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    recorder.Start("../test.mp4", fbw, fbh, 60);

    // === 主循环 ===
    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0/60.0) {
            if (!robot.getWarningMessage().empty()) break;

            // 切换任务：这里注释掉 task_two，调用 task_three
            // task.task_one();
            // double target_z = task.task_two();
            task.task_three();
        }

        // 渲染与UI更新
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        char time_str[50];
        sprintf(time_str, "Time: %.2f s", d->time);
        mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, viewport, time_str, NULL, &con);

        std::string warning = robot.getWarningMessage();
        if (!warning.empty()) {
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, viewport, warning.c_str(), NULL, &con);
        }

        recorder.RecordFrame(viewport, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    recorder.Stop();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}