/**
 * @file main.cpp
 * @brief Main simulation loop for biped robot walking demonstration.
 *
 * Implements a hierarchical task scheduler that coordinates standing,
 * forward walking, and backward walking phases with real-time visualization.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "SimulationUI.h"
#include "BipedRobot.h"

/**
 * @brief Task scheduler for coordinating robot locomotion phases.
 */
class Task {
public:
    Task(BipedRobot& robot, mjModel* m, mjData* d)
        : robot_(robot), m_(m), d_(d) {}

    /**
     * @brief Free fall test from elevated position.
     */
    void task_one() {
        if (d_->time == 0.0) {
            std::vector<double> drop_state = {0.0, 0.8, 0.2, 0.5, -1.0, 0.3, -1.0};
            robot_.resetToState(drop_state);
        }
        robot_.freeFall();
    }

    /**
     * @brief Execute main walking demonstration sequence.
     *
     * Phase timeline (t = time after 1s warmup):
     * - t <= 2.0s: Stand up and down motion
     * - 2.0s < t <= 5.0s: Prepare for walking
     * - 5.0s < t <= 11.44s: Forward walking
     * - 11.44s < t <= 13.0s: Standing transition
     * - 13.0s < t <= 18.0s: Backward walking (left foot first)
     * - t > 18.0s: Final standing
     *
     * @return Current status message for display.
     */
    std::string task_two() {
        const double warmup_time = 1.0;
        double t_now = d_->time;
        double t = t_now - warmup_time;
        std::string status_message;

        if (t_now < warmup_time) {
            robot_.stand(DESIRED_X, 0.45, DESIRED_PITCH);
            status_message = "";
        }
        else if (t <= 2.0) {
            double current_target_z;
            if (t <= 0.5) {
                double ratio = t / 0.5;
                current_target_z = 0.45 + ratio * (0.55 - 0.45);
            } else if (t <= 1.5) {
                double t_phase = t - 0.5;
                double ratio = t_phase / 1.0;
                current_target_z = 0.55 + ratio * (0.40 - 0.55);
            } else {
                current_target_z = 0.40;
            }
            robot_.stand(DESIRED_X, current_target_z, DESIRED_PITCH);
            status_message = "Standing Up and Down";
        }
        else if (t <= 5.0) {
            double current_target_z = DESIRED_Z;
            robot_.stand(DESIRED_X, current_target_z, DESIRED_PITCH);
            status_message = "Preparing to Walk";
        }
        else if (t <= 11.44) {
            robot_.forwardWalk(0.2072, DESIRED_Z, 0.05);
            DESIRED_X = d_->qpos[0];
            status_message = "Walking Forward";
        }
        else if (t <= 13.0) {
            robot_.stand(DESIRED_X, DESIRED_Z, DESIRED_PITCH);
            status_message = "Standing";
        }
        else if (t <= 18.515) {
            robot_.backwardWalkLeftFirst(0.1969, DESIRED_Z, -0.01);
            DESIRED_X = d_->qpos[0];
            status_message = "Walking Backward";
        }
        else {
            robot_.stand(DESIRED_X, DESIRED_Z, DESIRED_PITCH);
            status_message = "Standing";
        }

        return status_message;
    }

private:
    BipedRobot& robot_;
    mjModel* m_;
    mjData* d_;

    double DESIRED_X = 0.0;
    double DESIRED_Z = 0.48;
    double DESIRED_PITCH = 0.0;
};

int main(int argc, const char** argv) {
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("../robot.xml", 0, error, 1000);
    if (!m) return 1;
    d = mj_makeData(m);

    GLFWwindow* window = SimulationUI::SetupWindow(m, "MuJoCo Biped Walking Demo");
    if (!window) return 1;

    BipedRobot robot(m, d);
    Task task(robot, m, d);

    glfwSetWindowUserPointer(window, &robot);

    robot.resetToKeyframe();
    cam.azimuth = 90;
    cam.elevation = -5;
    cam.distance = 1.5;
    cam.lookat[2] = 0.5;

    SimulationUI::VideoRecorder recorder;
    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    recorder.Start("../test.mp4", fbw, fbh, 60);

    SimulationUI::DataLogger logger;
    logger.start("../robot_data.csv");

    std::string current_status;

    while (!glfwWindowShouldClose(window)) {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) {
            if (!robot.getWarningMessage().empty()) break;
            current_status = task.task_two();
            logger.log(d);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        SimulationUI::OverlayRenderer::render(
            viewport, &con,
            d->time,
            d->qpos[0], d->qpos[1], d->qpos[2],
            d->qvel[0],
            current_status,
            robot.getWarningMessage()
        );

        recorder.RecordFrame(viewport, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    recorder.Stop();
    logger.stop();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}
