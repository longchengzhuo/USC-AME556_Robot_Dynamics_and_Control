#include "SimulationUI.h"
#include "BipedRobot.h" // 需要引入 Robot 头文件以调用方法
#include <cstring>
#include <vector>

// 全局变量定义
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

namespace SimulationUI {

    void Init() {
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
    }

    void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
        // === 关键修改: 获取 Robot 实例 ===
        BipedRobot* robot = static_cast<BipedRobot*>(glfwGetWindowUserPointer(window));

        // 退格键：强制硬重置 (保留作为底层调试用)
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
            mj_resetData(m, d);
            mj_forward(m, d);
        }

        // 空格键：调用 Robot 的 resetToKeyframe
        if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
            if (robot) {
                robot->resetToKeyframe();
            }
        }

        // R 键：调用 Robot 的 resetToState (自定义初始状态)
        if (act == GLFW_PRESS && key == GLFW_KEY_R) {
            if (robot) {
                // 定义一个高空中的初始状态 (7维向量)
                // [trunk_x, trunk_z, trunk_ry, left_hip, left_knee, right_hip, right_knee]
                std::vector<double> custom_state = {
                    0.0,   // x
                    0.6,   // z (高空)
                    0.0,   // rotate_y
                    0.5,  // left hip (稍微抬起)
                    -1.0,   // left knee (弯曲)
                    1.0,  // right hip
                    -1.0    // right knee
                };
                robot->resetToState(custom_state);
            }
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

    void scroll(GLFWwindow* window, double xoffset, double yoffset) {
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
    }
}