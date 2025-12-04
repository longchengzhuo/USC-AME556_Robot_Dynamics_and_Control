#include "SimulationUI.h"
#include <cstring> // for memset

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
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
            mj_resetData(m, d);
            mj_forward(m, d);
        }
        if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
            // 按照 XML keyframe 重置
            int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
            if (key_id >= 0) mj_resetDataKeyframe(m, d, key_id);
            else mj_resetData(m, d);
            mj_forward(m, d);
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