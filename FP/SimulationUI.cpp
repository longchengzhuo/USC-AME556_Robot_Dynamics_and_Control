#include "SimulationUI.h"
#include "BipedRobot.h"
#include <cstring>
#include <vector>
#include <iostream>

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

    // === [新增] 封装后的初始化函数 ===
    GLFWwindow* SetupWindow(mjModel* m, const char* title) {
        if (!glfwInit()) return nullptr;

        GLFWwindow* window = glfwCreateWindow(1200, 900, title, NULL, NULL);
        if (!window) {
            glfwTerminate();
            return nullptr;
        }

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // 初始化 MuJoCo 可视化结构
        Init();
        mjv_makeScene(m, &scn, 2000);
        mjr_makeContext(m, &con, mjFONTSCALE_150);

        // 统一绑定回调
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);

        return window;
    }

    void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
        BipedRobot* robot = static_cast<BipedRobot*>(glfwGetWindowUserPointer(window));

        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
            mj_resetData(m, d);
            mj_forward(m, d);
        }

        if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
            if (robot) {
                robot->resetToKeyframe();
            }
        }

        if (act == GLFW_PRESS && key == GLFW_KEY_R) {
            if (robot) {
                std::vector<double> custom_state = {
                    0.0, 0.6, 0.0, 0.5, -1.0, 1.0, -1.0
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

    // === 视频录制器实现 ===
    VideoRecorder::VideoRecorder() : ffmpeg_pipe_(nullptr), image_buffer_(nullptr), width_(0), height_(0) {}

    VideoRecorder::~VideoRecorder() {
        Stop();
    }

    bool VideoRecorder::Start(const char* filename, int width, int height, int fps) {
        width_ = width;
        height_ = height;
        image_buffer_ = new unsigned char[width * height * 3];

        char cmd[1024];
        sprintf(cmd, "ffmpeg -y -loglevel error -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - -vf vflip -an -c:v libx264 -preset fast -pix_fmt yuv420p \"%s\"",
                width, height, fps, filename);

        std::cout << "[VideoRecorder] Start recording to " << filename << " (" << width << "x" << height << " @ " << fps << "fps)" << std::endl;

#ifdef _WIN32
        ffmpeg_pipe_ = _popen(cmd, "wb");
#else
        ffmpeg_pipe_ = popen(cmd, "w");
#endif

        if (!ffmpeg_pipe_) {
            std::cerr << "[VideoRecorder] Error: Failed to open ffmpeg pipe. Please ensure ffmpeg is installed." << std::endl;
            return false;
        }
        return true;
    }

    void VideoRecorder::RecordFrame(const mjrRect& viewport, const mjrContext* con) {
        if (!ffmpeg_pipe_) return;

        if (viewport.width != width_ || viewport.height != height_) {
            return;
        }

        mjr_readPixels(image_buffer_, nullptr, viewport, con);
        fwrite(image_buffer_, 1, width_ * height_ * 3, ffmpeg_pipe_);
    }

    void VideoRecorder::Stop() {
        if (ffmpeg_pipe_) {
#ifdef _WIN32
            _pclose(ffmpeg_pipe_);
#else
            pclose(ffmpeg_pipe_);
#endif
            ffmpeg_pipe_ = nullptr;
            std::cout << "[VideoRecorder] Recording saved." << std::endl;
        }

        if (image_buffer_) {
            delete[] image_buffer_;
            image_buffer_ = nullptr;
        }
    }
}