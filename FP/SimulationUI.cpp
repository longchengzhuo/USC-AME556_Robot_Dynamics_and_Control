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

    // === [新增] 视频录制器实现 ===
    VideoRecorder::VideoRecorder() : ffmpeg_pipe_(nullptr), image_buffer_(nullptr), width_(0), height_(0) {}

    VideoRecorder::~VideoRecorder() {
        Stop();
    }

    bool VideoRecorder::Start(const char* filename, int width, int height, int fps) {
        width_ = width;
        height_ = height;
        // 分配 RGB 缓冲区 (3 bytes per pixel)
        image_buffer_ = new unsigned char[width * height * 3];

        // 构建 ffmpeg 命令
        // -y: 覆盖输出文件
        // -f rawvideo -vcodec rawvideo -pix_fmt rgb24: 输入格式为原始 RGB
        // -s WxH: 分辨率
        // -r fps: 帧率
        // -i -: 从标准输入读取
        // -vf vflip: 垂直翻转 (因为 OpenGL 是左下角原点)
        // -c:v libx264 -preset fast -pix_fmt yuv420p: 编码为兼容性好的 H.264 MP4
        char cmd[1024];
        sprintf(cmd, "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - -vf vflip -an -c:v libx264 -preset fast -pix_fmt yuv420p \"%s\"",
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

        // 简单的尺寸检查，防止窗口缩放导致 ffmpeg 数据流错乱
        if (viewport.width != width_ || viewport.height != height_) {
            // 如果窗口大小变了，简单起见我们跳过这一帧，或者你应该在 Start 时锁定窗口大小
            // 这里我们只录制初始化时的大小
            return;
        }

        // 读取 OpenGL 像素
        mjr_readPixels(image_buffer_, nullptr, viewport, con);

        // 写入管道
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