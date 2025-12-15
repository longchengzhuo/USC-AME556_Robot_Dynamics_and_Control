/**
 * @file SimulationUI.cpp
 * @brief Implementation of simulation UI, video recording, and overlay rendering.
 */

#include "SimulationUI.h"
#include "BipedRobot.h"
#include <cstring>
#include <vector>
#include <iostream>

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

GLFWwindow* SetupWindow(mjModel* m, const char* title) {
    if (!glfwInit()) return nullptr;

    GLFWwindow* window = glfwCreateWindow(1200, 900, title, NULL, NULL);
    if (!window) {
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    Init();
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

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

void OverlayRenderer::render(const mjrRect& viewport, const mjrContext* con,
                              double time, double trunk_x, double trunk_z, double trunk_pitch,
                              double trunk_vx, const std::string& status_message,
                              const std::string& warning_message) {
    char buffer[256];

    // Time display: large font, top-center position (using TOPRIGHT as MuJoCo doesn't have TOPCENTER)
    sprintf(buffer, "Time: %.2f s", time);
    mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, viewport, buffer, NULL, con);

    // Trunk state display: medium font, bottom-left position
    sprintf(buffer, "X: %.3f m\nZ: %.3f m\nPitch: %.2f deg\nVx: %.3f m/s",
            trunk_x, trunk_z, trunk_pitch * 180.0 / 3.14159265, trunk_vx);
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, buffer, NULL, con);

    // Status message: large font, top-center (approximate with large offset from left)
    if (!status_message.empty()) {
        // Create centered text by padding
        char centered_status[256];
        sprintf(centered_status, "                    %s", status_message.c_str());
        mjr_overlay(mjFONT_BIG, mjGRID_TOP, viewport, centered_status, NULL, con);
    }

    // Warning message: large font, top-right position
    if (!warning_message.empty()) {
        mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, viewport, warning_message.c_str(), NULL, con);
    }
}

DataLogger::DataLogger() : log_file_(nullptr), last_log_time_(-LOG_INTERVAL) {}

DataLogger::~DataLogger() {
    stop();
}

bool DataLogger::start(const char* filename) {
    log_file_ = fopen(filename, "w");
    if (!log_file_) {
        std::cerr << "[DataLogger] Error: Failed to open " << filename << std::endl;
        return false;
    }

    fprintf(log_file_, "time,");
    fprintf(log_file_, "q_x,q_z,q_pitch,q_lh,q_lk,q_rh,q_rk,");
    fprintf(log_file_, "v_x,v_z,v_pitch,v_lh,v_lk,v_rh,v_rk,");
    fprintf(log_file_, "tau_lh,tau_lk,tau_rh,tau_rk\n");

    last_log_time_ = -LOG_INTERVAL;
    std::cout << "[DataLogger] Start logging to " << filename << " (100Hz)" << std::endl;
    return true;
}

void DataLogger::log(const mjData* d) {
    if (!log_file_) return;

    if (d->time - last_log_time_ >= LOG_INTERVAL - 1e-6) {
        fprintf(log_file_, "%.4f,", d->time);

        for (int i = 0; i < 7; ++i) {
            fprintf(log_file_, "%.6f,", d->qpos[i]);
        }

        for (int i = 0; i < 7; ++i) {
            fprintf(log_file_, "%.6f,", d->qvel[i]);
        }

        for (int i = 0; i < 4; ++i) {
            fprintf(log_file_, "%.6f%s", d->ctrl[i], (i < 3) ? "," : "\n");
        }

        last_log_time_ = d->time;
    }
}

void DataLogger::stop() {
    if (log_file_) {
        fclose(log_file_);
        log_file_ = nullptr;
        std::cout << "[DataLogger] Logging stopped." << std::endl;
    }
}

}  // namespace SimulationUI
