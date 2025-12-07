#pragma once
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <cstdio> // FILE*

// 全局变量声明 (在 cpp 中定义)
extern mjModel* m;
extern mjData* d;
extern mjvCamera cam;
extern mjvOption opt;
extern mjvScene scn;
extern mjrContext con;
extern bool button_left;
extern bool button_middle;
extern bool button_right;
extern double lastx;
extern double lasty;

namespace SimulationUI {
    // === [新增] 一键初始化窗口和 MuJoCo 上下文 ===
    // 自动处理 glfwInit, Window 创建, Scene/Context 初始化, 回调绑定
    GLFWwindow* SetupWindow(mjModel* m, const char* title = "MuJoCo Simulation");

    void Init(); // 初始化 UI 变量结构体
    void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    void mouse_button(GLFWwindow* window, int button, int act, int mods);
    void mouse_move(GLFWwindow* window, double xpos, double ypos);
    void scroll(GLFWwindow* window, double xoffset, double yoffset);

    // === 视频录制器类 ===
    class VideoRecorder {
    public:
        VideoRecorder();
        ~VideoRecorder();

        bool Start(const char* filename, int width, int height, int fps);
        void RecordFrame(const mjrRect& viewport, const mjrContext* con);
        void Stop();

    private:
        FILE* ffmpeg_pipe_;
        unsigned char* image_buffer_;
        int width_;
        int height_;
    };
}