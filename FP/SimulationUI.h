#pragma once
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <cstdio> // FILE*

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
    void Init(); // 初始化 UI 变量
    void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    void mouse_button(GLFWwindow* window, int button, int act, int mods);
    void mouse_move(GLFWwindow* window, double xpos, double ypos);
    void scroll(GLFWwindow* window, double xoffset, double yoffset);

    // === [新增] 视频录制器类 ===
    class VideoRecorder {
    public:
        VideoRecorder();
        ~VideoRecorder();

        // 开始录制
        bool Start(const char* filename, int width, int height, int fps);

        // 捕获一帧并写入管道
        void RecordFrame(const mjrRect& viewport, const mjrContext* con);

        // 停止录制
        void Stop();

    private:
        FILE* ffmpeg_pipe_;
        unsigned char* image_buffer_;
        int width_;
        int height_;
    };
}