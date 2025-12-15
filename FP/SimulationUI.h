/**
 * @file SimulationUI.h
 * @brief MuJoCo simulation window setup and rendering utilities.
 *
 * Provides window initialization, input handling, video recording,
 * and text overlay rendering for the biped robot simulation.
 */

#pragma once
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <cstdio>
#include <string>

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

/**
 * @brief Initialize GLFW window and MuJoCo rendering context.
 * @param m MuJoCo model pointer.
 * @param title Window title string.
 * @return GLFW window pointer, or nullptr on failure.
 */
GLFWwindow* SetupWindow(mjModel* m, const char* title = "MuJoCo Simulation");

void Init();
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);

/**
 * @brief Video recorder using FFmpeg pipe for real-time encoding.
 */
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

/**
 * @brief Text overlay renderer for on-screen display.
 *
 * Provides simple one-line function calls to display various information
 * overlays during simulation including time, robot state, and status messages.
 */
class OverlayRenderer {
public:
    /**
     * @brief Render all text overlays in one call.
     * @param viewport Current rendering viewport.
     * @param con MuJoCo rendering context.
     * @param time Current simulation time.
     * @param trunk_x Trunk X position.
     * @param trunk_z Trunk Z position (height).
     * @param trunk_pitch Trunk pitch angle.
     * @param status_message Status message (e.g., "Walking Forward").
     * @param warning_message Warning message (displayed if not empty).
     */
    static void render(const mjrRect& viewport, const mjrContext* con,
                       double time, double trunk_x, double trunk_z, double trunk_pitch,
                       const std::string& status_message,
                       const std::string& warning_message = "");
};

/**
 * @brief Data logger for recording robot state to CSV file.
 *
 * Records 7 DOF positions, 7 DOF velocities, and 4 joint torques
 * at 100Hz (every 0.01 seconds) starting from time zero.
 */
class DataLogger {
public:
    DataLogger();
    ~DataLogger();

    /**
     * @brief Start logging to specified CSV file.
     * @param filename Output CSV file path.
     * @return true if file opened successfully.
     */
    bool start(const char* filename);

    /**
     * @brief Log current robot state if logging interval has elapsed.
     * @param d MuJoCo data pointer containing current state.
     */
    void log(const mjData* d);

    /**
     * @brief Stop logging and close file.
     */
    void stop();

private:
    FILE* log_file_;
    double last_log_time_;
    static constexpr double LOG_INTERVAL = 0.01;
};

}  // namespace SimulationUI
