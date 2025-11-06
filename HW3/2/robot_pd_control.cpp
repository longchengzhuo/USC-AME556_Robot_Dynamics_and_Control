#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// For data recording
bool is_recording = false;
double recording_start_time = 0.0;
std::ofstream csv_file;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// simulation state
int key_id = -1;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation and pause
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
    // spacebar: reset to keyframe and start simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_SPACE )
    {
        if (key_id != -1) {
            mj_resetDataKeyframe(m, d, key_id);
            if (!is_recording) {
                is_recording = true;
                recording_start_time = d->time;
                csv_file.open("robot_control_data.csv");
                if (csv_file.is_open()) {
                    std::cout << "Started recording data to robot_control_data.csv" << std::endl;
                } else {
                    std::cerr << "Unable to open robot_control_data.csv for writing" << std::endl;
                    is_recording = false;
                }
            }
        }
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


int main(int argc, char** argv) {
    // Load the model
    char error[1000] = "Could not load model";
    m = mj_loadXML("/Users/chengzhuolong/Documents/AME556/code/HW3/2/robot.xml", nullptr, error, 1000);
    if (!m) {
        mju_error("Load model error: %s", error);
        return -1;
    }

    // Make data
    d = mj_makeData(m);

    // Get keyframe id and store it
    key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    std::vector<double> qpos_initial(m->nq);
    if (key_id != -1) {
        mj_resetDataKeyframe(m, d, key_id);
        mju_copy(qpos_initial.data(), d->qpos, m->nq);
    } else {
        std::cerr << "Could not find keyframe 'initial'" << std::endl;
        return -1; // Exit if keyframe is essential and not found
    }

    // PD Controller gains
    double Kp = 50.0;
    double Kd = 2.0;

    // Get joint and actuator IDs
    int joint_ids[4];
    int actuator_ids[4];
    const char* joint_names[] = {"left_hip", "left_knee", "right_hip", "right_knee"};
    for(int i=0; i<4; i++){
        joint_ids[i] = mj_name2id(m, mjOBJ_JOINT, joint_names[i]);
        actuator_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, joint_names[i]);
    }

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "PD Control", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        
        // Step simulation and apply control if is_simulating is true
        if (d->time < 200.0) {
            mjtNum simstart = d->time;
            while( d->time - simstart < 1.0/60.0)
            {
                // Apply PD control to all 4 leg joints
                for(int i=0; i<4; i++){
                    int j_id = joint_ids[i];
                    int a_id = actuator_ids[i];

                    double pos_error = qpos_initial[j_id] - d->qpos[j_id];
                    double vel_error = 0 - d->qvel[j_id];
                    d->ctrl[a_id] = Kp * pos_error + Kd * vel_error;
                }

                mj_step(m, d);

                // Write data to CSV if recording
                if (is_recording) {
                    if (d->time - recording_start_time < 2.0) {
                        csv_file << d->time << ","
                                 << -d->qpos[0] << "," // x is -slide_x
                                 << d->qpos[1] << ","  // y is slide_z
                                 << d->qpos[2] << ","  // theta is rotate_y
                                 << d->qpos[3] << ","  // q1 is left_hip
                                 << d->qpos[4] << ","  // q2 is left_knee
                                 << d->qpos[5] << ","  // q3 is right_hip
                                 << d->qpos[6] << "\n"; // q4 is right_knee
                    } else {
                        is_recording = false;
                        if (csv_file.is_open()) {
                            csv_file.close();
                            std::cout << "Finished recording data." << std::endl;
                        }
                    }
                }
            }
        } else {
            mj_forward(m, d);
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers
        glfwSwapBuffers(window);

        // process pending GUI events
        glfwPollEvents();
    }

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwTerminate();

    return 0;
}
