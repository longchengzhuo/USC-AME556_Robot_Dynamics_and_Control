#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "mujoco/mujoco.h"

int main() {
    // Load the model
    mjModel* m = nullptr;
    char error[1000] = "Could not load model";
    m = mj_loadXML("/Users/chengzhuolong/Documents/AME556/code/HW3/2/robot.xml", nullptr, error, 1000);
    if (!m) {
        mju_error("Load model error: %s", error);
        return -1;
    }

    // Make data
    mjData* d = mj_makeData(m);

    // Set initial state from keyframe
    int key_id = mj_name2id(m, mjOBJ_KEY, "initial");
    if (key_id != -1) {
        mj_resetDataKeyframe(m, d, key_id);
    } else {
        std::cerr << "Could not find keyframe 'initial'" << std::endl;
    }

    // Open CSV file for writing
    std::ofstream csv_file("robot_data.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Unable to open file robot_data.csv" << std::endl;
        return -1;
    }

    // Write CSV header
    csv_file << "time,x,y,theta,q1,q2,q3,q4\n";

    // Simulation loop
    while (d->time < 2.0) {
        mj_step(m, d);
        
        // Write data to CSV
        csv_file << d->time << ","
                 << -d->qpos[0] << ","  // x is -slide_x
                 << d->qpos[1] << ","   // y is slide_z
                 << d->qpos[2] << ","   // theta is rotate_y
                 << d->qpos[3] << ","   // q1 is left_hip
                 << d->qpos[4] << ","   // q2 is left_knee
                 << d->qpos[5] << ","   // q3 is right_hip
                 << d->qpos[6] << "\n"; // q4 is right_knee
    }

    // Close the file
    csv_file.close();

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);

    std::cout << "Simulation complete. Data saved to robot_data.csv" << std::endl;

    return 0;
}