#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "mujoco/mujoco.h"

// Function to write data to a CSV file
void write_csv(const std::string& filename, const std::vector<double>& data) {
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& value : data) {
            file << value << "\n";
        }
        file.close();
    } else {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}

int main() {
    // Load the model
    mjModel* m = nullptr;
    char error[1000] = "Could not load model";
    m = mj_loadXML("/Users/chengzhuolong/Documents/AME556/code/HW3/1/bar.xml", nullptr, error, 1000);
    if (!m) {
        mju_error("Load model error: %s", error);
        return -1;
    }

    // Make data
    mjData* d = mj_makeData(m);

    // Set initial state from keyframe
    int key_id = mj_name2id(m, mjOBJ_KEY, "init");
    if (key_id != -1) {
        mj_resetDataKeyframe(m, d, key_id);
    }

    // Data storage
    std::vector<double> theta_data;
    std::vector<double> x_data;
    std::vector<double> z_data;

    // Simulation loop
    while (d->time < 2.0) {
        mj_step(m, d);
        // qpos order is slide_x, slide_z, rotate_y
        x_data.push_back(d->qpos[0]);
        z_data.push_back(d->qpos[1]);
        theta_data.push_back(d->qpos[2]);
    }

    // Write data to files
    write_csv("theta.csv", theta_data);
    write_csv("x.csv", x_data);
    write_csv("z.csv", z_data);

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}