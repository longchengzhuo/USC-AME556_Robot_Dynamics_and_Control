#include <mujoco.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>

static inline double rad2deg(double r) {
    return r * (180.0 / 3.14159265358979323846);
}
static inline double wrap360(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w < 0.0) w += 360.0;
    return w;
}

int main() {
    const char* xmlpath = "/Users/chengzhuolong/Documents/AME556/code/HW2/2/cartpole.xml";
    double T = 2.0;  // total sim time (s)

    // Load model
    char error[1024] = {0};
    mjModel* m = mj_loadXML(xmlpath, nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Failed to load XML: " << xmlpath << "\nError: " << error << "\n";
        return 1;
    }
    mjData* d = mj_makeData(m);

    // Get ids and addresses
    int bid_pole  = mj_name2id(m, mjOBJ_BODY,  "pole");
    int jid_slide = mj_name2id(m, mjOBJ_JOINT, "slide");
    if (bid_pole < 0 || jid_slide < 0) {
        std::cerr << "Cannot find body 'pole' or joint 'slide' in model.\n";
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
    int adr_slide = m->jnt_qposadr[jid_slide];

    // Prepare output files
    std::ofstream f_angle("../angle.csv");
    std::ofstream f_pos("../position.csv");
    if (!f_angle || !f_pos) {
        std::cerr << "Failed to open output csv files.\n";
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
    f_angle << "t,angle_deg\n";
    f_pos   << "t,x\n";

    // Initial forward to fill xmat, etc.
    mj_resetData(m, d);
    mj_forward(m, d);

    const double dt = m->opt.timestep;
    const int steps = (int)std::floor(T / dt + 1e-9);

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;

        // ---- Position: cart x from qpos
        double x = d->qpos[adr_slide];

        // ---- Angle: compute from pole body's world rotation matrix
        // d->xmat is row-major 3x3 per body; index offset = 9 * bid
        const double* R = d->xmat + 9 * bid_pole;

        // Body local -Z mapped to world: u_world = R * [0,0,-1] = - (3rd column of R)
        // Third column (col=2) in row-major:
        double u_world_x = -R[0*3 + 2];
        double u_world_y = -R[1*3 + 2];
        double u_world_z = -R[2*3 + 2];
        (void)u_world_y; // not used for angle around +Y

        // Signed tilt around +Y: upright = 0, left positive, right negative
        double theta_rad  = std::atan2(-u_world_x, u_world_z);
        double theta_deg  = wrap360(rad2deg(theta_rad));  // map to [0,360)

        // Write logs
        f_angle << t << "," << theta_deg << "\n";
        f_pos   << t << "," << x << "\n";

        // Step simulation
        mj_step(m, d);
    }

    f_angle.close();
    f_pos.close();

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
