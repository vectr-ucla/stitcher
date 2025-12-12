#ifndef PRIMITIVEPARAMS_H
#define PRIMITIVEPARAMS_H

namespace Motion_Primitives {
struct primitiveParams;
}

struct Motion_Primitives::primitiveParams {
    double map_buffer;
    bool collision_check;
    bool verbose;
    double v_max;
    double a_max;
    double az_max;
    double j_max;
    std::vector<double> map_ub;
    std::vector<double> map_lb;
    std::string geometric_planner_type;
    double rho; 
    double resolution;
    std::vector<double> vf_magnitude;
    std::vector<double> vf_zenith;
    std::vector<double> vf_azimuth;
    double max_dist_btwn_wps; // max distance between waypoints
    double thrust_max;
    double thrust_min;
    double omega_max;
    double theta_max;
    double mass;
    double grav;
    double arm_length;
    double drag;
    double motor_thrust_max;
    double motor_thrust_min;
    double Jxx;
    double Jyy;
    double Jzz;
};

#endif
