#ifndef FEASIBILITY_CHECKER_H
#define FEASIBILITY_CHECKER_H

#include <Eigen/Dense>
#include "primitiveParams.h"

namespace Motion_Primitives {
    class FeasibilityChecker;
}

class Motion_Primitives::FeasibilityChecker {
public:
    /**
	* Constructor.
	*/
    FeasibilityChecker(const double sampling_rate);
     /**
	* Destructor.
	*/
    ~FeasibilityChecker();
    void setParams(const Motion_Primitives::primitiveParams &params);
    template <typename MotionPrimitiveType>
    bool validateMaxState(const MotionPrimitiveType &primitive,
                          const std::string &norm_type = "Linf");
    template <typename MotionPrimitiveType>
    bool validateMotorCommands(const MotionPrimitiveType &primitive,
                               const bool &indiv_motor_forces = false,
                               const bool &verbose = false); //default does not check individual motor forces and verbose is off
private:
    Eigen::Matrix3d getQuadRot(double thrust_des, Eigen::Vector3d accel_des);
    Eigen::VectorXd getMotorForces(const Eigen::Vector3d &a,
                                   const Eigen::Vector3d &a_dot,
                                   const Eigen::Vector3d &a_hat_dot,
                                   const Eigen::Vector3d &a_ddot,
                                   const Eigen::Vector3d &omega,
                                   const Eigen::Matrix3d &R,
                                   const double &thrust);
    double sampling_rate; // time step for checking constraints
    Motion_Primitives::primitiveParams params; // parameters for the feasibility checker
};

#include "FeasibilityChecker.tpp" //template implementation

#endif