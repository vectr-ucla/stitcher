#include "FeasibilityChecker.h"
#include <iostream>

Motion_Primitives::FeasibilityChecker::FeasibilityChecker(const double dt) {
  this->sampling_rate = dt; // time step for checking state/actuator constraints
}

Motion_Primitives::FeasibilityChecker::~FeasibilityChecker() {
  // ROS automatically deletes stuff
}

void Motion_Primitives::FeasibilityChecker::setParams(const Motion_Primitives::primitiveParams &params) {
  // Set the parameters for the feasibility checker
  this->params = params;
} 

Eigen::VectorXd Motion_Primitives::FeasibilityChecker::getMotorForces(const Eigen::Vector3d &a,
                                                                      const Eigen::Vector3d &a_dot,
                                                                      const Eigen::Vector3d &a_hat_dot,
                                                                      const Eigen::Vector3d &a_ddot,
                                                                      const Eigen::Vector3d &omega,
                                                                      const Eigen::Matrix3d &R,
                                                                      const double &thrust) {

  Eigen::Matrix3d omega_cross = Eigen::Matrix3d::Zero();
  Eigen::Vector3d omega_d_dot = Eigen::Vector3d::Zero();
  omega_cross << 0, -omega(2), omega(1),
                 omega(2), 0, -omega(0),
                 -omega(1), omega(0), 0; // angular velocity matrix
  
  if (a.norm() > 0.001){
    //calculating a_hat_ddot
    Eigen::Vector3d term1, term2, a_hat_ddot;
    term1 = a_ddot/a.norm() - a_dot*(a.dot(a_dot))/pow(a.norm(),3);
    term2 = (a*(a.dot(a_ddot)+a_dot.dot(a_dot))+a_dot*(a.dot(a_dot)))/(pow(a.norm(),3)) - 3*a*(pow(a.dot(a_dot),2))/pow(a.norm(),5);
    a_hat_ddot = term1 - term2;

    //calculate angular accel
    Eigen::Vector3d vec = R.transpose()*a_hat_ddot - omega_cross*R.transpose()*a_hat_dot; 
    omega_d_dot(0) = -vec(1); 
    omega_d_dot(1) = vec(0);
  }

  //calculate torque
  Eigen::Vector3d tau;
  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
  J(0,0) = this->params.Jxx;
  J(1,1) = this->params.Jyy;
  J(2,2) = this->params.Jzz;
  tau = J*omega_d_dot + omega_cross*J*omega; 

  //stack thrust and torque
  Eigen::VectorXd b(4);
  b << thrust, tau(0), tau(1), tau(2);
  Eigen::MatrixXd A(4,4);
  double l = this->params.arm_length;
  double c = this->params.drag;
  A << 1, 1, 1, 1,
       l, l, -l, -l,
       -l, l, l, -l,
       c, -c, c, -c;
  
  Eigen::VectorXd forces(4);
  forces = A.inverse()*b; // calculate motor commands

  return forces;
}

Eigen::Matrix3d Motion_Primitives::FeasibilityChecker::getQuadRot(double thrust_des, Eigen::Vector3d accel_des){
  Eigen::Vector3d a_d_hat = accel_des/accel_des.norm(); // normalize desired acceleration
  Eigen::Vector3d T_d_hat = Eigen::Vector3d(0, 0, thrust_des)/thrust_des; // normalize thrust vector

  // get axis of rotation
  Eigen::Vector3d n_hat = T_d_hat.cross(a_d_hat);

  if (n_hat.norm() > 0){
    n_hat = n_hat/n_hat.norm(); // normalize
  }
 
  // get angle of rotation
  double theta = std::acos(T_d_hat.dot(a_d_hat)); // angle between thrust and acceleration

  // cross product matrix of n_hat
  Eigen::Matrix3d n_hat_cross;
  n_hat_cross << 0, -n_hat(2), n_hat(1),
                 n_hat(2), 0, -n_hat(0),
                 -n_hat(1), n_hat(0), 0; 
  
  Eigen::Matrix3d R_des;
  R_des = Eigen::Matrix3d::Identity() + sin(theta)*n_hat_cross + (1-cos(theta))*n_hat_cross*n_hat_cross;

  return R_des;
}
