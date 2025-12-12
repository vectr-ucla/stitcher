#include "FeasibilityChecker.h"
#include <iostream>

template <typename MotionPrimitiveType>
bool Motion_Primitives::FeasibilityChecker::validateMaxState(const MotionPrimitiveType &primitive, 
                                                             const std::string &norm_type) {
    // Check if the maximum state of the primitive is within the allowed limits based on norms
    if (norm_type == "Linf"){
      for (double t = 0; t < primitive.horizon; t += this->sampling_rate){
        Eigen::MatrixXd state = primitive.getState(t);
        for (int i = 0; i < state.cols(); i++){
          if ((std::abs(state(2,i)) > this->params.a_max) || (std::abs(state(3,i)) > this->params.j_max) || (std::abs(state(1,i)) > this->params.v_max)){
            return false;
          }
        }
      }
    }
    else if (norm_type == "L2"){
      //if L2 norm
      for (double t = 0; t < primitive.horizon; t += this->sampling_rate){
        Eigen::MatrixXd state = primitive.getState(t);
        if (state.row(2).norm() > this->params.a_max || state.row(3).norm() > this->params.j_max || state.row(1).norm() > this->params.v_max){
          return false;
        }
      }
    }
    else {
        std::cerr << "Invalid norm type specified: " << norm_type << std::endl;
        return false;
    }

  return true;    
}

template <typename MotionPrimitiveType>
bool Motion_Primitives::FeasibilityChecker::validateMotorCommands(const MotionPrimitiveType &primitive,
                                                                  const bool &indiv_motor_forces,
                                                                  const bool &verbose) {
  //function to check if primitive exceeds max thrust, tilt, angular velocity, velocity and motor forces (optional) at any time
  for (double t = 0; t < primitive.horizon; t+=this->sampling_rate){
    Eigen::Vector3d accel = primitive.getAccel(t);
    Eigen::Vector3d accel_d = accel;
    accel_d(2) += this->params.grav;
    Eigen::Vector3d jerk = primitive.getJerk(t);

    // check max thrust
    double thrust = this->params.mass*(accel_d.norm());
    if(thrust > this->params.thrust_max){
      if(verbose){std::cout << "exceed thrust" << std::endl;}
      return false;
    }

    // check min thrust
    if(thrust < this->params.thrust_min){
      if(verbose){std::cout << "under min thrust" << std::endl;}
      return false;
    }

    // check max tilt
    double theta = std::acos((this->params.mass*accel_d(2))/thrust);
    if (theta > this->params.theta_max){
      if(verbose){std::cout << "exceed tilt, " << theta << " rad" << std::endl;}
      return false;
    }

    //check max angular velocity
    //get R for omega
    Eigen::Matrix3d R = getQuadRot(thrust, accel_d);
    Eigen::Vector3d a_hat_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    if (accel_d.norm() > 0.001){
      a_hat_dot = jerk/accel_d.norm() - accel_d*(accel_d.dot(jerk))/pow(accel_d.norm(),3); // normalize jerk
      Eigen::Vector3d vec = R.transpose()*a_hat_dot;
      omega(0) = -vec(1);
      omega(1) = vec(0);
    }

    if (omega.norm() > this->params.omega_max){
      if(verbose){std::cout << "exceed angular velocity: " << omega.transpose() << std::endl;}
      return false;
    }
    
    //check individual motor forces
    if(indiv_motor_forces){
      Eigen::Vector3d snap = primitive.getSnap(t);
      Eigen::VectorXd forces(4);
      forces = getMotorForces(accel_d, jerk, a_hat_dot, snap, omega, R, thrust);

      //if indiv exceed max
      if (forces.maxCoeff()>this->params.motor_thrust_max){
        if(verbose){std::cout << "exceed max motor forces: " << forces.transpose() << std::endl;}
        return false;
      }

      //if indiv under min
      if (forces.minCoeff()< this->params.motor_thrust_min){
        if(verbose){std::cout << "under min motor forces: " << forces.transpose() << std::endl;}
        return false;
      }
    }

    //check max vel
    if (primitive.getVel(t).norm() > this->params.v_max){
      if(verbose){
        std::cout << "exceed velocity" << std::endl;
        std::cout << "velocity norm: " << primitive.getVel(t).norm() << " xyz: " << primitive.getVel(t).transpose() << std::endl;
      }
      return false;
    }

  }
  return true;
}