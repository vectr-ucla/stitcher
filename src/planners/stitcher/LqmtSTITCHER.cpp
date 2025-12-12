#include <LqmtSTITCHER.h>

Motion_Primitives::LqmtSTITCHER::LqmtSTITCHER() {
  this->verbose = false;
};

Motion_Primitives::LqmtSTITCHER::~LqmtSTITCHER() {};

bool Motion_Primitives::LqmtSTITCHER::generatePrimitive(Motion_Primitives::LQMTPrimitive &primitive, 
                                                               const Eigen::MatrixXd &current_state, 
                                                               const Eigen::MatrixXd &des_state){   

  if ((des_state.row(0) - this->goal).norm() < 1e-3){
    //ensure end state has fixed zero accel
    Eigen::MatrixXd des_state_end_rest(3,3);
    des_state_end_rest.row(0) = des_state.row(0);
    des_state_end_rest.row(1) = des_state.row(1);
    des_state_end_rest.row(2) = Eigen::VectorXd::Zero(3);
    primitive.genTraj(current_state, des_state_end_rest, this->params.rho, -1); 
  }
  else{
    primitive.genTraj(current_state, des_state, this->params.rho, 2); //free accel state
  }
  
  //if want to use Linf constraints on max states instead of motor commands, uncomment
  //auto begin = std::chrono::high_resolution_clock::now();
  // if (!validateMaxState(primitive)){
  //   this->feasibility_check_time += std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - begin).count();
  //   return false;
  // }
  
  auto begin = std::chrono::high_resolution_clock::now();
  if (!validateMotorCommands(primitive)){
    this->feasibility_check_time += std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - begin).count();
    return false;
  }

  primitive.setCost(primitive.calcJ(this->params.rho)); //lqmt cost
  //primitive.setCost(this->params.rho*primitive.horizon); // time edge cost

  return true;
}

