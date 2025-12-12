#include "MinTimePrimitive.h"
#include <iostream>

Motion_Primitives::MinTimePrimitive::MinTimePrimitive() {
    this->x_coeff = Eigen::MatrixXd::Zero(3,4);
    this->y_coeff = Eigen::MatrixXd::Zero(3,4);
    this->z_coeff = Eigen::MatrixXd::Zero(3,4);
    this->x_switch = Eigen::VectorXd(3);
    this->y_switch = Eigen::VectorXd(3);
    this->z_switch = Eigen::VectorXd(3);
    this->max_accel = 5.;
    this->horizon = 2.;
    this->cost = 0;
    this->off_axis_strategy = "bang_bang";
}

Motion_Primitives::MinTimePrimitive::~MinTimePrimitive() {
    // destructor
}

void Motion_Primitives::MinTimePrimitive::genTraj(const Eigen::MatrixXd state,
                                                  const Eigen::MatrixXd desired_state,
                                                  const double max_accel,
                                                  double max_accel_z) {
  this->max_accel = max_accel;
  if(max_accel_z == 0.){
    max_accel_z = max_accel;
  }

  this->x_switch = genSwitchTimes(state.col(0), desired_state.col(0), max_accel);
  this->y_switch  = genSwitchTimes(state.col(1), desired_state.col(1), max_accel);
  this->z_switch  = genSwitchTimes(state.col(2), desired_state.col(2), max_accel_z);

  std::vector<double> time_horizons {this->x_switch .sum(),this->y_switch.sum(), this->z_switch .sum()};
  int max_axis = std::max_element(time_horizons.begin(),time_horizons.end()) - time_horizons.begin();
  
  this->horizon = time_horizons[max_axis];

  genCoeff(state.col(0), desired_state.col(0), this->horizon, this->x_coeff, this->x_switch, max_axis == 0);
  genCoeff(state.col(1), desired_state.col(1), this->horizon, this->y_coeff, this->y_switch, max_axis == 1);
  genCoeff(state.col(2), desired_state.col(2), this->horizon, this->z_coeff, this->z_switch, max_axis == 2);
}

void Motion_Primitives::MinTimePrimitive::checkTime(double& t) const {
  if (t > this->horizon) {
    t = this->horizon;
  }
}


Eigen::MatrixXd Motion_Primitives::MinTimePrimitive::getState(double t) const {
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(4, 3);
  result.row(0) = getPos(t);
  result.row(1) = getVel(t);
  result.row(2) = getAccel(t);
  result.row(3) = getJerk(t);
  return result;
}

Eigen::Vector3d Motion_Primitives::MinTimePrimitive::getPos(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 0);
  return result;
}

Eigen::Vector3d Motion_Primitives::MinTimePrimitive::getVel(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 1);
  return result;
}

Eigen::Vector3d Motion_Primitives::MinTimePrimitive::getAccel(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 2);
  return result;
}

Eigen::Vector3d Motion_Primitives::MinTimePrimitive::getJerk(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 3);
  return result;
}

Eigen::VectorXd Motion_Primitives::MinTimePrimitive::genSwitchTimes(const Eigen::VectorXd state,
                                                                    const Eigen::VectorXd desired_state,
                                                                    const double max_accel) {
  //check number of switches 
  double initial_accel = getControlSign(state,desired_state,max_accel)*max_accel;
  double final_accel = -initial_accel;

  Eigen::VectorXd t = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd A(3,3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);

  for (int i  = 0; i < 2; i++){
    A.row(0) << initial_accel, 0, final_accel;
    A.row(1) << 1, 0, 0;
    A.row(2) << 0, 1, 0;

    b(0) = desired_state(1)-state(1);
   
    Eigen::Vector2d roots = getRoots(initial_accel, 
                                     2*state(1), 
                                    (pow(state(1),2)-pow(desired_state(1),2))/(2*initial_accel)+state(0)-desired_state(0));
    
    int num_pos_roots = (roots.array() >= 0).count();

    switch(num_pos_roots){
      case 0:
        t << -1, -1, -1;
        initial_accel = -initial_accel;
        final_accel = -initial_accel;
        break;
      case 1:
        b(1) = roots.maxCoeff(); // use pos root
        t = A.inverse() * b;
        return t;
      case 2:
        b(1) = roots(0);
        t = A.inverse() * b;

        if (t.minCoeff() < 0){ //if one of switch times negative, pick other root
          b(1) = roots(1);
          t = A.inverse() * b;
        }
        return t;
      default:
        std::cout << "Error, invalid number of roots" << std::endl;
        return t;
    }
  } 

  return t;
}

void Motion_Primitives::MinTimePrimitive::genCoeff(const Eigen::VectorXd state,
                                                   const Eigen::VectorXd desired_state,
                                                   const double T,
                                                   Eigen::MatrixXd & coeffs,
                                                   Eigen::VectorXd & t_switch,
                                                   bool is_max_axis) {
  if (is_max_axis){
    genConstrainedBangBangCoeff(state, desired_state, T, coeffs, t_switch);
  } 
  else if (this->off_axis_strategy == "bang_bang"){
    genConstrainedBangBangCoeff(state, desired_state, T, coeffs, t_switch);
  }                           
  else if (this->off_axis_strategy == "min_accel"){
    genMinAccelCoeff(state, desired_state, T, coeffs, t_switch);
  }
  else{
    std::cout << "Error, invalid type" << std::endl;
  }
}

void Motion_Primitives::MinTimePrimitive::genConstrainedBangBangCoeff(const Eigen::VectorXd state,
                                                                      const Eigen::VectorXd desired_state,
                                                                      const double T,
                                                                      Eigen::MatrixXd & coeffs,
                                                                      Eigen::VectorXd & t_switch) {
  // if bang bang                                                                     //  
  Eigen::Vector2d a_maxes = getRoots(pow(T,2), 
                            (4*state(0) - 4*desired_state(0) + 2*T*(state(1)+desired_state(1))), 
                            -pow((state(1)-desired_state(1)),2));

  //generate switch times for both cases and see which gives positives times
  double a_0 = a_maxes(0);
  Eigen::Vector3d t = genConstrainedSwitchTimes(state, desired_state, a_0, T);

  if (t.minCoeff() < 0){
    a_0 = a_maxes(1);
    t = genConstrainedSwitchTimes(state, desired_state, a_0, T);
  }
  // if v0 and vf = 0 the trivial solution to the quadratic ^ is zero -> choose other if gives feasible t
  else if (a_0 == 0){
    if(a_maxes(1) != 0){
      Eigen::Vector3d t2 = genConstrainedSwitchTimes(state, desired_state, a_maxes(1), T);
      if (t2.minCoeff() >= 0){
        a_0 = a_maxes(1);
        t = t2;
      }
    }
  }

  double p0, v0, p1, v1;

  v0 = state(1)+a_0*t(0);
  p0 = state(0)+state(1)*t(0)+a_0/2*pow(t(0),2);

  v1 = v0;
  p1 = p0 + v0*t(1);

  coeffs.row(0) << state(0), state(1), a_0/2, 0; //for 0 to t1
  coeffs.row(1) << p0, v0, 0, 0; //for t1 to t2
  coeffs.row(2) << p1, v1, -a_0/2, 0; //for t2 to T

  t_switch = t;
}     

void Motion_Primitives::MinTimePrimitive::genMinAccelCoeff(const Eigen::VectorXd state,
                                                                      const Eigen::VectorXd desired_state,
                                                                      const double T,
                                                                      Eigen::MatrixXd & coeffs,
                                                                      Eigen::VectorXd & t_switch) {
  Eigen::MatrixXd A(4, 4);
  Eigen::VectorXd b(4);
  A.row(0) << 1., 0., 0., 0.;
  A.row(1) << 0., 1., 0., 0.;
  A.row(2) << 1., T, pow(T, 2), pow(T, 3);
  A.row(3) << 0., 1., 2 * T, 3 * pow(T, 2);
  
  b << state.head(2), desired_state.head(2);

  Eigen::VectorXd result;
  result = A.inverse() * b;

  coeffs.row(0) = Eigen::VectorXd::Zero(4);
  coeffs.row(1) = Eigen::VectorXd::Zero(4);
  coeffs.row(2) = result;

  t_switch << 0, 0, T;
}

Eigen::VectorXd Motion_Primitives::MinTimePrimitive::genConstrainedSwitchTimes(const Eigen::VectorXd state,
                                                                    const Eigen::VectorXd desired_state,
                                                                    const double initial_accel,
                                                                    const double T) {
  Eigen::VectorXd t = Eigen::VectorXd::Zero(3);

  if (initial_accel == 0){
    t << 0, 0, T;
    return t;
  }
    
  // if bang bang 
  t(0) = (desired_state(1)-state(1))/(2*initial_accel) + T/2;
  t(1) = 0;
  t(2) = T-t(0);

  return t;
}

Eigen::Vector2d Motion_Primitives::MinTimePrimitive::getRoots(const double &a, const double &b, const double &c){
  Eigen::Vector2d roots;
  double discriminant = pow(b,2)-4*a*c;
  if (discriminant >= 0) {
    double x1 = (-b + sqrt(discriminant))/(2*a);
    double x2 = (-b - sqrt(discriminant))/(2*a);
    roots << x1, x2;
    return roots;
  }
  else{
    // std::cout<< "Error, solution was imaginary" << std::endl;
    roots << -1, -1;
    return roots;
  }
}

int Motion_Primitives::MinTimePrimitive::getControlSign(const Eigen::VectorXd &state, const Eigen::VectorXd &desired_state, const double max_accel){
    //getting the first sign of bang-bang controller 
    //equations from https://iopscience.iop.org/article/10.1088/1742-6596/783/1/012024/pdf

    int control_sign = -1;
    double switching_func = 1/(2*max_accel)*(pow(state(1),2)-pow(desired_state(1),2));

    if ((abs(state(0)-desired_state(0)) < 10e-5) && (abs(state(1)-desired_state(1)) < 10e-5)){
      control_sign = 0;
    }

    else if ((state(1) <= desired_state(1)) && (state(0) - desired_state(0) <= switching_func)){
      control_sign = 1;
    }

    else if ((state(1) > desired_state(1)) && (state(0) - desired_state(0) < -switching_func)){
      control_sign = 1;
    }

    return control_sign;
}

void Motion_Primitives::MinTimePrimitive::getSwitchInterval(int &switch_id, double &t0, const double t, const Eigen::VectorXd switch_times) const{
  Eigen::VectorXd switch_t = Eigen::VectorXd::Zero(switch_times.size());
  switch_t(0) = switch_times(0);
  for (int j = 1; j < switch_t.size(); j++){
    switch_t(j) = switch_t(j-1) + switch_times(j); 
  }

  //loop through switch times to see where to evaluate
  for (int i = 0; i < switch_t.size(); i++){
    if (switch_t(i)-t > 1e-4){
      t0 = (i > 0) ? (switch_t(i-1)) : 0.; //check if i is above 0 and if so return last switch time
      switch_id = i;
      return; 
    }
  }

  //if time is larger than last switch time
  t0 = switch_t(switch_t.size()-2);
  switch_id = switch_t.size()-1;
  return;
}

Eigen::Vector3d
Motion_Primitives::MinTimePrimitive::evalPoly(const double t,
                                              const int derivative_order) const {                                       
  int N = this->x_coeff.cols();
  
  Eigen::Vector3i coeff_id = Eigen::Vector3i::Constant(3,this->x_coeff.rows()-1);
  Eigen::Vector3d t0 = Eigen::Vector3d::Zero(3);

  getSwitchInterval(coeff_id(0), t0(0), t, this->x_switch); // find out what switch interval t lies in
  getSwitchInterval(coeff_id(1), t0(1), t, this->y_switch);
  getSwitchInterval(coeff_id(2), t0(2), t, this->z_switch);

  Eigen::VectorXd D = Eigen::VectorXd::Zero(N);

  switch (derivative_order) {
  case 0:
    D << 1., 1., 1., 1.;
    break;
  case 1:
    D << 0., 1., 2., 3.;
    break;
  case 2:
    D << 0., 0., 2., 6.;
    break;
  case 3:
    D << 0., 0., 0., 6.;
    break;
  }

  double bx = D(N - 1) * this->x_coeff(coeff_id(0), N - 1);
  double by = D(N - 1) * this->y_coeff(coeff_id(1), N - 1);
  double bz = D(N - 1) * this->z_coeff(coeff_id(2), N - 1);

  for (int i = 2; i < N + 1 - derivative_order; i++) {
    bx = D(N - i) * this->x_coeff(coeff_id(0), N - i) + bx * (t-t0(0));
    by = D(N - i) * this->y_coeff(coeff_id(1), N - i) + by * (t-t0(1));
    bz = D(N - i) * this->z_coeff(coeff_id(2), N - i) + bz * (t-t0(2));
  }

  Eigen::Vector3d result;
  result << bx, by, bz;

  return result;
}

double Motion_Primitives::MinTimePrimitive::getCost() const {
  return this->cost;
}

void Motion_Primitives::MinTimePrimitive::setCost(const double cost) {
  this->cost = cost;
}

void Motion_Primitives::MinTimePrimitive::resetCoeffs() {
  this->x_coeff = Eigen::MatrixXd::Zero(3,4);
  this->y_coeff = Eigen::MatrixXd::Zero(3,4);
  this->z_coeff = Eigen::MatrixXd::Zero(3,4);
}