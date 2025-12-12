#include "LQMTPrimitive.h"
#include <iostream>

Motion_Primitives::LQMTPrimitive::LQMTPrimitive() {
  this->x_coeff = Eigen::VectorXd::Zero(6);
  this->y_coeff = Eigen::VectorXd::Zero(6);
  this->z_coeff = Eigen::VectorXd::Zero(6);
  this->max_horizon = 5.;
  this->horizon = 2.;
  this->cost = 0;
  this->valid_state = false; // uses bernstein hull to check if traj is valid
  this->max_states << 10., 10., 60.;
}

Motion_Primitives::LQMTPrimitive::~LQMTPrimitive() {
    // destructor
}

void Motion_Primitives::LQMTPrimitive::genTraj(const Eigen::MatrixXd state,
                                                const Eigen::MatrixXd desired_state,
                                                const double rho,
                                                const int free_state) {

  this->horizon = getHorizon(state, desired_state, rho, free_state);

  this->x_coeff = genCoeff(state.col(0), desired_state.col(0), this->horizon, free_state);
  this->y_coeff = genCoeff(state.col(1), desired_state.col(1), this->horizon, free_state);
  this->z_coeff = genCoeff(state.col(2), desired_state.col(2), this->horizon, free_state);
  
  //can check feasibility with convex hulls instead of sampling -> ensure changed in LqmtStitcher if using
  //note: this was found to be less efficient and overly conservative compared to FeasibilityChecker sampling methods  
  // this->valid_state = checkFeasibilityWithConvexHull(this->horizon);
}

void Motion_Primitives::LQMTPrimitive::checkTime(double& t) const {
  if (t > this->horizon) {
    t = this->horizon;
  }
}


Eigen::MatrixXd Motion_Primitives::LQMTPrimitive::getState(double t) const {
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(4, 3);
  result.row(0) = getPos(t);
  result.row(1) = getVel(t);
  result.row(2) = getAccel(t);
  result.row(3) = getJerk(t);
  return result;
}

Eigen::Vector3d Motion_Primitives::LQMTPrimitive::getPos(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 0);
  return result;
}

Eigen::Vector3d Motion_Primitives::LQMTPrimitive::getVel(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 1);
  return result;
}

Eigen::Vector3d Motion_Primitives::LQMTPrimitive::getAccel(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 2);
  return result;
}

Eigen::Vector3d Motion_Primitives::LQMTPrimitive::getJerk(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 3);
  return result;
}

Eigen::Vector3d Motion_Primitives::LQMTPrimitive::getSnap(double t) const {
  checkTime(t);
  Eigen::Vector3d result;
  result = evalPoly(t, 4);
  return result;
}

double Motion_Primitives::LQMTPrimitive::getHorizon(const Eigen::MatrixXd &state,
                                                     const Eigen::MatrixXd &desired_state,
                                                     const double &rho,
                                                     const int &free_state) {
  double t_star = 0.0;
  std::vector<double> t_bound = {0., 5.};
  double tol = 1e-4;
  int N = 0;
  int N_max = 20;

  return costDerivativeRoots(state,desired_state,rho,free_state);
}

bool Motion_Primitives::LQMTPrimitive::checkFeasibilityWithConvexHull(const double &horizon){
  if(!validState(horizon, 2)){ //accel
    return false; 
  }
  if(!validState(horizon, 3)){ //jerk
    return false;
  }
  if(!validState(horizon, 1)){ //vel
    return false;
  }
  
  return true;
}

bool Motion_Primitives::LQMTPrimitive::validState(const double &horizon, const int deriv_order){
  //get velocity coefficients 
  Eigen::MatrixXd coeffs(3,6);
  int N = this->x_coeff.size();
  Eigen::VectorXd D = Eigen::VectorXd::Zero(N);

  switch(deriv_order){
    case 1:{
      D << 0., 1., 2., 3., 4., 5.;

      coeffs.row(0) << D.cwiseProduct(this->x_coeff).transpose();
      coeffs.row(1) << D.cwiseProduct(this->y_coeff).transpose();
      coeffs.row(2) << D.cwiseProduct(this->z_coeff).transpose();

      //shift everything up because lost a degree with derivative
      coeffs.row(0) << coeffs.row(0).tail(5), 0.;
      coeffs.row(1) << coeffs.row(1).tail(5), 0.;
      coeffs.row(2) << coeffs.row(2).tail(5), 0.;
      break;
    }
    case 2: {
      D << 0., 0., 2., 6., 12., 20.;

      coeffs.row(0) << D.cwiseProduct(this->x_coeff).transpose();
      coeffs.row(1) << D.cwiseProduct(this->y_coeff).transpose();
      coeffs.row(2) << D.cwiseProduct(this->z_coeff).transpose();

      //shift everything up because lost a degree with derivative
      coeffs.row(0) << coeffs.row(0).tail(4), 0., 0.;
      coeffs.row(1) << coeffs.row(1).tail(4), 0., 0.;
      coeffs.row(2) << coeffs.row(2).tail(4), 0., 0.;
      break;
    }
    case 3: {
      D << 0., 0., 0., 6., 24., 60.;

      coeffs.row(0) << D.cwiseProduct(this->x_coeff).transpose();
      coeffs.row(1) << D.cwiseProduct(this->y_coeff).transpose();
      coeffs.row(2) << D.cwiseProduct(this->z_coeff).transpose();

      coeffs.row(0) << coeffs.row(0).tail(3), 0., 0., 0. ;
      coeffs.row(1) << coeffs.row(1).tail(3), 0., 0., 0. ;
      coeffs.row(2) << coeffs.row(2).tail(3), 0., 0., 0. ;
      break;
    }
  }
 
  //get control points
  Eigen::MatrixXd control_points = getControlPoints(coeffs, horizon);

  //check if inf norm of state exceeds max_state 
  for (int i = 0; i < 3; i++){
    if(control_points.row(i).cwiseAbs().maxCoeff() > this->max_states(deriv_order - 1)){
      return false;
    }
  }
  return true;
}

Eigen::MatrixXd Motion_Primitives::LQMTPrimitive::getControlPoints(Eigen::MatrixXd coeffs, double horizon) {
  // using bernstein/Bezier
  int n = coeffs.cols()-1;
  Eigen::MatrixXd control_points(3,6);
  control_points = Eigen::MatrixXd::Zero(3,6);

  for (int i = 0; i < n+1; i++) {
    for (int j = 0; j < i+1; j++){
      //have to scale by horizon for generalized bernstein form
      control_points.col(i) += (binomial(i,j)*pow(horizon,j))/binomial(n,j)*coeffs.col(j);
    }
  }

  return control_points;
}

double Motion_Primitives::LQMTPrimitive::binomial(int n, int k) const{
  if (k > n){
    return 0;
  }
  if (k == 0 || k == n){
    return 1;
  }
  return factorial(n)/(factorial(k)*factorial(n-k));
}

int Motion_Primitives::LQMTPrimitive::factorial(int n) const{
  if (n == 0){
    return 1;
  }
  else {
    return n * factorial(n-1);
  }
}

double Motion_Primitives::LQMTPrimitive::costDerivativeRoots(const Eigen::MatrixXd &state,
                                                         const Eigen::MatrixXd &desired_state,
                                                         const double &rho, const int &free_state){
  // finding roots of dC/dT = 0
  // c = [c0 c1 c2 c3 c4 c5 c6] where dC/dT = c0 + c1*(1/t) + c2*(1/t)^2 + ... + c6*(1/t)^6 (laurent polynomial)                                                        

  Eigen::VectorXd c = costDerivative(state, desired_state, rho, free_state); 

  Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
  solver.compute(c); //c0+c1*x+c6*x^6 where x = 1/t
  Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType roots_x = solver.roots();

  //convert to t
  double min_cost= std::numeric_limits<double>::infinity();
  double min_cost_root = 0.0;
  Eigen::MatrixXd coeffs(3,6);
  for (int i = 0; i < roots_x.size(); i++){
    if (roots_x(i).real() < 0){
      continue; // ignore negative roots
    }

    if (roots_x(i).imag() != 0.){
      continue; // ignore imaginary roots
    }
    
    double root_t = 1/roots_x(i).real();
    coeffs.row(0) << genCoeff(state.col(0), desired_state.col(0), root_t, free_state).transpose();
    coeffs.row(1) << genCoeff(state.col(1), desired_state.col(1), root_t, free_state).transpose();
    coeffs.row(2) << genCoeff(state.col(2), desired_state.col(2), root_t, free_state).transpose();
    
    //calc cost at root_t
    double cost_at_root = calcJ(rho, root_t, coeffs);
    if (cost_at_root < min_cost){
      min_cost = cost_at_root;
      min_cost_root = root_t;
    }
  }

  if (min_cost_root == 0.0){
    std::cout << "No valid roots found for cost derivative" << std::endl;
    return this->max_horizon; //return max horizon if no valid roots found
  }

  return min_cost_root;
}

Eigen::VectorXd Motion_Primitives::LQMTPrimitive::costDerivative(const Eigen::MatrixXd &state,
                                                                 const Eigen::MatrixXd &desired_state,
                                                                 const double &rho, const int &free_state){
  //coefficients of dC/dT where C is the cost int_0^T(j^2)dt + rho*T
  //see "Search-based Motion Planning for Aggressive Flight in SE(3)" appendix for derivation

  Eigen::VectorXd c(7); 
  //values for [p0 v0 a0], [pf vf] trajectory
  if (free_state == 2){
    c(0) = rho;
    c(1) = 0;
    c(2) = -8*state.row(2).dot(state.row(2));
    c(3) = -112*state.row(2).dot(state.row(1))-48*state.row(2).dot(desired_state.row(1));
    c(4) = -240*state.row(2).dot(state.row(0)-desired_state.row(0))-384*state.row(1).dot(state.row(1))-432*state.row(1).dot(desired_state.row(1))-144*desired_state.row(1).dot(desired_state.row(1));
    c(5) = -(1600*state.row(1)+960*desired_state.row(1)).dot(state.row(0)-desired_state.row(0));
    c(6) = -1600*(state.row(0)-desired_state.row(0)).dot(state.row(0)-desired_state.row(0));
  }  
  //values for [p0 v0 a0], [pf vf af] trajectory 
  else if (free_state == -1){
    c(0) = rho;
    c(1) = 0;
    c(2) = -9*state.row(2).dot(state.row(2)) + 6*state.row(2).dot(desired_state.row(2)) - 9*desired_state.row(2).dot(desired_state.row(2));
    c(3) = -144*state.row(2).dot(state.row(1)) - 96*state.row(2).dot(desired_state.row(1)) + 96*desired_state.row(2).dot(state.row(1)) + 144*desired_state.row(2).dot(desired_state.row(1));
    c(4) = -360*(state.row(2)-desired_state.row(2)).dot(state.row(0)-desired_state.row(0)) - 576*state.row(1).dot(state.row(1)) - 1008*state.row(1).dot(desired_state.row(1)) - 576*desired_state.row(1).dot(desired_state.row(1));
    c(5) = -2880*(state.row(1)+desired_state.row(1)).dot(state.row(0)-desired_state.row(0));
    c(6) = -3600*(state.row(0)-desired_state.row(0)).dot(state.row(0)-desired_state.row(0));
  }
  
  return c;
}

Eigen::MatrixXd Motion_Primitives::LQMTPrimitive::genCoeff(const Eigen::VectorXd state,
                                                            const Eigen::VectorXd desired_state,
                                                            const double T,
                                                            const int &free_state) {
  Eigen::MatrixXd A(6,6);
  Eigen::VectorXd b(6);

  A.row(0) << 1., 0., 0., 0., 0., 0.;
  A.row(1) << 0., 1., 0., 0., 0., 0.;
  A.row(2) << 0., 0., 2., 0., 0., 0.;
  A.row(3) << 1., T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5);
  A.row(4) << 0., 1., 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4);

  if (free_state == 2){
    A.row(5) << 0., 0., 0., 1., 4*T, 10*pow(T,2);
    b << state.head(3), desired_state.head(2), 0; 
  }
  else if (free_state == -1){
    A.row(5) << 0., 0., 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
    b << state.head(3), desired_state.head(3);
  }

  Eigen::VectorXd result; 

  result = A.inverse() * b;

  return result;
}

Eigen::Vector3d
Motion_Primitives::LQMTPrimitive::evalPoly(const double t,
                                            const int derivative_order) const {
  int N = this->x_coeff.size();
  Eigen::VectorXd D = Eigen::VectorXd::Zero(N);

  switch (derivative_order) {
  case 0:
    D << 1., 1., 1., 1., 1., 1.;
    break;
  case 1:
    D << 0., 1., 2., 3., 4., 5.;
    break;
  case 2:
    D << 0., 0., 2., 6., 12., 20.;
    break;
  case 3:
    D << 0., 0., 0., 6., 24., 60.;
    break;
  case 4: 
    D << 0., 0., 0., 0., 24., 120.;
    break;
  }

  double bx = D(N - 1) * this->x_coeff(N - 1);
  double by = D(N - 1) * this->y_coeff(N - 1);
  double bz = D(N - 1) * this->z_coeff(N - 1);

  for (int i = 2; i < N + 1 - derivative_order; i++) {
    bx = D(N - i) * this->x_coeff(N - i) + bx * t;
    by = D(N - i) * this->y_coeff(N - i) + by * t;
    bz = D(N - i) * this->z_coeff(N - i) + bz * t;
  }

  Eigen::Vector3d result;
  result << bx, by, bz;

  return result;
}

Eigen::Vector3d
Motion_Primitives::LQMTPrimitive::evalPoly(const double t,
                                            const int derivative_order,
                                            const Eigen::MatrixXd coeffs) const {
  Eigen::MatrixXd x_coeff = coeffs.row(0);
  Eigen::MatrixXd y_coeff = coeffs.row(1);
  Eigen::MatrixXd z_coeff = coeffs.row(2);

  int N = this->x_coeff.size();
  Eigen::VectorXd D = Eigen::VectorXd::Zero(N);

  switch (derivative_order) {
  case 0:
    D << 1., 1., 1., 1., 1., 1.;
    break;
  case 1:
    D << 0., 1., 2., 3., 4., 5.;
    break;
  case 2:
    D << 0., 0., 2., 6., 12., 20.;
    break;
  case 3:
    D << 0., 0., 0., 6., 24., 60.;
    break;
  case 4: 
    D << 0., 0., 0., 0., 24., 120.;
    break;
  }

  double bx = D(N - 1) * x_coeff(N - 1);
  double by = D(N - 1) * y_coeff(N - 1);
  double bz = D(N - 1) * z_coeff(N - 1);

  for (int i = 2; i < N + 1 - derivative_order; i++) {
    bx = D(N - i) * x_coeff(N - i) + bx * t;
    by = D(N - i) * y_coeff(N - i) + by * t;
    bz = D(N - i) * z_coeff(N - i) + bz * t;
  }

  Eigen::Vector3d result;
  result << bx, by, bz;

  return result;
}

double Motion_Primitives::LQMTPrimitive::calcJ(double rho) const {
  //only can run this after genTraj
  double J = 0;
  double T = this->horizon;
  Eigen::MatrixXd coeffs(3,6);
  coeffs << this->x_coeff.transpose(), this->y_coeff.transpose(), this->z_coeff.transpose();

  // rhoT + j^2
  J = T*rho + 2*(18*T*coeffs.col(3).dot(coeffs.col(3)) + pow(T,3)*(96*coeffs.col(4).dot(coeffs.col(4)) + 120*coeffs.col(3).dot(coeffs.col(5))) 
      + 360*pow(T,5)*coeffs.col(5).dot(coeffs.col(5)) + 72*pow(T,2)*coeffs.col(3).dot(coeffs.col(4)) + 360*pow(T,4)*coeffs.col(4).dot(coeffs.col(5)));

  return J;
}

double Motion_Primitives::LQMTPrimitive::calcJ(double rho, 
                                               double horizon,
                                               Eigen::MatrixXd &coeffs) const {
  double J = 0;
  double T = horizon;
 
  // Trho + j^2
  J = T*rho + 2*(18*T*coeffs.col(3).dot(coeffs.col(3)) + pow(T,3)*(96*coeffs.col(4).dot(coeffs.col(4)) + 120*coeffs.col(3).dot(coeffs.col(5))) 
      + 360*pow(T,5)*coeffs.col(5).dot(coeffs.col(5)) + 72*pow(T,2)*coeffs.col(3).dot(coeffs.col(4)) + 360*pow(T,4)*coeffs.col(4).dot(coeffs.col(5)));

  return J;
}                                                  

void Motion_Primitives::LQMTPrimitive::setMaxStates(const double v_max,
                                                    const double a_max,
                                                    const double j_max) {
  this->max_states << v_max, a_max, j_max;
}

double Motion_Primitives::LQMTPrimitive::getCost() const {
  return this->cost;
}

void Motion_Primitives::LQMTPrimitive::setCost(const double cost) {
  this->cost = cost;
}

void Motion_Primitives::LQMTPrimitive::resetCoeffs() {
  this->x_coeff = Eigen::MatrixXd::Zero(2,3);
  this->y_coeff = Eigen::MatrixXd::Zero(2,3);
  this->z_coeff = Eigen::MatrixXd::Zero(2,3);
}