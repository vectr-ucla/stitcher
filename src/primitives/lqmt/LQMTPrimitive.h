#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <vector>

namespace Motion_Primitives {
class LQMTPrimitive;
}

class Motion_Primitives::LQMTPrimitive {
public:
  /**
   * Constructor
   **/
  LQMTPrimitive
();

  Eigen::Vector3d getPos(double t) const;
  Eigen::Vector3d getVel(double t) const;
  Eigen::Vector3d getAccel(double t) const;
  Eigen::Vector3d getJerk(double t) const;
  Eigen::Vector3d getSnap(double t) const;
  Eigen::MatrixXd getState(double t) const;
  double getHorizon(const Eigen::MatrixXd &state,
                    const Eigen::MatrixXd &desired_state,
                    const double &rho,
                    const int &free_state);
  double calcJ(double rho) const;
  double getCost() const;
  void setCost(const double cost);
  void resetCoeffs();
  void setMaxStates(const double v_max, const double a_max, const double j_max);

  Eigen::MatrixXd x_coeff;
  Eigen::MatrixXd y_coeff;
  Eigen::MatrixXd z_coeff;
  bool valid_state;

  double horizon;
  double max_horizon;

  /**
   * Destructor
   */
  virtual ~LQMTPrimitive();
  void genTraj(const Eigen::MatrixXd desired_states,
               const Eigen::MatrixXd state,
               const double rho,
               const int free_state = 2);
  
  friend std::ostream& operator<<(std::ostream& os, const Motion_Primitives::LQMTPrimitive& primitive) {
    os << "Primitive position at start = " << primitive.getPos(0) << ", end = " << primitive.getPos(primitive.horizon);
    return os;
  }

private:
  void checkTime(double& t) const;
  Eigen::MatrixXd genCoeff(const Eigen::VectorXd state,
                           const Eigen::VectorXd desired_state,
                           const double T,
                           const int &free_state);
  double costDerivativeRoots(const Eigen::MatrixXd &state,
                             const Eigen::MatrixXd &desired_state,
                             const double &rho,
                             const int &free_state);
  Eigen::VectorXd costDerivative(const Eigen::MatrixXd &state,
                                 const Eigen::MatrixXd &desired_state,
                                 const double &rho,
                                 const int &free_state);
  double calcJ(double rho,
               double horizon,
               Eigen::MatrixXd &coeffs) const;                           
  Eigen::Vector3d evalPoly(const double t, const int derivative_order) const;
  Eigen::Vector3d evalPoly(const double t,
                           const int derivative_order,
                           const Eigen::MatrixXd coeffs) const;

  //functions for convex hull feasibility check (found to be less efficient and overly conservative than motor command check)                       
  bool checkFeasibilityWithConvexHull(const double &horizon);
  bool validState(const double &horizon, const int deriv_order);
  Eigen::MatrixXd getControlPoints(Eigen::MatrixXd coeffs, double horizon);
  double binomial(int n, int k) const;
  int factorial(int n) const;
  
  double cost; // cost to get to this primitive
  Eigen::Vector3d max_states; // max velocity, acceleration, jerk
};