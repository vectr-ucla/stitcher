#pragma once

#include <Eigen/Dense>
#include <vector>

namespace Motion_Primitives {
class MinTimePrimitive;
}

class Motion_Primitives::MinTimePrimitive {
public:
  /**
   * Constructor
   **/
  MinTimePrimitive();

  Eigen::Vector3d getPos(double t) const;
  Eigen::Vector3d getVel(double t) const;
  Eigen::Vector3d getAccel(double t) const;
  Eigen::Vector3d getJerk(double t) const;
  Eigen::MatrixXd getState(double t) const;
  double getCost() const;
  void setCost(const double cost);
  void resetCoeffs();

  Eigen::MatrixXd x_coeff;
  Eigen::MatrixXd y_coeff;
  Eigen::MatrixXd z_coeff;

  Eigen::VectorXd x_switch;
  Eigen::VectorXd y_switch;
  Eigen::VectorXd z_switch;
  
  double max_accel;

  double horizon;

  /**
   * Destructor
   */
  virtual ~MinTimePrimitive();
  void genTraj(const Eigen::MatrixXd desired_states,
               const Eigen::MatrixXd state,
               const double max_accel,
               double max_accel_z = 0.);
  
  friend std::ostream& operator<<(std::ostream& os, const Motion_Primitives::MinTimePrimitive& primitive) {
    os << "Primitive position at start = " << primitive.getPos(0) << ", end = " << primitive.getPos(primitive.horizon);
    // Print other member variables as needed
    return os;
  }

private:
  void checkTime(double& t) const;
  Eigen::VectorXd genSwitchTimes(const Eigen::VectorXd state,
                                 const Eigen::VectorXd desired_state,
                                 const double max_accel);
  Eigen::VectorXd genConstrainedSwitchTimes(const Eigen::VectorXd state,
                                 const Eigen::VectorXd desired_state,
                                 const double initial_accel,
                                 const double T);
  void genCoeff(const Eigen::VectorXd state,
                const Eigen::VectorXd desired_state,
                const double T,
                Eigen::MatrixXd & coeffs,
                Eigen::VectorXd & t_switch,
                bool is_max_axis);
  void genConstrainedBangBangCoeff(const Eigen::VectorXd state,
                                   const Eigen::VectorXd desired_state,
                                   const double T,
                                   Eigen::MatrixXd & coeffs,
                                   Eigen::VectorXd & t_switch);
  void genMinAccelCoeff(const Eigen::VectorXd state,
                        const Eigen::VectorXd desired_state,
                        const double T,
                        Eigen::MatrixXd & coeffs,
                        Eigen::VectorXd & t_switch);
  Eigen::Vector2d getRoots(const double &a, const double &b, const double &c);
  int getControlSign(const Eigen::VectorXd &state, const Eigen::VectorXd &desired_state, const double max_accel);
  void getSwitchInterval(int &switch_id, double &t0, const double t, const Eigen::VectorXd switch_times) const;
  Eigen::Vector3d evalPoly(const double t, const int derivative_order) const;
  
  double cost; // cost to get to this primitive
  std::string off_axis_strategy;
  // int sign;
};