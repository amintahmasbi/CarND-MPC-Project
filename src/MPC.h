#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
using CppAD::AD;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  static AD<double> polyPrimEval(Eigen::VectorXd coeffs, AD<double> x);
  static AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x);
};

#endif /* MPC_H */
