#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize coefficients
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // Initialize integral and derivative error
  i_error = 0.0;
  d_error = 0.0;

}

void PID::UpdateError(double cte) {
  // Update errors based on current cross-track error
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}
