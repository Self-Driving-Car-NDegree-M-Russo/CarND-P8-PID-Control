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

  // Initialize errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

}

void PID::UpdateError(double cte) {
  // previous cte
  double prev_cte = p_error;

  // Update errors based on current and previous cross-track error
  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte;
}

double PID::OutputSteeringAngle() {
  /**
   * Calculate steering angle
   */

  double steering = -Kp*p_error - Ki*i_error -Kd*d_error;

  return steering;  // TODO: Add your total error calc here!
}
