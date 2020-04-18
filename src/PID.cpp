#include "PID.h"
#include <limits>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool do_tune_) {
  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_, do_tune_) The initial PID coefficients, tuning flag
   */

  // Initialize gains
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  // Initialize errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  // Initialize tuning parameters
  do_tuning = do_tune_;

  if (do_tuning){
    it_count = 0;
    tune_interval = 250;
    best_err = std::numeric_limits<double>::max(); //Initialize to high value
  }
}

void PID::SetGains(double Kp_, double Ki_, double Kd_) {
  /**
   * Set PID gains
   * @param (Kp_, Ki_, Kd_) The PID gains
   */

  // Set gains
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

double PID::GetKp() {
  /**
   * Get the current proportional gain.
   * @output Current value for Kp
   */

  // Get Kp
  return Kp;
}

double PID::GetKi() {
  /**
   * Get the current integral gain.
   * @output Current value for Ki
   */

  // Get Ki
  return Ki;
}


double PID::GetKd() {
  /**
   * Get the current derivative gain.
   * @output Current value for Kd
   */

  // Get Kd
  return Kd;
}

void PID::UpdateError(double cte) {
  /**
   * Update the PID error variables given cross track error.
   * @param cte: The current cross track error
   */


  // NOTE: Previous cte is stored in previous p_error, and so the calculation of i_error must happen before the
  // update of p_error
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::OutputSteeringAngle() {
  /**
   * Calculate the total steering angle.
   * @output The total steering angle
   */

  double steering = -Kp*p_error - Ki*i_error -Kd*d_error;

  return steering;
}

void PID::TuneGains() {
  /**
   * Tune PID gains using Coordinate Ascent (Twiddle) method.
   */

  // TODO
}