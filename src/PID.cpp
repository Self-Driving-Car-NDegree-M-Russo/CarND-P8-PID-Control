#include "PID.h"
#include <limits>
#include <iostream>
#include <math.h>
#include <jmorecfg.h>

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

    // Initialize counters
    it_count = 0;         // Iteration counter
    tune_interval = 250;  // Number of iterations between 2 consecutive tuning adjustments
    ad_count = 0;         // Adjustment counter
    max_adjust = 5;       // Max number of adjustments

    // Initialize vectors
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;

    dp[0] = Kp*0.1;       // Delta vector initialized to 10% of gains
    dp[1] = Ki*0.1;
    dp[2] = Kd*0.1;

    p_it = 0;             // Iterator over p, dp vectors

    p_plus = true;
    p_minus = true;

    // Initialize error and tolerance
    best_err = std::numeric_limits<double>::max();  //Initialize to high value
    threshold = 0.001;
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

bool PID::GetTuneFlag() {
  /**
   * Get the current tuning flagn.
   * @output Current value for tuning flag
   */

  // Get tuning flag
  return do_tuning;
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

  s_error = sqrt(pow(cte,2));
}

double PID::OutputSteeringAngle() {
  /**
   * Calculate the total steering angle.
   * @output The total steering angle
   */

  double steering = -Kp * p_error - Ki * i_error -Kd * d_error;

  return steering;
}

void PID::TuneGains() {
  /**
   * Tune PID gains using Coordinate Ascent (Twiddle) method.
   */

  // TODO
  // Count iterations
  it_count += 1;
  std::cout<<"Iteration : " << it_count << std::endl;

  // Run tuning algorithm every tune_interval steps, and no more than max_adjust times
  if (it_count % tune_interval == 0){
    if (ad_count < max_adjust){
      ad_count += 1;
      std::cout << "ACTIVATE TUNING SEQUENCE FOR " << ad_count << " TIME" << std::endl;

      while((dp[0] + dp [1] + dp[2]) < threshold) {

        if (s_error < best_err) {
          best_err = s_error;
          dp[p_it] *= 1.1;
          p_plus = false;
          p_minus = false;
        }

        if (!p_plus && !p_minus) {
          // Case number one: best error found, no operations executed. Increment p[p_it] by dp[p_it]
          p[p_it] += dp[p_it];
          // TODO: Update actual gains based on p vector - use SetGains
          p_plus = true;
        } else if (p_plus && !p_minus) {
          // Case number two: increment p executed but NO best error found. Decrement p[it] by 2*dp[p_it]
          p[p_it] -= dp[p_it];
          // TODO: Update actual gains based on p vector - use SetGains
          p_minus = true;
        } else {
          // Case number three: increment and decrement executed, but NO best error found. Reduce increment interval and
          // restart
          p[p_it] += dp[p_it];
          // TODO: Update actual gains based on p vector - use SetGains
          p_plus = false;
          p_minus = false;

          // Increment p_it, looping over [0,1,2]
          p_it = (p_it + 1) % 3;
        }
        
      }
    }
  }
}