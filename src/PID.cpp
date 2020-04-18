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

  s_error += cte*cte;
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

  /*
   *
   * /**
    * Twiddle - Hyperparameter Tuning
    */

//  //Get total error based on cross track error
//  total_err += cte*cte;
//
//  //Run the twiddle algorithm every 'n' evaluation steps
//  if(step_num % numSteps == 0) {
//    //if the current error is a new best, update
//    if(total_err < best_err) {
//      best_err = total_err;
//      p[p_index] *= 1.1;
//
//      //Setup for the twiddler
//      p_add = p_sub = false;
//    }
//
//    if(!p_add && !p_sub) {
//      //First iteration after start of cycle, add elements
//      Twiddler(p_index, p[p_index]);
//      p_add = true;
//    } else if(p_add && !p_sub) {
//      //Second iteration after cycle
//      //No best error found,
//      Twiddler(p_index, -2*p[p_index]);
//      p_sub = true;
//    } else {
//      //Third iteration
//      //No best error found after two attempts, time to try something new
//      Twiddler(p_index, p[p_index]);
//      p[p_index] *= 0.9;
//      p_add = p_sub = false;
//
//      //Cycle through the 3 hyperparameters
//      p_index = (p_index + 1) % 3;
//    }
//    //Reset total error at end of cycle
//    total_err = 0;
//
//    //Debugging prompts
//    std::cout << "Adjusted parameters ..." << "\n";
//    std::cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << "\n\n";
//  }
//
//  step_num++;
   *
   *
   */
}