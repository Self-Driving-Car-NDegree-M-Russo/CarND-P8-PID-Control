#include "PID.h"
#include <limits>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool do_tune_) {
  // Initialize coefficients
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

void PID::UpdateError(double cte) {
  // Update errors based on current and previous cross-track error
  // NOTE: Previous cte is stored in previous p_error, and so the calculation of i_error must happen before the
  // update of p_error


  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::OutputSteeringAngle() {
  /**
   * Calculate steering angle
   */

  double steering = -Kp*p_error - Ki*i_error -Kd*d_error;

  return steering;
}
