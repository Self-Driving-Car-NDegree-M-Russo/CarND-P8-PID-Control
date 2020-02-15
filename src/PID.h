#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_, do_tune_) The initial PID coefficients, tuning flag
   */
  void Init(double Kp_, double Ki_, double Kd_, bool do_tune_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total steering angle.
   * @output The total steering angle
   */
  double OutputSteeringAngle();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Parameters to be used for tuning (twiddle)
   */
  bool do_tuning;     // Flag indicating whether or not tuning is needed
  int it_count;       // Counter of iterations
  int tune_interval;  // Number of steps between every parameter's adjustment
  double best_err;    // Best error, to be update
  double p[3];        // Vector for the coefficients
  double dp[3];       // Vector for coefficient changes
  double threshold;   // Threshold for the tuning
};

#endif  // PID_H