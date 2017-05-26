#ifndef PID_H
#define PID_H

#include <iostream>
#include <ctime>
#include <vector>
#include <cmath>

class PID {

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;
  double dt; // time step.
  double prev_time; // Used for FindSimulatorRate

  int twiddle_state; // Used to determine if twiddle should move to new gain
  // or if current gain is still being changed.
  // 0 means new gain can be changed.
  // 1 means current gain is done gain + delta and needs to evaluate.
  // 2 means current gain is gain - delta and needs to revaluate.
  double twiddle_time; // Time for next twiddle update.
  int twiddle_gain_count; // Used to decide which gain to change.
  // Delta values for each gain.
  std::vector<double> gain_delta;
  double total_error; // Total summed error during twiddle run.
  double best_error;

  double reset_time = 0.0;




public:
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void SetGains(double Kp, double Ki, double Kd);

  // Used to find simulator running rate for dt that is needed in
  // PID calculation.
  void FindSimulatorRate();

  // Used to update the gains based on Twiddle optimization algorithm.
  void TwiddleGains();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double current_time);

  /*
  * Calculate the total PID error.
  */
  double ControlOutput();

  // Output total error.
  double GetResetTime();

  void SetResetTime(double add);
};

#endif /* PID_H */
