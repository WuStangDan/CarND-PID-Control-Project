#ifndef PID_H
#define PID_H


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
  double prev_time;

  // Used for FindSimulatorRate


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

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double ControlOutput();
};

#endif /* PID_H */
