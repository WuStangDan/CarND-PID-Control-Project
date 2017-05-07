#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::SetGains(double Kp_in, double Ki_in, double Kd_in) {

  // Since all gains are negative it is easier to feed in positive numbers.
  Kp = Kp_in * -1;
  Ki = Ki_in * -1;
  Kd = Kd_in * -1;

  i_error = 0;

  dt = 1.0/10; // Assume 10 hz
}

void PID::UpdateError(double cte) {
  d_error = (cte - p_error)/dt;
  i_error += cte*dt;
  p_error = cte;

  cout << endl << "I error is " << i_error << endl;

  // Implement to prevent integrator windup.
  // Due to system not requiring a constant control output to maintain a set
  // plant output (unlike a cruise control system which always needs throttle
  // applied for example) I term is actually not needed to have zero steady state
  // error. However if there was an error in the steering output, like for
  // example when the steering wheel is set to angle 0, the car did not go
  // perfectly straight, the I term would be needed to eliminate steady state
  // error in such a system. For that reason a very small Ki is used along with
  // a small integrator.
  int windup = 6;
  if (i_error > windup) {
    i_error = windup;
  } else if (i_error < -windup) {
    i_error = -windup;
  }

}

double PID::ControlOutput() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}
