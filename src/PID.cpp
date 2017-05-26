#include "PID.h"

/*
NOTES:

Manually tuned to values (0.2, 0.025, 0.01) by slowly building up P, then slowly
increasing D inordered to smooth out BANG BANG actions. Then slightly increased i
till controller felt stable.

Ran twiddle first with deltas of (0.01, 0, 0.001) and got basically no move on
the values. Final was (0.18, 0.025, 0. 009).

Found bug in code. above results no good.

Then ran twiddle with those starting values and deltas of (0.1, 0, 0.005) to
further get improved values. Did 5 "second" roughly one lap, twiddle times.

Did some trial and error of watching the first few runs to roughly tune the deltas.
Saw a lot of off the track.

Lowered the speed from 0.5 to 0.2 as it was way to inconsistent.

Major problem running twiddle is can't get an effective restart time so each iteration
takes different corners. This gives inconsistent results.

Found out how to reset based on help form the carnd.


Twiddle modified values to (.22 and .024 and 0.01).
*/


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  // Initialize all required values.
  i_error = 0;


  dt = 1.0/500; // Found refresh rate was roughly 500 hz.

  // Initialize both time doubles to zero.
  prev_time = 0.0;
  twiddle_time = 0.0;
  gain_delta = {0.015, 0.001, 0.001};
  twiddle_state = 0;
  best_error = 99999;
  total_error = 0;
}

PID::~PID() {}

void PID::SetGains(double Kp_in, double Ki_in, double Kd_in) {

  // Since all gains are negative it is easier to feed in positive numbers.
  Kp = Kp_in * -1;
  Ki = Ki_in * -1;
  Kd = Kd_in * -1;
}

void PID::FindSimulatorRate()
{
  clock_t current = clock();

  double current_time  = double(current) / CLOCKS_PER_SEC;
  cout << "Refresh Rate is " << 1.0/(current_time - prev_time) << endl;

  prev_time = current_time;
}


void PID::TwiddleGains()
{
  vector<double> gains = {Kp, Ki, Kd};

  if (best_error == 99999) {
    best_error = total_error; // Used to capture baseline run.
  }

  // Determine which gain to modify.
  int count = twiddle_gain_count % 3;

  // Only change count if state requires new gain.
  if (twiddle_state == 0) {
    gains[count] += gain_delta[count];
    twiddle_state = 1;
  } else if (twiddle_state == 1) {  // Twiddle has run, compare total error and make decision.
    if (total_error < best_error) {
      // Improvement, save new best error and increase delta.
      best_error = total_error;
      gain_delta[count] *= 1.1;
      twiddle_state = 0;
      twiddle_gain_count += 1;
    } else {
      // No improvement, move gain in other direction and try again.
      gains[count] -= gain_delta[count]*2;
      twiddle_state = 2;
    }
  } else if (twiddle_state == 2) {
    if (total_error < best_error) {
      // Improvement, save new best error and increase delta.
      best_error = total_error;
      gain_delta[count] *= 1.1;
      twiddle_state = 0;
      twiddle_gain_count += 1;
    } else {
      // No improvement after moving gain in both directions, reset and decrease
      // size of gain delta.
      gains[count] += gain_delta[count];
      gain_delta[count] *= 0.9;
      twiddle_state = 0;
      twiddle_gain_count += 1;
    }
  }

  Kp = gains[0];
  Ki = gains[1];
  Kd = gains[2];
}

void PID::UpdateError(double cte, double current_time) {
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
  int windup = 1;
  if (i_error > windup) {
    i_error = windup;
  } else if (i_error < -windup) {
    i_error = -windup;
  }


  cout << "Current time " << current_time << " Twiddle time " << twiddle_time << endl;
  double sum_gain_delta = gain_delta[0] + gain_delta[1] + gain_delta[2];
  if (sum_gain_delta > 0.001) {
    if ((current_time > twiddle_time) && (current_time > 6)) {
      TwiddleGains();
      if (twiddle_state != 0) {
        twiddle_time = current_time + 6; // Only add time if in state 1 or 2
        // Otherwise re run twiddle gains immediately.
      }
      total_error = 0;
    } else {
      total_error += abs(cte*cte); // Sum of squared error.

      cout << "Gains [" << Kp << ", " << Ki << ", " << Kd << "]" << endl;
      cout << "Gains delta [" << gain_delta[0] << ", " << gain_delta[1] << ", " << gain_delta[2] << "]" << endl;
      cout << "Best Error " << best_error << " Total Error " << total_error << endl;
      cout << "Gains sum is " << sum_gain_delta << endl;
      cout << "Twiddle State " << twiddle_state << " Count " << twiddle_gain_count%2 << endl;
    }
  }
}

double PID::ControlOutput() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::GetResetTime() {
  return reset_time;
}

void PID::SetResetTime(double add) {
  reset_time += add;
}
