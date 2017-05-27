# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Submission

* Describe the effect each of the P, I, D components had in your implementation.

The P gain is multiplied by the error to give a steering angle that is proportional to the amount of error there is currently. The I gain is multiplied by error integrated over time or in other words a running sum of the error that has been seen so far. The D gain is multiplied by the current error minus the previous time steps error, divided by the time step. A rough estimate of the time step (the simulators refresh rate) was found by using the ctime library.

* Describe how the final hyperparameters were chosen.

First I determined the direction the gains would need to be in by using a large P gain and inspecting visually if the car turned towards the middle of the track or away from it. I found that with a positive P gain the car turned towards away from the middle of the track so I multiplied all gains by -1 in the initial set gains function. From now on I'll refer to all gains in positive form.

I then kept slightly increasing P gain till I felt that the car was steering towards the middle with sufficient steering angle. At this point the car was oscillating very violently so I decided to slowly increase the D gain to reduce the oscillations. Once most of the oscillations were gone I slowly increased the I gain. I also implemented a interrogator windup protection system that prevent the I term from growing to large (for example on one of the large corners where lots of error could be expected) as this is not the point of the I term.

Technically this system does not need an I gain to have zero steady state error (if an input of 0 makes the car run perfectly straight). If a the car was placed directly straight on the path and was given an input of 0, it would have zero steady state error. This is different than a system such as an altitude controller on a quadcopter. Even once a quadcopter reaches its desired height, it still requires some input to the motors to keep it there. This is the type of situation where I is needed to eliminate any steady state error. However I still implemented an I gain to account (for in this case a made up) error in the steering angle where perhaps a steering angle of 0 has the car move slightly to the left. This would be corrected by the I term over time and I felt was good practice for a real world scenario.

At this point the PID gains were all at acceptable values (0.2, 0.02, 0.015) that could operate multiple laps of the course without going off the track. However I decided to implement a twiddle algorithm to further tune the PID gains. I used deltas of (0.015, 0.001, 0.001) and had the simulator reset after a set amount of time so that each twiddle test would have identical starting conditions. I also used the error squared as the comparing factor in twiddle to more aggressively punish moving driving off the center line as just using error on its own was making it hard to differentiate between two very close tests.

I never had the final twiddle solution converge (the sum of all deltas falling below 0.001) even after running the simulator for 50 minutes, but it did roughly settle on new values (which I rounded off) of (0.25, 0.023, 0.017).

These new tuned values produced a very smooth and controlled lap.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
