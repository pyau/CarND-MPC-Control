# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Result: https://youtu.be/yX21Y8NSWnA
Vehicle driving on course with a target speed of 100mph.

## MPC Controller

## Model

A simple kinematic model is used for the controller to describe the vehicle's state and behavior across time steps. The states of the model includes x, y, orientation and velocity [x,y,psi,v] , and actuators steering, throttle [delta, a] as control inputs, to change the vehicle state over time.

Kinetic model is used to predict the state on next time step by using current vehicle state and actuators:

x_1 = x + v * cos(psi) * dt
y_1 = y + v * sin(psi) * dt
psi_1 = psi + v / Lf * delta * dt
v_1 = v + a * dt

Lf is the distance between the front of vehicle and center of gravity.

Errors cross track error (CTE) and psi error (epsi) are used to build the cost function for MPC. They are updated on a new time step by these equations:

cte_1 = cte + v * sin(epsi) * dt
epsi_1 = epsi + v / Lf * delta * dt

## Timestep length and Elapsed Duration

Parameters N and dt in MPC controls the time horizon in which MPC plans the driving path over. 

For tuning dt, I do not think it makes sense to set it to value lower than 0.1s, the expected control latency.  Too large value for dt sacrifies precision.

The relationship between N and dt is Time = N * dt.  So if number of steps N is large, T becomes large, which means MPC optimizes for road that is far away.  N also directly correlates to memory and computation overhead.

I chose N as 10, and dt as 0.1 (by trial and error), so the time horizon that the MPC plans for is 1 second.  I have also increased N to 20, but I see no improvement in the behavior.  Increasing dt to 0.2 will cause the car go off track at sharp turn. 

## Polynomial fitting and MPC preprocessing

At each control loop, 6 closest waypoints closest to the car is given by the simulator.  These waypoints are fitted with a third order polynomial (main.cpp line 112). This polynomial is then used to evaluate MPC's reference line, and also current CTE (main.cpp line 114).

Simulator provides coordinates in global reference system.  In main.cpp, line 97-104, the codes convert the waypoints to vehicle coordinates.

## Model Predictive Control with Latency

In main.cpp line 121-126, the program uses the motion model equations to predict the state of vehicle in 100ms.  This updated state is then feed into the MPC solver (main.cpp line 130).  This is to compensate the fact that the vehicle would have moved on for 100ms from the time the waypoints were received before the control can be actuated.

## Cost function

MPC consists of a solver for a cost function that includes several cost items:
CTE: whether the car follows the reference line
EPSI: heading error
V: difference from target velocity
DELTA, A: minimize the use of actuators
D_DELTA, D_A: minimize the value gap between sequential actuations

In general, I find that I need to put in a lot of weight to the cost items CTE and EPSI, to make sure the car is driving in the middle of the lane.  V, DELTA and A do not need much weight.  Putting weight in V actually can cause trouble in sharp turns, as the vehicle would try to keep its speed going into a turn.  Some weight should be put on the rate of change D_DELTA, D_A because we want the vehicle to drive smoothly.

The final weights are defined in MPC.cpp lines 34-40.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
