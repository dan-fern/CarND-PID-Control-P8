# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

### Introduction
The purpose of this project was to build a PID (proportional/integral/differential) controller and tune PID hyperparameters by applying the general processing flow as described in the lessons.  A simulator is provided to test solution results.  The simulator provides cross-track error (CTE), speed, and steering angle data via uWS. The controller must respond with steering and throttle commands to drive the car reliably around the simulator track.

### Rubric Discussion Points
#### Describe the effect each of the P, I, D components had in your implementation.
The proportional (P) component had the most visible effect on the car's behavior.  The P term causes the car to steer proportional and opposite to the car's distance from the lane center (the CTE).

The differential (D) component counteracts the P term's tendency to overshoot the center line. A properly tuned D parameter will cause the car to approach the lane center smoothly.

The integral (I) component counteracts any bias in the CTE which prevents the P-D controller from reaching the lane center. This bias can arise from steering drift or sensor error.

#### Describe how the final hyperparameters were chosen.
Hyperparameters were tuned manually at first because the narrow track left little room for error.  Methods of parameter optimization (such as Twiddle) can be aided by a good initial guess.  In this case, if the guess was not good enough, it was very common for the car to leave the track and invalidate any optimization attempt.  Once I found parameters that were able to get the car around the track reliably, I then used a Twiddle optimizer to arrive at the final values of:
* Kp = 0.318568,
* Ki = 0.000, and
* Kd = 4.19512

The Ki term arrives at zero because there is no systematic bias in the system.  That is to say that the steering commands are passed to the wheels noise free.

I attempted to implement a PID controller for the throttle, to maximize the car's speed around the track but it was ineffective.  

See [`./res/pid_3.mp4`](./res/pid_3.mp4) for [a video of a full loop around the track](./res/pid_3.mp4).  

All in all, the solution was not perfect and oscillated substantially, but the car completes loops while staying on the track.  I believe one area for improvement would be modeling the rate of steering angle change.  When a steering command is issued, it takes some amount of time for the wheels to actually turn.  Modeling and incorporating that time would improve the responsiveness of a PID controller.  


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
