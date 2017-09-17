# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Goal

In this implementation, I will build a MPC steering and throttle controller for an auto-driving car. Given an optimal path to follow, the job of this controller is to drive accordingly with respect to all the system contraints: maximum steering angle, rate of change of steering, throttle rate of change, etc. Tracking curvy trajectories and dealing with these contraints are two of the prominent strengths of MPC algorithm compared to basic controllers like PID.

The second goal of this project is also program the controller to cope with input latency. For example, any steering or throttle input is delayed by 100ms before it reflects under our car system.

The third goal, to challenge myself, is achieving 100km/h car speed on straight roads. Higher speed comes with higher difficulty, especially under the input-latency condition.

## Implementation

The system in the car simulator is a kinematic model. Assume **x,y** is the coordinate from the car perspective, **psi** is the car heading angle and **v** is the speed, we have:

where Lf measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate. Note that there are two inputs for this system: steering wheel change **delta** and throttle **a**.

Mathematically, the MPC controller tries to optimize a defined performance cost, with respect to input and state constraints. The performance cost is flexible and you can add:

1. Reference state cost. For examples: tracking error shows how good the car follow the trajectory, speed reference error is needed if you want it to drive at 100km/h but allow some flexibility for the car to reduce speed at shape turns.
2. Minimize the use of actuators. You do not want the car to act with sharp turning or overspeed. 
3. Minimize the value gap between sequential actuations. It avoids abruptedly reaction in either steering or throttle.

The state/input constraints are more for physical constraints of the car systems. For instance, you restrict steering to less than 30 deg, and acceleration is within [-1, 1].

### Tuning MPC

#### Prediction Range

At sampling rate **dt**, the MPC controller tries to look at the trajectory N steps ahead to follow. A good approach to setting N, dt, and T is to first determine a reasonable range for T and then tune dt and N appropriately, keeping the effect of each in mind. 

#### Weights in Performance Cost

1. To reduce tracking error, increase weights for x, y sum-of-square errors.
2. Try adding penalty for speed * steer to avoid high speed at turns.
3. Minimize the value gap between sequential actuations by increasing weights for changes of delta and throttle.

#### System Contraints

In my solution I have relaxed the steering angle constraint from 25 to 30 deg to allow better steering performance from the car MPC controller.

### Performance

With 50 and 70 km/h the tracking is good, so I decided to give you a look at how the car at 100km/h performed :D.

[Youtube Link](https://www.youtube.com/watch?v=Fqg6Cjc1lIw)

### References

1. [Jeremy Shannon's Respiratory](https://github.com/jeremy-shannon/CarND-MPC-Project). A good start.
2. [Ksakmann's Implementation](https://github.com/ksakmann/CarND-MPC-Project/). Here I learned from him how to deal with input latency.
