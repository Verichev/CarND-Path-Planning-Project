# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

![Intro](https://github.com/Verichev/CarND-Path-Planning-Project/blob/master/doc/main.png)

### Intoduction
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car's velocity is as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car tries to avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make one complete loop around the 6946m highway.
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Rubic Points

### Compilation

#### The code compiles correctly.

I used the additional library in this project:

- [spline](http://kluge.in-chemnitz.de/opensource/spline/) *(Cubic Spline interpolation implementation)*

### Valid Trajectories

#### The car is able to drive at least 14 miles without incident.

The car was able to drive 14 miles without incidents:

![14miles](https://github.com/Verichev/CarND-Path-Planning-Project/blob/master/doc/distance_14miles.png)


#### The car drives according to the speed limit.

The car was able to drive 14 miles without a red speed limit message.


#### Max Acceleration and Jerk are not Exceeded.

The car was able to drive 14 miles without red max jerk message.

#### Car does not have collisions.

The car was able to drive 14 miles without a collision.

#### The car stays in its lane, except for the time between changing lanes.

The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

#### The car is able to change lanes

The car change lanes when the there is a slow car in front of it, and it is safe to change lanes (no other cars around).


### Reflection

The basic idea are taken for Q&A video + used some tweeking for more smooth prediction and movement.  The functionality is separated into 3 main parts: Prediction, Behaviour Planning and Trajectory Calculation.


#### 1. Prediction

This part is at the lines 89 and 163.

It uses telemetry data for sensors to figure out the future positions of cars and calculate the cost functions for different states like Change Left Lane or Change Right Lane or Keep Lane.

#### 2. Behaviour Planning

This part is at the lines 165 and 194.

Depending on cost values of different state we choose the best future state for planner and set the velocity and target line if needed.

The basic algorith tries to keep the lane and speed up as long as possible, but if it meets the car ahead, it tries to change the lane if the speed at other lanes more then at the current. Avoiding collision is the essencial part of behaviour planning.


#### 3. Trajectory Calculation

Lines 196 - 254
We use the previous path and building new path on it, using spline library for generation smooth passing between paths and for smooth changing line if nessessary. While generating spline we are using the frame of reference of the last point of the previous path to assure the point are for the spline are correcly arranged and to simplify spline calculations. Then we calculate the path points considering the speed step that was defind by behaviour planner. At the end we return back to global frame of reference and pass data back to socket.


