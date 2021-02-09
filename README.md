# Project: Path Planning

### Udacity Self-Driving Car Engineer Nanodegree Program

--

The goal of this project are the following:

* Write (in C++) a path planner (including a trajectory generator) that can be used to guide the simulated vehicle in driving on a highway provided by Udacity [Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).
* Integrate the path planner into a uWebSocket-based server and verify the effectiveness of the path planning (and motion planning) algorithm.

[//]: # (Image References)
[simulator]: ./images/simulator.png
[overtaking]: ./images/overtaking.png

## Rubric Points

* The ego vehicle (simulated car) should try to drive as close as possible to the 50 MPH speed limit, which means it should try to pass slower traffic when possible.
* The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times unless switching from one lane to another.
* The car should not experience total acceleration of over 10 m/s^2 and jerk of over 10 m/s^3.

The complete rubric points for this project can be found [here](https://review.udacity.com/#!/rubrics/1971/view).

## Program Build & Execution

In the project root directory, execute (in a shell) the following sequence of commands.

```
# mkdir build
# cd build
# cmake .. && make
# ./path_planning
```

Besides, run the above mentioned simulator on the same machine, which connects and sends requests at port 4567 of the `localhost`. The program `path_planning` should listen to this port, connect, and responds to the simulator's requests by sending two json-encoded lists of waypoints: `"next_x"` and `"next_y"`, which correspond to the X & Y coordinates of the waypoints in the global coordinate system.

## Input Data

#### The map of the virtual highway

The map is provided by a data file [data/highway_map.csv](./data/highway_map.csv), which contains a list of `[x, y, s, dx, dy]` values. The `x` and `y` are the waypoint's map coordinate values (center of the road), the `s` value the distance along the road to get to thay waypoint (in meters), the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop. The highway's waypoints wrap around so that the `s` value ranges from 0.0 to 6945.554.

#### The ego vehicle's localization

Each of the request from the simulator contains the following set of values indicating the ego vehicle's localization information:

* `x`: X value of the vehicle's location in the map coordinates
* `y`: Y value of the vehicle's location in the map coordinates
* `s`: The vehicle's longitudinal distance along the highway in the Frenet coordinates
* `d`: The vehicle's lateral distance from the center of the highway in the Frenet coordinates
* `yaw`: The vehicle's heading (represented in degrees)
* `speed` The vehicle's driving speed (represented in MPH)

Besides, the simulator provides (json-encoded in the request) the following information regarding the previously planned motion:

* `previous_path_x`: X values of the waypoints in the previously generated path, which have not yet been visited
* `previus_path_y`: Y values of the waypoints in the previously generated path, which have not yet been visited
* `end_path_s`: The Frenet `s` value of the last waypoint in the previously generated path
* `end_path_d`: The Frenet `d` value of the last waypoint in the previously generated path

#### The other vehicle's localization

The `sensor_fusion` field included in the request (from the simulator) contains other (visible) vehicles' localization information assumed to be obtained from sensor data and their fusion. The field consists of a list of agent (vehicle) data, which in turn consists of a list containing:

* `[0]`: the agent's identifier
* `[1]`: X coordinate value of the agent (in the map coordniate system)
* `[2]`: Y coordinate value of the agent (in the map coordniate system)
* `[3]`: the agent's velocity in the X direction (in the map coordinate system)
* `[4]`: the agent's velocity in the Y direction (in the map coordinate system)
* `[5]`: the agent's `s` value of the Frenet coordinate system
* `[6]`: the agent's `d` value of the Frenet coordinate system

They should be used to determine which behaviour or which trajectory to take in planning the ego vehicle's motion.

## Path Planning Algorithm

The path planning algorithm executes in two distinct steps: (1) behaviour planning to determine which behaviour (keep the current lane, switch to the left lane, etc.) the vehicle should take, and (2) motion planning to generate a feasible (and preferably smooth) trajectory that is followed by the vehicle.

The behaviour planner is given all the information about the ego vehicle and its surrounding environment, as well as a target speed that the ego vehicle intends to achieve, every time the (server) program receives a request from the (client) simulator. It determines the next behaviour that the vehicle should take, based on its current status and traffic condition. The palnner is requested to generate a set of waypoints that will be used to guide the motion planner to generate a trajectory.

The motion planner is then given a set of waypoints (part of the previously generated trajectory that the vehicle is yet to follow) along with the vehicle's pose (X, Y, and yaw). It is then requested to generate a new trajectory (by appending waypoints to the previously generated path so that the total waypoint contained in it is a vector of a certain predefined length), so that the vehicle is driven along the waypoints given by the behaviour planner. The motion planner is also responsible to determine the speed at which the vehicle can be driven without causing any trouble (coming into contact with other vehicles, violating acceleration and jerk constraints, and so on).

Finally, the generated path (consisting of a certain number of waypoints to be visited) is given as a response to the simulator's request. The simulator controls the vehicle so that it travels along the generated path by visiting the sequence of waypoints provided by the planner. At the next request period, any waypoints that are not yet visited are conveyed back to the server to assist future trajectory generation.

## Implementation

The main implementation is in the module `planner.cpp`, which defines two classes: `class Behaviour` for behaviour planning, and `class Path` for motion planning. The header is in a separate file `planner.h`.

In the header, we define a set of configuration values with constants, including the number of lanes and lane width, maximum highway speed, various distance parameters used in the algorithm, etc.

The header also defines `struct Agent`. It captures each agent vehicles captured by sensor fusion, with `lane` denoting the lane the vehicle is in, `s` the longitudinal distance, and `v` the vehicle's speed (in m/s). Note that the default values are set so that it is ahead of and faster than any other vehicle. Those values are used as sentinels in determining the lane's allowable speed when there are no agent vehicles ahead of ego in a specific lane.

The behvaiour planner uses a simple set of three distinct type of behaviour: (1) keep the current lane (`keep_lane`), (2) switch to the left lane (`lane_change_left`), and (3) switch to the right lane (`lane_change_right`). 

## Results

























## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
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

