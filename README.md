# Project: Path Planning

### Udacity Self-Driving Car Engineer Nanodegree Program

--

The goal of this project are the following:

* Write (in C++) a path planner (including a trajectory generator) that can be used to guide the simulated vehicle in driving on a highway provided by Udacity [Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).
* Integrate the path planner into a uWebSocket-based server and verify the effectiveness of the path planning (and motion planning) algorithm.

[//]: # (Image References)
[simulator]: ./images/simulator.png
[overtaking]: ./images/overtaking.png
[one-lap]: ./images/one-lap.png

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

### Behavioural Planning

The main implementation is in the module `planner.cpp`, which defines two classes: `class Behaviour` for behaviour planning, and `class Path` for motion planning. The header is in a separate file `planner.h`.

In the header, we define a set of configuration values as constants, including the number of lanes and lane width, maximum highway speed, various distance parameters used in the algorithm, etc.

The header also defines `struct Agent`. It captures each agent vehicles captured by sensor fusion, with `lane` denoting the lane the vehicle is in, `s` the longitudinal distance, and `v` the vehicle's speed (in m/s). Note that the default values are set so that it is ahead of and faster than any other vehicle. Those values are used as sentinels in determining the lane's allowable speed when there are no agent vehicles ahead of ego in a specific lane.

The behvaiour planner uses a simple set of three distinct type of behaviour: (1) keep the current lane (`keep_lane`), (2) switch to the left lane (`lane_change_left`), and (3) switch to the right lane (`lane_change_right`). Upon initialisation, the planner sets the current behaviour to `keep_lane` and record the current lane given by the argument to the constructor. It also begins operation by copying map waypoints (read from file in `main.cpp`, passed as arguments) to member variables that will be used in determining the next behaviour.

When a request (from the simulator) arrives, the planner first recognises the current situation by copying the vehicle's pose and velocity, along with information about other vehicles obtained by sensor fusion. In order to later use this information efficiently, the planner arranges the vehicles by lane that they are driving in. The vehicles driving in lane `i` (0, 1, or 2) are recorded in the vector `agents_by_lane[i]`, which contains each vehicle's location and velocity. In determining the location, a longitudinal coordinate (in Frenet) is predicted so that it represents the state the vehicls is going to be in when the previously generated waypoints of the ego vehicle's path are all consumed (visited).

Based on the current situation, the planner is requested to determine the next behaviour to take. This job is done by the member function `determine_next()`, which does not receive any argument. The planner takes the following approach:

1. If the vehicle is blocked by another slower vehicle, it checks whether it is possible (not causing a collision) and desirable (target lane is faster) to change lanes. If those conditions are met, plan a lane change.
1. If a lane change offers chances of another lane change in the same direction for faster driving, plan a lane change (and probably another change in the near future).
1. If the vehicle is driving in the leftmost or the rightmost lane, consider switching to the center lane (based on the observation that it opens more opportunity for future lane changes).

If a lane change is planned, the planner's state machine is either in a `lane_change_left` or in a `lane_change_right` state. Once one of these states is reached, the state machine waits for the event of the vehicle entering the target lane (the `d` value is assumed to point the center of the car, with a fixed car width is also assumed). When the vehicle has entered the target lane, the state is set back to `keep_lane`.

After the next behaviour is determined, the behaviour planner provides a set of waypoints (in `get_waypoints()`) that will guide the vehicle. If the vehicle intends to keep the current lane, three waypoints are set at 40, 80, and 120 meters ahead of the last waypoint in the previously generated path, respectively, all at the center of the current lane. Otherwise (i.e., when the vehicle is changing lanes), three waypoints are set at the center of the target lane, and the distance between the three waypoints are dynamically adjusted according to the vehicle's current speed. This is done for the purpose of smoothing the lane switching maneuver by taking the vehicle's velocity into account.

The `class Behaviour` incorporates the following additional member functions used in the above explained behaviour planning:

* `check_ahead()`: Given a lane, this function checks an agent vehicle in that lane that is ahead of the ego vehicle and at the same time closest to it. The vehicle, if found, is returned in the format of `struct Agent` (as explained above). This method is used for (1) determining whether it will be safe to switch to that lane, and (2) deciding whether we are expected to drive at a faster speed by switching to that lane.
* `check_behind()`: The same check as the above function is done, but here the nearest agent vehicle behind the ego is checked. This method is used for determining whether it will be safe to switch to that lane.
* `can_change_lane()`: Using the above mentioned two methods, check whether we can safely switch to a specific lane.
* `lane_speed()`: Estimate the speed at which we are expected to drive in a specific lane. This is calculated according to the agent vehicle driving ahead of the ego, with a certain lookahead distance. If no vehicle is found in that lane (ahead of ego) or outside the lookahead distance although found, it is assumed that we can drive at the maximum speed in that lane.

### Motion Planning

A motion planner is initialized every time a trajectory is to be generated. The initialization (in the constructor for `class Path`) receives part of previously generated waypoints (not yet visited) and copies them to the path to be generated this time. Based on that part of the path, the ego vehicle's pose is estimated. Using this estimation, a spline is set up in a coordinate where the X axis is the vehicle's heading and Y axis pointing 90 degrees to the left hand side of the vehicle. This spline is set up so that it touches the following set of waypoints: (1) the last two waypoints contained in the last part of the previous path, and (2) the waypoints to guide the vehicle in future motion (generated by the behavioural planner and given as arguments). 

After the motion planner is initialized, it executes its final step in generating the trajectory for the simulated vehicle. The planner generates a set of waypoints so that the list of points submitted to the simulator always contains a fixed number of points, with 0.02 sec time gap between each pair of consecutive points.

If an agent vehicle is found in the lane that the ego is driving in, the planner generates a path that will lead the ego vehicle to follow the agent in front with a certain distance. The distance is calculated according to the current velocity of ego (to maintain a safety margin from the vehicle in front). Otherwise (i.e., if the lane is clear), the planner decides to accelerate at the maximum acceleration allowed, with a constraint that the resulting velocity does not exceed the speed limit. Note that we apply different limit values for acceleration (forward) and deceleration (backward) so that we avoid coming into contact with another vehicle even in an unlikely event of a car cutting abruptly through to our lane. This way, the planner calculates the corresponding acceleration to be used in actual trajectory generation.

The trajectory is generated using the spline set up in intialization. The planner calculates the number of waypoints to generate to maintain the predefined total number of points to submit to the simulator. Along the spline, we first calculate X coordinate value by applying the acceleration calculated above, and then Y coordinate value using the spline. Note that these values are in the ego-oriented coordinate system, and that we must transform these points back to the map coordinate system.

## Results

![The Simulator Screenshot at the Beginning of Simulation][simulator]

The above figure shows the screenshot of the simulator in action. Here we set the number of waypoints contained in the generated trajectory to be 50. This corresponds to the green points in front of the simulated vehicle, which illustrates the trajectory to follow in the coming second (0.02 sec * 50 points = 1.0 sec). Note that the speedometer indicate 49.45 mph, which is close to the speed limit we set in the code.

The vehicle follows the current lane for some time, until it is hindered by a car in front driving slower. The following figure illustrates such a situation, where the simulated ego vehicle plans and executes an overtaking maneuver by switching to the left lane.

![Overtaking Maneuver][overtaking]

The speedometer indicates 38.04 mph, which means the ego vehicle has been kept behind the white car in front driving at approximately 38 mph. The ego vehicle could not switch to the left lane earlier because of the car following closely behinc in that lane (part of it showing in the lower left corner of the screenshot). It did not choose to switch to the right lane either, because the red car in front in that lane was not any faster than the white car blocking the current lane and there was not an enough longitudinal distance between the two. Now that a sufficient gap has opened between the white car in front and the gray car behind in the left lane, the ego vehicle plans a lane switch and is moving into the left lane.

![Completing Simulated Driving for One Lap][one-lap]

The above figure shows the situation of the simulator when the ego vehicle is successfully completing the travel of one lap of the virtual highway (approximately 4.32 miles = 6945.554 meters where the Frenet `s` value wraps around).

## Discussion

We generate a trajectory containing 50 waypoints separated by 0.02 sec time gap. This corresponds to 1-second worth of distance to travel. It is expected that if we make this trajectory longer, we will have an advantage that the vehicle's motion is more predictable (planned ahead). On the other hand, if a shorter trajectory is used (for example, if we contain only 25 waypoints in each response, only half a second is planned ahead), the vehicle will presumably react in a more prompt manner to the changes in the environment, resulting in faster lane changes, etc. However, since these two aspects are in a tradeoff relationship, it is desirable to set that parameter empirically.

In addition, there are a number of parameters set as constants as explained earlier. It will be interesting to observe differences made by tuning or dynamically adjusting those parameters.
