# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Reflection

The approach followed to implement the path planner takes on the ideas and some of the code presented in the Q&A video.
In the following sections I will describe the implementation of this project.

The complete model for the path planner is implemented in `main.cpp`. There are two main parts in this model:

The **Behavior planner**, that takes the sensor fusion and localization information and decides, what action should the vehicle take. For example, changing lanes or setting the target velocity. This takes place between lines `409-504` in `main.cpp`.

The **Trajectory generator**, which executes the actions planned by the behavior planner, in the form of a list of waypoints representing a smooth and safe trajectory that the vehicle will follow. This takes place between lines `511-624`.

#### Behavior planning

The behavior planner has two tasks: plan the best lane to follow and the target speed.

##### Lane Changing
The basic algorithm for planning the best lane to follow is described here.
This algorithm is based on computing a score for each lane, based on different criteria. The planner will set the lane to follow as the one with the highest score, but it will forbid any lane change if it is not safe to do so.

First, lane scores are computed. Only scores for the lanes contiguous to the current one are computed, as we do not allow changing two lanes at the same time. This is done between lines `412-431`. This will select the best lane to follow in each moment.

```C++
for (int lane : allowed_lanes) {

   const double score = computeLaneScore(lane, current_lane, car_s, prev_size, sensor_fusion);
   if ((score > 0.0) && (score > max_score)) {
       max_score = score;
       best_lane = lane;
   }
 }
```

The actual score for a lane is computed in `computeLaneScore()`, defined in lines `118-172`. Given a lane to compute its score, this function also takes the location of the car and the sensor fusion data (location and speed of other vehicles located at a maximum distance of `SEARCH_DISTANCE` from the car). As a side note, the positions of the car and other vehicles are propagated forward at the end of the previous path, assuming a constant velocity (line `406` and `143`). It is also worth mentioning that care has been taken when calculating distances between cars (done in Frenet coordinates), taking into account that the circuit is circular and there is a wrapping of the s coordinate. `s_difference()` in line `99` takes care of this.

 The computation of the score weights and combines the following factors:

**Lane Velocity**, which is given by the velocity of the closest vehicle ahead in the lane whose score is being computed. Higher lane velocities imply a higher scores. This is the highest weighted factor.

**Front Gap**, which is the distance closest to vehicle ahead in the lane. Larger gap with the vehicles ahead mean a higher scores.

**Maneuverability**. The center lane scores higher because that lane provides with more lane-changing options. This is the least weighted factor.


```C++
double score = 0.71*(lane_velocity / MAX_SPEED) + 0.21*(front_gap / SEARCH_DISTANCE) + 0.08*(lane == 1);
```

`computeLaneScore()` also assigns negative scores to lanes which imply a lane change from the current one, but which is not safe as the closest vehicle ahead or behind in that lane are too close to the car (closer than `MIN_FRONT_DISTANCE` or `MIN_REAR_DISTANCE` respectively). A lane switch will never be performed to a lane with negative score.

The final decision on a lane change is made in lines `481-489`, which checks that the lane change is allowed (by checking that the lanes are contiguous with `isLaneShiftAllowed()`). It is also worth noting that it also performs the following check: `time_in_lane > TIME_IN_LANE_SHIFT_ALLOWED`, which is used to discourage rapid lane changing and jerky behavior. The variable `time_in_lane` keeps track of the time in the current lane (and it is reset with each lane change).


##### Target Speed
The behavior planner also sets the target speed which the Trajectory generator must follow, respecting the maximum allowable speed  (according to the project specification, it is 50 mph) and avoiding collisions with vehicles in front in the same lane. This is done in lines `434-476` and lines `492-504`.

The car starts at 0 km/h  and increases the target speed by 0.224 in each time step. This value was suggested in the Q&A video to stay under the maximum acceleration and to avoid jerk. Target speed increases are only performed if no other vehicles are detected ahead in the same lane closer than `FRONT_TRACKING_DISTANCE`. In any case, speed is always set to stay below `MAX_SPEED`, set at 48.5 mph.

The target speed setting part is also responsible of avoiding rear-ending other vehicles in the same lane. Through the sensor-fusion data it checks for other vehicles that drive under `FRONT_TRACKING_DISTANCE`. The car tries to follow and maintain the speed of the closest vehicle ahead under that distance, by setting `is_too_close = true` and `front_v` to the speed of that vehicle. The target speed is adjusted in steps of 0.224 mph until `front_v` is reached. If the distance to the closest vehicle ahead is less than `MIN_FRONT_DISTANCE*0.5`, the situation is considered unsafe (`is_distance_unsafe = true`) and the target speed is adjusted in larger steps of 0.5 until a safe distance is restored.

```C++
if ((distance_to_other > 0.0) &&
   (distance_to_other < FRONT_TRACKING_DISTANCE)) {

  is_too_close = true;
  // if other car is in the same lane closer than the safe //front distance, flag
  //  that it is too close in order to decelerate
  if (distance_to_other < closest_car_s) {
    // update the closest car in lane
    closest_car_s = distance_to_other;
    front_v = other_car_speed;
  }

  if (distance_to_other < MIN_FRONT_DISTANCE*0.5) {
    // distance is unsafe. This can happen when a vehicle //pulls in front of us
    is_distance_unsafe = true;
  }
}
```

```C++
// Adjust velocity
if (is_too_close) {
  if (ref_v >= front_v) { // Decrease speed until we follow the front car's speed
    ref_v -= SPEED_STEP;
  }

  if (is_distance_unsafe) {
    ref_v -= SPEED_STEP_UNSAFE; // Apply harder brakes. This can happen when a vehicle pulls in front of us
  }
}
else if (ref_v < MAX_SPEED) {
  ref_v += SPEED_STEP;

}
```

In the same fashion as in the lane change planning algorithm, the positions of the car and other vehicles are propagated forward at the end of the previous path, assuming a constant velocity (line `406` and `454`).

#### Trajectory generator

Once the lane and reference velocity to follow are planned, a jerk free smooth trajectory is generated, using a  spline interpolation library. The spline library used, was obtained from  http://kluge.in-chemnitz.de/opensource/spline, which was a really helpful resource. It is completely contained in `spline.h`.

In this section, the trajectory generation procedure is explained, which takes place between lines `511-624`.

In order to generate a smooth path with the spline interpolator, it must be fed with a few knot points and it will try to fit polynomials between these points with continuity assumptions. Five points, as explained below, are used as input to initialize spline.

* The first two points make the path tangent to the previous path's end point. If the previous path size is too small, those points are that make the path tangent to the car are the current car position and a previous point approximated with the current yaw angle. If the previous path size is enough, the tow last points of the previous path are used. (lines `520-549`).

* Three sequential points ahead of the starting reference that the car will visit every 0.02 seconds. In Frenet coordinates, these points are evenly spaced points ahead of the starting reference, at 30m, 60m, and 90m (lines `552-557`), centered in the target lane (which may or may have not changed).

The final path will always have 50 points. First, it is filled with the previous points (lines `590-593`). Then, up to the aforementioned 50 points, the path is fill up with points interpolated from the spline (lines `596-624`). The car will sequentially visit one path point every .02 seconds. Thus, the trajectory generator calculates how to break up the spline points so that the car travels at the desired reference velocity.

This path is finally sent to the simulator in lines `628-629`.


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

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
