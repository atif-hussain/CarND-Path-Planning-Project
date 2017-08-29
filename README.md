# CarND-Path-Planning-Project Submission
This is my submission of Path Planning Project towards Self-Driving Car Engineer Nanodegree Program. 

## Environment
Program Development was done on a Intel Core i5 2.3GHz system with 12GB RAM, installed with Windows-10 64-bit OS. 
The code was found to be performance sensitive, the numerical values used like generating 50 length path of 0.02s will need changes if code is ran on a machine with significant performance difference. 

For easier debugging, significant effort was put trying to work with Visual Studio 2017 Community version. However i failed to setup required libraries for VS, and finally Bash on Windows10 was used with Makefiles setup as given in instructions. 

All Debugging was performed with cout statements. The cout I/O prints created significant performance lags, so there were regularly removed after use. With large debug logs, the delay got so much that the car completed 50 instructions and waited with 0 velocity for new path input; for such debugging, path length generated was increased to adjust for log size. 

## Modularized with Utility classes
Utility classes served to modularize code to help debug & identify math errors everytime they occurred. See Point.h, Frenet.h. Standard library splines.h was also used. generatePath file & function contains the actual code for Path Planning. main.cpp interacts with the simulator. Given original main.cpp was trimmed by moving code to modules. 

### Units of Measurement 
For ease, all calculations were performed in m, m/(0.02s), m/(0.02s)^2, and angles in radians. 

## Driving within lanes 
To keep the car from driving outside of the lanes, a lane is selected to drive, and a path transitioning quickly to the lane center is created. 

## Splines library 
Splines library was used to generate Jerk minimizing trajectory. 

## Jerk minimizing
To traverse the car, Jerk minimizing trajectory (6-degree splines) were used, created from 5points - current position, previous position (based on current car angle), and 3 ahead waypoints. The speed changes along this path was also controlled. This was presumed to fit within max allowed jerk of 10m/s^3. Note: this solution approach cannot be tweaked to restrict jerk to 1m/s^3. 

## Acceleration < 10m/s^2
Tangential acceleration was restricted to 50% of this limit by control the rate of change of speed. If a car in front slows suddenly, deceleration was allowed upto 75% of this limit. Use of Jerk minimizing trajectories served to control orthogonal acceleration. 

## Speed limit < 50 mph
While generating path, traversing the splines passing through anchor points, i noticed that keeping same x-increment along car, results in speed going slightly over on turns. This was initially met by reducing target Speed to 47, but it causes car to go only at 47 for most of the path which is straight. So adjustment was applied for curves as described in project walkthrough. 

## Collision
To avoid collisions, we use sensor_fusion info to detect cars around us within [-5,+20] range along the path, and get the closest in each lane bucket. 

## Lane changing 
We select available free lanes using thumbrules based on current lane. Use current lane if free, else change lane if adjacent lane is free, else reduce speed following car in front. To prevent driving outside lanes for too long, restricted lane change if current previous path already has a lane change. 

## Solution crudeness
Sometimes based on the exact movement of surrounding cars on the road, there is collision while changing lanes, or car moves too long outside lane. This is not a robust solution catering to such safety. Based on system performance, and exact position of start, such cases might occur rarelt. Even if a driving infraction happens, just leave the car running, and it'll complete multiple rounds till a full lap is without incident and best reaches 4.32 miles


# Code Model Documentation

* Point.h simple class with few utils 
* Frenet.h master utility class, contains functions for finding closest/next waypoint from XY or S, and functions for converting XY to/from Frenet; 
* splines.h borrowed standard utility 
* main.cpp wrapper code to interact with the simulator, trimmed down, moving some code parts elsewhere for modularization. 
* generatePath.h - project rubric function to generate best path for the car; contains internal help functions too (all calculations in meters, and 0.02s units). 

 * makeFrenet convert sensor_fusion surrounding car' velocities from x,y to s,d coords. 
 * goodLane find the best lane and target speed for it
   
  1. generatePath  part1 (line 66-79): update path start coordinates, either car/last_path_end; use both position & direction, to get 2 anchor points
  2. generatePath  part2 (line 81-99): Sense surrounding cars to get tgtSpeed & lane; use lane to pick 3 more anchor points on map
  3. generatePath  part3 (line 100-140): Make spline through 5 anchorPts & pick car points along this path. To eliminate singularity, these calculations are done in car_coords (ie. transform before & after). Points on Splines are got by passing x. x is kept at intervals of speed * curving factor. Speed is incremented to goto_vel=50mph or decremented to tgt_Speed=front-car-speed in increments of max_acc. 



# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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
