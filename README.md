# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Objective
Navigate through a highway as fast as possible. While navigating, the car should drive atleast 4.32 miles without incident, drive according to the speed limit, acceleration should be less than 10m/s^2, jerk should be less than 10m/s^3, no collisions should occur and intelligent lane changes should be done to increase efficiency.

### Overview of planner
The planner can be assumed to be a finite state machine with 5 states. Keep Lane, Prepare Lane Change Left, Prepare Lane Change Right, Lane Change Left, Lane Change Right. Although the state machine is not explicitly coded but the code was writen keeping this in mind. The switching between these states is done based on Cost functions.

### Cost Functions
There are two cost functions in the project, One is the lane-speed-cost and the other is the lane-change-safety checker 

#### Lane Speed Cost   
```
// A function that assigns cost based on the speed of the lane(fastest lane gets the slowest cost).
// The speed of a lane is defined as the speed of the car with maximum speed in that lane given that, the car in question is less than 100m ahead of us and less than 20m behind us
double lane_speed_cost(int lane, vector<Vehicle>& vehicles, double s){
    double lane_speed = 50/2.24;
    double max_speed = lane_speed;
    for(Vehicle vehicle : vehicles){
        if(vehicle.lane == lane){
            double distance = vehicle.s - s;
            if(distance < 100 && distance > -20){
                if(vehicle.speed < lane_speed){
                    lane_speed = vehicle.speed;
                }
            }

        }
    }

    return max_speed - lane_speed;
}
```

#### Lane Change Safety Cost
```
// A function to check the safety of the lane change, given the destination lane and vehicles around us.
// Here it is assumed that the car in the destination lane has to have a distance of atleast 30m from the car
bool lane_change_safe(int lane, std::vector<Vehicle>& vehicles, double s){
    for(Vehicle vehicle : vehicles){
        if(vehicle.lane == lane){
            double distance = fabs(vehicle.s - s);
            if(distance<30){
                return false;
            }
        }
    }

    return true;
}
```

### Planner Working
####Initialize in Keep Lane state
This is done in code via setting a particular lane value and not changing it. This is when the current_lane and lane are equal and the current lane has lowest speed cost.

#### Check whether the car in front of us is going too slow
This is done via predicting the future position of the car ahead using its current speed and compare it to out future waypoint. If the car is at a unsafe distance(assumed 30m or less here), then the car ahead is deemed to be running too slow, Now we assume that the car has moved to a prepare lane change state. If the car is at a safe distance, no change in state occurs.
```
bool too_close = false;
// Only change lane if the car in front of us is at a unsafe distance(is only possible if it is too slow)
for(Vehicle vehicle : vehicles){
  if(vehicle.lane == current_lane){// Predict the location of car in the current lane in the future and check if we are getting too close, if we keep proceeding as we are. A car being too close is defined as 30m away from us or less.
    double check_speed = vehicle.speed;
    double check_car_s = vehicle.s;
    check_car_s += ((double) prev_size * 0.02 * check_speed);
    if ((check_car_s > car_s) && (check_car_s - car_s <30)){
      if(vehicle.s > current_car_s)
        too_close = true;
    }
  }
}
```

#### Prepare Lane Change state
Here both the costs are computed and it is determined if a lane change is needed and if it is safe or not. This is done cia the two functions listed in cost functions. If it is not safe or not needed as current lane is still faster then the other lanes, then we stay behind the car ahead until a lane change is safe and advantageous. If it is safe and advantageous. We switch to the lane change state. This is done by setting the lane value to a new value based on the destination lane. This state is when the current_lane and lane have different values and it is unsafe to change lanes.

check if out lane is not the fastest using cost function
```
int best_lane = lane;
double current_lane_cost = lane_speed_cost(lane, vehicles, current_car_s);
for (int i=0; i<NUM_LANES; i++){
  double cost = lane_speed_cost(i, vehicles, current_car_s);
  std::cout<< "COST : " << cost<< std::endl;
  if(cost<current_lane_cost){
    best_lane = i;
    current_lane_cost = cost;
  }
}
```
If it is not the fastest, check if it is safe to change lanes and if so assign new value to lane variable
```
if(lane == current_lane){ // check if we are not in the middle of a lane change
  if (best_lane > lane){
    if(lane_change_safe(lane+1, vehicles, current_car_s)){ // if lane change is safe, execite it
      lane = lane + 1;
    }
  }else if (best_lane < lane){
    if(lane_change_safe(lane-1, vehicles, current_car_s)){
      lane = lane - 1;
    }
  }
  // if out lane is fastest, decrease velocity
  if(lane == current_lane){
    if(ref_vel >0.224)
      ref_vel -= 0.224;
  }
}
```

#### Lane change state
Here the new trajectory is computed with the help of splines and when we are in the new lane and lane change is complete, the state is assumed to go back to Keep Lane State. This state is when the current_lane and lane have different values and it is deemed safe to change the lane. Splines help generate smooth trajectory based on a few future points in the map. This helps to avoid sudden corners and jerks in the path. As spline code examples and how to use them are there in the code walthrough of the course. I have ommited code examples here.

I would like the add that the code contains just the if statements and no explicit state machine is being used. A particular configuration of parameter calues is treated as a state. But a state machine would be much more modular and cleaner if used and implemented. 

### Future Improvements
* Add a dedicated class for the Finite State Machine to make the code more readable.
* There are some cases where two simultaneous lane changes are needed to achieve optimal result. My planner does not handle this correctly everytime and sometimes stays in the same lane as before.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

