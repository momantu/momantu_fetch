# Usage
## Navigation in Simulation
Firstly, start your simulation. For example:
```
roslaunch fetch_gazebo playground.launch
```

Start the navigation:
```
roslaunch fetch_sim fetch_nav.launch 
```

## Navigation with Real Fetch Robot
For real robot,  start the navigation with:
```
roslaunch fetch_real fetch_nav.launch # < map_file:=... >
```

# Parameter Tuning Note
_update_min_a_: Setting it smaller makes the particle filter estimation coverage faster.

_inflation_radius_: Too small: will be too close to the wall; too large: cannot cross the door.

costmap_local.yaml: _global_frame_ should be "map", but not "odom".

_yaw_goal_tolerance_ and _xy_goal_tolerance_:
Larger goal tolerance to avoid infinite rotation near the goal.

_min_in_place_vel_theta_: Too small can be difficult for the robot to reach the given angle before timeout.