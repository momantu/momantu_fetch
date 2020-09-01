## Dependency

```
$sudo apt install -y ros-melodic-desktop-full python-rosdep python-rosinstall \
                     ros-melodic-apriltag-ros \
                     ros-melodic-moveit \
                     ros-melodic-moveit-visual-tools \
                     ros-melodic-fetch* \
                     ros-melodic-cartographer-ros \
                     ros-melodic-trac-ik-kinematics-plugin \
                     ros-melodic-camera-calibration \
                     ros-melodic-pointgrey-camera-description \
                     ros-melodic-flexbe-behavior-engine \
                     ros-melodic-trac-ik \
                     ros-melodic-realsense2-description \
                     ros-melodic-effort-controllers 
```

## Init submodules

`$ git submodule update --init --recursive`

## Build
```
$ catkin_make
$ source devel/setup.bash
```

## Launch
### Object 6D Pose Estimation
Please download the source from [here](https://github.com/momantu/nocs_ros), and launch it before executing the demo.([Our docker image](https://hub.docker.com/repository/docker/yz16/momantu) already contains it.)
```
$ conda activate NOCS-env
$ source ~/nocs_ws/devel/setup.bash
$ roslaunch nocs_srv pose_estimation_server.launch
```

### Gazebo simulation
```
$ roslaunch fetch_sim demo.launch
$ roslaunch fetch_sim robot_service.launch
```

### Real Fetch Robot
```
$ roslaunch fetch_real demo.launch
$ roslaunch fetch_real robot_service.launch
```


---

 Website for our tutorials:
 https://github.com/momantu
