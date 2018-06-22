# EUFS Autonomous Simulation

ROS/Gazebo simulation packages for driverless FSAE vehicles.

![simulation](http://eufs.co/wp-content/uploads/2018/05/eufsa-sim.jpg)

### Contents
1. [Install Prerequisites](#requirements)
2. [Install dependencies](#dependencies)
3. [Compiling and running](#compiling)
4. [Sensors](#sensors)

## Setup Instructions
### 1. Install Prerequisites <a name="requirements"></a>
##### - Install Ubuntu 16.04 LTS
##### - Install [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation)
##### - Install ROS packages:
* ros-kinetic-ackermann-msgs
* ros-kinetic-twist-mux
* ros-kinetic-joy
* ros-kinetic-controller-manager
* ros-kinetic-robotnik-msgs
* ros-kinetic-velodyne-simulator
* ros-kinetic-effort-controllers
* ros-kinetic-velocity-controllers
* ros-kinetic-joint-state-controller
* ros-kinetic-gazebo-ros-control
* ros-kinetic-robotnik-msgs
* ros-kinetic-rbcar-robot-control

__Note:__ You may need to build [rbcar_robot_control](https://github.com/RobotnikAutomation/rbcar_sim) from source.


### 2. Install project dependencies <a name="dependencies"></a>
Navigate to your catkin workspace.

If you haven't used rosdep before do: `sudo rosdep init`

To install necessary packages:  `rosdep install --from-paths src`


### 3. Compiling and running <a name="compiling"></a>
Navigate to your workspace and run `catkin_make`

This will be the command you usually use to compile.

To enable ROS to find the EUFS packages you also need to run
`source ./devel/setup.bash`

_Note:_ source needs to be run on each new terminal you open. You can also include it in your `.bashrc` file.

To run the different simulation configurations:

* `roslaunch eufsa_gazebo eufsa.launch` - complete simulation with empty world
* `roslaunch eufsa_gazebo small_track.launch` - complete simulation with a very small arteficial track. Closed loop.
* `roslaunch eufsa_gazebo big_track.launch` - complete simulation with an a big artifically made track. Closed loop.
* `roslaunch eufsa_gazebo sprint17.launch` - complete simulation of the FSUK 2017 track. Note: quite resource heavy.
* `roslaunch eufsa_gazebo acceleration.launch` - complete simulation of an acceleration event as detailed by FSG.
* `roslaunch eufsa_gazebo skidpad.launch` - complete simulation of an skidpad event as detailed by FSG.

An easy way to control the car is via

`roslaunch eufsa_description rqt_robot_steering.launch `

### 4. Additional sensors <a name="sensors"></a>
Additional sensors for testing are avilable via the `ros-kinetic-robotnik-sensor` package. Some of them are already defined in `eufsa_description/robots/eufsa.urdf.xarco`. You can simply commment them in and attach them appropriately to the car.


**Sensor suit of the car by default:**

* VLP16 lidar
* ZED Stereo camera
* IMU
* GPS
* odometry
