# robot_control
This is the package that simulates the vehicle dynamics and odometry of the car in simulation

## Nodes
* `robot_control_node` - node that controls the car in simulation and outputs odometry. For this to work you must first load up all of the joints of the simulation model, otherwise this will brake. Odometry works in a similar way as in the real world ie. accounts for slip. Parameters are:
    - `joint_states` - string -  which topic to listen to receive the state of the joints.
    - `model` - string - name of the robot model. Uses this to control the joints.
    - `publish_odom_tf` - bool - wheather to publish a transform for odometry.
    - `wheelbase` - float - distance from front axle to back axle.
    - `wheel_diameter` - float - guess
    - `max_speed` - float - Maximum speed limit. Sometimes if you make it go too fast, you can break the sim.
    - `max_steering` - float - The maximum steering the car can do.
* `twist_to_ackermannDrive.py` - converts `Twist` messages to `AckermannDrive` messages to allow the car to be controlled with generic controllers.

## Launches
* `robot_control.launch` - launches the `robot_control_node` fully parametarised for our 2017 car SISU 2.
* `rqt_robot_control.launch` - launches a GUI to control the car WITH SLIDERS!!!

## Notes
- DO NOT RUN THIS IN THE REAL WORLD
