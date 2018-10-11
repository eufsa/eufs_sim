# eufs_gazebo
This is the main simulation package which contains the simulation model of the car an several environments.

## Nodes
- `ground_truth_republisher.py` - this takes the ground truth from the simulation as input and rotates it in correct direction of the car.

## Launches
- `acceleration.launch` - Launches a simulation of the acceleration event at competition. 50m straight track.
- `big_track.launch` - Launches a simulation of a relatively big arteficially created track (100x100m).
- `empty.launch` - Launches a simulation which only has the car without anything else.
- `eufs_control.launch` - Launches the joint controller for the robot. This is also included in all other simulation launches. Without this the car can't move.
- `skidpad.launch` - Launches a simulation of the skidpad event at competition. It's quite literally a figuire of 8.
- `small_track.launch` - Launches a simulation of an arteficially created small track. Better run this if you don't have a good performing computing.
- `sprint17.launch` - Launches a simulation of the 2017 FSUK sprint event. This is the biggest simulation by far and will probably wreck your computer. Also nobody has bothered so far to colour code the cones on it.

## Notes
- If you are running a low-power computer or a VM, it's best to run only `small_track.launch` as the other sims will most likely kill your computer.
