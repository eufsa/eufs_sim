# eufs_description
This package contains meshes and configurations for robots as well as tools for visualisation.

## Nodes
- `camera_overlay.py` - adds overlaying data to one of the camera images of the car. To run with camera overlay of speed and steering, change the use_overlay parameter,
 append to the above command ```use_overlay:=true```

## Launches
- `visualisator.launch` - default rviz layout.

## Notes
- Folder `robots` contains the description of our robots(cars) and their relevant sensor positions.
- Folder `sensors` contains various scripts for common sensors in robots.
