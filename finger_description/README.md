# finger_description package
* RDS Speedster team
* Spring 2026

## Description
This package provides urdf and mesh files for the speedster robotic finger.

## Launch files
finger_description.launch.xml - launches Rviz and robot state publisher using the urdf, allowing user to manipulate joints with the robot state gui.

1. `ros2 launch finger_description fingerviz.launch.xml` - launches kinematic visualization of finger in RVIZ

## Screenshot
![finger_img](img/rviz_finger.png)
