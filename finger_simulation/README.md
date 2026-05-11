# finger_simulation package
* RDS Speedster team
* Spring 2026

## Description
This package runs a drake simulation of our robotic finger, visualized in Rviz. 

It SDF version (4bar) models the four bar linkage correctly.
The URDF version (basic) does not model the four bar linkage kinematically, rather, the DIP joint is mimicing the position of the PIP joint at a specified ratio.

## Launch files
basic_fingersim.launch.xml - launches the drake simulation in the basic_finger_sim.py file with zero torque commanded to the joints displayed in rviz.

1. `ros2 launch finger_simulation 4barsim.launch.xml` - launches dynamic simulation of finger including closed loop 4 bar
2. `ros2 launch finger_simulation basic_fingersim.launch.xml` - launches kinematic simulation of finger