# robotic-finger
* RDS Speedster team
* Spring 2026

## Description
This project controls a 3DOF robotic finger designed by the speedster team for Robot Design Studio 2026 at Northwestern University as the capstone for our undergraduate degrees in Mechanical Engineering.

This repository can run a set of benchmarking demos as well as whackamole demos in both simulation and on hardware.

## Whackamole Hardware Demo
https://github.com/user-attachments/assets/4b4d7de7-5702-4842-89a0-102e31de5055

## Drake Simulation Demo
https://github.com/user-attachments/assets/2588c1cb-209b-4e95-b6a7-d67c1fb5cd49

## Installation Instructions
1.  Create a new ros2 workspace named ~/rds_ws and a src folder
        
        cd ~/
        mkdir ~/rds_ws/src -p
2. Download the `requirements.repos` file and place it in the src of your workspace. Install all required repositories using this command:

        vcs import < requirements.repos

3. Build using `colcon build`

## Drake ROS Installation
Drake must be installed on system as well as on drake_ros built locally in a overlay workspace.

1. Install drake on system [drake installation tutorial](https://drake.mit.edu/apt.html)

2. Source drake, add this to bashrc

        export PATH="/opt/drake/bin${PATH:+:${PATH}}"
        export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

3. Create overlay workspace with drakeros

        mkdir -p ~/drake_ws/src
        cd ~/drake_ws/src
        git clone https://github.com/RobotLocomotion/drake-ros.git

4. Build the drake overlay workspace. Build using gcc in ~/drake_ws.

        source /opt/ros/kilted/setup.bash
        export CC=gcc-13
        export CXX=g++-13
        colcon build --packages-select drake_ros \
        --cmake-args \
            -DCMAKE_PREFIX_PATH=/opt/drake \
            -Dpybind11_DIR=/opt/drake/lib/cmake/pybind11 \
            -DCMAKE_C_COMPILER=gcc-13 \
            -DCMAKE_CXX_COMPILER=g++-13 \
            -DBUILD_TESTING=OFF

5. Source overlay workspace, add this to bashrc

        source ~/drake_ws/install/setup.bash


## Launchfiles

### Simulation
1. `ros2 launch finger_simulation 4barsim.launch.xml` - Drake simulation with 4-bar linkage kinematics and Rviz2 visualization
2. `ros2 launch finger_simulation basic_fingersim.launch.xml` - Drake simulation with 4-bar linkage kinematics, Rviz2, and a fingertip tf frame

### Visualization
3. `ros2 launch finger_description fingerviz.launch.xml` - URDF visualization in Rviz2 with interactive joint_state_publisher_gui for manual joint control

### Control
4. `ros2 launch finger_control demo.launch.xml` - Full control stack running a selectable demo trajectory. Args:
   - `bridge:=simulation|hardware` (default: `simulation`)
   - `demo:=force_step|sinusoidal|linear|ik|cartesian_ik|chirp|chirp_velocity|none` (default: `none`)
   - `use_rviz:=true|false` (default: `true`)
   - `record:=true|false` (default: `false`)
5. `ros2 launch finger_control 3d_waypoint_tracking.launch.xml` - Full control stack with simulation vision for 3D waypoint tracking. Args:
   - `bridge:=simulation|hardware` (default: `simulation`)
   - `use_rviz:=true|false` (default: `true`)
   - `record:=true|false` (default: `false`)
6. `ros2 launch finger_control pixel_whackamole.launch.xml` - Whack-a-mole game using pixel-based vision and the whackamole server. Args:
   - `bridge:=simulation|hardware` (default: `simulation`)
   - `use_rviz:=true|false` (default: `true`)
   - `record:=true|false` (default: `false`)

### Vision
7. `ros2 launch finger_vision vision.launch.xml` - RealSense camera + AprilTag detection + finger_vision node + Rviz2
8. `ros2 launch finger_vision2 vision2.launch.xml` - RealSense camera + AprilTag detection + finger_vision2 node + Rviz2

