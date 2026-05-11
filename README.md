# drake-finger-sim
* RDS Speedster team
* Spring 2026

## Description
This project controls a 3DOF robotic finger designed by the speedster team for Robot Design Studio 2026 at Northwestern University as the capstone for our undergraduate degrees in Mechanical Engineering.

## Video
https://github.com/user-attachments/assets/51fc3995-e88c-482a-a0bd-d43471b91480

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
1. `ros2 finger_control hardware_control.launch.xml` - Launches the finger control with hardware bridge
2. `ros2 finger_control simulation_control.launch.xml` - Launches the finger control with simulation bridge
3. `ros2 finger_vision vision.launch.xml` - Launches the computer vision algorithm
4. `ros2 launch finger_simulation 4barsim.launch.xml` - Launches dynamic simulation of finger including closed loop 4 bar
5. `ros2 launch finger_simulation basic_fingersim.launch.xml` - Launches kinematic simulation of finger
6. `ros2 launch finger_description fingerviz.launch.xml` - Launches kinematic visualization of finger in RVIZ

## AI usage
AI was used to get the 'basic_' version up and runnning, provided with examples from the official drake_ros github repo.
