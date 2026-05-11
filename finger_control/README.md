# finger_control package
* RDS Speedster team
* Spring 2026

## Description
This packages serves as the high level controller controlling where the finger goes.

## Launch files
1. `hardware_control.launch.xml` - Launch the finger control with hardware bridge
2. `simulation_control.launch.xml` - Launch the finger control with simulation bridge

## Helpful commands
1. To cancel a repeating action from the command line use the following service call:

    ros2 service call /<action_name>/_action/cancel_goal action_msgs/srv/CancelGoal

