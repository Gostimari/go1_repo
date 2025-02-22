# Go1 ROS Interface

This package creates the bridge between the [Unitree Go1 EDU](https://shop.unitree.com/products/unitreeyushutechnologydog-artificial-intelligence-companion-bionic-companion-intelligent-robot-go1-quadruped-robot-dog?variant=42363559674089) internal UDP communication protocol and [ROS](https://www.ros.org/).

## Dependencies

All major dependencies are listed in the package.xml file. They are:

- [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk/)
- [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real/tree/master/unitree_legged_msgs)

## Objective of the Package

The `robot.launch` starts the ROS master and includes the `robotTF.launch` file that publishes the robot's ROS TF tree (depending on the boolean value of the parameter `publish_tf_tree`). It also launches the `publish_robot_state.cpp` node, which is responsible for:

- Connecting to the robot with the chosen method (`wireless` or `ethernet`, configurable with the parameter `connection_type`) and create the bridge between UDP and ROS, namely:
    - Getting the `highState` structure from the SDK via UDP, convert it to a [custom ROS message](https://github.com/unitreerobotics/unitree_ros_to_real/blob/master/unitree_legged_msgs/msg/HighState.msg) and publish it with a given frequency;
    - Subscribing to the `/cmd_vel` topic, converting it into the respective [data structure from the SDK](https://github.com/unitreerobotics/unitree_legged_sdk/blob/go1/include/unitree_legged_sdk/comm.h#L162) and send that information to the robot via UDP.


**Any further behavior that the user considers missing is a welcome addition as a pull request.**

## Usage

1. Connect to the Go1 via Ethernet (192.168.123.161) or Wi-Fi (192.168.12.1);
2. Run `roslaunch go1_ros_interface robot.launch`;
    - The default connectivity method is Wi-Fi. If you want to run via ethernet, change the `connection_type` parameter to "ethernet";
    - The default behavior will publish the TF tree of robot. If you don't want it to be published, change the `publish_tf_tree` parameter to "false".

At this point, the feedback from the robot's sensors should be published in ROS under the `/high_state` topic, the robot model and TF tree should also be published, and any messages published in the `/cmd_vel` topic should effectively make the robot move.