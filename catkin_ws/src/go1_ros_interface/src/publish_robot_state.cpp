// Includes required libraries and messages for ROS, networking, and thread management
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "sensor_msgs/JointState.h"
#include "unitree_legged_sdk/joystick.h"

// Definition of the HighLevelInterface class that manages the communication and state publishing
class HighLevelInterface
{
private:
    // ROS Publishers for the robot's state and joint states
    ros::Publisher pubHighState;
    ros::Publisher pubJointStates;
    
    // ROS Subscriber for command velocities
    ros::Subscriber subCmdVel;
    
    // UDP communication objects from UNITREE SDK
    UNITREE_LEGGED_SDK::UDP high_udp;
    UNITREE_LEGGED_SDK::HighCmd high_cmd;
    UNITREE_LEGGED_SDK::HighState high_state;

    // Loop functions for periodic execution of UDP send, receive, and state publishing
    UNITREE_LEGGED_SDK::LoopFunc loop_udpSend;
    UNITREE_LEGGED_SDK::LoopFunc loop_udpRecv;
    UNITREE_LEGGED_SDK::LoopFunc loop_publishHighState;
    
    // State for publishing joint data
    sensor_msgs::JointState jointStates;
    
    //
    xRockerBtnDataStruct keyData;

public:
    // Constructor that initializes communication, subscriptions, and publishers
    HighLevelInterface(ros::NodeHandle& nh, double highStatePublishRate, const char* ipAddress) 
    : high_udp(8090, ipAddress, 8082, sizeof(UNITREE_LEGGED_SDK::HighCmd), sizeof(UNITREE_LEGGED_SDK::HighState)),
      high_cmd({0}),
      high_state({0}),
      loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&HighLevelInterface::highUdpSend, this)),
      loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&HighLevelInterface::highUdpRecv, this)),
      loop_publishHighState("publish_high_state_ros", highStatePublishRate, 3, boost::bind(&HighLevelInterface::publishRobotStateROS, this)),
      pubHighState(nh.advertise<unitree_legged_msgs::HighState>("high_state", 1)),
      subCmdVel(nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&HighLevelInterface::cmdVelCallback, this, _1))),
      pubJointStates(nh.advertise<sensor_msgs::JointState>("/realRobot/joint_states", 20)),
      keyData({0})
    {
        // Initialize UDP command data and start communication loops
        high_udp.InitCmdData(high_cmd);

        // Setup joint states for publishing
        jointStates.name.resize(12);
        jointStates.position.resize(12);
        jointStates.velocity.resize(12);
        jointStates.effort.resize(12);

        // Start the UDP communication and state publishing loops
        loop_udpSend.start();
        loop_udpRecv.start();
        loop_publishHighState.start();
    }

    // Function to send commands to the robot via UDP
    void highUdpSend()
    {
        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    // Function to receive state information from the robot via UDP
    void highUdpRecv()
    {
        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }

    // Function to publish the robot's state to ROS topics
    void publishRobotStateROS()
    {
        unitree_legged_msgs::HighState high_state_ros = state2rosMsg(high_state);
        
        memcpy(&keyData, &high_state.wirelessRemote[0], 40);
        
        // std::cout << (float)keyData.lx << std::endl;
        
        // Update joint state timestamps and names
        jointStates.header.stamp = ros::Time::now();
        jointStates.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",  
                            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

        // Set joint positions, velocities, and efforts based on the received high state
        for(int i = 0; i < 12; ++i)
        {
            jointStates.position[i] = high_state_ros.motorState[i].q;
            jointStates.velocity[i] = high_state_ros.motorState[i].dq;
            jointStates.effort[i]   = high_state_ros.motorState[i].tauEst;
        }

        // Publish high state and joint states
        pubHighState.publish(high_state_ros);
        pubJointStates.publish(jointStates);
    }

    // Callback function to update high-level commands from ROS messages
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        high_cmd = rosMsg2Cmd(msg);
    }
};

// Main function setting up the ROS node and spinning to handle callbacks
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_udp");
    ros::NodeHandle private_nh("~"); // Handle for private namespace parameters
    ros::NodeHandle nh; // Handle for global namespace operations

    // Get parameters for the node operation from the parameter server
    double publish_frequency;
    std::string ipAddress;
    private_nh.param("high_state_publish_frequency", publish_frequency, 1.0);
    private_nh.param("ip_address", ipAddress, std::string("192.168.12.1"));

    // Compute the publishing rate from the frequency
    double highStatePublishRate = (publish_frequency > 0) ? 1.0 / publish_frequency : 1.0;

    // Instantiate the interface with calculated parameters
    HighLevelInterface custom(nh, highStatePublishRate, ipAddress.c_str());

    // Run the ROS event loop
    ros::spin();

    return 0;
}
