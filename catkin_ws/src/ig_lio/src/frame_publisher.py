#!/usr/bin/env python3

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import numpy as np  
from tf.transformations import quaternion_multiply, quaternion_from_euler

odom_topic = "/lio_odom"

class MapToOdomBroadcaster:
    
    def __init__(self):
        self.tf_broadcaster_map_odom = tf.TransformBroadcaster() # add the transform broadcaster
        self.initial_x = None
        self.initial_y = None
        self.initial_z = None

    def broadcast_base_to_odom(self):
        """
        Broadcasts the base to odom transform.
        """
        rate = rospy.Rate(10000)

        previous_pose = None
        current_pose = None

        while not rospy.is_shutdown():

            # Get the current robot odom from gazebo
            robot_odom = rospy.wait_for_message(odom_topic, Odometry)

            # Set the initial z-coordinate if it's the first time
            if self.initial_z is None and self.initial_x is None and self.initial_y is None:
                self.initial_x = robot_odom.pose.pose.position.x
                self.initial_y = robot_odom.pose.pose.position.y
                self.initial_z = robot_odom.pose.pose.position.z

            # roll = 0
            # pitch = 0
            # yaw = -1.1337  #-0.4226 #1.1337 
            # qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            # qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            # qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            # qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            # robot_odom.pose.pose.orientation.x = robot_odom.pose.pose.orientation.x * qx
            # robot_odom.pose.pose.orientation.y = robot_odom.pose.pose.orientation.y * qy
            # robot_odom.pose.pose.orientation.z = robot_odom.pose.pose.orientation.z * qz
            # robot_odom.pose.pose.orientation.w = robot_odom.pose.pose.orientation.w * qw

            #rotation_quaternion = quaternion_from_euler(0, 0, 0.819)
            rotation_quaternion = quaternion_from_euler(0, 0, 0)
            #rotation_quaternion = quaternion_from_euler(0, 0, 1.1337)
            odom_q = robot_odom.pose.pose.orientation
            odom_quaternion = [odom_q.x, odom_q.y, odom_q.z, odom_q.w]
            composed_q = quaternion_multiply(rotation_quaternion, odom_quaternion)

            # Broadcast the transform
            self.tf_broadcaster_map_odom.sendTransform(
                (robot_odom.pose.pose.position.x - self.initial_x, robot_odom.pose.pose.position.y - self.initial_y, robot_odom.pose.pose.position.z - self.initial_z),
                composed_q,
                rospy.Time.now(),
                "base",
                "odom"
            )

            rate.sleep()


if __name__ == '__main__':

    rospy.init_node('frame_publisher')

    map_to_odom_broadcaster = MapToOdomBroadcaster()
    map_to_odom_broadcaster.broadcast_base_to_odom()
    
    rospy.spin()