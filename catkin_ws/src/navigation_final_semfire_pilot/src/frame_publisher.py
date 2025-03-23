#!/usr/bin/env python3

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

pose_topic = "/gazebo/model_states"
odom_topic = "/gazebo/odom"

# Choose to use the pose msg or odom msg from gazebo. They both should give the same result, but the pose msg is heavier.
pose = False
odom = True

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
        rate = rospy.Rate(100000)

        previous_pose = None
        current_pose = None

        while not rospy.is_shutdown():

            if pose:
                # Get the current robot pose from gazebo models
                robot_pose = rospy.wait_for_message(pose_topic, ModelStates)

                if robot_pose.pose[-1].position != previous_pose:
                    current_pose = robot_pose.pose[-1]

                    # Set the initial z-coordinate if it's the first time
                    if self.initial_z is None and self.initial_x is None and self.initial_y is None:
                        self.initial_x = current_pose.position.x
                        self.initial_y = current_pose.position.y
                        self.initial_z = current_pose.position.z


                    # Use geometry_msgs.msg.Pose
                    base_to_odom_pose = Pose()
                    base_to_odom_pose.position.x = robot_pose.pose[-1].position.x - self.initial_x
                    base_to_odom_pose.position.y = robot_pose.pose[-1].position.y - self.initial_y
                    base_to_odom_pose.position.z = robot_pose.pose[-1].position.z - self.initial_z
                    base_to_odom_pose.orientation.x = robot_pose.pose[-1].orientation.x
                    base_to_odom_pose.orientation.y = robot_pose.pose[-1].orientation.y
                    base_to_odom_pose.orientation.z = robot_pose.pose[-1].orientation.z
                    base_to_odom_pose.orientation.w = robot_pose.pose[-1].orientation.w

                    # Broadcast the transform
                    self.tf_broadcaster_map_odom.sendTransform(
                        (base_to_odom_pose.position.x, base_to_odom_pose.position.y, base_to_odom_pose.position.z),
                        (base_to_odom_pose.orientation.x, base_to_odom_pose.orientation.y, base_to_odom_pose.orientation.z, base_to_odom_pose.orientation.w),
                        rospy.Time.now(),
                        "base",
                        "odom"
                    )
                    previous_pose = current_pose
                else:
                    print(f"frame not updated {current_pose.position.x} {current_pose.position.y} {current_pose.position.z}")

            if odom:
                # Get the current robot odom from gazebo
                robot_odom = rospy.wait_for_message(odom_topic, Odometry)

                # Set the initial z-coordinate if it's the first time
                if self.initial_z is None and self.initial_x is None and self.initial_y is None:
                    self.initial_x = robot_odom.pose.pose.position.x
                    self.initial_y = robot_odom.pose.pose.position.y
                    self.initial_z = robot_odom.pose.pose.position.z

                # Broadcast the transform
                self.tf_broadcaster_map_odom.sendTransform(
                    (robot_odom.pose.pose.position.x - self.initial_x, robot_odom.pose.pose.position.y - self.initial_y, robot_odom.pose.pose.position.z - self.initial_z),
                    (robot_odom.pose.pose.orientation.x, robot_odom.pose.pose.orientation.y, robot_odom.pose.pose.orientation.z, robot_odom.pose.pose.orientation.w),
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