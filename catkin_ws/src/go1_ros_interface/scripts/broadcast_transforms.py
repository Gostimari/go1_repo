#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from unitree_legged_msgs.msg import HighState

# Create a transform broadcaster
odom_to_base_broadcaster = tf2_ros.TransformBroadcaster()

def update_odom_to_base(msg):
	transform_stamped = geometry_msgs.msg.TransformStamped()
	
	transform_stamped.header.stamp = rospy.Time.now()
	transform_stamped.header.frame_id = "odom"
	transform_stamped.child_frame_id = "base"
	
	# Set the translation from the highState message
	transform_stamped.transform.translation.x = msg.position[0]
	transform_stamped.transform.translation.y = msg.position[1]
	transform_stamped.transform.translation.z = msg.position[2]

	# Set the rotation from the quaternion coming from the IMU
	transform_stamped.transform.rotation.x = msg.imu.quaternion[1]
	transform_stamped.transform.rotation.y = msg.imu.quaternion[2]
	transform_stamped.transform.rotation.z = msg.imu.quaternion[3]
	transform_stamped.transform.rotation.w = msg.imu.quaternion[0]
	
	# Broadcast the transform
	odom_to_base_broadcaster.sendTransform(transform_stamped)

def broadcast_statics():
	static_broadcaster = tf2_ros.StaticTransformBroadcaster()

	# Define the base to camera transform
	base_to_camera = geometry_msgs.msg.TransformStamped()
	base_to_camera.header.stamp = rospy.Time.now()
	base_to_camera.header.frame_id = "base"
	base_to_camera.child_frame_id = "camera_link"
	base_to_camera.transform.translation.x = 0.16
	base_to_camera.transform.translation.y = 0
	base_to_camera.transform.translation.z = 0.06
	base_to_camera.transform.rotation.x = 0
	base_to_camera.transform.rotation.y = 0
	base_to_camera.transform.rotation.z = 0
	base_to_camera.transform.rotation.w = 1

	# Define the map to odom transform
	map_to_odom = geometry_msgs.msg.TransformStamped()
	map_to_odom.header.stamp = rospy.Time.now()
	map_to_odom.header.frame_id = "map"
	map_to_odom.child_frame_id = "odom"
	map_to_odom.transform.translation.x = 0
	map_to_odom.transform.translation.y = 0
	map_to_odom.transform.translation.z = 0
	map_to_odom.transform.rotation.x = 0
	map_to_odom.transform.rotation.y = 0
	map_to_odom.transform.rotation.z = 0
	map_to_odom.transform.rotation.w = 1

	# Broadcast the static transforms
	static_broadcaster.sendTransform(base_to_camera)

def main():
	rospy.init_node('odom_to_base_broadcaster')

	# Broadcast the static transforms
	broadcast_statics()
	
	# Subscribe to the highState topic
	# rospy.Subscriber('high_state', HighState, update_odom_to_base)
	
	# Keep the node alive
	rospy.spin()

if __name__ == '__main__':
	main()
