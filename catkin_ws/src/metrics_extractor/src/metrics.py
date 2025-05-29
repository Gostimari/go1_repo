#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf import TransformListener
import csv
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from datetime import datetime
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import perf_counter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from metrics_extractor.msg import MotorState
from unitree_legged_msgs.msg import HighState
import os

class MetricsExtractor:

	def __init__(self):
		"""
		Function Description 
		"""
		# Temporary/auxiliary variables:
		self.odom_msg = Odometry()
		self.nav_goal_msg = MoveBaseActionGoal()
		self.bobcat_map_listener = TransformListener()
		self.vel = Twist()
		self.imu = Imu()
		self.robot_navigating = False
		self.once = False
		self.prev_trav_dist = 0
		self.prev_cumulative_height_gain = 0
		self.prev_cumulative_height_loss = 0
		self.prev_x = 0
		self.prev_y = 0
		self.prev_z = 0
		self.elapsed_time_secs = 0 # Total time elapsed moving from start->goal
		self.elapsed_time_nsecs = 0
		self.orientation = []
		self.RPY = []
		self.tic = []
		self.timeout = 350.0
		self.trigger_end = False
		self.filename_raw = datetime.now().strftime('logfile_raw-%Y-%m-%d-%H-%M')
		self.filename_metrics = datetime.now().strftime('logfile_metrics-%Y-%m-%d-%H-%M')
		self.start_time = None
		self.end_time = None
		self.goal_reached = False
		self.failed_goals_count = 0
		self.goal_counter = 0
		# Metrics:
		self.travelled_distance = 0 # Total distance travelled by the robot
		self.total_velocity = 0 # Total velocity accumulated by the robot
		self.velocity_count = 0
		self.mean_velocity = 0 # Mean velocity of the robot
		self.imu_derivation_x = 0
		self.imu_derivation_y = 0
		self.imu_derivation_z = 0
		self.instalibity_index = 0
		self.imu_append_x = []
		self.imu_append_y = []
		self.imu_append_z = []
		self.mean_torque = 0
		self.fl_calf_append = 0
		self.fl_hip_append = 0
		self.fl_thigh_append = 0
		self.fr_calf_append = 0
		self.fr_hip_append = 0
		self.fr_thigh_append = 0
		self.rl_calf_append = 0
		self.rl_hip_append = 0
		self.rl_thigh_append = 0
		self.rr_calf_append = 0
		self.rr_hip_append = 0
		self.rr_thigh_append = 0
		self.mean_foot_force = 0
		self.foot_force1_append = 0
		self.foot_force2_append = 0
		self.foot_force3_append = 0
		self.foot_force4_append = 0
		self.power_consumption = 0
		self.voltage_consumption = 0
		self.mean_power = 0
		self.mean_voltage = 0
		self.goal_counter = 0
		self.last_status = 0
  
		# Get the current directory and navigate to "src/metrics_extractor"
		self.current_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
		self.logfiles_path = os.path.join(self.current_path, "logfiles")

		# Create directory if it doesn't exist
		os.makedirs(self.logfiles_path, exist_ok=True)
		
		self.file_raw = open(os.path.join(self.logfiles_path, f'{self.filename_raw}.csv'), mode='a')
		self.file_metrics = open(os.path.join(self.logfiles_path, f'{self.filename_metrics}.csv'), mode='a')

	def read_odom_and_goal(self):
		"""
		This function is responsible for receiving the navigation goal at the
		beginning of the path and periodically read the robot's
		odometry information. 
		"""
		self.odom_msg = rospy.wait_for_message("lio_odom", Odometry)
		self.status_msg = rospy.wait_for_message("move_base/status", GoalStatusArray)
		self.vel_msg = rospy.wait_for_message("cmd_vel", Twist)
		#self.imu_msg = rospy.wait_for_message("camera/imu", Imu)
		self.imu_msg = rospy.wait_for_message("imu/data", Imu)
  
		self.high_state_msg = rospy.wait_for_message("high_state", HighState)
  
		
		if self.once == False:
			#self.nav_goal_msg = rospy.wait_for_message("move_base/goal", MoveBaseActionGoal)
			self.tic = perf_counter()

	def metrics_calculator(self):
		"""
		This function is responsible for using the odometry information to
		calculate the required metrics.
		"""
		##### Section 1 - Travelled distance calculation #####
		current_x = self.odom_msg.pose.pose.position.x
		current_y = self.odom_msg.pose.pose.position.y
		current_z = self.odom_msg.pose.pose.position.z


		if self.once == False: # The sole purpose of this block is to initialize the three variables
			self.prev_x = current_x
			self.prev_y = current_y
			self.prev_z = current_z
			self.imu_append_x = np.array([self.imu_msg.linear_acceleration.x])
			self.imu_append_y = np.array([self.imu_msg.linear_acceleration.y])
			self.imu_append_z = np.array([self.imu_msg.linear_acceleration.z])
			# self.fl_calf_append = np.array([self.high_state_msg.motorState[0].tauEst])
			# self.fl_hip_append = np.array([self.high_state_msg.motorState[1].tauEst])
			# self.fl_thigh_append = np.array([self.high_state_msg.motorState[2].tauEst])
			# self.fr_calf_append = np.array([self.high_state_msg.motorState[3].tauEst])
			# self.fr_hip_append = np.array([self.high_state_msg.motorState[4].tauEst])
			# self.fr_thigh_append = np.array([self.high_state_msg.motorState[5].tauEst])
			# self.rl_calf_append = np.array([self.high_state_msg.motorState[6].tauEst])
			# self.rl_hip_append = np.array([self.high_state_msg.motorState[7].tauEst])
			# self.rl_thigh_append = np.array([self.high_state_msg.motorState[8].tauEst])
			# self.rr_calf_append = np.array([self.high_state_msg.motorState[9].tauEst])
			# self.rr_hip_append = np.array([self.high_state_msg.motorState[10].tauEst])
			# self.rr_thigh_append = np.array([self.high_state_msg.motorState[11].tauEst])
			# self.power_consumption = np.array([self.high_state_msg.bms.current])
			# self.voltage_consumption = np.array([self.high_state_msg.bms.cell_vol[0]])
			self.once = True

		self.travelled_distance = self.prev_trav_dist + np.sqrt(np.square(current_x - self.prev_x) + np.square(current_y - self.prev_y) + np.square(current_z - self.prev_z))

		print(f"Travelled distance = {self.travelled_distance:.3f}")

		self.prev_x = current_x
		self.prev_y = current_y
		self.prev_trav_dist = self.travelled_distance
		##### End of section 1 #####

		##### Section 2 - Velocity Calculation #####
		self.total_velocity = self.vel_msg.linear.x
		#self.velocity_count += 1
		#self.mean_velocity = self.total_velocity / self.velocity_count
		self.mean_velocity = self.total_velocity
		##### End of section 2 #####
  
		print(f"Velocity = {self.mean_velocity:.3f}")
  
		##### Section 3 - Instability Index #####
		self.imu_append_x = np.append(self.imu_append_x, [self.imu_msg.linear_acceleration.x])
		self.imu_append_y = np.append(self.imu_append_y, [self.imu_msg.linear_acceleration.y])
		self.imu_append_z = np.append(self.imu_append_z, [self.imu_msg.linear_acceleration.z])
		self.imu_derivation_x = np.std(self.imu_append_x)
		self.imu_derivation_y = np.std(self.imu_append_y)
		self.imu_derivation_z = np.std(self.imu_append_z)
		# self.imu_append_x = self.imu_msg.linear_acceleration.x
		# self.imu_append_y = self.imu_msg.linear_acceleration.y
		# self.imu_append_z = self.imu_msg.linear_acceleration.z
		# self.imu_derivation_x = np.std(self.imu_append_x)
		# self.imu_derivation_y = np.std(self.imu_append_y)
		# self.imu_derivation_z = np.std(self.imu_append_z)		

		self.instalibity_index = np.sqrt(np.square(self.imu_derivation_x) + np.square(self.imu_derivation_y) + np.square(self.imu_derivation_z))
		#print(f"Instability index = {self.instalibity_index:.3f}")
		##### End of section 3 #####
  
		print(f"Instability index = {self.instalibity_index:.3f}")
  
		##### Section 4 - Torque Mean #####
		self.fl_calf_append = self.high_state_msg.motorState[0].tauEst
		self.fl_hip_append = self.high_state_msg.motorState[1].tauEst
		self.fl_thigh_append = self.high_state_msg.motorState[2].tauEst
		self.fr_calf_append = self.high_state_msg.motorState[3].tauEst
		self.fr_hip_append = self.high_state_msg.motorState[4].tauEst
		self.fr_thigh_append = self.high_state_msg.motorState[5].tauEst
		self.rl_calf_append = self.high_state_msg.motorState[6].tauEst
		self.rl_hip_append = self.high_state_msg.motorState[7].tauEst
		self.rl_thigh_append = self.high_state_msg.motorState[8].tauEst
		self.rr_calf_append = self.high_state_msg.motorState[9].tauEst
		self.rr_hip_append = self.high_state_msg.motorState[10].tauEst
		self.rr_thigh_append = self.high_state_msg.motorState[11].tauEst
		#print(f"FL Calf = {self.high_state_msg.motorState[0].tauEst:.3f}")
  
		self.mean_torque = (self.fl_calf_append + self.fl_hip_append + self.fl_thigh_append + self.fr_calf_append + self.fr_hip_append + self.fr_thigh_append + self.rl_calf_append + self.rl_hip_append + self.rl_thigh_append + self.rr_calf_append + self.rr_hip_append + self.rr_thigh_append) / 12
		##### End of section 4 #####
  
		print(f"Mean torque = {self.mean_torque:.3f}")
  
		##### Section 5 - Foot Force Mean #####
		self.foot_force1_append = self.high_state_msg.footForce[0]
		self.foot_force2_append = self.high_state_msg.footForce[1]
		self.foot_force3_append = self.high_state_msg.footForce[2]
		self.foot_force4_append = self.high_state_msg.footForce[3]
  
		self.mean_foot_force = (self.foot_force1_append + self.foot_force2_append + self.foot_force3_append + self.foot_force4_append) / 4
		##### End of section 5 #####
  
		print(f"Mean foot force = {self.mean_foot_force:.3f}")
  
  		##### Section 6 - Power Consumption #####
		self.power_consumption = self.high_state_msg.bms.current

		#self.mean_power = np.mean(self.power_consumption)

		self.voltage_consumption = self.high_state_msg.bms.cell_vol[0]
  
		#self.mean_voltage = np.mean(self.voltage_consumption)
		##### End of section 6 #####

		print(f"Mean power = {self.power_consumption:.3f}")
  
		print (f"Mean voltage = {self.voltage_consumption:.3f}")


	def fill_logfile(self):
		"""
		Function responsible for writing the required logfiles with the data
		"""
		writer_raw = csv.writer(self.file_raw, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		writer_raw.writerow(["%.3f" % self.prev_x, "%.3f" % self.prev_y,
		 "%.6f" % self.prev_z, self.odom_msg.header.stamp.secs, self.odom_msg.header.stamp.nsecs])
		writer_metrics = csv.writer(self.file_metrics, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		writer_metrics.writerow(["%.3f" % self.travelled_distance, "%.3f" % self.mean_velocity, "%.3f" % self.instalibity_index, "%.3f" % self.mean_torque, "%.3f" % self.mean_foot_force, "%.3f" % self.power_consumption, "%.3f" % self.voltage_consumption])

	def detect_end_condition(self):
		if self.status_msg.status_list:
			current_status = self.status_msg.status_list[0].status
			current_goal_id = self.status_msg.status_list[0].goal_id.id

			# Check if the goal ID has changed (new goal received)	
			if current_status != self.last_status:
				if self.last_status == 1 and current_status == 3:
					self.goal_counter += 1
					self.goal_reached = True
					self.goal_processed = False  # Reset the processed flag for the new goal
				self.last_status = current_status

			# Goal reached (status 3) and not yet processed
			if self.goal_reached and not self.goal_processed:
				writer_raw = csv.writer(self.file_raw, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_raw.writerow(["REACHED GOAL %d" % self.goal_counter])
				writer_metrics = csv.writer(self.file_metrics, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_metrics.writerow(["REACHED GOAL %d" % self.goal_counter])
				print(f"Goal {self.goal_counter} reached")
				if self.goal_counter == 3:
					self.file_raw.close()
					self.file_metrics.close()
					self.trigger_end == True
					print("All goals reached. Exiting...")
					exit()
				self.goal_reached = False
				self.goal_processed = True
			elif current_status == 4 or self.trigger_end:
				writer_raw = csv.writer(self.file_raw, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_raw.writerow(["ABORTED"])
				writer_metrics = csv.writer(self.file_metrics, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_metrics.writerow(["ABORTED"])
				print("Goal aborted")
				#exit()


	def main(self):
		"""
		Main function, responsible for calling the remaining functions.
		"""
		rate = rospy.Rate(5)
  
		while not rospy.is_shutdown():

			self.read_odom_and_goal()
			self.metrics_calculator()
			self.fill_logfile()
			self.detect_end_condition()
			rate.sleep()




if __name__ == "__main__":

	rospy.init_node('metrics_extractor')

	metrics = MetricsExtractor()
	metrics.main()
