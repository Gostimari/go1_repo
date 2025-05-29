#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from statistics import mean, stdev, median
import csv
import os

class TableGenerator:

	def __init__(self):
		self.timestamp_secs = []
		self.timestamp_nsecs = []
		self.position_x = []
		self.position_y = []
		self.position_z = []
		self.travelled_distance = []
		self.velocity = []
		self.instability_index = []
		self.mean_torque = []
		self.duration = []
		self.fails = []
		self.file_table = open(f'../plots/elev/table.tex', mode='w')
		self.files_metrics = []
		self.files_raw = []
		self.dirs = []
		self.reached = []

		directory = "../logfiles/elev/"

		self.number_of_files=0
		self.number_of_experiments=50

		exclude = set(['misc'])
		for root,dirs,files in os.walk(directory):
			dirs[:] = [d for d in dirs if d not in exclude]
			dirs.sort()
			# if len(dirs) != 0:
			# 	self.number_of_experiments = len(dirs)
			# 	self.dirs = dirs
			for file in files:
				if file.startswith("logfile_metrics"):
					self.files_metrics.append(open(root+'/'+file, 'r'))
					self.number_of_files += 1
				elif file.startswith("logfile_raw"):
					self.files_raw.append(open(root+'/'+file, 'r'))
					

	def read_logfiles(self):
		for counter in range(self.number_of_files):
			# print(self.number_of_files)
			data = []
			csv_reader = csv.reader(self.files_raw[counter], delimiter=',')
			line_count = 0
			for row in csv_reader:
				for i in range(len(row)):
					if row[i] == "REACHED GOAL 1":
						pass
					elif row[i] == "REACHED GOAL 2":
						pass
					elif row[i] == "REACHED GOAL 3":
						pass
					else:
						data.append(row[i])
						line_count += 1

			aux = 0
			robot_x = []
			robot_y = []
			robot_z = []
			secs = []
			nsecs = []

			for elem in data:

				if(aux == 5):
					aux = 0

				if aux == 0:
					robot_x.append(elem)
				elif aux == 1:
					robot_y.append(elem)
				elif aux == 2:
					robot_z.append(elem)
				elif aux == 3:
					secs.append(elem)
				elif aux == 4:
					nsecs.append(elem)
				else:
					pass
				aux += 1

			self.timestamp_secs.append(secs)
			self.timestamp_nsecs.append(nsecs)
			self.position_x.append(robot_x)
			self.position_x.append(robot_y)
			self.position_z.append(robot_z)

			data = []
			csv_reader = csv.reader(self.files_metrics[counter], delimiter=',')
			line_count = 0
			for row in csv_reader:
				for i in range(len(row)):
					if row[i] == "ABORTED":
						self.fails.append(counter)
					elif row[i] == "REACHED GOAL 1":
						self.reached.append(counter)
					elif row[i] == "REACHED GOAL 2":
						self.reached.append(counter)
					elif row[i] == "REACHED GOAL 3":
						self.reached.append(counter)
					else:						
						data.append(row[i])
						line_count += 1

			aux = 0
			travelled_distance = []
			velocity = []
			instability_index = []
			mean_torque = []
			for elem in data[:-1]:

				if(aux == 4):
					aux = 0

				if aux == 0:
					travelled_distance.append(elem)
				elif aux == 1:
					velocity.append(elem)
				elif aux == 2:
					instability_index.append(elem)
				else:
					mean_torque.append(elem)
				aux += 1

			self.travelled_distance.append(travelled_distance[-1])
			self.velocity.append(velocity)
			self.instability_index.append(instability_index[-1])
			self.mean_torque.append(mean_torque)

			print(travelled_distance[-1])

			time_nsec = np.asanyarray(self.timestamp_nsecs[counter]).astype('float')[1:]
			time_nsec *= 1E-9
			time = np.asanyarray(self.timestamp_secs[counter]).astype('float')[1:]

			# Ensure both arrays have the same length
			min_length = min(len(time), len(time_nsec))
			time = time[:min_length]
			time_nsec = time_nsec[:min_length]

			time -= time[0]
			time += time_nsec
			self.duration.append(time[-1])  # Use the last element directly

	def table_generator(self):

		travelled_distance = []
		velocity = []
		instability_index = []
		mean_torque = []
		all_travelled_distance = []
		all_velocity = []
		all_instability_index = []
		all_mean_torque = []
		duration = []
		all_duration = []
		failure_rate = []

		begin_tabular = []
		line1 = []
		line2 = []
		line3 = []
		line4 = []
		line5 = []
		line6 = []
		#line7 = []
		end_tabular = []

		runs = int(self.number_of_files/self.number_of_experiments)


		columns = "l"
		for j in range(self.number_of_experiments):
			columns += "r"

		begin_tabular.append(f"\\begin{{tabular}}{{@{{}}{columns}@{{}}}}\n")
		line1.append(f"\\toprule Metrics")
		line2.append(f"Travelled Distance (m)")
		line3.append(f"Instant velocity (m/s)")
		line4.append(f"instability_index")
		line5.append(f"mean_torque (N/m)")
		line6.append(f"Elapsed Time (s)") 	
		#line7.append(f"Failure Rate (\\%)")
		end_tabular.append(f"\\end{{tabular}}")

		iterator = 0
		for counter in range(self.number_of_experiments):
			for i in range(runs):
					velocity.append(float(np.asanyarray(self.velocity[i][1])))
					mean_torque.append(float(np.asanyarray(self.mean_torque[i][-1])))
					duration.append(float(np.asanyarray(self.duration[i])))

			instability_index.append(float(np.asanyarray(self.instability_index[counter])))
			travelled_distance.append(float(np.asanyarray(self.travelled_distance[counter])))


			all_travelled_distance.append(travelled_distance)
			all_velocity.append(velocity)
			all_instability_index.append(instability_index)
			all_mean_torque.append(mean_torque)
			all_duration.append(duration)
			#print(velocity)
			travelled_distance = []
			velocity = []
			instability_index = []
			mean_torque = []
			duration = []

		# aux = 0
		# for iter in range(self.number_of_experiments):
		# 	for elem in self.fails:
		# 		if elem >= iter*runs and elem < (iter+1)*runs:
		# 			aux += 1
		# 	failure_rate.append(aux)
		# 	aux = 0

		# trav = np.array(all_travelled_distance).flatten()
		# vel = np.array(all_velocity).flatten()
		# insta = np.array(all_instability_index).flatten()
		# torque = np.array(all_mean_torque).flatten()

		

		for counter in range(self.number_of_experiments):
			mean_travelled_distance = mean(all_travelled_distance[counter])
			stdev_travelled_distance = stdev(trav)
			median_travelled_distance = median(all_travelled_distance[counter])
			mean_velocity = mean(all_velocity[counter])
			stdev_velocity = stdev(vel)
			median_velocity = median(all_velocity[counter])
			mean_instability_index = mean(all_instability_index[counter])
			stdev_instability_index = stdev(insta)
			median_instability_index = median(all_instability_index[counter])
			mean_mean_torque = mean(all_mean_torque[counter])
			stdev_mean_torque = stdev(torque)
			median_mean_torque = median(all_mean_torque[counter])
			mean_duration = mean(all_duration[counter])
			stdev_duration = stdev(all_duration)
			median_duration = median(all_duration[counter])
			#failure_rate_percentage = failure_rate[counter]/runs*100




		line1.append(f" & \\Mean & Median")
		line2.append(f" & ${mean_travelled_distance:.3f} \\pm${stdev_travelled_distance:.3f} & ${median_travelled_distance:.3f}")
		line3.append(f" & ${mean_velocity:.3f} \\pm${stdev_velocity:.3f} & ${median_velocity:.3f}")
		line4.append(f" & ${mean_instability_index:.3f} \\pm${stdev_instability_index:.3f} & ${median_instability_index:.3f}")
		line5.append(f" & ${mean_mean_torque:.3f} \\pm${stdev_mean_torque:.3f} & ${median_mean_torque:.3f}")
		line6.append(f" & ${mean_duration:.3f} \\pm${stdev_duration:.3f} & ${median_duration:.3f}")
			#line7.append(f" & ${failure_rate_percentage:.3f}$")

		line1.append(" \\\\ \\midrule\n")
		line2.append(" \\\\\n")
		line3.append(" \\\\\n")
		line4.append(" \\\\\n")
		line5.append(" \\\\\n")
		line6.append(" \\\\ \\bottomrule\n")

		self.file_table.writelines(begin_tabular)
		self.file_table.writelines(line1)
		self.file_table.writelines(line2)
		self.file_table.writelines(line3)
		self.file_table.writelines(line4)
		self.file_table.writelines(line5)
		self.file_table.writelines(line6)
		self.file_table.writelines(end_tabular)

	# def post_processing_effort(self):
	# 	aux_mean_ = []
	# 	for counter in range(self.number_of_files):
	# 		if counter not in self.fails:
	# 			pitch = np.abs(np.asanyarray(self.pitch[counter][1:]).astype('float'))
	# 			aux_mean_.append(np.mean(pitch))
	# 	mean_ = np.rad2deg(np.mean(aux_mean_))
	# 	stddev = np.rad2deg(np.std(aux_mean_))
	# 	print(f"{mean_=:0.3f} and {stddev=:0.3f}")


	# def post_processing_risks(self):
		
	# 	sigmoid_pitch = lambda x: 1 / (1 + np.exp(-0.25*x + 6))
	# 	sigmoid_roll = lambda x: 1 / (1 + np.exp(-0.20*x + 3))
		
	# 	aux_roll = []
	# 	aux_pitch = []
	# 	for counter in range(self.number_of_files):
	# 		if counter not in self.fails:
	# 			roll = np.rad2deg(np.abs(np.#!/usr/bin/env python3

	def main(self):

		self.read_logfiles()
		# self.post_processing_effort()
		# self.post_processing_risks()
		self.table_generator()
			

if __name__ == "__main__":

	table_generator = TableGenerator()
	table_generator.main()

	# 	mean_roll_danger = np.mean(aux_roll)
	# 	stddev_roll_danger = np.std(aux_roll)
	# 	mean_pitch_danger = np.mean(aux_pitch)
	# 	stddev_pitch_danger = np.std(aux_pitch)
	# 	print(f"{mean_roll_danger=:0.3f} and {stddev_roll_danger=:0.3f} and {mean_pitch_danger=:0.3f} and {stddev_pitch_danger=:0.3f}")	