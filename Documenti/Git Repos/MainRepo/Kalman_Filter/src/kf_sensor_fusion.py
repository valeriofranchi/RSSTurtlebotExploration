import csv 
import pymap3d as pm
from math import pi, sin, cos, acos
import numpy as np
import matplotlib.pyplot as plt
import copy

class KFSensorFusion:
	def __init__(self, Rt_diagonal=(0.0001, 0.0001, 1 * pi / 180), Qt_diagonal=(0.01, 0.01, pi / 270)):
		#initialising variables for the Kalman Filter Algorithm 
		self.T_fused = np.identity(3)
		self.T_odom = np.identity(3)
		self.mean = np.zeros((3, 1), dtype="float32")
		self.covariance = np.identity(3)
		self.z = np.zeros((3, 1), dtype="float32")
		self.mean = self.initialise_mean()
		self.R_t = np.zeros((3, 3), dtype="float32")
		np.fill_diagonal(self.R_t, Rt_diagonal)
		self.Q_t = np.zeros((3, 3), dtype="float32")
		np.fill_diagonal(self.Q_t, Qt_diagonal)

		#initialising helper variables 
		self.odom_step = 0
		self.gps_step = 0
		self.odom_storage = []
		self.gps_storage = []
		self.fused_estimates = []
		self.x_cov, self.y_cov, self.yaw_cov = [], [], []
		self.x_innov, self.y_innov, self.yaw_innov = [], [], []

		#retrieving odometry and gps values from csv files, storing them and converting 
		#the gps values to the correct format 
		self.store_odometry()
		self.convert_gps_to_local()

	"""
	This function initialises the mean of the probability distribution by reading the first row 
	of the csv file and storing the odometry values.
	"""
	def initialise_mean(self):
		with open('../data/odometry.csv') as csv_file:
			csv_reader = csv.DictReader(csv_file, delimiter=",")
			first_row = csv_reader.next()
			mean = ([first_row["x"], first_row["y"], first_row["yaw"]])
		return mean
	
	"""
	This function takes as input x,y and yaw values and returns the equivalent rotational 
	transformation matrix. 
	"""
	def compose_trans_matrix(self, x, y, yaw):
		return np.matrix([[cos(yaw), -sin(yaw), x], [sin(yaw), cos(yaw), y], [0, 0, 1]])
	
	"""
	This function returns the output of the motion model for both the fused mean from the KF and 
	the mean calculated solely on the motion model (using only the odometry).
	"""
	def motion_model(self):
		#get the values of the robot's position at a particular timestamp 
		(x, y, yaw) = self.odom_storage[self.odom_step][:3]

		#NOTE:The odometry values are flipped on the x-axis compared to the gps values therefore
		#it is necessary to multiply by -1 the x-value otherwise the fusion would produce an 
		#incorrect result

		#calculate motion model solely based on the odometry data (for display purposes)
		self.T_odom = self.T_odom.dot(self.compose_trans_matrix(x, y, yaw))
		odom_pose = [-1* self.T_odom[0,2], self.T_odom[1,2], acos(self.T_odom[0,0])]

		#calculate motion model using fused estimates of previous state
		self.T_fused = self.T_fused.dot(self.compose_trans_matrix(x, y, yaw))
		fused_pose = [-1* self.T_fused[0,2], self.T_fused[1,2], acos(self.T_fused[0,0])]  
		return odom_pose, fused_pose

	"""
	This function opens the odometry.csv file and stores the contents of it (x, y, yaw) inside a global numpy array
	together with the timestamp.
	"""
	def store_odometry(self):
		print("[INFO] Opening 'odometry.csv'...")
		print("[INFO] Storing odometry data...")
		with open('../data/odometry.csv') as csv_file:
			csv_reader = csv.DictReader(csv_file, delimiter=",")
			for row in csv_reader:
				self.odom_storage.append([float(row['x']), float(row['y']), float(row['yaw']), float(row['source_timestamp'])])
		np.asarray(self.odom_storage)	
	
	"""
	This function opens the gps.csv file, retrieves the longitude and latitude values, converts them to North-East-Up format 
	and stores these converted values (which correspond to x and y) inside a global numpy array together with the timestamp. 
	"""
	def convert_gps_to_local(self):
		lat, lon = [], []
		print("[INFO] Opening 'gps.csv'...")
		print("[INFO] Transforming geodetic coordinates to local east-north-up...")
		print("[INFO] Storing converted GPS data...")
		with open('../data/gps.csv') as csv_file:
			csv_reader = csv.DictReader(csv_file, delimiter=",")
			for row in csv_reader:
				lat.append(float(row['latitude']))
				lon.append(float(row['longitude']))
				current_lat = float(row['latitude'])
				current_lon = float(row['longitude'])
				x, y, _ = pm.geodetic2enu(current_lat, current_lon, 0, lat[0], lon[0], 0)
				self.gps_storage.append([x, y, float(row['timestamp'])])
		np.asarray(self.gps_storage)
	
	"""
	This function returns the gps coordinate with the timestamp closest to the timestamp of the odometry coordinate 
	since the odometry has more data points than the gps. 
	"""
	def get_gps_position(self):
		#retrieve gps coordinates of the current gps timestamp 
		gps_x, gps_y = self.gps_storage[self.gps_step][0], self.gps_storage[self.gps_step][1]

		#first iteration, increase the gps timestep since there is no previous timestamp to the initial, which will need to be 
		#checked in following loop
		if self.gps_step == 0:
			self.gps_step += 1 

		#from second step onwards check whether the previous or current gps timestamp is closer to the current odometry timestamp 
		if self.gps_step > 0:
			#if previous is closer, set the gps position to the previous step, else increase the counter and return the coordinates
			if abs(self.gps_storage[self.gps_step - 1][2] - self.odom_storage[self.odom_step][3]) < abs(self.gps_storage[self.gps_step][2] - self.odom_storage[self.odom_step][3]):
				gps_x, gps_y = self.gps_storage[self.gps_step - 1][0], self.gps_storage[self.gps_step - 1][1]
			else:
				self.gps_step += 1 
		return gps_x, gps_y		

	"""
	This function checks if the Fusion_ValerioFranchi.csv file is free. If it is not, it clears it and then appends the 
	timestamp, x, y and yaw fused estimates from the global list filled up from the KF sensor fusion algorithm.
	"""
	def write_fused_estimates(self):
		#read number of lines in file 
		lines = 0
		with open("../data/Fusion_ValerioFranchi.csv", "r") as f:
			 lines = sum(1 for row in f)

		#clear file if number of rows is more than 0, then write fused data 
		with open("../data/Fusion_ValerioFranchi.csv", "w") as f:
			if lines > 0:
				print("[INFO] Truncating past fused estimates from the csv file...")
				f.truncate()
			print("[INFO] Appending new fused estimates in the csv file...")
			writer = csv.writer(f, delimiter=",")
			writer.writerow(['timestamp', 'x', 'y', 'yaw'])
			for t, x, y, z in self.fused_estimates:
				writer.writerow([t, x, y, z])
		
		#check number of rows in the written file
		with open("../data/Fusion_ValerioFranchi.csv", "r") as f:
			print("[INFO] There are {} number of rows in the csv file".format(sum(1 for row in f)))		

	"""
	This function creates two figures. The first figure has four subplots: the motion model, the sensor model,
	the fused sensor data and all of the three overlapped. The second figure has three figures: the covariance 
	of x, the covariance of y, the covariance of the yaw over the timestamps retrieved from the covariance matrix.
	"""
	def plot_graphs(self, odom, gps, fused):
		plt.style.use('ggplot')
		fig = plt.figure(figsize=(10, 10))

		ax1 = fig.add_subplot(2, 2, 1)
		ax1.plot(odom[0], odom[1], color="red", linewidth=1.0)
		ax1.set_title("Motion Model")

		ax2 = fig.add_subplot(2, 2, 2)
		ax2.plot(gps[0], gps[1], color="blue", linewidth=1.0)
		ax2.set_title("Sensor Model")
		
		ax3 = fig.add_subplot(2, 2, 3)
		ax3.plot(fused[0], fused[1], color="lime", linewidth=1.0)
		ax3.set_title("Sensor Fusion")

		ax4 = fig.add_subplot(2, 2, 4)
		ax4.plot(odom[0], odom[1], color="red", label="motion model", linewidth=1.0)
		ax4.plot(gps[0], gps[1], color="blue", label="sensor model", linewidth=1.0)
		ax4.plot(fused[0], fused[1], color="lime", label="sensor fusion", linewidth=1.0)
		ax4.set_title("Models and Fusion Overlayed")
		ax4.legend()
		#fig.savefig("../plots/KF.png")

		fig2 = plt.figure(figsize=(10, 3))
		self.x_cov, self.y_cov, self.yaw_cov = np.asarray(self.x_cov), np.asarray(self.y_cov), np.asarray(self.yaw_cov)
		ax1 = fig2.add_subplot(1, 3, 1)
		ax1.plot(self.x_cov[:, 0], self.x_cov[:, 1], color="black", linewidth=1.5)
		ax1.set_title("X Covariance Over Time")

		ax2 = fig2.add_subplot(1, 3, 2)
		ax2.plot(self.y_cov[:, 0], self.y_cov[:, 1], color="violet", linewidth=1.5)
		ax2.set_title("Y Covariance Over Time")

		ax3 = fig2.add_subplot(1, 3, 3)
		ax3.plot(self.yaw_cov[:, 0], self.yaw_cov[:, 1], color="orange", linewidth=1.5)
		ax3.set_title("Yaw Covariance Over Time")
		#fig2.savefig("../plots/Cov.png")

		fig3 = plt.figure(figsize=(14, 4))
		fig4 = plt.figure(figsize=(14, 4))
		self.x_innov, self.y_innov, self.yaw_innov = np.asarray(self.x_innov), np.asarray(self.y_innov), np.asarray(self.yaw_innov)
		ax1 = fig3.add_subplot(1, 3, 1)
		ax1.plot(self.x_innov[:, 0], self.x_innov[:, 1], color="blue", linewidth=1.5)
		ax1.set_title("X Innovation Sequence Over Time")
		ax4 = fig4.add_subplot(1, 3, 1)
		ax4.plot(self.x_innov[:, 0], self.x_innov[:, 2], color="orange", linewidth=1.5)
		ax4.set_title("X Innovation Covariance Over Time")

		ax2 = fig3.add_subplot(1, 3, 2)
		ax2.plot(self.y_innov[:, 0], self.y_innov[:, 1], color="blue", linewidth=1.5)
		ax2.set_title("Y Innovation Sequence Over Time")
		ax5 = fig4.add_subplot(1, 3, 2)
		ax5.plot(self.x_innov[:, 0], self.x_innov[:, 2], color="orange", linewidth=1.5)
		ax5.set_title("Y Innovation Covariance Over Time")

		ax3 = fig3.add_subplot(1, 3, 3)
		ax3.plot(self.yaw_innov[:, 0], self.yaw_innov[:, 1], color="blue", linewidth=1.5)
		ax3.set_title("Yaw Innovation Sequence Over Time")
		ax6 = fig4.add_subplot(1, 3, 3)
		ax6.plot(self.x_innov[:, 0], self.x_innov[:, 2], color="orange", linewidth=1.5)
		ax6.set_title("Yaw Innovation Covariance Over Time")
		#fig3.savefig("../plots/SeqInnov.png")
		#fig4.savefig("../plots/CovInnov.png")

		#plot sensor fusion alone to inspect it better 
		plt.figure(figsize=(5, 5))
		plt.plot(fused[0], fused[1])
		plt.title("Fused Estimates")
		#plt.savefig("../plots/FusedEstimates.png")
		plt.show()

	"""
	This function appends the innovation covariance and sequence on x, y and yaw retrieved from the matrix and 
	vector in the parameters, together with the time-step, to global lists.
	"""
	def add_innovations(self, step, innovation_s, innovation_cov):
		diag_cov = np.diagonal(innovation_cov)
		self.x_innov.append([step, innovation_s[0], diag_cov[0]])
		self.y_innov.append([step, innovation_s[1], diag_cov[1]])
		self.yaw_innov.append([step, innovation_s[2], diag_cov[2]])
	
	"""
	This function appends the covariance on x, on y and on the yaw retrieved from the current version
	of the covariance matrix, together with the time-step, to global lists. 
	"""
	def add_covariance_errors(self, step):
		S = np.diagonal(self.covariance)
		self.x_cov.append([step, S[0]])
		self.y_cov.append([step, S[1]])
		self.yaw_cov.append([step, S[2]])
	
	"""
	This function is used to check whether the Fusion_ValerioFranchi.csv file contents generated by the kalman 
	filter function has the correct values by pulling up a figure of the sensor fusion coordinates. 
	"""
	def check_fused_estimates_file(self):
		with open('../data/Fusion_ValerioFranchi.csv') as csv_file:
			x = []
			y = []
			csv_reader = csv.DictReader(csv_file, delimiter=",")
			for row in csv_reader:
				x.append(float(row['x']))
				y.append(float(row['y']))
			plt.figure(figsize=(5, 5))
			plt.style.use('ggplot')
			plt.plot(x, y)
			plt.title("Fused Estimates")
			plt.show()

	"""
	This function runs the Kalman Filter Algorithm to fuse the motion model, which in this case is the
	local odometry values of the robot's position and the sensor model, which is the gps coordinates.
	The results are plotted on some graphs and appended to a csv file.  
	"""
	def kalman_filter(self):
		print("[INFO] Odometry poses array size is %d" %len(self.odom_storage))
		print("[INFO] GPS positions array size is %d" %len(self.gps_storage))

		#initialize lists to store values to display the graphs 
		odom_x, odom_y, odom_coords = [], [], []
		gps_x, gps_y, gps_coords = [], [], []
		fused_x, fused_y, fused_coords = [], [], []

		print("[INFO] Starting KF...")

		while self.odom_step < len(self.odom_storage):
			#update the lists with new covariance values for x,y and yaw at the current timestep
			self.add_covariance_errors(self.odom_step)

			#prediction step (mean and covariance prediction)
			motion_mean, self.mean = self.motion_model()
			self.covariance += self.R_t
			#appending motion model only values to arrays odom_x and odom_y
			odom_x.append(motion_mean[0])
			odom_y.append(motion_mean[1])

			#get gps positions and create z array with yaw=0 since gps has no orientation
			if self.gps_step < len(self.gps_storage):
				gpsX_val, gpsY_val = self.get_gps_position()
				self.z = [gpsX_val, gpsY_val, 0]
				#append gps values (sensor model only) to arrays gps_x and gps_y
				gps_x.append(gpsX_val)
				gps_y.append(gpsY_val)
			
			#correction step (kalman gain calculation, mean and covariance correction
			innovation_covariance = self.covariance + self.Q_t
			kalman_gain = self.covariance.dot(np.linalg.inv(innovation_covariance))
			innovation_sequence = np.array(self.z) - np.array(self.mean)
			self.mean += kalman_gain.dot(innovation_sequence)
			self.covariance = (np.identity(3) - kalman_gain).dot(self.covariance)

			#append x and y of fused values to arrays fused_x and fused_y
			fused_x.append(self.mean[0])
			fused_y.append(self.mean[1])
			
			#append fused mean and timestamps to array
			new_x, new_y, new_yaw = self.mean
			self.fused_estimates.append([self.odom_storage[self.odom_step][3], new_x, new_y, new_yaw])

			#update the transformation matrix describing the current position of the robot 
			self.T_fused[0, :] = [cos(new_yaw), -sin(new_yaw), -1*new_x]
			self.T_fused[1, :] = [sin(new_yaw), cos(new_yaw), new_y]

			#update the lists with new innovation values for x,y and yaw at the current timestep
			self.add_innovations(self.odom_step, innovation_sequence, innovation_covariance)

			#increase the timestep
			self.odom_step +=1 
		
		print("[INFO] Sensor Fusion finished.")
		print("[INFO] Storing Fused Sensor Data in Fusion_ValerioFranchi.csv file...")
		#write fused estimates results into csv file 
		self.write_fused_estimates()
		print("[INFO] Plotting Results...")
		#plot the graphs and figures 
		odom_coords.extend([odom_x, odom_y])
		gps_coords.extend([gps_x, gps_y])
		fused_coords.extend([fused_x, fused_y])		
		self.plot_graphs(odom_coords, gps_coords, fused_coords)
		
#create KFSensorFusion object and run the kalman filter function 
kf_localization = KFSensorFusion(Rt_diagonal=(0.0001, 0.0001, 1 * pi / 180), Qt_diagonal=(0.01, 0.01, pi / 270))
kf_localization.kalman_filter()






		
		
