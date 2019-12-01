The kf_sensor_fusion Python file uses odometry and gps data from csv files, runs the Kalman filter algorithm and writes the fuse estimates into a third csv file. 

In order to run the Kalman Filter Algorithm, create a KFSensorFusion Object then call the function kalman_filter(). 

To try out different results for the fused estimates, modify the parameters of the object Rt_diagonal and Qt_diagonal. 
R(t) and Q(t) are two 3x3 matrices which represent process and measurement noise. 
The Rt_diagonal and Qt_diagonal parameters represent the 3 values in the diagonals in each of the 3x3 matrices and will influence how much the fused estimates will tend to resemble the motion or measurement model. 

To check the fused estimates written to file without rerunning the entire algorithm, instead of calling the kalman_filter() function, call the check_fused_estimates_file() function that will plot the robot's path from the fused estimates csv file.
