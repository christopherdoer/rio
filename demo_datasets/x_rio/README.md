# x_rio Demo Datasets

The demo dataset have been recorded with a sensor setup consisting of an Analog Devices ADIS16448 IMU, two TI
 IWR6843AOPEVM point at approx. 45 deg to the left and right and a IWR6843RISK pointing forward. 
The IMU rate is ca. 210 Hz and the radar rates are 10Hz. 
The radar measurements are hardware triggered to synchronize the measurements with the IMU data.
The radar sensor are using Time Division Multiple Access (TDMA) to prevent interference.


# demo_x_rio (RIO Point Cloud Format)

This datasets has been recored in an office building and features the same start and end pose.
The extrinsic calibrations of the three radar sensors are given in [radar_left](./x_rio_demo_calib_left.yaml), [radar_center](./x_rio_demo_calib_center.yaml) and [radar_right](./x_rio_demo_calib_right.yaml).
More information regarding the sensor rig is given in the corresponding paper: [x-RIO: Radar Inertial Odometry with Multiple Radar Sensors and Yaw Aiding](https://christopherdoer.github.io/publication/2022_02_JGN2022).

The rosbag contains the following topics:
- /ground_truth/pose (geometry_msgs/PoseStamped): Ground truth or pseudo ground truth pose
- /ground_truth/twist (geometry_msgs/TwistStamped): Ground truth velocity (vicon dataset only)
- /ground_truth/twist_body (geometry_msgs/TwistStamped): Ground truth velocity of body frame (vicon dataset only)
- /sensor_platform/imu (sensor_msgs/Imu): IMU data
- /sensor_platform/baro (sensor_msgs/FluidPressure): Barometer data
- /sensor_platform/radar_left/trigger (std_msgs/Header): Left radar trigger, marks the start of a radar scan
- /sensor_platform/radar_left/scan (sensor_msgs/PointCloud2): Left radar scan whereas each point consists of: x, y, z, snr_db
, v_doppler_mps, noise_db and  range. The time stamp is already in sync with the corresponding trigger header.
- /sensor_platform/radar_center/trigger (std_msgs/Header): Center radar trigger
- /sensor_platform/radar_center/scan (sensor_msgs/PointCloud2): Center radar scan
- /sensor_platform/radar_center/trigger (std_msgs/Header): Right radar trigger
- /sensor_platform/radar_center/scan (sensor_msgs/PointCloud2): Right radar scan 


Run with [x_rio](../../x_rio):

~~~[shell]
roslaunch x_rio x_rio_demo.launch type:=rosbag_node enable_rviz:=True enable_plot:=True
~~~

# sim.bag
Sample dataset of simulated sensor data to test the x_rio characteristics. Simulates a sensor rig of three radar sensors and an IMU.

Run with [x_rio](../../x_rio) including 10 Monte Carlo runs and an evalution of the result:

~~~[shell]
rosrun x_rio run_sim.py 
~~~