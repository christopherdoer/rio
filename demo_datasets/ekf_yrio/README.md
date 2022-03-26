# ekf_yrio Demo Dataset

The demo datasets have been recorded with a sensor setup consisting of an Analog Devices ADIS16448 IMU and a TI
 IWR6843AOPEVM. The IMU rate is ca. 410 Hz and the radar rate is 10Hz. The radar measurements are hardware triggered to synchronize the measurements with the IMU data.

# icins2021_datasets_ekf-yrio_rosbag (RIO Point Cloud Format)
A dataset recorded in an office building. Pseudo ground truth has been  created using VINS with loop closure. The radar sensor is mounted facing at approx. 45 degrees to the left. The
  extrinsic calibration is given in [calib_radar_inertial_datasets_icins_2021_carried](calib_radar_inertial_datasets_icins_2021_carried.yaml).
More information regarding the sensor rig is given in the corresponding paper: [Yaw aided Radar Inertial Odometry using Manhattan World Assumptions](https://christopherdoer.github.io/publication/2021_05_ICINS2021).


The rosbag contains the following topics:
- /ground_truth/pose (geometry_msgs/PoseStamped): Ground truth or pseudo ground truth pose
- /ground_truth/twist (geometry_msgs/TwistStamped): Ground truth velocity (vicon dataset only)
- /ground_truth/twist_body (geometry_msgs/TwistStamped): Ground truth velocity of body frame (vicon dataset only)
- /sensor_platform/imu (sensor_msgs/Imu): IMU data
- /sensor_platform/baro (sensor_msgs/FluidPressure): Barometer data
- /sensor_platform/radar/trigger (std_msgs/Header): Radar trigger, marks the start of a radar scan
- /sensor_platform/radar/scan (sensor_msgs/PointCloud2): Radar scan whereas each point consists of: x, y, z, snr_db
, v_doppler_mps, noise_db and  range. The time stamp is already in sync with the corresponding trigger header.

Run with [ekf_yrio](../../ekf_yrio):
~~~[shell]
roslaunch ekf_yrio demo_datasets_ekf-yrio_rosbag.launch
~~~

