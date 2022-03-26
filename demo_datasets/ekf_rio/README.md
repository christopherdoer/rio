# ekf_rio Demo Dataset

The demo datasets have been recorded with a sensor setup consisting of an Analog Devices ADIS16448 IMU and a TI
 IWR6843AOPEVM. The IMU rate is ca. 410 Hz and the radar rate is 10Hz. The radar measurements are hardware triggered to synchronize the measurements with the IMU data.


# vicon_easy (RIO Point Cloud Format)
A dataset recorded at our motion capture lab. Ground truth has been created fusing the motion capture and inertial data with a batch optimization. The radar sensor is facing forward. The extrinsic calibration is given in [calib_vicon_dataset](calib_vicon_dataset.yaml).
More information regarding the sensor rig is given in the corresponding paper: [Radar Inertial Odometry with Online Calibration](https://christopherdoer.github.io/publication/2020_11_ENC2020).

Run with [ekf_rio](../../ekf_rio):
~~~[shell]
roslaunch ekf_rio demo_datasets_ekf-rio_rosbag.launch do_plot:=True 
~~~

The rosbag contains the following topics:
- /ground_truth/pose (geometry_msgs/PoseStamped): Ground truth or pseudo ground truth pose
- /ground_truth/twist (geometry_msgs/TwistStamped): Ground truth velocity (vicon dataset only)
- /ground_truth/twist_body (geometry_msgs/TwistStamped): Ground truth velocity of body frame (vicon dataset only)
- /sensor_platform/imu (sensor_msgs/Imu): IMU data
- /sensor_platform/baro (sensor_msgs/FluidPressure): Barometer data
- /sensor_platform/radar/trigger (std_msgs/Header): Radar trigger, marks the start of a radar scan
- /sensor_platform/radar/scan (sensor_msgs/PointCloud2): Radar scan whereas each point consists of: x, y, z, snr_db
, v_doppler_mps, noise_db and  range. The time stamp is already in sync with the corresponding trigger header.


# ti_mmwave_rospkg_demo (ti_mmwave_rospkg Point Cloud Format)
This demo dataset has been recorded with the ti_mmwave_rospkg (version 3.3.0) driver.
It also features trigger messages, the calibration is provided: [calib_ti_mmwave_rospkg_dataset](calib_ti_mmwave_rospkg_dataset.yaml).

Run with [ekf_rio](../../ekf_rio):
~~~[shell]
roslaunch ekf_rio ti_mmwave_ekf-rio_rosbag.launch run_without_radar_trigger:=False
~~~

The rosbag contains the following topics:
- /ground_truth/pose (geometry_msgs/PoseStamped): Ground truth or pseudo ground truth pose
- /ground_truth/twist (geometry_msgs/TwistStamped): Ground truth velocity (vicon dataset only)
- /ground_truth/twist_body (geometry_msgs/TwistStamped): Ground truth velocity of body frame (vicon dataset only)
- /sensor_platform/imu (sensor_msgs/Imu): IMU data
- /sensor_platform/baro (sensor_msgs/FluidPressure): Barometer data
- /sensor_platform/radar/trigger (std_msgs/Header): Radar trigger, marks the start of a radar scan
- /sensor_platform/radar/scan (sensor_msgs/PointCloud2): Radar scan whereas each point consists of: x, y, z, intensity, velocity. The time stamp is already in sync with the corresponding trigger header.


