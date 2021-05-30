# Demo Datasets

The demo datasets have been recorded with a sensor setup consisting of an Analog Devices ADIS16448 IMU and a TI
 IWR6843AOPEVM. The IMU rate is ca. 410 Hz and the radar rate is 10Hz. The radar measurements are hardware triggered to synchronize the measurements with the IMU data.

Datasets:
- vicon_easy: A dataset recorded at our motion capture lab. Ground truth has been created fusing the motion capture and inertial data with a batch optimization. The radar sensor is facing forward. The extrinsic calibration is given in [calib_vicon_dataset](calib_vicon_dataset.yaml).
- icins2021_datasets_ekf-yrio_rosbag.bag: A dataset recorded in an office building. Pseudo ground truth has been
 created using VINS with loop closure. The radar sensor is mounted facing at approx. 45 degrees to the left. The
  extrinsic calibration is given in [calib_radar_inertial_datasets_icins_2021_carried](calib_radar_inertial_datasets_icins_2021_carried.yaml).

The rosbags contain the following topics:
- /ground_truth/pose (geometry_msgs/PoseStamped): Ground truth or pseudo ground truth pose
- /ground_truth/twist (geometry_msgs/TwistStamped): Ground truth velocity (vicon dataset only)
- /ground_truth/twist_body (geometry_msgs/TwistStamped): Ground truth velocity of body frame (vicon dataset only)
- /sensor_platform/imu (sensor_msgs/Imu): IMU data
- /sensor_platform/baro (sensor_msgs/FluidPressure): Barometer data
- /sensor_platform/radar/trigger (std_msgs/Header): Radar trigger, marks the start of a radar scan
- /sensor_platform/radar/scan (sensor_msgs/PointCloud2): Radar scan whereas each point consists of: x, y, z, snr_db
, v_doppler_mps, noise_db and  range. The time stamp is already in sync with the corresponding trigger header.

The point cloud point type is sketched below, for an example implementation see [radar_point_cloud
](../rio_utils/src/radar_point_cloud.cpp):

~~~
struct RadarPointCloudType
{
  PCL_ADD_POINT4D;      // position in [m]
  float snr_db;         // CFAR cell to side noise ratio in [dB]
  float v_doppler_mps;  // Doppler velocity in [m/s]
  float noise_db;       // CFAR noise level of the side of the detected cell in [dB]
  float range;          // range in [m]
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
                                  
pcl::PointCloud<RadarPointCloudType> your_pcl;
~~~



