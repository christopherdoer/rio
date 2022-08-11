// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2022  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <atomic>
#include <queue>
#include <mutex>
#include <random>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>

#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>

#include <rio_utils/data_types.h>
#include <rio_utils/simple_profiler.h>
#include <radar_ego_velocity_estimator/radar_point_cloud.h>

#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator.h>

#include <x_rio/baro_altimeter.h>
#include <x_rio/yaw_aiding_manhattan_world.h>

#include <x_rio/x_rio_ros.h>

#include <gnss_x_rio/gnss_state.h>
#include <gnss_x_rio/gnss_x_rio_filter.h>
#include <gnss_x_rio/GnssXRioConfig.h>

#include <gnss_x_rio/GnssMeasurement.h>

namespace rio
{
struct GnssPointCloudType
{
  float x;
  float y;
  float z;
  uint pos_inlier;
  uint vel_inlier;
  float pos_accuracy;
  float vel_accuracy;
  uint n_satelites;
  float gdop;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

/**
 * @brief The XRioRos class is the ros interface of the XRIO class providing an online processing (ros mode) and
 * postprocessing (rosbag mode)
 */

class GnssXRioRos
{
public:
  /**
   * @brief EkfRioRos constructor
   * @param nh   node handle
   */
  GnssXRioRos(ros::NodeHandle& nh);

  /**
   * @brief Run in ros mode (sensor data is received via the subscribers)
   */
  void run();

  /**
   * @brief Run in rosbag mode (sensor data is read from a rosbag) executing the filter a maximum speed
   * @param rosbag_path    absolute path to the rosbag
   * @param bag_start      skip the first bag_start seconds of the rosbag (-1: start at 0s)
   * @param bag_duration   process only until bag_start + bag_duration (-1: full bag)
   * @param sleep_ms       sleep for sleep_ms milliseconds after each radar scan to limit the execution speed of the
   * filter
   */
  void
  runFromRosbag(const std::string& rosbag_path, const Real bag_start, const Real bag_duration, const Real sleep_ms);

protected:
  /**
   * @brief Interal update function
   */
  void iterate();

  /**
   * @brief Sensor data queue iteration functions
   */
  void iterateImu();
  void iterateBaro();
  void iterateRadarTrigger();
  void iterateRadarScan();
  void iterateRadarSim();
  void iterateGpsValid();
  void iterateGps();

  /**
   * @brief Tries to initializes the filter with the provided imu mueasurement
   * @returns true init successfull
   */
  bool init(const ImuDataStamped& imu_data);

  /**
   * @brief Init functions
   */
  bool initBaro();
  bool initGnss();

  /**
   * @brief Reconfigure callback, enables online reconfigure using rqt_reconfigure
   */
  void reconfigureCallback(gnss_x_rio::GnssXRioConfig& config, uint32_t level);

  template <class ConfigContainingXRioConfig>
  void configure(ConfigContainingXRioConfig& config)
  {
    gnss_x_rio_filter_.configure(config);
    radar_ego_velocity_.configure(config);
    yaw_aiding_manhattan_world_.configure(config);
    gnss_state_.configure(config);
  }

  /**
   * @brief IMU callback, called by ros::spin_once (ros mode) or by the rosbag loop (rosbag mode)
   */
  void callbackIMU(const sensor_msgs::ImuConstPtr& imu_msg);

  /**
   * @brief Baro callback, called by ros::spin_once (ros mode) or by the rosbag loop (rosbag mode)
   */
  void callbackBaroAltimter(const sensor_msgs::FluidPressureConstPtr& baro_msg);

  /**
   * @brief Radar scan callback, called by ros::spin_once (ros mode) or by the rosbag loop (rosbag mode)
   * @note: the pcl is expected to feature for each point: x, y, z, snr_db, noise_db, v_doppler_mps see
   * radar_ego_velocity_estimator.cpp
   */
  void callbackRadarScan(const uint id, const sensor_msgs::PointCloud2ConstPtr& radar_msg);

  /**
   * @brief Radar trigger callback (indicates the begin of a radar scan measurement), called by ros::spin_once (ros
   * mode) or by the rosbag loop (rosbag mode)
   */
  void callbackRadarTrigger(const uint id, const std_msgs::HeaderConstPtr& trigger_msg);

  /**
   * @brief GNSS valid callback -> marks the time stamp of a GNSS measurement
   * @param valid_msg
   */
  void callbackGnssValid(const std_msgs::HeaderConstPtr& valid_msg);

  /**
   * @brief GNSS measurment callback
   */
  void callbackGnssMeasurement(const gnss_x_rio::GnssMeasurementConstPtr& gnss_msg);

  /**
   * @brief Does all ros publishing
   */
  void publish();

  /**
   * @brief Prints evaluation stats assuming the start and end pose are equal, only on rosbag mode
   */
  void printStats();

  const std::string kStreamingPrefix = "[GnssXRioRos]: ";

  dynamic_reconfigure::Server<gnss_x_rio::GnssXRioConfig> reconfigure_server_;

  ros::NodeHandle nh_;

  ros::Subscriber sub_imu_;
  ros::Subscriber sub_baro_;
  std::vector<ros::Subscriber> subs_radar_scan_;
  std::vector<ros::Subscriber> subs_radar_trigger_;
  ros::Subscriber sub_gnss_valid_;
  ros::Subscriber sub_gnss_measurement_;

  ros::Publisher pub_cov_;
  ros::Publisher pub_nom_;
  ros::Publisher pub_odometry_;
  ros::Publisher pub_pose_path_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_combined_radar_scan_;
  std::vector<ros::Publisher> pubs_radar_scan_inlier_;
  ros::Publisher pub_radar_yaw_inlier_;
  ros::Publisher pub_global_pose_;
  ros::Publisher pub_local_orign_;
  ros::Publisher pub_gnss_pcl_;

  ros::Publisher pub_ground_truth_pose_;
  ros::Publisher pub_ground_truth_twist_;
  ros::Publisher pub_ground_truth_twist_body_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  GnssXRioFilter gnss_x_rio_filter_;
  gnss_x_rio::GnssXRioConfig config_;

  YawAidingManhattanWorld yaw_aiding_manhattan_world_;

  std::mutex mutex_;

  std::atomic_bool initialized_;
  std::vector<ImuDataStamped> imu_init_;

  ImuDataStamped last_imu_;
  ImuDataStamped imu_data_;

  std::vector<ImuDataStamped> radar_w_queue_;

  std::queue<ImuDataStamped> queue_imu_;
  std::queue<sensor_msgs::FluidPressure> queue_baro_;
  std::queue<std::pair<uint, sensor_msgs::PointCloud2>> queue_radar_;
  std::queue<std::pair<uint, std_msgs::Header>> queue_radar_trigger_;
  std::queue<std::pair<uint, geometry_msgs::TwistStamped>> queue_v_r_sim_;
  std::queue<gnss_x_rio::GnssMeasurement> queue_gnss_measurement_;
  std::queue<std_msgs::Header> queue_gnss_valid_;

  SimpleProfiler profiler_;

  BaroAltimeter baro_altimeter_;
  bool baro_initialized_ = false;
  std::vector<Real> baro_init_vec_;
  Real baro_h_0_ = 0.0;

  reve::RadarEgoVelocityEstimator radar_ego_velocity_;
  std::vector<std::pair<std::shared_ptr<Isometry>, pcl::PointCloud<reve::RadarPointCloudType>>> radar_inlier_pcls_;
  RadarExtrinsicsVec radar_extrinsics_0_;

  ros::Time last_timestamp_pub_      = ros::Time(0.0);
  ros::Time last_timestamp_pose_pub_ = ros::Time(0.0);
  ros::Time filter_start_stamp_;
  ros::WallTime filter_start_wall_time_;

  std::string body_frame_id_ = "";
  std::vector<std::string> radar_frame_ids_;

  nav_msgs::Path pose_path_;

  // noise generator for sim mode
  std::default_random_engine noise_gen_;
  std::normal_distribution<double> noise_acc_;
  std::normal_distribution<double> noise_gyro_;
  std::normal_distribution<double> noise_v_r_x_;
  std::normal_distribution<double> noise_v_r_y_;
  std::normal_distribution<double> noise_v_r_z_;
  Vector3 sim_acc_offset_ = Vector3(0, 0, 0);

  std::vector<gnss_x_rio::GnssMeasurement> gnss_init_buffer_;
  bool gnss_initialized = false;
  Vector3 local_origin_ecef_;
  Vector3 local_origin_llh_;
  pcl::PointCloud<GnssPointCloudType> gnss_point_cloud_;
  ros::Time last_valid_gps_   = ros::TIME_MIN;
  ros::Time last_invalid_gps_ = ros::TIME_MIN;

  GnssState gnss_state_;
};

}  // namespace rio
