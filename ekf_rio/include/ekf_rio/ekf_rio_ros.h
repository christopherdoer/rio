// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

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

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>

#include <rio_utils/data_types.h>
#include <rio_utils/simple_profiler.h>

#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator.h>

#include <ekf_rio/baro_altimeter.h>
#include <ekf_rio/ekf_rio_filter.h>
#include <ekf_rio/EkfRioConfig.h>

namespace rio
{
/**
 * @brief The EkfRioRos class is the ros interface of the EkfRio class providing an online processing (ros mode) and
 * postprocessing (rosbag mode)
 */
class EkfRioRos
{
public:
  /**
   * @brief EkfRioRos constructor
   * @param nh   node handle
   */
  EkfRioRos(ros::NodeHandle& nh);

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

private:
  /**
   * @brief Interal update function
   */
  void iterate();

  /**
   * @brief Tries to initializes the filter with the provided imu mueasurement
   * @returns true init successfull
   */
  bool initImu(const ImuDataStamped& imu_data);

  /**
   * @brief Reconfigure callback, enables online reconfigure using rqt_reconfigure
   */
  void reconfigureCallback(ekf_rio::EkfRioConfig& config, uint32_t level);

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
  void callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_msg);

  /**
   * @brief Radar trigger callback (indicates the begin of a radar scan measurement), called by ros::spin_once (ros
   * mode) or by the rosbag loop (rosbag mode)
   */
  void callbackRadarTrigger(const std_msgs::HeaderConstPtr& trigger_msg);

  /**
   * @brief Does all ros publishing
   */
  void publish();

  /**
   * @brief Prints evaluation stats assuming the start and end pose are equal, only on rosbag mode
   */
  void printStats();

  const std::string kStreamingPrefix = "[EkfRioRos]: ";

  dynamic_reconfigure::Server<ekf_rio::EkfRioConfig> reconfigure_server_;

  ros::Subscriber sub_imu_;
  ros::Subscriber sub_baro_;
  ros::Subscriber sub_radar_;
  ros::Subscriber sub_radar_trigger_;

  ros::Publisher pub_cov_;
  ros::Publisher pub_nom_;
  ros::Publisher pub_pose_path_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_radar_scan_inlier_;

  ros::Publisher pub_ground_truth_pose_;
  ros::Publisher pub_ground_truth_twist_;
  ros::Publisher pub_ground_truth_twist_body_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  EkfRioFilter ekf_rio_filter_;
  ekf_rio::EkfRioConfig config_;

  std::mutex mutex_;

  std::atomic_bool initialized_;
  std::vector<ImuDataStamped> imu_init_;

  ImuDataStamped last_imu_;
  ImuDataStamped imu_data_;

  std::vector<ImuDataStamped> radar_w_queue_;

  std::queue<ImuDataStamped> queue_imu_;
  std::queue<sensor_msgs::FluidPressure> queue_baro_;
  std::queue<sensor_msgs::PointCloud2> queue_radar_;
  std::queue<std_msgs::Header> queue_radar_trigger_;
  std::queue<geometry_msgs::PoseWithCovarianceStamped> queue_gloal_pose_;

  SimpleProfiler profiler_;

  BaroAltimeter baro_altimeter_;
  bool baro_initialized_ = false;
  std::vector<Real> baro_init_vec_;

  reve::RadarEgoVelocityEstimator radar_ego_velocity_;

  ros::Time last_timestamp_pub_      = ros::Time(0.0);
  ros::Time last_timestamp_pose_pub_ = ros::Time(0.0);
  ros::Time filter_start_stamp_;
  ros::WallTime filter_start_wall_time_;

  std::string body_frame_id_  = "";
  std::string radar_frame_id_ = "";

  nav_msgs::Path pose_path_;
};
}  // namespace rio
