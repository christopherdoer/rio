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

#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>

#include <rio_utils/ros_helper.h>
#include <rio_utils/math_helper.h>
#include <radar_ego_velocity_estimator/radar_point_cloud.h>

#include <x_rio/XRioState.h>
#include <x_rio/XRioCovariance.h>
#include <x_rio/msg_conversion.h>

#include <gnss_x_rio/gnss_x_rio_ros.h>

using namespace rio;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(GnssPointCloudType,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint, pos_inlier, pos_inlier)
                                  (uint, vel_inlier, vel_inlier)
                                  (float, pos_accuracy,   pos_accuracy)
                                  (float, vel_accuracy,   vel_accuracy)
                                  (uint, n_satelites,   n_satelites)
                                  )
// clang-format on

GnssXRioRos::GnssXRioRos(ros::NodeHandle& nh) : nh_{nh}, initialized_{false}, yaw_aiding_manhattan_world_(nh)
{
  reconfigure_server_.setCallback(boost::bind(&GnssXRioRos::reconfigureCallback, this, _1, _2));

  // subscribers
  sub_imu_  = nh.subscribe<sensor_msgs::Imu>(config_.topic_imu, 2, boost::bind(&GnssXRioRos::callbackIMU, this, _1));
  sub_baro_ = nh.subscribe<sensor_msgs::FluidPressure>(
      config_.topic_baro_altimeter, 2, boost::bind(&GnssXRioRos::callbackBaroAltimter, this, _1));
  sub_gnss_valid_ = nh.subscribe<std_msgs::Header>(
      config_.topic_gnss_valid, 2, boost::bind(&GnssXRioRos::callbackGnssValid, this, _1));
  sub_gnss_measurement_ = nh.subscribe<gnss_x_rio::GnssMeasurement>(
      config_.topic_gnss_measurement, 2, boost::bind(&GnssXRioRos::callbackGnssMeasurement, this, _1));

  // publishers
  pub_cov_                 = nh.advertise<x_rio::XRioCovariance>("covariance", 5);
  pub_nom_                 = nh.advertise<x_rio::XRioState>("state", 5);
  pub_odometry_            = nh.advertise<nav_msgs::Odometry>("odom", 1);
  pub_pose_path_           = nh.advertise<nav_msgs::Path>("pose_path", 1);
  pub_pose_                = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pub_twist_               = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
  pub_combined_radar_scan_ = nh.advertise<sensor_msgs::PointCloud2>("combined_radar_scan", 10);
  pub_radar_yaw_inlier_    = nh.advertise<sensor_msgs::PointCloud2>("yaw_inlier", 10);
  pub_local_orign_         = nh.advertise<sensor_msgs::NavSatFix>("local_origin", 10);
  pub_gnss_pcl_            = nh.advertise<sensor_msgs::PointCloud2>("gnss_pcl", 10);

  if (config_.republish_ground_truth)
  {
    pub_ground_truth_pose_  = nh.advertise<geometry_msgs::PoseStamped>(config_.topic_ground_truth_pose, 1000);
    pub_ground_truth_twist_ = nh.advertise<geometry_msgs::TwistStamped>(config_.topic_ground_truth_twist, 1000);
    pub_ground_truth_twist_body_ =
        nh.advertise<geometry_msgs::TwistStamped>(config_.topic_ground_truth_twist_body, 1000);
  }

  int n_radar = 0;

  if (getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, "n_radar", n_radar))
  {
    for (uint k = 0; k < n_radar; ++k)
    {
      const std::string prefix = "radar_" + std::to_string(k) + "/";
      Vector3 l_b_r;
      Quaternion q_b_r;

      bool success = true;
      success &= getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, prefix + "l_b_r_x", l_b_r.x());
      success &= getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, prefix + "l_b_r_y", l_b_r.y());
      success &= getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, prefix + "l_b_r_z", l_b_r.z());
      success &= getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, prefix + "q_b_r_w", q_b_r.w());
      success &= getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, prefix + "q_b_r_x", q_b_r.x());
      success &= getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, prefix + "q_b_r_y", q_b_r.y());
      success &= getRosParameter(nh, kStreamingPrefix, RosParameterType::Required, prefix + "q_b_r_z", q_b_r.z());

      std::string topic_radar_scan = "";
      success &= getRosParameter(
          nh, kStreamingPrefix, RosParameterType::Required, prefix + "topic_radar_scan", topic_radar_scan);

      std::string topic_radar_trigger = "";
      success &= getRosParameter(
          nh, kStreamingPrefix, RosParameterType::Required, prefix + "topic_radar_trigger", topic_radar_trigger);

      if (success)
      {
        radar_frame_ids_.resize(n_radar);
        radar_inlier_pcls_.resize(n_radar);

        Isometry T_b_r;
        T_b_r.translation() = l_b_r;
        T_b_r.linear()      = Matrix3(q_b_r);
        radar_extrinsics_0_.emplace_back(T_b_r);

        subs_radar_scan_.emplace_back(nh.subscribe<sensor_msgs::PointCloud2>(
            topic_radar_scan, 2, [k, this](const sensor_msgs::PointCloud2ConstPtr& pcl2_msg) {
              this->callbackRadarScan(k, pcl2_msg);
            }));

        subs_radar_trigger_.emplace_back(nh.subscribe<std_msgs::Header>(
            topic_radar_trigger, 2, [k, this](const std_msgs::HeaderConstPtr& trigger_msg) {
              this->callbackRadarTrigger(k, trigger_msg);
            }));

        pubs_radar_scan_inlier_.emplace_back(
            nh.advertise<sensor_msgs::PointCloud2>("radar_" + std::to_string(k) + "_inlier", 10));
      }
    }
  }

  // in sim mode setup noise generators
  if (config_.sim_mode)
  {
    ROS_WARN_STREAM(kStreamingPrefix << "Running in simulation mode!");
    if (config_.generate_imu_noise)
      ROS_WARN_STREAM(kStreamingPrefix << "Generating IMU noise!");

    if (config_.generate_radar_noise)
      ROS_WARN_STREAM(kStreamingPrefix << "Generating v_r noise!");

    noise_gen_   = std::default_random_engine(ros::WallTime::now().nsec);
    noise_acc_   = std::normal_distribution<double>(0, config_.sim_sigma_acc);
    noise_gyro_  = std::normal_distribution<double>(0, config_.sim_sigma_gyro);
    noise_v_r_x_ = std::normal_distribution<double>(0, config_.sim_sigma_v_r_x);
    noise_v_r_y_ = std::normal_distribution<double>(0, config_.sim_sigma_v_r_y);
    noise_v_r_z_ = std::normal_distribution<double>(0, config_.sim_sigma_v_r_z);

    std::normal_distribution<double> noise_bias_acc_ = std::normal_distribution<double>(0, config_.sigma_b_a);
  }

  last_valid_gps_ = ros::TIME_MIN;
  ros::Duration(0.5).sleep();
}

bool GnssXRioRos::init(const ImuDataStamped& imu_data)
{
  imu_init_.emplace_back(imu_data);

  const auto T_init_so_far = (imu_init_.back().time_stamp - imu_init_.front().time_stamp).toSec();
  if (T_init_so_far > config_.T_init)
  {
    initialized_            = gnss_x_rio_filter_.init(imu_init_, radar_extrinsics_0_);
    filter_start_stamp_     = gnss_x_rio_filter_.getTimestamp();
    filter_start_wall_time_ = ros::WallTime::now();
    initBaro();
  }
  else
  {
    const uint mod = 1.0 / imu_data.dt + 1;
    if (imu_init_.size() % mod == 0)
      ROS_INFO("%s Init progress: %0.2f / %0.2f seconds", kStreamingPrefix.c_str(), T_init_so_far, config_.T_init);
  }
  return initialized_;
}

bool GnssXRioRos::initBaro()
{
  baro_h_0_ = std::accumulate(baro_init_vec_.begin(), baro_init_vec_.end(), 0.0) / baro_init_vec_.size();
  ROS_INFO_STREAM(kStreamingPrefix << "Initialized baro h_0: " << baro_h_0_);
  baro_initialized_ = true;

  return true;
}

void GnssXRioRos::run()
{
  ROS_INFO_STREAM(kStreamingPrefix << "Navigation filter started!");

  ros::WallRate r(1000);
  last_timestamp_pub_ = ros::TIME_MIN;
  while (ros::ok())
  {
    iterate();
    ros::spinOnce();
    r.sleep();
  }
}

bool GnssXRioRos::initGnss()
{
  // take measurement with best accuracy
  if (gnss_init_buffer_.empty())
  {
    ROS_WARN_THROTTLE(1, "Unable to init GNSS --> no meaaurements!");
    return false;
  }
  auto best_measurement = gnss_init_buffer_.front();
  for (const auto& measurement : gnss_init_buffer_)
  {
    if (measurement.pos_accuracy < best_measurement.pos_accuracy)
      best_measurement = measurement;
  }

  if (best_measurement.pos_accuracy < config_.min_pos_accuracy_init)
  {
    local_origin_ecef_ =
        Vector3(best_measurement.pos_ecef[0], best_measurement.pos_ecef[1], best_measurement.pos_ecef[2]);
    local_origin_llh_ = Vector3(angles::to_degrees(best_measurement.pos_llnh[0]),
                                angles::to_degrees(best_measurement.pos_llnh[1]),
                                -best_measurement.pos_llnh[2]);
    gnss_initialized  = true;
    ROS_INFO("%s GNSS initialized (llh): %0.9f, %0.9f, %0.6f @posAccuracy %0.2f",
             kStreamingPrefix.c_str(),
             local_origin_llh_.x(),
             local_origin_llh_.y(),
             local_origin_llh_.z(),
             best_measurement.pos_accuracy);
    return true;
  }

  ROS_WARN_THROTTLE(
      1, "Unable to init GNSS --> best posAccuracy still too large: %0.2f", best_measurement.pos_accuracy);
  return false;
}

void GnssXRioRos::runFromRosbag(const std::string& rosbag_path,
                                const Real bag_start,
                                const Real bag_duration,
                                const Real sleep_ms)
{
  rosbag::Bag source_bag;
  source_bag.open(rosbag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(config_.topic_imu);
  topics.push_back(config_.topic_baro_altimeter);
  topics.push_back(config_.topic_radar_scan);
  topics.push_back(config_.topic_radar_trigger);
  topics.push_back(config_.topic_gnss_valid);
  topics.push_back(config_.topic_gnss_measurement);

  if (config_.republish_ground_truth)
  {
    topics.push_back(config_.topic_ground_truth_pose);
    topics.push_back(config_.topic_ground_truth_twist);
    topics.push_back(config_.topic_ground_truth_twist_body);
  }

  std::unordered_map<std::string, uint> radar_topic_to_id;
  for (uint id = 0; id < subs_radar_scan_.size(); ++id)
  {
    topics.emplace_back(subs_radar_scan_.at(id).getTopic());
    radar_topic_to_id[subs_radar_scan_.at(id).getTopic()] = id;

    topics.emplace_back(subs_radar_trigger_.at(id).getTopic());
    radar_topic_to_id[subs_radar_trigger_.at(id).getTopic()] = id;
  }

  if (config_.sim_mode)
  {
    topics.push_back(config_.topic_v_r_0);

    if (subs_radar_scan_.size() > 1)
      topics.push_back(config_.topic_v_r_1);
    if (subs_radar_scan_.size() > 2)
      topics.push_back(config_.topic_v_r_2);
  }

  rosbag::View view(source_bag, rosbag::TopicQuery(topics));

  auto first_timestamp = ros::TIME_MIN;

  for (const rosbag::MessageInstance& m : view)
  {
    if (first_timestamp == ros::TIME_MIN)
      first_timestamp = m.getTime();

    if ((m.getTime() - first_timestamp).toSec() < bag_start)
      continue;

    if ((m.getTime() - first_timestamp).toSec() > bag_start + bag_duration)
      break;

    const auto topic = m.getTopic();
    if (topic == config_.topic_imu)
    {
      const auto imu_msg_bag = m.instantiate<sensor_msgs::Imu>();
      if (imu_msg_bag != NULL)
      {
        callbackIMU(imu_msg_bag);
      }
    }
    else if (topic == config_.topic_baro_altimeter)
    {
      const auto baro_msg = m.instantiate<sensor_msgs::FluidPressure>();
      if (baro_msg != NULL)
        callbackBaroAltimter(baro_msg);
    }
    if (topic == config_.topic_ground_truth_pose)
    {
      const auto msg = m.instantiate<geometry_msgs::PoseStamped>();
      if (msg)
        pub_ground_truth_pose_.publish(msg);
    }
    else if (topic == config_.topic_ground_truth_twist)
    {
      const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
      if (msg)
        pub_ground_truth_twist_.publish(msg);
    }
    else if (topic == config_.topic_ground_truth_twist_body)
    {
      const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
      if (msg)
        pub_ground_truth_twist_body_.publish(msg);
    }
    else if (topic == config_.topic_v_r_0)
    {
      const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
      if (msg)
        queue_v_r_sim_.push({0, *msg});
    }
    else if (topic == config_.topic_v_r_1)
    {
      const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
      if (msg)
        queue_v_r_sim_.push({1, *msg});
    }
    else if (topic == config_.topic_v_r_2)
    {
      const auto msg = m.instantiate<geometry_msgs::TwistStamped>();
      if (msg)
        queue_v_r_sim_.push({2, *msg});
    }
    else if (topic == config_.topic_gnss_valid)
    {
      const auto gnss_valid_msg = m.instantiate<std_msgs::Header>();
      if (gnss_valid_msg != NULL)
        callbackGnssValid(gnss_valid_msg);
    }
    else if (topic == config_.topic_gnss_measurement)
    {
      const auto gnss_msg = m.instantiate<gnss_x_rio::GnssMeasurement>();
      if (gnss_msg != NULL)
        callbackGnssMeasurement(gnss_msg);
    }
    else
    {
      // should be trigger or scan
      const auto radar_scan = m.instantiate<sensor_msgs::PointCloud2>();
      if (radar_scan)
      {
        // check id
        const auto iter_id = radar_topic_to_id.find(topic);

        if (iter_id != radar_topic_to_id.end())
        {
          callbackRadarScan(iter_id->second, radar_scan);
          if (sleep_ms > 0)
            ros::Duration(sleep_ms / 1.0e3).sleep();
        }
      }
      else
      {
        const auto iter_id = radar_topic_to_id.find(topic);

        if (iter_id != radar_topic_to_id.end())
        {
          const auto radar_trigger = m.instantiate<std_msgs::Header>();
          if (radar_trigger)
            callbackRadarTrigger(iter_id->second, radar_trigger);
        }
      }
    }

    iterate();
    ros::spinOnce();
  }

  publish();
  printStats();
}

void GnssXRioRos::iterate()
{
  mutex_.lock();
  iterateImu();
  iterateBaro();
  iterateRadarTrigger();
  iterateRadarScan();
  iterateRadarSim();
  iterateGpsValid();
  iterateGps();

  mutex_.unlock();
}

void GnssXRioRos::iterateImu()
{
  if (queue_imu_.size() > 0)
  {
    imu_data_ = queue_imu_.front();
    queue_imu_.pop();

    if (body_frame_id_.empty())
      body_frame_id_ = imu_data_.frame_id;

    if (!initialized_)
    {
      init(imu_data_);
    }
    else
    {
      profiler_.start("processIMU");
      if (!gnss_x_rio_filter_.propagate(imu_data_))
      {
        ROS_ERROR_STREAM(kStreamingPrefix << "Error during propagation!");
      }
      profiler_.stop("processIMU");

      if (1.0 / (imu_data_.time_stamp - last_timestamp_pub_).toSec() < config_.publisher_rate)
      {
        profiler_.start("publishSpinOnce");
        publish();
        last_timestamp_pub_ = imu_data_.time_stamp;
        profiler_.stop("publishSpinOnce");
      }
    }

    // collect angluar rate measurements during the radar scan
    if (!radar_w_queue_.empty() && (radar_w_queue_.back().time_stamp - radar_w_queue_.front().time_stamp).toSec() <
                                       config_.radar_frame_ms / 1.0e-3)
      radar_w_queue_.emplace_back(imu_data_);
  }
}
void GnssXRioRos::iterateBaro()
{
  if (!queue_baro_.empty())
  {
    auto baro_temp_msg = queue_baro_.front();
    if (!baro_initialized_)
    {
      baro_init_vec_.emplace_back(baro_altimeter_.calculate_rel_neg_height(baro_temp_msg, baro_h_0_));
      queue_baro_.pop();
    }
    else if (gnss_x_rio_filter_.getTimestamp() >= baro_temp_msg.header.stamp)
    {
      queue_baro_.pop();
      if (config_.altimeter_update)
      {
        if (initialized_)
        {
          profiler_.start("baro_altimeter_update");
          const auto h_rel = baro_altimeter_.calculate_rel_neg_height(baro_temp_msg, baro_h_0_);
          gnss_x_rio_filter_.updateAltimeter(h_rel, config_.sigma_altimeter);
          profiler_.stop("baro_altimeter_update");
        }
      }
    }
  }
}
void GnssXRioRos::iterateRadarTrigger()
{
  if (!queue_radar_trigger_.empty())
  {
    if (!initialized_ || !config_.radar_update)
    {
      queue_radar_trigger_.pop();
    }
    else
    {
      const auto trigger = queue_radar_trigger_.front();

      const auto radar_trigger_msg = trigger.second;
      const auto time_diff_filter  = gnss_x_rio_filter_.getTimestamp().toSec() - radar_trigger_msg.stamp.toSec();
      const auto radar_trigger_to_clone_delay = 0.5 * config_.radar_frame_ms / 1.0e3;

      if (std::fabs(time_diff_filter) <= imu_data_.dt / 2)
      {
        ROS_DEBUG_STREAM(kStreamingPrefix << "Received radar trigger with time stamp: " << radar_trigger_msg.stamp
                                          << " start to collect angular velocity measurement.");
        radar_w_queue_.clear();
        radar_w_queue_.emplace_back(imu_data_);

        // catch corner case
        if (radar_trigger_to_clone_delay < imu_data_.dt)
        {
          ROS_DEBUG_STREAM(kStreamingPrefix << "Adding radar clone with trigger time stamp: " << radar_trigger_msg.stamp
                                            << " at filter time " << gnss_x_rio_filter_.getTimestamp());
          gnss_x_rio_filter_.addRadarStateClone(trigger.first, radar_trigger_msg.stamp);
          queue_radar_trigger_.pop();
        }
      }
      else if (std::fabs(time_diff_filter - radar_trigger_to_clone_delay) < imu_data_.dt)
      {
        // compensate for the radar frame time ("exposure time" of the radar scan) --> center clone on radar scan
        ROS_DEBUG_STREAM(kStreamingPrefix << "Adding radar clone with trigger time stamp: " << radar_trigger_msg.stamp
                                          << " at filter time " << gnss_x_rio_filter_.getTimestamp());
        gnss_x_rio_filter_.addRadarStateClone(trigger.first, radar_trigger_msg.stamp);
        queue_radar_trigger_.pop();
      }
      else if (time_diff_filter - radar_trigger_to_clone_delay > imu_data_.dt)
      {
        ROS_ERROR_STREAM(kStreamingPrefix << "Radar trigger too old, " << time_diff_filter * 1.0e3
                                          << "ms older than current filter state, reject trigger!");
        queue_radar_trigger_.pop();
      }
    }
  }
}
void GnssXRioRos::iterateRadarScan()
{
  if (!queue_radar_.empty())
  {
    if (!initialized_ || !config_.radar_update)
    {
      queue_radar_.pop();
    }
    else
    {
      if (queue_radar_.size() > radar_frame_ids_.size())
        ROS_ERROR_STREAM(kStreamingPrefix << "Radar data queue size > number of radars: " << queue_radar_.size()
                                          << " this should not happen!");

      const auto radar_data     = queue_radar_.front();
      const auto radar_data_msg = radar_data.second;

      if (radar_frame_ids_.at(radar_data.first).empty())
        radar_frame_ids_.at(radar_data.first) = radar_data_msg.header.frame_id;

      if (gnss_x_rio_filter_.getTimestamp().toSec() + config_.radar_frame_ms / 1.0e3 >=
          radar_data_msg.header.stamp.toSec())
      {
        if (gnss_x_rio_filter_.getTimestamp().toSec() - radar_data_msg.header.stamp.toSec() > 1.0 / config_.radar_rate)
        {
          ROS_ERROR_STREAM(kStreamingPrefix << "Did not receive a trigger for the radar data. Rejecting!");
          queue_radar_.pop();
        }
        else
        {
          FullRadarCloneState radar_clone_state;
          uint clone_state_id;
          if (gnss_x_rio_filter_.getFullRadarState(radar_data.first, radar_clone_state, clone_state_id))
          {
            const auto time_diff_clone =
                radar_data_msg.header.stamp.toSec() - radar_clone_state.trigger_time_stamp.toSec();

            if (time_diff_clone < -20.0e-3)
              queue_radar_.pop();
            else if (time_diff_clone > -20.0e-3 && time_diff_clone < 1.0 / config_.radar_rate)
            {
              queue_radar_.pop();

              Vector3 w_mean(0, 0, 0);
              for (const auto& imu : radar_w_queue_) w_mean += imu.w_b_ib;
              w_mean /= radar_w_queue_.size();

              Matrix3 P_r;
              Vector3 v_r;
              profiler_.start("estimate_radar_velocity");

              const Matrix3 C_stab_r = radar_clone_state.nav_sol.getC_n_b() * radar_clone_state.T_b_r.linear();

              pcl::PointCloud<reve::RadarPointCloudType> inlier_radar_scan;
              if (radar_ego_velocity_.estimate(radar_data_msg, v_r, P_r, inlier_radar_scan, C_stab_r))
              {
                profiler_.stop("estimate_radar_velocity");
                profiler_.start("radar_velocity_kf_update");

                if (config_.use_diagonal_noise)
                {
                  P_r = Vector3(P_r(0, 0), P_r(1, 1), P_r(2, 2)).asDiagonal();
                }

                // adaptive threshold --> very low on ZUPTs
                const Real outlier_percentil_radar =
                    (v_r.norm() > 0.05) ? config_.outlier_percentil_radar : config_.outlier_percentil_radar / 100.0;

                bool valid = false;
                if (config_.use_w)
                  valid = gnss_x_rio_filter_.updateRadarEgoVelocity(
                      radar_data.first, v_r, P_r, w_mean, outlier_percentil_radar);
                else
                  valid = gnss_x_rio_filter_.updateRadarEgoVelocity(
                      radar_data.first, v_r, P_r, Vector3(0, 0, 0), outlier_percentil_radar);
                profiler_.stop("radar_velocity_kf_update");

                sensor_msgs::PointCloud2 inlier_radar_scan_msg;
                pclToPcl2msg(inlier_radar_scan, inlier_radar_scan_msg);
                inlier_radar_scan_msg.header = radar_data_msg.header;

                pubs_radar_scan_inlier_.at(radar_data.first).publish(inlier_radar_scan_msg);

                auto T_n_r = std::shared_ptr<Isometry>(
                    new Isometry(radar_clone_state.nav_sol.getPose() * radar_clone_state.T_b_r));

                radar_inlier_pcls_.at(radar_data.first) = {T_n_r, inlier_radar_scan};

                if (config_.radar_yaw_update)
                {
                  // reset yaw if good gnss
                  // TODO make parameters!
                  if ((radar_data_msg.header.stamp - last_valid_gps_).toSec() < 1.0 &&
                      (last_valid_gps_ - last_invalid_gps_).toSec() > 30.)
                  {
                    ROS_WARN_STREAM(kStreamingPrefix << "Reseting yaw aiding!");
                    yaw_aiding_manhattan_world_.reset();
                  }

                  if ((radar_data_msg.header.stamp - last_valid_gps_).toSec() > 1.0)
                  {
                    Real yaw_m;
                    sensor_msgs::PointCloud2 yaw_inlier_pcl_msg;
                    bool yaw_estimation_successful = yaw_aiding_manhattan_world_.update(
                        inlier_radar_scan_msg, radar_clone_state, yaw_m, yaw_inlier_pcl_msg);
                    if (yaw_estimation_successful)
                    {
                      bool yaw_inlier = gnss_x_rio_filter_.updateYaw(radar_data.first,
                                                                     yaw_m,
                                                                     angles::from_degrees(config_.sigma_radar_yaw_deg),
                                                                     config_.outlier_percentil_radar_yaw);

                      if (yaw_inlier)
                      {
                        pub_radar_yaw_inlier_.publish(yaw_inlier_pcl_msg);
                      }
                    }
                  }
                }
              }

              gnss_x_rio_filter_.removeClone(clone_state_id);

              // publish combined radar scan --> if id == id_max
              if (radar_data.first == radar_inlier_pcls_.size() - 1)
              {
                pcl::PointCloud<reve::RadarPointCloudType> combined_cloud;
                for (uint k = 0; k < radar_inlier_pcls_.size(); ++k)
                {
                  auto& inlier_cloud = radar_inlier_pcls_.at(k).second;
                  if (inlier_cloud.size() > 0)
                  {
                    std::shared_ptr<RadarClone> radar_clone;

                    const Isometry T_n_r = *radar_inlier_pcls_.at(k).first;
                    const Isometry T_b_r = gnss_x_rio_filter_.getNavigationSolution().getPose().inverse() * T_n_r;
                    for (const auto& p : inlier_cloud)
                    {
                      const Vector3 p_r(p.x, p.y, p.z);
                      const Vector3 p_b_now = T_b_r * p_r;

                      reve::RadarPointCloudType p_body = p;
                      p_body.x                         = p_b_now.x();
                      p_body.y                         = p_b_now.y();
                      p_body.z                         = p_b_now.z();
                      combined_cloud.push_back(p_body);
                    }
                  }
                  inlier_cloud.clear();
                }
                sensor_msgs::PointCloud2 combined_cloud_msg;
                pclToPcl2msg(combined_cloud, combined_cloud_msg);
                combined_cloud_msg.header.stamp    = gnss_x_rio_filter_.getTimestamp();
                combined_cloud_msg.header.frame_id = body_frame_id_;
                pub_combined_radar_scan_.publish(combined_cloud_msg);
              }
            }
            else
            {
              ROS_DEBUG_STREAM(kStreamingPrefix << "Did not process radar data due to time diff: " << time_diff_clone);
            }
          }
        }
      }
    }
  }
}

void GnssXRioRos::iterateRadarSim()
{
  if (!queue_v_r_sim_.empty())
  {
    if (!initialized_ || !config_.radar_update)
    {
      queue_v_r_sim_.pop();
    }
    else
    {
      const auto v_r_sim     = queue_v_r_sim_.front();
      const auto v_r_sim_mgs = v_r_sim.second;

      if (radar_frame_ids_.at(v_r_sim.first).empty())
        radar_frame_ids_.at(v_r_sim.first) = v_r_sim_mgs.header.frame_id;

      if (gnss_x_rio_filter_.getTimestamp().toSec() + config_.radar_frame_ms / 1.0e3 >=
          v_r_sim_mgs.header.stamp.toSec())
      {
        if (gnss_x_rio_filter_.getTimestamp().toSec() - v_r_sim_mgs.header.stamp.toSec() > 1.0 / config_.radar_rate)
        {
          ROS_ERROR_STREAM(kStreamingPrefix << "Did not received a trigger for the radar data. Rejecting!");
          queue_v_r_sim_.pop();
        }
        else
        {
          FullRadarCloneState radar_clone_state;
          uint clone_state_id;
          if (gnss_x_rio_filter_.getFullRadarState(v_r_sim.first, radar_clone_state, clone_state_id))
          {
            const auto time_diff_clone =
                v_r_sim_mgs.header.stamp.toSec() - radar_clone_state.trigger_time_stamp.toSec();

            if (time_diff_clone < -20.0e-3)
              queue_v_r_sim_.pop();
            else if (time_diff_clone > -20.0e-3 && time_diff_clone < 1.0 / config_.radar_rate)
            {
              queue_v_r_sim_.pop();

              Vector3 w_mean(0, 0, 0);
              for (const auto& imu : radar_w_queue_) w_mean += imu.w_b_ib;
              w_mean /= radar_w_queue_.size();

              Vector3 v_r(v_r_sim_mgs.twist.linear.x, v_r_sim_mgs.twist.linear.y, v_r_sim_mgs.twist.linear.z);
              if (config_.generate_radar_noise)
                v_r += Vector3(noise_v_r_x_(noise_gen_), noise_v_r_y_(noise_gen_), noise_v_r_z_(noise_gen_));

              const Vector3 sigma_r = Vector3(config_.sim_sigma_v_r_x + config_.sigma_offset_radar_x,
                                              config_.sim_sigma_v_r_y + config_.sigma_offset_radar_y,
                                              config_.sim_sigma_v_r_z + config_.sigma_offset_radar_z)
                                          .array()
                                          .square();
              const Matrix3 P_r = sigma_r.asDiagonal();

              profiler_.start("radar_velocity_kf_update");

              bool valid = false;
              if (config_.use_w)
                valid = gnss_x_rio_filter_.updateRadarEgoVelocity(
                    v_r_sim.first, v_r, P_r, w_mean, config_.outlier_percentil_radar);
              else
                valid = gnss_x_rio_filter_.updateRadarEgoVelocity(
                    v_r_sim.first, v_r, P_r, Vector3(0, 0, 0), config_.outlier_percentil_radar);
              gnss_x_rio_filter_.removeClone(clone_state_id);
              profiler_.stop("radar_velocity_kf_update");
            }
          }
        }
      }
    }
  }
}

void GnssXRioRos::iterateGpsValid()
{
  if (!queue_gnss_valid_.empty())
  {
    if (!initialized_ || !(config_.gnss_pos_update || config_.gnss_vel_update))
    {
      queue_gnss_valid_.pop();
    }
    else
    {
      const auto valid = queue_gnss_valid_.front();

      if (gnss_x_rio_filter_.getTimestamp() >= valid.stamp)
      {
        gnss_x_rio_filter_.addGnssStateClone(valid.stamp);
        queue_gnss_valid_.pop();
      }
    }
  }
}

Matrix3 getRneFromLlnh(double aLat, double aLon)
{
  Eigen::Matrix3d Rne;

  double sinLat = sin(aLat);
  double cosLat = cos(aLat);
  double sinLon = sin(aLon);
  double cosLon = cos(aLon);

  Rne(0, 0) = -sinLat * cosLon;
  Rne(0, 1) = -sinLat * sinLon;
  Rne(0, 2) = cosLat;
  Rne(1, 0) = -sinLon;
  Rne(1, 1) = cosLon;
  Rne(1, 2) = 0.0;
  Rne(2, 0) = -cosLat * cosLon;
  Rne(2, 1) = -cosLat * sinLon;
  Rne(2, 2) = -sinLat;

  return Rne;
}

void GnssXRioRos::iterateGps()
{
  if (!queue_gnss_measurement_.empty())
  {
    if (!gnss_initialized)
    {
      gnss_init_buffer_.emplace_back(queue_gnss_measurement_.front());
      queue_gnss_measurement_.pop();
      if (initialized_)
        initGnss();
    }
    else
    {
      if (gnss_x_rio_filter_.getTimestamp() >= queue_gnss_measurement_.front().header.stamp)
      {
        // fallback gnss state (if not valid received or availble)
        GnssCloneState gnss_state;
        gnss_state.p_n_b              = gnss_x_rio_filter_.getNavigationSolution().getPosition_n_b();
        gnss_state.v_n_b              = gnss_x_rio_filter_.getNavigationSolution().v_n_b;
        gnss_state.time_stamp         = queue_gnss_measurement_.front().header.stamp;
        gnss_state.trigger_time_stamp = queue_gnss_measurement_.front().header.stamp;

        GnssClone gnss_clone(0, gnss_state);
        if (!gnss_x_rio_filter_.getGnssClone(gnss_clone))
        {
          // TODO make parameter error message only if running with GNSS valid
          ROS_WARN_STREAM(kStreamingPrefix << "No GNSS clone available using the current system state...");
        }

        const auto gnss_measurement = queue_gnss_measurement_.front();
        queue_gnss_measurement_.pop();

        ROS_DEBUG("Gnss delay: %0.3f",
                  gnss_measurement.header.stamp.toSec() - gnss_clone.getGnssCloneState().trigger_time_stamp.toSec());

        // make sure the clone and the measurement matches
        // TODO make parameter GNSS rate
        if (std::fabs(gnss_measurement.header.stamp.toSec() -
                      gnss_clone.getGnssCloneState().trigger_time_stamp.toSec()) > 100.e-3)
        {
          ROS_ERROR_STREAM(kStreamingPrefix << "GNSS clone timestamp and gnss measurement does not match, skipping "
                                               "GNSS measurement!");
          gnss_x_rio_filter_.removeClone(gnss_clone.getCloneStateId());
          return;
        }

        const Vector3 m_llnh(gnss_measurement.pos_llnh[0], gnss_measurement.pos_llnh[1], gnss_measurement.pos_llnh[2]);
        const Matrix R_n_e = getRneFromLlnh(m_llnh(0), m_llnh(1));
        const Vector3 m_ecef(gnss_measurement.pos_ecef[0], gnss_measurement.pos_ecef[1], gnss_measurement.pos_ecef[2]);
        const Vector3 dpos_ecef = m_ecef - local_origin_ecef_;
        const Vector3 dpos_n    = R_n_e * dpos_ecef;

        if (gnss_measurement.pos_valid && gnss_measurement.vel_valid &&
            gnss_x_rio_filter_.getNavigationSolution().v_n_b.norm() > 0.25)
        {
          uint pos_inlier = 0;
          const bool gnss_state_valid =
              gnss_state_.gnssStateValid(gnss_measurement.header.stamp,
                                         gnss_measurement.pos_accuracy,
                                         gnss_x_rio_filter_.getNavigationSolution().getPosition_n_b());
          if (config_.gnss_pos_update)
          {
            if (gnss_state_valid)
            {
              const Vector3 sigma_p(gnss_measurement.pos_accuracy * config_.sigma_pos_scaling,
                                    gnss_measurement.pos_accuracy * config_.sigma_pos_scaling,
                                    gnss_measurement.pos_accuracy * config_.sigma_pos_scaling);
              if (sigma_p.x() < config_.max_pos_acc_thresh && gnss_measurement.num_satellites >= config_.min_n_sat &&
                  gnss_x_rio_filter_.updateGnssPosition(
                      gnss_clone, dpos_n, sigma_p, config_.outlier_percentil_gnss_pos))
              {
                ROS_DEBUG_STREAM(kStreamingPrefix << "GNSS Pos inlier");
                pos_inlier = 3;
              }
              else
              {
                ROS_INFO_STREAM(kStreamingPrefix << "GNSS Pos outlier");
                pos_inlier = 2;
              }
            }
            else
            {
              pos_inlier = 1;
            }
          }

          uint vel_inlier = 0;

          if (config_.gnss_vel_update)
          {
            if (gnss_state_valid)
            {
              const Vector3 v_ecef(
                  gnss_measurement.vel_ecef[0], gnss_measurement.vel_ecef[1], gnss_measurement.vel_ecef[2]);
              const Vector3 v_n = R_n_e * v_ecef;
              const Vector3 sigma_v(
                  gnss_measurement.vel_accuracy, gnss_measurement.vel_accuracy, gnss_measurement.vel_accuracy);
              if (sigma_v.x() < config_.max_vel_acc_thresh && gnss_measurement.num_satellites >= config_.min_n_sat &&
                  gnss_x_rio_filter_.updateGnssVelocity(gnss_clone, v_n, sigma_v, config_.outlier_percentil_gnss_vel))
              {
                ROS_DEBUG_STREAM(kStreamingPrefix << "GNSS vel inlier");
                vel_inlier = 3;
              }
              else
              {
                ROS_INFO_STREAM(kStreamingPrefix << "GNSS vel outlier");
                vel_inlier = 2;
              }
            }
            else
            {
              vel_inlier = 1;
            }
          }

          if (pos_inlier == 3 && vel_inlier == 3)
            last_valid_gps_ = gnss_measurement.header.stamp;
          else
            last_invalid_gps_ = gnss_measurement.header.stamp;

          GnssPointCloudType gnss_point;
          gnss_point.x            = dpos_n.y();
          gnss_point.y            = dpos_n.x();
          gnss_point.z            = -dpos_n.z();
          gnss_point.pos_inlier   = pos_inlier;
          gnss_point.vel_inlier   = vel_inlier;
          gnss_point.pos_accuracy = gnss_measurement.pos_accuracy;
          gnss_point.vel_accuracy = gnss_measurement.vel_accuracy;
          gnss_point.n_satelites  = gnss_measurement.num_satellites;

          gnss_point_cloud_.push_back(gnss_point);
        }
        gnss_x_rio_filter_.removeClone(gnss_clone.getCloneStateId());
      }
    }
  }
}

void GnssXRioRos::reconfigureCallback(gnss_x_rio::GnssXRioConfig& config, uint32_t level)
{
  configure(config);
  config_ = config;
}

void GnssXRioRos::callbackIMU(const sensor_msgs::ImuConstPtr& imu_msg)
{
  mutex_.lock();
  Real dt = 2.4e-3;
  if (std::fabs(last_imu_.dt) > 1.0e-6)
    dt = (imu_msg->header.stamp - last_imu_.time_stamp).toSec();

  if (dt < 0)
  {
    ROS_ERROR_STREAM(kStreamingPrefix << "Got negative imu dt, skip measurement!");
  }
  else
  {
    if (dt > 50.0e-3)
    {
      ROS_ERROR_STREAM(kStreamingPrefix << "Too large IMU dt: " << dt << " limit to 50ms!");
      dt = 50.0e-3;
    }
    else
    {
      last_imu_ = ImuDataStamped(imu_msg, dt);

      if (config_.sim_mode && config_.generate_imu_noise)
      {
        last_imu_.a_b_ib +=
            Vector3(noise_acc_(noise_gen_), noise_acc_(noise_gen_), noise_acc_(noise_gen_)) + sim_acc_offset_;
        last_imu_.w_b_ib += Vector3(noise_gyro_(noise_gen_), noise_gyro_(noise_gen_), noise_gyro_(noise_gen_));
      }

      queue_imu_.push(last_imu_);
    }
  }
  mutex_.unlock();
}

void GnssXRioRos::callbackBaroAltimter(const sensor_msgs::FluidPressureConstPtr& baro_msg)
{
  mutex_.lock();
  queue_baro_.push(*baro_msg);
  mutex_.unlock();
}

void GnssXRioRos::callbackRadarScan(const uint id, const sensor_msgs::PointCloud2ConstPtr& radar_msg)
{
  mutex_.lock();
  queue_radar_.push({id, *radar_msg});
  mutex_.unlock();
}

void GnssXRioRos::callbackRadarTrigger(const uint id, const std_msgs::HeaderConstPtr& trigger_msg)
{
  mutex_.lock();
  queue_radar_trigger_.push({id, *trigger_msg});
  mutex_.unlock();
}

void GnssXRioRos::callbackGnssValid(const std_msgs::HeaderConstPtr& valid_msg)
{
  mutex_.lock();
  queue_gnss_valid_.push(*valid_msg);
  mutex_.unlock();
}

void GnssXRioRos::callbackGnssMeasurement(const gnss_x_rio::GnssMeasurementConstPtr& gnss_msg)
{
  mutex_.lock();
  if (gnss_msg->pos_valid && gnss_msg->vel_valid)
    queue_gnss_measurement_.push(*gnss_msg);
  mutex_.unlock();
}

void GnssXRioRos::publish()
{
  const NavigationSolution nav_sol = gnss_x_rio_filter_.getNavigationSolution();
  const Isometry pose_ros          = nav_sol.getPoseRos();
  const Matrix covariance          = gnss_x_rio_filter_.getCovarianceMatrix();
  const auto state_idx             = gnss_x_rio_filter_.getErrorIdx();

  // pose
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp    = gnss_x_rio_filter_.getTimestamp();
  pose_stamped.header.frame_id = config_.frame_id;
  pose_stamped.pose            = tf2::toMsg(pose_ros);
  pub_pose_.publish(pose_stamped);

  // twist
  geometry_msgs::TwistStamped twist_stamped;
  twist_stamped.header = pose_stamped.header;
  tf2::toMsg(nav_sol.getVelocityRos(), twist_stamped.twist.linear);
  tf2::toMsg(pose_ros.linear() * (imu_data_.w_b_ib - gnss_x_rio_filter_.getBias().gyro), twist_stamped.twist.angular);
  pub_twist_.publish(twist_stamped);

  // odom
  nav_msgs::Odometry odom;
  odom.header                 = pose_stamped.header;
  odom.child_frame_id         = body_frame_id_;
  odom.pose.pose              = pose_stamped.pose;
  odom.pose.covariance.at(0)  = covariance(state_idx.position, state_idx.position);
  odom.pose.covariance.at(4)  = covariance(state_idx.position + 1, state_idx.position + 1);
  odom.pose.covariance.at(8)  = covariance(state_idx.position + 2, state_idx.position + 2);
  odom.twist.twist            = twist_stamped.twist;
  odom.twist.covariance.at(0) = covariance(state_idx.velocity, state_idx.velocity);
  odom.twist.covariance.at(4) = covariance(state_idx.velocity + 1, state_idx.velocity + 1);
  odom.twist.covariance.at(8) = covariance(state_idx.velocity + 2, state_idx.velocity + 2);
  pub_odometry_.publish(odom);

  // pose path
  pose_path_.poses.emplace_back(pose_stamped);
  if ((gnss_x_rio_filter_.getTimestamp() - last_timestamp_pose_pub_).toSec() > 1.0 / config_.pose_path_publisher_rate)
  {
    pose_path_.header = pose_stamped.header;
    pub_pose_path_.publish(pose_path_);
    last_timestamp_pose_pub_ = gnss_x_rio_filter_.getTimestamp();
  }

  // covariance
  pub_cov_.publish(msg_conversion::toCovMsg(gnss_x_rio_filter_, config_.frame_id));

  // filter state
  pub_nom_.publish(msg_conversion::toStateMsg(gnss_x_rio_filter_, config_.frame_id));

  // tf global -> body
  geometry_msgs::TransformStamped T_global_body;
  T_global_body                = tf2::eigenToTransform(pose_ros);
  T_global_body.header         = pose_stamped.header;
  T_global_body.child_frame_id = body_frame_id_;
  tf_broadcaster_.sendTransform(T_global_body);

  // tf body -> radar
  for (uint k = 0; k < radar_frame_ids_.size(); ++k)
  {
    if (radar_frame_ids_.at(k).size() > 0)
    {
      geometry_msgs::TransformStamped T_body_radar;
      T_body_radar                 = tf2::eigenToTransform(gnss_x_rio_filter_.getTbr(k));
      T_body_radar.header.stamp    = gnss_x_rio_filter_.getTimestamp();
      T_body_radar.header.frame_id = body_frame_id_;
      T_body_radar.child_frame_id  = radar_frame_ids_.at(k);
      tf_broadcaster_.sendTransform(T_body_radar);
    }
  }

  // local origin
  if (gnss_initialized)
  {
    sensor_msgs::NavSatFix local_origin;
    local_origin.header    = pose_stamped.header;
    local_origin.latitude  = local_origin_llh_.x();
    local_origin.longitude = local_origin_llh_.y();
    local_origin.altitude  = local_origin_llh_.z();
    pub_local_orign_.publish(local_origin);
  }

  // gnss point cloud
  sensor_msgs::PointCloud2 pcl_msg;
  pcl::PCLPointCloud2 tmp;
  pcl::toPCLPointCloud2(gnss_point_cloud_, tmp);
  pcl_conversions::fromPCL(tmp, pcl_msg);
  pcl_msg.header.stamp    = gnss_x_rio_filter_.getTimestamp();
  pcl_msg.header.frame_id = "local_enu";
  pub_gnss_pcl_.publish(pcl_msg);
}
void GnssXRioRos::printStats()
{
  const auto dataset_length  = (gnss_x_rio_filter_.getTimestamp() - filter_start_stamp_).toSec();
  const auto processing_time = (ros::WallTime::now() - filter_start_wall_time_).toSec();
  ROS_INFO_STREAM(kStreamingPrefix << "Analysis");

  std::cout << "Detailed runtimes:\n" << profiler_.toString() << std::endl;

  std::cout << "Runtime" << std::endl;
  printf("  Took %0.2fs to process %0.2fs --> %0.2f x realtime\n",
         processing_time,
         dataset_length,
         dataset_length / processing_time);

  printf("  Pure processing took %0.2fs --> %0.2f x realtime\n",
         profiler_.getTotalRuntime(),
         dataset_length / profiler_.getTotalRuntime());

  Real trajectory_length = 0.0;
  const uint step        = 50;
  for (uint k = 0; k < pose_path_.poses.size() - 2 * step - 1; k += step)
  {
    const auto p_0 = pose_path_.poses.at(k).pose.position;
    const auto p_1 = pose_path_.poses.at(k + step).pose.position;
    trajectory_length += Vector3(p_0.x - p_1.x, p_0.y - p_1.y, p_0.z - p_1.z).norm();
  }

  const auto idx = gnss_x_rio_filter_.getErrorIdx();
  const Matrix C = gnss_x_rio_filter_.getCovarianceMatrix();

  const Vector3 p_final   = gnss_x_rio_filter_.getNavigationSolution().getPosition_n_b();
  const Vector3 att_final = gnss_x_rio_filter_.getNavigationSolution().getEuler_n_b().to_degrees();

  const Vector3 p_0(pose_path_.poses.front().pose.position.x,
                    pose_path_.poses.front().pose.position.y,
                    pose_path_.poses.front().pose.position.z);

  const Quaternion q_0(pose_path_.poses.front().pose.orientation.w,
                       pose_path_.poses.front().pose.orientation.x,
                       pose_path_.poses.front().pose.orientation.y,
                       pose_path_.poses.front().pose.orientation.z);

  const Quaternion q_final_err = q_0.inverse() * gnss_x_rio_filter_.getNavigationSolution().getQuaternion_n_b();
  NavigationSolution q_err_tmp;
  q_err_tmp.setQuaternion(q_final_err);

  const Vector3 p_error = p_final - p_0;

  // clang-format off
  std::cout << "Evaluation (assuming the start and end pose are equal:" << std::endl;
  printf("  Trajectory length: %0.2fm\n", trajectory_length);
  printf("  Final pose: %0.2f m, %0.2f m, %0.2f m, %0.2f deg, %0.2f deg, %0.2f deg\n",
           p_final.x(), p_final.y(), p_final.z(),att_final.x(), att_final.y(), att_final.z());

  printf("  Final 3-Sigma Position: %0.2fm, %0.2fm, %0.2fm\n",
           3 * std::sqrt(C(idx.position, idx.position)),
           3 * std::sqrt(C(idx.position + 1, idx.position + 1)),
           3 * std::sqrt(C(idx.position + 2, idx.position + 2)));

  printf("  Position Error 3D: %0.2f m, %0.2f m, %0.2f m -> %0.2f m -> %0.2f percent\n",
           p_error.x(), p_error.y(), p_error.z(), p_error.norm(), p_error.norm() / trajectory_length * 100.0);

  printf("  Position Error 2D: %0.2f m, %0.2f m -> %0.2f m -> %0.2f percent\n",
           p_error.x(), p_error.y(), p_error.head(2).norm(), p_error.head(2).norm() / trajectory_length * 100.0);

  printf("  Final 3-Sigma Yaw: %0.2fdeg\n",
         3 * angles::to_degrees(std::sqrt(C(idx.attitude + 2, idx.attitude + 2))));

  printf("  Attitude Error: %0.2fdeg\n\n", config_.yaw_0_deg - att_final.z());
  // clang-format on
}
