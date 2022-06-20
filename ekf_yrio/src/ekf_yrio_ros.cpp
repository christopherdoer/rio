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

#include <math.h>
#include <numeric>

#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <rio_utils/ros_helper.h>
#include <rio_utils/math_helper.h>

#include <ekf_rio/EkfRioState.h>
#include <ekf_rio/EkfRioCovariance.h>
#include <ekf_rio/msg_conversion.h>

#include <ekf_yrio/ekf_yrio_ros.h>

using namespace rio;

EkfYRioRos::EkfYRioRos(ros::NodeHandle& nh) : yaw_aiding_manhattan_world_(nh), initialized_{false}
{
  reconfigure_server_.setCallback(boost::bind(&EkfYRioRos::reconfigureCallback, this, _1, _2));

  // subscribers
  sub_imu_  = nh.subscribe<sensor_msgs::Imu>(config_.topic_imu, 2, boost::bind(&EkfYRioRos::callbackIMU, this, _1));
  sub_baro_ = nh.subscribe<sensor_msgs::FluidPressure>(
      config_.topic_baro_altimeter, 2, boost::bind(&EkfYRioRos::callbackBaroAltimter, this, _1));
  sub_radar_ = nh.subscribe<sensor_msgs::PointCloud2>(
      config_.topic_radar_scan, 2, boost::bind(&EkfYRioRos::callbackRadarScan, this, _1));
  sub_radar_trigger_ = nh.subscribe<std_msgs::Header>(
      config_.topic_radar_trigger, 2, boost::bind(&EkfYRioRos::callbackRadarTrigger, this, _1));

  // publishers
  pub_cov_               = nh.advertise<ekf_rio::EkfRioCovariance>("covariance", 5);
  pub_nom_               = nh.advertise<ekf_rio::EkfRioState>("state", 5);
  pub_pose_path_         = nh.advertise<nav_msgs::Path>("pose_path", 1);
  pub_pose_              = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  pub_twist_             = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
  pub_radar_scan_inlier_ = nh.advertise<sensor_msgs::PointCloud2>("radar_scan_inlier", 10);
  pub_radar_yaw_inlier_  = nh.advertise<sensor_msgs::PointCloud2>("radar_scan_yaw_inlier", 1);

  if (config_.republish_ground_truth)
  {
    pub_ground_truth_pose_  = nh.advertise<geometry_msgs::PoseStamped>(config_.topic_ground_truth_pose, 1000);
    pub_ground_truth_twist_ = nh.advertise<geometry_msgs::TwistStamped>(config_.topic_ground_truth_twist, 1000);
    pub_ground_truth_twist_body_ =
        nh.advertise<geometry_msgs::TwistStamped>(config_.topic_ground_truth_twist_body, 1000);
  }

  ros::Duration(0.5).sleep();
}

bool EkfYRioRos::initImu(const ImuDataStamped& imu_data)
{
  imu_init_.emplace_back(imu_data);

  const auto T_init_so_far = (imu_init_.back().time_stamp - imu_init_.front().time_stamp).toSec();
  if (T_init_so_far > config_.T_init)
  {
    Real baro_h_0 = 0.0;
    if (config_.altimeter_update)
    {
      if (baro_init_vec_.size() > 0)
      {
        baro_h_0 = std::accumulate(baro_init_vec_.begin(), baro_init_vec_.end(), 0.0) / baro_init_vec_.size();
        ROS_INFO_STREAM(kStreamingPrefix << "Initialized baro h_0: " << baro_h_0);
        baro_initialized_ = true;
      }
      else
      {
        ROS_ERROR_STREAM(kStreamingPrefix << "Unable to init baro --> no measurements received!");
      }
    }

    initialized_            = ekf_yrio_filter_.init(imu_init_, baro_h_0);
    filter_start_stamp_     = ekf_yrio_filter_.getTimestamp();
    filter_start_wall_time_ = ros::WallTime::now();
  }
  else
  {
    const uint mod = 1.0 / imu_data.dt + 1;
    if (imu_init_.size() % mod == 0)
      ROS_INFO("%s Init progress: %0.2f / %0.2f seconds", kStreamingPrefix.c_str(), T_init_so_far, config_.T_init);
  }
  return initialized_;
}

void EkfYRioRos::run()
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

void EkfYRioRos::runFromRosbag(const std::string& rosbag_path,
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

  if (config_.republish_ground_truth)
  {
    topics.push_back(config_.topic_ground_truth_pose);
    topics.push_back(config_.topic_ground_truth_twist);
    topics.push_back(config_.topic_ground_truth_twist_body);
  }

  rosbag::View view(source_bag, rosbag::TopicQuery(topics));

  auto first_timestamp = ros::TIME_MIN;

  for (const rosbag::MessageInstance& m : view)
  {
    if (first_timestamp == ros::TIME_MIN)
      first_timestamp = m.getTime();

    if ((m.getTime() - first_timestamp).toSec() < bag_start)
      continue;

    if ((m.getTime() - first_timestamp).toSec() > bag_duration)
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
    else if (topic == config_.topic_radar_scan)
    {
      const auto radar_scan = m.instantiate<sensor_msgs::PointCloud2>();
      if (radar_scan != NULL)
        callbackRadarScan(radar_scan);

      if (sleep_ms > 0)
        ros::Duration(sleep_ms / 1.0e3).sleep();
    }
    else if (topic == config_.topic_radar_trigger)
    {
      const auto radar_trigger_msg = m.instantiate<std_msgs::Header>();
      if (radar_trigger_msg != NULL)
        callbackRadarTrigger(radar_trigger_msg);
    }
    else if (config_.republish_ground_truth)
    {
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
    }

    iterate();
    ros::spinOnce();
  }

  publish();

  printStats();
}

void EkfYRioRos::iterate()
{
  mutex_.lock();
  if (queue_imu_.size() > 0)
  {
    imu_data_ = queue_imu_.front();
    queue_imu_.pop();

    if (body_frame_id_.empty())
      body_frame_id_ = imu_data_.frame_id;

    if (!initialized_)
    {
      initImu(imu_data_);
    }
    else
    {
      profiler_.start("processIMU");
      if (!ekf_yrio_filter_.propagate(imu_data_))
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

  if (!queue_baro_.empty())
  {
    auto baro_msg = queue_baro_.front();
    if (!baro_initialized_)
    {
      baro_init_vec_.emplace_back(baro_altimeter_.calculate_rel_neg_height(baro_msg));
      queue_baro_.pop();
    }
    else if (ekf_yrio_filter_.getTimestamp() >= baro_msg.header.stamp)
    {
      queue_baro_.pop();
      if (config_.altimeter_update)
      {
        if (initialized_)
        {
          profiler_.start("baro_altimeter_update");
          const auto h_rel = baro_altimeter_.calculate_rel_neg_height(baro_msg);
          ekf_yrio_filter_.updateAltimeter(h_rel, config_.sigma_altimeter);
          profiler_.stop("baro_altimeter_update");
        }
      }
    }
  }

  if (!queue_radar_trigger_.empty())
  {
    if (!initialized_ || !config_.radar_update)
    {
      queue_radar_trigger_.pop();
    }
    else
    {
      auto radar_trigger_msg      = queue_radar_trigger_.front();
      const auto time_diff_filter = ekf_yrio_filter_.getTimestamp().toSec() - radar_trigger_msg.stamp.toSec();
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
                                            << " at filter time " << ekf_yrio_filter_.getTimestamp());
          ekf_yrio_filter_.addRadarStateClone(radar_trigger_msg.stamp);
          queue_radar_trigger_.pop();
        }
      }
      else if (std::fabs(time_diff_filter - radar_trigger_to_clone_delay) < imu_data_.dt)
      {
        // compensate for the radar frame time ("exposure time" of the radar scan) --> center clone on radar scan
        ROS_DEBUG_STREAM(kStreamingPrefix << "Adding radar clone with trigger time stamp: " << radar_trigger_msg.stamp
                                          << " at filter time " << ekf_yrio_filter_.getTimestamp());
        ekf_yrio_filter_.addRadarStateClone(radar_trigger_msg.stamp);
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

  if (!queue_radar_.empty())
  {
    if (radar_frame_id_.empty())
      radar_frame_id_ = queue_radar_.front().header.frame_id;

    if (!initialized_ || !config_.radar_update)
    {
      queue_radar_.pop();
    }
    else
    {
      if (queue_radar_.size() > 1)
      {
        if (queue_radar_.front().header.stamp == queue_radar_.back().header.stamp)
        {
          ROS_WARN_STREAM(kStreamingPrefix << "Got two radar scans with the same timestamp, dropping one.");
          queue_radar_.pop();
        }
        else
        {
          ROS_ERROR_STREAM(kStreamingPrefix << "Radar data queue size > 1: " << queue_radar_.size()
                                            << " this should not happen!");
        }
      }

      auto radar_data_msg = queue_radar_.front();

      if (ekf_yrio_filter_.getTimestamp().toSec() + config_.radar_frame_ms / 1.0e3 >=
          radar_data_msg.header.stamp.toSec())
      {
        if (ekf_yrio_filter_.getTimestamp().toSec() - radar_data_msg.header.stamp.toSec() > 1.0 / config_.radar_rate)
        {
          ROS_ERROR_STREAM(kStreamingPrefix << "Did not received a trigger for the radar data. Rejecting!");
          queue_radar_.pop();
        }
        else
        {
          const auto time_diff_clone =
              ekf_yrio_filter_.getRadarCloneState().trigger_time_stamp.toSec() - radar_data_msg.header.stamp.toSec();
          if (std::fabs(time_diff_clone) < 1.0 / config_.radar_rate)
          {
            queue_radar_.pop();

            Vector3 w_mean(0, 0, 0);
            for (const auto& imu : radar_w_queue_) w_mean += imu.w_b_ib;
            w_mean /= radar_w_queue_.size();

            Vector3 v_r, sigma_v_r;
            profiler_.start("estimate_radar_velocity");

            sensor_msgs::PointCloud2 inlier_radar_scan;
            if (radar_ego_velocity_.estimate(radar_data_msg, v_r, sigma_v_r, inlier_radar_scan))
            {
              profiler_.stop("estimate_radar_velocity");
              profiler_.start("radar_velocity_kf_update");

              bool valid = false;
              if (config_.use_w)
                valid =
                    ekf_yrio_filter_.updateRadarEgoVelocity(v_r, sigma_v_r, w_mean, config_.outlier_percentil_radar);
              else
                valid = ekf_yrio_filter_.updateRadarEgoVelocity(
                    v_r, sigma_v_r, Vector3(0, 0, 0), config_.outlier_percentil_radar);
              profiler_.stop("radar_velocity_kf_update");

              pub_radar_scan_inlier_.publish(inlier_radar_scan);

              if (config_.enable_yaw_aiding)
              {
                Real yaw_m;
                sensor_msgs::PointCloud2 yaw_inlier_pcl_msg;
                bool yaw_estimation_successful = yaw_aiding_manhattan_world_.update(
                    inlier_radar_scan, ekf_yrio_filter_.getRadarCloneState(), yaw_m, yaw_inlier_pcl_msg);
                if (yaw_estimation_successful)
                {
                  bool yaw_inlier = ekf_yrio_filter_.updateYaw(
                      yaw_m, angles::from_degrees(config_.sigma_radar_yaw_deg), config_.outlier_percentil_radar_yaw);
                  if (yaw_inlier)
                  {
                    pub_radar_yaw_inlier_.publish(yaw_inlier_pcl_msg);
                  }
                }
              }
            }
            ROS_DEBUG_STREAM(kStreamingPrefix << "Removing radar clone with time stamp: "
                                              << ekf_yrio_filter_.getRadarCloneState().time_stamp);
            ekf_yrio_filter_.removeRadarStateClone();
          }
          else
          {
            ROS_DEBUG_STREAM(kStreamingPrefix << "Did not process radar data due to time diff: " << time_diff_clone);
          }
        }
      }
    }
  }
  mutex_.unlock();
}

void EkfYRioRos::reconfigureCallback(ekf_yrio::EkfYRioConfig& config, uint32_t level)
{
  ekf_yrio_filter_.configure(config);
  radar_ego_velocity_.configure(config);
  yaw_aiding_manhattan_world_.configure(config);
  config_ = config;
}

void EkfYRioRos::callbackIMU(const sensor_msgs::ImuConstPtr& imu_msg)
{
  mutex_.lock();
  Real dt = 2.4e-3;
  if (std::fabs(last_imu_.dt) > 1.0e-6)
    dt = (imu_msg->header.stamp - last_imu_.time_stamp).toSec();
  last_imu_ = ImuDataStamped(imu_msg, dt);
  queue_imu_.push(last_imu_);
  mutex_.unlock();
}

void EkfYRioRos::callbackBaroAltimter(const sensor_msgs::FluidPressureConstPtr& baro_msg)
{
  mutex_.lock();
  queue_baro_.push(*baro_msg);
  mutex_.unlock();
}

void EkfYRioRos::callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_msg)
{
  mutex_.lock();
  queue_radar_.push(*radar_msg);
  mutex_.unlock();
}

void EkfYRioRos::callbackRadarTrigger(const std_msgs::HeaderConstPtr& trigger_msg)
{
  mutex_.lock();
  queue_radar_trigger_.push(*trigger_msg);
  mutex_.unlock();
}

void EkfYRioRos::publish()
{
  const NavigationSolution nav_sol = ekf_yrio_filter_.getNavigationSolution();
  const Isometry pose_ros          = nav_sol.getPoseRos();

  // pose
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp    = ekf_yrio_filter_.getTimestamp();
  pose_stamped.header.frame_id = config_.frame_id;
  pose_stamped.pose            = tf2::toMsg(pose_ros);
  pub_pose_.publish(pose_stamped);

  // twist
  geometry_msgs::TwistStamped twist_stamped;
  twist_stamped.header = pose_stamped.header;
  tf2::toMsg(nav_sol.getVelocityRos(), twist_stamped.twist.linear);
  tf2::toMsg(pose_ros.linear() * (imu_data_.w_b_ib - ekf_yrio_filter_.getBias().gyro), twist_stamped.twist.angular);
  pub_twist_.publish(twist_stamped);

  // pose path
  pose_path_.poses.emplace_back(pose_stamped);
  if ((ekf_yrio_filter_.getTimestamp() - last_timestamp_pose_pub_).toSec() > 1.0 / config_.pose_path_publisher_rate)
  {
    pose_path_.header = pose_stamped.header;
    pub_pose_path_.publish(pose_path_);
    last_timestamp_pose_pub_ = ekf_yrio_filter_.getTimestamp();
  }

  // covariance
  pub_cov_.publish(msg_conversion::toCovMsg(ekf_yrio_filter_, config_.frame_id));

  // filter state
  pub_nom_.publish(msg_conversion::toStateMsg(ekf_yrio_filter_, config_.frame_id));

  // tf global -> body
  geometry_msgs::TransformStamped T_global_body;
  T_global_body                = tf2::eigenToTransform(pose_ros);
  T_global_body.header         = pose_stamped.header;
  T_global_body.child_frame_id = body_frame_id_;
  tf_broadcaster_.sendTransform(T_global_body);

  // tf body -> radar
  geometry_msgs::TransformStamped T_body_radar;
  T_body_radar                 = tf2::eigenToTransform(ekf_yrio_filter_.getTbr());
  T_body_radar.header.stamp    = ekf_yrio_filter_.getTimestamp();
  T_body_radar.header.frame_id = body_frame_id_;
  T_body_radar.child_frame_id  = radar_frame_id_;
  tf_broadcaster_.sendTransform(T_body_radar);
}

void EkfYRioRos::printStats()
{
  const auto dataset_length  = (ekf_yrio_filter_.getTimestamp() - filter_start_stamp_).toSec();
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

  const auto idx = ekf_yrio_filter_.getErrorIdx();
  const Matrix C = ekf_yrio_filter_.getCovarianceMatrix();

  const Vector3 p_final   = ekf_yrio_filter_.getNavigationSolution().getPosition_n_b();
  const Vector3 att_final = ekf_yrio_filter_.getNavigationSolution().getEuler_n_b().to_degrees();

  const Vector3 p_0(pose_path_.poses.front().pose.position.x,
                    pose_path_.poses.front().pose.position.y,
                    pose_path_.poses.front().pose.position.z);

  const Quaternion q_0(pose_path_.poses.front().pose.orientation.w,
                       pose_path_.poses.front().pose.orientation.x,
                       pose_path_.poses.front().pose.orientation.y,
                       pose_path_.poses.front().pose.orientation.z);

  const Quaternion q_final_err = q_0.inverse() * ekf_yrio_filter_.getNavigationSolution().getQuaternion_n_b();
  NavigationSolution q_err_tmp;
  q_err_tmp.setQuaternion(q_final_err);

  const Vector3 p_error = p_final - p_0;

  // clang-format off
  std::cout << "Evaluation (assuming the start and end pose are equal) :" << std::endl;
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
