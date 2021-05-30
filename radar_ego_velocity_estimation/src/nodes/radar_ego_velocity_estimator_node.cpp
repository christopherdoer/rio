// This file is part of RIO - Radar Inertial Odometry and Radar ego velocity estimation.
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>
// (Institute of Control Systems, Karlsruhe Institute of Technology)

// RIO is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.

// RIO is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with RIO.  If not, see <https://www.gnu.org/licenses/>.

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <rio_utils/simple_profiler.h>
#include <rio_utils/ros_helper.h>

#include <radar_ego_velocity_estimation/RadarEgoVelocityEstimatorConfig.h>
#include <radar_ego_velocity_estimation/radar_ego_velocity_estimator.h>

using namespace rio;

const std::string kNodeName = "radar_ego_velocity_node";
const std::string kPrefix   = "[" + kNodeName + "]: ";

static RadarEgoVelocityEstimator estimator;
static ros::Publisher pub_twist;
static SimpleProfiler profiler;

void reconfigureCallback(radar_ego_velocity_estimation::RadarEgoVelocityEstimatorConfig& config, uint32_t level)
{
  estimator.configure(config);
}

void callbackRadarScan(const sensor_msgs::PointCloud2ConstPtr& radar_msg)
{
  Vector3 v_r, sigma_v_r;
  profiler.start("estimate");
  if (estimator.estimate(*radar_msg, v_r, sigma_v_r))
  {
    profiler.stop("estimate");

    geometry_msgs::TwistWithCovarianceStamped twist;
    twist.header.stamp         = radar_msg->header.stamp;
    twist.twist.twist.linear.x = v_r.x();
    twist.twist.twist.linear.y = v_r.y();
    twist.twist.twist.linear.z = v_r.z();

    twist.twist.covariance.at(0)  = std::pow(sigma_v_r.x(), 2);
    twist.twist.covariance.at(7)  = std::pow(sigma_v_r.y(), 2);
    twist.twist.covariance.at(14) = std::pow(sigma_v_r.z(), 2);

    pub_twist.publish(twist);
  }
  else
  {
    profiler.stop("estimate");
  }

  ROS_INFO_THROTTLE(
      5, "%s Runtime statistics: %s", kPrefix.c_str(), profiler.getStatistics("estimate").toStringMs().c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, kNodeName);
  ros::NodeHandle nh("~");

  dynamic_reconfigure::Server<radar_ego_velocity_estimation::RadarEgoVelocityEstimatorConfig> reconfigure_server;
  reconfigure_server.setCallback(&reconfigureCallback);

  std::string topic_twist = "twist";
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "topic_twist", topic_twist);
  pub_twist = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(topic_twist, 5);

  std::string topic_radar_scan = "/sensor_platform/radar/scan";
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "topic_radar_scan", topic_radar_scan);
  ros::Subscriber sub_radar_data = nh.subscribe<sensor_msgs::PointCloud2>(topic_radar_scan, 50, &callbackRadarScan);

  ros::spin();

  return 0;
}
