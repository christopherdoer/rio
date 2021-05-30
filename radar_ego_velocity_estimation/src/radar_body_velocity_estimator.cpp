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

#include <random>
#include <algorithm>
#include <angles/angles.h>

#include <rio_utils/ros_helper.h>
#include <rio_utils/math_helper.h>
#include <radar_ego_velocity_estimation/radar_body_velocity_estimator.h>

using namespace rio;

RadarBodyVelocityEstimator::RadarBodyVelocityEstimator(ros::NodeHandle nh)
{
  Vector3 l_b_r;
  getRosParameter(nh, kPrefix, RosParameterType::Required, "l_b_r_x", l_b_r.x());
  getRosParameter(nh, kPrefix, RosParameterType::Required, "l_b_r_y", l_b_r.y());
  getRosParameter(nh, kPrefix, RosParameterType::Required, "l_b_r_z", l_b_r.z());

  Quaternion q_b_r;
  getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_w", q_b_r.w());
  getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_x", q_b_r.x());
  getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_y", q_b_r.y());
  getRosParameter(nh, kPrefix, RosParameterType::Required, "q_b_r_z", q_b_r.z());

  T_b_r_.translation() = l_b_r;
  T_b_r_.linear()      = Matrix3(q_b_r);
}

bool RadarBodyVelocityEstimator::estimate(const sensor_msgs::PointCloud2& radar_scan_msg,
                                          const Vector3& w_b,
                                          Vector3& v_b_r,
                                          Matrix3& P_v_b)
{
  Vector3 v_r;
  Vector3 sigma_v_r;

  if (radar_ego_velocity_estimator_.estimate(radar_scan_msg, v_r, sigma_v_r))
  {
    // v_b & sigma_v_b
    const Vector3 v_b_w = math_helper::skewVec(w_b) * T_b_r_.translation();
    v_b_r               = T_b_r_.linear() * v_r - v_b_w;
    P_v_b = T_b_r_.linear() * Vector3(sigma_v_r.array().square()).asDiagonal() * T_b_r_.linear().transpose();

    return true;
  }

  return false;
}
