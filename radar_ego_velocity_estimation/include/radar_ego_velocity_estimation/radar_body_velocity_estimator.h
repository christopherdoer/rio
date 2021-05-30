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

#pragma once

#include <angles/angles.h>
#include <sensor_msgs/PointCloud2.h>

#include <rio_utils/data_types.h>

#include <radar_ego_velocity_estimation/radar_ego_velocity_estimator.h>

namespace rio
{
/**
 * @brief The RadarBodyVelocityEstimator class
 */
class RadarBodyVelocityEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   * @brief RadarBodyVelocityEstimator constructor
   * @param[in] nh   Node handle
   */
  RadarBodyVelocityEstimator(ros::NodeHandle nh);

  /**
   * @brief Estimates the body velocity based on the radar scan and angular velocity
   * @param[in] radar_scan_msg   radar scan
   * @param[in] w_b              angular velocity observed during the radar scan
   * @param[out] v_b_r           estimated 3D body velocity
   * @param[out] P_v_b           estimated covariance matrix
   * @returns true if estimation successful
   */
  bool estimate(const sensor_msgs::PointCloud2& radar_scan_msg, const Vector3& w_b, Vector3& v_b_r, Matrix3& P_v_b);

  /**
   * @brief Recofigure callback
   * @param[in/out] cfg  cfg has to contain the members of RadarEgoVelocityEstimatorConfig
   */
  template <class ConfigContainingRadarEgoVelocityEstimatorConfig>
  void configure(ConfigContainingRadarEgoVelocityEstimatorConfig& cfg)
  {
    radar_ego_velocity_estimator_.configure(cfg);
  }

private:
  const std::string kPrefix = "[RadarBodyVelocityEstimator]: ";

  RadarEgoVelocityEstimator radar_ego_velocity_estimator_;
  Isometry T_b_r_;
};

}  // namespace rio
