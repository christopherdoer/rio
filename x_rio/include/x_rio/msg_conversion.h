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

#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>

#include <rio_utils/data_types.h>

#include <x_rio/XRioState.h>
#include <x_rio/XRioCovariance.h>
#include <x_rio/x_rio_filter.h>

namespace rio
{
namespace msg_conversion
{
/**
 * @brief Creates an ekf_rio::EkfRioState message
 * @param ekf_rio_filter  filter
 * @param frame_id        frame_id of the global states
 * @returns an ekf_rio::EkfRioState
 */
static inline x_rio::XRioState toStateMsg(const XRioFilter& ekf_rio_filter,
                                                         const std::string& frame_id)
{
  x_rio::XRioState state_msg;

  state_msg.header.stamp    = ekf_rio_filter.getTimestamp();
  state_msg.header.frame_id = frame_id;

  const Quaternion q = ekf_rio_filter.getNavigationSolution().getQuaternion_n_b();
  tf2::Quaternion q_tf2(q.x(), q.y(), q.z(), q.w());
  Real roll, pitch, yaw;
  tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);

  tf2::toMsg(ekf_rio_filter.getNavigationSolution().getPosition_n_b(), state_msg.p);

  tf2::toMsg(ekf_rio_filter.getNavigationSolution().v_n_b, state_msg.v);

  state_msg.q = tf2::toMsg(q);
  tf2::toMsg(EulerAngles(roll, pitch, yaw).to_degrees(), state_msg.eul_deg);

  tf2::toMsg(ekf_rio_filter.getBias().acc, state_msg.b_a);

  tf2::toMsg(EulerAngles(ekf_rio_filter.getBias().gyro).to_degrees(), state_msg.b_g_deg);
  state_msg.b_alt = ekf_rio_filter.getBias().alt;

  for (uint id = 0; id < ekf_rio_filter.getErrorIdx().radar_extrinsics.size(); ++id)
  {
    const Quaternion q_b_r = Quaternion(ekf_rio_filter.getCbr(id));
    tf2::Quaternion q_b_r_tf2(q_b_r.x(), q_b_r.y(), q_b_r.z(), q_b_r.w());
    Real roll_b_r, pitch_b_r, yaw_b_r;
    tf2::Matrix3x3(q_b_r_tf2).getRPY(roll_b_r, pitch_b_r, yaw_b_r);

    state_msg.q_b_r.push_back(tf2::toMsg(q_b_r));

    geometry_msgs::Vector3 eul_b_r;
    tf2::toMsg(EulerAngles(roll_b_r, pitch_b_r, yaw_b_r).to_degrees(), eul_b_r);
    state_msg.eul_b_r_deg.push_back(eul_b_r);

    geometry_msgs::Vector3 l_b_r;
    tf2::toMsg(ekf_rio_filter.getlbr(id), l_b_r);
    state_msg.l_b_r.push_back(l_b_r);
  }
  return state_msg;
}

/**
 * @brief Creates an ekf_rio::EkfRioCovariance message
 * @param ekf_rio_filter  filter
 * @param frame_id        frame_id of the global states
 * @returns an ekf_rio::EkfRioCovariance
 */
static inline x_rio::XRioCovariance toCovMsg(const XRioFilter& ekf_rio_filter,
                                                            const std::string& frame_id)
{
  x_rio::XRioCovariance cov_msg;

  cov_msg.header.stamp    = ekf_rio_filter.getTimestamp();
  cov_msg.header.frame_id = frame_id;

  const Matrix C = ekf_rio_filter.getCovarianceMatrix();
  const auto idx = ekf_rio_filter.getErrorIdx();

  cov_msg.sigma_p.x = std::sqrt(C(idx.position, idx.position));
  cov_msg.sigma_p.y = std::sqrt(C(idx.position + 1, idx.position + 1));
  cov_msg.sigma_p.z = std::sqrt(C(idx.position + 2, idx.position + 2));

  cov_msg.sigma_v.x = std::sqrt(C(idx.velocity, idx.velocity));
  cov_msg.sigma_v.y = std::sqrt(C(idx.velocity + 1, idx.velocity + 1));
  cov_msg.sigma_v.z = std::sqrt(C(idx.velocity + 2, idx.velocity + 2));

  cov_msg.sigma_eul_deg.x = angles::to_degrees(std::sqrt(C(idx.attitude, idx.attitude)));
  cov_msg.sigma_eul_deg.y = angles::to_degrees(std::sqrt(C(idx.attitude + 1, idx.attitude + 1)));
  cov_msg.sigma_eul_deg.z = angles::to_degrees(std::sqrt(C(idx.attitude + 2, idx.attitude + 2)));

  cov_msg.sigma_b_a.x = std::sqrt(C(idx.bias_acc, idx.bias_acc));
  cov_msg.sigma_b_a.y = std::sqrt(C(idx.bias_acc + 1, idx.bias_acc + 1));
  cov_msg.sigma_b_a.z = std::sqrt(C(idx.bias_acc + 2, idx.bias_acc + 2));

  cov_msg.sigma_b_g_deg.x = angles::to_degrees(std::sqrt(C(idx.bias_gyro, idx.bias_gyro)));
  cov_msg.sigma_b_g_deg.y = angles::to_degrees(std::sqrt(C(idx.bias_gyro + 1, idx.bias_gyro + 1)));
  cov_msg.sigma_b_g_deg.z = angles::to_degrees(std::sqrt(C(idx.bias_gyro + 2, idx.bias_gyro + 2)));

  cov_msg.sigma_alt = std::sqrt(C(idx.bias_alt, idx.bias_alt));

  for (uint id = 0; id < ekf_rio_filter.getErrorIdx().radar_extrinsics.size(); ++id)
  {
    geometry_msgs::Vector3 sigma_l_b_r, sigma_eul_b_r_deg;

    sigma_l_b_r.x = std::sqrt(C(idx.radar_extrinsics.at(id).l_b_r, idx.radar_extrinsics.at(id).l_b_r));
    sigma_l_b_r.y = std::sqrt(C(idx.radar_extrinsics.at(id).l_b_r + 1, idx.radar_extrinsics.at(id).l_b_r + 1));
    sigma_l_b_r.z = std::sqrt(C(idx.radar_extrinsics.at(id).l_b_r + 2, idx.radar_extrinsics.at(id).l_b_r + 2));
    cov_msg.sigma_l_b_r.push_back(sigma_l_b_r);

    sigma_eul_b_r_deg.x =
        angles::to_degrees(std::sqrt(C(idx.radar_extrinsics.at(id).eul_b_r, idx.radar_extrinsics.at(id).eul_b_r)));
    sigma_eul_b_r_deg.y = angles::to_degrees(
        std::sqrt(C(idx.radar_extrinsics.at(id).eul_b_r + 1, idx.radar_extrinsics.at(id).eul_b_r + 1)));
    sigma_eul_b_r_deg.z = angles::to_degrees(
        std::sqrt(C(idx.radar_extrinsics.at(id).eul_b_r + 2, idx.radar_extrinsics.at(id).eul_b_r + 2)));
    cov_msg.sigma_eul_b_r_deg.push_back(sigma_eul_b_r_deg);
  }

  return cov_msg;
}

}  // namespace msg_conversion
}  // namespace rio
