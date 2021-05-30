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

#include <sensor_msgs/Imu.h>
#include <rio_utils/data_types.h>
#include <rio_utils/strapdown.h>

#include <ekf_yrio/EkfYRioConfig.h>
#include <ekf_rio/ekf_rio_filter.h>

namespace rio
{
/**
 * @brief The EkfYRioFilter class provides the EKF based Yaw aided Radar Inertial Odometry Filter
 * \note Uses the North East Down (NED) convention --> the z-axis point downwards
 */
class EkfYRioFilter : public EkfRioFilter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Reconfigure callback, enables online reconfigure using rqt_reconfigure
   * @param config   must contain the members of EkfRioConfig
   */
  template <class ConfigContainingEkfRioConfig>
  bool configure(ConfigContainingEkfRioConfig& config);

  /**
   * @brief Implements an Extended Kalman Filter update
   * @param r       observed residuum
   * @param H       measurement jacobian
   * @param R_diag  diagonal elements of the measurement noise matrix
   * @returns true if successful
   */
  bool kfUpdate(const Vector& r, const Matrix& H, const Vector& R_diag) { return EkfRioFilter::kfUpdate(r, H, R_diag); }

protected:
  std::string kStreamingPrefix = "[EkfRioFilter]: ";
};

template <typename ConfigContainingEkfRioConfig>
bool EkfYRioFilter::configure(ConfigContainingEkfRioConfig& config)
{
  bool success = true;
  success |= EkfRioFilter::configure(config);
  return success;
}

}  // namespace rio
