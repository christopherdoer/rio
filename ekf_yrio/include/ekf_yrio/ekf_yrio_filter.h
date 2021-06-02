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
#include <rio_utils/math_helper.h>

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
   * @brief Implements a global yaw angle update
   * @param yaw_m                     global yaw angle measurement
   * @param sigma_yaw                  noise of meausrement
   * @param outlier_rejection_thresh   reject measurement if below this percentil
   * @returns true if the measurement is an inlier
   */
  bool updateYaw(const Real& yaw_m, const Real& sigma_yaw, const Real& outlier_rejection_thresh);

protected:
  std::string kStreamingPrefix = "[EkfYRioFilter]: ";
};

}  // namespace rio
