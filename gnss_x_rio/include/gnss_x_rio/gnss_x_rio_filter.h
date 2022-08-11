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

#include <sensor_msgs/Imu.h>

#include <rio_utils/data_types.h>
#include <rio_utils/strapdown.h>

#include <x_rio/x_rio_filter.h>

#include <gnss_x_rio/GnssXRioConfig.h>
#include <gnss_x_rio/gnss_clone.h>

namespace rio
{
/**
 * @brief The GNSSXRioFilter class provides the EKF based Radar Inertial Odometry Filter for multiple radar sensors
 * \note Uses the North East Down (NED) convention --> the z-axis point downwards
 */
class GnssXRioFilter : public XRioFilter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Reconfigure callback, enables online reconfigure using rqt_reconfigure
   * @param config   must contain the members of EkfRioConfig
   */
  template <class ConfigContainingGnssXRioConfig>
  bool configure(ConfigContainingGnssXRioConfig& config);

  /**
   * @brief Augments the the filter state with the states needed for the radar filter update
   * @param trigger_stamp   time stamp of the radar trigger
   * @returns true if successful
   */
  bool addGnssStateClone(const ros::Time& trigger_stamp);

  /**
   * @brief Returns the GNSS clonse (assuming there is only one)
   */
  bool getGnssClone(GnssClone& gnss_clone) const;

  /**
   * @brief Performs a GNSS position update using the local dpos
   */
  bool updateGnssPosition(const GnssClone& gnss_clone,
                          const Vector3 dpos_n,
                          const Vector3& sigma_p,
                          const Real& outlier_rejection_thresh);
  /**
   * @brief Performs a GNSS velocity update given the global velocity measurement
   */
  bool updateGnssVelocity(const GnssClone& gnss_clone,
                          const Vector3& v_n_gpnss,
                          const Vector3& sigma_v,
                          const Real& outlier_rejection_thresh);

protected:
  std::string kStreamingPrefix = "[GnssXRioFilter]: ";
};

template <typename ConfigContainingGnssXRioConfig>
bool GnssXRioFilter::configure(ConfigContainingGnssXRioConfig& config)
{
  return XRioFilter::configure(config);
}

}  // namespace rio
