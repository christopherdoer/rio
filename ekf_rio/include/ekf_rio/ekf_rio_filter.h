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

#include <ekf_rio/EkfRioConfig.h>
#include <ekf_rio/data_types.h>

namespace rio
{
/**
 * @brief The EkfRioFilter class provides the EKF based Radar Inertial Odometry Filter
 * \note Uses the North East Down (NED) convention --> the z-axis point downwards
 */
class EkfRioFilter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /**
   * @brief Reconfigure callback, enables online reconfigure using rqt_reconfigure
   * @param config   must contain the members of EkfRioConfig
   */
  template <class ConfigContainingEkfRioConfig>
  bool configure(ConfigContainingEkfRioConfig& config);

  /**
   * @brief Initializes the navigation filter
   * @param imu_init_vec   vector of imu measurements used for initialization
   * @returns true if init successful
   */
  bool init(const std::vector<ImuDataStamped>& imu_init_vec, const Real &baro_h0);

  /**
   * @brief Propagates the filter state using the provided IMU measurement
   * @returns true if successful
   */
  bool propagate(const ImuDataStamped& imu);

  /**
   * @brief Augments the the filter state with the states needed for the radar filter update
   * @param trigger_stamp   time stamp of the radar trigger
   * @returns true if successful
   */
  bool addRadarStateClone(const ros::Time& trigger_stamp);

  /**
   * @brief Removes the augmented radar state clone
   * @returns true if successful
   */
  bool removeRadarStateClone();

  /**
   * @brief Implements the altimeter kalman filter update model
   * @param neg_rel_h   relative negative height [m]
   * @param sigma       sigma of this measurement
   * @returns true if successful
   */
  bool updateAltimeter(const Real neg_rel_h, const Real& sigma);

  /**
   * @brief Implements the radar ego velocity Kalman filter update model
   * @param v_r                       3D radar ego velocity
   * @param sigma_v_r                 sigma of 3D velocity
   * @param w                         angular velocity observed
   * @param outlier_rejection_thresh  threshold for Mahalanbis distance check
   * @returns true if successful
   */
  bool updateRadarEgoVelocity(const Vector3 v_r,
                              const Vector3 sigma_v_r,
                              const Vector3 w,
                              const Real outlier_rejection_thresh);

  /**
   * @brief Returns the current filter ros time stamp
   */
  ros::Time getTimestamp() const { return time_stamp_; }

  /**
   * @brief Returns the covariance matrix
   */
  Matrix getCovarianceMatrix() const { return covariance_; }

  /**
   * @brief Returns the error state idx struct
   */
  EkfRioFilterStateIdx getErrorIdx() const { return error_idx_; }

  /**
   * @brief Returns the current navigation solution
   */
  NavigationSolution getNavigationSolution() const { return nav_sol_; }

  /**
   * @brief Returns the estimated biases (acc, gyro, altimeter)
   */
  Offsets getBias() const { return bias_; }

  /**
   * @brief Returns the estimated translation of the extrinsic radar transform
   */
  Vector3 getlbr() const { return T_b_r_.translation(); }

  /**
   * @brief Returns the estimated rotational part of the extrinsic radar transform
   */
  Matrix3 getCbr() const { return T_b_r_.linear(); }

  /**
   * @brief Returns the extrinsic radar transform as Isometry
   */
  Isometry getTbr() const { return T_b_r_; }

  /**
   * @brief Returns the cloned radar state
   */
  RadarCloneState getRadarCloneState() const { return radar_clone_state_; }

protected:
  /**
   * @brief Returns the state transtition matrix Phi used for the propagation of the covariance
   * @param a_b_ib    observed acceleration
   * @param T         discretization time
   * @returns Phi
   */
  Matrix getPhi(Vector3 a_b_ib, const Real& T) const;

  /**
   * @brief Corrects the nominal filter state based on the given error state
   * @returns true if successful
   */
  bool correctNominalState(const Vector x_error);

  /**
   * @brief Implements an Extended Kalman Filter update
   * @param r       observed residuum
   * @param H       measurement jacobian
   * @param R_diag  diagonal elements of the measurement noise matrix
   * @returns true if successful
   */
  bool kfUpdate(const Vector& r, const Matrix& H, const Vector& R_diag);

  /**
   * @brief Returns the corrected quaternion based on the given euler error and quaternion (Hamilton convention!)
   * @param err_euler   Euler error angle
   * @param q           Quaterion to be corrected
   * @returns the corrected quaternion
   */
  Quaternion getCorrectedQuaternion(const Vector3& err_euler, const Quaternion& q) const;

  std::string kStreamingPrefix = "[EkfRioFilter]: ";

  Strapdown strapdown_;

  InitStruct init_struct_;
  SystemNoisePsd system_noise_;

  ros::Time time_stamp_;
  NavigationSolution nav_sol_;
  Offsets bias_;
  Isometry T_b_r_;

  RadarCloneState radar_clone_state_;

  Vector x_error_;
  EkfRioFilterStateIdx error_idx_;

  Matrix covariance_;
};

template <typename ConfigContainingEkfRioConfig>
bool EkfRioFilter::configure(ConfigContainingEkfRioConfig& config)
{
  bool success = true;

  success |= init_struct_.configure(error_idx_, config);
  success |= system_noise_.configure(config);

  return success;
}

}  // namespace rio
