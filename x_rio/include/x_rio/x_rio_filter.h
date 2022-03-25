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

#include <x_rio/XRioConfig.h>
#include <x_rio/data_types.h>
#include <x_rio/clone_base.h>
#include <x_rio/radar_clone.h>
#include <x_rio/radar_clone_reduced.h>

namespace rio
{
/**
 * @brief The XRioFilter class provides the EKF based Radar Inertial Odometry Filter for multiple radar sensors
 * \note Uses the North East Down (NED) convention --> the z-axis point downwards
 */
class XRioFilter
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
  bool init(const std::vector<ImuDataStamped>& imu_init_vec, const RadarExtrinsicsVec& radar_extrinsics_0);

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
  bool addRadarStateClone(const uint radar_id, const ros::Time& trigger_stamp);

  /**
   * @brief Removes the augmented clone with given id
   * @returns true if successful
   */
  bool removeClone(const uint clone_state_id);

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
  bool updateRadarEgoVelocity(const uint radar_id,
                              const Vector3& v_r,
                              const Matrix3& P_v_r,
                              const Vector3& w,
                              const Real outlier_rejection_thresh);

  bool updateYaw(const uint radar_id, const Real& yaw_m, const Real& sigma_yaw, const Real& outlier_rejection_thresh);

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
  FilterStateIdx getErrorIdx() const { return error_idx_; }

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
  Vector3 getlbr(const uint id) const
  {
    assert(id < radar_extrinsics_.size());
    //    return radar_extrinsics_0_.at(id).translation() + radar_extrinsics_.at(id).translation();
    return radar_extrinsics_.at(id).translation();
  }

  /**
   * @brief Returns the estimated rotational part of the extrinsic radar transform
   */
  Matrix3 getCbr(const uint id) const
  {
    assert(id < radar_extrinsics_.size());
    //    return radar_extrinsics_0_.at(id).linear() * radar_extrinsics_.at(id).linear();
    return radar_extrinsics_.at(id).linear();
  }

  /**
   * @brief Returns the extrinsic radar transform as Isometry
   */
  Isometry getTbr(const uint id) const
  {
    assert(id < radar_extrinsics_.size());
    //    return radar_extrinsics_0_.at(id) * radar_extrinsics_.at(id);
    return radar_extrinsics_.at(id);
  }

  /**
   * @brief Getter for the cloned radar state
   * @returns true if radar clone state valid
   */
  template <class RadarCloneType>
  bool getRadarClone(const uint radar_id, std::shared_ptr<RadarCloneType>& radar_clone_state) const
  {
    assert(radar_id < radar_extrinsics_.size());

    if (clones_.size() > 0)
    {
      for (const auto& clone : clones_)
      {
        if (auto radar_clone_state_ = std::dynamic_pointer_cast<RadarCloneType>(clone))
        {
          if (radar_clone_state_->getRadarId() == radar_id)
          {
            radar_clone_state = radar_clone_state_;
            return true;
          }
        }
      }
    }
    return false;
  }

  bool getFullRadarState(const uint radar_id, FullRadarCloneState& radar_clone_state, uint& clone_state_id) const
  {
    assert(radar_id < radar_extrinsics_.size());

    if (clones_.size() > 0)
    {
      for (const auto& clone : clones_)
      {
        if (auto radar_clone = std::dynamic_pointer_cast<RadarClone>(clone))
        {
          if (radar_clone->getRadarId() == radar_id)
          {
            radar_clone_state = radar_clone->getRadarCloneState();
            clone_state_id    = radar_clone->getCloneStateId();
            return true;
          }
        }
        else if (auto radar_clone = std::dynamic_pointer_cast<ReducedRadarClone>(clone))
        {
          if (radar_clone->getRadarId() == radar_id)
          {
            radar_clone_state =
                FullRadarCloneState(radar_clone->getRadarCloneState(), bias_.gyro, radar_extrinsics_.at(radar_id));
            clone_state_id = radar_clone->getCloneStateId();

            return true;
          }
        }
      }
    }
    return false;
  }

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
   * @brief Kalman filter update with prior Mahalanobis outlier rejection
   * @param r                          residual
   * @param H                          Jacobian
   * @param R                          Measurement noise
   * @param chi_squared_dof            degrees of freedom for chi squred function
   * @param outlier_rejection_thresh   Mahalanobis distance threshold
   * @param failure_log_msg            console message if outlier detected
   * @returns true if successfull
   */
  bool updateWithOutlierRejection(const Vector& r,
                                  const Matrix& H,
                                  const Matrix& R,
                                  const uint chi_squared_dof,
                                  const Real& outlier_rejection_thresh,
                                  const std::string& failure_log_msg);

  /**
   * @brief Implements an Extended Kalman Filter update
   * @param r       observed residuum
   * @param H       measurement jacobian
   * @param R_diag  diagonal elements of the measurement noise matrix
   * @returns true if successful
   */
  bool kfUpdate(const Vector& r, const Matrix& H, const Matrix& R);

  /**
   * @brief Adds a clone of type CloneBase or derived
   *
   * @param clone
   */
  void addClone(std::shared_ptr<CloneBase> clone);

  std::string kStreamingPrefix = "[EkfRioFilter]: ";

  Strapdown strapdown_;

  BaseStateInitStruct init_struct_;
  SystemNoisePsd system_noise_;

  ros::Time time_stamp_;
  NavigationSolution nav_sol_;
  Offsets bias_;
  RadarExtrinsicsVec radar_extrinsics_;
  RadarExtrinsicsVec radar_extrinsics_0_;

  FilterStateIdx error_idx_;

  Matrix covariance_;

  std::vector<std::shared_ptr<CloneBase>> clones_;

  bool estimate_extrinsics_;
  bool use_reduced_radar_clone_;
  bool estimated_extrinsics_error_att_;
};

template <typename ConfigContainingEkfRioConfig>
bool XRioFilter::configure(ConfigContainingEkfRioConfig& config)
{
  bool success = true;

  success |= init_struct_.configure(error_idx_, config);
  success |= system_noise_.configure(config);
  estimate_extrinsics_            = config.estimate_extrinsics;
  use_reduced_radar_clone_        = config.use_reduced_radar_clone;
  estimated_extrinsics_error_att_ = config.estimated_extrinsics_error_att;
  return success;
}

}  // namespace rio
