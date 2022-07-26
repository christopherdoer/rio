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

#include <boost/math/distributions/chi_squared.hpp>

#include <rio_utils/math_helper.h>
#include <rio_utils/ros_helper.h>

#include <x_rio/x_rio_filter.h>

using namespace rio;

bool XRioFilter::init(const std::vector<ImuDataStamped>& imu_init_vec, const RadarExtrinsicsVec& radar_extrinsics_0)
{
  nav_sol_.setPosition_n_b(init_struct_.p_0);
  nav_sol_.v_n_b = init_struct_.v_0;

  // init att from acc
  Vector3 a_accu(0, 0, 0), w_accu(0, 0, 0);
  for (const auto& imu_data : imu_init_vec)
  {
    a_accu += imu_data.a_b_ib;
    w_accu += imu_data.w_b_ib;
  }
  const Vector3 acc_mean = a_accu / imu_init_vec.size();
  const Vector3 w_mean   = w_accu / imu_init_vec.size();

  EulerAngles roll_pitch = math_helper::initFromAcc(acc_mean, init_struct_.gravity);

  ROS_INFO_STREAM(kStreamingPrefix << "Initialized attitude: " << roll_pitch.to_degrees().x() << "deg, "
                                   << roll_pitch.to_degrees().y() << "deg");

  nav_sol_.setEuler_n_b(EulerAngles(roll_pitch.roll(), roll_pitch.pitch(), init_struct_.yaw_0));

  bias_.acc  = init_struct_.b_a_0;
  bias_.gyro = init_struct_.omega_calibration ? w_mean + init_struct_.b_w_0 : init_struct_.b_w_0;
  bias_.alt  = init_struct_.b_alt_0;

  ROS_INFO_STREAM(kStreamingPrefix << "Initialized w_bias: " << EulerAngles(bias_.gyro).to_degrees().transpose());

  error_idx_.state_length =
      error_idx_.base_state_length + radar_extrinsics_0.size() * error_idx_.extrinsics_state_length;
  covariance_ = Matrix::Identity(error_idx_.state_length, error_idx_.state_length);

  covariance_.block(0, 0, error_idx_.base_state_length, error_idx_.base_state_length) = init_struct_.P_kk_0;
  strapdown_                                                                          = Strapdown(init_struct_.gravity);
  time_stamp_                                                                         = imu_init_vec.back().time_stamp;

  // setup extrinsics dynamically
  radar_extrinsics_0_ = radar_extrinsics_0;
  uint state_offset   = error_idx_.base_state_length;

  for (uint k = 0; k < radar_extrinsics_0.size(); ++k)
  {
    Isometry T_b_r;

    if (estimated_extrinsics_error_att_)
      T_b_r.linear() = Matrix3::Identity();
    else
      T_b_r.linear() = radar_extrinsics_0.at(k).linear();
    T_b_r.translation() = radar_extrinsics_0.at(k).translation();

    radar_extrinsics_.emplace_back(T_b_r);

    if (estimate_extrinsics_)
    {
      error_idx_.radar_extrinsics.emplace_back(RadarExtrinsicsIdx(state_offset));
      state_offset += error_idx_.extrinsics_state_length;

      covariance_.block(error_idx_.radar_extrinsics.back().l_b_r, error_idx_.radar_extrinsics.back().l_b_r, 3, 3)
          .diagonal() = init_struct_.l_b_r_sigma_0.array().pow(2);

      covariance_.block(error_idx_.radar_extrinsics.back().eul_b_r, error_idx_.radar_extrinsics.back().eul_b_r, 3, 3)
          .diagonal() = init_struct_.eul_b_r_sigma_0.array().pow(2);
    }
  }

  return true;
}

bool XRioFilter::propagate(const ImuDataStamped& imu)
{
  time_stamp_ = imu.time_stamp;

  const Vector3 a_b_ib_corrected = imu.a_b_ib - bias_.acc;
  const Vector3 w_b_ib_corrected = imu.w_b_ib - bias_.gyro;
  nav_sol_                       = strapdown_.propagate(nav_sol_, a_b_ib_corrected, w_b_ib_corrected, imu.dt);

  const Matrix Phi = getPhi(a_b_ib_corrected, imu.dt);
  const Matrix G   = system_noise_.getG(nav_sol_.getC_n_b(), error_idx_, covariance_.rows());

  // TODO: consider more efficient implementation!!
  const Matrix Phi_16 = Phi.block(0, 0, error_idx_.base_state_length, error_idx_.base_state_length);
  const Matrix G_16   = G.block(0, 0, error_idx_.base_state_length, G.cols());

  // speeds up propagation runtime by factor 2 (instead of "standard propagation)

  // upper left
  Matrix covariance_prop = Matrix::Zero(covariance_.rows(), covariance_.cols());
  covariance_prop.block(0, 0, error_idx_.base_state_length, error_idx_.base_state_length) =
      Phi_16 * covariance_.block(0, 0, error_idx_.base_state_length, error_idx_.base_state_length) *
          Phi_16.transpose() +
      G_16 * system_noise_.getQ(imu.dt) * G_16.transpose();

  // upper right
  covariance_prop.block(0,
                        error_idx_.base_state_length,
                        error_idx_.base_state_length,
                        covariance_.cols() - error_idx_.base_state_length) =
      Phi_16 * covariance_.block(0,
                                 error_idx_.base_state_length,
                                 error_idx_.base_state_length,
                                 covariance_.cols() - error_idx_.base_state_length);

  // lower left
  covariance_prop.block(error_idx_.base_state_length,
                        0,
                        covariance_.rows() - error_idx_.base_state_length,
                        error_idx_.base_state_length) =
      covariance_.block(error_idx_.base_state_length,
                        0,
                        covariance_.rows() - error_idx_.base_state_length,
                        error_idx_.base_state_length) *
      Phi_16.transpose();

  // lower right --> no need to propagate :)
  covariance_prop.block(error_idx_.base_state_length,
                        error_idx_.base_state_length,
                        covariance_.rows() - error_idx_.base_state_length,
                        covariance_.cols() - error_idx_.base_state_length) =
      covariance_.block(error_idx_.base_state_length,
                        error_idx_.base_state_length,
                        covariance_.rows() - error_idx_.base_state_length,
                        covariance_.cols() - error_idx_.base_state_length);

  covariance_ = covariance_prop;

  return true;
}

Matrix XRioFilter::getPhi(Vector3 a_b_ib, const Real& T) const
{
  Matrix F = Matrix::Zero(error_idx_.prob_noise_state_length, error_idx_.prob_noise_state_length);
  F.block(error_idx_.position, error_idx_.velocity, 3, 3)  = Matrix::Identity(3, 3);
  F.block(error_idx_.velocity, error_idx_.attitude, 3, 3)  = -1 * math_helper::skewVec(nav_sol_.getC_n_b() * a_b_ib);
  F.block(error_idx_.velocity, error_idx_.bias_acc, 3, 3)  = -nav_sol_.getC_n_b();
  F.block(error_idx_.attitude, error_idx_.bias_gyro, 3, 3) = -nav_sol_.getC_n_b();

  Matrix Phi = Matrix::Identity(covariance_.rows(), covariance_.cols());
  Phi.block(0, 0, F.rows(), F.cols()) += F * T;

  return Phi;
}

bool XRioFilter::kfUpdate(const Vector& r, const Matrix& H, const Matrix& R)
{
  const Matrix S = H * covariance_ * H.transpose() + R;

  // so far fastest impl (1.2x faster than S.inv())
  Matrix S_inv = Matrix::Identity(S.rows(), S.cols());
  S.llt().solveInPlace(S_inv);

  const Matrix K       = covariance_ * H.transpose() * S_inv;
  const Vector x_error = K * r;

  covariance_ = covariance_ - K * H * covariance_;
  correctNominalState(x_error);

  return true;  // TODO add check for invalid result
}

bool XRioFilter::correctNominalState(const Vector x_error)
{
  // correct base state
  nav_sol_.setPosition_n_b(nav_sol_.getPosition_n_b() - x_error.segment(error_idx_.position, 3));
  nav_sol_.v_n_b -= x_error.segment(error_idx_.velocity, 3);
  nav_sol_.setQuaternion(
      math_helper::getCorrectedQuaternion(x_error.segment(error_idx_.attitude, 3), nav_sol_.getQuaternion_n_b()));

  bias_.acc -= x_error.segment(error_idx_.bias_acc, 3);
  bias_.gyro -= x_error.segment(error_idx_.bias_gyro, 3);
  bias_.alt -= x_error(error_idx_.bias_alt);

  if (estimate_extrinsics_)
  {
    assert(radar_extrinsics_.size() == error_idx_.radar_extrinsics.size());
    for (uint k = 0; k < radar_extrinsics_.size(); ++k)
    {
      Isometry& T_b_r     = radar_extrinsics_.at(k);
      T_b_r.translation() = T_b_r.translation() - x_error.segment(error_idx_.radar_extrinsics.at(k).l_b_r, 3);
      T_b_r.linear()      = math_helper::getCorrectedQuaternion(
                           x_error.segment(error_idx_.radar_extrinsics.at(k).eul_b_r, 3), Quaternion(T_b_r.linear()))
                           .normalized()
                           .toRotationMatrix();
    }
  }

  // correct clones
  for (auto& clone : clones_) clone->correct(x_error);

  return true;
}

bool XRioFilter::addRadarStateClone(const uint radar_id, const ros::Time& trigger_stamp)
{
  assert(radar_id < radar_extrinsics_.size());  // && radar_id < error_idx_.radar_extrinsics.size());

  // clone already in filter state --> remove it!
  std::vector<uint> states_to_be_removed;
  for (const auto& clone : clones_)
  {
    if (auto radar_clone = std::dynamic_pointer_cast<RadarClone>(clone))
    {
      if (radar_clone->getRadarId() == radar_id)
        states_to_be_removed.emplace_back(clone->getCloneStateId());
    }
  }

  for (const auto& state_id : states_to_be_removed) removeClone(state_id);

  if (use_reduced_radar_clone_ || !estimate_extrinsics_)
  {
    ReducedRadarCloneState radar_clone_state;
    radar_clone_state.time_stamp         = time_stamp_;
    radar_clone_state.trigger_time_stamp = trigger_stamp;
    radar_clone_state.nav_sol            = nav_sol_;
    addClone(std::shared_ptr<CloneBase>(new ReducedRadarClone(radar_id, clones_.size(), radar_clone_state)));
  }
  else
  {
    FullRadarCloneState radar_clone_state;
    radar_clone_state.time_stamp         = time_stamp_;
    radar_clone_state.trigger_time_stamp = trigger_stamp;
    radar_clone_state.nav_sol            = nav_sol_;
    radar_clone_state.offset_gyro        = bias_.gyro;
    radar_clone_state.T_b_r              = radar_extrinsics_.at(radar_id);
    addClone(std::shared_ptr<CloneBase>(new RadarClone(radar_id, clones_.size(), radar_clone_state)));
  }

  return true;
}

void XRioFilter::addClone(std::shared_ptr<CloneBase> clone)
{
  covariance_ = clone->getAugmentedCovariance(covariance_, error_idx_);
  clones_.emplace_back(clone);
}

bool XRioFilter::removeClone(const uint clone_state_id)
{
  if (clones_.size() == 0)
  {
    ROS_ERROR_STREAM(kStreamingPrefix << "No clone to be removed.");
    return false;
  }

  for (auto iter = clones_.begin(); iter != clones_.end(); ++iter)
  {
    if ((*iter)->getCloneStateId() == clone_state_id)
    {
      covariance_ = (*iter)->getPrunedCovariance(covariance_);

      // correct idx of all subsequent states if there are any
      for (auto& state : clones_)
      {
        if (state->getCloneStateId() > (*iter)->getCloneStateId())
        {
          state->updateIndices(state->getCloneStateId() - 1, (*iter)->getErrorStateLength());
        }
      }

      clones_.erase(iter);
      return true;
    }
  }
  return false;
}

bool XRioFilter::updateAltimeter(const Real neg_rel_h, const Real& sigma)
{
  Matrix H = Matrix::Zero(1, getCovarianceMatrix().cols());
  Vector r(1);
  Vector R(1);

  H(0, getErrorIdx().position + 2) = 1;
  H(0, getErrorIdx().bias_alt)     = 1;
  const Real h_filter              = getNavigationSolution().getPosition_n_b().z();
  const Real h_meas                = neg_rel_h - getBias().alt;
  r(0)                             = h_filter - h_meas;
  R(0)                             = sigma * sigma;
  kfUpdate(r, H, R);

  return true;
}

bool XRioFilter::updateRadarEgoVelocity(const uint radar_id,
                                        const Vector3& v_r,
                                        const Matrix3& P_v_r,
                                        const Vector3& w,
                                        const Real outlier_rejection_thresh)
{
  Matrix H = Matrix::Zero(3, getCovarianceMatrix().cols());
  Vector r(3);

  std::shared_ptr<RadarClone> radar_clone;
  std::shared_ptr<ReducedRadarClone> reduced_radar_clone;

  if (getRadarClone(radar_id, radar_clone))
  {
    const auto radar_clone_state = radar_clone->getRadarCloneState();

    // full clone
    const Matrix C_n_b      = radar_clone_state.nav_sol.getC_n_b();
    const Matrix C_b_r      = radar_clone_state.T_b_r.linear();
    const Vector3 l_b_br    = radar_clone_state.T_b_r.translation();
    const Vector3 v_n_b     = radar_clone_state.nav_sol.v_n_b;
    const Vector3 bias_gyro = radar_clone_state.offset_gyro;

    const Vector3 v_w = math_helper::skewVec(w - bias_gyro) * l_b_br;
    const Vector3 v_b = C_n_b.transpose() * v_n_b;

    const Matrix3 H_v     = C_b_r.transpose() * C_n_b.transpose();
    const Matrix3 H_q     = C_b_r.transpose() * C_n_b.transpose() * math_helper::skewVec(v_n_b);
    const Matrix3 H_bg    = -C_b_r.transpose() * math_helper::skewVec(l_b_br);
    const Matrix3 H_l_b_r = C_b_r.transpose() * math_helper::skewVec(w);
    const Matrix3 H_q_b_r = C_b_r.transpose() * math_helper::skewVec(v_w + v_b);

    H.block(0, radar_clone->getStateIdx().velocity(), 3, 3)  = H_v;
    H.block(0, radar_clone->getStateIdx().attitude(), 3, 3)  = H_q;
    H.block(0, radar_clone->getStateIdx().bias_gyro(), 3, 3) = H_bg;
    H.block(0, radar_clone->getStateIdx().l_b_r(), 3, 3)     = H_l_b_r;
    H.block(0, radar_clone->getStateIdx().eul_b_r(), 3, 3)   = H_q_b_r;

    const Vector3 v_r_filter = C_b_r.transpose() * (v_w + v_b);

    if (estimated_extrinsics_error_att_)
      r = v_r_filter - radar_extrinsics_0_.at(radar_id).linear() * v_r;
    else
      r = v_r_filter - v_r;
  }
  else if (getRadarClone(radar_id, reduced_radar_clone))
  {
    const auto reduced_radar_clone_state = reduced_radar_clone->getRadarCloneState();

    // reduced clone
    const Matrix C_n_b  = reduced_radar_clone_state.nav_sol.getC_n_b();
    const Vector3 v_n_b = reduced_radar_clone_state.nav_sol.v_n_b;

    const Matrix C_b_r      = getCbr(radar_id);
    const Vector3 l_b_br    = getlbr(radar_id);
    const Vector3 bias_gyro = bias_.gyro;

    const Vector3 v_w = math_helper::skewVec(w - bias_gyro) * l_b_br;
    const Vector3 v_b = C_n_b.transpose() * v_n_b;

    const Matrix3 H_v  = C_b_r.transpose() * C_n_b.transpose();
    const Matrix3 H_q  = C_b_r.transpose() * C_n_b.transpose() * math_helper::skewVec(v_n_b);
    const Matrix3 H_bg = -C_b_r.transpose() * math_helper::skewVec(l_b_br);

    if (estimate_extrinsics_)
    {
      const Matrix3 H_l_b_r = C_b_r.transpose() * math_helper::skewVec(w);
      const Matrix3 H_q_b_r = C_b_r.transpose() * math_helper::skewVec(v_w + v_b);

      H.block(0, reduced_radar_clone->getStateIdx().velocity(), 3, 3)    = H_v;
      H.block(0, reduced_radar_clone->getStateIdx().attitude(), 3, 3)    = H_q;
      H.block(0, error_idx_.bias_gyro, 3, 3)                             = H_bg;
      H.block(0, error_idx_.radar_extrinsics.at(radar_id).l_b_r, 3, 3)   = H_l_b_r;
      H.block(0, error_idx_.radar_extrinsics.at(radar_id).eul_b_r, 3, 3) = H_q_b_r;
    }
    else
    {
      H.block(0, reduced_radar_clone->getStateIdx().velocity(), 3, 3) = H_v;
      H.block(0, reduced_radar_clone->getStateIdx().attitude(), 3, 3) = H_q;
      H.block(0, error_idx_.bias_gyro, 3, 3)                          = H_bg;
    }

    const Vector3 v_r_filter = C_b_r.transpose() * (v_w + v_b);

    if (estimated_extrinsics_error_att_)
      r = v_r_filter - radar_extrinsics_0_.at(radar_id).linear() * v_r;
    else
      r = v_r_filter - v_r;
  }
  else
  {
    ROS_ERROR_STREAM(kStreamingPrefix << "Unable to perform updateRadarEgoVelocity, no radar clone state!");

    return false;
  }

  return updateWithOutlierRejection(
      r, H, P_v_r, 3, outlier_rejection_thresh, "Outlier radar velocity id " + std::to_string(radar_id));
}

bool XRioFilter::updateYaw(const uint radar_id,
                           const Real& yaw_m,
                           const Real& sigma_yaw,
                           const Real& outlier_rejection_thresh)
{
  Matrix H = Matrix::Zero(1, covariance_.cols());
  Vector r(1);
  Vector R(1);

  std::shared_ptr<RadarClone> radar_clone;
  std::shared_ptr<ReducedRadarClone> reduced_radar_clone;

  if (getRadarClone(radar_id, radar_clone))
  {
    const auto radar_clone_state                                = radar_clone->getRadarCloneState();
    H.block(0, radar_clone->getStateIdx().attitude() + 2, 1, 1) = Matrix::Ones(1, 1);
    r[0] = math_helper::wrapToCentered(radar_clone_state.nav_sol.getEuler_n_b().yaw() - yaw_m, M_PI);
  }
  else if (getRadarClone(radar_id, reduced_radar_clone))
  {
    const auto reduced_radar_clone_state                                = reduced_radar_clone->getRadarCloneState();
    H.block(0, reduced_radar_clone->getStateIdx().attitude() + 2, 1, 1) = Matrix::Ones(1, 1);
    r[0] = math_helper::wrapToCentered(reduced_radar_clone_state.nav_sol.getEuler_n_b().yaw() - yaw_m, M_PI);
  }
  else
  {
    ROS_ERROR_STREAM(kStreamingPrefix << "Unable to perform updateRadarEgoVelocity, no radar clone state!");
    return false;
  }

  R[0] = sigma_yaw * sigma_yaw;

  return updateWithOutlierRejection(
      r, H, R, 1, outlier_rejection_thresh, "Outlier radar yaw id " + std::to_string(radar_id));
}

bool XRioFilter::updateWithOutlierRejection(const Vector& r,
                                            const Matrix& H,
                                            const Matrix& R,
                                            const uint chi_squared_dof,
                                            const Real& outlier_rejection_thresh,
                                            const std::string& failure_log_msg)
{
  if (outlier_rejection_thresh > 0.00001)
  {
    const Real gamma = r.transpose() * (H * getCovarianceMatrix() * H.transpose() + R).inverse() * r;
    boost::math::chi_squared chiSquaredDist(chi_squared_dof);
    const double gamma_thresh = boost::math::quantile(chiSquaredDist, 1 - outlier_rejection_thresh);

    if (gamma < gamma_thresh)
    {
      kfUpdate(r, H, R);
    }
    else
    {
      ROS_INFO_STREAM(kStreamingPrefix << failure_log_msg);
      return false;
    }
  }
  else
  {
    kfUpdate(r, H, R);
  }
  return true;
}
