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

#include <rio_utils/data_types.h>
#include <rio_utils/math_helper.h>

#include <x_rio/data_types.h>
#include <x_rio/clone_base.h>

namespace rio
{
class RadarCloneReducedErrorStateIdx
{
public:
  void setErrorStateOffset(const uint error_state_offset) { error_state_offset_ = error_state_offset; }
  uint getErrorStateOffset() const { return error_state_offset_; }
  uint getRadarCloneLength() const { return radar_clone_length_; }

  uint position() { return error_state_offset_ + position_; }
  uint velocity() { return error_state_offset_ + velocity_; }
  uint attitude() { return error_state_offset_ + attitude_; }

private:
  uint error_state_offset_ = 0;
  uint position_           = 0;
  uint velocity_           = 3;
  uint attitude_           = 6;
  uint radar_clone_length_ = 9;
};

/**
 * @brief The ReducedRadarClone class implements the full radar state clone
 */
class ReducedRadarClone : public CloneBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReducedRadarClone(const uint radar_id, const uint clone_state_id, const ReducedRadarCloneState& state) :
    CloneBase(clone_state_id, RadarCloneReducedErrorStateIdx().getRadarCloneLength()),
    state_{state},
    radar_id_{radar_id}
  {
  }

  virtual Matrix getAugmentedCovariance(const Matrix& covariance, const FilterStateIdx& error_idx)
  {
    const auto error_state_length = covariance.rows();
    idx_.setErrorStateOffset(error_state_length);

    // setup jacobian
    Matrix J_            = Matrix::Zero(idx_.getRadarCloneLength(), error_state_length);
    J_.block(0, 0, 9, 9) = Matrix::Identity(9, 9);

    Matrix J = Matrix::Zero(error_state_length + idx_.getRadarCloneLength(), error_state_length);
    J.block(0, 0, error_state_length, error_state_length) = Matrix::Identity(error_state_length, error_state_length);
    J.block(error_state_length, 0, J_.rows(), J_.cols())  = J_;

    return J * covariance * J.transpose();
  }

  virtual Matrix getPrunedCovariance(const Matrix& covariance)
  {
    // general case
    const uint pruned_state_length = covariance.rows() - idx_.getRadarCloneLength();
    const uint radar_state_end_idx = idx_.getErrorStateOffset() + idx_.getRadarCloneLength();

    Matrix covariance_pruned = Matrix::Zero(pruned_state_length, pruned_state_length);

    // left top
    covariance_pruned.block(0, 0, idx_.getErrorStateOffset(), idx_.getErrorStateOffset()) =
        covariance.block(0, 0, idx_.getErrorStateOffset(), idx_.getErrorStateOffset());

    // right top
    covariance_pruned.block(
        0, idx_.getErrorStateOffset(), idx_.getErrorStateOffset(), covariance.cols() - radar_state_end_idx) =
        covariance.block(0, radar_state_end_idx, idx_.getErrorStateOffset(), covariance.cols() - radar_state_end_idx);

    // left bottom
    covariance_pruned.block(
        idx_.getErrorStateOffset(), 0, covariance.rows() - radar_state_end_idx, idx_.getErrorStateOffset()) =
        covariance.block(radar_state_end_idx, 0, covariance.rows() - radar_state_end_idx, idx_.getErrorStateOffset());

    // right bottom
    covariance_pruned.block(idx_.getErrorStateOffset(),
                            idx_.getErrorStateOffset(),
                            covariance.rows() - radar_state_end_idx,
                            covariance.cols() - radar_state_end_idx) =
        covariance.block(radar_state_end_idx,
                         radar_state_end_idx,
                         covariance.rows() - radar_state_end_idx,
                         covariance.cols() - radar_state_end_idx);

    return covariance_pruned;
  }

  virtual void correct(const Vector& x_error)
  {
    state_.nav_sol.setPosition_n_b(state_.nav_sol.getPosition_n_b() - x_error.segment(idx_.position(), 3));
    state_.nav_sol.v_n_b -= x_error.segment(idx_.velocity(), 3);

    state_.nav_sol.setQuaternion(
        math_helper::getCorrectedQuaternion(x_error.segment(idx_.attitude(), 3), state_.nav_sol.getQuaternion_n_b()));
  }

  virtual void updateIndices(const uint new_id, const uint shift_state)
  {
    CloneBase::updateIndices(new_id, shift_state);
    idx_.setErrorStateOffset(idx_.getErrorStateOffset() - shift_state);
  }

  ReducedRadarCloneState getRadarCloneState() const { return state_; }

  uint getRadarId() const { return radar_id_; }

  RadarCloneReducedErrorStateIdx getStateIdx() const { return idx_; }

protected:
  RadarCloneReducedErrorStateIdx idx_;
  ReducedRadarCloneState state_;
  uint radar_id_;
};

};  // namespace rio
