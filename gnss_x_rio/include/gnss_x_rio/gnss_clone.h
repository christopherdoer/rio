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

#include <rio_utils/data_types.h>
#include <rio_utils/math_helper.h>

#include <x_rio/data_types.h>
#include <x_rio/clone_base.h>

namespace rio
{
class GnssCloneErrorStateIdx
{
public:
  void setErrorStateOffset(const uint error_state_offset) { error_state_offset_ = error_state_offset; }
  uint getErrorStateOffset() const { return error_state_offset_; }
  uint getCloneLength() const { return gnss_clone_length_; }

  uint position() { return error_state_offset_ + position_; }
  uint velocity() { return error_state_offset_ + velocity_; }

private:
  uint error_state_offset_ = 0;
  uint position_           = 0;
  uint velocity_           = 3;
  uint gnss_clone_length_  = 6;
};

class GnssClone : public CloneBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssClone(const uint clone_state_id, const GnssCloneState& state) :
    CloneBase(clone_state_id, GnssCloneErrorStateIdx().getCloneLength()),
    state_{state}
  {
  }

  virtual Matrix getAugmentedCovariance(const Matrix& covariance, const FilterStateIdx& error_idx)
  {
    const auto error_state_length = covariance.rows();
    idx_.setErrorStateOffset(error_state_length);

    // setup jacobian
    Matrix J_            = Matrix::Zero(idx_.getCloneLength(), error_state_length);
    J_.block(0, 0, 6, 6) = Matrix::Identity(6, 6);

    Matrix J = Matrix::Zero(error_state_length + idx_.getCloneLength(), error_state_length);
    J.block(0, 0, error_state_length, error_state_length) = Matrix::Identity(error_state_length, error_state_length);
    J.block(error_state_length, 0, J_.rows(), J_.cols())  = J_;

    return J * covariance * J.transpose();
  }

  virtual Matrix getPrunedCovariance(const Matrix& covariance)
  {
    // general case
    const uint pruned_state_length = covariance.rows() - idx_.getCloneLength();
    const uint clone_state_end_idx = idx_.getErrorStateOffset() + idx_.getCloneLength();

    Matrix covariance_pruned = Matrix::Zero(pruned_state_length, pruned_state_length);

    // left top
    covariance_pruned.block(0, 0, idx_.getErrorStateOffset(), idx_.getErrorStateOffset()) =
        covariance.block(0, 0, idx_.getErrorStateOffset(), idx_.getErrorStateOffset());

    // right top
    covariance_pruned.block(
        0, idx_.getErrorStateOffset(), idx_.getErrorStateOffset(), covariance.cols() - clone_state_end_idx) =
        covariance.block(0, clone_state_end_idx, idx_.getErrorStateOffset(), covariance.cols() - clone_state_end_idx);

    // left bottom
    covariance_pruned.block(
        idx_.getErrorStateOffset(), 0, covariance.rows() - clone_state_end_idx, idx_.getErrorStateOffset()) =
        covariance.block(clone_state_end_idx, 0, covariance.rows() - clone_state_end_idx, idx_.getErrorStateOffset());

    // right bottom
    covariance_pruned.block(idx_.getErrorStateOffset(),
                            idx_.getErrorStateOffset(),
                            covariance.rows() - clone_state_end_idx,
                            covariance.cols() - clone_state_end_idx) =
        covariance.block(clone_state_end_idx,
                         clone_state_end_idx,
                         covariance.rows() - clone_state_end_idx,
                         covariance.cols() - clone_state_end_idx);

    return covariance_pruned;
  }

  virtual void correct(const Vector& x_error)
  {
    state_.p_n_b -= x_error.segment(idx_.position(), 3);
    state_.v_n_b -= x_error.segment(idx_.velocity(), 3);
  }

  virtual void updateIndices(const uint new_id, const uint shift_state)
  {
    CloneBase::updateIndices(new_id, shift_state);
    idx_.setErrorStateOffset(idx_.getErrorStateOffset() - shift_state);
  }

  GnssCloneState getGnssCloneState() const { return state_; }

  GnssCloneErrorStateIdx getStateIdx() const { return idx_; }

protected:
  GnssCloneErrorStateIdx idx_;
  GnssCloneState state_;
};

};  // namespace rio
