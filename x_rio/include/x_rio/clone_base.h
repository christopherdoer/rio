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
#include <x_rio/data_types.h>

namespace rio
{
/**
 * @brief The CloneBase class provides the base class of the filter clones used for stochastic cloning
 */
class CloneBase
{
public:
  CloneBase(const uint clone_state_id, const uint clone_error_state_length) :
    clone_state_id_{clone_state_id},
    error_state_length_{clone_error_state_length}
  {
  }
  /**
   * @brief Returns the augmented covariance matrix, implemented by the derived class
   * @param covariance  current covariance matrix
   * @param error_idx   FilterStateIdx
   * @returns the augmented covariance matrix
   */
  virtual Matrix getAugmentedCovariance(const Matrix& covariance, const FilterStateIdx& error_idx) = 0;

  /**
   * @brief Returns the pruned covariance matrix, implemented by the derived class
   * @param covariance  current covariance matrix
   * @returns the the pruned covariance matrix
   */
  virtual Matrix getPrunedCovariance(const Matrix& covariance) = 0;

  /**
   * @brief Corrects the nominal state using the provided error state, implemented by the derived class
   * @param x_error   current error state
   */
  virtual void correct(const Vector& x_error) = 0;

  /**
   * @brief Updates the state indices e.g. if another clone is removed
   * @param new_id    new clone state id
   * @param shift_state
   */
  virtual void updateIndices(const uint new_id,  const uint shift_state) { clone_state_id_ = new_id; }

  /**
   * Returns the clone state id (=unique identifier of the clone)
   */
  uint getCloneStateId() const { return clone_state_id_; }

  /**
   * Returns the error state length
   */
  uint getErrorStateLength() const { return error_state_length_; }

protected:
  uint clone_state_id_;
  uint error_state_length_;
};

}  // namespace rio
