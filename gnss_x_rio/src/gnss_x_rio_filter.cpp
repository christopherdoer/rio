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

#include <gnss_x_rio/gnss_x_rio_filter.h>

using namespace rio;

bool GnssXRioFilter::addGnssStateClone(const ros::Time& trigger_stamp)
{
  // clone already in filter state --> remove it!
  std::vector<uint> states_to_be_removed;
  for (const auto& clone : clones_)
  {
    if (auto gnss_clone = std::dynamic_pointer_cast<GnssClone>(clone))
      states_to_be_removed.emplace_back(clone->getCloneStateId());
  }

  for (const auto& state_id : states_to_be_removed) removeClone(state_id);

  GnssCloneState gnss_state;
  gnss_state.time_stamp         = time_stamp_;
  gnss_state.trigger_time_stamp = trigger_stamp;
  gnss_state.p_n_b              = nav_sol_.getPosition_n_b();
  gnss_state.v_n_b              = nav_sol_.v_n_b;
  addClone(std::shared_ptr<CloneBase>(new GnssClone(clones_.size(), gnss_state)));

  return true;
}

bool GnssXRioFilter::getGnssClone(GnssClone& gnss_clone) const
{
  if (clones_.size() > 0)
  {
    for (const auto& clone : clones_)
    {
      if (auto gnss_clone_ = std::dynamic_pointer_cast<GnssClone>(clone))
      {
        gnss_clone = *gnss_clone_;
        return true;
      }
    }
  }
  return false;
}

bool GnssXRioFilter::updateGnssPosition(const GnssClone& gnss_clone,
                                        const Vector3 dpos_n,
                                        const Vector3& sigma_p,
                                        const Real& outlier_rejection_thresh)
{
  Matrix H                                              = Matrix::Zero(3, getCovarianceMatrix().cols());
  H.block(0, gnss_clone.getStateIdx().position(), 3, 3) = Matrix3::Identity(3, 3);

  const Vector3 r     = gnss_clone.getGnssCloneState().p_n_b - dpos_n;
  const Vector R_diag = (sigma_p).array().square();
  return updateWithOutlierRejection(r, H, R_diag.asDiagonal(), 3, outlier_rejection_thresh, "Outlier GPS position!");
}

bool GnssXRioFilter::updateGnssVelocity(const GnssClone& gnss_clone,
                                        const Vector3& v_n_gpnss,
                                        const Vector3& sigma_v,
                                        const Real& outlier_rejection_thresh)
{
  Matrix H                                              = Matrix::Zero(3, getCovarianceMatrix().cols());
  H.block(0, gnss_clone.getStateIdx().velocity(), 3, 3) = Matrix3::Identity(3, 3);

  const Vector3 r     = gnss_clone.getGnssCloneState().v_n_b - v_n_gpnss;
  const Vector R_diag = (sigma_v).array().square();

  return updateWithOutlierRejection(r, H, R_diag.asDiagonal(), 3, outlier_rejection_thresh, "Outlier GPS velocity!");
}
