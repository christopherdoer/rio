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

#include <deque>
#include <ros/ros.h>

#include <rio_utils/data_types.h>

#include <gnss_x_rio/GnssStateConfig.h>

namespace rio
{
struct Observation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Observation(const ros::Time& stamp, const Real& quality, const Vector3& dpos) :
    stamp{stamp},
    quality{quality},
    dpos_x(dpos.x()),
    dpos_y(dpos.y()),
    dpos_z(dpos.z())
  {
  }
  ros::Time stamp;
  Real quality = 0;
  Real dpos_x;
  Real dpos_y;
  Real dpos_z;
};

class GnssState
{
public:
  /**
   * @brief Reconfigure callback
   * @param config   has to contain the fields of GnssStateConfig
   */
  template <class GnssStateConfig>
  void configure(GnssStateConfig& config);

  bool gnssStateValid(const ros::Time& stamp, const Real& quality, const Vector3& d_pos)
  {
    observations_.emplace_back(Observation(stamp, quality, d_pos));

    while ((observations_.back().stamp - observations_.front().stamp).toSec() > config_.gnss_window_s)
      observations_.pop_front();

    auto observations_old = observations_;
    observations_.clear();
    observations_.push_front(observations_old.back());
    Real distance = 0.;
    for (auto iter = observations_old.rbegin() + 1; iter != observations_old.rend(); ++iter)
    {
      distance += Vector3(iter->dpos_x - (iter - 1)->dpos_x,
                          iter->dpos_y - (iter - 1)->dpos_y,
                          iter->dpos_z - (iter - 1)->dpos_z)
                      .norm();
      observations_.push_front(*iter);
      if (distance > config_.gnss_window_m)
        break;
    }

    const uint expected_observations =
        (observations_.back().stamp - observations_.front().stamp).toSec() * config_.gnss_nominal_rate;

    if (observations_.size() < 0.9 * expected_observations)
      return false;

    Real score = 0.;
    for (const auto& observation : observations_)
    {
      score += observation.quality;
    }

    score /= observations_.size();

    return score < config_.gnss_score_thresh_valid;
  }

private:
  const std::string kPrefix = "[GnssState]: ";

  gnss_x_rio::GnssStateConfig config_;

  std::deque<Observation> observations_;
};

template <class GnssStateConfig>
void GnssState::configure(GnssStateConfig& config)
{
  config_.enable_gnss_state       = config.enable_gnss_state;
  config_.gnss_nominal_rate       = config.gnss_nominal_rate;
  config_.gnss_window_s           = config.gnss_window_s;
  config_.gnss_window_m           = config.gnss_window_m;
  config_.gnss_score_thresh_valid = config.gnss_score_thresh_valid;
}
}  // namespace rio
