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

#include <Eigen/StdVector>

#include <rio_utils/data_types.h>
#include <radar_ego_velocity_estimator/radar_point_cloud.h>

#include <ekf_rio/data_types.h>

namespace rio
{
struct RadarDetection
{
  RadarDetection(const Real x, const Real y, const Real z, const Real snr, const RadarCloneState& radar_state) :
    snr{snr}
  {
    p_radar = Vector3(x, y, z);

    r         = p_radar.norm();
    azimuth   = std::atan2(y, x) - M_PI_2;
    elevation = std::atan2(std::sqrt(x * x + y * y), z) - M_PI_2;

    p_body   = radar_state.T_b_r.linear() * p_radar;  // - radar_state.T_b_r.translation();
    p_stab_n = radar_state.nav_sol.getC_n_b() * p_body;
    p_n      = radar_state.nav_sol.getC_n_b() * p_body + radar_state.nav_sol.getPosition_n_b();
    p_ros    = radar_state.nav_sol.getPoseRos() * radar_state.T_b_r * p_radar;

    NavigationSolution stab_tmp;
    stab_tmp.setEuler_n_b(
        EulerAngles(radar_state.nav_sol.getEuler_n_b().roll(), radar_state.nav_sol.getEuler_n_b().pitch(), 0));
    p_stab = stab_tmp.getC_n_b() * p_body;

    C_n_b = radar_state.nav_sol.getC_n_b();
  }

  RadarDetection() {}

  Real r         = 0.;
  Real azimuth   = 0.;
  Real elevation = 0.;
  Real snr       = 0.;

  Real azimuth_n   = 0;
  Real elevation_n = 0;

  Vector3 p_radar;
  Vector3 p_body;
  Vector3 p_stab_n;
  Vector3 p_stab;
  Vector3 p_n;
  Vector3 p_ros;

  Matrix3 C_n_b;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct RadarDetections
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RadarDetections(const pcl::PointCloud<reve::RadarPointCloudType>& scan, const RadarCloneState& radar_clone_state)
  {
    for (uint k = 0; k < scan.size(); ++k)
    {
      reve::RadarPointCloudType p = scan.at(k);
      detections.emplace_back(RadarDetection(p.x, p.y, p.z, p.snr_db, radar_clone_state));
    }
    nav_sol = radar_clone_state.nav_sol;
  }

  RadarDetections() {}

  std::vector<RadarDetection> detections;
  NavigationSolution nav_sol;
};

struct CandidateAngle
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CandidateAngle(const Real& angle, const RadarDetection& radar_detection) :
    angle{angle},
    radar_detection{radar_detection}
  {
  }
  CandidateAngle() {}

  Real angle = 0.;
  RadarDetection radar_detection;
};

typedef std::vector<CandidateAngle> Candidates;
typedef Eigen::Matrix<double, 360, 1> Vector360;

}  // namespace rio
