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

#include <x_rio/Float64Stamped.h>

#include <x_rio/yaw_aiding_manhattan_world.h>
#include <radar_ego_velocity_estimator/radar_point_cloud.h>

using namespace rio;

YawAidingManhattanWorld::YawAidingManhattanWorld(ros::NodeHandle& nh)
{
  pub_filtered_     = nh.advertise<sensor_msgs::PointCloud2>("yaw_aiding_filtered", 1);
  pub_init_         = nh.advertise<sensor_msgs::PointCloud2>("yaw_aiding_init", 1);
  pub_raw_yaw_meas_ = nh.advertise<x_rio::Float64Stamped>("yaw_measurement", 1);
  pub_raw_yaw_filter_err_ = nh.advertise<x_rio::Float64Stamped>("filter_err_measurement", 1);
}

bool YawAidingManhattanWorld::update(const sensor_msgs::PointCloud2& radar_scan_msg,
                                     const FullRadarCloneState& radar_filter_state,
                                     Real& yaw_m,
                                     sensor_msgs::PointCloud2& yaw_inlier)
{
  if (!initialized_)
    init(radar_scan_msg, radar_filter_state);
  if (initialized_)
  {
    RadarDetections detections_filtered;
    if (getFilteredDetections(radar_scan_msg, radar_filter_state, detections_filtered))
    {
      std::pair<Real, RadarDetection> best_fit{2 * M_PI, RadarDetection()};
      for (const auto& detection : detections_filtered.detections)
      {
        const Real diff =
            std::fabs(math_helper::wrapToCentered(detection.azimuth_n + M_PI - gamma_manhattan_, M_PI / 4));

        if (diff < best_fit.first)
          best_fit = std::pair<Real, RadarDetection>(diff, detection);
      }

      ROS_DEBUG_STREAM(kPrefix << " min diff: " << best_fit.first);

      if (best_fit.first < angles::from_degrees(config_.yaw_aiding_yaw_inlier_thresh_deg))
      {
        Real azi_stab, ele_stab, r_stab;
        math_helper::cartesianToSpherical(best_fit.second.p_stab, r_stab, azi_stab, ele_stab);
        const Real yaw_m_shifted = math_helper::wrapToPositive(-azi_stab + gamma_manhattan_, 2 * M_PI);
        const Real yaw_filter = math_helper::wrapToPositive(radar_filter_state.nav_sol.getEuler_n_b().yaw(), 2 * M_PI);

        const int k               = static_cast<int>(std::round((yaw_filter - yaw_m_shifted) / (M_PI / 2)));
        const Real yaw_m_positive = math_helper::wrapToPositive(yaw_m_shifted + k * M_PI / 2, 2 * M_PI);
        yaw_m                     = math_helper::wrapToCentered(yaw_m_positive, M_PI);

        ROS_DEBUG("best fit diff: %0.2f, azi_stab: %0.2f, yaw_m_shifted: %0.2f, k: %d, yaw_filter: %0.2f, "
                  "yaw_m_positive: "
                  "%0.2f",
                  angles::to_degrees(best_fit.first),
                  angles::to_degrees(azi_stab),
                  angles::to_degrees(yaw_m_shifted),
                  k,
                  angles::to_degrees(yaw_filter),
                  angles::to_degrees(yaw_m_positive));

        RadarDetections inlier;
        inlier.nav_sol = radar_filter_state.nav_sol;
        inlier.detections.emplace_back(best_fit.second);
        convertToPcl(inlier, radar_scan_msg.header, yaw_inlier);

        x_rio::Float64Stamped yaw_msg;
        yaw_msg.header.stamp = radar_filter_state.time_stamp;
        yaw_msg.header.frame_id = "world";
        yaw_msg.data = yaw_m;
        pub_raw_yaw_meas_.publish(yaw_msg);

        x_rio::Float64Stamped err_msg;
        err_msg.header.stamp = radar_filter_state.time_stamp;
        err_msg.header.frame_id = "world";
        err_msg.data = yaw_m - yaw_filter;
        pub_raw_yaw_filter_err_.publish(err_msg);

        return true;
      }
    }
  }
  return false;
}

bool YawAidingManhattanWorld::init(const sensor_msgs::PointCloud2& radar_scan_msg,
                                   const FullRadarCloneState& radar_filter_state)
{
  RadarDetections detections_filtered;
  if (getFilteredDetections(radar_scan_msg, radar_filter_state, detections_filtered))
  {
    if (start_init_ == ros::TIME_MIN)
      start_init_ = radar_filter_state.time_stamp;
    for (const auto& detection : detections_filtered.detections)
      candidates_.emplace_back(
          CandidateAngle(math_helper::wrapToPositive(angles::to_degrees(detection.azimuth_n), 90.), detection));

    ++N_init_scans_;

    // try to init
    if (N_init_scans_ > config_.yaw_aiding_N_init)
    {
      // retransform --> body radar extrinsics should recalibrated by now
      for (auto& candidate : candidates_)
      {
        const Isometry T_b_r = radar_filter_state.T_b_r;

        candidate.radar_detection.p_body = T_b_r.linear() * candidate.radar_detection.p_radar;
        const Vector3 p_stab_n           = candidate.radar_detection.C_n_b * candidate.radar_detection.p_body;

        Real r, azi, ele;
        math_helper::cartesianToSpherical(p_stab_n, r, azi, ele);
        candidate.angle = math_helper::wrapToPositive(angles::to_degrees(azi), 360.);
      }

      // init
      Vector angles = Vector::Zero(candidates_.size(), 1);
      for (uint i = 0; i < candidates_.size(); ++i) angles[i] = candidates_.at(i).angle;

      const Vector angle_hist_raw = getCandidateHistogram(candidates_);
      std::pair<uint, Real> max_hist(0, -1);
      for (uint i = 0; i < angle_hist_raw.size(); ++i)
        if (angle_hist_raw[i] > max_hist.second)
          max_hist = {i, angle_hist_raw[i]};

      {
        // shift angles
        auto candidates_shifted = candidates_;
        for (auto& candidate : candidates_shifted) candidate.angle = candidate.angle - max_hist.first + 45.0;
        const Vector angle_hist = getCandidateHistogram(candidates_shifted);

        const Vector g = getConvKernel();
        Vector conv;

        if (convolve(angle_hist, g, math_helper::ConvolveType::VALID, conv))
        {
          std::pair<uint, Real> max_conv(0, -1);
          for (uint i = 0; i < conv.size(); ++i)
            if (conv[i] > max_conv.second)
              max_conv = {i, conv[i]};
          ROS_INFO_STREAM(kPrefix << "Conv peak: " << max_conv.second);
          if (max_conv.second > config_.yaw_aiding_min_N_peak)
          {
            const auto shift                   = max_conv.first + int(config_.yaw_aiding_N_gaussian / 2);
            const Real gamma_manhattan_deg_raw = math_helper::wrapToCentered(shift + max_hist.first - 45., 45.);

            std::vector<Real> inlier_candidates;
            RadarDetections init_detections;
            for (const auto& candidate : candidates_)
            {
              const auto a = math_helper::wrapToCentered(candidate.angle - gamma_manhattan_deg_raw, 45);
              if (std::fabs(a) < config_.yaw_aiding_init_inlier_thresh_deg)
              {
                inlier_candidates.emplace_back(a);
                init_detections.detections.emplace_back(candidate.radar_detection);
              }
            }
            const Real gamma_manhattan_deg =
                gamma_manhattan_deg_raw +
                std::accumulate(inlier_candidates.begin(), inlier_candidates.end(), 0.0) / inlier_candidates.size();

            ROS_INFO("Gamma Manhattan Raw = %0.2f; Gamma Manhattan refined %0.2f @ yaw = %0.2f, within interval of "
                     "%0.2f seconds",
                     gamma_manhattan_deg_raw,
                     gamma_manhattan_deg,
                     radar_filter_state.nav_sol.getEuler_n_b().to_degrees().z(),
                     (radar_filter_state.time_stamp).toSec() - start_init_.toSec());

            initialized_     = true;
            gamma_manhattan_ = angles::from_degrees(gamma_manhattan_deg);

            sensor_msgs::PointCloud2 msg_pcl_init;
            convertToPcl(init_detections, radar_scan_msg.header, msg_pcl_init, false);
            msg_pcl_init.header.frame_id = frame_id_;
            pub_init_.publish(msg_pcl_init);
            return true;
          }
        }
      }
    }
  }

  return false;
}

bool YawAidingManhattanWorld::getFilteredDetections(const sensor_msgs::PointCloud2& radar_scan_msg,
                                                    const FullRadarCloneState& radar_state,
                                                    RadarDetections& detections_filtered)
{
  auto radar_scan(new pcl::PointCloud<reve::RadarPointCloudType>);

  if (pcl2msgToPcl(radar_scan_msg, *radar_scan))
  {
    if (radar_state.nav_sol.v_n_b.head(2).norm() > config_.yaw_aiding_min_v_xy)
    {
      RadarDetections detections_raw(*radar_scan, radar_state);
      detections_filtered.nav_sol = detections_raw.nav_sol;

      // filter
      for (auto detection : detections_raw.detections)
      {
        const bool range_valid = detection.r > config_.yaw_aiding_min_dist && detection.r < config_.yaw_aiding_max_dist;
        const bool snr_valid   = detection.snr > config_.yaw_aiding_min_snr_detection;

        Real r, azi, ele;
        math_helper::cartesianToSpherical(detection.p_stab_n, r, azi, ele);
        const bool elevation_valid =
            std::fabs(std::fabs(angles::to_degrees(ele)) - 90) < config_.yaw_aiding_ele_thresh_deg;

        Real r_b, azi_b, ele_b;
        math_helper::cartesianToSpherical(detection.p_body, r_b, azi_b, ele_b);
        const bool azimuth_v_x_valid = std::fabs(radar_state.nav_sol.v_n_b.x()) > config_.yaw_aiding_min_v_xy &&
                                       (30 < angles::to_degrees(std::fabs(azi_b)) < 150);
        const bool azimuth_v_y_valid = std::fabs(radar_state.nav_sol.v_n_b.y()) > config_.yaw_aiding_min_v_xy &&
                                       (angles::to_degrees(std::fabs(azi_b)) < 60);

        if (range_valid && snr_valid && elevation_valid && (azimuth_v_x_valid || azimuth_v_y_valid))
        {
          detection.azimuth_n   = azi;
          detection.elevation_n = ele;
          detections_filtered.detections.emplace_back(detection);
        }
      }

      sensor_msgs::PointCloud2 msg_pcl_filtered;
      convertToPcl(detections_filtered, radar_scan_msg.header, msg_pcl_filtered);
      pub_filtered_.publish(msg_pcl_filtered);

      return true;
    }
  }

  return false;
}

void YawAidingManhattanWorld::convertToPcl(const RadarDetections& detections,
                                           const std_msgs::Header& header,
                                           sensor_msgs::PointCloud2& pcl_msg,
                                           const bool use_p_r) const
{
  pcl::PointCloud<pcl::PointXYZI> point_cloud_xyzi;

  for (auto detection : detections.detections)
  {
    pcl::PointXYZI p;
    if (use_p_r)
      p.getVector3fMap() = Eigen::Vector3f(detection.p_radar.x(), detection.p_radar.y(), detection.p_radar.z());
    else
      p.getVector3fMap() = Eigen::Vector3f(detection.p_ros.x(), detection.p_ros.y(), detection.p_ros.z());

    p._PointXYZI::intensity = detection.snr;
    point_cloud_xyzi.push_back(p);
  }

  pcl::PCLPointCloud2 tmp;
  pcl::toPCLPointCloud2(point_cloud_xyzi, tmp);
  pcl_conversions::fromPCL(tmp, pcl_msg);
  pcl_msg.header = header;
}

Vector360 YawAidingManhattanWorld::getCandidateHistogram(const Candidates& candidates)
{
  Vector360 hist = Vector360::Zero(360, 1);

  for (const auto& candidate : candidates)
  {
    const int idx = int(math_helper::wrapToPositive(candidate.angle, 359) + 0.5);
    hist[idx] += 1;
  }

  return hist;
}

Vector YawAidingManhattanWorld::getConvKernel()
{
  Vector v = Vector::Zero(271, 1);
  v[0]     = 1;
  v[90]    = 1;
  v[180]   = 1;
  v[270]   = 1;

  Vector gaussian = Vector::Zero(int(config_.yaw_aiding_N_gaussian / 2) * 2 + 1, 1);
  for (uint i = 0; i < gaussian.size(); ++i)
  {
    const Real x = Real(i) - int(gaussian.size() / 2);
    gaussian[i]  = 1 / (config_.yaw_aiding_gaussian_sigma * std::sqrt(2 * M_PI)) *
                  std::exp(-0.5 * std::pow(x / config_.yaw_aiding_gaussian_sigma, 2));
  }
  Vector conv_kernel;
  convolve(v, gaussian, math_helper::ConvolveType::FULL, conv_kernel);

  return conv_kernel;
}
