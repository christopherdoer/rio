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

namespace rio
{
struct EkfRioFilterStateIdx
{
  const uint position                = 0;
  const uint velocity                = 3;
  const uint attitude                = 6;
  const uint bias_acc                = 9;
  const uint bias_gyro               = 12;
  const uint bias_alt                = 15;
  const uint l_b_r                   = 16;
  const uint eul_b_r                 = 19;
  const uint base_state_length       = 22;
  const uint prob_noise_state_length = 15;
  const uint sc_position             = 22 + 0;
  const uint sc_velocity             = 22 + 3;
  const uint sc_attitude             = 22 + 6;
  const uint sc_bias_gyro            = 22 + 9;
  const uint sc_l_b_r                = 22 + 12;
  const uint sc_eul_b_r              = 22 + 15;
  const uint radar_clone_length      = 18;
};

struct RadarCloneState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  ros::Time time_stamp;
  ros::Time trigger_time_stamp;
  NavigationSolution nav_sol;
  Vector3 offset_gyro = Vector3(-1, -1, -1);
  Isometry T_b_r;
};

struct Offsets
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Vector3 acc  = Vector3(-1, -1, -1);
  Vector3 gyro = Vector3(-1, -1, -1);
  Real alt     = -1;
};

struct SystemNoisePsd
{
  template <class ConfigContainingEkfRioConfig>
  bool configure(ConfigContainingEkfRioConfig& config)
  {
    acc       = config.noise_psd_a;
    gyro      = angles::from_degrees(config.noise_psd_w_deg);
    bias_acc  = config.noise_psd_b_a;
    bias_gyro = angles::from_degrees(config.noise_psd_b_w_deg);
    bias_alt  = config.noise_psd_b_alt;

    return true;
  }

  Matrix getQ(Real T)
  {
    Matrix Q                       = Matrix::Zero(13, 13);
    Q.block(0, 0, 3, 3).diagonal() = Vector::Ones(3) * std::pow(acc, 2) * T;
    Q.block(3, 3, 3, 3).diagonal() = Vector::Ones(3) * std::pow(gyro, 2) * T;
    Q.block(6, 6, 3, 3).diagonal() = Vector::Ones(3) * std::pow(bias_acc, 2) / T;
    Q.block(9, 9, 3, 3).diagonal() = Vector::Ones(3) * std::pow(bias_gyro, 2) / T;
    Q(12, 12)                      = std::pow(bias_alt, 2) / T;

    return Q;
  }

  Matrix getG(const Matrix3& C_n_b, const EkfRioFilterStateIdx& error, const uint error_state_length) const
  {
    Matrix G = Matrix::Zero(error_state_length, 13);

    G.block(error.velocity, 0, 3, 3)  = C_n_b;
    G.block(error.attitude, 3, 3, 3)  = C_n_b;
    G.block(error.bias_acc, 6, 3, 3)  = Matrix3::Identity();
    G.block(error.bias_gyro, 9, 3, 3) = Matrix3::Identity();
    G(error.bias_alt, 12)             = 1;

    return G;
  }

  Real acc       = -1;
  Real gyro      = -1;
  Real bias_acc  = -1;
  Real bias_gyro = -1;
  Real bias_alt  = -1;
};

struct InitStruct
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  template <class Config>
  bool configure(const EkfRioFilterStateIdx& error, Config& config)
  {
    p_0     = Vector3(config.p_0_x, config.p_0_y, config.p_0_z);
    v_0     = Vector3(config.v_0_x, config.v_0_y, config.v_0_z);
    yaw_0   = angles::from_degrees(config.yaw_0_deg);
    b_a_0   = Vector3(config.b_0_a_x, config.b_0_a_y, config.b_0_a_z);
    b_w_0   = Vector3(angles::from_degrees(config.b_0_w_x_deg),
                    angles::from_degrees(config.b_0_w_y_deg),
                    angles::from_degrees(config.b_0_w_z_deg));
    b_alt_0 = config.b_0_alt;
    l_b_r_0 = Vector3(config.l_b_r_x, config.l_b_r_y, config.l_b_r_z);
    q_b_r_0 = Quaternion(config.q_b_r_w, config.q_b_r_x, config.q_b_r_y, config.q_b_r_z);

    P_kk_0 = Matrix::Zero(error.base_state_length, error.base_state_length);
    P_kk_0.block(error.position, error.position, 3, 3).diagonal() = Vector3(1, 1, 1) * std::pow(config.sigma_p, 2);

    P_kk_0.block(error.velocity, error.velocity, 3, 3).diagonal() = Vector3(1, 1, 1) * std::pow(config.sigma_v, 2);

    P_kk_0.block(error.attitude, error.attitude, 3, 3).diagonal() =
        Vector3(angles::from_degrees(config.sigma_roll_pitch_deg),
                angles::from_degrees(config.sigma_roll_pitch_deg),
                angles::from_degrees(config.sigma_yaw_deg))
            .array()
            .pow(2);

    P_kk_0.block(error.bias_acc, error.bias_acc, 3, 3).diagonal() = Vector3(1, 1, 1) * std::pow(config.sigma_b_a, 2);

    P_kk_0.block(error.bias_gyro, error.bias_gyro, 3, 3).diagonal() =
        Vector3(1, 1, 1) * std::pow(angles::from_degrees(config.sigma_b_w_deg), 2);

    P_kk_0(error.bias_alt, error.bias_alt) = std::pow(config.sigma_b_alt, 2);

    P_kk_0.block(error.l_b_r, error.l_b_r, 3, 3).diagonal() =
        Vector3(config.sigma_l_b_r_x, config.sigma_l_b_r_y, config.sigma_l_b_r_z).array().pow(2);

    P_kk_0.block(error.eul_b_r, error.eul_b_r, 3, 3).diagonal() =
        Vector3(angles::from_degrees(config.sigma_eul_b_r_roll_deg),
                angles::from_degrees(config.sigma_eul_b_r_pitch_deg),
                angles::from_degrees(config.sigma_eul_b_r_yaw_deg))
            .array()
            .pow(2);

    gravity           = config.g_n;
    omega_calibration = config.calib_gyro;

    return true;
  }

  Vector3 p_0        = Vector3(0, 0, 0);
  Vector3 v_0        = Vector3(0, 0, 0);
  Real yaw_0         = 0.0;
  Vector3 b_a_0      = Vector3(0, 0, 0);
  Vector3 b_w_0      = Vector3(0, 0, 0);
  Real b_alt_0       = 0.0;
  Vector3 l_b_r_0    = Vector3(0, 0, 0);
  Quaternion q_b_r_0 = Quaternion(1, 0, 0, 0);

  Matrix P_kk_0;

  Real gravity           = 0;
  bool omega_calibration = true;
};

}  // namespace rio
