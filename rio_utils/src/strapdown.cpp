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

#include "rio_utils/strapdown.h"

#include <iostream>

namespace rio
{
Strapdown::Strapdown(const double local_gravity) : local_gravity_(0, 0, local_gravity) {}

NavigationSolution
Strapdown::propagate(const NavigationSolution nav_sol_prev, const Vector3 a_b_ib, const Vector3 w_b_ib, const Real dt)
{
  const Matrix3 C_n_b_prev = nav_sol_prev.getC_n_b();
  const Vector3 v_n_b_prev = nav_sol_prev.v_n_b;

  // propagate attitude with forth order runge kutta
  const Vector4 zero_omega(0, w_b_ib.x(), w_b_ib.y(), w_b_ib.z());
  const Quaternion q_n_b_prev = nav_sol_prev.getQuaternion_n_b();
  const Vector4 q_n_b_vec(q_n_b_prev.w(), q_n_b_prev.x(), q_n_b_prev.y(), q_n_b_prev.z());

  const Vector4 k_1(0.5 * getQLeftMatrix(q_n_b_vec) * zero_omega);
  const Vector4 k_2(0.5 * getQLeftMatrix(q_n_b_vec + k_1 * dt / 2) * zero_omega);
  const Vector4 k_3(0.5 * getQLeftMatrix(q_n_b_vec + k_2 * dt / 2) * zero_omega);
  const Vector4 k_4(0.5 * getQLeftMatrix(q_n_b_vec + k_3 * dt) * zero_omega);

  const Vector4 new_q_vec = q_n_b_vec + (k_1 + 2 * k_2 + 2 * k_3 + k_4) * dt / 6;
  const Quaternion q_n_b(new_q_vec[0], new_q_vec[1], new_q_vec[2], new_q_vec[3]);

  // propagate velocity using Simpson's rule
  const Quaternion q_b1_b = q_n_b_prev * q_n_b.inverse();
  const Matrix3 C_b1_b    = q_b1_b.toRotationMatrix();

  // clang-format off
  const Vector3 s_l = (a_b_ib
                       + 4 * (a_b_ib + 0.5 * (C_b1_b - Matrix3::Identity()) * a_b_ib)
                       + (a_b_ib + (C_b1_b - Matrix3::Identity()) * a_b_ib)) *
                      dt / 6;
  // clang-format on

  Vector3 v_n_b = nav_sol_prev.v_n_b + C_n_b_prev * s_l + local_gravity_ * dt;

  // propagate position using chained Simpson's rule
  // clang-format off
  const Vector3 v_01 = (a_b_ib
                        + 4 * (a_b_ib + 0.25 * (C_b1_b - Matrix3::Identity()) * a_b_ib)
                        + (a_b_ib + 0.5 * (C_b1_b - Matrix3::Identity()) * a_b_ib))* dt / 12;
  // clang-format on

  const Vector3 y_l    = (4 * v_01 + s_l) * dt / 6;
  const Vector3 p_n1_n = v_n_b_prev * dt + C_n_b_prev * y_l + 0.5 * local_gravity_ * dt * dt;
  const Vector3 p_n_b  = Vector3(nav_sol_prev.getPosition_n_b() + p_n1_n);

  return NavigationSolution(p_n_b, q_n_b, v_n_b);
}

Matrix4 Strapdown::getQLeftMatrix(const Vector4& v)
{
  Matrix4 m;
  // clang-format off
  m << v[0],  -v[1],  -v[2],  -v[3],
       v[1],   v[0],  -v[3],   v[2],
       v[2],   v[3],   v[0],  -v[1],
       v[3],  -v[2],   v[1],   v[0];
  // clang-format on

  return m;
}

}  // namespace rio
