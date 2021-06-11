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

#include <sensor_msgs/FluidPressure.h>

namespace rio
{
/**
 * @brief The BaroAltimeter class converts fluid pressure into relative height measurements
 */
class BaroAltimeter
{
public:
  /**
   * @brief Pressure to negative height conversion
   * @param pressure_msg   fluid pressure ros message containing the air pressure in Pascal
   * @returns the negative height in [m]
   */
  Real calculate_rel_neg_height(const sensor_msgs::FluidPressure& pressure_msg)
  {
    return (R * T_0) / (g_0 * M) * std::log(pressure_msg.fluid_pressure / P_0);
  }

private:
  const std::string kPrefix = "[BaroAltimeter]: ";
  const Real g_0            = 9.80665;
  const Real M              = 0.0289644;
  const Real R              = 8.3144598;

  const Real T_0 = 288.15;
  const Real P_0 = 101320;
};

}  // namespace rio
