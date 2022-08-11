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
#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <gnss_x_rio/gnss_x_rio_ros.h>

using namespace rio;

const std::string kNodeName = "rio";
const std::string kPrefix   = "[" + kNodeName + "]: ";

int main(int argc, char** argv)
{
  ros::init(argc, argv, kNodeName);
  ros::NodeHandle nh("~");

  GnssXRioRos rio(nh);

  rio.run();

  return 0;
}
