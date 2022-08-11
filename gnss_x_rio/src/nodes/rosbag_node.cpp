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

#include <rio_utils/ros_helper.h>
#include <gnss_x_rio/gnss_x_rio_ros.h>

using namespace rio;

const std::string kNodeName = "rio";
const std::string kPrefix   = "[" + kNodeName + "]: ";

int main(int argc, char** argv)
{
  ros::init(argc, argv, kNodeName);
  ros::NodeHandle nh("~");

  GnssXRioRos rio(nh);

  double bag_start = -1.0;
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "bag_start", bag_start);

  double bag_duration = -1.0;
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "bag_duration", bag_duration);
  if (bag_duration <= 0)
    bag_duration = 1.0e6;

  double sleep_ms = -1.0;
  getRosParameter(nh, kPrefix, RosParameterType::Optional, "sleep_ms", sleep_ms);

  std::string rosbag_path = "";
  if (getRosParameter(nh, kPrefix, RosParameterType::Required, "rosbag_path", rosbag_path))
    rio.runFromRosbag(rosbag_path, bag_start, bag_duration, sleep_ms);

  return 0;
}
