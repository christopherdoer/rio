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

#include <set>
#include <rio_utils/radar_point_cloud.h>

using namespace rio;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPointCloudType,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, snr_db, snr_db)
                                  (float, noise_db,   noise_db)
                                  (float, v_doppler_mps,   v_doppler_mps)
                                  )


POINT_CLOUD_REGISTER_POINT_STRUCT (mmWaveCloudType,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, velocity, velocity))
// clang-format on

bool rio::pcl2msgToPcl(const sensor_msgs::PointCloud2& pcl_msg, pcl::PointCloud<RadarPointCloudType>& scan)
{
  // TODO: add support for ti_mmwave_rospkg clound type

  std::set<std::string> fields;
  std::string fields_str = "";

  for (const auto& field : pcl_msg.fields)
  {
    fields.emplace(field.name);
    fields_str += field.name + ", ";
  }

  if (fields.find("x") != fields.end() && fields.find("y") != fields.end() && fields.find("z") != fields.end() &&
      fields.find("snr_db") != fields.end() && fields.find("noise_db") != fields.end() &&
      fields.find("v_doppler_mps") != fields.end())
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pcl_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, scan);
    return true;
  }
  else if (fields.find("x") != fields.end() && fields.find("y") != fields.end() && fields.find("z") != fields.end() &&
           fields.find("intensity") != fields.end() && fields.find("velocity") != fields.end())
  {
    pcl::PointCloud<mmWaveCloudType> scan_mmwave;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pcl_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, scan_mmwave);

    scan.clear();
    for (const auto& p : scan_mmwave)
    {
      RadarPointCloudType p_;
      p_.x             = -p.y;
      p_.y             = p.x;
      p_.z             = p.z;
      p_.snr_db        = p.intensity;
      p_.v_doppler_mps = p.velocity;
      p_.range         = p.getVector3fMap().norm();
      p_.noise_db      = -1.;
      scan.push_back(p_);
    }
    return true;
  }
  else
  {
    ROS_ERROR_STREAM(
        "[pcl2msgToPcl]: Unsupported point cloud with fields: " << fields_str.substr(0, fields_str.size() - 2));
    return false;
  }
}

bool rio::pclToPcl2msg(pcl::PointCloud<RadarPointCloudType> scan, sensor_msgs::PointCloud2& pcl_msg)
{
  scan.height = 1;
  scan.width  = scan.size();

  pcl::PCLPointCloud2 tmp;
  pcl::toPCLPointCloud2<RadarPointCloudType>(scan, tmp);
  pcl_conversions::fromPCL(tmp, pcl_msg);
}
