# Demo Datasets

We provide some demo datasets to try out our radar inertial pipelines. 
The individual datasets are given in the corresponding directories including READMEs.

There are two different types of point clouds:
- RIO point cloud format: used by all our own datasets
- ti_mmwave_rospkg format: point clouds recorded with the ti_mmwave_rospkg driver

# RIO Point Cloud Format
The point cloud point type is sketched below, for an example implementation see [radar_point_cloud
](https://github.com/christopherdoer/reve/blob/master/radar_ego_velocity_estimator/src/radar_point_cloud.cpp):

~~~
struct RadarPointCloudType
{
  PCL_ADD_POINT4D;      // position in [m]
  float snr_db;         // CFAR cell to side noise ratio in [dB]
  float v_doppler_mps;  // Doppler velocity in [m/s]
  float noise_db;       // CFAR noise level of the side of the detected cell in [dB]
  float range;          // range in [m]
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
                                  
pcl::PointCloud<RadarPointCloudType> your_pcl;
~~~

# ti_mmwave_rospkg Point Cloud Format
The point cloud point type is sketched below, for an example implementation see [radar_point_cloud
](https://github.com/christopherdoer/reve/blob/master/radar_ego_velocity_estimator/src/radar_point_cloud.cpp):

~~~
POINT_CLOUD_REGISTER_POINT_STRUCT (mmWaveCloudType,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, velocity, velocity))
pcl::PointCloud<mmWaveCloudType> your_pcl;
~~~
