# ekf_yrio: Extended Kalman Filter based Yaw aided Radar Inertial Odmetry

This package extends [ekf_rio](../ekf_rio) with yaw aiding based on Manhattan world assumptions for indoor environments.
We used an instantaneous approach which can provide yaw aiding without scan matching.
In contrast to [ekf_rio](../ekf_rio), ekf_yrio estimates all IMU offsets (even the yaw rate offset) which results in
 accurate state estimation even on long indoor datasets.

## Cite
If you use ekf_yrio or the provided datasets for your academic research, please cite our related paper:

~~~[bibtex]
@INPROCEEDINGS{DoerICINS2021,
  author={Doer, Christopher and Trommer, Gert F.},
  booktitle={2021 28th Saint Petersburg International Conference on Integrated Navigation Systems (ICINS)}, 
  title={Yaw aided Radar Inertial Odometry uisng Manhattan World Assumptions}, 
  year={2021},
  pages={1-10}
~~~

## Paper Datasets
All carried and flight datasets presented in our Paper "Yaw aided Radar Inertial Odometry uisng Manhattan World
 Assumptions" are available online: [radar_inertial_datasets_icins_2021](https://bwsyncandshare.kit.edu/s/75NYFkskLTfrGeG). These datasets also include pseudo ground truth which has been created using VINS
  with loop closure.

## Demo Result

### Indoor Dataset "Carried 2"

The image below shows the estimation result of ekf_yrio for the indoor dataset "carried_2" (part of
  [radar_inertial_datasets_icins_2021](https://bwsyncandshare.kit.edu/s/75NYFkskLTfrGeG)).   
This dataset covers a trajectory length of 451m over a period of 392s whereas the start and end pose are equal.
 ekf_yrio achieved a final position error of 0.14% of the distance traveled. 
 Processing of the whole dataset took just 4.32s with an Intel NUC i7 and 36s with an UpCore embedded computer
  
![image](res/carried_2_ground_plan.jpg)

## Getting Started

Run ekf_yrio with the "carried_2" dataset in rosbag mode and start rviz for visualization:

~~~[shell]
roslaunch ekf_yrio demo_datasets_ekf-yrio_rosbag.launch
~~~

Run without rviz:

~~~[shell]
roslaunch ekf_yrio demo_datasets_ekf-yrio_rosbag.launch enable_rviz:=False
~~~

Run in online mode:

~~~[shell]
roslaunch ekf_yrio demo_datasets_ekf-yrio_ros.launch
rosbag play --clock radar_inertial_datasets_icins_2021_carried_2.bag
~~~

Run the [radar_inertial_datasets_icins_2021](https://bwsyncandshare.kit.edu/s/75NYFkskLTfrGeG) dataset:

~~~[shell]
roslaunch ekf_yrio icins2021_datasets_ekf-yrio_rosbag.launch dataset_dir:=<path_to_dataset> rosbag:=carried_1.bag
~~~

## Nodes

Our ekf_yrio implementation provides two nodes for ROS interfacing:
- ***ros_node:*** Subscribes to all topics and does online processing
- ***rosbag_node:*** Reads a rosbag and runs ekf_yrio at maximum speed

A set of demo parameters is given in [ekf_yrio_default](./config/ekf_yrio_default.yaml).
Most of the parameters can be changed online using rqt_reconfigure. Further documentation of the parameters can be found using the tooltip text in rqt_reconfigure
and in the python files, see [cfg](./cfg).