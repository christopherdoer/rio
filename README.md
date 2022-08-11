# RIO - Radar Inertial Odometry and Radar based Ego Velocity Estimation
Navigation in GNSS denied and visually degraded environments is still very challenging. 
Approaches based on visual sensors tend to fail in conditions such as darkness, direct sunlight, fog or smoke.
Therefore, we are using 4D mmWave FMCW radar sensors and inertial sensor data as these are not affected by such conditions.

***Highlights:***
- Robust and accurate navigation even in Degraded Visual and GNSS denied Environments
- Super fast: [x_rio](./x_rio) achieves runtimes ~125x faster than realtime on an [Intel NUC i7](https://www.intel.com/content/www/us/en/products/sku/130392/intel-nuc-kit-nuc7i7dnke/specifications.html) and ~21x on an [Up Core](https://up-shop.org/up-core-series.html) embedded computer
- Demonstrated for online navigation of drones even in confined indoor environments

## News
- 08/2022: [gnss_x_rio](./gnss_x_rio) is released adding global information from GNSS-measurements to [x_rio](./x_rio) and is based on our [paper](https://christopherdoer.github.io/publications/2022_02_Aeroconf2022).
- 03/2022: [x_rio](./x_rio) is released generalizing  [ekf_rio](./ekf_rio) and [ekf_yrio](./ekf_yrio) for multi radar sensor setups and providing a faster implementation using approximated radar clones as described in our [paper](https://christopherdoer.github.io/publication/2022_02_JGN2022). 
  The [paper datasets](https://christopherdoer.github.io/datasets/multi_radar_inertial_datasets_JGN2022) are also released and can be evaluated with a single [script](./x_rio/python/evaluate_jgn2022_datasets.py).
- 06/2021: The radar inertial datasets with pseudo ground truth used in our [Yaw aided Radar Inertial Odometry](https://christopherdoer.github.io/publication/2021_05_ICINS2021) paper are released: [radar_inertial_datasets_icins_2021](https://christopherdoer.github.io/datasets/icins_2021_radar_inertial_odometry). 
  Both ekf_rio and ekf_yrio can be evaluated on the whole dataset with a single [script](ekf_yrio/python/icins_2021_evaluation.py).
- 05/2021: Initial release of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.

## Introduction
RIO is a toolbox for EKF-based Radar Inertial Odometry.
RIO features the following packages:
- [gnss_x_rio](./gnss_x_rio) (recommended): Adds global information from GNSS-measurements to [x_rio](./x_rio).
- [x_rio](./x_rio) (recommended): An EKF-based Multi-Radar Inertial Odometry Pipeline with online calibration of the radar sensor extrinsics and yaw aiding using Manhattan world assumptions. Can be used with a single or multi radar setups. 
- [ekf_rio](./ekf_rio) (deprecated): An EKF-based Radar Inertial Odometry Pipeline with online calibration of the radar sensor extrinsics
- [ekf_yrio](./ekf_yrio) (deprecated): An extension of ekf_rio featuring yaw aiding based on Manhattan world assumptions 

Checkout the README files of the individual packages for more details.

## Demos

### Autonomous Radar Inertial Drone Navigation even in Dense Fog ([x_rio](./x_rio))   
[![Autonomous Radar Inertial Drone Navigation even in Dense Fog](http://img.youtube.com/vi/FjsV1TouY-A/0.jpg)](https://www.youtube.com/watch?v=FjsV1TouY-A "Autonomous Radar Inertial Drone Navigation even in Dense Fog")

### Autonomous Indoor Drone Flight using Yaw aided Radar Inertial Odometry ([ekf_yrio](./ekf_yrio))   
[![Autonomous Indoor Drone Flights using Yaw aided Radar Inertial Odometry](http://img.youtube.com/vi/KhWPqMC6gSE/0.jpg)](http://www.youtube.com/watch?v=KhWPqMC6gSE "Autonomous Indoor Drone Flights using Yaw aided Radar Inertial Odometry")

### Indoor Demo and Evaluation of Yaw aided Radar Inertial Odometry ([ekf_yrio](./ekf_yrio))   
[![Autonomous UAV Flights using Radar Inertial Odometry](http://img.youtube.com/vi/EIcBMo1sM_g/0.jpg)](http://www.youtube.com/watch?v=EIcBMo1sM_g "Autonomous UAV Flights using Radar Inertial Odometry")

### Autonomous UAV Flights using Radar Inertial Odometry ([ekf_rio](./ekf_rio))
[![Autonomous UAV Flights using Radar Inertial Odometry](http://img.youtube.com/vi/8DofG1iXHAE/0.jpg)](http://www.youtube.com/watch?v=8DofG1iXHAE "Autonomous UAV Flights using Radar Inertial Odometry")


## References

If you use our implementation for your academic research, please cite the related paper:

***gnss_x_rio:***
~~~[bibtex]
@INPROCEEDINGS{DoerAeroConf2022,
    author = {Doer, Christopher and Atman, Jamal and Trommer, Gert F.},
    title = { GNSS aided Radar Inertial Odometry for UAS Flights in Challenging Conditions },
    booktitle={2022 IEEE Aerospace Conference (AeroConf}, 
    year={2022}
}
~~~

***x_rio:***
~~~[bibtex]
@INPROCEEDINGS{DoerJGN2022,
    author = {Doer, Christopher and Trommer, Gert F.},
    year = {2022},
    month = {02},
    pages = {329-339},
    title = {x-RIO: Radar Inertial Odometry with Multiple Radar Sensors and Yaw Aiding},
    volume = {12},
    journal = {Gyroscopy and Navigation}}
~~~

***ekf_yrio:***
~~~[bibtex]
@INPROCEEDINGS{DoerICINS2021,
  author={Doer, Christopher and Trommer, Gert F.},
  booktitle={2021 28th Saint Petersburg International Conference on Integrated Navigation Systems (ICINS)}, 
  title={Yaw aided Radar Inertial Odometry uisng Manhattan World Assumptions}, 
  year={2021},
  pages={1-10}}
~~~

***ekf_rio:***
~~~[bibtex]
@INPROCEEDINGS{DoerENC2020,
  author={Doer, Christopher and Trommer, Gert F.},
  booktitle={2020 European Navigation Conference (ENC)}, 
  title={Radar Inertial Odometry with Online Calibration}, 
  year={2020},
  pages={1-10},
  doi={10.23919/ENC48637.2020.9317343}}
~~~
~~~[bibtex]
@INPROCEEDINGS{DoerMFI2020,
  author={Doer, Christopher and Trommer, Gert F.},
  booktitle={2020 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI)}, 
  title={An EKF Based Approach to Radar Inertial Odometry}, 
  year={2020},
  pages={152-159},
  doi={10.1109/MFI49285.2020.9235254}}
~~~


## Getting Started
RIO supports:
- Ubuntu 16.04 and ROS Kinetic
- Ubuntu 18.04 and ROS Melodic
- Ubuntu 20.04 and ROS Noetic

RIO depends on:
- [catkin_simple](https://github.com/catkin/catkin_simple.git)
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) (for convenience)
- Pull dependencies via submodules, run once: ` git submodule update --init --recursive `. This will setup the following two submodules: 
  - [reve](https://github.com/christopherdoer/reve)
  - [rpg_trajectory_evaluation](https://github.com/christopherdoer/rpg_trajectory_evaluation) (optional, for comprehensive evaluation)
    To use the evaluation scripts, the following dependencies are required:
    - sudo apt-get install texlive-latex-extra texlive-fonts-recommended dvipng cm-super
    - pip2 install -U PyYAML colorama ruamel.yaml==0.15.0

**Build in Release is highly recommended**:
~~~[shell]
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
~~~

We provide some demo datasets which can be run using the demo launch files of each package. 
Check out the ***Getting Started*** section of the READMEs in the individual packages for more details. 


## License
The source code is released under the [GPLv3](http://www.gnu.org/licenses/) license.
