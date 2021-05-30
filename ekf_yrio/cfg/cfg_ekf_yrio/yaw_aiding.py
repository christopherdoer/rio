#!/usr/bin/env python
PACKAGE = "ekf_yrio"

from dynamic_reconfigure.parameter_generator_catkin import *


def configure(gen):
    rio_yaw_aiding = gen.add_group("RIO Yaw Aiding")
    rio_yaw_aiding.add("enable_yaw_aiding", bool_t, 0, "", True)

    rio_yaw_aiding.add("yaw_aiding_min_dist", double_t, 0, "", 0.25, 0, 50)
    rio_yaw_aiding.add("yaw_aiding_max_dist", double_t, 0, "", 5.0, 0, 50)
    rio_yaw_aiding.add("yaw_aiding_ele_thresh_deg", double_t, 0, "", 5, 0, 100)
    rio_yaw_aiding.add("yaw_aiding_min_snr_detection", double_t, 0, "", 10, 0, 50)
    rio_yaw_aiding.add("yaw_aiding_min_v_xy", double_t, 0, "", 0.25, 0, 10)

    rio_yaw_aiding.add("yaw_aiding_N_init", int_t, 0, "", 150, 0, 5000)
    rio_yaw_aiding.add("yaw_aiding_min_N_peak", int_t, 0, "", 100, 0, 5000)
    rio_yaw_aiding.add("yaw_aiding_N_gaussian", double_t, 0, "", 11, 1, 5000)
    rio_yaw_aiding.add("yaw_aiding_gaussian_sigma", int_t, 0, "", 3, 1, 500)

    rio_yaw_aiding.add("yaw_aiding_init_inlier_thresh_deg", double_t, 0, "", 1.0, 0, 100)
    rio_yaw_aiding.add("yaw_aiding_yaw_inlier_thresh_deg", double_t, 0, "", 5.0, 0, 100)

    rio_yaw_aiding.add("outlier_percentil_radar_yaw", double_t, 0, "", 0.25, 0, 10)
    rio_yaw_aiding.add("sigma_radar_yaw_deg", double_t, 0, "", 2.5, 0, 200)

    return gen
