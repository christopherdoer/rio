#!/usr/bin/env python
PACKAGE = "radar_ego_velocity_estimation"

from dynamic_reconfigure.parameter_generator_catkin import *


def configure(gen):
    radar_refiner = gen.add_group("Radar Ego Velocity Estimator")
    radar_refiner.add("min_dist", double_t, 0, "Min distance of targets", 0.25, 0, 100)
    radar_refiner.add("max_dist", double_t, 0, "Max distance of targets", 100, 0, 1000)
    radar_refiner.add("min_db", double_t, 0, "Min dB of targets", 5, 0, 50)
    radar_refiner.add("elevation_thresh_deg", double_t, 0, "Absolute elevation angle threshold", 60, 0, 100)
    radar_refiner.add("azimuth_thresh_deg", double_t, 0, "Absolute azimuth angle threshold", 60, 0, 100)
    radar_refiner.add("doppler_velocity_correction_factor", double_t, 0, "Correction factor of Doppler velocity measurements", 1.0, 0, 2)

    radar_refiner.add("thresh_zero_velocity", double_t, 0, "Threshold for zero velocity detection", 0.05, 0, 10)
    radar_refiner.add("allowed_outlier_percentage", double_t, 0, "Percentage of allowed outlier for detection", 0.75, 0, 1)
    radar_refiner.add("sigma_zero_velocity_x", double_t, 0, "Sigma for zero velocity v_x", 0.01, 0, 10)
    radar_refiner.add("sigma_zero_velocity_y", double_t, 0, "Sigma for zero velocity v_y", 0.01, 0, 10)
    radar_refiner.add("sigma_zero_velocity_z", double_t, 0, "Sigma for zero velocity v_z", 0.01, 0, 10)

    radar_refiner.add("sigma_offset_radar_x", double_t, 0, "Sigma offset x", 0.0, 0, 10)
    radar_refiner.add("sigma_offset_radar_y", double_t, 0, "Sigma offset y", 0.0, 0, 10)
    radar_refiner.add("sigma_offset_radar_z", double_t, 0, "Sigma offset z", 0.0, 0, 10)

    radar_refiner.add("max_sigma_x", double_t, 0, "Max sigma for valid result", 0.1, 0, 10)
    radar_refiner.add("max_sigma_y", double_t, 0, "Max sigma for valid result", 0.1, 0, 10)
    radar_refiner.add("max_sigma_z", double_t, 0, "Max sigma for valid result", 0.9, 0, 10)
    radar_refiner.add("max_r_cond", double_t, 0, "Max condition number for LSQ solution attempt", 1.0e3, 0, 1.0e4)
    radar_refiner.add("use_cholesky_instead_of_bdcsvd", bool_t, 0, "Faster but less stable", True)

    radar_refiner.add("use_ransac", bool_t, 0, "Enables RANSAC LSQ", True)
    radar_refiner.add("outlier_prob", double_t, 0, "Outlier probabilty", 0.4, 0, 1)
    radar_refiner.add("success_prob", double_t, 0, "Probability for RANSAC success", 0.9999, 0, 1)
    radar_refiner.add("N_ransac_points", int_t, 0, "Number of RANSAC points (at least 3 required)", 3, 3, 20)
    radar_refiner.add("inlier_thresh", double_t, 0, "Inlier threshold", 0.15, 0, 2)

    return gen
