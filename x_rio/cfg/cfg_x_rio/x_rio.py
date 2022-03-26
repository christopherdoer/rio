#!/usr/bin/env python
PACKAGE = "x_rio"

from dynamic_reconfigure.parameter_generator_catkin import *


def configure(gen):
    # general
    general = gen.add_group("Filter Mode")
    general.add("frame_id", str_t, 0, "Frame of estimated pose", "odom")
    general.add("estimate_extrinsics", bool_t, 0, "Enable online calibration of the extrinsic radar transform", True)

    # subscribers
    subscribers = gen.add_group("Subscribers")
    subscribers.add("topic_imu", str_t, 0, "Topic imu", "empty")
    subscribers.add("topic_baro_altimeter", str_t, 0, "Topic baro altimeter", "empty")
    subscribers.add("topic_radar_scan", str_t, 0, "Topic radar scan", "empty")
    subscribers.add("topic_radar_trigger", str_t, 0, "Topic radar data trigger", "empty")

    # publishers
    publishers = gen.add_group("Publishers")
    publishers.add("publisher_rate", double_t, 0, "Publisher rate of navigation filter", 1, 0, 420)
    publishers.add("pose_path_publisher_rate", double_t, 0, "Publisher rate of pose path filter", 1, 0, 50)

    # rosbag mode
    rosbag_mode = gen.add_group("Rosbag Mode")
    rosbag_mode.add("rosbag_sleep_ms", double_t, 0, "Sleep of each iteration in rosbag mode --> useful slow down in rosbag mode", 0.01, -100, 100)
    rosbag_mode.add("republish_ground_truth", bool_t, 0, "Publish ground truth topics --> useful when using the rosbag mode with python evaluation", False)
    rosbag_mode.add("topic_ground_truth_pose", str_t, 0, "Ground truth pose topic which will be republished", "/ground_truth/pose")
    rosbag_mode.add("topic_ground_truth_twist", str_t, 0, "Ground truth twist topic which will be republished", "/ground_truth/twist")
    rosbag_mode.add("topic_ground_truth_twist_body", str_t, 0, "Ground truth twist of body frame topic which will be republished", "/ground_truth/twist_body")

    # kf update
    kf_update = gen.add_group("KF Update")
    kf_update.add("altimeter_update", bool_t, 0, "enable altimeter update", False)
    kf_update.add("radar_update", bool_t, 0, "Enable radar update", False)
    kf_update.add("sigma_altimeter", double_t, 0, "Sigma of altimeter measurement", 1.0, 0, 10)

    # radar model
    radar = gen.add_group("Radar Measurement Model")
    radar.add("outlier_percentil_radar", double_t, 0, "Percentil of chi^2 distribution for Mahalnobis distance outlier rejection", 0.25, 0, 10)
    radar.add("use_w", bool_t, 0, "Use omega to compensate for lever arm of radar", True)
    radar.add("radar_frame_ms", double_t, 0, "Length of each radar frame (=exposure time of radar scan) in milliseconds", 0, 0, 1000)
    radar.add("radar_rate", double_t, 0, "Nominal update rate of radar", 10, 0, 100)
    radar.add("use_reduced_radar_clone", bool_t, 0, "Use reduced radar clones instead of full clones", False)
    radar.add("use_diagonal_noise", bool_t, 0, "Use only the diagonal of the estimated covariance instead of the full matrix", False)
    radar.add("estimated_extrinsics_error_att", bool_t, 0, "", False)

    # initial nominal states
    initial_nominal_states = gen.add_group("Initial Nominal States")
    initial_nominal_states.add("p_0_x", double_t, 0, "initial pos x^n in [m]", 0, -100, +100)
    initial_nominal_states.add("p_0_y", double_t, 0, "initial pos y^n in [m]", 0, -100, +100)
    initial_nominal_states.add("p_0_z", double_t, 0, "initial pos z^n in [m]", 0, -100, +100)

    initial_nominal_states.add("v_0_x", double_t, 0, "initial vel x^n in [m/2]", 0, -5, +5)
    initial_nominal_states.add("v_0_y", double_t, 0, "initial vel y^n in [m/2]", 0, -5, +5)
    initial_nominal_states.add("v_0_z", double_t, 0, "initial vel z^n in [m/2]", 0, -5, +5)

    initial_nominal_states.add("yaw_0_deg", double_t, 0, "initial yaw in [deg]", 0, -360, +360)

    initial_nominal_states.add("b_0_a_x", double_t, 0, "initial bias acc x^b in [m/s^2]", 0, -2, +2)
    initial_nominal_states.add("b_0_a_y", double_t, 0, "initial bias acc y^b in [m/s^2]", 0, -2, +2)
    initial_nominal_states.add("b_0_a_z", double_t, 0, "initial bias acc z^b in [m/s^2]", 0, -2, +2)

    initial_nominal_states.add("b_0_w_x_deg", double_t, 0, "initial bias omega x^b in [deg/s]", 0, -1, +1)
    initial_nominal_states.add("b_0_w_y_deg", double_t, 0, "initial bias omega y^b in [deg/s]", 0, -1, +1)
    initial_nominal_states.add("b_0_w_z_deg", double_t, 0, "initial bias omega z^b in [deg/s]", 0, -1, +1)
    initial_nominal_states.add("calib_gyro", bool_t, 0, "calib gyro", True)

    initial_nominal_states.add("b_0_alt", double_t, 0, "initial bias altimeter x^n in [m]", 0, -100, +100)

    initial_nominal_states.add("l_b_r_x", double_t, 0, "x-translation body to radar expressed in body frame", 0, -1, 1)
    initial_nominal_states.add("l_b_r_y", double_t, 0, "y-translation body to radar expressed in body frame", 0, -1, 1)
    initial_nominal_states.add("l_b_r_z", double_t, 0, "z-translation body to radar expressed in body frame", 0, -1, 1)
    initial_nominal_states.add("q_b_r_w", double_t, 0, "rotation radar frame to body frame", 0, -1, 1)
    initial_nominal_states.add("q_b_r_x", double_t, 0, "rotation radar frame to body frame", 0, -1, 1)
    initial_nominal_states.add("q_b_r_y", double_t, 0, "rotation radar frame to body frame", 0, -1, 1)
    initial_nominal_states.add("q_b_r_z", double_t, 0, "rotation radar frame to body frame", 0, -1, 1)

    initial_nominal_states.add("g_n", double_t, 0, "initial gravity g^n in [m/s^2]", 9.81, 0, 10)

    # initial uncertainty
    initial_uncertainty = gen.add_group("Initial Uncertainty")
    initial_uncertainty.add("sigma_p", double_t, 0, "initial sigma pos^n in [m]", 0, 0, +1000)
    initial_uncertainty.add("sigma_v", double_t, 0, "initial sigma vel^n in [m/s]", 0, 0, +1000)
    initial_uncertainty.add("sigma_roll_pitch_deg", double_t, 0, "initial sigma roll and pitch in [deg]", 1, 0, +360)
    initial_uncertainty.add("sigma_yaw_deg", double_t, 0, "initial sigma yaw in [deg]", 1, 0, +360)
    initial_uncertainty.add("sigma_b_a", double_t, 0, "initial sigma bias acc^b in [m/s^2]", 0.01, 0, +1)
    initial_uncertainty.add("sigma_b_w_deg", double_t, 0, "initial sigma bias omega^b in [deg/s]", 0.00035, 0, +1)
    initial_uncertainty.add("sigma_b_alt", double_t, 0, "initial sigma alt^n in [m]", 0, 0, +1000)

    initial_uncertainty.add("sigma_l_b_r_x", double_t, 0, "initial sigma l_b_r [m]", 0, 0, +1)
    initial_uncertainty.add("sigma_l_b_r_y", double_t, 0, "initial sigma l_b_r [m]", 0, 0, +1)
    initial_uncertainty.add("sigma_l_b_r_z", double_t, 0, "initial sigma l_b_r [m]", 0, 0, +1)
    initial_uncertainty.add("sigma_eul_b_r_roll_deg", double_t, 0, "initial sigma eul_b_r [deg]", 0, 0, 360)
    initial_uncertainty.add("sigma_eul_b_r_pitch_deg", double_t, 0, "initial sigma eul_b_r [deg]", 0, 0, 360)
    initial_uncertainty.add("sigma_eul_b_r_yaw_deg", double_t, 0, "initial sigma eul_b_r [deg]", 0, 0, 360)

    # noise psds
    noise_psds = gen.add_group("Noise PSDs")
    noise_psds.add("noise_psd_a", double_t, 0, "noise psd acc^b in [m/s^2]", 0.03, 0, 1)
    noise_psds.add("noise_psd_w_deg", double_t, 0, "noise psd omega^b in [deg/s]", 0.18, 0, 1)
    noise_psds.add("noise_psd_b_a", double_t, 0, "noise psd bias acc^b in [m/s^2]", 0.03, 0, 1)
    noise_psds.add("noise_psd_b_w_deg", double_t, 0, "noise psd bias omega^b in [deg/s]", 0.01, 0, 1)
    noise_psds.add("noise_psd_b_alt", double_t, 0, "noise psd altimeter bias in [m]", 0.01, 0, 1)

    initialization = gen.add_group("Initialization")
    initialization.add("T_init", double_t, 0, "Initialization duration in sec for IMU data averaging", 10, 0, 100)

    return gen
