#!/usr/bin/env python

from dynamic_reconfigure.parameter_generator_catkin import *


def configure(gen):
    sim = gen.add_group("Simulation parameters")
    sim.add("sim_mode", bool_t, 0, "", False)
    sim.add("generate_imu_noise", bool_t, 0, "", False)
    sim.add("generate_radar_noise", bool_t, 0, "", False)

    sim.add("topic_v_r_0", str_t, 0, "", "v_r_0_not_used")
    sim.add("topic_v_r_1", str_t, 0, "", "v_r_1_not_used")
    sim.add("topic_v_r_2", str_t, 0, "", "v_r_2_not_used")

    sim.add("sim_sigma_v_r_x", double_t, 0, "", 0.1, 0, 1000)
    sim.add("sim_sigma_v_r_y", double_t, 0, "", 0.1, 0, 1000)
    sim.add("sim_sigma_v_r_z", double_t, 0, "", 0.1, 0, 1000)

    sim.add("sim_sigma_acc", double_t, 0, "", 0.03, 0, 1000)
    sim.add("sim_sigma_gyro", double_t, 0, "", 0.003, 0, 1000)
    return gen
