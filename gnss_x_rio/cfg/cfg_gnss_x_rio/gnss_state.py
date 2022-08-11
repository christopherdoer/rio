#!/usr/bin/env python

from dynamic_reconfigure.parameter_generator_catkin import *


def configure(gen):
    gnss_state = gen.add_group("GNSS State")
    gnss_state.add("enable_gnss_state", bool_t, 0, "", True)

    gnss_state.add("threshold_accuracy_good", double_t, 0, "", 10, 0, 50)
    gnss_state.add("threshold_accuracy_medium", double_t, 0, "", 14, 0, 50)
    gnss_state.add("threshold_accuracy_poor", double_t, 0, "", 20, 0, 50)

    gnss_state.add("gnss_nominal_rate", int_t, 0, "", 10, 0, 50)
    gnss_state.add("gnss_score_thresh_valid", double_t, 0, "", 15, 0, 100)

    gnss_state.add("gnss_window_s", double_t, 0, "", 5, 0, 100)
    gnss_state.add("gnss_window_m", double_t, 0, "", 10, 0, 1000)

    return gen
