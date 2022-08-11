#!/usr/bin/env python
PACKAGE = "gnss_x_rio"

from dynamic_reconfigure.parameter_generator_catkin import *


def configure(gen):
    # GNSS XRio Additions
    additions = gen.add_group("GNSS_XRIO")
    additions.add("topic_gnss_valid", str_t, 0, "Topic gnss vlid measurment", "empty")
    additions.add("topic_gnss_measurement", str_t, 0, "Topic gnss measurment", "empty")

    additions.add("gnss_pos_update", bool_t, 0, "Enable gnss position update", False)
    additions.add("gnss_vel_update", bool_t, 0, "Enable gnss velocity update", False)

    additions.add("outlier_percentil_gnss_pos", double_t, 0, "Percentil of chi^2 distribution for Mahalnobis distance outlier rejection", 1.0, 0, 10)
    additions.add("outlier_percentil_gnss_vel", double_t, 0, "Percentil of chi^2 distribution for Mahalnobis distance outlier rejection", 1.0, 0, 10)
    additions.add("min_pos_accuracy_init", double_t, 0, "Min pos accuracy for init", 1.0, 0, 10)
    additions.add("sigma_pos_scaling", double_t, 0, "Scaling for sigma pos", 1.0, 0, 1000)
    additions.add("max_pos_acc_thresh", double_t, 0, "No fusion above this threshold", 1.0, 0, 1000)
    additions.add("max_vel_acc_thresh", double_t, 0, "No fusion above this threshold", 1.0, 0, 1000)
    additions.add("min_n_sat", int_t, 0, "Min num sat for fusion", 4, 0, 30)

    return gen
