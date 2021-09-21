#!/usr/bin/env python2

"""
This file is part of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement

import threading
import numpy as np
import time

import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped


class VelocityEstimationEvaluator:
    def __init__(self, topic_radar_velocity, topic_velocity_gt="/ground_truth/twist_body"):

        self.last_sub_walltime = None
        self.list_v_b = []
        self.list_v_gt = []
        self.lock = threading.Lock()

        self.sub_v_body = rospy.Subscriber(topic_radar_velocity, TwistWithCovarianceStamped, self.callback_v_body, queue_size=10)
        self.sub_v_ground_truth = rospy.Subscriber(topic_velocity_gt, TwistStamped, self.callback_v_ground_truth, queue_size=10)

    def callback_v_body(self, msg=TwistWithCovarianceStamped()):
        with self.lock:
            self.list_v_b.append([msg.header.stamp.to_sec(),
                                  msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z,
                                  np.sqrt(msg.twist.covariance[0]),
                                  np.sqrt(msg.twist.covariance[7]),
                                  np.sqrt(msg.twist.covariance[14])])
            self.last_sub_walltime = time.time()

    def callback_v_ground_truth(self, msg=TwistStamped()):
        with self.lock:
            self.list_v_gt.append([msg.header.stamp.to_sec(),
                                   msg.twist.linear.x,
                                   msg.twist.linear.y,
                                   msg.twist.linear.z])
            self.last_sub_walltime = time.time()

    def analyze(self):
        v_body_raw = np.vstack(self.list_v_b)

        if len(self.list_v_gt) == 0:
            fig_v, (ax_comp) = plt.subplots(3, 1, sharex=True, sharey=True, figsize=(10, 8))
            fig_v.suptitle('Radar Body Velocity Estimation', fontsize="large")

            xyz = "xyz"
            for k in range(3):
                ax_comp[k].plot(v_body_raw[:, 0], v_body_raw[:, k + 1], label="radar")
                ax_comp[k].grid(True)
                ax_comp[k].set_ylabel("v_b_" + xyz[k] + " [m/s]")

            ax_comp[2].legend(loc='upper right', fontsize="medium")
            ax_comp[2].set_xlabel("ros timestamp [s]")
        else:
            v_body_gt_raw = np.vstack(self.list_v_gt)

            min_t = max([v_body_raw[0, 0], v_body_gt_raw[0, 0]])

            v_body = v_body_raw[v_body_raw[:, 0] > min_t, :]
            v_body_gt = v_body_gt_raw[v_body_gt_raw[:, 0] > min_t, :]

            v_body_gt_inter = np.zeros((v_body.shape[0], 3))
            for k in range(3):
                v_body_gt_inter[:, k] = np.interp(v_body[:, 0], v_body_gt[:, 0], v_body_gt[:, k + 1])

            err_v = v_body[:, 1:4] - v_body_gt_inter

            # err_v[np.linalg.norm(err_v, axis=1) > 1, :] = 0

            self.print_stats(err_v)

            fig_v, (ax_comp) = plt.subplots(3, 1, sharex=True, sharey=True, figsize=(10, 8))
            fig_v.suptitle('Radar Body Velocity Estimation Comparison', fontsize="large")

            fig_err, (ax_v_err) = plt.subplots(3, 1, sharex=True, sharey=True, figsize=(10, 8))
            fig_err.suptitle('Radar Body Velocity Estimation Error with Estimated Sigmas', fontsize="large")

            xyz = "xyz"
            for k in range(3):
                ax_comp[k].plot(v_body_raw[:, 0], v_body_raw[:, k + 1], label="radar")
                ax_comp[k].plot(v_body_gt_raw[:, 0], v_body_gt_raw[:, k + 1], label="groundtruth")
                ax_comp[k].grid(True)
                ax_comp[k].set_ylabel("v_b_" + xyz[k] + " [m/s]")

                ax_v_err[k].plot(v_body[:, 0], err_v[:, k], 'k', label="velocity error")
                ax_v_err[k].plot(v_body[:, 0], 3 * v_body[:, k + 4], 'r-', linewidth=3, label="est. +-3 sigma")
                ax_v_err[k].plot(v_body[:, 0], -3 * v_body[:, k + 4], 'r-', linewidth=3)
                ax_v_err[k].grid(True)
                ax_v_err[k].set_ylabel("err v_b_" + xyz[k] + " [m/s]")

            ax_comp[2].legend(loc='upper right', fontsize="medium")
            ax_comp[2].set_xlabel("ros timestamp [s]")

            ax_v_err[0].legend(loc='upper right', fontsize="medium")
            ax_v_err[2].set_xlabel("ros timestamp [s]")

    def print_stats(self, err):
        err_mean = err.mean(axis=0)
        abs_err_mean = np.abs(err).mean(axis=0)
        err_norm_mean = np.linalg.norm(err, axis=1).mean()
        std = np.std(err, axis=0)

        print("##############################################")
        print("Analysis of the radar body velocity estimation")
        print("###############################################")
        print("  Mean error: %0.3f, %0.3f, %0.3f" % (err_mean[0], err_mean[1], err_mean[2]))
        print("  Mean absolute error: %0.3f, %0.3f, %0.3f" % (abs_err_mean[0], abs_err_mean[1], abs_err_mean[2]))
        print("  Mean error norm: %0.3f" % err_norm_mean)
        print("  STD: %0.3f, %0.3f, %0.3f" % (std[0], std[1], std[2]))


if __name__ == "__main__":
    rospy.init_node('velocity_estimation_evaluator')

    topic_v_b = rospy.get_param('~topic_v_b', "/radar_body_velocity_estimation_node/twist_body")

    evaluator = VelocityEstimationEvaluator(topic_v_b)

    try:
        while not rospy.core.is_shutdown():
            if evaluator.last_sub_walltime is not None and time.time() - evaluator.last_sub_walltime > 0.5:
                break
            rospy.rostime.wallsleep(0.5)
        evaluator.analyze()
        while not rospy.core.is_shutdown():
            plt.pause(0.1)

    except KeyboardInterrupt:
        rospy.core.signal_shutdown('keyboard interrupt')
