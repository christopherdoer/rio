#!/usr/bin/env python

"""
This file is part of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement

import threading
import numpy as np
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
import tf.transformations


def quat_to_euler(q):
    return tf.transformations.euler_from_quaternion(q[[1, 2, 3, 0]])


def euler_to_rot_mat(eul):
    return tf.transformations.euler_matrix(eul[0], eul[1], eul[2])[0:3, 0:3]


class PoseVelocityEvaluator:
    def __init__(self, topic_pose, topic_twist, topic_pose_ground_truth, topic_twist_ground_truth, filter_name, do_plot):

        self.do_plot = do_plot

        self.last_sub_walltime = None

        self.filter_name = filter_name

        self.list_pose_filter = []
        self.list_twist_filter = []

        self.list_pose_ground_truth = []
        self.list_twist_ground_truth = []

        self.lock = threading.Lock()

        self.sub_pose = rospy.Subscriber(topic_pose, PoseStamped, self.callback_pose_filter, queue_size=100000)
        self.sub_twist = rospy.Subscriber(topic_twist, TwistStamped, self.callback_twist_filter, queue_size=100000)

        self.sub_pose_ground_truth = rospy.Subscriber(topic_pose_ground_truth, PoseStamped, self.callback_pose_ground_truth, queue_size=100000)
        self.sub_twist_ground_truth = rospy.Subscriber(topic_twist_ground_truth, TwistStamped, self.callback_twist_ground_truth, queue_size=100000)

    def callback_pose_filter(self, msg=PoseStamped()):
        with self.lock:
            self.list_pose_filter.append([msg.header.stamp.to_sec(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                          msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
            self.last_sub_walltime = time.time()

    def callback_twist_filter(self, msg=TwistStamped()):
        with self.lock:
            self.list_twist_filter.append([msg.header.stamp.to_sec(), msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            self.last_sub_walltime = time.time()

    def callback_pose_ground_truth(self, msg=PoseStamped()):
        with self.lock:
            self.list_pose_ground_truth.append([msg.header.stamp.to_sec(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                                msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
            self.last_sub_walltime = time.time()

    def callback_twist_ground_truth(self, msg=TwistStamped()):
        with self.lock:
            self.list_twist_ground_truth.append([msg.header.stamp.to_sec(), msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
            self.last_sub_walltime = time.time()

    def print_stats(self, err, title):
        err_mean = err.mean(axis=0)
        abs_err_mean = np.abs(err).mean(axis=0)
        err_norm_mean = np.linalg.norm(err, axis=1).mean()
        std = np.std(err, axis=0)

        print("#" * len(title))
        print(title)
        print("#" * len(title))
        print("  Mean error: %0.3f, %0.3f, %0.3f" % (err_mean[0], err_mean[1], err_mean[2]))
        print("  Mean absolute error: %0.3f, %0.3f, %0.3f" % (abs_err_mean[0], abs_err_mean[1], abs_err_mean[2]))
        print("  Mean error norm: %0.3f" % err_norm_mean)
        print("  STD: %0.3f, %0.3f, %0.3f" % (std[0], std[1], std[2]))

    def analyze(self):
        pose_filter_raw = np.vstack(self.list_pose_filter)
        velocity_filter_raw = np.vstack(self.list_twist_filter)

        pose_gt_raw = np.vstack(self.list_pose_ground_truth)
        velocity_gt_raw = np.vstack(self.list_twist_ground_truth)

        min_t = max([pose_filter_raw[0, 0], velocity_filter_raw[0, 0], pose_gt_raw[0, 0], velocity_gt_raw[0, 0]])

        pose_filter = pose_filter_raw[pose_filter_raw[:, 0] > min_t, :]
        velocity_filter = velocity_filter_raw[velocity_filter_raw[:, 0] > min_t, :]
        pose_gt = pose_gt_raw[pose_gt_raw[:, 0] > min_t, :]
        velocity_gt = velocity_gt_raw[velocity_gt_raw[:, 0] > min_t, :]

        p_0_gt = pose_gt[0, 1:4]

        yaw_filter_0 = quat_to_euler(pose_filter[0, 4:])[2]
        yaw_gt_0 = quat_to_euler(pose_gt[0, 4:])[2]

        C = euler_to_rot_mat([0, 0, yaw_gt_0 - yaw_filter_0])

        p_filter_rot_raw = C.dot(pose_filter_raw[:, 1:4].transpose()).transpose()
        p_filter_rot_raw = p_filter_rot_raw - (p_filter_rot_raw[0, :] - p_0_gt)
        p_filter_rot = p_filter_rot_raw[pose_filter_raw[:, 0] > min_t, :]

        v_filter_rot = C.dot(velocity_filter[:, 1:4].transpose()).transpose()

        p_gt_inter = np.zeros((p_filter_rot.shape[0], 3))
        v_gt_inter = np.zeros((v_filter_rot.shape[0], 3))
        for k in range(3):
            p_gt_inter[:, k] = np.interp(pose_filter[:, 0], pose_gt[:, 0], pose_gt[:, k + 1])
            v_gt_inter[:, k] = np.interp(velocity_filter[:, 0], velocity_gt[:, 0], velocity_gt[:, k + 1])

        err_p = p_filter_rot - p_gt_inter
        err_v = v_filter_rot - v_gt_inter

        print("Alignment with ground truth is done using pos-yaw on the first ground truth state")
        self.print_stats(err_p, "Position Error Analysis")
        self.print_stats(err_v, "Velocity Error Analysis")

        if self.do_plot:
            import matplotlib.pyplot as plt

            fig_comp, (ax_comp) = plt.subplots(3, 2, sharex=True, figsize=(15, 8))
            fig_comp.suptitle('Filter vs. Ground Truth', fontsize="x-large")

            fig_err, (ax_err) = plt.subplots(3, 2, sharex=True, figsize=(15, 8))
            fig_err.suptitle('Evaluation', fontsize="x-large")

            xyz = "xyz"
            for k in range(3):
                ax_comp[k][0].plot(pose_filter_raw[:, 0], p_filter_rot_raw[:, k], label=self.filter_name)
                ax_comp[k][0].plot(pose_gt_raw[:, 0], pose_gt_raw[:, k + 1], label="groundtruth")
                ax_comp[k][0].grid(True)
                ax_comp[k][0].set_title("Position " + xyz[k])
                ax_comp[k][0].set_ylabel("[m]")

                ax_comp[k][1].plot(velocity_filter[:, 0], v_filter_rot[:, k], label=self.filter_name)
                ax_comp[k][1].plot(velocity_filter[:, 0], v_gt_inter[:, k], label="groundtruth")
                ax_comp[k][1].grid(True)
                ax_comp[k][1].set_title("Velocity " + xyz[k])
                ax_comp[k][1].set_ylabel("[m/s]")

                ax_err[k][0].plot(pose_filter[:, 0], err_p[:, k], 'k')
                ax_err[k][0].grid(True)
                ax_err[k][0].set_title("Error Position " + xyz[k])
                ax_err[k][0].set_ylabel("[m]")

                ax_err[k][1].plot(velocity_filter[:, 0], err_v[:, k], 'k')
                ax_err[k][1].grid(True)
                ax_err[k][1].set_title("Error Velocity " + xyz[k])
                ax_err[k][1].set_ylabel("[m/s]")

            ax_comp[2][1].legend(loc='upper right', fontsize="medium")
            ax_comp[2][0].set_xlabel("ros time [s]")
            ax_comp[2][1].set_xlabel("ros time [s]")

            ax_err[2][1].legend(loc='upper right', fontsize="medium")
            ax_err[2][0].set_xlabel("ros timestamp [s]")
            ax_err[2][1].set_xlabel("ros timestamp [s]")


if __name__ == "__main__":
    rospy.init_node('pose_velocity_evaluator')

    topic_pose = rospy.get_param('~topic_pose', "/rio_calib_filter_live/pose")
    topic_twist = rospy.get_param('~topic_twist', "/rio_calib_filter_live/twist")

    topic_pose_gt = rospy.get_param('~topic_pose_gt', "/ground_truth/pose")
    topic_twist_gt = rospy.get_param('~topic_twist_gt', "/ground_truth/twist")

    filter_name = rospy.get_param('~filter_name', "ekf-rio")
    do_plot = rospy.get_param('~do_plot', True)

    evaluator = PoseVelocityEvaluator(topic_pose, topic_twist, topic_pose_gt, topic_twist_gt, filter_name, do_plot)

    try:
        while not rospy.core.is_shutdown():
            if evaluator.last_sub_walltime is not None and time.time() - evaluator.last_sub_walltime > 0.5:
                break
            rospy.rostime.wallsleep(0.5)

        evaluator.analyze()

        if do_plot:
            import matplotlib.pyplot as plt

            while not rospy.core.is_shutdown():
                plt.pause(0.1)

    except KeyboardInterrupt:
        rospy.core.signal_shutdown('keyboard interrupt')
