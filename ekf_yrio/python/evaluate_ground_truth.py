#!/usr/bin/env python

"""
This file is part of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement

import threading
import numpy as np
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped

VICON = "vicon"
PSEUDO_GT = "pseudo_gt"


class GroundTruth:
    def __init__(self, csv_file_path, dataset_type):

        if dataset_type == VICON:
            gt_raw = np.genfromtxt(csv_file_path, delimiter=',', skip_header=1)
            self.t = gt_raw[:, 0] * 1.0e-9
            self.p = gt_raw[:, 1:4] - gt_raw[0, 1:4]
            self.q = gt_raw[:, 4:8]
            self.v = gt_raw[:, 8:11]
            self.bw = gt_raw[:, 11:14]
            self.ba = gt_raw[:, 14:17]

        elif dataset_type == PSEUDO_GT:
            gt_raw = np.genfromtxt(csv_file_path, delimiter=' ', skip_header=1)
            self.t = gt_raw[:, 0]
            self.p = gt_raw[:, 1:4]
            q_xyzw = gt_raw[:, 4:8]
            self.q = np.hstack((q_xyzw[:, 3].reshape(-1, 1), q_xyzw[:, :3]))
            self.v = None

        self.step = 5
        self.trajectory_lengths = np.cumsum(np.linalg.norm(self.p[self.step::self.step, :] - self.p[:-self.step:self.step, :], axis=1))
        print("##### Loaded trajectory with length %0.2fm and duration %0.2fs" % (self.trajectory_lengths[-1], self.t[-1] - self.t[0]))


class EvaluateGroundTruth:
    def __init__(self, rosbag_name, rosbag_dir, ground_truth_csv, pose_topic, export_directory):

        self.rosbag_name = rosbag_name
        self.rosbag_dir = rosbag_dir

        if len(export_directory) > 0:
            self.export_directory = export_directory
        else:
            self.export_directory = ""

        gt_csv_path = rosbag_dir + ground_truth_csv
        self.ground_truth = GroundTruth(gt_csv_path, PSEUDO_GT)

        self.last_timestamp_odometry = None
        self.timestamp_first_odometry = None
        self.pose_data = []
        self.start_time = None
        self.last_pub_walltime = time.time()

        self.lock = threading.Lock()

        self.sub_rio_pose = rospy.Subscriber(pose_topic, PoseStamped, self.callback_rio_pose, queue_size=1000000)
        self.print_("Subscribed to topic %s" % self.sub_rio_pose.name)

    def callback_rio_pose(self, msg=PoseStamped()):
        if self.start_time is None:
            self.start_time = time.time()

        with self.lock:
            self.last_timestamp_odometry = time.time()

        p = np.hstack((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        q = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

        self.pose_data.append(np.hstack((msg.header.stamp.to_sec(), p, q[1:], q[0])))

    def print_(self, s):
        rospy.loginfo("[evaluate_ground_truth] : " + s)

    def export(self):

        time_taken = self.last_timestamp_odometry - self.start_time
        time_data = self.pose_data[-1][0] - self.pose_data[0][0]
        runtime = "%0.2f &  %0.2f & %0.2f" % (time_taken, time_data, time_data / time_taken)
        self.print_(runtime)

        if len(self.export_directory) > 0:
            self.print_("Exporting to %s" % self.export_directory)

            if not os.path.isdir(self.export_directory):
                os.makedirs(self.export_directory)

            csv_gt = self.export_directory + "/" + "stamped_groundtruth.txt"
            csv_vio = self.export_directory + "/" + "stamped_traj_estimate.txt"

            data_gt = np.hstack((self.ground_truth.t.reshape(-1, 1), self.ground_truth.p, self.ground_truth.q[:, 1:], self.ground_truth.q[:, 0].reshape(-1, 1)))

            data_vio = np.vstack(self.pose_data)

            np.savetxt(csv_gt, data_gt, delimiter=' ')
            np.savetxt(csv_vio, data_vio, delimiter=' ')

            dataset = self.export_directory[self.export_directory.rfind("/") + 1:]
            algo = self.export_directory[self.export_directory[:self.export_directory.rfind("/")].rfind("/") + 1:self.export_directory.rfind("/")]
            export_path_eval = self.export_directory[:self.export_directory[:self.export_directory.rfind("/")].rfind("/")]
            timing_file = open(export_path_eval + "/" + "eval.txt", "a")
            timing_file.write(dataset + "_" + algo + " & " + runtime + "\n")
            timing_file.close()


if __name__ == "__main__":
    rospy.init_node('evaluate_ground_truth', anonymous=True)

    rosbag_name = rospy.get_param('~rosbag_name', "")
    rosbag_dir = rospy.get_param('~rosbag_dir', "")
    ground_truth_csv = rospy.get_param('~ground_truth_csv', "")
    pose_topic = rospy.get_param('~pose_topic', "")
    export_directory = rospy.get_param('~export_directory', "")

    evaluator = EvaluateGroundTruth(rosbag_name, rosbag_dir, ground_truth_csv, pose_topic, export_directory)
    while not rospy.core.is_shutdown():
        if evaluator.last_timestamp_odometry is not None and time.time() - evaluator.last_timestamp_odometry > 1:
            break
        time.sleep(0.1)

    evaluator.export()
