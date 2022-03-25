#!/usr/bin/env python

"""
This file is part of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.
@author Christopher Doer <christopher.doer@kit.edu>
"""

import os
import shutil
import rospkg
import rospy
import time
from threading import Thread
from math import ceil
import evaluate_ground_truth
import numpy as np

# use tail to suppress console output
tail = " > /dev/null"

def eval(datasets, k):
    ws = "  "
    s = "Datasets:\n"
    for dataset in datasets:
        s += ws + dataset[0] + ":\n"
        s += ws + ws + "label: " + dataset[0] + "\n"

    s += "Algorithms:\n"
    for algo in [filter["name"] for filter in filters]:
        s += ws + algo + ":\n"
        s += ws + ws + "fn: traj_est\n"
        s += ws + ws + "label: " + algo.replace("_", "") + "\n"

    s += evaluation["rel_distances"]

    eval_config = open(result_base_directory + "/evaluation_config" + str(k) + ".yaml", "w")
    eval_config.write(s)
    eval_config.close()

    cmd = "rosrun rpg_trajectory_evaluation analyze_trajectories.py " + result_base_directory + "/evaluation_config" + str(k) + ".yaml --output_dir=" \
          + evaluation_dir + " --results_dir=" + result_base_directory + " --platform " + platform + \
          " --odometry_error_per_dataset --overall_odometry_error --png --no_sort_names"
    print(cmd)
    os.system(cmd + tail)


def run(datasets, k):
    for dataset in datasets:

        if final_only:
            export_directory_run = export_directory + name + "/"
        else:
            export_directory_run = export_directory + name + "/" + platform + "_" + name + "_" + dataset[0]

        if os.path.isdir(export_directory_run):
            print(export_directory_run + ": exists, removing old content....")
            shutil.rmtree(export_directory_run, ignore_errors=False)

        cmd = "roslaunch x_rio " + launch_file + \
              " ID:=" + str(k) + \
              " rosbag:=" + dataset[0] + ".bag" + \
              " pkg:=" + filter["pkg"] + \
              " export_directory:=" + export_directory_run + \
              " dataset_dir:=" + rosbag_dir + \
              " ground_truth_csv:=" + dataset[0] + "_groundtruth.csv" + \
              " config:=" + config_file + " " + filter["params"] + \
              " bag_start:=" + str(dataset[1]) + \
              " bag_duration:=" + str(dataset[2]) + \
              " dataset_type:=" + dataset[3]
        print(cmd)
        os.system(cmd + tail)

        if not final_only:
            config_file_eval_run = open(export_directory + name + "/" + platform + "_" + name + "_" + dataset[0] + "/eval_cfg.yaml", "w")
            config_file_eval_run.write(eval_config_align)
            config_file_eval_run.close()


def evaluate_final_pos():
    evaluation_result_2d = ""
    evaluation_result_3d = ""
    evaluation_result_z = ""
    z_list = []
    for evaluation in evaluations:
        for base_config in base_configs:
            for variable_config in variable_configs:
                dataset_result_2d = ""
                dataset_result_3d = ""
                dataset_result_z = ""
                header = "algo & "
                for dataset in evaluation["datasets"]:
                    eval_name = variable_config[0] + "_" + base_config[0]
                    csv_file_path = export_directory + eval_name + "/" + dataset[0] + "_rio.csv"
                    raw = np.genfromtxt(csv_file_path, delimiter=' ', skip_header=1)
                    t = raw[:, 0]
                    p = raw[:, 1:4]
                    q_xyzw = raw[:, 4:8]
                    q = np.hstack((q_xyzw[:, 3].reshape(-1, 1), q_xyzw[:, :3]))

                    step = 30
                    trajectory_lengths = np.cumsum(np.linalg.norm(p[step::step, :] - p[:-step:step, :], axis=1))

                    if len(dataset) == 5:
                        err = p[-1, :] - p[0, :] - np.hstack(dataset[4])
                    else:
                        err = p[-1, :] - p[0, :]
                    err_3d_norm = np.linalg.norm(err)
                    err_2d_norm = np.linalg.norm(err[:2])

                    rel_3d = err_3d_norm / trajectory_lengths[-1] * 100.0
                    rel_2d = err_2d_norm / trajectory_lengths[-1] * 100.0
                    dataset_result_2d += "%0.2f & " % rel_2d
                    dataset_result_3d += "%0.2f & " % rel_3d
                    dataset_result_z += "%0.2f & " % err[2]
                    header += "%s_%0.2fm_%ds & " % (dataset[0], trajectory_lengths[-1], t[-1] - t[0])
                    z_list.append(err[2])
                evaluation_result_2d += eval_name + "_2d & " + dataset_result_2d[:-2] + "\n"
                evaluation_result_3d += eval_name + "_3d & " + dataset_result_3d[:-2] + "\n"
                evaluation_result_z += eval_name + "_z & " + dataset_result_z[:-2] + "\n"
    evaluation_result_2d = header[:-2] + "\n" + evaluation_result_2d
    evaluation_result_3d = header[:-2] + "\n" + evaluation_result_3d
    evaluation_result_z = header[:-2] + "\n" + evaluation_result_z
    print(evaluation_result_2d)
    print(evaluation_result_3d)
    print(evaluation_result_z)
    eval_file = open(export_directory + "/" + "evaluation_final_pos.txt", "w")
    eval_file.write(evaluation_result_2d + "\n" + evaluation_result_3d + "\n" + evaluation_result_z)
    eval_file.close()


if __name__ == "__main__":
    rospy.init_node('evaluate_jgn2022_datasets')

    rosbag_base_dir = rospy.get_param('~rosbag_base_dir', "/home/doer/datasets/final_datasets/diss/release_datasets") + "/"
    eval_name = rospy.get_param('~eval_name', "")
    final_only = rospy.get_param('~final_only', False)

    if len(rosbag_base_dir) == 0 or len(eval_name) == 0:
        print("<rosbag_base_dir> not set, use: _rosbag_base_dir:=<path_to_jgn2022_datasets> _eval_name:=<eval_name>")
    else:
        evaluations = [
            {"dataset_name": "carried_datasets",
             "datasets": [

                 ["office_floor", 0, -1, evaluate_ground_truth.PSEUDO_GT],
                 ["lab_floor", 0, -1, evaluate_ground_truth.PSEUDO_GT],
                 ["basement", 0, -1, evaluate_ground_truth.PSEUDO_GT],
                 ["workshop", 0, -1, evaluate_ground_truth.PSEUDO_GT],

             ],
             "rel_distances": "RelDistances: [25,50,75,100]"},
        ]

        base_configs = [
            ["l", "n_radar:=1 calibration_0:=left"],
            ["r", "n_radar:=1 calibration_0:=right"],
            ["c", "n_radar:=1 calibration_0:=center"],
            ["l_r", "n_radar:=2 calibration_0:=left calibration_1:=right"],
            ["l_c_r", "n_radar:=3 calibration_0:=left calibration_1:=right calibration_2:=center"]
        ]
        variable_configs = [
            ["rio_extr_red_lsq", "altimeter_update:=False radar_yaw_update:=False use_odr:=True estimate_extrinsics:=True use_reduced_radar_clone:=True"],
            ["rio_extr_red_lsq_yaw_alt", "altimeter_update:=True radar_yaw_update:=True use_odr:=False"]
        ]
        filters = []
        for k in range(len(base_configs)):
            for l in range(len(variable_configs)):
                filters.append({"name": variable_configs[l][0] + "_" + base_configs[k][0],
                                "pkg": "ekf_multi_rio",
                                "params": variable_configs[l][1] + " " + base_configs[k][1]})

        config_file = "jgn2022_datasets"
        platform = "nuc"

        eval_config_align = "align_type: posyaw\n" \
                            "align_num_frames: -1"

        launch_file = "jgn2022_evaluation_single_run.launch"

        for evaluation in evaluations:
            dataset_name = evaluation["dataset_name"]
            datasets = evaluation["datasets"]

            N_per_thread = int(ceil(len(datasets) / 4.0))

            for filter in filters:
                name = filter["name"]

                rosbag_dir = rosbag_base_dir + "/"
                result_base_directory = rosbag_base_dir + "/results/" + eval_name + "/"
                export_directory = result_base_directory + platform + "/"
                evaluation_dir = result_base_directory + "evaluation_full_align/"

                rospack = rospkg.RosPack()

                # eval multi threaded
                threads = []
                for k in range(0, len(datasets), N_per_thread):
                    threads.append(Thread(target=run, args=(datasets[k:k + N_per_thread], k,)))
                    threads[-1].start()
                time.sleep(0.5)

                done = False
                while not done and not rospy.is_shutdown():
                    for i in range(len(threads)):
                        if not threads[i].is_alive():
                            threads[i].join(1)
                            if len(threads) == 1:
                                threads = []
                                done = True
                                break
                            else:
                                del threads[i]
                                break
                        else:
                            time.sleep(0.1)

                if not os.path.isdir(evaluation_dir):
                    os.mkdir(evaluation_dir)

            if final_only:
                evaluate_final_pos()
            else:
                ws = "  "
                s = "Datasets:\n"
                for dataset in datasets:
                    s += ws + dataset[0] + ":\n"
                    s += ws + ws + "label: " + dataset[0] + "\n"

                s += "Algorithms:\n"
                for algo in [filter["name"] for filter in filters]:
                    s += ws + algo + ":\n"
                    s += ws + ws + "fn: traj_est\n"
                    s += ws + ws + "label: " + algo.replace("_", "") + "\n"

                s += evaluation["rel_distances"]

                eval_config = open(result_base_directory + "/evaluation_config.yaml", "w")
                eval_config.write(s)
                eval_config.close()

                cmd = "rosrun rpg_trajectory_evaluation analyze_trajectories.py " + result_base_directory + "/evaluation_config.yaml --output_dir=" + evaluation_dir + \
                      " --results_dir=" + result_base_directory + " --platform " + platform + \
                      " --odometry_error_per_dataset --overall_odometry_error --plot_trajectories --rmse_table --rmse_boxplot --png --no_sort_names"
                print(cmd)
                os.system(cmd)
