#!/usr/bin/env python

"""
This file is part of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.
@author Christopher Doer <christopher.doer@kit.edu>
"""

import os
import rospkg
import rospy

if __name__ == "__main__":
    rospy.init_node('icins_2021_evaluations')

    rosbag_base_dir = rospy.get_param('~rosbag_base_dir', "") + "/"

    if len(rosbag_base_dir) == 0:
        print("<rosbag_base_dir> not set, use: _rosbag_base_dir:=<path_to_icins_datasets>")
    else:
        evaluations = [
            {"dataset_name": "carried_datasets",
             "datasets": ["carried_1", "carried_2", "carried_3", "carried_4", "carried_5"],
             "rel_distances": "RelDistances: [48, 96, 144, 192]"},
            {"dataset_name": "flight_datasets",
             "datasets": ["flight_1", "flight_2", "flight_3", "flight_4"],
             "rel_distances": "RelDistances: [12, 24, 36, 48]"}
        ]

        filters = [
            {"name": "ekf_rio", "pkg": "ekf_rio", "params": "altimeter_update:=False enable_yaw_aiding:=False"},
            {"name": "ekf_rio_b", "pkg": "ekf_rio", "params": "altimeter_update:=True enable_yaw_aiding:=False"},
            {"name": "ekf_yrio", "pkg": "ekf_yrio", "params": "altimeter_update:=False enable_yaw_aiding:=True"},
            {"name": "ekf_yrio_b", "pkg": "ekf_yrio", "params": "altimeter_update:=True enable_yaw_aiding:=True"}
        ]

        config_file = "ekf_yrio_default"
        platform = "nuc"

        eval_config_align = "align_type: posyaw\n" \
                            "align_num_frames: -1"

        launch_file = "icins_2021_evaluation_single_run.launch"

        for evaluation in evaluations:
            for filter in filters:
                name = filter["name"]
                dataset_name = evaluation["dataset_name"]
                datasets = evaluation["datasets"]

                rosbag_dir = rosbag_base_dir + dataset_name + "/"
                result_base_directory = rosbag_base_dir + "/results/" + dataset_name + "/"
                export_directory = result_base_directory + platform + "/"
                evaluation_dir = result_base_directory + "evaluation_full_align/"

                rospack = rospkg.RosPack()

                for dataset in datasets:
                    export_directory_run = export_directory + name + "/" + platform + "_" + name + "_" + dataset

                    cmd = "roslaunch ekf_yrio " + launch_file + \
                          " rosbag:=" + dataset + ".bag" + \
                          " pkg:=" + filter["pkg"] + \
                          " export_directory:=" + export_directory_run + \
                          " dataset_dir:=" + rosbag_dir + \
                          " ground_truth_csv:=" + dataset + "_pseudo_ground_truth.csv" + \
                          " config:=" + config_file + " " + filter["params"]
                    print(cmd)
                    os.system(cmd)

                    config_file_eval_run = open(export_directory + name + "/" + platform + "_" + name + "_" + dataset + "/eval_cfg.yaml", "w")
                    config_file_eval_run.write(eval_config_align)
                    config_file_eval_run.close()

                if not os.path.isdir(evaluation_dir):
                    os.mkdir(evaluation_dir)

            ws = "  "
            s = "Datasets:\n"
            for dataset in datasets:
                s += ws + dataset + ":\n"
                s += ws + ws + "label: " + dataset + "\n"

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
