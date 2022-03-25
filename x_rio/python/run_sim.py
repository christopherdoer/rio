#!/usr/bin/env python

"""
This file is part of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement
import pickle
import numpy as np
import time
import threading
import matplotlib.pyplot as plt
import os

import tf.transformations
import rospy
import rospkg

from x_rio.msg import XRioState, XRioCovariance


def euler_to_quat(eul):
    return tf.transformations.quaternion_from_euler(eul[0], eul[1], eul[2])[[3, 0, 1, 2]]


def from_vec3_msg(v):
    return np.array([v.x, v.y, v.z])


class EvaluateSim:
    def __init__(self, dataset, ground_truth_file, prefix=""):
        rospy.init_node('generate_sim_data', anonymous=True)
        self.dataset = dataset
        self.prefix = prefix

        data_raw = pickle.load(open(ground_truth_file, 'r'))
        data_states = data_raw[:, 1:]

        self.t = data_raw[:, 0]
        self.p_n = data_states[:, :3]
        self.v_n = data_states[:, 3:6]
        self.eul_n_deg = np.rad2deg(data_states[:, 6:9])
        self.bias_acc = data_states[:, 9:12]
        self.bias_w_deg = np.rad2deg(data_states[:, 12:15])
        self.radar_extrinsics_gt = data_states[:, 15:33]

        self.state_list = []
        self.sigma_list = []

        self.states_runs = []
        self.sigmas_runs = []
        self.lock = threading.Lock()
        self.last_callback = None

        self.sub_state = rospy.Subscriber("/rio/state", XRioState, self.callback_state, queue_size=1000000)
        self.sub_covariance = rospy.Subscriber("/rio/covariance", XRioCovariance, self.callback_covariance, queue_size=1000000)

    def callback_state(self, msg=XRioState()):
        with self.lock:
            self.last_callback = time.time()
            state = [msg.header.stamp.to_sec(),
                     from_vec3_msg(msg.p),
                     from_vec3_msg(msg.v),
                     from_vec3_msg(msg.eul_deg),
                     from_vec3_msg(msg.b_a),
                     from_vec3_msg(msg.b_g_deg),
                     msg.b_alt]
            for k in range(len(msg.l_b_r)):
                state.append(np.hstack((from_vec3_msg(msg.l_b_r[k]), from_vec3_msg(msg.eul_b_r_deg[k]))))
            self.state_list.append(np.hstack(state))

    def callback_covariance(self, msg=XRioCovariance()):
        with self.lock:
            self.last_callback = time.time()
            sigmas = [msg.header.stamp.to_sec(),
                      from_vec3_msg(msg.sigma_p),
                      from_vec3_msg(msg.sigma_v),
                      from_vec3_msg(msg.sigma_eul_deg),
                      from_vec3_msg(msg.sigma_b_a),
                      from_vec3_msg(msg.sigma_b_g_deg),
                      msg.sigma_alt]
            for k in range(len(msg.sigma_l_b_r)):
                sigmas.append(np.hstack((from_vec3_msg(msg.sigma_l_b_r[k]), from_vec3_msg(msg.sigma_eul_b_r_deg[k]))))
            self.sigma_list.append(np.hstack(sigmas))

    def evaluate(self):
        N_radar = (self.states_runs[0].shape[1] - 16) / 6

        fig_data, (ax_p, ax_v, ax_eul, ax_b_a, ax_b_w, ax_aux) = plt.subplots(6, 3, sharex=True)
        fig_extrinsics, ax = plt.subplots(N_radar * 2, 3, sharex=True)

        fig_data.suptitle("Base states " + self.dataset + " N_radar=" + str(N_radar), fontsize="large")
        fig_extrinsics.suptitle("Extrinsics " + self.dataset + " N_radar=" + str(N_radar), fontsize="large")

        p_err_list = []
        v_err_list = []
        eul_err_list = []
        for j in range(len(self.states_runs)):
            states = self.states_runs[j]
            sigmas = self.sigmas_runs[j]

            # convert states
            t_filter = states[:, 0]
            p_filter = states[:, 1:4]
            v_filter = states[:, 4:7]
            eul_filter_deg = states[:, 7:10]
            b_a_filter = states[:, 10:13]
            b_w_filter_deg = states[:, 13:16]
            b_alt_filter = states[:, 16]

            # convert variances
            t_sigma = sigmas[:, 0]
            sigma_p_filter = sigmas[:, 1:4]
            sigma_v_filter = sigmas[:, 4:7]
            sigma_eul_filter_deg = sigmas[:, 7:10]
            sigma_b_a_filter = sigmas[:, 10:13]
            sigma_b_w_filter_deg = sigmas[:, 13:16]
            sigma_alt_filter = sigmas[:, 16]

            # interpolate gt
            p_gt_inter = np.vstack((np.interp(t_filter, self.t, self.p_n[:, 0]),
                                    np.interp(t_filter, self.t, self.p_n[:, 1]),
                                    np.interp(t_filter, self.t, self.p_n[:, 2]))).transpose()
            v_gt_inter = np.vstack((np.interp(t_filter, self.t, self.v_n[:, 0]),
                                    np.interp(t_filter, self.t, self.v_n[:, 1]),
                                    np.interp(t_filter, self.t, self.v_n[:, 2]))).transpose()
            eul_gt_deg_inter = np.vstack((np.interp(t_filter, self.t, self.eul_n_deg[:, 0]),
                                          np.interp(t_filter, self.t, self.eul_n_deg[:, 1]),
                                          np.interp(t_filter, self.t, self.eul_n_deg[:, 2]))).transpose()
            b_a_gt_inter = np.vstack((np.interp(t_filter, self.t, self.bias_acc[:, 0]),
                                      np.interp(t_filter, self.t, self.bias_acc[:, 1]),
                                      np.interp(t_filter, self.t, self.bias_acc[:, 2]))).transpose()
            b_w_gt_deg_inter = np.vstack((np.interp(t_filter, self.t, self.bias_w_deg[:, 0]),
                                          np.interp(t_filter, self.t, self.bias_w_deg[:, 1]),
                                          np.interp(t_filter, self.t, self.bias_w_deg[:, 2]))).transpose()

            # calc errors
            p_err = p_filter - p_gt_inter
            v_err = v_filter - v_gt_inter
            eul_deg_err = np.mod(eul_filter_deg - eul_gt_deg_inter + 180., 360.) - 180.
            b_a_err = b_a_filter - b_a_gt_inter
            b_w_deg_err = b_w_filter_deg - b_w_gt_deg_inter
            b_alt_err = b_alt_filter

            p_err_list.append(p_err)
            v_err_list.append(v_err)
            eul_err_list.append(eul_deg_err)

            # plot
            xyz = "xyz"
            for k in range(3):
                ax_p[k].plot(t_filter, p_err[:, k])
                ax_v[k].plot(t_filter, v_err[:, k])
                ax_eul[k].plot(t_filter, eul_deg_err[:, k])
                ax_b_a[k].plot(t_filter, b_a_err[:, k])
                ax_b_w[k].plot(t_filter, b_w_deg_err[:, k])

                if j == 1:
                    ax_p[k].plot(t_sigma, sigma_p_filter[:, k] * 3, "r--", linewidth=2)
                    ax_p[k].plot(t_sigma, -sigma_p_filter[:, k] * 3, "r--", linewidth=2)
                    ax_v[k].plot(t_sigma, sigma_v_filter[:, k] * 3, "r--", linewidth=2)
                    ax_v[k].plot(t_sigma, -sigma_v_filter[:, k] * 3, "r--", linewidth=2)
                    ax_eul[k].plot(t_sigma, sigma_eul_filter_deg[:, k] * 3, "r--", linewidth=2)
                    ax_eul[k].plot(t_sigma, -sigma_eul_filter_deg[:, k] * 3, "r--", linewidth=2)
                    ax_b_a[k].plot(t_sigma, sigma_b_a_filter[:, k] * 3, "r--", linewidth=2)
                    ax_b_a[k].plot(t_sigma, -sigma_b_a_filter[:, k] * 3, "r--", linewidth=2)
                    ax_b_w[k].plot(t_sigma, sigma_b_w_filter_deg[:, k] * 3, "r--", linewidth=2)
                    ax_b_w[k].plot(t_sigma, -sigma_b_w_filter_deg[:, k] * 3, "r--", linewidth=2)

                    ax_p[k].set_title("p_err_" + xyz[k])
                    ax_p[k].grid(True)
                    ax_v[k].set_title("v_err_" + xyz[k])
                    ax_v[k].grid(True)
                    ax_eul[k].set_title("eul_err_" + xyz[k])
                    ax_eul[k].grid(True)
                    ax_b_a[k].set_title("b_a_err_" + xyz[k])
                    ax_b_a[k].grid(True)
                    ax_b_w[k].set_title("b_w_err_" + xyz[k])
                    ax_b_w[k].grid(True)

            ax_aux[0].plot(t_filter, b_alt_filter)
            ax_aux[0].plot(t_sigma, sigma_alt_filter * 3, "r--", linewidth=2)
            ax_aux[0].plot(t_sigma, -sigma_alt_filter * 3, "r--", linewidth=2)
            ax_aux[0].set_title("b_alt_err")
            ax_aux[0].grid(True)

            radar_extinsics_filter = states[:, 17:]
            radar_extinsics_sigmas = sigmas[:, 17:]
            radar_extrinsics_gt_inter = np.zeros(radar_extinsics_filter.shape)
            for k in range(N_radar):
                for n in range(3):
                    radar_extrinsics_gt_inter[:, k * 6 + n] = np.interp(t_filter, self.t, self.radar_extrinsics_gt[:, k * 6 + n])
                for n in range(3):
                    radar_extrinsics_gt_inter[:, k * 6 + 3 + n] = np.rad2deg(np.interp(t_filter, self.t, self.radar_extrinsics_gt[:, k * 6 + 3 + n]))

            radar_extinsics_err = radar_extinsics_filter - radar_extrinsics_gt_inter

            for k in range(N_radar):
                for n in range(3):
                    ax[k * 2, n].plot(t_filter, radar_extinsics_err[:, k * 6 + n])
                    ax[k * 2 + 1, n].plot(t_filter, np.mod(radar_extinsics_err[:, k * 6 + n + 3] + 180., 360.) - 180.)

                    if j == 1:
                        ax[k * 2, n].plot(t_sigma, radar_extinsics_sigmas[:, k * 6 + n] * 3, "r--", linewidth=2)
                        ax[k * 2, n].plot(t_sigma, -radar_extinsics_sigmas[:, k * 6 + n] * 3, "r--", linewidth=2)
                        ax[k * 2 + 1, n].plot(t_sigma, radar_extinsics_sigmas[:, k * 6 + n + 3] * 3, "r--", linewidth=2)
                        ax[k * 2 + 1, n].plot(t_sigma, -radar_extinsics_sigmas[:, k * 6 + n + 3] * 3, "r--", linewidth=2)
                        ax[k * 2, n].set_title("err_l_b_r_" + xyz[n])
                        ax[k * 2, n].grid(True)
                        ax[k * 2 + 1, n].set_title("err_eul_b_r_" + xyz[n])
                        ax[k * 2 + 1, n].grid(True)

    def run_mcl_extrinsics(self, N_MCL, N_radar):

        for k in range(N_MCL):
            print("##############################")
            print("Monte Carlo Iteration %d / %d" % (k + 1, N_MCL))
            print("##############################")

            launch_cmd = "roslaunch x_rio sim_x_rio.launch " + "n_radar:=" + str(N_radar) + " "

            sigma_l = 0.018
            sigma_eul = np.deg2rad(1.35)
            sigma_b_a = 0.1
            for n in range(N_radar):
                l_b_r_gt = self.radar_extrinsics_gt[0, n * 6:n * 6 + 3]
                eul_b_r_gt = self.radar_extrinsics_gt[0, n * 6 + 3:n * 6 + 6]

                l_b_r_run = l_b_r_gt + sigma_l * np.random.randn(3)
                eul_b_r_run = eul_b_r_gt + sigma_eul * np.random.randn(3)
                q_b_r_run = euler_to_quat(eul_b_r_run)
                N = str(n)
                launch_cmd += " l_b_r_x_" + N + ":=" + str(l_b_r_run[0]) \
                              + " l_b_r_y_" + N + ":=" + str(l_b_r_run[1]) \
                              + " l_b_r_z_" + N + ":=" + str(l_b_r_run[2]) \
                              + " q_b_r_w_" + N + ":=" + str(q_b_r_run[0]) \
                              + " q_b_r_x_" + N + ":=" + str(q_b_r_run[1]) \
                              + " q_b_r_y_" + N + ":=" + str(q_b_r_run[2]) \
                              + " q_b_r_z_" + N + ":=" + str(q_b_r_run[3]) \
                              + " b_0_a_x:=" + str(sigma_b_a * np.random.randn(1)[0]) \
                              + " b_0_a_y:=" + str(sigma_b_a * np.random.randn(1)[0]) \
                              + " b_0_a_z:=" + str(sigma_b_a * np.random.randn(1)[0])

            os.system(launch_cmd + "> /dev/null")
            while not rospy.is_shutdown():
                with self.lock:
                    if self.last_callback is not None and time.time() - self.last_callback > 0.5:
                        self.states_runs.append(np.vstack(self.state_list))
                        self.sigmas_runs.append(np.vstack(self.sigma_list))
                        self.state_list = []
                        self.sigma_list = []
                        break
                time.sleep(0.1)

        self.sub_state.unregister()
        self.evaluate()


if __name__ == "__main__":

    start = time.time()
    ground_truth_file = rospkg.rospack.RosPack().get_path("x_rio") + "/../demo_datasets/x_rio/sim.pickle"
    eval = EvaluateSim("demo_sim", ground_truth_file)
    eval.run_mcl_extrinsics(N_MCL=10, N_radar=3)
    print("Took %0.2f min" % ((time.time() - start) / 60.))

    while not rospy.is_shutdown():
        plt.pause(0.1)
