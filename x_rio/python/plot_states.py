#!/usr/bin/env python

"""
This file is part of RIO - Radar Inertial Odometry and Radar based ego velocity estimation.
@author Christopher Doer <christopher.doer@kit.edu>
"""

from __future__ import with_statement
import numpy as np
import time
import threading
import matplotlib.pyplot as plt

import rospy

from x_rio.msg import XRioState, XRioCovariance


def from_vec3_msg(v):
    return np.array([v.x, v.y, v.z])


class PlotStates:
    def __init__(self):
        rospy.init_node('plot_states', anonymous=True)

        self.state_list = []
        self.sigma_list = []
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

    def plot(self):
        N_radar = int((self.state_list[0].shape[0] - 16) / 6)

        fig_data, (ax_p, ax_v, ax_eul, ax_b_a, ax_b_w, ax_aux) = plt.subplots(6, 3, sharex=True)
        fig_data_s, (ax_p_s, ax_v_s, ax_eul_s, ax_b_a_s, ax_b_w_s, ax_aux_s) = plt.subplots(6, 3, sharex=True)
        fig_extrinsics, ax = plt.subplots(N_radar * 2, 3, sharex=True)

        fig_data.suptitle("Base states N_radar=" + str(N_radar), fontsize="large")
        fig_data_s.suptitle("Estimated +- 3 Sigma Bounds", fontsize="large")
        fig_extrinsics.suptitle("Extrinsics N_radar=" + str(N_radar), fontsize="large")

        states = np.vstack(self.state_list)
        sigmas = np.vstack(self.sigma_list)
        min_idx = min([states.shape[0], sigmas.shape[0]])
        states = states[:min_idx, :]
        sigmas = sigmas[:min_idx, :]

        # convert states
        t_filter = states[:, 0]
        p_filter = states[:, 1:4]
        v_filter = states[:, 4:7]
        eul_filter_deg = states[:, 7:10]
        b_a_filter = states[:, 10:13]
        b_w_filter_deg = states[:, 13:16]
        b_alt_filter = states[:, 16]

        eul_filter_deg[:, 0] = np.mod(eul_filter_deg[:, 0], 360.0)

        # convert variances
        t_sigma = sigmas[:, 0]
        sigma_p_filter = sigmas[:, 1:4]
        sigma_v_filter = sigmas[:, 4:7]
        sigma_eul_filter_deg = sigmas[:, 7:10]
        sigma_b_a_filter = sigmas[:, 10:13]
        sigma_b_w_filter_deg = sigmas[:, 13:16]
        sigma_alt_filter = sigmas[:, 16]

        # plot
        xyz = "xyz"
        for k in range(3):
            ax_p[k].plot(t_filter, p_filter[:, k])
            ax_v[k].plot(t_filter, v_filter[:, k])
            ax_eul[k].plot(t_filter, eul_filter_deg[:, k])
            ax_b_a[k].plot(t_filter, b_a_filter[:, k])
            ax_b_w[k].plot(t_filter, b_w_filter_deg[:, k])

            ax_p[k].plot(t_sigma, p_filter[:, k] + sigma_p_filter[:, k] * 3, "r--", linewidth=2)
            ax_p[k].plot(t_sigma, p_filter[:, k] - sigma_p_filter[:, k] * 3, "r--", linewidth=2)
            ax_v[k].plot(t_sigma, v_filter[:, k] + sigma_v_filter[:, k] * 3, "r--", linewidth=2)
            ax_v[k].plot(t_sigma, v_filter[:, k] - sigma_v_filter[:, k] * 3, "r--", linewidth=2)
            ax_eul[k].plot(t_sigma, eul_filter_deg[:, k] + sigma_eul_filter_deg[:, k] * 3, "r--", linewidth=2)
            ax_eul[k].plot(t_sigma, eul_filter_deg[:, k] - sigma_eul_filter_deg[:, k] * 3, "r--", linewidth=2)
            ax_b_a[k].plot(t_sigma, b_a_filter[:, k] + sigma_b_a_filter[:, k] * 3, "r--", linewidth=2)
            ax_b_a[k].plot(t_sigma, b_a_filter[:, k] - sigma_b_a_filter[:, k] * 3, "r--", linewidth=2)
            ax_b_w[k].plot(t_sigma, b_w_filter_deg[:, k] + sigma_b_w_filter_deg[:, k] * 3, "r--", linewidth=2)
            ax_b_w[k].plot(t_sigma, b_w_filter_deg[:, k] - sigma_b_w_filter_deg[:, k] * 3, "r--", linewidth=2)

            ax_p[k].set_title("p_" + xyz[k])
            ax_p[k].grid(True)
            ax_v[k].set_title("v_" + xyz[k])
            ax_v[k].grid(True)
            ax_eul[k].set_title("eul_" + xyz[k])
            ax_eul[k].grid(True)
            ax_b_a[k].set_title("b_a_" + xyz[k])
            ax_b_a[k].grid(True)
            ax_b_w[k].set_title("b_w_" + xyz[k])
            ax_b_w[k].grid(True)

            ax_p_s[k].plot(t_sigma, + sigma_p_filter[:, k] * 3, "r--", linewidth=2)
            ax_p_s[k].plot(t_sigma, - sigma_p_filter[:, k] * 3, "r--", linewidth=2)
            ax_v_s[k].plot(t_sigma, + sigma_v_filter[:, k] * 3, "r--", linewidth=2)
            ax_v_s[k].plot(t_sigma, - sigma_v_filter[:, k] * 3, "r--", linewidth=2)
            ax_eul_s[k].plot(t_sigma, + sigma_eul_filter_deg[:, k] * 3, "r--", linewidth=2)
            ax_eul_s[k].plot(t_sigma, - sigma_eul_filter_deg[:, k] * 3, "r--", linewidth=2)
            ax_b_a_s[k].plot(t_sigma, + sigma_b_a_filter[:, k] * 3, "r--", linewidth=2)
            ax_b_a_s[k].plot(t_sigma, - sigma_b_a_filter[:, k] * 3, "r--", linewidth=2)
            ax_b_w_s[k].plot(t_sigma, + sigma_b_w_filter_deg[:, k] * 3, "r--", linewidth=2)
            ax_b_w_s[k].plot(t_sigma, - sigma_b_w_filter_deg[:, k] * 3, "r--", linewidth=2)

            ax_p_s[k].set_title("p_" + xyz[k])
            ax_p_s[k].grid(True)
            ax_v_s[k].set_title("v_" + xyz[k])
            ax_v_s[k].grid(True)
            ax_eul_s[k].set_title("eul_" + xyz[k])
            ax_eul_s[k].grid(True)
            ax_b_a_s[k].set_title("b_a_" + xyz[k])
            ax_b_a_s[k].grid(True)
            ax_b_w_s[k].set_title("b_w_" + xyz[k])
            ax_b_w_s[k].grid(True)

        ax_aux[0].plot(t_filter, b_alt_filter)
        ax_aux[0].plot(t_sigma, b_alt_filter + sigma_alt_filter * 3, "r--", linewidth=2)
        ax_aux[0].plot(t_sigma, b_alt_filter - sigma_alt_filter * 3, "r--", linewidth=2)
        ax_aux[0].set_title("b_alt")
        ax_aux[0].grid(True)

        ax_aux_s[0].plot(t_sigma, sigma_alt_filter * 3, "r--", linewidth=2)
        ax_aux_s[0].plot(t_sigma, - sigma_alt_filter * 3, "r--", linewidth=2)
        ax_aux_s[0].set_title("b_alt")
        ax_aux_s[0].grid(True)

        radar_extinsics_filter = states[:, 17:]
        radar_extinsics_sigmas = sigmas[:, 17:]

        for k in range(N_radar):
            for n in range(3):
                ax[k * 2, n].plot(t_filter, radar_extinsics_filter[:, k * 6 + n])
                ax[k * 2 + 1, n].plot(t_filter, np.mod(radar_extinsics_filter[:, k * 6 + n + 3] + 180., 360.) - 180.)

                ax[k * 2, n].plot(t_filter, radar_extinsics_filter[:, k * 6 + n] + radar_extinsics_sigmas[:, k * 6 + n] * 3, "r--", linewidth=2)
                ax[k * 2, n].plot(t_filter, radar_extinsics_filter[:, k * 6 + n] - radar_extinsics_sigmas[:, k * 6 + n] * 3, "r--", linewidth=2)
                ax[k * 2 + 1, n].plot(t_filter,
                                      np.mod(radar_extinsics_filter[:, k * 6 + n + 3] + 180., 360.) - 180. + radar_extinsics_sigmas[:, k * 6 + n + 3] * 3,
                                      "r--", linewidth=2)
                ax[k * 2 + 1, n].plot(t_filter,
                                      np.mod(radar_extinsics_filter[:, k * 6 + n + 3] + 180., 360.) - 180. - radar_extinsics_sigmas[:, k * 6 + n + 3] * 3,
                                      "r--", linewidth=2)
                ax[k * 2, n].set_title("l_b_r_" + xyz[n])
                ax[k * 2, n].grid(True)
                ax[k * 2 + 1, n].set_title("eul_b_r_" + xyz[n])
                ax[k * 2 + 1, n].grid(True)

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.last_callback is not None and time.time() - self.last_callback > 0.5:
                    break
            time.sleep(0.1)

        self.sub_state.unregister()

        print("#### preparing plots ####")

        self.plot()
        while not rospy.is_shutdown():
            plt.pause(0.1)


if __name__ == "__main__":
    plot = PlotStates()
    plot.run()
