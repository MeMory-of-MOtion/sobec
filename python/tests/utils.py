#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""

import ndcurves
import numpy as np
import matplotlib.pyplot as plt

import time
import os


DEFAULT_SAVE_DIR = "/local/src/sobec/python/tests"


def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = np.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v


def axisangle_to_q(v, theta):
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = np.cos(theta)
    x = x * np.sin(theta)
    y = y * np.sin(theta)
    z = z * np.sin(theta)
    return np.array([x, y, z, w])


def q_mult(q1, q2):
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return np.array([x, y, z, w])


def yawRotation(yaw):
    Ro = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )
    return Ro


def extractYaw(Ro):
    return np.arctan2(Ro[1, 0], Ro[0, 0])


def save_trajectory(
    xss,
    uss,
    LF_pose,
    RF_pose,
    LF_force,
    RF_force,
    save_name=None,
    save_dir=DEFAULT_SAVE_DIR,
):
    """
    Saves data to a compressed npz file (binary)
    """
    simu_data = {}
    simu_data["xss"] = xss
    simu_data["uss"] = uss
    simu_data["LF_pose"] = LF_pose
    simu_data["RF_pose"] = RF_pose
    simu_data["LF_force"] = LF_force
    simu_data["RF_force"] = RF_force
    print("Compressing & saving data...")
    if save_name is None:
        save_name = "sim_data_NO_NAME" + str(time.time())
    if save_dir is None:
        save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../data"))
    save_path = save_dir + "/" + save_name + ".npz"
    np.savez_compressed(save_path, data=simu_data)
    print("Saved data to " + str(save_path) + " !")


def load_data(npz_file):
    """
    Loads a npz archive of sim_data into a dict
    """
    d = np.load(npz_file, allow_pickle=True, encoding="latin1")
    return d["data"][()]


# from time import time
def defineBezier(height, time_init, time_final, placement_init, placement_final):
    wps = np.zeros([3, 9])
    for i in range(4):  # init position. init vel,acc and jerk == 0
        wps[:, i] = placement_init.translation
    # compute mid point (average and offset along z)
    wps[:, 4] = (placement_init.translation + placement_final.translation) / 2.0
    wps[2, 4] += height
    for i in range(5, 9):  # final position. final vel,acc and jerk == 0
        wps[:, i] = placement_final.translation
    translation = ndcurves.bezier(wps, time_init, time_final)
    pBezier = ndcurves.piecewise_SE3(
        ndcurves.SE3Curve(
            translation, placement_init.rotation, placement_final.rotation
        )
    )
    return pBezier


def foot_trajectory(T, time_to_land, initial_pose, final_pose, trajectory_swing, TsingleSupport):
    """Functions to generate steps."""
    landing_advance = 0
    takeoff_delay = 0
    placement = []
    for t in range(
        time_to_land - landing_advance, time_to_land - landing_advance - T, -1
    ):
        if t <= 0:
            placement.append(final_pose)
        elif t > TsingleSupport - landing_advance - takeoff_delay:
            placement.append(initial_pose)
        else:
            swing_pose = initial_pose.copy()
            swing_pose.translation = trajectory_swing.translation(
                float(TsingleSupport - t) / float(TsingleSupport)
            )
            swing_pose.rotation = trajectory_swing.rotation(
                float(TsingleSupport - t) / float(TsingleSupport)
            )
            placement.append(swing_pose)

    return placement


def print_trajectory(ref):
    u = [y.translation for y in ref]
    t = np.array([z[2] for z in u])
    fig = plt.figure()
    ax = fig.gca()
    ax.plot(t)
    ax.set_ylim(0, 0.05)