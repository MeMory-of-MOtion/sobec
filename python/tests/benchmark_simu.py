#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
import matplotlib.pyplot as plt

import configuration as conf

from bullet_Talos import BulletTalos
import pinocchio as pin
from sobec import RobotDesigner, WBC, HorizonManager, ModelMaker
import numpy as np
from time import time


def foot_trajectory(T, time_to_land, translation, trajectory="sine"):
    """Functions to generate steps."""
    tmax = conf.T1contact
    landing_advance = 3
    takeoff_delay = 9
    foot_height = 0.05
    z = []
    if trajectory == "sine":
        for t in range(
            time_to_land - landing_advance, time_to_land - landing_advance - T, -1
        ):
            z.append(
                0
                if t < 0 or t > tmax - landing_advance - takeoff_delay
                else (np.sin(t * np.pi / (tmax - landing_advance - takeoff_delay)))
                * foot_height
            )
    else:
        for t in range(
            time_to_land - landing_advance, time_to_land - 2 * landing_advance - T, -1
        ):
            z.append(
                0
                if t < 0 or t > tmax - landing_advance - takeoff_delay
                else (
                    1 - np.cos(2 * t * np.pi / (tmax - landing_advance - takeoff_delay))
                )
                * foot_height
                / 2
            )
    return [np.array([translation[0], translation[1], move_z]) for move_z in z]


def print_trajectory(ref):
    u = [y.translation for y in ref]
    t = np.array([z[2] for z in u])
    fig = plt.figure()
    ax = fig.gca()
    ax.plot(t)


# ####### CONFIGURATION  ############
# ### RobotWrapper
design_conf = dict(
    urdfPath=conf.modelPath + conf.URDF_SUBPATH,
    srdfPath=conf.modelPath + conf.SRDF_SUBPATH,
    leftFootName=conf.lf_frame_name,
    rightFootName=conf.rf_frame_name,
    robotDescription="",
    controlledJointsNames=[
        "root_joint",
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "torso_1_joint",
        "torso_2_joint",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
    ],
)
design = RobotDesigner()
design.initialize(design_conf)

# Vector of Formulations
MM_conf = dict(
    timeStep=conf.DT,
    gravity=conf.gravity,
    mu=conf.mu,
    coneBox=conf.cone_box,
    minNforce=conf.minNforce,
    maxNforce=conf.maxNforce,
    comHeight=conf.normal_height,
    omega=conf.omega,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wVCoM=conf.wVCoM,
    wWrenchCone=conf.wWrenchCone,
    wFootTrans=conf.wFootTrans,
    wFootXYTrans=conf.wFootXYTrans,
    wFootRot=conf.wFootRot,
    wGroundCol=conf.wGroundCol,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMaker()
formuler.initialize(MM_conf, design)
all_models = formuler.formulateHorizon(lenght=conf.T)

# Horizon

H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, all_models[-1])

# MPC
wbc_conf = dict(
    horizonSteps=conf.preview_steps,
    totalSteps=conf.total_steps,
    T=conf.T,
    TdoubleSupport=conf.T2contact,
    TsingleSupport=conf.T1contact,
    Tstep=conf.Tstep,
    ddpIteration=conf.ddpIteration,
    Dt=conf.DT,
    simu_step=conf.simu_period,
    Nc=conf.Nc,
)

mpc = WBC()
mpc.initialize(
    wbc_conf,
    design,
    horizon,
    design.get_q0Complete(),
    design.get_v0Complete(),
    "actuationTask",
)
mpc.generateWalkigCycle(formuler)

device = BulletTalos(conf, design.get_rModelComplete())
device.initializeJoints(design.get_q0Complete().copy())
device.showTargetToTrack(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])
q_current, v_current = device.measureState()

# ### SIMULATION LOOP ###
t1 = time()
sum_solve_time = 0
sum_bullet_time = 0
for s in range(conf.T_total * conf.Nc):
    if mpc.timeToSolveDDP(s):
        # Trajectory
        LF_refs = foot_trajectory(
            len(mpc.ref_LF_poses),
            mpc.t_land_LF[0],
            mpc.ref_LF_poses[0].translation,
            "cosine",
        )[len(mpc.ref_LF_poses) - 1]
        RF_refs = foot_trajectory(
            len(mpc.ref_RF_poses),
            mpc.t_land_RF[0],
            mpc.ref_RF_poses[0].translation,
            "cosine",
        )[len(mpc.ref_LF_poses) - 1]

        mpc.ref_LF_poses[len(mpc.ref_LF_poses) - 1] = pin.SE3(np.eye(3), LF_refs)
        mpc.ref_RF_poses[len(mpc.ref_LF_poses) - 1] = pin.SE3(np.eye(3), RF_refs)

    t_solve_start = time()
    mpc.iterate(s, q_current, v_current)
    torques = horizon.currentTorques(mpc.x0)
    t_solve_end = time()
    sum_solve_time += t_solve_end - t_solve_start

    # pybullet
    t_bullet_start = time()
    device.execute(torques)
    q_current, v_current = device.measureState()
    device.moveMarkers(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])
    t_bullet_end = time()
    sum_bullet_time += t_bullet_end - t_bullet_start

t2 = time()
total_t = t2 - t1
iteration_time = total_t / s
average_solve_time = sum_solve_time / s
average_bullet_time = sum_bullet_time / s
resting_time = iteration_time - average_bullet_time - average_solve_time

print("#####################  Benchmark Times ####################### ")
print("##")
print("## Time per iteration:\t\t ", iteration_time)
print("##")
print("## Time solving ddp problem:\t ", average_solve_time)
print("##")
print("## Time solving pyBullet:\t\t ", average_bullet_time)
print("##")
print("## Other times spent:\t\t\t ", resting_time)
print("##")
print("#############################################################")

device.close()
