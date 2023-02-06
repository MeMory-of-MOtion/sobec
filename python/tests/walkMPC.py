#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
import matplotlib.pyplot as plt

import configuration as conf

from bullet_Talos import BulletTalos
from cricket.virtual_talos import VirtualPhysics

# from pyRobotWrapper import PinTalos
# from pyMPC import CrocoWBC
# from pyModelMaker import modeller
# import pinocchio as pin
from sobec import (
    RobotDesigner,
    WBCHorizon,
    HorizonManager,
    ModelMaker,
    Support,
    Experiment,
    # LocomotionType,
    Flex,
)
import ndcurves
import numpy as np
import time
import os

# from scipy.spatial.transform import Rotation as R

# from flex_joints import Flex

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


def foot_trajectory(T, time_to_land, initial_pose, final_pose, trajectory_swing):
    """Functions to generate steps."""
    # tmax = conf.TsingleSupport
    landing_advance = 0
    takeoff_delay = 0
    placement = []
    for t in range(
        time_to_land - landing_advance, time_to_land - landing_advance - T, -1
    ):
        if t <= 0:
            placement.append(final_pose)
        elif t > conf.TsingleSupport - landing_advance - takeoff_delay:
            placement.append(initial_pose)
        else:
            swing_pose = initial_pose.copy()
            swing_pose.translation = trajectory_swing.translation(
                float(conf.TsingleSupport - t) / float(conf.TsingleSupport)
            )
            swing_pose.rotation = trajectory_swing.rotation(
                float(conf.TsingleSupport - t) / float(conf.TsingleSupport)
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
        # "arm_left_1_joint",
        # "arm_left_2_joint",
        # "arm_left_3_joint",
        # "arm_left_4_joint",
        # "arm_right_1_joint",
        # "arm_right_2_joint",
        # "arm_right_3_joint",
        # "arm_right_4_joint",
    ],
)
design = RobotDesigner()
design.initialize(design_conf)

# Rotate initial configuration by theta
theta = 0  # np.pi / 2
xStart = design.get_x0().copy()
qYaw = axisangle_to_q(np.array([0, 0, 1]), theta)
xStart[3:7] = q_mult(qYaw, xStart[3:7])

design.set_q0(xStart[: design.get_rModel().nq])
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
    height=1,
    dist=1,
    width=1,
    footSize=conf.footSize,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wWrenchCone=conf.wWrenchCone,
    wForceTask=conf.wForceTask,
    wCoP=conf.wCoP,
    wDCM=conf.wDCM,
    wVCoM=0,
    wFootRot=0,
    wPCoM=0,
    wFlyHigh=0,
    wVelFoot=0,
    wColFeet=0,
    wBaseRot=0,
    flyHighSlope=1,
    footMinimalDistance=0.2,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    forceWeights=conf.forceWeights,
    lowKinematicLimits=conf.lowKinematicLimits,
    highKinematicLimits=conf.highKinematicLimits,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMaker()
formuler.initialize(MM_conf, design)
cycles = [Support.DOUBLE for i in range(conf.T)]
all_models = formuler.formulateHorizon(supports=cycles, experiment=Experiment.WALK)
ter_model = formuler.formulateTerminalStepTracker(Support.DOUBLE)

# Horizon
H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()

horizon.initialize(H_conf, xStart, all_models, ter_model)

# MPC
wbc_conf = dict(
    totalSteps=conf.total_steps,
    T=conf.T,
    TdoubleSupport=conf.TdoubleSupport,
    TsingleSupport=conf.TsingleSupport,
    Tstep=conf.Tstep,
    ddpIteration=conf.ddpIteration,
    Dt=conf.DT,
    simu_step=conf.simu_period,
    min_force=150,
    support_force=-design.getRobotMass() * conf.gravity[2],
    Nc=conf.Nc,
)

# Flex
flex = Flex()
flex.initialize(
    dict(
        left_stiffness=np.array(conf.H_stiff[:2]),
        right_stiffness=np.array(conf.H_stiff[2:]),
        left_damping=np.array(conf.H_damp[:2]),
        right_damping=np.array(conf.H_damp[2:]),
        flexToJoint=conf.flexToJoint,
        dt=conf.simu_period,
        MA_duration=0.01,
        left_hip_indices=np.array([0, 1, 2]),
        right_hip_indices=np.array([6, 7, 8]),
        filtered=True,
    )
)
qStartComplete = design.get_q0Complete().copy()
qStartComplete[3:7] = q_mult(qYaw, qStartComplete[3:7])
mpc = WBCHorizon()
mpc.initialize(
    wbc_conf, design, horizon, qStartComplete, design.get_v0Complete(), "actuationTask"
)
mpc.generateFullHorizon(formuler, Experiment.WALK)

if conf.simulator == "bullet":
    device = BulletTalos(conf, design.get_rModelComplete())
    device.initializeJoints(qStartComplete)
    device.showTargetToTrack(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])
    q_current, v_current = device.measureState()

elif conf.simulator == "pinocchio":
    design.rmodelComplete = design.get_rModelComplete()
    design.rmodelComplete.q0 = qStartComplete
    design.rmodelComplete.v0 = design.get_v0Complete()

    device = VirtualPhysics(conf, view=True, block_joints=conf.blocked_joints)
    device.initialize(design.rmodelComplete)
    init_state = device.measure_state(device.Cq0, device.Cv0, device.Cv0 * 0)
    q_current = init_state["q"]
    v_current = init_state["dq"]

# ### SIMULATION LOOP ###
steps = 0
# Define the constant yaw rotation for the feet
yaw = 0

# Define the rotation and translation used to move the feet
translationRight = np.array([conf.xForward, -conf.footSeparation + conf.sidestep, 0])
translationLeft = np.array([conf.xForward, conf.footSeparation, 0])
rotationDiff = yawRotation(yaw)

# Define the first reference trajectories for both feet
starting_position_right = mpc.designer.get_RF_frame().copy()
final_position_right = mpc.designer.get_LF_frame().copy()
yawLeft = extractYaw(mpc.designer.get_LF_frame().rotation)
final_position_right.translation += yawRotation(yawLeft) @ translationRight
final_position_right.rotation = rotationDiff @ final_position_right.rotation

xForward = conf.xForward

starting_position_left = mpc.designer.get_LF_frame().copy()
final_position_left = final_position_right.copy()
yawRight = extractYaw(final_position_right.rotation)
final_position_left.translation += yawRotation(yawRight) @ translationLeft
# final_position_left.rotation = rotationDiff @ final_position_left.rotation

swing_trajectory_right = defineBezier(
    conf.swingApex, 0, 1, starting_position_right, final_position_right
)
swing_trajectory_left = defineBezier(
    conf.swingApex, 0, 1, starting_position_left, final_position_left
)

ref_pose_right = [swing_trajectory_right for i in range(conf.T)]
ref_pose_left = [swing_trajectory_left for i in range(conf.T)]


T_total = conf.total_steps * conf.Tstep + 4 * conf.T

# ## Save trajectory in npz file
xss = []
uss = []
LF_pose = []
RF_pose = []
LF_force = []
RF_force = []

for s in range(5 * T_total * conf.Nc):
    #    time.sleep(0.001)
    if mpc.timeToSolveDDP(s):
        xss.append(mpc.horizon.ddp.xs[0])
        uss.append(mpc.horizon.ddp.us[0])
        LF_pose.append(mpc.designer.get_LF_frame().copy())
        RF_pose.append(mpc.designer.get_RF_frame().copy())
        LF_force.append(
            mpc.horizon.ddp.problem.runningDatas[0]
            .differential.costs.costs["wrench_LF"]
            .residual.contact.f.copy()
        )
        RF_force.append(
            mpc.horizon.ddp.problem.runningDatas[0]
            .differential.costs.costs["wrench_RF"]
            .residual.contact.f.copy()
        )
        # print(mpc.horizon.ddp.problem.runningDatas[0]
        # 	.differential.costs.costs["wrench_RF"]
        # 	.residual.contact.f.linear[2])

        land_LF = -1
        if mpc.land_LF():
            land_LF = mpc.land_LF()[0]

        land_RF = -1
        if mpc.land_RF():
            land_RF = mpc.land_RF()[0]

        takeoff_LF = -1
        if mpc.takeoff_LF():
            takeoff_LF = mpc.takeoff_LF()[0]

        takeoff_RF = -1
        if mpc.takeoff_RF():
            takeoff_RF = mpc.takeoff_RF()[0]
        print(
            "takeoff_RF = " + str(takeoff_RF) + ", landing_RF = ",
            str(land_RF) + ", takeoff_LF = " + str(takeoff_LF) + ", landing_LF = ",
            str(land_LF),
        )

        if land_RF == 0:
            steps += 1
        if land_LF == 0:
            steps += 1

        if steps == conf.total_steps:
            xForward = 0

        if takeoff_RF < conf.TdoubleSupport:
            # print("Update right trajectory")
            starting_position_right = mpc.designer.get_RF_frame().copy()
            final_position_right = mpc.designer.get_LF_frame().copy()
            yawLeft = extractYaw(mpc.designer.get_LF_frame().rotation)
            final_position_right.translation += yawRotation(yawLeft) @ translationRight
            final_position_right.rotation = rotationDiff @ final_position_right.rotation

            starting_position_left = mpc.designer.get_LF_frame().copy()
            final_position_left = final_position_right.copy()
            yawRight = extractYaw(final_position_right.rotation)
            final_position_left.translation += yawRotation(yawRight) @ translationLeft
            # final_position_left.rotation = rotationDiff @ final_position_left.rotation

            swing_trajectory_right = defineBezier(
                conf.swingApex, 0, 1, starting_position_right, final_position_right
            )
            swing_trajectory_left = defineBezier(
                conf.swingApex, 0, 1, starting_position_left, final_position_left
            )
        if takeoff_LF < conf.TdoubleSupport:
            # print("Update left trajectory")
            starting_position_left = mpc.designer.get_LF_frame().copy()
            final_position_left = mpc.designer.get_RF_frame().copy()
            yawRight = extractYaw(mpc.designer.get_RF_frame().rotation)
            final_position_left.translation += yawRotation(yawRight) @ translationLeft
            # final_position_left.rotation = rotationDiff @ final_position_left.rotation

            starting_position_right = mpc.designer.get_RF_frame().copy()
            final_position_right = final_position_left.copy()
            yawLeft = extractYaw(final_position_left.rotation)
            final_position_right.translation += yawRotation(yawLeft) @ translationRight
            final_position_right.rotation = rotationDiff @ final_position_right.rotation

            swing_trajectory_left = defineBezier(
                conf.swingApex, 0, 1, starting_position_left, final_position_left
            )
            swing_trajectory_right = defineBezier(
                conf.swingApex, 0, 1, starting_position_right, final_position_right
            )

        LF_refs = (
            foot_trajectory(
                len(mpc.ref_LF_poses),
                land_LF,
                starting_position_left,
                final_position_left,
                swing_trajectory_left,
            )
            if land_LF > -1
            else ([starting_position_left for i in range(len(mpc.ref_LF_poses))])
        )

        RF_refs = (
            foot_trajectory(
                len(mpc.ref_RF_poses),
                land_RF,
                starting_position_right,
                final_position_right,
                swing_trajectory_right,
            )
            if land_RF > -1
            else ([starting_position_right for i in range(len(mpc.ref_RF_poses))])
        )

        for i in range(len(mpc.ref_LF_poses)):
            mpc.ref_LF_poses[i] = LF_refs[i]
            mpc.ref_RF_poses[i] = RF_refs[i]

        # print_trajectory(mpc.ref_LF_poses)

    mpc.iterate(s, q_current, v_current)
    torques = horizon.currentTorques(mpc.x0)

    if conf.simulator == "bullet":
        device.execute(torques)
        q_current, v_current = device.measureState()
        q_current[3:7] = q_mult(qYaw, q_current[3:7])
        # q_current[3:7] = qStartComplete[3:7]
        device.moveMarkers(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])

    elif conf.simulator == "pinocchio":
        correct_contacts = mpc.horizon.get_contacts(0)
        command = {"tau": torques}
        real_state, _ = device.execute(command, correct_contacts, s)

        # ####### Generating the forces ########## TODO: cast it in functions.

        # LW = mpc.horizon.pinData(0).f[2].linear
        # RW = mpc.horizon.pinData(0).f[8].linear
        # TW = mpc.horizon.pinData(0).f[1].linear

        # if not all(correct_contacts.values()):
        #     Lforce = TW - LW if correct_contacts["leg_left_sole_fix_joint"] else -LW
        #     Rforce = TW - RW if correct_contacts["leg_right_sole_fix_joint"] else -RW
        # else:
        #     Lforce = TW / 2 - LW
        #     Rforce = TW / 2 - RW

        if conf.model_name == "talos_flex":
            qc, dqc = flex.correctEstimatedDeflections(
                torques, real_state["q"][7:], real_state["dq"][6:]  # , Lforce, Rforce
            )

            q_current = np.hstack([real_state["q"][:7], qc])
            v_current = np.hstack([real_state["dq"][:6], dqc])

        elif conf.model_name == "talos":
            q_current = real_state["q"]
            v_current = real_state["dq"]

            # if s == 0:
            # stop

# save_trajectory(
# xss, uss, LF_pose, RF_pose, LF_force, RF_force, save_name="trajectories_xs_us"
# )
if conf.simulator == "bullet":
    device.close()
