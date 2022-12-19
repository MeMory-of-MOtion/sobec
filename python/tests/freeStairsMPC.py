#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
import matplotlib.pyplot as plt

import configurationFree as conf

from bullet_Talos import BulletTalos
from cricket.virtual_talos import VirtualPhysics

# from pyRobotWrapper import PinTalos
# from pyMPC import CrocoWBC
# from pyModelMaker import modeller
import pinocchio as pin
from sobec import (
    RobotDesigner,
    WBCHorizon,
    HorizonManager,
    ModelMaker,
    Flex,
    Support,
    Experiment,
    FootTrajectory,
)
import ndcurves
import numpy as np
import time

DEFAULT_SAVE_DIR = "/local/src/sobec/python/tests"


def yawRotation(yaw):
    Ro = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )
    return Ro


def q_mult(q1, q2):
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return np.array([x, y, z, w])


def axisangle_to_q(v, theta):
    x, y, z = v
    theta /= 2
    w = np.cos(theta)
    x = x * np.sin(theta)
    y = y * np.sin(theta)
    z = z * np.sin(theta)
    return [x, y, z, w]


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
"""
design.get_rModel().inertias[1].lever[1] +=0.02
design.get_rModel().inertias[2].lever[1] +=0.02
design.get_rModel().inertias[3].lever[1] +=0.02
design.get_rModel().inertias[4].lever[1] +=0.02
design.get_rModel().inertias[5].lever[1] +=0.02
design.get_rModel().inertias[6].lever[1] +=0.02
design.get_rModel().inertias[7].lever[1] -=0.02
design.get_rModel().inertias[8].lever[1] -=0.02
design.get_rModel().inertias[9].lever[1] -=0.02
design.get_rModel().inertias[10].lever[1] -=0.02
design.get_rModel().inertias[11].lever[1] -=0.02
design.get_rModel().inertias[12].lever[1] -=0.02"""

# Vector of Formulations
alpha = 6 * np.pi / 180
MM_conf = dict(
    timeStep=conf.DT,
    gravity=conf.gravity,
    mu=conf.mu,
    coneBox=conf.cone_box,
    minNforce=conf.minNforce,
    maxNforce=conf.maxNforce,
    comHeight=conf.normal_height,
    omega=conf.omega,
    footSize=conf.footSize,
    height=conf.height,
    dist=conf.dist,
    width=conf.width,
    flyHighSlope=conf.flyHighSlope,
    footMinimalDistance=conf.footMinimalDistance,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wVCoM=conf.wVCoM,
    wPCoM=conf.wPCoM,
    wWrenchCone=conf.wWrenchCone,
    wForceTask=0,
    wFootRot=conf.wFootRot,
    wCoP=conf.wCoP,
    wFlyHigh=conf.wFlyHigh,
    wVelFoot=conf.wVelFoot,
    wColFeet=conf.wColFeet,
    wDCM=conf.wDCM,
    wBaseRot=conf.wBaseRot,
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
all_models = formuler.formulateHorizon(
    supports=cycles, experiment=Experiment.WWT_STAIRS
)
ter_model = formuler.formulateTerminalWWT(Support.DOUBLE, True)

# Horizon
H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, ter_model)

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

mpc = WBCHorizon()
mpc.initialize(
    wbc_conf,
    design,
    horizon,
    design.get_q0Complete(),
    design.get_v0Complete(),
    "actuationTask",
)
mpc.generateFullHorizon(formuler, Experiment.WWT_STAIRS)
print("mpc generated")

# ### SIMULATION LOOP ###
moyenne = 0

comRef = mpc.designer.get_com_position().copy()

starting_position_right = design.get_RF_frame().copy()
starting_position_left = design.get_LF_frame().copy()
starting_position_right.translation[2] += conf.height - 0.02
starting_position_right.translation[0] += 0.25
starting_position_left.translation[2] += conf.height - 0.02
starting_position_left.translation[0] += 0.25
LF_ref = [starting_position_left for i in range(horizon.size())]
RF_ref = [starting_position_right for i in range(horizon.size())]

for i in range(len(mpc.ref_LF_poses)):
    mpc.ref_LF_poses[i] = starting_position_left
    mpc.ref_RF_poses[i] = starting_position_right

comRef[0] += 0.3
comRef[1] += 0
comRef[2] += 0.0
baseRotation = design.get_root_frame().rotation @ yawRotation(np.pi / 6)
ref_com_vel = np.array([0.0, 0.0, 0])
mpc.ref_com = comRef
# mpc.ref_base_rot = baseRotation

T_total = conf.total_steps * conf.Tstep + 5 * conf.T

v1 = [0, -1, 0]
v2 = [0, 0, 1]

q1 = axisangle_to_q(v1, alpha)
q2 = axisangle_to_q(v2, np.pi / 2)
qtot = q_mult(q1, q2)


if conf.simulator == "bullet":
    device = BulletTalos(conf, design.get_rModelComplete())
    device.initializeJoints(design.get_q0Complete().copy())
    device.showTargetToTrack(starting_position_left, starting_position_right)
    # device.addStairs(DEFAULT_SAVE_DIR, [0.5,-0.75,-0.03], qtot)
    device.addStairs(DEFAULT_SAVE_DIR, [0.6, -0.8, 0.0], q2)
    q_current, v_current = device.measureState()

elif conf.simulator == "pinocchio":
    design.rmodelComplete = design.get_rModelComplete()
    design.rmodelComplete.q0 = design.get_q0Complete()
    design.rmodelComplete.v0 = design.get_v0Complete()

    device = VirtualPhysics(conf, view=True, block_joints=conf.blocked_joints)
    device.initialize(design.rmodelComplete)
    init_state = device.measure_state(device.Cq0, device.Cv0, device.Cv0 * 0)
    q_current = init_state["q"]
    v_current = init_state["dq"]

for s in range(T_total * conf.Nc):
    #    time.sleep(0.001)
    if s // conf.Nc == conf.TdoubleSupport + conf.T:
        mpc.ref_com_vel = ref_com_vel
    if mpc.timeToSolveDDP(s):

        land_LF = mpc.land_LF()[0] if mpc.land_LF() else (-1)
        land_RF = mpc.land_RF()[0] if mpc.land_RF() else (-1)
        takeoff_LF = mpc.takeoff_LF()[0] if mpc.takeoff_LF() else (-1)
        takeoff_RF = mpc.takeoff_RF()[0] if mpc.takeoff_RF() else (-1)
        print(
            "takeoff_RF = " + str(takeoff_RF) + ", landing_RF = ",
            str(land_RF) + ", takeoff_LF = " + str(takeoff_LF) + ", landing_LF = ",
            str(land_LF),
        )

    start = time.time()
    mpc.iterateNoThinking(s, q_current, v_current)
    end = time.time()
    """if end-start > 0.01:
		print(end-start)
		moyenne += end - start"""
    torques = horizon.currentTorques(mpc.x0)
    """if (s == 100 * 10):
		for t in range(conf.T):
			print("t = " + str(t))
			time.sleep(0.1)
			device.resetState(mpc.horizon.ddp.xs[t])
		exit()"""
    if conf.simulator == "bullet":
        device.execute(torques)
        q_current, v_current = device.measureState()
        device.moveMarkers(starting_position_left, starting_position_right)

    elif conf.simulator == "pinocchio":

        correct_contacts = mpc.horizon.get_contacts(0)
        command = {"tau": torques}
        real_state, _ = device.execute(command, correct_contacts, s)

        ######## Generating the forces ########## TODO: cast it in functions.

        LW = mpc.horizon.pinData(0).f[2].linear
        RW = mpc.horizon.pinData(0).f[8].linear
        TW = mpc.horizon.pinData(0).f[1].linear

        if not all(correct_contacts.values()):
            Lforce = TW - LW if correct_contacts["leg_left_sole_fix_joint"] else -LW
            Rforce = TW - RW if correct_contacts["leg_right_sole_fix_joint"] else -RW
        else:
            Lforce = TW / 2 - LW
            Rforce = TW / 2 - RW

        if conf.model_name == "talos_flex":
            qc, dqc = flex.correctEstimatedDeflections(
                torques, real_state["q"][7:], real_state["dq"][6:], Lforce, Rforce
            )

            q_current = np.hstack([real_state["q"][:7], qc])
            v_current = np.hstack([real_state["dq"][:6], dqc])

        elif conf.model_name == "talos":

            q_current = real_state["q"]
            v_current = real_state["dq"]

            # if s == 0:
            # stop

if conf.simulator == "bullet":
    device.close()
