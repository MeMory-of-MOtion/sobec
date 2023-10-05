#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""

import configurationWalk as conf
import numpy as np
import matplotlib.pyplot as plt
from bullet_Talos import BulletTalos
from utils import axisangle_to_q, q_mult, yawRotation, extractYaw, defineBezier, foot_trajectory

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
import time
import os


# ####### CONFIGURATION  ############
# ### RobotWrapper
design_conf = dict(
    urdfPath=conf.modelPath + conf.URDF_SUBPATH,
    srdfPath=conf.modelPath + conf.SRDF_SUBPATH,
    leftFootName=conf.lf_frame_name,
    rightFootName=conf.rf_frame_name,
    rightHandName = "arm_right_7_link",
	leftHandName = "arm_left_7_link",
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
    wTauLimit=conf.wTauLimit,
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
    torqueLimits=conf.torqueLimits,
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

wbc_conf2 = dict(
    totalSteps=conf.total_steps,
    T=conf.T,
    TdoubleSupport=conf.TdoubleSupport,
    TsingleSupport=conf.TsingleSupport,
    Tstep=conf.Tstep,
    ddpIteration=20,
    Dt=conf.DT,
    simu_step=0.01,
    min_force=150,
    support_force=-design.getRobotMass() * conf.gravity[2],
    Nc=1,
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

mpc2 = WBCHorizon()
mpc2.initialize(
    wbc_conf2, design, horizon, qStartComplete, design.get_v0Complete(), "actuationTask"
)
mpc2.generateFullHorizon(formuler, Experiment.WALK)

device = BulletTalos(conf, design.get_rModelComplete())
device.initializeJoints(qStartComplete)
device.showTargetToTrack(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])
q_current, v_current = device.measureState()


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

device.changeCamera(2,30,-10,np.array([0.5,0.5,0.9]))

for s in range(T_total * conf.Nc):
    #    time.sleep(0.001)
    if mpc.timeToSolveDDP(s):
        xss.append(mpc.horizon.ddp.xs[0])
        uss.append(mpc.horizon.ddp.us[0])
        LF_pose.append(mpc.designer.get_LF_frame().copy())
        RF_pose.append(mpc.designer.get_RF_frame().copy())

        LF_force.append(
            mpc.horizon.ddp.problem.runningDatas[0]
            .differential.costs.costs["wrench_LF"]
            .residual.contact.f
        )
        RF_force.append(
            mpc.horizon.ddp.problem.runningDatas[0]
            .differential.costs.costs["wrench_RF"]
            .residual.contact.f
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
                conf.TsingleSupport,
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
                conf.TsingleSupport,
            )
            if land_RF > -1
            else ([starting_position_right for i in range(len(mpc.ref_RF_poses))])
        )

        for i in range(len(mpc.ref_LF_poses)):
            mpc.ref_LF_poses[i] = LF_refs[i]
            mpc.ref_RF_poses[i] = RF_refs[i]

        # print_trajectory(mpc.ref_LF_poses)

    mpc.iterate(s, q_current, v_current)
    #mpc2.iterate(s, q_current, v_current)
    torques = mpc.horizon.ddp.us[0] #mpc.horizon.currentTorques(mpc.x0)

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

#save_trajectory(
#xss, uss, LF_pose, RF_pose, LF_force, RF_force, save_name="walk_K0_xs_us"
#)
device.close()
