#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
# import matplotlib.pyplot as plt

from bullet_Talos import BulletTalos

import example_robot_data
from sobec import (
    RobotDesigner,
    WBCHorizon,
    HorizonManager,
    ModelMaker,
    Flex,
    Support,
    Experiment,
)

from utils import yawRotation
import numpy as np
import time

DEFAULT_SAVE_DIR = "/local/src/sobec/python/tests"


class conf:
    # PATHS

    URDF_FILENAME = "talos_reduced.urdf"
    SRDF_FILENAME = "talos.srdf"
    SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
    modelPath = example_robot_data.getModelPath(URDF_SUBPATH)

    # Joint settings

    blocked_joints = [
        "universe",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_left_5_joint",
        "arm_left_6_joint",
        "arm_left_7_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "gripper_left_joint",
        "gripper_right_joint",
        "head_1_joint",
        "head_2_joint",
    ]

    # #### TIMING #####
    total_steps = 10
    DT = 1e-2  # Time step of the DDP
    T = 100  # Time horizon of the DDP (number of nodes)
    TdoubleSupport = 10  # Double support time  # TODO: (check with 20)
    TsingleSupport = 60  # Single support time
    simu_step = simu_period = 1e-3  #

    Nc = int(
        round(DT / simu_step)
    )  # Number of control knots per planification timestep

    # TODO: landing_advance and takeoff_delay are missing
    ddpIteration = 1  # Number of DDP iterations

    Tstep = TsingleSupport + TdoubleSupport

    # #### PHYSICS #####

    simulator = (
        "bullet"
        # "pinocchio"
    )

    gravity = np.array([0, 0, -9.81])

    mu = 0.3
    footSize = 0.05
    cone_box = np.array([0.1, 0.05])
    minNforce = 150
    maxNforce = 1200  # This may be still too low

    planned_push = [[(0, 10000 * simu_period)], [np.zeros(6)], ["base_link"]]

    model_name = "talos"  #

    # Flexibility Parameters
    compensate_deflections = True
    exact_deflection = False

    if model_name == "talos_flex":

        H_stiff = [
            2200,
            2200,
            5000,
            5000,
        ]  # [12000, 12000, 12000, 12000]#[LH_pitch, LH_roll, RH_pitch, RH_roll]
        H_damp = 2 * np.sqrt(H_stiff)

        # Number of times that the flexibility is computed in each control period
        flex_ratio = round(4.5 + 8.5e-4 * (max(H_stiff) ** 2) / 10000)

    elif model_name == "talos":
        H_stiff = [
            np.inf,
            np.inf,
            np.inf,
            np.inf,
        ]  # [LH_pitch, LH_roll, RH_pitch, RH_roll]
        H_damp = [np.inf, np.inf, np.inf, np.inf]
        flex_ratio = 1

    flex_esti_delay = 0.0  # [s]
    flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)

    flex_esti_delay = 0.0  # [s]
    flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)

    flexToJoint = np.array([0, 0, 0.09])

    # ###### WALKING GEOMETRY #########
    footMinimalDistance = 0.2
    flyHighSlope = 300

    normal_height = 0.87
    omega = np.sqrt(-gravity[2] / normal_height)

    # ##### CROCO - CONFIGURATION ########
    # relevant frame names

    rightFoot = rf_frame_name = "leg_right_sole_fix_joint"
    leftFoot = lf_frame_name = "leg_left_sole_fix_joint"

    # Weights for all costs

    wFootPlacement = 10000
    wStateReg = 100
    wControlReg = 0.001
    wLimit = 1e3
    wTauLimit = 0
    wVCoM = 0
    wPCoM = 1000
    wWrenchCone = 0.005
    wFootRot = 10000
    wCoP = 20
    wFlyHigh = 50000
    wVelFoot = 1000
    wColFeet = 3000
    wDCM = 0
    wBaseRot = 200

    weightBasePos = [0, 0, 0, 1000, 1000, 0]  # [x, y, z| x, y, z]
    weightBaseVel = [0, 0, 10, 100, 100, 10]  # [x, y, z| x, y, z]
    weightLegPos = [0.1, 0.1, 0.1, 0.01, 0.1, 1]  # [z, x, y, y, y, x]
    weightLegVel = [10, 10, 1, 0.1, 1, 10]  # [z, x, y, y, y, x]
    weightArmPos = [10, 10, 10, 10]  # [z, x, z, y, z, x, y]
    weightArmVel = [100, 100, 100, 100]  # [z, x, z, y, z, x, y]
    weightTorsoPos = [5, 5]  # [z, y]
    weightTorsoVel = [5, 5]  # [z, y]
    stateWeights = np.array(
        weightBasePos
        + weightLegPos * 2
        + weightTorsoPos
        # + weightArmPos * 2
        + weightBaseVel
        + weightLegVel * 2
        + weightTorsoVel
        # + weightArmVel * 2
    )

    weightuBase = "not actuated"
    weightuLeg = [1, 1, 1, 1, 1, 1]
    weightuArm = [10, 10, 10, 10]
    weightuTorso = [1, 1]
    controlWeight = np.array(
        weightuLeg * 2
        + weightuTorso
        # + weightuArm * 2
    )

    forceWeights = np.array([1, 1, 1, 10, 10, 10])
    lowKinematicLimits = np.array(
        [
            -0.35,
            -0.52,
            -2.10,
            0.0,
            -1.31,
            -0.52,  # left leg
            -1.57,
            -0.52,
            -2.10,
            0.0,
            -1.31,
            -0.52,  # right leg
            -1.3,
            -0.1,
        ]
    )  # torso
    highKinematicLimits = np.array(
        [
            1.57,
            0.52,
            0.7,
            2.62,
            0.77,
            0.52,  # left leg
            0.35,
            0.52,
            0.7,
            2.62,
            0.77,
            0.52,  # right leg
            1.3,
            0.2,
        ]
    )  # torso
    
    torqueLimits = np.array(
		[
			100,
			160,
			160,
			300,
			160,
			100,
			100,
			160,
			160,
			300,
			160,
			100,
			78,
			78,
		]
	)

    th_stop = 1e-6  # threshold for stopping criterion
    th_grad = 1e-9  # threshold for zero gradient.


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
    height=0,
    dist=1,
    width=1,
    flyHighSlope=conf.flyHighSlope,
    footMinimalDistance=conf.footMinimalDistance,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wTauLimit=conf.wTauLimit,
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
    torqueLimits=conf.torqueLimits,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMaker()
formuler.initialize(MM_conf, design)
cycles = [Support.DOUBLE for i in range(conf.T)]
all_models = formuler.formulateHorizon(supports=cycles, experiment=Experiment.WWT)
ter_model = formuler.formulateTerminalWWT(Support.DOUBLE, False)

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
mpc.generateFullHorizon(formuler, Experiment.WWT)
print("mpc generated")

""" ustar = mpc.horizon.ddp.us[0]
Kstar = mpc.horizon.ddp.K[0]
xs0 = mpc.horizon.ddp.xs
us0 = mpc.horizon.ddp.us
x0 = design.get_x0()
ndx = design.get_rModel().nv * 2
h = 0.001
Kdiff = np.zeros((design.get_rModel().nv - 6, ndx))

for i in range(ndx):
	hvec = np.zeros(ndx)
	hvec[i] = h
	xh =  mpc.horizon.ddp.problem.runningModels[0].state.integrate(x0,hvec)
	mpc.horizon.ddp.problem.x0 = xh
	xs0[0] = xh
	mpc.horizon.ddp.solve(xs0,us0,100,False)
	Kdiff[:,i] = (ustar - mpc.horizon.ddp.us[0])/h
	

print("Diff between K0 and Kdiff = ", np.linalg.norm(Kstar-Kdiff)/np.linalg.norm(Kstar))
exit() """

# ### SIMULATION LOOP ###
moyenne = 0

comRef = mpc.designer.get_com_position().copy()

comRef[0] += 1
comRef[1] += 0
comRef[2] += 0.0
baseRotation = design.get_root_frame().rotation @ yawRotation(np.pi / 6)
ref_com_vel = np.array([0.0, 0.0, 0])
mpc.ref_com = comRef
# mpc.ref_base_rot = baseRotation

T_total = conf.total_steps * conf.Tstep + 5 * conf.T


if conf.simulator == "bullet":
    device = BulletTalos(conf, design.get_rModelComplete())
    device.initializeJoints(design.get_q0Complete().copy())
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
        # print(
        # "takeoff_RF = " + str(takeoff_RF) + ", landing_RF = ",
        # str(land_RF) + ", takeoff_LF = " + str(takeoff_LF) + ", landing_LF = ",
        # str(land_LF),
        # )

    start = time.time()
    mpc.iterateNoThinking(s, q_current, v_current)
    end = time.time()

    torques = horizon.currentTorques(mpc.x0)
    if conf.simulator == "bullet":
        device.execute(torques)
        q_current, v_current = device.measureState()

    elif conf.simulator == "pinocchio":

        correct_contacts = mpc.horizon.get_contacts(0)
        command = {"tau": torques}
        real_state, _ = device.execute(command, correct_contacts, s)

        # ####### Generating the forces ########## TODO: cast it in functions.

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
