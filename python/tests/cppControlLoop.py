#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""


import configuration as conf

from bullet_Talos import BulletTalos
from cricket.virtual_talos import VirtualPhysics

# from pyRobotWrapper import PinTalos
# from pyMPC import CrocoWBC
from pyModelMaker import modeller

from sobec import RobotDesigner, WBC, HorizonManager, ModelMaker

######## CONFIGURATION  ############
#### RobotWrapper
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

## Vector of Formulations
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

## Horizon

H_conf = dict(leftFootName = conf.lf_frame_name, 
              rightFootName = conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, all_models[-1])

## MPC
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

if conf.simulator == "bullet":
    device = BulletTalos(conf, design.get_rModelComplete())
    device.initializeJoints(design.get_q0Complete().copy())
    #    device.showTargetToTrack(mpc.LF_sample, mpc.RF_sample)
    q_current, v_current = device.measureState()

elif conf.simulator == "pinocchio":
    design.rmodelComplete = design.get_rModelComplete()
    design.rmodelComplete.q0 = design.get_q0Complete()
    design.rmodelComplete.v0 = design.get_v0Complete()

    device = VirtualPhysics(conf, view=True, block_joints=conf.blocked_joints)
    device.initialize(design.rmodelComplete)
    q_current, v_current = device.Cq0, device.Cv0

### SIMULATION LOOP ###

for s in range(conf.T_total * conf.Nc):
    #    time.sleep(0.001)
    torques = mpc.iterate(s, q_current, v_current)

    if conf.simulator == "bullet":
        device.execute(torques)
        q_current, v_current = device.measureState()
    #        device.moveMarkers(mpc.LF_sample,
    #                           mpc.RF_sample)

    elif conf.simulator == "pinocchio":

        correct_contacts = mpc.horizon.get_contacts(0)
        command = {"tau": torques}
        real_state, _ = device.execute(command, correct_contacts, s)
        esti_state = real_state  # wbc.joint_estimation(real_state, command)
        q_current, v_current = esti_state["q"], esti_state["dq"]

#    if s == 0:stop


if conf.simulator == "bullet":
    device.close()
