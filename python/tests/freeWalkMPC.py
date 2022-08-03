#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
import matplotlib.pyplot as plt

import configurationFree as conf

from bullet_Talos import BulletTalos
#from cricket.virtual_talos import VirtualPhysics

# from pyRobotWrapper import PinTalos
# from pyMPC import CrocoWBC
# from pyModelMaker import modeller
import pinocchio as pin
from sobec import RobotDesigner, WBCNoThinking, HorizonManager, ModelMakerNoThinking, Flex, Support
import ndcurves
import numpy as np
import time
import ndcurves


def defineBezier(height, time_init,time_final,translation_init,translation_final):
	wps = np.zeros([3, 9])
	for i in range(4):  # init position. init vel,acc and jerk == 0
		wps[:, i] = translation_init
	# compute mid point (average and offset along z)
	wps[:, 4] = (translation_init + translation_final) / 2.
	wps[2, 4] += height
	for i in range(5, 9):  # final position. final vel,acc and jerk == 0
		wps[:, i] = translation_final
	translation = ndcurves.bezier(wps, time_init, time_final)
	return translation

def foot_trajectory(T, time_to_land, translation_init,translation_final, trajectory_swing):
	"""Functions to generate steps."""
	tmax = conf.TsingleSupport
	landing_advance = 0
	takeoff_delay = 0
	translation = []
	for t in range(time_to_land - landing_advance, time_to_land - landing_advance - T, -1):
		if (t < 0):
			translation.append(translation_final)
		elif (t > conf.TsingleSupport - landing_advance - takeoff_delay):
			translation.append(translation_init)
		else:
			swing_pose = translation_init.copy()
			swing_pose[0] = trajectory_swing(float(conf.TsingleSupport - t) / float(conf.TsingleSupport))[0]
			swing_pose[1] = trajectory_swing(float(conf.TsingleSupport - t) / float(conf.TsingleSupport))[1]
			swing_pose[2] = trajectory_swing(float(conf.TsingleSupport - t) / float(conf.TsingleSupport))[2]
			translation.append(swing_pose)

	return translation

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
        #"arm_left_1_joint",
        #"arm_left_2_joint",
        #"arm_left_3_joint",
        #"arm_left_4_joint",
        #"arm_right_1_joint",
        #"arm_right_2_joint",
        #"arm_right_3_joint",
        #"arm_right_4_joint",
    ],
)
design = RobotDesigner()
design.initialize(design_conf)
print("Model ok")
design.addToeAndHeel(0.1,0.1)
print("Added heel")
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
    footSize = conf.footSize,
    flyHighSlope = conf.flyHighSlope,
    footMinimalDistance = conf.footMinimalDistance,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wVCoM=conf.wVCoM,
    wCoM=conf.wCoM,
    wWrenchCone=conf.wWrenchCone,
    wFootRot=conf.wFootRot,
    wGroundCol=conf.wGroundCol,
    wCoP = conf.wCoP,
    wFlyHigh = conf.wFlyHigh,
    wVelFoot = conf.wVelFoot,
    wColFeet = conf.wColFeet,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMakerNoThinking()
formuler.initialize(MM_conf, design)
all_models = formuler.formulateHorizon(length=conf.T)
ter_model = formuler.formulateNoThinkingTerminalTracker(Support.DOUBLE)
print("horizon formulated")
# Horizon

H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, ter_model)
print("horizon initialized")
# MPC
wbc_conf = dict(
    horizonSteps=conf.preview_steps,
    totalSteps=conf.total_steps,
    T=conf.T,
    TdoubleSupport=conf.TdoubleSupport,
    TsingleSupport=conf.TsingleSupport,
    Tstep=conf.Tstep,
    ddpIteration=conf.ddpIteration,
    Dt=conf.DT,
    simu_step=conf.simu_period,
    Nc=conf.Nc,
    typeOfCommand=conf.typeOfCommand,
)

# Flex
flex = Flex()
flex.initialize(
    dict(
        left_stiffness=np.array(conf.H_stiff[:2]),
        right_stiffness=np.array(conf.H_stiff[2:]),
        left_damping=np.array(conf.H_damp[:2]),
        right_damping=np.array(conf.H_damp[2:]),
        dt=conf.simu_period,
        MA_duration=0.01,
        left_hip_indices=np.array([0, 1, 2]),
        right_hip_indices=np.array([6, 7, 8]),
    )
)

mpc = WBCNoThinking()
mpc.initialize(
    wbc_conf,
    design,
    horizon,
    design.get_q0Complete(),
    design.get_v0Complete(),
    "actuationTask",
)
mpc.generateWalkingCycle(formuler)
mpc.generateStandingCycle(formuler)
print("mpc generated")
if conf.simulator == "bullet":
    device = BulletTalos(conf, design.get_rModelComplete())
    device.initializeJoints(design.get_q0Complete().copy())
    device.showTargetToTrack(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])
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

# ### SIMULATION LOOP ###
moyenne = 0

fz_ref_1contact = -design.getRobotMass() * conf.gravity[2];
fz_ref_2contact = fz_ref_1contact / 2.;

MIN_CONTACT_FORCE = 150
normal_force_traj_first = ndcurves.polynomial.MinimumJerk(np.array([fz_ref_2contact]), np.array([fz_ref_1contact - MIN_CONTACT_FORCE]))
normal_force_traj = ndcurves.polynomial.MinimumJerk(np.array([MIN_CONTACT_FORCE]), np.array([fz_ref_1contact - MIN_CONTACT_FORCE])) 
normal_force_traj_end = ndcurves.polynomial.MinimumJerk(np.array([MIN_CONTACT_FORCE]), np.array([fz_ref_2contact])) 
TdoubleSupport = 1;

wrench_reference_2contact_right = np.array([0,0,fz_ref_2contact,0,0,0])
wrench_reference_2contact_left = np.array([0,0,fz_ref_2contact,0,0,0])
wrench_reference_1contact = np.array([0,0,fz_ref_1contact,0,0,0])
ref_force = fz_ref_2contact
comRef = mpc.designer.get_com_position().copy()

comRef[0] += 0.5
comRef[1] += 0
mpc.ref_com = comRef
mpc.ref_com_vel = np.array([0.2,0,0])

for s in range(conf.T_total * conf.Nc):
	#    time.sleep(0.001)
	if mpc.timeToSolveDDP(s):

		land_LF = (
			mpc.land_LF()[0]
			if mpc.land_LF()
			else (
				mpc.land_LF_cycle() + mpc.horizon.size()
			)
		)
		land_RF = (
			mpc.land_RF()[0]
			if mpc.land_RF()
			else (
				mpc.land_RF_cycle() + mpc.horizon.size()
			)
		)
		takeoff_LF = (
			mpc.takeoff_LF()[0]
			if mpc.takeoff_LF()
			else (
				mpc.takeoff_LF_cycle() + mpc.horizon.size()
			)
		)
		takeoff_RF = (
			mpc.takeoff_RF()[0]
			if mpc.takeoff_RF()
			else (
				mpc.takeoff_RF_cycle() + mpc.horizon.size()
			)
		)
		print("takeoff_RF = " + str(takeoff_RF) + ", landing_RF = ", str(land_RF) + ", takeoff_LF = " + str(takeoff_LF) + ", landing_LF = ", str(land_LF))
		
		
		if (mpc.walkingCycle.contacts(0).getContactStatus("leg_left_sole_fix_joint")):
			if (mpc.walkingCycle.contacts(0).getContactStatus("leg_right_sole_fix_joint")):
				if (s / conf.Nc <= conf.TdoubleSupport):
					ref_force = normal_force_traj_first(float(TdoubleSupport)/float(conf.TdoubleSupport))[0]
				elif (s / conf.Nc < conf.total_steps * conf.Tstep):
					ref_force = normal_force_traj(float(TdoubleSupport)/float(conf.TdoubleSupport))[0]
				elif (TdoubleSupport <= conf.TdoubleSupport):
					ref_force = normal_force_traj_end(float(TdoubleSupport)/float(conf.TdoubleSupport))[0]
				
				TdoubleSupport += 1
				if (mpc.takeoff_RF_cycle() < mpc.takeoff_LF_cycle()):
					# Next foot to take off is right foot
					wrench_reference_2contact_right[2] = fz_ref_1contact - ref_force
					wrench_reference_2contact_left[2] = ref_force
				else:
					# Next foot to take off is left foot
					wrench_reference_2contact_left[2] = fz_ref_1contact - ref_force
					wrench_reference_2contact_right[2] = ref_force
				print("Change left force to " + str(wrench_reference_2contact_left[2]))
				print("Change right force to " + str(wrench_reference_2contact_right[2]))
				mpc.walkingCycle.setForceReferenceLF(0,"wrench_LF",wrench_reference_2contact_left)
				mpc.walkingCycle.setForceReferenceRF(0,"wrench_RF",wrench_reference_2contact_right)
			else:
				TdoubleSupport = 1
		else:
			TdoubleSupport = 1

		#print_trajectory(mpc.ref_LF_poses)

	start = time.time()
	mpc.iterate(s, q_current, v_current)
	end = time.time()
	if end-start > 0.01:
		print(end-start)
		moyenne += end - start
	torques = horizon.currentTorques(mpc.x0)
	'''if (s == 150 * 10):
		for t in range(conf.T):
			print("t = " + str(t))
			time.sleep(0.1)
			device.resetState(mpc.horizon.ddp.xs[t])
		exit()'''
	if conf.simulator == "bullet":
		device.execute(torques)
		q_current, v_current = device.measureState()
		device.moveMarkers(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])

	elif conf.simulator == "pinocchio":

		correct_contacts = mpc.horizon.get_contacts(0)
		command = {"tau": torques}
		real_state, _ = device.execute(command, correct_contacts, s)

		qc, dqc = flex.correctEstimatedDeflections(
			torques, real_state["q"][7:], real_state["dq"][6:]
		)

		q_current = np.hstack([real_state["q"][:7], qc])
		v_current = np.hstack([real_state["dq"][:6], dqc])

		# esti_state = flex.correctEstimatedDeflections(
		# torques, q_current[7:], v_current[6:]
		# )

		# q_current = np.hstack([q_current[:7], esti_state[0]])
		# v_current = np.hstack([v_current[:6], esti_state[1]])

	# if s == 0:
	# stop

if conf.simulator == "bullet":
    device.close()
