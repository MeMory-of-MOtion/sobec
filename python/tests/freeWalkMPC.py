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
from sobec import RobotDesigner, WBC, HorizonManager, ModelMakerNoThinking, Flex, Support, FootTrajectory
import ndcurves
import numpy as np
import time
import ndcurves

def yawRotation(yaw):
    Ro = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )
    return Ro

def defineBezier(height, time_init,time_final,placement_init,placement_final):
	wps = np.zeros([3, 9])
	for i in range(4):  # init position. init vel,acc and jerk == 0
		wps[:, i] = placement_init.translation
	# compute mid point (average and offset along z)
	wps[:, 4] = (placement_init.translation + placement_final.translation) / 2.
	wps[2, 4] += height
	for i in range(5, 9):  # final position. final vel,acc and jerk == 0
		wps[:, i] = placement_final.translation
	translation = ndcurves.bezier(wps, time_init, time_final)
	pBezier = ndcurves.piecewise_SE3(ndcurves.SE3Curve(translation, placement_init.rotation, placement_final.rotation))
	return pBezier

def foot_trajectory(T, time_to_land, initial_pose,final_pose, trajectory_swing):
	"""Functions to generate steps."""
	tmax = conf.TsingleSupport
	landing_advance = 0
	takeoff_delay = 0
	placement = []
	for t in range(time_to_land - landing_advance, time_to_land - landing_advance - T, -1):
		if (t <= 0):
			placement.append(final_pose)
		elif (t > conf.TsingleSupport - landing_advance - takeoff_delay):
			placement.append(initial_pose)
		else:
			swing_pose = initial_pose.copy()
			swing_pose.translation = trajectory_swing.compute(float(conf.TsingleSupport - t) * conf.DT).translation
			placement.append(swing_pose)

	return placement

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
'''
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
design.get_rModel().inertias[12].lever[1] -=0.02'''

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
    wCoP = conf.wCoP,
    wFlyHigh = conf.wFlyHigh,
    wVelFoot = conf.wVelFoot,
    wColFeet = conf.wColFeet,
    wDCM = conf.wDCM,
    wBaseRot = conf.wBaseRot,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    forceWeights=conf.forceWeights,
    lowKinematicLimits=conf.lowKinematicLimits,
    highKinematicLimits=conf.highKinematicLimits,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMakerNoThinking()
formuler.initialize(MM_conf, design)

all_models = formuler.formulateHorizon(length=conf.T)
ter_model = formuler.formulateTerminalStepTracker(Support.DOUBLE)

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
        flexToJoint=conf.flexToJoint,
        dt=conf.simu_period,
        MA_duration=0.01,
        left_hip_indices=np.array([0, 1, 2]),
        right_hip_indices=np.array([6, 7, 8]),
        filtered=True,
    )
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
mpc.generateWalkingCycleNoThinking(formuler)
mpc.generateStandingCycleNoThinking(formuler)
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

starting_position_right = mpc.designer.get_RF_frame().copy()
final_position_right = mpc.designer.get_RF_frame().copy()
final_position_right.translation[2] = final_position_right.translation[2] - conf.footPenetration 

xForward = conf.xForward

starting_position_left = mpc.designer.get_LF_frame().copy()
final_position_left = mpc.designer.get_LF_frame().copy()
final_position_left.translation[2] = final_position_left.translation[2] - conf.footPenetration 

swing_trajectory_right = FootTrajectory(conf.swingApex,0.0,0)
swing_trajectory_left = FootTrajectory(conf.swingApex,0.0,0)
swing_trajectory_right.generate(0,conf.TsingleSupport * conf.DT,starting_position_right,final_position_right, False)
swing_trajectory_left.generate(0,conf.TsingleSupport * conf.DT,starting_position_left,final_position_left, False)

comRef[0] += 1
comRef[1] += 0
baseRotation = design.get_root_frame().rotation @ yawRotation(np.pi / 6)
ref_com_vel = np.array([0.,0.,0])
mpc.ref_com = comRef
#mpc.ref_base_rot = baseRotation

T_total = conf.total_steps * conf.Tstep + 5 * conf.T

for s in range(T_total * conf.Nc):
	#    time.sleep(0.001)
	if (s // conf.Nc == conf.TdoubleSupport + conf.T):
		mpc.ref_com_vel = ref_com_vel 
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
		
		'''LF_refs = foot_trajectory(
			len(mpc.ref_LF_poses),
			land_LF,
			starting_position_left,
			final_position_left,
			swing_trajectory_left
		)
		RF_refs = foot_trajectory(
			len(mpc.ref_RF_poses),
			land_RF,
			starting_position_right,
			final_position_right,
			swing_trajectory_right
		)

		for i in range(len(mpc.ref_LF_poses)):
			mpc.ref_LF_poses[i] = LF_refs[i]
			mpc.ref_RF_poses[i] = RF_refs[i]'''
		
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
				mpc.walkingCycle.setWrenchReference(0,"wrench_LF",wrench_reference_2contact_left)
				mpc.walkingCycle.setWrenchReference(0,"wrench_RF",wrench_reference_2contact_right)
			else:
				TdoubleSupport = 1
		else:
			TdoubleSupport = 1

		#print_trajectory(mpc.ref_LF_poses)

	
	start = time.time()
	mpc.iterateNoThinking(s, q_current, v_current)
	end = time.time()
	if end-start > 0.01:
		print(end-start)
		moyenne += end - start
	torques = horizon.currentTorques(mpc.x0)
	'''if (s == 100 * 10):
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
