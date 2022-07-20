#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
import matplotlib.pyplot as plt

import configuration as conf

from bullet_Talos import BulletTalos
#from cricket.virtual_talos import VirtualPhysics

# from pyRobotWrapper import PinTalos
# from pyMPC import CrocoWBC
# from pyModelMaker import modeller
import pinocchio as pin
from sobec import RobotDesigner, WBC, HorizonManager, ModelMaker, Flex, Support
import ndcurves
import numpy as np
import time


# from time import time
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
'''
def foot_trajectory(T, time_to_land, initial_pose,final_pose):
	"""Functions to generate steps."""
	tmax = conf.TsingleSupport
	takeoff_delay = 0
	pose_trans = []
	for t in range(time_to_land, time_to_land - T, -1):
		trans = np.array([0,initial_pose.translation[1],0])
		if t < 0:
			trans[0] = final_pose.translation[0]
			trans[2] = 0
		elif t > tmax - takeoff_delay:
			trans[0] = initial_pose.translation[0]
			trans[2] = 0
		else:
			trans[2] = (np.sin(t * np.pi / (tmax - takeoff_delay)))* 0.05
			trans[0] = initial_pose.translation[0] * (time_to_land - t) / float(T) + final_pose.translation[0] * (t - time_to_land + T) / float(T)
		pose_trans.append(trans)
	return pose_trans
'''
def foot_trajectory(T, time_to_land, time_to_takeoff, initial_pose, final_pose):
	"""Functions to generate steps."""
	tmax = conf.TsingleSupport
	takeoff_delay = 0
	pose_trans = []
	if time_to_land < time_to_takeoff:
		for t in range(0,T):
			trans = np.array([0,initial_pose.translation[1],0])
			if t < time_to_land:
				trans[0] = final_pose.translation[0] * (conf.TsingleSupport - time_to_land + t) / float(conf.TsingleSupport) + \
				    initial_pose.translation[0] * (time_to_land - t) / float(conf.TsingleSupport)
				trans[2] = np.sin((conf.TsingleSupport - time_to_land + t)* np.pi / float(conf.TsingleSupport))* 0.05
			else:
				trans[0] = final_pose.translation[0]
				trans[2] = 0
			pose_trans.append(trans)
	else:
		for t in range(0,T):
			trans = np.array([0,initial_pose.translation[1],0])
			if t < time_to_takeoff:
				trans[0] = initial_pose.translation[0]
				trans[2] = 0
			elif t < time_to_land:
				trans[0] = final_pose.translation[0] * (t - time_to_takeoff) / float(conf.TsingleSupport) + \
				    initial_pose.translation[0] * (time_to_land - t) / float(conf.TsingleSupport)
				trans[2] = np.sin((t - time_to_takeoff)* np.pi / float(conf.TsingleSupport))* 0.05
			else:
				trans[0] = final_pose.translation[0]
				trans[2] = 0
			pose_trans.append(trans)
	return pose_trans

def foot_trajectory2(T, time_to_land, initial_pose,final_pose, trajectory_swing):
	"""Functions to generate steps."""
	tmax = conf.TsingleSupport
	landing_advance = 0
	takeoff_delay = 0
	placement = []
	for t in range(time_to_land - landing_advance, time_to_land - landing_advance - T, -1):
		if (t < 0):
			placement.append(final_pose)
		elif (t > conf.TsingleSupport - landing_advance - takeoff_delay):
			placement.append(initial_pose)
		else:
			swing_pose = initial_pose.copy()
			swing_pose.translation[0] = float(trajectory_swing.translation(float(conf.TsingleSupport - t) / float(conf.TsingleSupport))[0])
			swing_pose.translation[1] = float(trajectory_swing.translation(float(conf.TsingleSupport - t) / float(conf.TsingleSupport))[1])
			swing_pose.translation[2] = float(trajectory_swing.translation(float(conf.TsingleSupport - t) / float(conf.TsingleSupport))[2])
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
    wCoP = conf.wCoP,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMaker()
formuler.initialize(MM_conf, design)
all_models = formuler.formulateHorizon(lenght=conf.T)
ter_model = formuler.formulateStepTracker(Support.DOUBLE)

# Horizon

H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, ter_model)

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

mpc = WBC()
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

starting_position_right = mpc.designer.get_RF_frame().copy()
final_position_right = mpc.designer.get_LF_frame().copy()
final_position_right.translation[0] = final_position_right.translation[0] + conf.xForward
final_position_right.translation[1] = final_position_right.translation[1] - conf.footSeparation
#trajectory_right = foot_trajectory2(mpc.designer.get_RF_frame().translation,final_pose_right)

xForward = conf.xForward

starting_position_left = mpc.designer.get_LF_frame().copy()
final_position_left = mpc.designer.get_RF_frame().copy()
final_position_left.translation[0] = final_position_left.translation[0] + 2 * conf.xForward
final_position_left.translation[1] = final_position_left.translation[1] + conf.footSeparation
#trajectory_left = foot_trajectory2(mpc.designer.get_LF_frame().translation,final_pose_left)

swing_trajectory_right = defineBezier(conf.swingApex,0,1,starting_position_right,final_position_right)
swing_trajectory_left = defineBezier(conf.swingApex,0,1,starting_position_left,final_position_left)

ref_pose_right = [swing_trajectory_right for i in range(conf.T)]
ref_pose_left = [swing_trajectory_left for i in range(conf.T)]

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
'''
tt = np.linspace(0,conf.T,conf.T+1)
xx = []
zz = [] 
for i in range(conf.T+1):
	xx.append(swing_trajectory_left.translation(i / float(conf.T))[0])
	zz.append(swing_trajectory_left.translation(i / float(conf.T))[2])

plt.plot(tt,xx,label="x")
plt.plot(tt,zz,label="z")
plt.grid()
plt.legend(loc="upper left")
plt.show()

exit()'''
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
		
		if (takeoff_RF < conf.TdoubleSupport):
			#print("Update right trajectory")
			final_position_right = mpc.designer.get_LF_frame().copy()
			final_position_right.translation[0] += conf.xForward
			final_position_right.translation[1] -= conf.footSeparation
			starting_position_right = mpc.designer.get_RF_frame().copy()
			
			starting_position_left = mpc.designer.get_LF_frame().copy()
			final_position_left = final_position_right.copy()
			final_position_left.translation[0] += conf.xForward
			final_position_left.translation[1] += conf.footSeparation
			swing_trajectory_right = defineBezier(conf.swingApex,0,1,starting_position_right,final_position_right)
			swing_trajectory_left = defineBezier(conf.swingApex,0,1,starting_position_left,final_position_left)
		if (takeoff_LF < conf.TdoubleSupport):
			#print("Update left trajectory")
			final_position_left = mpc.designer.get_RF_frame().copy()
			final_position_left.translation[0] += conf.xForward
			final_position_left.translation[1] += conf.footSeparation
			starting_position_left = mpc.designer.get_LF_frame().copy()
			
			starting_position_right = mpc.designer.get_RF_frame().copy()
			final_position_right = final_position_left.copy()
			final_position_right.translation[0] += conf.xForward
			final_position_right.translation[1] -= conf.footSeparation
			swing_trajectory_left = defineBezier(conf.swingApex,0,1,starting_position_left,final_position_left)
			swing_trajectory_right = defineBezier(conf.swingApex,0,1,starting_position_right,final_position_right)
		
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
		'''
		for i in range(conf.T):
			if (takeoff_RF < land_RF):
				if (i < takeoff_RF):
					mpc.ref_RF_poses[i].translation = starting_position_right.translation
				elif (i < land_RF):
					mpc.ref_RF_poses[i].translation[0] = float(swing_trajectory_right.translation((float(i - takeoff_RF) / float(conf.TsingleSupport)))[0])
					mpc.ref_RF_poses[i].translation[1] = float(swing_trajectory_right.translation((float(i - takeoff_RF) / float(conf.TsingleSupport)))[1])
					mpc.ref_RF_poses[i].translation[2] = float(swing_trajectory_right.translation((float(i - takeoff_RF) / float(conf.TsingleSupport)))[2])
				else:
					mpc.ref_RF_poses[i].translation = final_position_right.translation
			else:
				if (i < land_RF):
					mpc.ref_RF_poses[i].translation[0] = float(swing_trajectory_right.translation((float(conf.TsingleSupport - land_RF + i) / float(conf.TsingleSupport)))[0])
					mpc.ref_RF_poses[i].translation[1] = float(swing_trajectory_right.translation((float(conf.TsingleSupport - land_RF + i) / float(conf.TsingleSupport)))[1])
					mpc.ref_RF_poses[i].translation[2] = float(swing_trajectory_right.translation((float(conf.TsingleSupport - land_RF + i) / float(conf.TsingleSupport)))[2])
				else:
					mpc.ref_RF_poses[i].translation = final_position_right.translation
			if (takeoff_LF < land_LF):
				if (i < takeoff_LF):
					mpc.ref_LF_poses[i].translation = starting_position_left.translation
				elif (i < land_LF):
					mpc.ref_LF_poses[i].translation[0] = float(swing_trajectory_left.translation((float(i - takeoff_LF) / float(conf.TsingleSupport)))[0])
					mpc.ref_LF_poses[i].translation[1] = float(swing_trajectory_left.translation((float(i - takeoff_LF) / float(conf.TsingleSupport)))[1])
					mpc.ref_LF_poses[i].translation[2] = float(swing_trajectory_left.translation((float(i - takeoff_LF) / float(conf.TsingleSupport)))[2])
				else:
					mpc.ref_LF_poses[i].translation = final_position_left.translation
			else:
				if (i < land_LF):
					mpc.ref_LF_poses[i].translation[0] = float(swing_trajectory_left.translation((float(conf.TsingleSupport - land_LF + i) / float(conf.TsingleSupport)))[0])
					mpc.ref_LF_poses[i].translation[1] = float(swing_trajectory_left.translation((float(conf.TsingleSupport - land_LF + i) / float(conf.TsingleSupport)))[1])
					mpc.ref_LF_poses[i].translation[2] = float(swing_trajectory_left.translation((float(conf.TsingleSupport - land_LF + i) / float(conf.TsingleSupport)))[2])
				else:
					mpc.ref_LF_poses[i].translation = final_position_left.translation

		'''
		LF_refs = foot_trajectory2(
			len(mpc.ref_LF_poses),
			land_LF,
			starting_position_left,
			final_position_left,
			swing_trajectory_left
		)
		RF_refs = foot_trajectory2(
			len(mpc.ref_RF_poses),
			land_RF,
			starting_position_right,
			final_position_right,
			swing_trajectory_right
		)
		
		'''
		LF_refs = foot_trajectory(
			len(mpc.ref_LF_poses),
			land_LF,
			takeoff_LF,
			starting_position_left,
			final_position_left)
		RF_refs = foot_trajectory(
			len(mpc.ref_RF_poses),
			land_RF,
			takeoff_RF,
			starting_position_right,
			final_position_right)'''
		for i in range(len(mpc.ref_LF_poses)):
			#mpc.ref_LF_poses[i] = pin.SE3(np.eye(3), LF_refs[i])
			#mpc.ref_RF_poses[i] = pin.SE3(np.eye(3), RF_refs[i])
			mpc.ref_LF_poses[i] = LF_refs[i]
			mpc.ref_RF_poses[i] = RF_refs[i]

		#print_trajectory(mpc.ref_LF_poses)

	start = time.time()
	mpc.iterate(s, q_current, v_current)
	end = time.time()
	if end-start > 0.01:
		print(end-start)
		moyenne += end - start
	torques = horizon.currentTorques(mpc.x0)
	'''if (s == 3223):
		for i in range(conf.T):
			time.sleep(0.1)
			print("i = " + str(i))
			device.resetState(mpc.horizon.ddp.xs[i])
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
print("moyenne = " + str(moyenne * 10 / float(s)))
if conf.simulator == "bullet":
    device.close()
