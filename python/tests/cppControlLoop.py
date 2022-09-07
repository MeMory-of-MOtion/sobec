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
from sobec import RobotDesigner, WBC, HorizonManager, ModelMaker, Flex, Support, LocomotionType
import ndcurves
import numpy as np
import time

DEFAULT_SAVE_DIR = '/local/src/sobec/python/tests'

def save_trajectory(xss,uss,LF_pose,RF_pose,LF_force,RF_force,  save_name=None, save_dir=DEFAULT_SAVE_DIR):
	'''
	Saves data to a compressed npz file (binary)
	'''
	simu_data = {}
	simu_data['xss'] = xss
	simu_data['uss'] = uss
	simu_data['LF_pose'] = LF_pose
	simu_data['RF_pose'] = RF_pose
	simu_data['LF_force'] = LF_force
	simu_data['RF_force'] = RF_force
	print('Compressing & saving data...')
	if(save_name is None):
		save_name = 'sim_data_NO_NAME'+str(time.time())
	if(save_dir is None):
		save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),'../data'))
	save_path = save_dir+'/'+save_name+'.npz'
	np.savez_compressed(save_path, data=simu_data)
	print("Saved data to "+str(save_path)+" !")

def load_data(npz_file):
    '''
    Loads a npz archive of sim_data into a dict
    '''
    d = np.load(npz_file, allow_pickle=True, encoding='latin1')
    return d['data'][()]

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
    footSize = conf.footSize,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wWrenchCone=conf.wWrenchCone,
    wCoP = conf.wCoP,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    lowKinematicLimits=conf.lowKinematicLimits,
    highKinematicLimits=conf.highKinematicLimits,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMaker()
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
    Nc=conf.Nc
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
    "actuationTask"
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
steps = 0

starting_position_right = mpc.designer.get_RF_frame().copy()
final_position_right = mpc.designer.get_LF_frame().copy()
final_position_right.translation[0] = final_position_right.translation[0] + conf.xForward
final_position_right.translation[1] = final_position_right.translation[1] - conf.footSeparation + conf.sidestep
final_position_right.translation[2] = final_position_right.translation[2] - conf.footPenetration

xForward = conf.xForward

starting_position_left = mpc.designer.get_LF_frame().copy()
final_position_left = mpc.designer.get_RF_frame().copy()
final_position_left.translation[0] = final_position_left.translation[0] + 2 * conf.xForward
final_position_left.translation[1] = final_position_left.translation[1] + conf.footSeparation 
final_position_left.translation[2] = final_position_left.translation[2] - conf.footPenetration 

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

T_total = conf.total_steps * conf.Tstep + 5 * conf.T

### Save trajectory in npz file
xss = []
uss = []
LF_pose = []
RF_pose = []
LF_force = []
RF_force = []

for s in range(T_total * conf.Nc):
	#    time.sleep(0.001)
	if mpc.timeToSolveDDP(s):
		xss.append(mpc.horizon.ddp.xs[0])
		uss.append(mpc.horizon.ddp.us[0])
		LF_pose.append(mpc.designer.get_LF_frame().copy())
		RF_pose.append(mpc.designer.get_RF_frame().copy())
		LF_force.append(mpc.horizon.ddp.problem.runningDatas[0].differential.costs.costs["wrench_LF"].residual.contact.f)
		RF_force.append(mpc.horizon.ddp.problem.runningDatas[0].differential.costs.costs["wrench_RF"].residual.contact.f)

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
		
		if land_RF == 0:
			steps += 1
		if land_LF == 0:
			steps += 1
		if steps == conf.total_steps:
			xForward = 0
		if (s // conf.Nc == conf.Tstep * (conf.total_steps + 1)):
			# Switch to stand at the beginning of the last double support step in walking cycle
			# Given one more step to put feet together
			mpc.switchToStand()
			print("switch to stand")
		if (takeoff_RF < conf.TdoubleSupport):
			#print("Update right trajectory")
			final_position_right = mpc.designer.get_LF_frame().copy()
			final_position_right.translation[0] += xForward
			final_position_right.translation[1] -= conf.footSeparation + conf.sidestep
			final_position_right.translation[2] = final_position_right.translation[2] - conf.footPenetration 
			starting_position_right = mpc.designer.get_RF_frame().copy()
			
			starting_position_left = mpc.designer.get_LF_frame().copy()
			final_position_left = final_position_right.copy()
			final_position_left.translation[0] += xForward
			final_position_left.translation[1] += conf.footSeparation 
			swing_trajectory_right = defineBezier(conf.swingApex,0,1,starting_position_right,final_position_right)
			swing_trajectory_left = defineBezier(conf.swingApex,0,1,starting_position_left,final_position_left)
		if (takeoff_LF < conf.TdoubleSupport):
			#print("Update left trajectory")
			final_position_left = mpc.designer.get_RF_frame().copy()
			final_position_left.translation[0] += xForward
			final_position_left.translation[1] += conf.footSeparation 
			final_position_left.translation[2] = final_position_left.translation[2] - conf.footPenetration 
			starting_position_left = mpc.designer.get_LF_frame().copy()
			
			starting_position_right = mpc.designer.get_RF_frame().copy()
			final_position_right = final_position_left.copy()
			final_position_right.translation[0] += xForward
			final_position_right.translation[1] -= conf.footSeparation + conf.sidestep
			swing_trajectory_left = defineBezier(conf.swingApex,0,1,starting_position_left,final_position_left)
			swing_trajectory_right = defineBezier(conf.swingApex,0,1,starting_position_right,final_position_right)
		
		if (mpc.walkingCycle.contacts(0).getContactStatus("leg_left_sole_fix_joint")):
			if (mpc.walkingCycle.contacts(0).getContactStatus("leg_right_sole_fix_joint")):
				if (s // conf.Nc <= conf.TdoubleSupport):
					ref_force = normal_force_traj_first(float(TdoubleSupport)/float(conf.TdoubleSupport))[0]
				elif(s // conf.Nc < conf.Tstep * (conf.total_steps + 1)):
					ref_force = normal_force_traj(float(TdoubleSupport)/float(conf.TdoubleSupport))[0]
				elif (TdoubleSupport < conf.TdoubleSupport):
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
				#print("Change left force to " + str(wrench_reference_2contact_left[2]))
				#print("Change right force to " + str(wrench_reference_2contact_right[2]))
				mpc.walkingCycle.setForceReferenceLF(0,"wrench_LF",wrench_reference_2contact_left)
				mpc.walkingCycle.setForceReferenceRF(0,"wrench_RF",wrench_reference_2contact_right)
				mpc.standingCycle.setForceReferenceLF(0,"wrench_LF",wrench_reference_2contact_left)
				mpc.standingCycle.setForceReferenceRF(0,"wrench_RF",wrench_reference_2contact_right)
			else:
				TdoubleSupport = 1
		else:
			TdoubleSupport = 1
		LF_refs = foot_trajectory(
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
			mpc.ref_RF_poses[i] = RF_refs[i]

		#print_trajectory(mpc.ref_LF_poses)
	start = time.time()

	mpc.iterate(s,q_current, v_current)
	end = time.time()
	if end-start > 0.01:
		#print(end-start)
		moyenne += end - start
	torques = horizon.currentTorques(mpc.x0)
	
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

#save_trajectory(xss,uss,LF_pose,RF_pose,LF_force,RF_force, save_name="trajectories_xs_us")
print('Mean computation time')
print(moyenne / T_total)
if conf.simulator == "bullet":
    device.close()
