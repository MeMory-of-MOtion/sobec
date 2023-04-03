#!/usr/bin/env python
import time
import pinocchio as pin
import crocoddyl
import example_robot_data
from pinocchio.utils import rand,zero
from bullet_Talos import BulletTalos

import numpy as np
from sobec import (
    RobotDesigner,
    HorizonManager,
    WBCHand,
    ModelMaker
)


ROBOT_PATH= "/opt/openrobots/share/example-robot-data/robots/talos_data/robots/talos_reduced.urdf"


# =====================================================================
# Useful functions

class conf:
	# PATHS

	URDF_FILENAME = "talos_reduced.urdf"
	SRDF_FILENAME = "talos.srdf"
	SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
	URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
	modelPath = example_robot_data.getModelPath(URDF_SUBPATH)

	# Joint settings

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
	]
	blocked_joints=[
		"universe",
		"arm_left_5_joint",
		"arm_left_6_joint",
		"arm_left_7_joint",
		"arm_right_5_joint",
		"arm_right_6_joint",
		"arm_right_7_joint",
		"gripper_left_joint",
		"gripper_right_joint",
		"head_1_joint",
		"head_2_joint",
	]
    

	# #### TIMING #####
	DT = 1e-2  # Time step of the DDP
	T = 100  # Time horizon of the DDP (number of nodes)
	Tduration = 1000
	ddpIteration = 1
	simu_step = simu_period = 1e-3  #
	Nc = 10

	gravity = np.array([0, 0, -9.81])

	mu = 0.3
	footSize = 0.05
	cone_box = np.array([0.1, 0.05])
	minNforce = 200
	maxNforce = 1200  # This may be still too low

	# ###### WALKING GEOMETRY #########
	dist = 0.06 # collision threshold
	normal_height = 0.87
	omega = np.sqrt(-gravity[2] / normal_height)

	# ##### CROCO - CONFIGURATION ########
	# relevant frame names

	rightFoot = rf_frame_name = "leg_right_sole_fix_joint"
	leftFoot = lf_frame_name = "leg_left_sole_fix_joint"
	end_effector_name = "gripper_left_joint"


	# Weights for all costs

	wGripperPos = 10
	wGripperRot = 0
	wGripperVel = 10
	wStateReg = 0.1
	wControlReg = 0.0001
	wLimit = 1e3
	wVCoM = 0
	wPCoM = 100
	wWrenchCone = 0.05
	wFootRot = 0
	wCoP = 0
	wFlyHigh = 0
	wVelFoot = 0
	wColFeet = 10000
	wDCM = 0
	wBaseRot = 0

	weightBasePos = [10, 10, 10, 500, 500, 500]  # [x, y, z| x, y, z]
	weightBaseVel = [0, 0, 10, 100, 100, 10]  # [x, y, z| x, y, z]
	weightLegPos = [1000,1000,4500,4500,4500,1000]  # [z, x, y, y, y, x]
	weightLegVel = [10,10,10,10,10,10]  # [z, x, y, y, y, x]
	weightArmPosLeft = [1,1, 1,1]  # [z, x, z, y, z, x, y]
	weightArmPosRight = [10,10, 10,10]  # [z, x, z, y, z, x, y]
	weightArmVel = [10, 10, 10, 10]  # [z, x, z, y, z, x, y]
	weightTorsoPos = [10,10]  # [z, y]
	weightTorsoVel = [1,1]  # [z, y]
	stateWeights = np.array(
		weightBasePos
		+ weightLegPos * 2
		+ weightTorsoPos
		+ weightArmPosLeft + weightArmPosRight
		+ weightBaseVel
		+ weightLegVel * 2
		+ weightTorsoVel
		+ weightArmVel * 2
	)

	weightuLeg = [1, 1, 1, 1, 1, 1]
	weightuArm = [1, 1, 1, 1]
	weightuTorso = [1, 1]
	controlWeights = np.array(weightuLeg * 2 
							+ weightuTorso 
							+ weightuArm * 2
	)
	forceWeights = np.array([1,1,1,1,1,1])
	lowKinematicLimits = np.array([-0.35, -0.52,-2.10, 0.0,-1.31,-0.52, # left leg
							   -1.57,-0.52,-2.10,0.0,-1.31,-0.52, # right leg
							   -1.3,-0.1, # torso
							   -1.57,0.2,-2.44,-2.1, # left arm
							   -0.52,-2.88,-2.44,-2.1])  # right arm
	highKinematicLimits = np.array([1.57, 0.52, 0.7, 2.62, 0.77, 0.52, # left leg
							   0.35,0.52,0.7,2.62,0.77,0.52, # right leg
							   1.3,0.2,  # torso     
							   0.52,2.88,2.44,0, # left arm
							   1.57,-0.2,2.44,0]) # right arm              

	th_stop = 1e-6  # threshold for stopping criterion
	th_grad = 1e-9  # threshold for zero gradient.


#######Creating Complete Model#######
# ### RobotWrapper
design_conf = dict(
	urdfPath=conf.modelPath + conf.URDF_SUBPATH,
	srdfPath=conf.modelPath + conf.SRDF_SUBPATH,
	leftFootName=conf.lf_frame_name,
	rightFootName=conf.rf_frame_name,
	endEffectorName=conf.end_effector_name,
	robotDescription="",
	controlledJointsNames=conf.controlledJointsNames,
)
design = RobotDesigner()
design.initialize(design_conf)

target = np.array([0.6, 0.3, 1])

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
	footSize=conf.footSize,
	height=0,
	dist=conf.dist,
	width=1,
	flyHighSlope=0,
	footMinimalDistance=0.,
	wGripperPos=conf.wGripperPos,
	wGripperRot=0,
	wGripperVel=conf.wGripperVel,
	wFootPlacement=0,
	wStateReg=conf.wStateReg,
	wControlReg=conf.wControlReg,
	wLimit=conf.wLimit,
	wVCoM=0,
	wPCoM=conf.wPCoM,
	wWrenchCone=conf.wWrenchCone,
	wForceTask=0,
	wFootRot=0,
	wCoP=0,
	wFlyHigh=0,
	wVelFoot=0,
	wColFeet=conf.wColFeet,
	wDCM=0,
	wBaseRot=0,
	stateWeights=conf.stateWeights,
	controlWeights=conf.controlWeights,
	forceWeights=conf.forceWeights,
	lowKinematicLimits=conf.lowKinematicLimits,
	highKinematicLimits=conf.highKinematicLimits,
	th_grad=conf.th_grad,
	th_stop=conf.th_stop,
)

formuler = ModelMaker()
formuler.initialize(MM_conf, design)
	
run_models = [formuler.formulateColFullTask() for i in range(conf.T)]
ter_model = formuler.formulateTerminalColFullTask()

# Horizon
H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), run_models, ter_model)

wbc_conf = dict(
    T=conf.T,
    Tduration=conf.Tduration,
    ddpIteration=conf.ddpIteration,
    Dt=conf.DT,
    simu_step=conf.simu_period,
    Nc=conf.Nc
)

mpc = WBCHand()
mpc.initialize(
    wbc_conf,
    design,
    horizon,
    design.get_q0Complete(),
    design.get_v0Complete(),
    "actuationTask",
)
mpc.generateFullHorizon(formuler)
mpc.ref_hand_pose = target


ustar = mpc.horizon.ddp.us[0]
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
	

print("Diff between K0 and Kdiff = ", np.linalg.norm(Kstar-Kdiff))
exit()

poseTarget = design.get_EndEff_frame()
poseTarget.translation = target
device = BulletTalos(conf, design.get_rModelComplete())
device.initializeJoints(design.get_q0Complete())
device.showTargetToTrack(poseTarget,poseTarget)
q_current, v_current = device.measureState()

'''for i in range(horizon.size()):
	time.sleep(0.1)
	print("i = ", i)
	device.resetState(horizon.ddp.xs[i][:design.get_rModel().nq])
exit()'''
for s in range(10*(conf.Tduration + 500)):
	if mpc.timeToSolveDDP(s):
		print("iteration " + str(mpc.iteration()))
	mpc.iterate(s, q_current, v_current)
	torques = horizon.currentTorques(mpc.x0)
	device.execute(torques)
	q_current, v_current = device.measureState()

