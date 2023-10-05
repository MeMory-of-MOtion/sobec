#!/usr/bin/env python
import example_robot_data
from bullet_Talos import BulletTalos

import numpy as np
from sobec import (
    RobotDesigner,
    HorizonManager,
    WBCHand,
    ModelMakerHand
)


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

	# ###### Reaching GEOMETRY #########
	obstacleRadius = 0.3
	obstaclePosition = np.array([0.6,0.3,1])
	obstacleHeight = 2
	normal_height = 0.87
	omega = np.sqrt(-gravity[2] / normal_height)

	# ##### CROCO - CONFIGURATION ########
	# relevant frame names

	rightFoot = rf_frame_name = "leg_right_sole_fix_joint"
	leftFoot = lf_frame_name = "leg_left_sole_fix_joint"
	end_effector_name = "gripper_left_joint"


	# Weights for all costs

	wHandTranslation = 50
	wHandRotation = 0
	wHandVelocity = 10
	wHandCollision = 0
	wStateReg = 0.1
	wControlReg = 0.001
	wLimit = 1e3
	wCoM = 100
	wWrenchCone = 0.05
	wForceHand = 0
	wFrictionHand = 0
	wDCM = 0


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
	torqueLimits = np.array([100, 160, 160, 300, 160, 100, 100, 160, 
						  160, 300, 160, 100, 78, 78,
						  100,100,100,100,
						  100,100,100,100])
	th_stop = 1e-6  # threshold for stopping criterion
	th_grad = 1e-9  # threshold for zero gradient.


#######Creating Complete Model#######
# ### RobotWrapper
design_conf = dict(
	urdfPath=conf.modelPath + conf.URDF_SUBPATH,
	srdfPath=conf.modelPath + conf.SRDF_SUBPATH,
	leftFootName=conf.lf_frame_name,
	rightFootName=conf.rf_frame_name,
	rightHandName = "arm_right_7_link",
	leftHandName = "arm_left_7_link",
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
	obstacleRadius=conf.obstacleRadius,
	obstaclePosition=conf.obstaclePosition,
	obstacleHeight=conf.obstacleHeight,
	wHandTranslation=conf.wHandTranslation,
	wHandRotation=conf.wHandRotation,
	wHandCollision=conf.wHandCollision,
	wHandVelocity=conf.wHandVelocity,
	wStateReg=conf.wStateReg,
	wControlReg=conf.wControlReg,
	wLimit=conf.wLimit,
	wForceHand=conf.wForceHand,
	wFrictionHand=conf.wFrictionHand,
	wWrenchCone=conf.wWrenchCone,
	wDCM=0,
	wCoM=conf.wCoM,
	stateWeights=conf.stateWeights,
	controlWeights=conf.controlWeights,
	lowKinematicLimits=conf.lowKinematicLimits,
	highKinematicLimits=conf.highKinematicLimits,
	th_grad=conf.th_grad,
	th_stop=conf.th_stop,
)

formuler = ModelMakerHand()
formuler.initialize(MM_conf, design)
	
run_models = [formuler.formulateColFullTask() for i in range(conf.T)]
ter_model = formuler.formulateTerminalColFullTask()

# Horizon
H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), run_models, ter_model)

poseTarget = np.array([0.7,0.3,1.2])
for i in range(horizon.size()):
	horizon.changeCostStatus(i, 'position_RH', False)
	horizon.changeCostStatus(i, 'position_LH', False)
	horizon.setTranslationReference(i,'position_LH',poseTarget)
horizon.changeTerminalCostStatus('position_LH', False)
horizon.setTerminalTranslationReference('position_LH', poseTarget)
xs_init = []
us_init = []
for i in range(conf.T):
	xs_init.append(design.get_x0())
	us_init.append(np.zeros(len(conf.controlWeights)))
xs_init.append(design.get_x0())

horizon.ddp.solve(xs_init, us_init, 500, False)


device = BulletTalos(conf, design.get_rModelComplete())
device.initializeJoints(design.get_q0Complete())
device.showHandToTrack(poseTarget)
q_current, v_current = device.measureState()
xMeasured = design.shapeState(q_current,v_current)

T_total = 1000
itrack = 0
for s in range(T_total * conf.Nc):
	#time.sleep(0.001)
	if s % 10 == 0:
		itrack += 1
		if itrack <= conf.T:
			horizon.changeCostStatus(conf.T - itrack, 'position_LH', True)
		horizon.solve(xMeasured,1,False)
	torques = horizon.currentTorques(xMeasured)
	# Send torque and recover robot state
	device.execute(torques)
	q_current, v_current = device.measureState()
	xMeasured = design.shapeState(q_current,v_current)

