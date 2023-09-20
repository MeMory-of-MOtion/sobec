import matplotlib.pyplot as plt

from bullet_Talos import BulletTalos

import pinocchio as pin
import example_robot_data
from sobec import RobotDesigner, HorizonManager, ModelMakerHand, Flex, Phase, WBCHand
import numpy as np
import time


def m2a(m): return np.array(m.flat)

def a2m(a): return np.matrix(a).T

# coding: utf8

import numpy as np
import pybullet as pyb

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
		#"arm_left_1_joint",
		#"arm_left_2_joint",
		#"arm_left_3_joint",
		#"arm_left_4_joint",
		"arm_left_5_joint",
		"arm_left_6_joint",
		"arm_left_7_joint",
		#"arm_right_1_joint",
		#"arm_right_2_joint",
		#"arm_right_3_joint",
		#"arm_right_4_joint",
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
	T = 150  # Time horizon of the DDP (number of nodes)
	Tcontact = 300
	TtrackingToContact = 100
	simu_step = simu_period = 1e-3  #
	ddpIteration = 1

	Nc = int(round(DT / simu_step)) 

	gravity = np.array([0, 0, -9.81])

	mu = 0.3
	footSize = 0.05
	coneBox = np.array([0.1, 0.05])
	minNforce = 200
	maxNforce = 1200  # This may be still too low

	normal_height = 0.87
	omega = np.sqrt(-gravity[2] / normal_height)

	# ##### CROCO - CONFIGURATION ########
	# relevant frame names

	rightFootName = "leg_right_sole_fix_joint"
	leftFootName  = "leg_left_sole_fix_joint"
	rightHandName = "arm_right_7_link"
	leftHandName = "arm_left_7_link"

	# Weights for all costs

	wHandTranslation = 5
	wHandRotation = 0
	wHandVelocity = 0
	wStateReg = 0.1
	wControlReg = 0.0001
	wLimit = 1e3
	wWrenchCone = 0.05
	wForceHand = 0.1
	wCoM = 0
	wDCM = 0

	weightBasePos = [10, 10, 10, 500, 500, 500]  # [x, y, z| x, y, z]
	weightBaseVel = [0, 0, 10, 100, 100, 10]  # [x, y, z| x, y, z]
	weightLegPos = [1000,1000,4500,4500,4500,1000]  # [z, x, y, y, y, x]
	weightLegVel = [10,10,10,10,10,10]  # [z, x, y, y, y, x]
	weightArmPos = [10,10, 10,10]  # [z, x, z, y, z, x, y]
	weightArmVel = [10, 10, 10, 10]  # [z, x, z, y, z, x, y]
	weightTorsoPos = [1,1]  # [z, y]
	weightTorsoVel = [1,1]  # [z, y]
	stateWeights = np.array(
		weightBasePos
		+ weightLegPos * 2
		+ weightTorsoPos
		+ weightArmPos * 2
		+ weightBaseVel
		+ weightLegVel * 2
		+ weightTorsoVel
		+ weightArmVel * 2
	)

	weightuBase = "not actuated"
	weightuLeg = [1, 1, 1, 1, 1, 1]
	weightuArm = [1, 1, 1, 1]
	weightuTorso = [1, 1]
	controlWeight = np.array(weightuLeg * 2 
							+ weightuTorso 
							+ weightuArm * 2
	)
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
	simulator = "bullet"
# ####### CONFIGURATION  ############
# ### RobotWrapper
design_conf = dict(
    urdfPath=conf.modelPath + conf.URDF_SUBPATH,
    srdfPath=conf.modelPath + conf.SRDF_SUBPATH,
    leftFootName=conf.leftFootName,
    rightFootName=conf.rightFootName,
    rightHandName=conf.rightHandName,
    leftHandName=conf.leftHandName,
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
q0 = design.get_q0().copy()
q0[design.get_rModel().getJointId("arm_right_4_joint")+5] = -1.5
design.set_q0(q0)

MM_conf = dict(
    timeStep=conf.DT,
    gravity=conf.gravity,
    comHeight=conf.normal_height,
    omega=conf.omega,
    colDist=0,
    mu=conf.mu,
    coneBox=conf.coneBox,
    minNforce=conf.minNforce,
    maxNforce=conf.maxNforce,
    obstacleRadius=0.05,
    obstaclePosition=np.array([0.5,0.3,1]),
    obstacleHeight=2,
    wHandTranslation=conf.wHandTranslation,
    wHandRotation=conf.wHandRotation,
    wHandVelocity=conf.wHandVelocity,
    wHandCollision=0,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wForceHand=conf.wForceHand,
    wFrictionHand=0,
    wWrenchCone=conf.wWrenchCone,
    wDCM = conf.wDCM,
    wCoM = conf.wCoM,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    lowKinematicLimits=conf.lowKinematicLimits,
    highKinematicLimits=conf.highKinematicLimits,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMakerHand()
formuler.initialize(MM_conf, design)
phases = [Phase.NO_HAND for i in range(conf.T)]
all_models = formuler.formulateHorizon(phases)
ter_model = formuler.formulateTerminalHandTracker(Phase.NO_HAND)

# Horizon
H_conf = dict(leftFootName=conf.leftFootName, rightFootName=conf.rightFootName)
horizon = HorizonManager()

horizon.initialize(H_conf, design.get_x0(), all_models, ter_model)

# Full Horizon

wbc_conf = dict(
    T=conf.T,
    TtrackingToContact=conf.TtrackingToContact,
    Tcontact=conf.Tcontact,
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

targetContact = np.array([0.5, -0.2, 0.9])

mpc.ref_RH_pose = targetContact
mpc.ref_force = pin.Force(np.array([0,0,50,0,0,0]))

fullq0 = design.get_q0Complete().copy()
fullq0[design.get_rModelComplete().getJointId("arm_right_4_joint")+5] = -1.5
fullq0[design.get_rModelComplete().getJointId("arm_right_6_joint")+5] = -1.2
fullq0[design.get_rModelComplete().getJointId("arm_right_7_joint")+5] = -0.5

# Initialize Bullet simulator
device = BulletTalos(conf, design.get_rModelComplete())
device.initializeJoints(fullq0)
device.addTable("/local/src/bullet3/data",[1.0,-0.5,0.28])
device.showHandToTrack(targetContact)
q_current, v_current = device.measureState()

T_total = 1000
for s in range(T_total * conf.Nc):
	#time.sleep(0.001)
	if mpc.timeToSolveDDP(s):
		print("iteration " + str(mpc.iteration()) + ", time to land " + str(mpc.land_hand() + conf.T) + ", time to takeoff " + str(mpc.takeoff_hand() + conf.T))
	mpc.iterate(s, q_current, v_current)
	torques = horizon.currentTorques(mpc.x0)
	device.execute(torques)
	q_current, v_current = device.measureState()
	xMeasured = np.concatenate((q_current,v_current))
		 
device.close()
