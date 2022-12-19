#!/usr/bin/env python3
"""
Created on Mon May  9 17:15:22 2022

@author: nvilla
"""
import example_robot_data
import numpy as np

# PATHS

URDF_FILENAME = "talos_reduced.urdf"
SRDF_FILENAME = "talos.srdf"
SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
modelPath = example_robot_data.getModelPath(URDF_SUBPATH)

# Joint settings

blocked_joints = [
    "universe",
    # "torso_1_joint",
    # "torso_2_joint",
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
total_steps = 2
DT = 1e-2  # Time step of the DDP
T = 100  # Time horizon of the DDP (number of nodes)
TdoubleSupport = 100  # Double support time  # TODO: (check with 20)
TsingleSupport = 100  # Single support time

Tstep = TsingleSupport + TdoubleSupport

simu_step = simu_period = 1e-3  #

Nc = int(round(DT / simu_step))  # Number of control knots per planification timestep

# TODO: landing_advance and takeoff_delay are missing
ddpIteration = 1  # Number of DDP iterations

# #### PHYSICS #####

simulator = (
    "bullet"
    # "pinocchio"
)


gravity = np.array([0, 0, -9.81])

mu = 0.3
footSize = 0.05
cone_box = np.array([0.1, 0.05])
minNforce = 100
maxNforce = 1500  # This may be still too low

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
    H_stiff = [np.inf, np.inf, np.inf, np.inf]  # [LH_pitch, LH_roll, RH_pitch, RH_roll]
    H_damp = [np.inf, np.inf, np.inf, np.inf]
    flex_ratio = 1

flex_esti_delay = 0.0  # [s]
flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)

flex_esti_delay = 0.0  # [s]
flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)

flexToJoint = np.array([0, 0, 0.09])

# ###### WALKING GEOMETRY #########
xForward = 0.0  # step size
sidestep = 0.0
swingApex = 0.2  # foot height
footSeparation = 0.2  # 0.005 # Correction in y to push the feet away from each other
footPenetration = 0.0  # foot penetration in the ground

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
wWrenchCone = 1  # 0.05
wForceTask = 0
wCoP = 10
wDCM = 0

weightBasePos = [0, 0, 0, 100, 100, 0]  # [x, y, z| x, y, z]
weightBaseVel = [10, 10, 10, 10, 10, 10]  # [x, y, z| x, y, z]
weightLegPos = [1, 10, 10, 0.01, 0.1, 1]  # [z, x, y, y, y, x]
weightLegVel = [10, 10, 1, 0.1, 1, 10]  # [z, x, y, y, y, x]
weightArmPos = [0.01, 100, 1, 0.1]  # [z, x, z, y, z, x, y]
weightArmVel = [1, 100, 1, 1]  # [z, x, z, y, z, x, y]
weightTorsoPos = [10, 5]  # [z, y]
weightTorsoVel = [10, 5]  # [z, y]
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
weightuArm = [1, 1, 1, 1]
weightuTorso = [1, 1]
controlWeight = np.array(
    weightuLeg * 2
    + weightuTorso
    # + weightuArm * 2
)

forceWeights = np.array([1, 1, 1, 1, 1, 1])

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

th_stop = 1e-6  # threshold for stopping criterion
th_grad = 1e-9  # threshold for zero gradient.
