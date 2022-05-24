import sys
import pybullet as p  # PyBullet simulator
import time  # Time module to sleep()
import pybullet_data

import numpy as np
import crocoddyl
import example_robot_data
import pinocchio as pin
from pinocchio.utils import zero

# import matplotlib.pylab as plt
import sobec

pin.switchToNumpyMatrix()


def m2a(m):
    return np.array(m.flat)


def a2m(a):
    return np.matrix(a).T


def getCurrentState():
    jointStates = p.getJointStates(robotId, JointIndicesComplete)  # State of all joints
    baseState = p.getBasePositionAndOrientation(robotId)
    baseVel = p.getBaseVelocity(robotId)

    # Joint vector for Pinocchio
    q = np.vstack(
        (
            np.array([baseState[0]]).transpose(),
            np.array([baseState[1]]).transpose(),
            np.array(
                [[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]
            ).transpose(),
        )
    )
    v = np.vstack(
        (
            np.array([baseVel[0]]).transpose(),
            np.array([baseVel[1]]).transpose(),
            np.array(
                [[jointStates[i_joint][1] for i_joint in range(len(jointStates))]]
            ).transpose(),
        )
    )
    return (q, v)


##################################################################################
#
# Initialize robot model
#
##################################################################################

URDF_FILENAME = "talos_reduced_corrected.urdf"
SRDF_FILENAME = "talos.srdf"
SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
modelPath = example_robot_data.getModelPath(URDF_SUBPATH)

rmodelComplete, geomModelComplete, visualModelComplete = pin.buildModelsFromUrdf(
    modelPath + URDF_SUBPATH, modelPath, pin.JointModelFreeFlyer()
)

# Take rotor inertia and gear ratio into account
pin.loadRotorParameters(rmodelComplete, modelPath + SRDF_SUBPATH, False)
rmodelComplete.armature = np.multiply(
    rmodelComplete.rotorInertia.flat, np.square(rmodelComplete.rotorGearRatio.flat)
)
pin.loadReferenceConfigurations(rmodelComplete, modelPath + SRDF_SUBPATH, False)

# Add free flyers joint limits
ub = rmodelComplete.upperPositionLimit
ub[:7] = 1
rmodelComplete.upperPositionLimit = ub
lb = rmodelComplete.lowerPositionLimit
lb[:7] = -1
rmodelComplete.lowerPositionLimit = lb

q0Complete = rmodelComplete.referenceConfigurations["half_sitting"]
q0CompleteStart = q0Complete.copy()
rdataComplete = rmodelComplete.createData()
nonControlledJoints = [
    "universe",
    # "arm_left_1_joint",
    # "arm_left_2_joint",
    # "arm_left_3_joint",
    # "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
    # "arm_right_1_joint",
    # "arm_right_2_joint",
    # "arm_right_3_joint",
    # "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
    "gripper_left_joint",
    "gripper_right_joint",
    "head_1_joint",
    "head_2_joint",
]

pinocchioControlledJoints = [
    i for (i, n) in enumerate(rmodelComplete.names) if n not in nonControlledJoints
]
# 1-6 leg_left, 7-12 leg_right, 13-14 torso, 15-21 arm_left, 22 gripper_left,
# 23-29 arm_right, 30 gripper_right, 31-32 head if using talos_reduced
JointsToLockId = [
    i for i in range(1, rmodelComplete.njoints) if i not in pinocchioControlledJoints
]

# Create reduced model
rmodel = pin.buildReducedModel(rmodelComplete, JointsToLockId, q0Complete)
rdata = rmodel.createData()
pin.loadRotorParameters(rmodel, modelPath + SRDF_SUBPATH, False)
rmodel.armature = np.multiply(
    rmodel.rotorInertia.flat, np.square(rmodel.rotorGearRatio.flat)
)
# Load reference configuration
pin.loadReferenceConfigurations(rmodel, modelPath + SRDF_SUBPATH)
q0 = rmodel.referenceConfigurations["half_sitting"]
q0Start = q0.copy()
rmodel.defaultState = np.concatenate([q0, np.zeros((rmodel.nv, 1))])

xStart = np.concatenate([q0Start, np.zeros((rmodel.nv, 1))])
endEffectorRight = "right_sole_link"
endEffectorIdRight = rmodel.getFrameId(endEffectorRight)
endEffectorLeft = "left_sole_link"
endEffectorIdLeft = rmodel.getFrameId(endEffectorLeft)
rightFoot = "right_sole_link"
rightFootId = rmodel.getFrameId(rightFoot)
leftFoot = "left_sole_link"
leftFootId = rmodel.getFrameId(leftFoot)
contactNames = [rightFoot, leftFoot]
# Calculate center of mass of the default position
pin.forwardKinematics(rmodel, rdata, q0)
pin.updateFramePlacements(rmodel, rdata)
comRef = pin.centerOfMass(rmodelComplete, rdataComplete, q0Complete)

# z coordinate of robot base when standing on ground
zBase_reference = 1.01927

##################################################################################
#
# Choose time and walk settings
#
##################################################################################

# Total number of nodes of the simulation
T_total = 20  ## 2000   ### CHANGE ME to 2000 for a full locomotion ... kept shorter for CI duration

# Time step of the DDP
DT = 1e-2

# Time horizon of the DDP (number of node)
T = 100

# Double support time
T2contact = 50

# Single support time
T1contact = 100

# Number of DDP iterations
ddpIteration = 1

# step size
xForward = 0.1

# foot height
foot_height = 0.03

# Foot depth in ground
TFootDepth = 220

# Correction in y to push the feet away from each other
yCorrection = 0.005

##################################################################################
#
# Create cost functions and contacts
#
##################################################################################

# Data structure
state = crocoddyl.StateMultibody(rmodel)
actuation = crocoddyl.ActuationModelFloatingBase(state)

# Add contact to the model
contactModelDouble = crocoddyl.ContactModelMultiple(state, actuation.nu)
framePlacementLeft = crocoddyl.FramePlacement(leftFootId, rdata.oMf[leftFootId])
supportContactModelLeft = crocoddyl.ContactModel6D(
    state, framePlacementLeft, actuation.nu, np.matrix([0, 50]).T
)
contactModelDouble.addContact(leftFoot + "_contact", supportContactModelLeft)
framePlacementRight = crocoddyl.FramePlacement(
    rightFootId, rdata.oMf[rightFootId]
)  # rdata.oMi[1:][rightFootId]
supportContactModelRight = crocoddyl.ContactModel6D(
    state, framePlacementRight, actuation.nu, np.matrix([0, 50]).T
)
contactModelDouble.addContact(rightFoot + "_contact", supportContactModelRight)

contactModelLeft = crocoddyl.ContactModelMultiple(state, actuation.nu)
contactModelLeft.addContact(leftFoot + "_contact", supportContactModelLeft)

contactModelRight = crocoddyl.ContactModelMultiple(state, actuation.nu)
contactModelRight.addContact(rightFoot + "_contact", supportContactModelRight)

#  Create the cost functions

#  Cost for self-collision
maxfloat = sys.float_info.max
xlb = np.vstack(
    [
        -maxfloat * np.matrix(np.ones((6, 1))),  # dimension of the SE(3) manifold
        rmodel.lowerPositionLimit[7:],
        -maxfloat * np.matrix(np.ones((state.nv, 1))),
    ]
)
xub = np.vstack(
    [
        maxfloat * np.matrix(np.ones((6, 1))),  # dimension of the SE(3) manifold
        rmodel.upperPositionLimit[7:],
        maxfloat * np.matrix(np.ones((state.nv, 1))),
    ]
)
bounds = crocoddyl.ActivationBounds(xlb, xub, 1.0)

limitCost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ActivationModelQuadraticBarrier(bounds),
    crocoddyl.ResidualModelState(state, zero(state.nx), actuation.nu),
)


# Wrench cone cost
coneRotationLeft = rdata.oMf[leftFootId].copy().rotation.T
coneRotationRight = rdata.oMf[rightFootId].copy().rotation.T
mu = 0.1
cone_box = np.array([0.1, 0.075])
minNforce = 200
maxNforce = 1200
wrenchConeFrameLeft = crocoddyl.WrenchCone(
    coneRotationLeft, mu, cone_box, 4, True, minNforce, maxNforce
)
wrenchConeFrameRight = crocoddyl.WrenchCone(
    coneRotationRight, mu, cone_box, 4, True, minNforce, maxNforce
)
boundsFrictionLeft = crocoddyl.ActivationBounds(
    wrenchConeFrameLeft.lb, wrenchConeFrameLeft.ub, 1.0
)
boundsFrictionRight = crocoddyl.ActivationBounds(
    wrenchConeFrameRight.lb, wrenchConeFrameRight.ub, 1.0
)


wrenchRefTwoSupports = zero(len(wrenchConeFrameLeft.ub))
fz_ref2 = 400
wrenchRefTwoSupports[4] = fz_ref2
wrenchRefTwoSupports[5] = -cone_box[1] * fz_ref2
wrenchRefTwoSupports[6] = -cone_box[1] * fz_ref2
wrenchRefTwoSupports[7] = -cone_box[0] * fz_ref2
wrenchRefTwoSupports[8] = -cone_box[0] * fz_ref2
for i in range(4):
    wrenchRefTwoSupports[i] = -fz_ref2 * mu
for i in range(8):
    wrenchRefTwoSupports[i + 9] = -fz_ref2 * mu * (cone_box[0] + cone_box[1])

wrenchRef1 = zero(len(wrenchConeFrameLeft.ub))
fz_ref1 = 800
wrenchRef1[4] = fz_ref1
wrenchRef1[5] = -cone_box[1] * fz_ref1
wrenchRef1[6] = -cone_box[1] * fz_ref1
wrenchRef1[7] = -cone_box[0] * fz_ref1
wrenchRef1[8] = -cone_box[0] * fz_ref1
for i in range(4):
    wrenchRef1[i] = -fz_ref1 * mu
for i in range(8):
    wrenchRef1[i + 9] = -fz_ref1 * mu * (cone_box[0] + cone_box[1])

wrenchConeResidualLeft = crocoddyl.ResidualModelContactWrenchCone(
    state, leftFootId, wrenchConeFrameLeft, actuation.nu
)
wrenchConeResidualRight = crocoddyl.ResidualModelContactWrenchCone(
    state, rightFootId, wrenchConeFrameRight, actuation.nu
)
wrenchConeCostLeft = crocoddyl.CostModelResidual(
    state, sobec.ActivationModelQuadRef(wrenchRef1), wrenchConeResidualLeft
)
wrenchConeCostRight = crocoddyl.CostModelResidual(
    state, sobec.ActivationModelQuadRef(wrenchRef1), wrenchConeResidualRight
)

wrenchConeResidualLeft2 = crocoddyl.ResidualModelContactWrenchCone(
    state, leftFootId, wrenchConeFrameLeft, actuation.nu
)
wrenchConeResidualRight2 = crocoddyl.ResidualModelContactWrenchCone(
    state, rightFootId, wrenchConeFrameRight, actuation.nu
)
wrenchConeCostLeft2 = crocoddyl.CostModelResidual(
    state, sobec.ActivationModelQuadRef(wrenchRefTwoSupports), wrenchConeResidualLeft2
)
wrenchConeCostRight2 = crocoddyl.CostModelResidual(
    state, sobec.ActivationModelQuadRef(wrenchRefTwoSupports), wrenchConeResidualRight2
)

#  Cost for state and control

runningCosts = np.array(
    [1000.0, 0.1, 0.001, 0, 1e3, 0.005, 100]
)  # [1.,0.02,0.0004,0.0,1e3]
# GoalTracking cost, State regularization cost, control cost, limit cost
terminalCosts = np.array([8000.0, 0.02, 0.0, 0, 0.0])  # [10.0,0.02,0.0,0.0,0.0]

weightBasePos = [0, 0, 0, 100000, 100000, 100000]
weightBaseVel = [0, 0, 0, 1000, 1000, 1000]
weightLegPos = [100, 100, 100, 10000, 100, 100]
weightLegVel = [1000, 1000, 1000, 1000, 1000, 1000]
weightArmRightPos = [10000, 10000, 10000, 10000]  # ,100,100,100 for 3 last arm joint
weightArmRightVel = [1000, 1000, 1000, 1000]  # ,100,100,100 for 3 last arm joint
weightArmLeftPos = [10000, 10000, 10000, 10000]  # ,100,100,100 for 3 last arm joint
weightArmLeftVel = [100, 100, 100, 100]  # ,100,100,100 for 3 last arm joint
weightTorsoPos = [500, 500]
weightTorsoVel = [500, 500]

stateWeights = np.array(
    weightBasePos
    + weightLegPos * 2
    + weightTorsoPos
    + weightArmLeftPos
    + weightArmRightPos
    + weightBaseVel
    + weightLegVel * 2
    + weightTorsoVel
    + weightArmLeftVel
    + weightArmRightVel
)

weightuBase = [0, 0, 0, 0, 0, 0]
weightuLeg = [1, 1, 1, 1, 10, 10]
weightuArm = [10, 10, 10, 10]
weightuTorso = [1, 1]
controlWeight = np.array(weightuLeg * 2 + weightuTorso + weightuArm * 2)


xRegResidual = crocoddyl.ResidualModelState(state, rmodel.defaultState, actuation.nu)
xRegCost = crocoddyl.CostModelResidual(
    state,
    crocoddyl.ActivationModelWeightedQuad(np.matrix(stateWeights).T),
    xRegResidual,
)

uResidual = crocoddyl.ResidualModelControl(state, actuation.nu)
uRegCost = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelWeightedQuad(np.matrix(controlWeight).T), uResidual
)

# Foot placement cost
startingPosLeftFoot = rdata.oMf[endEffectorIdLeft].copy()
startingPosRightFoot = rdata.oMf[endEffectorIdRight].copy()

residualPlacementRight = crocoddyl.ResidualModelFramePlacement(
    state, endEffectorIdRight, startingPosRightFoot, actuation.nu
)
residualPlacementLeft = crocoddyl.ResidualModelFramePlacement(
    state, endEffectorIdLeft, startingPosLeftFoot, actuation.nu
)

goalTrackingCostRight = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelQuadFlatLog(6, 0.002), residualPlacementRight
)
goalTrackingCostLeft = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelQuadFlatLog(6, 0.002), residualPlacementLeft
)

# CoM velocity regularization
residualCoMVelocity = sobec.ResidualModelCoMVelocity(
    state, a2m(np.array([0, 0, 0])), actuation.nu
)
comVelCost = crocoddyl.CostModelResidual(state, residualCoMVelocity)

##################################################################################
#
# Initialize cost model, action model and DDP
#
##################################################################################

# Weights for all costs

wFootPlacement = 1000
wStateReg = 0.1
wControlReg = 0.001
wLimit = 1e3
wVCoM = 0
wWrenchCone = 0.005

# Create cost model per each action model
runningCostModel = crocoddyl.CostModelSum(state, actuation.nu)
runningCostModel.addCost("stateReg", xRegCost, wStateReg)
runningCostModel.addCost("ctrlReg", uRegCost, wControlReg)
runningCostModel.addCost("limitcost", limitCost, wLimit)
runningCostModel.addCost("wrenchConeRight", wrenchConeCostRight2, wWrenchCone)
runningCostModel.addCost("wrenchConeLeft", wrenchConeCostLeft2, wWrenchCone)
runningCostModel.addCost("comVelCost", comVelCost, wVCoM)

dmodelRunning = crocoddyl.DifferentialActionModelContactFwdDynamics(
    state, actuation, contactModelDouble, runningCostModel, 0, True
)
runningModel = crocoddyl.IntegratedActionModelEuler(dmodelRunning, DT)

# Update control reference to gravity compensating torque in half sitting position
temp_data = runningModel.createData()
uResidual.reference = runningModel.quasiStatic(temp_data, rmodel.defaultState)

# Create running cost model for simple support
runningCostModelLeftSwing = crocoddyl.CostModelSum(state, actuation.nu)
runningCostModelRightSwing = crocoddyl.CostModelSum(state, actuation.nu)

runningCostModelLeftSwing.addCost(
    "gripperPoseLeft", goalTrackingCostLeft, wFootPlacement
)
runningCostModelLeftSwing.addCost("stateReg", xRegCost, wStateReg)
runningCostModelLeftSwing.addCost("ctrlReg", uRegCost, wControlReg)
runningCostModelLeftSwing.addCost("limitcost", limitCost, wLimit)
runningCostModelLeftSwing.addCost("wrenchConeRight", wrenchConeCostRight, wWrenchCone)
runningCostModelLeftSwing.addCost("comVelCost", comVelCost, wVCoM)

dmodelRunningLeftSwing = crocoddyl.DifferentialActionModelContactFwdDynamics(
    state, actuation, contactModelRight, runningCostModelLeftSwing, 0, True
)
runningModelLeftSwing = crocoddyl.IntegratedActionModelEuler(dmodelRunningLeftSwing, DT)

runningCostModelRightSwing = crocoddyl.CostModelSum(state, actuation.nu)
runningCostModelRightSwing.addCost(
    "gripperPoseRight", goalTrackingCostRight, wFootPlacement
)
runningCostModelRightSwing.addCost("stateReg", xRegCost, wStateReg)
runningCostModelRightSwing.addCost("ctrlReg", uRegCost, wControlReg)
runningCostModelRightSwing.addCost("limitcost", limitCost, wLimit)
runningCostModelRightSwing.addCost("wrenchConeLeft", wrenchConeCostLeft, wWrenchCone)
runningCostModelRightSwing.addCost("comVelCost", comVelCost, wVCoM)

dmodelRunningRightSwing = crocoddyl.DifferentialActionModelContactFwdDynamics(
    state, actuation, contactModelLeft, runningCostModelRightSwing, 0, True
)
runningModelRightSwing = crocoddyl.IntegratedActionModelEuler(
    dmodelRunningRightSwing, DT
)

problem = crocoddyl.ShootingProblem(xStart, [runningModel] * T, runningModel)

# Creating the DDP solver for this OC problem, defining a logger

ddp = crocoddyl.SolverFDDP(problem)
# ddp.setCallbacks([crocoddyl.CallbackVerbose()])
ddp.th_stop = 1e-6
ddp.th_grad = 1e-9
# Solving it with the DDP algorithm
xs0 = [xStart] * (T + 1)
us0 = [
    ddp.problem.runningModels[0].quasiStatic(ddp.problem.runningDatas[0], xStart)
] * T

ddp.solve(xs0, us0, 500, False)

xs = ddp.xs
us = ddp.us


##################################################################################
#
# Initialize pybullet
#
##################################################################################

# Start the client for PyBullet
physicsClient = p.connect(p.DIRECT)
# p.resetSimulation()
p.setTimeStep(1e-3)
# Set gravity (disabled by default)
p.setGravity(0, 0, -8.91)


# Load horizontal plane
robotStartPosition = [0.0, 0.0, 1.01927]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load horizontal plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setAdditionalSearchPath(modelPath + "/talos_data/robots/")
robotId = p.loadURDF(
    URDF_FILENAME, robotStartPosition, robotStartOrientation, useFixedBase=False
)

localInertiaPos = a2m(p.getDynamicsInfo(robotId, -1)[3])
# Get the number of joints
numJoints = p.getNumJoints(robotId)

# leg_left (45-50), leg_right (52-57), torso (0-1), arm_left (11-17),
# gripper_left (21), arm_right (28-34), gripper_right (38), head (3,4)
bulletJointNames = [
    p.getJointInfo(robotId, i)[1].decode() for i in range(p.getNumJoints(robotId))
]
JointIndicesComplete = [
    bulletJointNames.index(rmodelComplete.names[i])
    for i in range(2, rmodelComplete.njoints)
]

# Joints controlled with crocoddyl
bulletControlledJoints = [
    i
    for i in JointIndicesComplete
    if p.getJointInfo(robotId, i)[1].decode() not in nonControlledJoints
]

# Disable default position controler in torque controlled joints
# Default controller will take care of other joints
p.setJointMotorControlArray(
    robotId,
    jointIndices=bulletControlledJoints,
    controlMode=p.VELOCITY_CONTROL,
    forces=[0.0 for m in bulletControlledJoints],
)

# Augment friction to forbid feet sliding
p.changeDynamics(1, 50, lateralFriction=100, spinningFriction=30)
p.changeDynamics(1, 57, lateralFriction=100, spinningFriction=30)

# Initialize position in pyBullet
initial_joint_positions = m2a(q0CompleteStart[7:]).tolist()
for i in range(len(initial_joint_positions)):
    p.enableJointForceTorqueSensor(1, i, True)
    p.resetJointState(robotId, JointIndicesComplete[i], initial_joint_positions[i])


# Visual representation of the target to track
# The visual will not appear unless the physics client is set to SHARED_MEMMORY
mesh_scale = [0.05, 0.05, 0.05]
visualShapeTarget = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.1, 0.075, 0.05],
    rgbaColor=[0.0, 0.0, 1.0, 1.0],
    specularColor=[0.4, 0.4, 0],
    visualFramePosition=[0.0, 0.0, 0.0],
)

sphereIdRight = p.createMultiBody(
    baseMass=0.0,
    baseInertialFramePosition=[0, 0, 0],
    baseVisualShapeIndex=visualShapeTarget,
    basePosition=[
        startingPosRightFoot.translation[0],
        startingPosRightFoot.translation[1],
        startingPosRightFoot.translation[2],
    ],
    useMaximalCoordinates=True,
)

sphereIdLeft = p.createMultiBody(
    baseMass=0.0,
    baseInertialFramePosition=[0, 0, 0],
    baseVisualShapeIndex=visualShapeTarget,
    basePosition=[
        startingPosLeftFoot.translation[0],
        startingPosLeftFoot.translation[1],
        startingPosLeftFoot.translation[2],
    ],
    useMaximalCoordinates=True,
)

visualShapeTargetCom = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.05, 0.05, 0.05],
    rgbaColor=[0.0, 1.0, 0.0, 1.0],
    specularColor=[0.4, 0.4, 0],
    visualFramePosition=[0.0, 0.0, 0.0],
)
##################################################################################
#
# Launch simulation
#
##################################################################################
# Number of control knots per planification timestep
Nc = int(DT / 1e-3)

# Counter until beginning of single phase
Tswitch2single = T + T2contact
# Counter until beginning of double phase
Tswitch2double = T + T2contact + T1contact

swingRightPhaseDouble = True
swingRightPhaseSimple = True
firstStep = True

for s in range(T_total):
    # print("Tp = ", s)
    # Controler works 10 times faster than trajectory generation
    for k in range(Nc):
        time.sleep(0.001)
        # Compute one step of simulation
        q_current, v_current = getCurrentState()

        # Compute current position and velocity of all crocoddyl joints
        qc = np.vstack((q_current[[i + 5 for i in pinocchioControlledJoints[1:]]]))
        vc = np.vstack((v_current[[i + 4 for i in pinocchioControlledJoints[1:]]]))
        xinit0 = np.vstack((q_current[:7], qc, v_current[:6], vc))
        xinit0[:3] -= localInertiaPos
        # Compute torque to be applied by adding Riccati term

        torques = us[0] + ddp.K[0] * (state.diff(xinit0, xs[0]))

        # Apply torque on complete model
        p.setJointMotorControlArray(
            robotId,
            bulletControlledJoints,
            controlMode=p.TORQUE_CONTROL,
            forces=torques,
        )
        p.stepSimulation()

    # Initialize MPC with current state of simulation
    pin.forwardKinematics(rmodel, rdata, xs[0][: rmodel.nq])
    pin.updateFramePlacements(rmodel, rdata)

    if Tswitch2single >= 0 and Tswitch2single <= T:
        if swingRightPhaseSimple:
            ddp.problem.updateModel(Tswitch2single, runningModelRightSwing)
            print("update to swing right contact model ", Tswitch2single)
        else:
            ddp.problem.updateModel(Tswitch2single, runningModelLeftSwing)
            print("update to swing left contact model ", Tswitch2single)
    if Tswitch2double >= 0 and Tswitch2double <= T:
        ddp.problem.updateModel(Tswitch2double, runningModel)
        print("update to double contact model ", Tswitch2double)
    if Tswitch2double >= 0 and Tswitch2double <= T1contact:
        if swingRightPhaseDouble:
            # Update right foot desired placement
            targetFrameNow = startingPosRightFoot.copy()
            targetFrameNow.translation[0] = startingPosRightFoot.translation[
                0
            ] + xForward * float(T1contact - Tswitch2double) / float(T1contact)
            targetFrameNow.translation[1] = startingPosRightFoot.translation[
                1
            ] - yCorrection * float(T1contact - Tswitch2double) / float(T1contact)
            targetFrameNow.translation[2] = startingPosRightFoot.translation[
                2
            ] + foot_height * np.sin(
                float(T1contact - Tswitch2double)
                / float(T1contact)
                * TFootDepth
                * 3.14
                / 180
            )
            residualPlacementRight.reference = targetFrameNow

        else:
            # Update left foot desired placement
            targetFrameNow = startingPosLeftFoot.copy()
            targetFrameNow.translation[0] = startingPosLeftFoot.translation[
                0
            ] + xForward * float(T1contact - Tswitch2double) / float(T1contact)
            targetFrameNow.translation[1] = startingPosLeftFoot.translation[
                1
            ] + yCorrection * float(T1contact - Tswitch2double) / float(T1contact)
            targetFrameNow.translation[2] = startingPosLeftFoot.translation[
                2
            ] + foot_height * np.sin(
                float(T1contact - Tswitch2double)
                / float(T1contact)
                * TFootDepth
                * 3.14
                / 180
            )
            residualPlacementLeft.reference = targetFrameNow

    p.resetBasePositionAndOrientation(
        sphereIdRight,
        posObj=[
            residualPlacementRight.reference.translation[0],
            residualPlacementRight.reference.translation[1],
            residualPlacementRight.reference.translation[2],
        ],
        ornObj=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    p.resetBasePositionAndOrientation(
        sphereIdLeft,
        posObj=[
            residualPlacementLeft.reference.translation[0],
            residualPlacementLeft.reference.translation[1],
            residualPlacementLeft.reference.translation[2],
        ],
        ornObj=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    Tswitch2single -= 1
    Tswitch2double -= 1

    if Tswitch2single < 0:
        Tswitch2single = T1contact + T2contact
        swingRightPhaseSimple = not (swingRightPhaseSimple)
    if Tswitch2double < 0:
        if firstStep:
            xForward = 2 * xForward
            firstStep = False
        startingPosLeftFoot = rdata.oMf[endEffectorIdLeft].copy()
        startingPosRightFoot = rdata.oMf[endEffectorIdRight].copy()

        Tswitch2double = T1contact + T2contact
        swingRightPhaseDouble = not (swingRightPhaseDouble)

        # Update wrench cone orientation
        coneRotationLeft = rdata.oMf[leftFootId].copy().rotation.T
        coneRotationRight = rdata.oMf[rightFootId].copy().rotation.T
        wrenchConeFrameLeft = crocoddyl.WrenchCone(
            coneRotationLeft, mu, cone_box, 4, True, minNforce, maxNforce
        )
        wrenchConeFrameRight = crocoddyl.WrenchCone(
            coneRotationRight, mu, cone_box, 4, True, minNforce, maxNforce
        )

        wrenchConeResidualLeft.reference = wrenchConeFrameLeft
        wrenchConeResidualLeft2.reference = wrenchConeFrameLeft
        wrenchConeResidualRight.reference = wrenchConeFrameRight
        wrenchConeResidualRight2.reference = wrenchConeFrameRight

    # Warm start problem with previous shifted solution
    xs = list(xs[1:]) + [xs[-1]]
    xs[0] = xinit0
    us = list(us[1:]) + [us[-1]]
    ddp.problem.x0 = xs[0]
    ddp.solve(xs, us, ddpIteration, False)
    xs = ddp.xs
    us = ddp.us

p.disconnect()
