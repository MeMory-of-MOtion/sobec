import hppfcl
import sys

import numpy as np
import crocoddyl
import example_robot_data
import pinocchio as pin
from pinocchio.utils import zero

# import matplotlib.pylab as plt
import sobec

import example_robot_data as robex

# ## Load model with some frozen joints
robot = robex.load("talos")
robot.model.q0 = robot.model.referenceConfigurations["half_sitting"]
blockedJointNames = [
    # "universe",
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
blockedJointIds = [
    i for (i, n) in enumerate(robot.model.names) if n in blockedJointNames
]
rmodel, [gmodel_vis, gmodel_col] = pin.buildReducedModel(
    robot.model,
    [robot.visual_model, robot.collision_model],
    blockedJointIds,
    robot.model.q0,
)
rmodel.q0 = rmodel.referenceConfigurations["half_sitting"]


# ## Open display
viz = pin.visualize.GepettoVisualizer(rmodel, gmodel_col, gmodel_vis)
viz.initViewer(loadModel=True)
viz.display(rmodel.q0)

rdata = rmodel.createData()

rmodel.defaultState = np.concatenate([rmodel.q0, np.zeros(rmodel.nv)])

# #################################################################################
#
# Choose time and walk settings
#
# #################################################################################

# Total number of nodes of the simulation
T_total = 2000

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

rightFoot = "right_sole_link"
rightFootId = rmodel.getFrameId(rightFoot)
leftFoot = "left_sole_link"
leftFootId = rmodel.getFrameId(leftFoot)


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
    state, framePlacementLeft, actuation.nu, np.array([0, 50])
)
contactModelDouble.addContact(leftFoot + "_contact", supportContactModelLeft)
framePlacementRight = crocoddyl.FramePlacement(
    rightFootId, rdata.oMf[rightFootId]
)  # rdata.oMi[1:][rightFootId]
supportContactModelRight = crocoddyl.ContactModel6D(
    state, framePlacementRight, actuation.nu, np.array([0, 50])
)
contactModelDouble.addContact(rightFoot + "_contact", supportContactModelRight)

contactModelLeft = crocoddyl.ContactModelMultiple(state, actuation.nu)
contactModelLeft.addContact(leftFoot + "_contact", supportContactModelLeft)

contactModelRight = crocoddyl.ContactModelMultiple(state, actuation.nu)
contactModelRight.addContact(rightFoot + "_contact", supportContactModelRight)

#  Create the cost functions

#  Cost for self-collision
maxfloat = sys.float_info.max
xlb = np.concatenate(
    [
        -maxfloat * np.ones(6),  # dimension of the SE(3) manifold
        rmodel.lowerPositionLimit[7:],
        -maxfloat * np.ones(state.nv),
    ]
)
xub = np.concatenate(
    [
        maxfloat * np.ones(6),  # dimension of the SE(3) manifold
        rmodel.upperPositionLimit[7:],
        maxfloat * np.ones(state.nv),
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
    crocoddyl.ActivationModelWeightedQuad(np.array(stateWeights)),
    xRegResidual,
)

uResidual = crocoddyl.ResidualModelControl(state, actuation.nu)
uRegCost = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelWeightedQuad(np.array(controlWeight)), uResidual
)

# Foot placement cost
startingPosLeftFoot = rdata.oMf[leftFootId].copy()
startingPosRightFoot = rdata.oMf[rightFootId].copy()

residualPlacementRight = crocoddyl.ResidualModelFramePlacement(
    state, rightFootId, startingPosRightFoot, actuation.nu
)
residualPlacementLeft = crocoddyl.ResidualModelFramePlacement(
    state, leftFootId, startingPosLeftFoot, actuation.nu
)

goalTrackingCostRight = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelQuadFlatLog(6, 0.002), residualPlacementRight
)
goalTrackingCostLeft = crocoddyl.CostModelResidual(
    state, crocoddyl.ActivationModelQuadFlatLog(6, 0.002), residualPlacementLeft
)

# CoM velocity regularization
residualCoMVelocity = sobec.ResidualModelCoMVelocity(
    state, np.array([0, 0, 0]), actuation.nu
)
comVelCost = crocoddyl.CostModelResidual(state, residualCoMVelocity)

# #####################################################################################
se3ObsPose = pin.SE3.Identity()
se3ObsPose.translation = np.array([0.0, 0.0, -0.05])
se3FootPose = pin.SE3.Identity()
se3FootPose.translation = np.array([0.0, 0.0, 0.0])

obstSize = [3, 1, 0.0]

# Add geometry objects for the feet
rightObstacle = "leg_right_6_link"
rightObstacleId = rmodel.getFrameId(rightObstacle)
leftObstacle = "leg_left_6_link"
leftObstacleId = rmodel.getFrameId(leftObstacle)

ig_foot_right = gmodel_col.addGeometryObject(
    pin.GeometryObject(
        "right_foot",
        rightFootId,
        rmodel.frames[rightFootId].parent,
        hppfcl.Sphere(0),
        se3FootPose,
    ),
    rmodel,
)

ig_foot_left = gmodel_col.addGeometryObject(
    pin.GeometryObject(
        "left_foot",
        leftFootId,
        rmodel.frames[leftFootId].parent,
        hppfcl.Sphere(0),
        se3FootPose,
    ),
    rmodel,
)
# Add obstacle in the world
ig_obs_ground = gmodel_col.addGeometryObject(
    pin.GeometryObject(
        "obstacle",
        rmodel.getFrameId("universe"),
        rmodel.frames[rmodel.getFrameId("universe")].parent,
        hppfcl.Box(obstSize[0], obstSize[1], obstSize[2]),
        se3ObsPose,
    ),
    rmodel,
)

ig_obs_left = gmodel_col.addGeometryObject(
    pin.GeometryObject(
        "obstacle_left",
        leftObstacleId,
        rmodel.frames[leftObstacleId].parent,
        hppfcl.Sphere(0),
        se3FootPose,
    ),
    rmodel,
)
ig_obs_right = gmodel_col.addGeometryObject(
    pin.GeometryObject(
        "obstacle_right",
        rightObstacleId,
        rmodel.frames[rightObstacleId].parent,
        hppfcl.Sphere(0),
        se3FootPose,
    ),
    rmodel,
)

gmodel_col.addCollisionPair(pin.CollisionPair(ig_foot_right, ig_obs_ground))
gmodel_col.addCollisionPair(pin.CollisionPair(ig_foot_left, ig_obs_ground))
gmodel_col.addCollisionPair(pin.CollisionPair(ig_foot_left, ig_obs_right))
gmodel_col.addCollisionPair(pin.CollisionPair(ig_foot_right, ig_obs_left))


residualPairCollisionRight = sobec.ResidualModelVelCollision(
    state, actuation.nu, gmodel_col, 0, rightFootId, pin.WORLD, 0.01
)
obstacleCostRight = crocoddyl.CostModelResidual(state, residualPairCollisionRight)

residualPairCollisionLeft = sobec.ResidualModelVelCollision(
    state, actuation.nu, gmodel_col, 1, leftFootId, pin.WORLD, 0.01
)
obstacleCostLeft = crocoddyl.CostModelResidual(state, residualPairCollisionLeft)

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

problem = crocoddyl.ShootingProblem(
    rmodel.defaultState, [runningModel] * T, runningModel
)

# Creating the DDP solver for this OC problem, defining a logger

ddp = crocoddyl.SolverFDDP(problem)
# ddp.setCallbacks([crocoddyl.CallbackVerbose()])
ddp.th_stop = 1e-6
ddp.th_grad = 1e-9
# Solving it with the DDP algorithm
xs0 = [rmodel.defaultState] * (T + 1)
us0 = [
    ddp.problem.runningModels[0].quasiStatic(
        ddp.problem.runningDatas[0], rmodel.defaultState
    )
] * T

ddp.solve(xs0, us0, 500, False)

xs = ddp.xs
us = ddp.us

viz.play(np.array(xs)[:, : rmodel.nq].T, DT)
