"""
Test the complete implementation of the OCP+MPC in C++.

"""


import numpy as np

# from numpy.linalg import norm
import example_robot_data as robex

# import crocoddyl as croc
import sobec
from sobec.walk_without_think import miscdisp

urdf = robex.load("talos_legs")
urdf.model.name = "talos"
robot = sobec.OCPRobotWrapper(urdf.model, "sole_link", "half_sitting")

params = sobec.OCPWalkParams()

params.DT = 0.01
params.mainJointIds = []
params.baumgartGains = np.array([0.0, 100.0])
params.stateImportance = np.array(
    [
        0.0,
        0.0,
        0.0,
        50.0,
        50.0,
        0.0,
        5.0,
        5.0,
        1.0,
        2.0,
        1.0,
        1.0,
        5.0,
        5.0,
        1.0,
        2.0,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        3.0,
        3.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
    ]
)
params.stateTerminalImportance = np.array(
    [
        3.0,
        3.0,
        0.0,
        0.0,
        0.0,
        30.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0,
    ]
)
params.controlImportance = np.array(
    [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
)
params.vcomImportance = np.array([0.0, 0.0, 1.0])
params.forceImportance = np.array(
    [
        1.0,
        1.0,
        0.1,
        10.0,
        10.0,
        2.0,
    ]
)
params.vcomRef = np.array(
    [
        0.05,
        0.0,
        0.0,
    ]
)
params.footSize = 0.05
params.refStateWeight = 0.1
params.refTorqueWeight = 0.0
params.comWeight = 0.0
params.vcomWeight = 1.0
params.copWeight = 2.0
params.conePenaltyWeight = 0.0
params.coneAxisWeight = 0.0002
params.refForceWeight = 10.0
params.impactAltitudeWeight = 20000.0
params.impactVelocityWeight = 10000.0
params.impactRotationWeight = 200.0
params.refMainJointsAtImpactWeight = 0.0
params.verticalFootVelWeight = 20.0
params.flyHighSlope = 42.857142857142854
params.flyHighWeight = 200.0
params.groundColWeight = 200.0
params.footMinimalDistance = 0.2
params.feetCollisionWeight = 1000.0
params.kktDamping = 0.0
params.stateTerminalWeight = 20.0
params.solver_th_stop = 0.001
params.transitionDuration = 6

mpcparams = sobec.MPCWalkParams()
mpcparams.Tstart = 20
mpcparams.Tsingle = 15
mpcparams.Tdouble = 40
mpcparams.Tend = 20
mpcparams.Tmpc = 120
mpcparams.DT = params.DT
mpcparams.solver_th_stop = 1e-3
mpcparams.vcomRef = params.vcomRef
mpcparams.solver_reg_min = 1e-6
mpcparams.solver_maxiter = 2

# --- CONTACT PATTERN

contactPattern = np.array(
    []
    + [[1, 1]] * mpcparams.Tstart
    + [[1, 1]] * mpcparams.Tdouble
    + [[1, 0]] * mpcparams.Tsingle
    + [[1, 1]] * mpcparams.Tdouble
    + [[0, 1]] * mpcparams.Tsingle
    + [[1, 1]] * mpcparams.Tdouble
    + [[1, 1]] * mpcparams.Tend
    + [[1, 1]]
).T

assert params.transitionDuration * 2 < mpcparams.Tdouble

# --- OCP
ocp = sobec.OCPWalk(robot, params, contactPattern)
ocp.buildSolver()

# --- MPC

mpc = sobec.MPCWalk(mpcparams, ocp.problem)

mpc.initialize(ocp.solver.xs[: mpcparams.Tmpc + 1], ocp.solver.us[: mpcparams.Tmpc])
mpc.solver.setCallbacks(
    [
        # croc.CallbackVerbose(),
        miscdisp.CallbackMPCWalk(robot.contactIds)
    ]
)
x = robot.x0

for t in range(2 * mpcparams.Tmpc):
    mpc.calc(x, t)
    x = mpc.solver.xs[1]


# Check final state
assert abs(x[5] + 0.5510828857463467) < 1e-6
