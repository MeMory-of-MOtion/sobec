"""
Test the complete implementation of the OCP+MPC in C++.

"""


import numpy as np

# from numpy.linalg import norm
import example_robot_data as robex

# import crocoddyl as croc
import sobec
from sobec.walk import miscdisp

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

# --- CONTACT PATTERN
Tstart = 20
Tsingle = 15
Tdouble = 40
Tend = 20
Tmpc = 120

contactPattern = np.array(
    []
    + [[1, 1]] * Tstart
    + [[1, 1]] * Tdouble
    + [[1, 0]] * Tsingle
    + [[1, 1]] * Tdouble
    + [[0, 1]] * Tsingle
    + [[1, 1]] * Tdouble
    + [[1, 1]] * Tend
    + [[1, 1]]
).T

assert params.transitionDuration * 2 < Tdouble

# --- OCP
ocp = sobec.OCPWalk(robot, params, contactPattern)
ocp.buildSolver()

# --- MPC
mpc = sobec.MPCWalk(ocp.problem)
mpc.Tmpc = Tmpc
mpc.Tstart = Tstart
mpc.Tdouble = Tdouble
mpc.Tsingle = Tsingle
mpc.Tend = Tend
mpc.DT = params.DT
mpc.solver_th_stop = 1e-3
mpc.vcomRef = params.vcomRef
mpc.solver_reg_min = 1e-6
mpc.solver_maxiter = 2

mpc.initialize(ocp.solver.xs[: mpc.Tmpc + 1], ocp.solver.us[: mpc.Tmpc])
mpc.solver.setCallbacks(
    [
        # croc.CallbackVerbose(),
        miscdisp.CallbackMPCWalk(robot.contactIds)
    ]
)
x = robot.x0

for t in range(2 * mpc.Tmpc):
    mpc.calc(x, t)
    x = mpc.solver.xs[1]


# Check final state
assert abs(x[5] + 0.5510828857463467) < 1e-6
