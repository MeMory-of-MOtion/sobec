import crocoddyl as croc
import numpy as np
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import example_robot_data as robex

# Local imports
import sobec
from sobec.walk.robot_wrapper import RobotWrapper
from sobec.walk import ocp

# from mpc_params import WalkParams
from sobec.walk.config_mpc import configureMPCWalk
from sobec.walk.miscdisp import CallbackMPCWalk

print("---TOTOT--", CallbackMPCWalk)
# #####################################################################################
# ### LOAD ROBOT ######################################################################
# #####################################################################################

# ## LOAD AND DISPLAY TALOS
# Load the robot model from example robot data and display it if possible in
# Gepetto-viewer
urdf = robex.load("talos_legs")
robot = RobotWrapper(urdf.model, contactKey="sole_link")

# #####################################################################################
# ## TUNING ###########################################################################
# #####################################################################################

# In the code, cost terms with 0 weight are commented for reducing execution cost
# An example of working weight value is then given as comment at the end of the line.
# When setting them to >0, take care to uncomment the corresponding line.
# All these lines are marked with the tag ##0##.


class WalkParams:
    DT = 0.010

    basisQWeight = [0, 0, 0, 50, 50, 0]
    legQWeight = [5, 5, 1, 2, 1, 1]
    torsoQWeight = [10, 10]
    armQWeight = [3, 3]
    basisVWeight = [0, 0, 0, 3, 3, 1]  # ## was 003331
    legVWeight = [1] * 6
    torsoVWeight = [20] * 2
    armVWeight = [2] * 2

    stateImportance = np.array(
        basisQWeight + legQWeight + legQWeight + basisVWeight + legVWeight + legVWeight
    )

    stateTerminalImportance = np.array([3, 3, 0, 0, 0, 30] + [0] * 12 + [1] * 18)

    legUWeight = [1, 1, 1, 1, 10, 10]
    torsoUWeight = [1, 1]
    armUWeight = [10, 10]
    controlImportance = np.array(legUWeight * 2)

    # ## Gains for force continuity: wfref for tracking the reference, wfcont for time
    # difference
    refTorqueWeight = 0
    refStateWeight = 1e-1
    flatBaseWeight = 0  # 20
    forceImportance = np.array([1, 1, 0.1, 10, 10, 2])
    coneAxisWeight = 2e-4
    comWeight = 0  # 20
    vcomImportance = np.array([0.0, 0, 1])
    vcomWeight = 1
    acomWeight = 0  # 16*DT
    copWeight = 2
    verticalFootVelWeight = 20
    footVelWeight = 0  # 20
    footAccWeight = 0  # 2
    flyWeight = 200
    groundColWeight = 200
    conePenaltyWeight = 0
    feetCollisionWeight = 1000

    lowbandwidthweight = 0  # 2e-1
    minTorqueDiffWeight = 0  # 2e-2

    refForceWeight = 10
    contiForceWeight = 0

    impactAltitudeWeight = 20000
    impactVelocityWeight = 10000
    impactRotationWeight = 200
    refMainJointsAtImpactWeight = 0  # 2e2 # For avoinding crossing legs

    stateTerminalWeight = 20  # 2000
    terminalNoVelocityWeight = 2000
    terminalXTargetWeight = 0  # ##DDP## 2000

    enforceMinimalFootDistance = False

    refFootFlyingAltitude = 7e-2
    flyHighSlope = 3 / refFootFlyingAltitude
    footMinimalDistance = 0.2  # (.17 is the max value wrt initial config)
    soleCollision = True
    towCollision = False
    heelCollision = False
    MAIN_JOINTS = [
        "leg_%s_%s_joint" % (side, idx)
        for side in ["left", "right"]
        for idx in [1, 2, 4]
    ]

    vcomRef = np.array([0.05, 0, 0])

    footSize = 0.05

    kktDamping = 0  # 1e-6
    baumgartGains = np.array([0, 100])

    solver_th_stop = 1e-3
    solver_maxiter = 2
    solver_reg_min = 1e-6

    # New parameters
    Tstart = int(0.3 / DT)
    Tsingle = int(0.8 / DT)  # 60
    Tdouble = int(0.11 / DT)  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.6 / DT)  # 120

    guessFile = None


walkParams = WalkParams()
assert len(walkParams.stateImportance) == robot.model.nv * 2

contactPattern = (
    []
    + [[1, 1]] * walkParams.Tstart
    + [[1, 1]] * walkParams.Tdouble
    + [[0, 1]] * walkParams.Tsingle
    + [[1, 1]] * walkParams.Tdouble
    + [[1, 0]] * walkParams.Tsingle
    + [[1, 1]] * walkParams.Tdouble
    + [[1, 1]] * walkParams.Tend
    + [[1, 1]]
)
# #####################################################################################
# ### DDP #############################################################################
# #####################################################################################

ddp = ocp.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = ocp.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks(
    [
        croc.CallbackVerbose(),
    ]
)

ddp.solve(x0s, u0s, 200)

# ### MPC #############################################################################
# ### MPC #############################################################################
# ### MPC #############################################################################

mpc = sobec.MPCWalk(ddp.problem)
configureMPCWalk(mpc, walkParams)
mpc.initialize(ddp.xs[: walkParams.Tmpc + 1], ddp.us[: walkParams.Tmpc])
mpc.solver.setCallbacks(
    [
        # croc.CallbackVerbose(),
        CallbackMPCWalk(robot.contactIds)
    ]
)

x = robot.x0.copy()
for t in range(1, 100):
    mpc.calc(x, t)
    x = mpc.solver.xs[1]
