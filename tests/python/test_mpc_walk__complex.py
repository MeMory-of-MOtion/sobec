import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import example_robot_data as robex

# Local imports
import sobec
from sobec.walk.robot_wrapper import RobotWrapper
import sobec.walk.ocp as walk
from sobec.walk.config_mpc import configureMPCWalk

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
    basisVWeight = [0, 0, 0, 3, 3, 1]  # ## was 003331
    legVWeight = [1] * 6

    stateImportance = np.array(
        basisQWeight + legQWeight + legQWeight + basisVWeight + legVWeight + legVWeight
    )

    stateTerminalImportance = np.array([3, 3, 0, 0, 0, 30] + [0] * 12 + [1] * 18)

    legUWeight = [1, 1, 1, 1, 10, 10]
    controlImportance = np.array(legUWeight * 2)

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
    flyWeight = 20
    groundColWeight = 200
    conePenaltyWeight = 20
    feetCollisionWeight = 1000

    lowbandwidthweight = 0  # 2e-1
    minTorqueDiffWeight = 0  # 2e-2

    refForceWeight = 10
    contiForceWeight = 0

    impactAltitudeWeight = 20000
    impactVelocityWeight = 200
    impactRotationWeight = 200
    refMainJointsAtImpactWeight = 0  # 2e2 # For avoinding crossing legs

    stateTerminalWeight = 20  # 2000
    terminalNoVelocityWeight = 2000
    terminalXTargetWeight = 0  # ##DDP## 2000

    refFootFlyingAltitude = 3e-2
    flyHighSlope = 5 / refFootFlyingAltitude
    footMinimalDistance = 0.2  # (.17 is the max value wrt initial config)
    soleCollision = True
    towCollision = False
    heelCollision = False
    MAIN_JOINTS = [
        f"leg_{side}_{idx}_joint" for side in ["left", "right"] for idx in [1, 2, 4]
    ]

    vcomRef = np.array([0.1, 0, 0])
    vcomSelection = [0, 1, 2]
    vcomImportance = np.array([0.0, 0, 1])
    FOOT_SIZE = 0.05

    kktDamping = 0  # 1e-6
    baumgartGains = np.array([0, 50])
    solver_th_stop = 1e-3
    solver_reg_min = 1e-6
    solver_maxiter = 2

    guessFile = "/tmp/test_walk.npy"
    saveFile = "/tmp/test_walk.npy"

    Tstart = int(0.3 / DT)
    Tsingle = int(0.8 / DT)  # 60
    Tdouble = int(0.11 / DT)  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.6 / DT)  # 120


walkParams = WalkParams()
assert len(walkParams.stateImportance) == robot.model.nv * 2

# When the config file is not found ...
# Initial config, also used for warm start, both taken from robot wrapper.
# Contact are specified with the order chosen in <contactIds>.
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

ddp = walk.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = walk.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose()])

ddp.solve(x0s, u0s, 200)

# #####################################################################################
# ### MPC #############################################################################
# #####################################################################################

mpc = sobec.MPCWalk(ddp.problem)
configureMPCWalk(mpc, walkParams)
mpc.initialize(ddp.xs[: walkParams.Tmpc + 1], ddp.us[: walkParams.Tmpc])
mpc.solver.setCallbacks([croc.CallbackVerbose()])
x = robot.x0

for t in range(1, 15):
    x = mpc.solver.xs[1]
    mpc.calc(x, t)

check = np.array(
    [
        8.19811307e-04,
        -7.07255527e-03,
        1.02688869e00,
        -8.02112629e-04,
        1.17162324e-03,
        -2.72496255e-03,
        9.99995279e-01,
        5.48002571e-03,
        1.29941872e-02,
        -3.88250814e-01,
        8.05909236e-01,
        -4.20006554e-01,
        -1.31048098e-02,
        5.48000220e-03,
        1.29846596e-02,
        -3.88707695e-01,
        8.03790868e-01,
        -4.17431304e-01,
        -1.30952821e-02,
        1.79525819e-03,
        -9.11847767e-02,
        5.42256698e-02,
        -7.59432201e-05,
        7.22967759e-03,
        -3.05032681e-02,
        3.09301339e-02,
        1.38597821e-01,
        1.80816609e-01,
        -4.04190683e-01,
        2.16138447e-01,
        -1.38563774e-01,
        3.09298128e-02,
        1.38493824e-01,
        1.73809030e-01,
        -4.06037793e-01,
        2.24993143e-01,
        -1.38459777e-01,
    ]
)

assert norm(x - check) < 1e-6
