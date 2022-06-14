import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import example_robot_data as robex

# Local imports
from sobec.walk.robot_wrapper import RobotWrapper
import sobec.walk.ocp as walk

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
    flyHighSlope = 5.0 / refFootFlyingAltitude
    footMinimalDistance = 0.2  # (.17 is the max value wrt initial config)
    soleCollision = True
    towCollision = False
    heelCollision = False
    MAIN_JOINTS = [
        "leg_%s_%s_joint" % (side, idx)
        for side in ["left", "right"]
        for idx in [1, 2, 4]
    ]

    vcomRef = np.array([0.1, 0, 0])
    vcomSelection = [0, 1, 2]
    vcomImportance = np.array([0.0, 0, 1])
    footSize = 0.05

    kktDamping = 0  # 1e-6
    baumgartGains = np.array([0, 50])
    solver_th_stop = 1e-3

    guessFile = "/tmp/test_walk.npy"
    saveFile = "/tmp/test_walk.npy"


walkParams = WalkParams()
assert len(walkParams.stateImportance) == robot.model.nv * 2

# When the config file is not found ...
# Initial config, also used for warm start, both taken from robot wrapper.
# Contact are specified with the order chosen in <contactIds>.
contactPattern = (
    []
    + [[1, 1]] * 40
    + [[1, 0]] * 50
    + [[1, 1]] * 11
    + [[0, 1]] * 50
    + [[1, 1]] * 11
    + [[1, 0]] * 50
    + [[1, 1]] * 11
    + [[0, 1]] * 50
    + [[1, 1]] * 11
    + [[1, 1]] * 40
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

check = np.array(
    [
        3.15812463e-01,
        -4.44631989e-03,
        1.03557792e00,
        -2.84186401e-03,
        6.41050576e-03,
        2.50842913e-06,
        9.99975414e-01,
        2.73143210e-02,
        5.62718368e-02,
        -3.56145437e-01,
        7.35471487e-01,
        -3.92421939e-01,
        -4.96056156e-02,
        3.89824155e-02,
        1.54883314e-02,
        -4.02183036e-01,
        7.25496089e-01,
        -3.39097274e-01,
        -1.76945975e-02,
        4.50751241e-03,
        2.12906042e-03,
        2.81653759e-04,
        -1.59541228e-03,
        4.26372467e-03,
        -5.12017268e-05,
        3.21540456e-04,
        -1.05209226e-03,
        1.13691676e-03,
        -6.82627950e-04,
        -4.79744147e-03,
        2.45665041e-03,
        1.34906157e-04,
        -1.04575429e-03,
        1.87336735e-03,
        -2.17019766e-03,
        -4.05249273e-03,
        2.40620228e-03,
    ]
)
assert norm(ddp.xs[-1] - check) < 1e-6
