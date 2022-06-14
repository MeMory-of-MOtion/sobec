import sys

import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
from utils.save_traj import loadProblemConfig, save_traj
from params import WalkParams
import utils.walk_plotter as walk_plotter
import utils.talos_low as talos_low
from sobec.walk.robot_wrapper import RobotWrapper
from sobec.walk import ocp


# workaround python 2
if sys.version_info.major < 3:
    FileNotFoundError = IOError


# #####################################################################################
# ### LOAD ROBOT ######################################################################
# #####################################################################################

# ## LOAD AND DISPLAY TALOS
# Load the robot model from example robot data and display it if possible in
# Gepetto-viewer

urdf = talos_low.load()
robot = RobotWrapper(urdf.model, contactKey="sole_link")

# #####################################################################################
# ### VIZ #############################################################################
# #####################################################################################
try:
    Viz = pin.visualize.GepettoVisualizer
    viz = Viz(urdf.model, urdf.collision_model, urdf.visual_model)
    viz.initViewer()
    viz.loadViewerModel()
    gv = viz.viewer.gui
except (ImportError, AttributeError):
    print("No viewer")


# #####################################################################################
# ## TUNING ###########################################################################
# #####################################################################################

# In the code, cost terms with 0 weight are commented for reducing execution cost
# An example of working weight value is then given as comment at the end of the line.
# When setting them to >0, take care to uncomment the corresponding line.
# All these lines are marked with the tag ##0##.

walkParams = WalkParams()
assert len(walkParams.stateImportance) == robot.model.nv * 2

try:
    # If possible, the initial state and contact pattern are taken from a file.
    ocpConfig = loadProblemConfig()
    contactPattern = ocpConfig["contactPattern"]
    robot.x0 = ocpConfig["x0"]
    stateTerminalTarget = ocpConfig["stateTerminalTarget"]
except (KeyError, FileNotFoundError):
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

q0 = robot.x0[: robot.model.nq]
print(
    "Start from q0=",
    "half_sitting"
    if norm(q0 - robot.model.referenceConfigurations["half_sitting"]) < 1e-9
    else q0,
)

# #####################################################################################
# ### DDP #############################################################################
# #####################################################################################

ddp = ocp.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = ocp.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose()])

ddp.solve(x0s, u0s, 200)

# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

sol = ocp.Solution(robot, ddp)

# plotter = walk_plotter.WalkPlotter(robot.model, robot.contactIds)
# plotter.setData(contactPattern, sol.xs, sol.us, sol.fs0)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference
forceRef = [
    walk_plotter.getReferenceForcesFromProblemModels(problem, cid)
    for cid in robot.contactIds
]
forceRef = [np.concatenate(fs) for fs in zip(*forceRef)]

# plotter.plotBasis(target)
# plotter.plotTimeCop()
# plotter.plotCopAndFeet(walkParams.FOOT_SIZE, 0.6)
# plotter.plotForces(forceRef)
# plotter.plotCom(robot.com0)
# plotter.plotFeet()
# plotter.plotFootCollision(walkParams.footMinimalDistance)
print("Run ```plt.ion(); plt.show()``` to display the plots.")

# ### SAVE #####################################################################
# ### SAVE #####################################################################
# ### SAVE #####################################################################

if walkParams.saveFile is not None:
    save_traj(np.array(sol.xs), filename=walkParams.saveFile)

# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)

viz.play(np.array(ddp.xs)[:, : robot.model.nq].T, walkParams.DT)
