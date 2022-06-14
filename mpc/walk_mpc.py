import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import time

# Local imports
import sobec
from utils.save_traj import save_traj
import utils.walk_plotter as walk_plotter
from sobec.walk.robot_wrapper import RobotWrapper
from sobec.walk import ocp
from mpcparams import WalkParams
import utils.talos_low as talos_low
from sobec.walk.config_mpc import configureMPCWalk
import utils.viewer_multiple as viewer_multiple
import sobec.walk.miscdisp as miscdisp

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
        # croc.CallbackVerbose(),
        miscdisp.CallbackMPCWalk(robot.contactIds)
    ]
)

ddp.solve(x0s, u0s, 200)

# ### MPC #############################################################################
# ### MPC #############################################################################
# ### MPC #############################################################################

mpc = sobec.MPCWalk(ddp.problem)
configureMPCWalk(mpc, walkParams)
mpc.initialize(ddp.xs[: walkParams.Tmpc + 1], ddp.us[: walkParams.Tmpc])
# mpc.solver.setCallbacks([ croc.CallbackVerbose() ])
x = robot.x0

hx = [x.copy()]
for t in range(1, 1500):
    x = mpc.solver.xs[1]
    mpc.calc(x, t)

    print(
        f"{t:4d} {miscdisp.dispocp(mpc.problem,robot.contactIds)} "
        f"{mpc.solver.iter:4d} "
        f"reg={mpc.solver.x_reg:.3} "
        f"a={mpc.solver.stepLength:.3} "
    )

    hx.append(mpc.solver.xs[1].copy())

    if not t % 10:
        viz.display(x[: robot.model.nq])
        # time.sleep(walkParams.DT)

# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

plotter = walk_plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, np.array(hx), None, None)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference

plotter.plotBasis(target)
plotter.plotCom(robot.com0)
plotter.plotFeet()
plotter.plotFootCollision(walkParams.footMinimalDistance)

# mpcplotter = walk_plotter.WalkRecedingPlotter(robot.model, robot.contactIds, hxs)
# mpcplotter.plotFeet()

print("Run ```plt.ion(); plt.show()``` to display the plots.")

# ### SAVE #####################################################################
# ### SAVE #####################################################################
# ### SAVE #####################################################################

if walkParams.saveFile is not None:
    save_traj(np.array(hx), filename=walkParams.saveFile)

# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)
