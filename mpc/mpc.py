import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt   ### Keep me, I am useful
from numpy.linalg import norm,pinv,inv,svd,eig ### Me as well, I am also super useful
import time

# Local imports
import sobec
from save_traj import loadProblemConfig,save_traj
import walk_plotter
from robot_wrapper import RobotWrapper
import walk_ocp as walk
from mpcparams import WalkParams
import miscdisp
import talos_low
from walk_mpc import WalkMPC

# #####################################################################################
# ### LOAD ROBOT ######################################################################
# #####################################################################################

# ## LOAD AND DISPLAY TALOS
# Load the robot model from example robot data and display it if possible in
# Gepetto-viewer
urdf = talos_low.load()
robot = RobotWrapper(urdf.model,contactKey='sole_link')

# #####################################################################################
# ### VIZ #############################################################################
# #####################################################################################
try:
    Viz = pin.visualize.GepettoVisualizer
    viz = Viz(urdf.model, urdf.collision_model, urdf.visual_model)
    viz.initViewer()
    viz.loadViewerModel()
    gv = viz.viewer.gui
except (ImportError,AttributeError):
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
    + [[1, 1]] * walkParams.T_START
    + [[1, 1]] * walkParams.T_DOUBLE
    + [[0, 1]] * walkParams.T_SINGLE
    + [[1, 1]] * walkParams.T_DOUBLE
    + [[1, 0]] * walkParams.T_SINGLE
    + [[1, 1]] * walkParams.T_DOUBLE
    + [[1, 1]] * walkParams.T_END
    + [[1, 1]]
)
# #####################################################################################
# ### DDP #############################################################################
# #####################################################################################

ddp = walk.buildSolver(robot,contactPattern,walkParams)
problem = ddp.problem
x0s,u0s = walk.buildInitialGuess(ddp.problem,walkParams)
ddp.setCallbacks([croc.CallbackVerbose()])

ddp.solve(x0s, u0s, 200)

# ### MPC #############################################################################
# ### MPC #############################################################################
# ### MPC #############################################################################

mpc = WalkMPC(robot,ddp.problem,walkParams,xs_init=ddp.xs,us_init=ddp.us)

x = robot.x0

for t in range(1, 500):
    x = mpc.solver.xs[1]
    mpc.run(x,t)

    if not t % 10:
        viz.display(x[: robot.model.nq])
        time.sleep(walkParams.DT)


# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

plotter = walk_plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, np.array(hx), None, None)

target = problem.terminalModel.differential.costs.costs["stateReg"].cost.residual.reference

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
    save_traj(np.array(hx),filename=walkParams.saveFile)

# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)
