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

x0 = robot.x0.copy()

mpcProblem = croc.ShootingProblem(x0, problem.runningModels[:walkParams.Tmpc], problem.terminalModel)
mpcSolver = croc.SolverFDDP(mpcProblem)
# mpcSolver.setCallbacks([croc.CallbackVerbose()])
mpcSolver.th_stop = 1e-3

stateTarget = x0.copy()
stateTarget[:3] = x0[:3] + walkParams.VCOM_TARGET * walkParams.Tmpc * walkParams.DT
mpcProblem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference = stateTarget

mpcSolver.solve(ddp.xs[: walkParams.Tmpc + 1], ddp.us[:walkParams.Tmpc], 10, isFeasible=True)
x = mpcSolver.xs[1].copy()

hxs = [np.array(ddp.xs)]
hx = [x]
hiter = [mpcSolver.iter]
for t in range(1, 500):
    stateTarget = x0.copy()
    stateTarget[:3] = x0[:3] + walkParams.VCOM_TARGET * (t + walkParams.Tmpc) * walkParams.DT
    mpcProblem.terminalModel.differential.costs.costs[
        "stateReg"
    ].cost.residual.reference = stateTarget
    # tlast = t+walkParams.Tmpc
    tlast = walkParams.T_START + 1 + ((t + walkParams.Tmpc - walkParams.T_START - 1) % (2 * walkParams.T_SINGLE + 2 * walkParams.T_DOUBLE))
    # print(f't={t} ... last is {tlast}')
    mpcProblem.circularAppend(problem.runningModels[tlast], problem.runningDatas[tlast])
    mpcProblem.x0 = x.copy()
    # assert(mpcProblem.runningModels[0] == problem.runningModels[t])
    xg = list(mpcSolver.xs)[1:] + [mpcSolver.xs[-1]]
    ug = list(mpcSolver.us)[1:] + [mpcSolver.us[-1]]
    # if t==100: stophere
    mpcSolver.solve(xg, ug, maxiter=1)
    print(
        f"{t:4d} {miscdisp.dispocp(mpcProblem,robot.contactIds)} "
        f"{stateTarget[0]:.03} {mpcSolver.iter:4d}"
    )
    x = mpcSolver.xs[1].copy()
    hx.append(x)
    hxs.append(np.array(mpcSolver.xs))
    hiter.append(mpcSolver.iter)
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
