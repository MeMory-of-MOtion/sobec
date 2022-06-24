import pinocchio as pin
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import crocoddyl as croc

# Local imports
import sobec
import sobec.walk_without_think.plotter
import specific_params

# #####################################################################################
# ## TUNING ###########################################################################
# #####################################################################################

# In the code, cost terms with 0 weight are commented for reducing execution cost
# An example of working weight value is then given as comment at the end of the line.
# When setting them to >0, take care to uncomment the corresponding line.
# All these lines are marked with the tag ##0##.

walkParams = specific_params.WalkParams("talos_low")

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
# ### LOAD ROBOT ######################################################################
# #####################################################################################

# ## LOAD AND DISPLAY TALOS
# Load the robot model from example robot data and display it if possible in
# Gepetto-viewer
urdf = sobec.talos_collections.robexLoadAndReduce("talos", walkParams.robotName)
robot = sobec.wwt.RobotWrapper(urdf.model, contactKey="sole_link")
assert len(walkParams.stateImportance) == robot.model.nv * 2

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
# ### DDP #############################################################################
# #####################################################################################

ddp = sobec.wwt.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks(
    [
        croc.CallbackVerbose(),
        croc.CallbackLogger(),
        # miscdisp.CallbackMPCWalk(robot.contactIds)
    ]
)

with open("/tmp/mpc-repr.ascii", "w") as f:
    f.write(sobec.reprProblem(ddp.problem))
    print("OCP described in /tmp/mpc-repr.ascii")

ddp.solve(x0s, u0s, 200)

assert sobec.logs.checkGitRefs(ddp.getCallbacks()[1], "refs/mpc-logs.npy")

# ### MPC #############################################################################
# ### MPC #############################################################################
# ### MPC #############################################################################

mpcparams = sobec.MPCWalkParams()
sobec.wwt.config_mpc.configureMPCWalk(mpcparams, walkParams)
mpc = sobec.MPCWalk(mpcparams, ddp.problem)
mpc.initialize(ddp.xs[: walkParams.Tmpc + 1], ddp.us[: walkParams.Tmpc])
# mpc.solver.setCallbacks([ croc.CallbackVerbose() ])
x = robot.x0

hx = [x.copy()]
for t in range(walkParams.Tsimu):
    x = mpc.solver.xs[1]
    mpc.calc(x, t)

    print(
        "{:4d} {} {:4d} reg={:.3} a={:.3} ".format(
            t,
            sobec.wwt.dispocp(mpc.problem, robot.contactIds),
            mpc.solver.iter,
            mpc.solver.x_reg,
            mpc.solver.stepLength,
        )
    )

    hx.append(mpc.solver.xs[1].copy())

    if not t % 10:
        viz.display(x[: robot.model.nq])
        # time.sleep(walkParams.DT)

# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

plotter = sobec.wwt.plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, np.array(hx), None, None)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference

plotter.plotBasis(target)
plotter.plotCom(robot.com0)
plotter.plotFeet()
plotter.plotFootCollision(walkParams.footMinimalDistance)

print("Run ```plt.ion(); plt.show()``` to display the plots.")

# ### SAVE #####################################################################
# ### SAVE #####################################################################
# ### SAVE #####################################################################

if walkParams.saveFile is not None:
    sobec.wwt.save_traj(np.array(hx), filename=walkParams.saveFile)

# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)
