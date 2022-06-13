import pinocchio as pin
import crocoddyl as croc
import numpy as np
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import matplotlib.pylab as plt  # noqa: F401
import time

# Local imports
from save_traj import save_traj
from robot_wrapper import RobotWrapper
import walk_ocp as walk
from mpcparams import WalkParams
import walk_plotter
import talos_low
from walk_mpc import WalkMPC
from pinbullet import SimuProxy
import viewer_multiple

# ## SIMU #############################################################################
# ## Load urdf model in pinocchio and bullet
simu = SimuProxy()
simu.loadExampleRobot("talos")
simu.loadBulletModel()  # pyb.GUI)
simu.freeze(talos_low.jointToLockNames)
simu.setTorqueControlMode()
simu.setTalosDefaultFriction()
# ## OCP ########################################################################
# ## OCP ########################################################################

robot = RobotWrapper(simu.rmodel, contactKey="sole_link")
walkParams = WalkParams()
assert len(walkParams.stateImportance) == robot.model.nv * 2

assert norm(robot.x0 - simu.getState()) < 1e-6

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

# DDP for a full walk cycle, use as a standard pattern for the MPC.
ddp = walk.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = walk.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose()])
ddp.solve(x0s, u0s, 200)

mpc = WalkMPC(robot, ddp.problem, walkParams, xs_init=ddp.xs, us_init=ddp.us)

# #####################################################################################
# ### VIZ #############################################################################
# #####################################################################################
try:
    Viz = pin.visualize.GepettoVisualizer
    viz = Viz(simu.rmodel, simu.gmodel_col, simu.gmodel_vis)
    viz.initViewer()
    viz.loadViewerModel()
    gv = viz.viewer.gui
    viz.display(simu.getState()[: robot.model.nq])
    viz0 = viewer_multiple.GepettoGhostViewer(simu.rmodel, simu.gmodel_col,simu.gmodel_vis,.8)
    viz0.hide()
except (ImportError,AttributeError):
    print("No viewer")

# ## MAIN LOOP ##################################################################

hx = []
hu = []
hxs = []


class SolverError(Exception):
    pass


# FOR LOOP
for s in range(2000):

    # ###############################################################################
    # # For timesteps without MPC updates
    for k in range(int(walkParams.DT / 1e-3)):
        # Get simulation state
        x = simu.getState()

        # Compute Ricatti feedback
        torques = mpc.solver.us[0] + mpc.solver.K[0] @ (
            mpc.state.diff(x, mpc.solver.xs[0])
        )

        # Run one step of simu
        simu.step(torques)

        hx.append(simu.getState())
        hu.append(torques.copy())

    # ###############################################################################
    # mpc.run(mpc.solver.xs[1],s)
    mpc.run(simu.getState(), s)
    if mpc.solver.iter == 0:
        raise SolverError("0 iterations")
    hxs.append(np.array(mpc.solver.xs))

    lrm = mpc.problem.runningModels[20].differential.costs.costs
    if (
        "%s_altitudeimpact" % robot.model.frames[robot.contactIds[0]].name in lrm
        or "%s_altitudeimpact" % robot.model.frames[robot.contactIds[1]].name in lrm
    ):
        mpc.moreIterations(50)
        print("+%s" % mpc.solver.iter)

    viz.display(simu.getState()[: robot.model.nq])

    if (len(mpc.problem.runningModels[0].differential.contacts.contacts)==2 and
        len(mpc.problem.runningModels[1].differential.contacts.contacts)==1):
        print('Ready to take off!')
        viz0.show()
        viz0.play(np.array(mpc.solver.xs)[:,:robot.model.nq].T,walkParams.DT)
        time.sleep(1)
        viz0.play(np.array(mpc.solver.xs)[::-1,:robot.model.nq].T,walkParams.DT)
        time.sleep(1)
        viz0.play(np.array(mpc.solver.xs)[:,:robot.model.nq].T,walkParams.DT)
        time.sleep(1)

# #####################################################################################
# #####################################################################################
# #####################################################################################

if walkParams.saveFile is not None:
    save_traj(np.array(hx), filename=walkParams.saveFile)

# #####################################################################################
# #####################################################################################
# #####################################################################################

plotter = walk_plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, np.array(hx), None, None)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference

plotter.plotBasis(target)
plotter.plotCom(robot.com0)
plotter.plotFeet()
plotter.plotFootCollision(walkParams.footMinimalDistance,50)

# mpcplotter = walk_plotter.WalkRecedingPlotter(robot.model, robot.contactIds, hxs)
# mpcplotter.plotFeet()

print("Run ```plt.ion(); plt.show()``` to display the plots.")

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)
