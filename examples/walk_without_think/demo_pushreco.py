import pinocchio as pin
import crocoddyl as croc
import numpy as np
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import time
import numpy.random
import pybullet as pyb

# Local imports
import sobec
from sobec.pinbullet import SimuProxy
import specific_params
import random


# from sobec.walk_without_think.talos_collections import jointToLockCollection

q_init = np.array(
    [
        0.00000e00,
        0.00000e00,
        1.01927e00,
        0.00000e00,
        0.00000e00,
        0.00000e00,
        1.00000e00,
        0.00000e00,
        0.00000e00,
        -4.11354e-01,
        8.59395e-01,
        -4.48041e-01,
        -1.70800e-03,
        0.00000e00,
        0.00000e00,
        -4.11354e-01,
        8.59395e-01,
        -4.48041e-01,
        -1.70800e-03,
        0.00000e00,
        6.76100e-03,
        2.58470e-01,
        1.73046e-01,
        -2.00000e-04,
        -5.25366e-01,
        0.00000e00,
        0.00000e00,
        1.00000e-01,
        0.00000e00,
        -2.58470e-01,
        -1.73046e-01,
        2.00000e-04,
        -5.25366e-01,
        0.00000e00,
        0.00000e00,
        1.00000e-01,
        0.00000e00,
        0.00000e00,
        0.00000e00,
    ]
)
q_init_robot = np.concatenate([q_init[:19], [q_init[24], q_init[24 + 8]]])
walkParams = specific_params.PushParams("talos_low")

# ## SIMU #############################################################################
# ## Load urdf model in pinocchio and bullet
simu = SimuProxy()
simu.loadExampleRobot("talos")
# simu.rmodel.q0 = q_init
simu.loadBulletModel()  # pyb.GUI)
simu.freeze(walkParams.jointNamesToLock)
simu.setTorqueControlMode()
simu.setTalosDefaultFriction()
# ## OCP ########################################################################
# ## OCP ########################################################################

robot = sobec.wwt.RobotWrapper(simu.rmodel, contactKey="sole_link")
# robot.x0 = np.concatenate([q_init_robot, np.zeros(simu.rmodel.nv)])
assert len(walkParams.stateImportance) == robot.model.nv * 2

assert norm(robot.x0 - simu.getState()) < 1e-6

# Left foot moves first
# contactPattern = (
#     []
#     + [[1, 1]] * walkParams.Tstart
#     + [[1, 1]] * walkParams.Tdouble
#     + [[0, 1]] * walkParams.Tsingle
#     + [[1, 1]] * walkParams.Tdouble
#     + [[1, 0]] * walkParams.Tsingle
#     + [[1, 1]] * walkParams.Tdouble
#     + [[1, 1]] * walkParams.Tend
#     + [[1, 1]]
# )

# Right foot moves first
contactPattern = (
    []
    + [[1, 1]] * walkParams.Tstart
    + [[1, 1]] * walkParams.Tdouble
    + [[1, 0]] * walkParams.Tsingle
    + [[1, 1]] * walkParams.Tdouble
    + [[0, 1]] * walkParams.Tsingle
    + [[1, 1]] * walkParams.Tdouble
    + [[1, 1]] * walkParams.Tend
    + [[1, 1]]
)

# DDP for a full walk cycle, use as a standard pattern for the MPC.
ddp = sobec.wwt.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose()])
ddp.solve(x0s, u0s, 200)
with open("/tmp/pushreco-repr.ascii", "w") as f:
    f.write(sobec.reprProblem(ddp.problem))
    print("OCP described in /tmp/pushreco-repr.ascii")

mpcparams = sobec.MPCWalkParams()
sobec.wwt.config_mpc.configureMPCWalk(mpcparams, walkParams)
mpc = sobec.MPCWalk(mpcparams, ddp.problem)
mpc.initialize(ddp.xs[: walkParams.Tmpc + 1], ddp.us[: walkParams.Tmpc])
# mpc.solver.setCallbacks([
# croc.CallbackVerbose(),
# sobec.wwt.CallbackMPCWalk(robot.contactIds)
# ])
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
    viz0 = sobec.GepettoGhostViewer(simu.rmodel, simu.gmodel_col, simu.gmodel_vis, 0.8)
    viz0.hide()
except (ImportError, AttributeError):
    print("No viewer")

# ## MAIN LOOP ##################################################################

hx = [simu.getState()]
hu = []
hxs = []


class SolverError(Exception):
    pass


def play():
    import time

    for i in range(0, len(hx), 10):
        viz.display(hx[i][: robot.model.nq])
        time.sleep(1e-2)


croc.enable_profiler()
# viz.play(np.array(ddp.xs)[:, : robot.model.nq].T, walkParams.DT)

viz.viewer.gui.deleteNode("world/cub", True)
viz.viewer.gui.addBox("world/cub", 0.15, 0.15, 0.15, [0.5, 1, 1, 1])
viz.viewer.gui.setVisibility("world/cub", "OFF")

Timpact = 111
direc = np.array([0.8, 0.2, 0])
Tcub = 15
mass = 5
vcub = 100.0

nextSolve = -1

hiter = []
# FOR LOOP
mpcPeriod = int(walkParams.DT / 1e-3)
for s in range(walkParams.Tsimu):
    # ###############################################################################
    # # For timesteps without MPC updates
    for k in range(mpcPeriod):
        # Get simulation state
        x = simu.getState()

        # Compute Ricatti feedback
        torques = mpc.solver.us[0] + np.dot(
            mpc.solver.K[0], (mpc.state.diff(x, mpc.solver.xs[0]))
        )

        # generate random numbers close to 1 that multiply the desired torques
        noise = np.ones_like(torques) + walkParams.torque_noise * (
            2 * np.random.rand(torques.shape[0]) - 1.0
        )
        real_torques = noise * torques

        # Run one step of simu
        simu.step(real_torques)

        hx.append(simu.getState())
        hu.append(torques.copy())

    # ###############################################################################

    # ### CUBE
    if s > Tcub and Timpact - (s % Timpact) < Tcub:
        xcub = x[:3] + direc * vcub * walkParams.DT * (Timpact - (s % Timpact)) / Tcub
        viz.viewer.gui.applyConfiguration("world/cub", xcub.tolist() + [1, 0, 0, 0])
        viz.viewer.gui.refresh()
        viz.viewer.gui.setVisibility("world/cub", "ON")
        # viz.viewer.gui.refresh()
        print("CUB at ", xcub)

    if s > Tcub and (Timpact - s < 5):
        pyb.applyExternalForce(simu.robotId, 1, -direc * vcub * mass, np.zeros(3), True)
        print("Impact!", direc)

    if s > Tcub and s % Timpact == 0:
        print("Finish, new dir!")
        viz.viewer.gui.setVisibility("world/cub", "OFF")
        Timpact += random.randint(50, 250)
        direc = np.random.rand(3) * 2 - 1
        direc[2] = 0
        direc /= np.linalg.norm(direc)
        mpc.solver.solve(mpc.solver.xs, mpc.solver.us, 50)
        hiter[-1] += mpc.solver.iter

    start_time = time.time()
    mpc.calc(simu.getState(), s)
    solve_time = time.time() - start_time
    if mpc.solver.iter == 0:
        raise SolverError("0 iterations")
    hxs.append(np.array(mpc.solver.xs))
    hiter.append(mpc.solver.iter)

    # f"{mpc.basisRef[0]:.03} "
    print(
        "{:4d} {} {:4d} reg={:.3} a={:.3} solveTime={:.3}".format(
            s,
            sobec.wwt.dispocp(mpc.problem, robot.contactIds),
            mpc.solver.iter,
            mpc.solver.x_reg,
            mpc.solver.stepLength,
            solve_time,
        )
    )
    viz.display(simu.getState()[: robot.model.nq])

    # Before each takeoff, the robot display the previewed movement (3 times)
    if (
        walkParams.showPreview
        and len(mpc.problem.runningModels[0].differential.contacts.contacts) == 2
        and len(mpc.problem.runningModels[1].differential.contacts.contacts) == 1
    ):
        print("Ready to take off!")
        viz0.show()
        viz0.play(np.array(mpc.solver.xs)[:, : robot.model.nq].T, walkParams.DT)
        time.sleep(1)
        viz0.play(np.array(mpc.solver.xs)[::-1, : robot.model.nq].T, walkParams.DT)
        time.sleep(1)
        viz0.play(np.array(mpc.solver.xs)[:, : robot.model.nq].T, walkParams.DT)
        time.sleep(1)

croc.stop_watch_report(3)

# #####################################################################################
# #####################################################################################
# #####################################################################################

if walkParams.saveFile is not None:
    sobec.wwt.save_traj(np.array(hx), filename=walkParams.saveFile)

# #####################################################################################
# #####################################################################################
# #####################################################################################

# The 2 next import must not be included **AFTER** pyBullet starts.
import matplotlib.pylab as plt  # noqa: E402,F401
import sobec.walk_without_think.plotter  # noqa: E402,F401

plotter = sobec.wwt.plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, np.array(hx), None, None)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference

plotter.plotBasis(target)
plotter.plotCom(robot.com0)
plotter.plotFeet()
plotter.plotFootCollision(walkParams.footMinimalDistance, 50)

# mpcplotter = walk_plotter.WalkRecedingPlotter(robot.model, robot.contactIds, hxs)
# mpcplotter.plotFeet()

print("Run ```plt.ion(); plt.show()``` to display the plots.")

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)

print("Run ```play()``` to visualize the motion.")
