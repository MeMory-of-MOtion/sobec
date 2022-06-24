import pinocchio as pin
import crocoddyl as croc
import numpy as np
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import time
import numpy.random
import example_robot_data.robots_loader as robex

# Local imports
import sobec
from sobec.walk_without_think.save_traj import save_traj
from sobec.walk_without_think.robot_wrapper import RobotWrapper
from sobec.walk_without_think import ocp
import mpcparams
from sobec.walk_without_think.config_mpc import configureMPCWalk
from sobec.pinbullet import SimuProxy
import sobec.viewer_multiple as viewer_multiple
from sobec.walk_without_think import miscdisp
from sobec.walk_without_think import talos_collections


class PyreneLoader(robex.RobotLoader):
    """
    Load the model used by Pierre on Pyrene.
    This model is not in robex, and as been copied on my (nmsd) /opt
    manually ... baaaad.
    If you are not me, change l87 with the following diff:
    - simu.loadExampleRobot("pyrene")
    + simu.loadExampleRobot("talos")
    Not much changes to expect, both models are quite similar.
    """

    path = "talos_data"
    urdf_filename = "pyrene.urdf"
    srdf_filename = "talos.srdf"
    free_flyer = True
    has_rotor_parameters = True


robex.ROBOTS["pyrene"] = PyreneLoader

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
walkParams = mpcparams.StandParams("talos_legs")
walkParams.jointNamesToLock = talos_collections.jointToLockCollection["pyrene_legs"]


# ## SIMU #############################################################################
# ## Load urdf model in pinocchio and bullet
simu = SimuProxy()
simu.loadExampleRobot("pyrene")
# simu.rmodel.q0 = q_init
simu.loadBulletModel()  # pyb.GUI)
simu.freeze(walkParams.jointNamesToLock)
simu.setTorqueControlMode()
simu.setTalosDefaultFriction()
# ## OCP ########################################################################
# ## OCP ########################################################################

# Reference model used in Pyrene simulation.
# Would be nice to add the as a standard parameters in walkParam ... for later.
pyrene_default_x0 = np.array(
    [
        -0.0251348,
        0.00373202,
        1.04257,
        -0.00022295,
        -0.00306881,
        -1.7334e-06,
        0.999995,
        2.74527e-05,
        -0.000336093,
        -0.367617,
        0.672117,
        -0.302917,
        0.000342691,
        -2.16974e-05,
        0.000216317,
        -0.367698,
        0.67208,
        -0.303009,
        -0.000275359,
        0.03354,
        9.05423e-05,
        0.000637797,
        -5.4381e-05,
        0.000833764,
        -5.16007e-05,
        -1.62615e-06,
        -8.1946e-05,
        -0.00016472,
        -0.000271272,
        -0.000346178,
        7.36229e-05,
        6.53619e-06,
        8.45607e-05,
        -0.000133411,
        -0.000298404,
        -0.000370441,
        -3.99304e-05,
    ]
)
pyrene_default_x0[simu.rmodel.nq :] = 0

robot = RobotWrapper(simu.rmodel, contactKey="sole_link")
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
ddp = ocp.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = ocp.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose()])
# ddp.problem.x0 = pyrene_default_x0
ddp.solve(x0s, u0s, 200)

mpcparams = sobec.MPCWalkParams()
configureMPCWalk(mpcparams, walkParams)
mpc = sobec.MPCWalk(mpcparams, ddp.problem)
mpc.initialize(ddp.xs[: walkParams.Tmpc + 1], ddp.us[: walkParams.Tmpc])
mpc.solver.setCallbacks(
    [
        croc.CallbackVerbose(),
        # miscdisp.CallbackMPCWalk(robot.contactIds)
    ]
)

"""
import matplotlib.pylab as plt  # noqa: E402,F401
import utils.walk_plotter as walk_plotter  # noqa: E402

sol = ocp.Solution(robot, ddp)

ocpplotter = walk_plotter.WalkPlotter(robot.model, robot.contactIds)
ocpplotter.setData(contactPattern, sol.xs, sol.us, sol.fs0)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference
forceRef = [
    walk_plotter.getReferenceForcesFromProblemModels(problem, cid)
    for cid in robot.contactIds
]
forceRef = [np.concatenate(fs) for fs in zip(*forceRef)]



wp=walkParams
frefocp = np.hstack([walk_plotter.getReferenceForcesFromProblemModels(ddp.problem,cid)
                              for cid in robot.contactIds ])
Tcycle = (wp.Tdouble+wp.Tsingle)*2

plt.figure()
plt.plot(frefocp[:,[2,8]])
plt.plot([wp.Tstart,wp.Tstart],[0,1000],'r')
wp=walkParams
plt.plot([wp.Tstart,wp.Tstart],[0,1000],'r')
plt.plot([wp.Tstart+wp.Tdouble]*2,[0,1000],'r')
plt.plot([wp.Tstart+wp.Tdouble+wp.Tsingle]*2,[0,1000],'r')
plt.plot([wp.Tstart+wp.Tdouble+wp.Tsingle+wp.Tdouble]*2,[0,1000],'r')
plt.plot([wp.Tstart+wp.Tdouble+wp.Tsingle+wp.Tdouble+wp.Tsingle]*2,[0,1000],'r')
plt.plot([wp.Tstart+wp.Tdouble+wp.Tsingle+wp.Tdouble+wp.Tsingle+wp.Tdouble]*2,[0,1000],'r')
plt.plot([wp.Tstart+wp.Tdouble+wp.Tsingle+wp.Tdouble+wp.Tsingle+wp.Tdouble+wp.Tend]*2,[0,1000],'r')
plt.plot([wp.Tstart+wp.Tdouble//2]*2,[0,1000],'k')
plt.plot([wp.Tstart+wp.Tdouble//2+Tcycle]*2,[0,1000],'k')

hf = [ f for f in frefocp[:wp.Tmpc,:]]
for t in range(1000):
    Tinit = wp.Tstart+wp.Tdouble//2;
    tlast = Tinit + ((t + wp.Tmpc - Tinit) % Tcycle);
    hf.append(  frefocp[tlast,:] )
hf = np.array(hf)
plt.figure()
plt.plot(hf[:,[2,8]])
stophere
"""

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
    viz0 = viewer_multiple.GepettoGhostViewer(
        simu.rmodel, simu.gmodel_col, simu.gmodel_vis, 0.8
    )
    viz0.hide()
except (ImportError, AttributeError):
    print("No viewer")

# ## MAIN LOOP ##################################################################

hx = [simu.getState()]
hu = []
hus = []
hxs = []
hf = []
hfref = []
hfsref = []


class SolverError(Exception):
    pass


def play():
    import time

    for i in range(0, len(hx), 10):
        viz.display(hx[i][: robot.model.nq])
        time.sleep(1e-2)


croc.enable_profiler()

# FOR LOOP
mpcPeriod = int(walkParams.DT / 1e-3)
for s in range(walkParams.Tsimu):  # int(20.0 / walkParams.DT)):

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

    start_time = time.time()
    mpc.calc(simu.getState(), s)
    solve_time = time.time() - start_time
    if mpc.solver.iter == 0:
        raise SolverError("0 iterations")

    # ### LOG DATA
    hxs.append(np.array(mpc.solver.xs))
    hus.append(np.array(mpc.solver.us))
    contactsm0 = mpc.problem.runningModels[0].differential.contacts.contacts
    contactsd0 = mpc.problem.runningDatas[0].differential.multibody.contacts.contacts
    fs = []
    fsref = []
    for cid in robot.contactIds:
        if "%s_contact" % robot.model.frames[cid].name in contactsm0:
            cd = contactsd0["%s_contact" % robot.model.frames[cid].name]
            fs.append((cd.jMf * cd.f).vector)
            cstsm = mpc.problem.runningModels[0].differential.costs.costs
            cm = cstsm["%s_forceref" % robot.model.frames[cid].name].cost
            fsref.append(cm.residual.reference.vector)
        else:
            fs.append(np.zeros(6))
            fsref.append(np.zeros(6))
    hf.append(np.concatenate(fs))
    hfref.append(np.concatenate(fsref))
    import sobec.walk_without_think.plotter as walk_plotter  # noqa: E402

    hfsref.append(
        np.hstack(
            [
                walk_plotter.getReferenceForcesFromProblemModels(mpc.problem, cid)
                for cid in robot.contactIds
            ]
        )
    )

    # ### DISPLAY
    print(
        "{:4d} {} {:4d} reg={:.3} a={:.3} solveTime={:.3}".format(
            s,
            miscdisp.dispocp(mpc.problem, robot.contactIds),
            mpc.solver.iter,
            mpc.solver.x_reg,
            mpc.solver.stepLength,
            solve_time,
        )
    )
    if not s % 10:
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
    save_traj(np.array(hx), filename=walkParams.saveFile)

# #####################################################################################
# #####################################################################################
# #####################################################################################

# The 2 next import must not be included **AFTER** pyBullet starts.
import matplotlib.pylab as plt  # noqa: E402,F401
import sobec.walk_without_think.plotter as walk_plotter  # noqa: E402

plotter = walk_plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(
    contactPattern, np.array(hx)[::mpcPeriod], np.array(hu)[::mpcPeriod], np.array(hf)
)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference

plotter.plotBasis(target)
plotter.plotCom(robot.com0)
plotter.plotFeet()
plotter.plotFootCollision(walkParams.footMinimalDistance, 50)
plotter.plotJointTorques()
plotter.plotForces(hfref)
plotter.plotAllForces()
plotter.plotTimeCop()
plotter.plotTimeCof()

print("Run ```plt.ion(); plt.show()``` to display the plots.")

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)

print("Run ```play()``` to visualize the motion.")

print("Run the following line to save the config:")
print(
    """
with open('autogen/stand12-autogen.hpp','w') as f:
  f.write(sobec.walk_without_think.params.generateParamFileForTheRobot(walkParams,robot))
"""
)
