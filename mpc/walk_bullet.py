import pinocchio as pin
import crocoddyl as croc
import numpy as np
from numpy.linalg import norm,pinv,inv,svd,eig ### Me as well, I am also super useful
#import time

# Local imports
from save_traj import save_traj
from robot_wrapper import RobotWrapper
import walk_ocp as walk
from mpcparams import WalkParams
#import miscdisp
import talos_low
from walk_mpc import WalkMPC
from pinbullet import SimuProxy
import pybullet as pyb

# ## SIMU #############################################################################
# ## Load urdf model in pinocchio and bullet
simu = SimuProxy()
simu.loadExampleRobot("talos")
simu.loadBulletModel()#pyb.GUI)
simu.freeze(talos_low.jointToLockNames)
simu.setTorqueControlMode()
simu.setTalosDefaultFriction()
# ## OCP ########################################################################
# ## OCP ########################################################################

robot = RobotWrapper(simu.rmodel,contactKey='sole_link')
walkParams = WalkParams()
assert len(walkParams.stateImportance) == robot.model.nv * 2

assert(norm(robot.x0-simu.getState())<1e-6)

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
ddp = walk.buildSolver(robot,contactPattern,walkParams)
problem = ddp.problem
x0s,u0s = walk.buildInitialGuess(ddp.problem,walkParams)
ddp.setCallbacks([croc.CallbackVerbose()])
ddp.solve(x0s, u0s, 200)

mpc = WalkMPC(robot,ddp.problem,walkParams,xs_init=ddp.xs,us_init=ddp.us)

# #####################################################################################
# ### VIZ #############################################################################
# #####################################################################################
try:
    Viz = pin.visualize.GepettoVisualizer
    viz = Viz(simu.rmodel, simu.gmodel_col,simu.gmodel_vis)
    viz.initViewer()
    viz.loadViewerModel()
    gv = viz.viewer.gui
    viz.display(simu.getState()[:robot.model.nq])
except (ImportError,AttributeError):
    print("No viewer")

# ## MAIN LOOP ##################################################################

hx = []
hu = []

# FOR LOOP
for s in range(1000):


    # ###############################################################################
    # # For timesteps without MPC updates
    for k in range(int(walkParams.DT / 1e-3)):
        # Get simulation state
        x = simu.getState()

        # Compute Ricatti feedback
        torques = mpc.solver.us[0] + mpc.solver.K[0] @ (mpc.state.diff(x, mpc.solver.xs[0]))

        # Run one step of simu
        simu.step(torques)

    # ###############################################################################
    #mpc.run(mpc.solver.xs[1],s)
    mpc.run(simu.getState(),s)
    viz.display(simu.getState()[:robot.model.nq])

    '''
    for t in range(1, 500):
        x = mpc.solver.xs[1]
    mpc.run(x,t)

    if not t % 10:
        viz.display(x[: robot.model.nq])
        time.sleep(walkParams.DT)
    '''
if walkParams.saveFile is not None:
    save_traj(np.array(hx),filename=walkParams.saveFile)
