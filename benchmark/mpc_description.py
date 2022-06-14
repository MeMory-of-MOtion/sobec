import crocoddyl as croc
import numpy as np
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import example_robot_data as robex

# Local imports
import sobec
from sobec.walk.robot_wrapper import RobotWrapper
from sobec.walk import ocp

# from mpc_params import WalkParams
from sobec.walk.config_mpc import configureMPCWalk
from sobec.walk.miscdisp import CallbackMPCWalk
from sobec.walk.params import WalkParams as WalkDefaultParams

# #####################################################################################
# ### LOAD ROBOT ######################################################################
# #####################################################################################

# ## LOAD AND DISPLAY TALOS
# Load the robot model from example robot data and display it if possible in
# Gepetto-viewer
urdf = robex.load("talos_legs")
urdf.model.name = 'talos'
robot = RobotWrapper(urdf.model, contactKey="sole_link")

# #####################################################################################
# ## TUNING ###########################################################################
# #####################################################################################

# In the code, cost terms with 0 weight are commented for reducing execution cost
# An example of working weight value is then given as comment at the end of the line.
# When setting them to >0, take care to uncomment the corresponding line.
# All these lines are marked with the tag ##0##.


class WalkParams(WalkDefaultParams):
    pass

walkParams = WalkParams(robot.name)
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
        croc.CallbackVerbose(),
    ]
)

ddp.solve(x0s, u0s, 200)

# ### MPC #############################################################################
# ### MPC #############################################################################
# ### MPC #############################################################################

mpc = sobec.MPCWalk(ddp.problem)
configureMPCWalk(mpc, walkParams)
mpc.initialize(ddp.xs[: walkParams.Tmpc + 1], ddp.us[: walkParams.Tmpc])
mpc.solver.setCallbacks(
    [
        # croc.CallbackVerbose(),
        CallbackMPCWalk(robot.contactIds)
    ]
)


'''
if __name__ == "__main__":
    print("*** MAIN ***")
    x = robot.x0.copy()
    for t in range(1, 100):
        mpc.calc(x, t)
        x = mpc.solver.xs[1]
'''
