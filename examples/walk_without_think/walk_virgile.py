import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

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

walkParams = specific_params.WalkBattobotParams()

# #####################################################################################
# ### LOAD ROBOT ######################################################################
# #####################################################################################

# ## LOAD AND DISPLAY TALOS
# Load the robot model from example robot data and display it if possible in
# Gepetto-viewer

#urdf = sobec.talos_collections.robexLoadAndReduce("talos", walkParams.robotName)
urdffile= "robot.urdf"
urdfpath = "model_robot_virgile/model_simplified"
urdf = pin.RobotWrapper.BuildFromURDF(urdfpath + "/" + urdffile,urdfpath,
                                      root_joint=pin.JointModelFreeFlyer())
# urdf.q0 = pin.neutral(urdf.model)
# urdf.q0[2] = +0.5507357853479324 ## So that the feet are at z=0

# Optimized with compute_init_config_virgile (pin3) so that:
# - both foot flat on the ground
# - COM in between the two feet
# - torso at 0 orientation
urdf.q0 = np.array([ 0.085858,  0.000065,  0.570089,  0.      ,  0.      ,  1.      ,  0.      , -0.      , -0.00009 , -0.208644, -0.043389,  0.252034, -0.00009 ,  0.      , -0.00009 ,  0.208644, -0.043389,  0.252034, -0.00009 ])

urdf.model.referenceConfigurations['half_sitting'] = urdf.q0.copy()
# robot = sobec.wwt.RobotWrapper(urdf.model, contactKey="ankle_x")
robot = sobec.wwt.RobotWrapper(urdf.model, contactKey="foot_frame")
assert len(walkParams.stateImportance) == robot.model.nv * 2

# #####################################################################################
# ### CONTACT PATTERN #################################################################
# #####################################################################################
try:
    # If possible, the initial state and contact pattern are taken from a file.
    ocpConfig = sobec.wwt.loadProblemConfig()
    contactPattern = ocpConfig["contactPattern"]
    robot.x0 = ocpConfig["x0"]
    stateTerminalTarget = ocpConfig["stateTerminalTarget"]
except (KeyError, FileNotFoundError):
    # When the config file is not found ...
    # Initial config, also used for warm start, both taken from robot wrapper.
    # Contact are specified with the order chosen in <contactIds>.
    cycle = ( [[1, 0]] * walkParams.Tsingle
              + [[1, 1]] * walkParams.Tdouble
              + [[0, 1]] * walkParams.Tsingle
              + [[1, 1]] * walkParams.Tdouble
             )
    contactPattern = (
        []
        + [[1, 1]] * walkParams.Tstart
        + (cycle * 1)
        + [[1, 1]] * walkParams.Tend
        + [[1, 1]]
    )

# #####################################################################################
# ### VIZ #############################################################################
# #####################################################################################
try:
    import meshcat
    from pinocchio.visualize import MeshcatVisualizer
    viz = MeshcatVisualizer(urdf.model, urdf.collision_model, urdf.visual_model)
    server = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    # server = None
    viz.initViewer(loadModel=True, viewer=server)
except (ImportError, AttributeError):
    print("No viewer")


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

ddp = sobec.wwt.buildSolver(robot, contactPattern, walkParams)
problem = ddp.problem
x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose(), croc.CallbackLogger()])

with open("/tmp/virgile-repr.ascii", "w") as f:
    f.write(sobec.reprProblem(ddp.problem))
    print("OCP described in /tmp/virgile-repr.ascii")

croc.enable_profiler()
ddp.solve(x0s, u0s, 200)

# assert sobec.logs.checkGitRefs(ddp.getCallbacks()[1], "refs/virgile-logs.npy")

# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

sol = sobec.wwt.Solution(robot, ddp)

plotter = sobec.wwt.plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, sol.xs, sol.us, sol.fs0)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference
forceRef = [
    sobec.wwt.plotter.getReferenceForcesFromProblemModels(problem, cid)
    for cid in robot.contactIds
]
forceRef = [np.concatenate(fs) for fs in zip(*forceRef)]

plotter.plotBasis(target)
plotter.plotTimeCop()
plotter.plotCopAndFeet(walkParams.footSize, 0.6)
plotter.plotForces(forceRef)
plotter.plotCom(robot.com0)
plotter.plotFeet()
plotter.plotFootCollision(walkParams.footMinimalDistance)
print("Run ```plt.ion(); plt.show()``` to display the plots.")
plt.show()
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)

viz.play(np.array(ddp.xs)[:, : robot.model.nq], walkParams.DT)

'''
for x in ddp.xs:
    viz.display(x[:robot.model.nq])
    ims.append( viz.viewer.get_image())
import imageio # pip install imageio[ffmpeg]
imageio.mimsave("/tmp/battobot.mp4", imgs, 1//walkParams.DT)
'''
