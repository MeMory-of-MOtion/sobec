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

walkParams = specific_params.JumpBattobotParams()

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
#urdf.q0 = np.array([ 0.085858,  0.000065,  0.570089,  0.      ,  0.      ,  1.      ,  0.      , -0.      , -0.00009 , -0.208644, -0.043389,  0.252034, -0.00009 ,  0.      , -0.00009 ,  0.208644, -0.043389,  0.252034, -0.00009 ])
urdf.q0 = np.array([ 0.177111, -0.117173,  0.692838,  0.      ,  0.      , -0.      ,  1.      ,  0.      , -0.000377,  1.203784,  0.807122, -0.396662,  0.000377,  0.      , -0.000377, -1.204769, -0.81081 ,  0.39396 ,  0.000377])

urdf.model.referenceConfigurations['half_sitting'] = urdf.q0.copy()
# robot = sobec.wwt.RobotWrapper(urdf.model, contactKey="ankle_x")
robot = sobec.wwt.RobotWrapper(urdf.model, contactKey="foot_frame")
assert len(walkParams.stateImportance) == robot.model.nv * 2

# #####################################################################################
# ### CONTACT PATTERN #################################################################
# #####################################################################################

Tpush = 25
Tflight = 28
Tland = 25


cycle = ( [[1, 1]] * Tpush
          + [[0, 0]] * Tflight
          + [[1, 1]] * Tland
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
    viz.display(robot.x0[:robot.model.nq])
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


# #####################################################################################
# TWEAK
# #####################################################################################

Tmid = walkParams.Tstart+Tpush+Tflight//2
# href = .5 # elevation reference
# vref = 4*href/Tflight # take off vel reference
# alpha = 1+vref/(9.8*Tpush)
DT=walkParams.DT
v0 = 9.81*Tflight/2*DT # velocity to arrive at Tmid with 0 vel
href = v0*Tflight*DT/4 # gain in elevation with v0 at take off
alpha = 1+v0/(9.81*Tpush*DT)

cost = problem.runningModels[Tmid].differential.costs.costs['stateReg'].cost
w = np.array(cost.activation.weights)
w[2] = 10000
cost.activation.weights = w
r =np.array(cost.residual.reference)
#r[2] = 3.5
r[2] += href
cost.residual.reference = r


for t in range(walkParams.Tstart,walkParams.Tstart+Tpush):
    for n in ['left_foot_frame_forceref', 'right_foot_frame_forceref']:
        problem.runningModels[t].differential.costs.costs[n].weight *= 20
        cost = problem.runningModels[t].differential.costs.costs[n].cost
        mg = cost.residual.reference.linear[2]*2
        cost.residual.reference.linear[2] = alpha/2*mg
        
alpha = 1+v0/(9.81*Tland*DT)
for t in range(walkParams.Tstart+Tpush+Tflight,walkParams.Tstart+Tpush+Tflight+Tland):
    for n in ['left_foot_frame_forceref', 'right_foot_frame_forceref']:
        problem.runningModels[t].differential.costs.costs[n].weight *= 10
        cost = problem.runningModels[t].differential.costs.costs[n].cost
        mg = cost.residual.reference.linear[2]*2
        cost.residual.reference.linear[2] = alpha/2*mg
    

# #####################################################################################
# SOLVE
# #####################################################################################

with open("/tmp/jump-virgile-repr.ascii", "w") as f:
    f.write(sobec.reprProblem(ddp.problem))
    print("OCP described in /tmp/jump-virgile-repr.ascii")

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

# plotter.plotBasis(target)
# plotter.plotTimeCop()
# plotter.plotCopAndFeet(walkParams.footSize, [0,1.2,-.3,.3])
plotter.plotForces(forceRef)
# plotter.plotCom(robot.com0)
# plotter.plotFeet()
# plotter.plotFootCollision(walkParams.footMinimalDistance)
# plotter.plotJointTorques()
# plotter.plotComAndCopInXY()
print("Run ```plt.ion(); plt.show()``` to display the plots.")
#plt.ion(); plt.show()
plt.show()
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)

def play():
    viz.play(np.array(ddp.xs)[:, : robot.model.nq], walkParams.DT)

def recordVideo():
    from tqdm import tqdm
    ims = []
    for x in tqdm(ddp.xs):
        viz.display(x[:robot.model.nq])
        ims.append( viz.viewer.get_image())
        import imageio # pip install imageio[ffmpeg]
        imageio.mimsave(f"/tmp/jump_battobot.mp4", [np.array(i) for i in ims],fps=int(1//walkParams.DT))

