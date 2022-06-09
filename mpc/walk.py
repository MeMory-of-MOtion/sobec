import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pyplot as plt

# plt.ion()
from numpy.linalg import norm

# Local imports
import talos_low
from weight_share import computeReferenceForces
import sobec

from params import (
    flyWeight,
    forceImportance,
    coneAxisWeight,
    impactAltitudeWeight,
    verticalFootVelWeight,
    flyHighSlope,
    footMinimalDistance,
    STATE_WEIGHT,
    refStateWeight,
    CONTROL_WEIGHT,
    refTorqueWeight,
    VCOM_TARGET,
    vcomWeight,
    copWeight,
    conePenaltyWeight,
    terminalNoVelocityWeight,
)

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)

# ## HYPER PARAMETERS
# Hyperparameters defining the optimal control problem.

DT = 0.010
X_TARGET = 0.35
FOOT_SIZE = 0.05

# ## LOAD AND DISPLAY SOLO
# Load the robot model from example robot data and display it if possible in
# Gepetto-viewer
robot = talos_low.load()

contactIds = [i for i, f in enumerate(robot.model.frames) if "sole_link" in f.name]
ankleToTow = 0.1
ankleToHeel = -0.1
for cid in contactIds:
    f = robot.model.frames[cid]
    robot.model.addFrame(
        pin.Frame(
            f"{f.name}_tow",
            f.parent,
            f.previousFrame,
            f.placement * pin.SE3(np.eye(3), np.array([ankleToTow, 0, 0])),
            pin.FrameType.OP_FRAME,
        )
    )
    robot.model.addFrame(
        pin.Frame(
            f"{f.name}_heel",
            f.parent,
            f.previousFrame,
            f.placement * pin.SE3(np.eye(3), np.array([ankleToHeel, 0, 0])),
            pin.FrameType.OP_FRAME,
        )
    )

try:
    viz = pin.visualize.GepettoVisualizer(
        robot.model, robot.collision_model, robot.visual_model
    )
    viz.initViewer()
    viz.loadViewerModel()
    gv = viz.viewer.gui
except ImportError:
    print("No viewer")

# The pinocchio model is what we are really interested by.
model = robot.model
model.q0 = robot.q0
data = model.createData()

# Initial config, also used for warm start
x0 = np.concatenate([model.q0, np.zeros(model.nv)])

# Some key elements of the model
towIds = {idf: model.getFrameId(f"{model.frames[idf].name}_tow") for idf in contactIds}
heelIds = {
    idf: model.getFrameId(f"{model.frames[idf].name}_heel") for idf in contactIds
}
baseId = model.getFrameId("root_joint")
robotweight = -sum(Y.mass for Y in model.inertias) * model.gravity.linear[2]
com0 = pin.centerOfMass(model, data, model.q0)

pin.framesForwardKinematics(model, data, x0[: model.nq])

# #####################################################################################
# ## TUNING ###########################################################################
# #####################################################################################

# In the code, cost terms with 0 weight are commented for reducing execution cost
# An example of working weight value is then given as comment at the end of the line.
# When setting them to >0, take care to uncomment the corresponding line.
# All these lines are marked with the tag ##0##.


assert len(STATE_WEIGHT) == model.nv * 2

kktDamping = 0  # 1e-6
baumgartGains = np.array([0, 0])  # ##X## 50

# Contact are specified with the order chosen in <contactIds>
contactPattern = (
    []
    + [[1, 1]] * 30
    + [[1, 0]] * 50
    + [[1, 1]] * 11
    + [[0, 1]] * 50
    + [[1, 1]] * 50
    + [[1, 1]]
)
T = len(contactPattern) - 1


def patternToId(pattern):
    """Return the tuple of active contact from a pattern like [0,1], [1,0] or [1,1]."""
    return tuple(contactIds[i] for i, c in enumerate(pattern) if c == 1)


# ### REF FORCES ######################################################################
# The force costs are defined using a reference (smooth) force.

# # Search the contact phase of minimal duration (typically double support)
# contactState=[]
# dur=mindur=len(contactPattern)
# for t,s in enumerate(contactPattern):
#     dur+=1
#     if s!=contactState:
#         contactState=s
#         mindur=min(mindur,dur)
#         dur=0
# # Select the smoothing transition to be smaller than half of the minimal duration.
# transDuration=(mindur-1)//2
# # Compute contact importance, ie how much of the weight should be supported by each
# # foot at each time.
# contactImportance = weightShareSmoothProfile(contactPattern,transDuration,switch=switch_linear)
# # Contact reference forces are set to contactimportance*weight
# weightReaction = np.array([0,0,robotweight,0,0,0])
# referenceForces = [ 
#     [ weightReaction*contactImportance[t,cid] for cid,__c in enumerate(pattern) ]
#       for t,pattern in enumerate(contactPattern) ]
# # Take care, we suppose here that foot normal is vertical.

referenceForces = computeReferenceForces(contactPattern,robotweight)

# #####################################################################################
# #####################################################################################
# #####################################################################################


models = []

# #####################################################################################
for t, pattern in enumerate(contactPattern[:-1]):
    # print(f'time t={t}')

    # Basics
    state = croc.StateMultibody(model)
    actuation = croc.ActuationModelFloatingBase(state)

    # Contacts
    contacts = croc.ContactModelMultiple(state, actuation.nu)
    for cid in patternToId(pattern):
        contact = croc.ContactModel6D(
            state, cid, pin.SE3.Identity(), actuation.nu, baumgartGains
        )
        contacts.addContact(model.frames[cid].name + "_contact", contact)

    # Costs
    costs = croc.CostModelSum(state, actuation.nu)

    xRegResidual = croc.ResidualModelState(state, x0, actuation.nu)
    xRegCost = croc.CostModelResidual(
        state, croc.ActivationModelWeightedQuad(STATE_WEIGHT**2), xRegResidual
    )
    costs.addCost("stateReg", xRegCost, refStateWeight)

    uResidual = croc.ResidualModelControl(state, actuation.nu)
    uRegCost = croc.CostModelResidual(
        state,
        croc.ActivationModelWeightedQuad(np.array(CONTROL_WEIGHT**2)),
        uResidual,
    )
    costs.addCost("ctrlReg", uRegCost, refTorqueWeight)

    comVelResidual = sobec.ResidualModelCoMVelocity(state, VCOM_TARGET, actuation.nu)
    comVelCost = croc.CostModelResidual(state, comVelResidual)
    costs.addCost("comVelCost", comVelCost, vcomWeight)

    # Contact costs
    #for cid in patternToId(pattern):
    for forceIndex,activation in enumerate(contactPattern[t]):
        if not activation: continue
        cid = contactIds[forceIndex]
        
        copResidual = sobec.ResidualModelCenterOfPressure(state,cid,actuation.nu)
        copAct = croc.ActivationModelWeightedQuad(np.array([ 1/FOOT_SIZE**2 ]*2))
        copCost = croc.CostModelResidual( state,copAct,copResidual)
        costs.addCost(f'{model.frames[cid].name}_cop',copCost,copWeight)

        # Cone with enormous friction (Assuming the robot will barely ever slide).
        # FOOT_SIZE is the allowed area size, while cone expects the corner coordinates
        # => x2
        cone = croc.WrenchCone(
            np.eye(3), 1000, np.array([FOOT_SIZE * 2] * 2), 4, True, 1, 10000
        )
        coneCost = croc.ResidualModelContactWrenchCone(state, cid, cone, actuation.nu)
        ub = cone.ub.copy()
        ub[:4] = np.inf
        # ub[5:] = np.inf  ### DEBUG
        ub[-8:] = np.inf
        coneAct = croc.ActivationModelQuadraticBarrier(
            croc.ActivationBounds(cone.lb, ub)
        )
        coneCost = croc.CostModelResidual(state, coneAct, coneCost)
        costs.addCost(f"{model.frames[cid].name}_cone", coneCost, conePenaltyWeight)

        # Penalize the distance to the central axis of the cone ...
        #  ... using normalization weights depending on the axis.
        # The weights are squared to match the tuning of the CASADI formulation.
        coneAxisResidual = croc.ResidualModelContactForce(
            state, cid, pin.Force.Zero(), 6, actuation.nu
        )
        w = np.array(forceImportance**2)
        w[2] = 0
        coneAxisAct = croc.ActivationModelWeightedQuad(w)
        coneAxisCost = croc.CostModelResidual(state, coneAxisAct, coneAxisResidual)
        costs.addCost(
            f"{model.frames[cid].name}_coneaxis", coneAxisCost, coneAxisWeight
        )

        # Follow reference (smooth) contact forces
        forceRefResidual = croc.ResidualModelContactForce(state,cid,pin.Force(referenceForces[t][forceIndex]),6,actuation.nu)
        forceRefCost = croc.CostModelResidual(state,forceRefResidual)
        costs.addCost(f'{model.frames[cid].name}_forceref',forceRefCost,refForceWeight/robotweight**2)
        
    # IMPACT
    for k,cid in enumerate(contactIds):
        if t>0 and not contactPattern[t-1][k] and pattern[k]:
            # REMEMBER TO divide the weight by DT, as impact should be independant of the node duration
            # (at least, that s how weights are tuned in casadi).
            
            print(f'Impact {cid} at time {t}')
            impactResidual = croc.ResidualModelFrameTranslation(state,cid,np.zeros(3),actuation.nu)
            impactAct = croc.ActivationModelWeightedQuad(np.array([0,0,1]))
            impactCost = croc.CostModelResidual(state,impactAct,impactResidual)
            costs.addCost(f'{model.frames[cid].name}_atitudeimpact',impactCost,impactAltitudeWeight/DT)

            impactVelResidual = croc.ResidualModelFrameVelocity(state,cid,pin.Motion.Zero(),pin.ReferenceFrame.LOCAL,actuation.nu)
            impactVelCost = croc.CostModelResidual(state,impactVelResidual)
            costs.addCost(f'{model.frames[cid].name}_velimpact',impactVelCost,impactVelocityWeight/DT)
            
    # Flying foot
    for k, cid in enumerate(contactIds):
        if pattern[k]:
            continue
        verticalFootVelResidual = croc.ResidualModelFrameVelocity(
            state,
            cid,
            pin.Motion.Zero(),
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            actuation.nu,
        )
        verticalFootVelAct = croc.ActivationModelWeightedQuad(
            np.array([0, 0, 1, 0, 0, 0])
        )
        verticalFootVelCost = croc.CostModelResidual(
            state, verticalFootVelAct, verticalFootVelResidual
        )
        costs.addCost(
            f"{model.frames[cid].name}_vfoot_vel",
            verticalFootVelCost,
            verticalFootVelWeight,
        )

        # Slope is /2 since it is squared in casadi (je me comprends)
        flyHighResidual = sobec.ResidualModelFlyHigh(
            state, cid, flyHighSlope / 2, actuation.nu
        )
        flyHighCost = croc.CostModelResidual(state, flyHighResidual)
        costs.addCost(f"{model.frames[cid].name}_flyhigh", flyHighCost, flyWeight)

        groundColRes = croc.ResidualModelFrameTranslation(state,cid,np.zeros(3),actuation.nu)
        #groundColBounds = croc.ActivationBounds(np.array([-np.inf,-np.inf,0.01]),np.array([np.inf,np.inf,np.inf]))
        # np.inf introduces an error on lb[2] ... why? TODO ... patch by replacing np.inf with 1000
        groundColBounds = croc.ActivationBounds(np.array([-1000,-1000,0.0]),np.array([1000,1000,1000]))
        groundColAct = croc.ActivationModelQuadraticBarrier(groundColBounds)
        groundColCost = croc.CostModelResidual(state,groundColAct,groundColRes)
        costs.addCost(f'{model.frames[cid].name}_groundcol',groundColCost,groundColWeight)
            
    # Action
    damodel = croc.DifferentialActionModelContactFwdDynamics(
        state, actuation, contacts, costs, kktDamping, True
    )
    amodel = croc.IntegratedActionModelEuler(damodel, DT)

    models.append(amodel)
    # stophere

# #####################################################################################
pattern = contactPattern[-1]

state = croc.StateMultibody(model)
actuation = croc.ActuationModelFloatingBase(state)

# Contacts
contacts = croc.ContactModelMultiple(state, actuation.nu)
for cid in patternToId(pattern):
    contact = croc.ContactModel6D(
        state, cid, pin.SE3.Identity(), actuation.nu, baumgartGains
    )
    contacts.addContact(model.frames[cid].name + "_contact", contact)

# Costs
costs = croc.CostModelSum(state, actuation.nu)

xRegResidual = croc.ResidualModelState(state, x0, actuation.nu)
xRegCost = croc.CostModelResidual(
    state,
    croc.ActivationModelWeightedQuad(np.array([0] * model.nv + [1] * model.nv)),
    xRegResidual,
)
costs.addCost("stateReg", xRegCost, terminalNoVelocityWeight)

damodel = croc.DifferentialActionModelContactFwdDynamics(
    state, actuation, contacts, costs, kktDamping, True
)
termmodel = croc.IntegratedActionModelEuler(damodel, DT)

# #####################################################################################
GUESS_FILE = "/tmp/sol.npy"
guess = np.load(GUESS_FILE, allow_pickle=True)[()]
print(f'Load "{GUESS_FILE}"!')
# #####################################################################################

problem = croc.ShootingProblem(x0, models, termmodel)
ddp = croc.SolverFDDP(problem)
x0s = [x0.copy()] * (len(models) + 1)
u0s = [
    m.quasiStatic(d, x)
    for m, d, x in zip(problem.runningModels, problem.runningDatas, x0s)
]
ddp.setCallbacks([croc.CallbackVerbose()])

x0s = [x for x in guess["xs"]]
u0s = [u for u in guess["us"]]

# l = pin.StdVec_Double()
# l.append(2)
# ddp.alphas = l

ddp.th_acceptStep = 0.0000001
ddp.solve(x0s, u0s, 20000)

dmodel = problem.runningModels[0].differential
ddata = problem.runningDatas[0].differential

contactmodell = dmodel.contacts.contacts["left_sole_link_contact"].contact
contactmodelr = dmodel.contacts.contacts["right_sole_link_contact"].contact
contactdatal = ddata.multibody.contacts.contacts["left_sole_link_contact"]
contactdatar = ddata.multibody.contacts.contacts["right_sole_link_contact"]

copcostmodell = dmodel.costs.costs["left_sole_link_cop"].cost
copcostmodelr = dmodel.costs.costs["right_sole_link_cop"].cost
copcostdatal = ddata.costs.costs["left_sole_link_cop"]
copcostdatar = ddata.costs.costs["right_sole_link_cop"]

# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

xs_sol = np.array(ddp.xs)
us_sol = np.array(ddp.us)
acs_sol = np.array([d.differential.xout for d in problem.runningDatas])
fs_sol = [
    [
        (cd.data().jMf.inverse() * cd.data().f).vector
        for cd in d.differential.multibody.contacts.contacts
    ]
    for d in problem.runningDatas
]
fs_sol0 = [
    np.concatenate(
        [
            (
                d.differential.multibody.contacts.contacts[
                    f"{model.frames[cid].name}_contact"
                ].jMf.inverse()
                * d.differential.multibody.contacts.contacts[
                    f"{model.frames[cid].name}_contact"
                ].f
            ).vector
            if f"{model.frames[cid].name}_contact"
            in d.differential.multibody.contacts.contacts
            else np.zeros(6)
            for cid in contactIds
        ]
    )
    for m, d in zip(problem.runningModels, problem.runningDatas)
]

# Robot basis movement
legend = ["x", "y", "z"]
plt.figure("Basis move")
for i in range(3):
    plt.subplot(3, 1, i + 1)
    plt.title("Base link position_" + legend[i])
    plt.plot(xs_sol[:, i])
    if i == 0:
        plt.axhline(y=X_TARGET, color="black", linestyle="--")

# Cop of each foot vs time
plt.figure("cop time local")
for ifig, cid in enumerate(contactIds):
    plt.subplot(len(contactIds), 1, ifig + 1)
    ftraj = [
        [t, f[6 * ifig : 6 * ifig + 6]]
        for t, (f, p) in enumerate(zip(fs_sol0, contactPattern))
        if cid in patternToId(p)
    ]
    cop = [[t, [f[4] / f[2], -f[3] / f[2]]] for (t, f) in ftraj]
    plt.plot([t for t, p in cop], [p for t, p in cop], ".")

# Cop of each foot in x-vs-y (with limits)
plt.figure(figsize=(12, 6))
plt.title("cop local")
l_foot = np.array(
    [
        [-FOOT_SIZE, -FOOT_SIZE, 0, 1],
        [-FOOT_SIZE, FOOT_SIZE, 0, 1],
        [FOOT_SIZE, FOOT_SIZE, 0, 1],
        [FOOT_SIZE, -FOOT_SIZE, 0, 1],
        [-FOOT_SIZE, -FOOT_SIZE, 0, 1],
    ]
).T
for ifig, cid in enumerate(contactIds):
    plt.subplot(1, len(contactIds), ifig + 1)
    ARENA_SIZE = 0.6
    plt.axis([-ARENA_SIZE / 4, ARENA_SIZE * 3 / 4, -ARENA_SIZE / 2, ARENA_SIZE / 2])
    plt.xlabel(model.frames[cid].name)
    for t, pattern in enumerate(contactPattern[:-1]):
        if cid not in patternToId(pattern):
            continue
        f = fs_sol0[t][6 * ifig : 6 * ifig + 6]
        l_cop = np.array([f[4] / f[2], -f[3] / f[2], 0])
        pin.framesForwardKinematics(model, data, xs_sol[t][: model.nq])
        w_cop = data.oMf[cid] * l_cop
        plt.plot(w_cop[0], w_cop[1], "r.")
        w_foot = data.oMf[cid].homogeneous @ l_foot
        plt.plot(w_foot[0, :], w_foot[1, :], "grey")

# Forces and reference forces wrt time
plt.figure("forces")
frefplot = np.array([np.concatenate(fs) for fs in referenceForces])
fs0plot = np.array(fs_sol0)
plt.subplot(211)
plt.plot(fs0plot[:, 2])
plt.plot(frefplot[:, 2])
plt.xlabel(model.frames[contactIds[0]].name)
plt.subplot(212)
plt.plot(fs0plot[:, 8])
plt.plot(frefplot[:, 8])
plt.xlabel(model.frames[contactIds[1]].name)

# COM position and velocity (x+y separated from z)
plt.figure("com", figsize=(6, 8))
complot = []
vcomplot = []
for x in xs_sol:
    pin.centerOfMass(model, data, x[: model.nq], x[model.nq :])
    complot.append(data.com[0].copy())
    vcomplot.append(data.vcom[0].copy())
complot = np.array(complot)
vcomplot = np.array(vcomplot)
plt.subplot(411)
plt.plot(complot[:, :2])
plt.ylabel("pos x-y")
plt.subplot(412)
plt.plot(complot[:, 2])
plt.ylabel("pos z")
ax = plt.axis()
plt.axis(
    (ax[0], ax[1], com0[2] - 2.5e-2, com0[2] + 2.5e-2)
)  # yaxis is 5cm around 0 position
plt.subplot(413)
plt.plot(vcomplot[:, :2])
plt.ylabel("vel x-y")
plt.legend(["x", "y"])
plt.subplot(414)
plt.plot(vcomplot[:, 2])
plt.ylabel("vel z")

# Foot position and velocity
plt.figure("foot")
foottraj = []
footvtraj = []
for x in xs_sol:
    pin.forwardKinematics(model, data, x[: model.nq], x[model.nq :])
    pin.updateFramePlacements(model, data)
    foottraj.append(np.concatenate([data.oMf[cid].translation for cid in contactIds]))
    footvtraj.append(
        np.concatenate(
            [pin.getFrameVelocity(model, data, cid).vector for cid in contactIds]
        )
    )
foottraj = np.array(foottraj)
footvtraj = np.array(footvtraj)
plt.subplot(311)
hplot = []
names = []
for i, cid in enumerate(contactIds):
    hplot.extend(plt.plot(foottraj[:, 3 * i + 2]))
    names.append(model.frames[cid].name)
plt.legend(hplot, names)
plt.ylabel("altitude")
plt.subplot(313)
hplot = []
for i, cid in enumerate(contactIds):
    hplot.extend(plt.plot(foottraj[:, 3 * i], foottraj[:, 3 * i + 2]))
plt.legend(hplot, names)
plt.ylabel("x-z traj")
plt.subplot(312)
hplot = []
for i, cid in enumerate(contactIds):
    hplot.extend(plt.plot(np.sqrt(np.sum(footvtraj[:, 6 * i : 6 * i + 2] ** 2, 1))))
plt.legend(hplot, names)
plt.ylabel("horz vel")

plt.figure("foot collision")
h1 = plt.plot([f[0] for f in foottraj], [f[1] for f in foottraj])
h2 = plt.plot([f[3] for f in foottraj], [f[4] for f in foottraj])
plt.legend(h1 + h2, ["left", "right"])
for t in range(T):
    a = foottraj[t][:2]
    b = foottraj[t][3:5]
    m = (a + b) / 2
    d = b - a
    d /= norm(d)
    aa = m + d * footMinimalDistance / 2
    bb = m - d * footMinimalDistance / 2
    plt.plot([aa[0], bb[0]], [aa[1], bb[1]], "grey")
plt.axis([-0.1, 0.4, -0.25, 0.25])


# ### SAVE #####################################################################
def save():
    SOLU_FILE = "/tmp/ddp.npy"
    np.save(
        open(SOLU_FILE, "wb"),
        {
            "xs": xs_sol,
            "us": us_sol,
            "acs": acs_sol,
            "fs": np.array(fs_sol0),
        },
    )
    print(f'Save "{SOLU_FILE}"!')


"""
-B < tau/f < B
tau < Bf
Bf - tau >=0
tau >= -Bf
Bf + tau >= 0
"""

# ## DEBUG

# dmodel = problem.runningModels[0].differential
# ddata = problem.runningDatas[0].differential

# contactmodell = dmodel.contacts.contacts['left_sole_link_contact'].contact
# contactmodelr = dmodel.contacts.contacts['right_sole_link_contact'].contact
# contactdatal = ddata.multibody.contacts.contacts['left_sole_link_contact']
# contactdatar = ddata.multibody.contacts.contacts['right_sole_link_contact']

# copcostmodell = dmodel.costs.costs['left_sole_link_cop'].cost
# copcostmodelr = dmodel.costs.costs['right_sole_link_cop'].cost
# copcostdatal = ddata.costs.costs['left_sole_link_cop']
# copcostdatar = ddata.costs.costs['right_sole_link_cop']

t = 60; fid=48
t=119; fid=34
t=80; cid=48 # impact
xs=guess['xs']
us=guess['us']
fs0=guess['fs']
acs=guess['acs']
dadata=problem.runningDatas[t].differential
damodel=problem.runningModels[t].differential
damodel.calc(dadata,xs[t],us[t])
damodel.calcDiff(dadata,xs[t],us[t])
cosname='left_sole_link_cone'
cosname='right_sole_link_cone'
#cosname='altitudeImpact'
cosname='right_sole_link_vfoot_vel' # t = 60
cosname='right_sole_link_flyhigh'
cosname=f'{model.frames[fid].name}_flyhigh'
cosname=f'{model.frames[fid].name}_groundcol'
cosname=f'{model.frames[cid].name}_velimpact'
cosdata = dadata.costs.costs[cosname]
cosmodel = damodel.costs.costs[cosname].cost
np.set_printoptions(precision=6, linewidth=300, suppress=True,threshold=10000)
