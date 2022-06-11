import pinocchio as pin
import crocoddyl as croc
import numpy as np
import example_robot_data as robex
import matplotlib.pyplot as plt
from numpy.linalg import norm,inv,pinv,svd,eig
import time
# Local imports
import sobec
import talos_low
from weight_share import computeReferenceForces
from save_traj import save_traj,saveProblemConfig
from mpcparams import WalkParams
import miscdisp
pin.SE3.__repr__=pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True,threshold=10000)
# plt.ion()

### HYPER PARAMETERS
# Hyperparameters defining the optimal control problem.
T_START = 30
T_SINGLE = 50
T_DOUBLE = 11
T_END = 30

### LOAD AND DISPLAY TALOS
# Load the robot model from example robot data and display it if possible in Gepetto-viewer
robot = talos_low.load()

contactIds = [ i for i,f in enumerate(robot.model.frames) if "sole_link" in f.name ]
ankleToTow=0.1
ankleToHeel=-0.1
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
except AttributeError: 
    print("No viewer")

# The pinocchio model is what we are really interested by.
model = robot.model
model.q0 = robot.q0
data = model.createData()

# Initial config, also used for warm start
x0 = np.concatenate([model.q0, np.zeros(model.nv)])

# Some key elements of the model
towIds = {idf: model.getFrameId(f"{model.frames[idf].name}_tow") for idf in contactIds}
heelIds = {idf: model.getFrameId(f"{model.frames[idf].name}_heel") for idf in contactIds}
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

p = WalkParams()
assert(len(p.stateImportance)==model.nv*2)

# Contact are specified with the order chosen in <contactIds>
# contactPattern = [] \
#     + [ [ 1,1 ] ] * 40 \
#     + [ [ 1,0 ] ] * 50  \
#     + [ [ 1,1 ] ] * 11  \
#     + [ [ 0,1 ] ] * 50  \
#     + [ [ 1,1 ] ] * 11 \
#     + [ [ 1,1 ] ] * 40 \
#     + [ [ 1,1 ] ]
contactPattern = [] \
    + [ [ 1,1 ] ] * T_START \
    + [ [ 1,1 ] ] * T_DOUBLE \
    + [ [ 0,1 ] ] * T_SINGLE \
    + [ [ 1,1 ] ] * T_DOUBLE \
    + [ [ 1,0 ] ] * T_SINGLE \
    + [ [ 1,1 ] ] * T_DOUBLE \
    + [ [ 1,1 ] ] * T_END \
    + [ [ 1,1 ] ]
T = len(contactPattern)-1
    
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
    for k,cid in enumerate(contactIds):
        if not pattern[k]: continue
        contact = croc.ContactModel6D(state, cid, pin.SE3.Identity(), actuation.nu, p.baumgartGains)
        contacts.addContact(model.frames[cid].name + "_contact", contact)

    # Costs
    costs = croc.CostModelSum(state, actuation.nu)

    xRegResidual = croc.ResidualModelState(state, x0, actuation.nu)
    xRegCost = croc.CostModelResidual(
        state, croc.ActivationModelWeightedQuad(p.stateImportance**2), xRegResidual
    )
    costs.addCost("stateReg", xRegCost, p.refStateWeight)

    uResidual = croc.ResidualModelControl(state, actuation.nu)
    uRegCost = croc.CostModelResidual(
        state,
        croc.ActivationModelWeightedQuad(np.array(p.controlImportance**2)),
        uResidual,
    )
    costs.addCost("ctrlReg", uRegCost, p.refTorqueWeight)

    comVelResidual = sobec.ResidualModelCoMVelocity(state, p.VCOM_TARGET, actuation.nu)
    comVelAct = croc.ActivationModelWeightedQuad(np.array([0,0,1]))
    comVelCost = croc.CostModelResidual( state,comVelAct,comVelResidual )
    costs.addCost("comVelCost", comVelCost, p.vcomWeight)

    # Contact costs
    for k,cid in enumerate(contactIds):
        if not pattern[k]: continue
        
        copResidual = sobec.ResidualModelCenterOfPressure(state,cid,actuation.nu)
        copAct = croc.ActivationModelWeightedQuad(np.array([ 1/p.FOOT_SIZE**2 ]*2))
        copCost = croc.CostModelResidual( state,copAct,copResidual)
        costs.addCost(f'{model.frames[cid].name}_cop',copCost,p.copWeight)

        # Cone with enormous friction (Assuming the robot will barely ever slide).
        # p.FOOT_SIZE is the allowed area size, while cone expects the corner coordinates => x2
        cone = croc.WrenchCone(np.eye(3), 1000, np.array([ p.FOOT_SIZE*2 ]*2),4, True,1,10000)
        coneCost = croc.ResidualModelContactWrenchCone(state,cid,cone,actuation.nu)
        ub = cone.ub.copy()
        ub[:4] = np.inf
        # ub[5:] = np.inf  ### DEBUG
        ub[-8:] = np.inf
        coneAct = croc.ActivationModelQuadraticBarrier(croc.ActivationBounds(cone.lb,ub))
        coneCost = croc.CostModelResidual(state,coneAct,coneCost)
        costs.addCost(f'{model.frames[cid].name}_cone',coneCost,p.conePenaltyWeight)
        
        # Penalize the distance to the central axis of the cone ...
        #  ... using normalization weights depending on the axis.
        # The weights are squared to match the tuning of the CASADI formulation.
        coneAxisResidual = croc.ResidualModelContactForce(
            state, cid, pin.Force.Zero(), 6, actuation.nu
        )
        w = np.array(p.forceImportance**2)
        w[2] = 0
        coneAxisAct = croc.ActivationModelWeightedQuad(w)
        coneAxisCost = croc.CostModelResidual(state, coneAxisAct, coneAxisResidual)
        costs.addCost(f"{model.frames[cid].name}_coneaxis", coneAxisCost, p.coneAxisWeight)

        # Follow reference (smooth) contact forces
        forceRefResidual = croc.ResidualModelContactForce(state,cid,pin.Force(referenceForces[t][k]),6,actuation.nu)
        forceRefCost = croc.CostModelResidual(state,forceRefResidual)
        costs.addCost(f'{model.frames[cid].name}_forceref',forceRefCost,p.refForceWeight/robotweight**2)
        
    # IMPACT
    for k,cid in enumerate(contactIds):
        if t>0 and not contactPattern[t-1][k] and pattern[k]:
            # REMEMBER TO divide the weight by p.DT, as impact should be independant of the node duration
            # (at least, that s how weights are tuned in casadi).
            
            print(f'Impact {cid} at time {t}')
            impactResidual = croc.ResidualModelFrameTranslation(state,cid,np.zeros(3),actuation.nu)
            impactAct = croc.ActivationModelWeightedQuad(np.array([0,0,1]))
            impactCost = croc.CostModelResidual(state,impactAct,impactResidual)
            costs.addCost(f'{model.frames[cid].name}_altitudeimpact',impactCost,p.impactAltitudeWeight/p.DT)
            # if 'left' in model.frames[cid].name:
            #     itarget = np.array([0,.1,0])
            # else:
            #     itarget = np.array([0,-.1,0])
            # impactResidual = croc.ResidualModelFrameTranslation(state,cid,itarget,actuation.nu)
            # impactAct = croc.ActivationModelWeightedQuad(np.array([0,0,1]))
            # impactCost = croc.CostModelResidual(state,impactAct,impactResidual)
            # costs.addCost(f'{model.frames[cid].name}_altitudeimpact',impactCost,p.impactAltitudeWeight/p.DT)

            impactVelResidual = croc.ResidualModelFrameVelocity(state,cid,pin.Motion.Zero(),pin.ReferenceFrame.LOCAL,actuation.nu)
            impactVelCost = croc.CostModelResidual(state,impactVelResidual)
            costs.addCost(f'{model.frames[cid].name}_velimpact',impactVelCost,p.impactVelocityWeight/p.DT)

            impactRotResidual = croc.ResidualModelFrameRotation(state,cid,np.eye(3),actuation.nu)
            impactRotAct = croc.ActivationModelWeightedQuad(np.array([1,1,0]))
            impactRotCost = croc.CostModelResidual(state,impactRotAct,impactRotResidual)
            costs.addCost(f'{model.frames[cid].name}_rotimpact',impactRotCost,p.impactRotationWeight/p.DT)

            impactRefJointsResidual = croc.ResidualModelState(state,x0,actuation.nu)
            jselec = np.zeros(model.nv*2)
            jselec[[ model.joints[model.getJointId(name)].idx_v for name in p.MAIN_JOINTS ]]=1
            impactRefJointsAct = croc.ActivationModelWeightedQuad(jselec)
            impactRefJointCost = croc.CostModelResidual(state,impactRefJointsAct,impactRefJointsResidual)
            costs.addCost('impactRefJoint',impactRefJointCost,p.refMainJointsAtImpactWeight/p.DT)
            
            
    # Flying foot
    for k,cid in enumerate(contactIds):
        if pattern[k]: continue
        verticalFootVelResidual = croc.ResidualModelFrameVelocity(state,cid,pin.Motion.Zero(),
                                                                  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,actuation.nu)
        verticalFootVelAct = croc.ActivationModelWeightedQuad(np.array([0,0,1,0,0,0]))
        verticalFootVelCost = croc.CostModelResidual(state,verticalFootVelAct,verticalFootVelResidual)
        costs.addCost(f'{model.frames[cid].name}_vfoot_vel',verticalFootVelCost,p.verticalFootVelWeight)

        # Slope is /2 since it is squared in casadi (je me comprends)
        flyHighResidual = sobec.ResidualModelFlyHigh(state,cid,p.flyHighSlope/2,actuation.nu)
        flyHighCost = croc.CostModelResidual(state,flyHighResidual)
        costs.addCost(f'{model.frames[cid].name}_flyhigh',flyHighCost,p.flyWeight)

        groundColRes = croc.ResidualModelFrameTranslation(state,cid,np.zeros(3),actuation.nu)
        #groundColBounds = croc.ActivationBounds(np.array([-np.inf,-np.inf,0.01]),np.array([np.inf,np.inf,np.inf]))
        # np.inf introduces an error on lb[2] ... why? TODO ... patch by replacing np.inf with 1000
        groundColBounds = croc.ActivationBounds(np.array([-1000,-1000,0.0]),np.array([1000,1000,1000]))
        groundColAct = croc.ActivationModelQuadraticBarrier(groundColBounds)
        groundColCost = croc.CostModelResidual(state,groundColAct,groundColRes)
        costs.addCost(f'{model.frames[cid].name}_groundcol',groundColCost,p.groundColWeight)
            
    # Action
    damodel = croc.DifferentialActionModelContactFwdDynamics(
        state, actuation, contacts, costs, p.kktDamping, True
    )
    amodel = croc.IntegratedActionModelEuler(damodel, p.DT)

    models.append(amodel)

# ### TERMINAL MODEL ##################################################################
pattern = contactPattern[-1]

state = croc.StateMultibody(model)
actuation = croc.ActuationModelFloatingBase(state)

# Contacts
contacts = croc.ContactModelMultiple(state, actuation.nu)
for k,cid in enumerate(contactIds):
    if not pattern[k]: continue
    contact = croc.ContactModel6D(state, cid, pin.SE3.Identity(), actuation.nu, p.baumgartGains)
    contacts.addContact(model.frames[cid].name + "_contact", contact)

# Costs
costs = croc.CostModelSum(state, actuation.nu)

stateTerminalTarget = x0.copy()
stateTerminalTarget[:3] += p.VCOM_TARGET*T*p.DT
stateTerminalResidual = croc.ResidualModelState(state, stateTerminalTarget, actuation.nu)
stateTerminalAct = croc.ActivationModelWeightedQuad(p.stateTerminalImportance**2)
stateTerminalCost = croc.CostModelResidual(state, stateTerminalAct,stateTerminalResidual)
costs.addCost("stateReg", stateTerminalCost, p.stateTerminalWeight)

damodel = croc.DifferentialActionModelContactFwdDynamics(
    state, actuation, contacts, costs, p.kktDamping, True
)
termmodel = croc.IntegratedActionModelEuler(damodel, p.DT)

# ##############################################################################
problem = croc.ShootingProblem(x0,models,termmodel)
ddp = croc.SolverFDDP(problem)
ddp.setCallbacks([croc.CallbackVerbose()])

# ##############################################################################
try:
    GUESS_FILE = '/tmp/sol.npy'
    guess = np.load(GUESS_FILE,allow_pickle=True)[()]
    print(f'Load "{GUESS_FILE}"!')
    x0s = [x for x in guess['xs']]
    u0s = [u for u in guess['us']]
    if len(x0s)!=T+1 or len(u0s)!=T: raise ImportError
except:
    print('No valid solution file, build quasistatic initial guess!')
    x0s = [x0.copy()]*(T+1)
    u0s = [ m.quasiStatic(d,x) for m,d,x in zip(problem.runningModels,problem.runningDatas,x0s) ]

# l = pin.StdVec_Double()
# l.append(2)
# ddp.alphas = l

#ddp.th_acceptStep = 0.1
ddp.th_stop = 1e-3
ddp.solve(x0s,u0s,200)

# ### MPC #########################################################################################

Tmpc = 100
mpcProblem = croc.ShootingProblem(x0,models[:Tmpc],termmodel)
mpcSolver =  croc.SolverFDDP(mpcProblem)
#mpcSolver.setCallbacks([croc.CallbackVerbose()])
mpcSolver.th_stop = 1e-3

stateTarget = x0.copy()
stateTarget[:3] = x0[:3] + p.VCOM_TARGET*T*p.DT
mpcProblem.terminalModel.differential.costs.costs['stateReg'].cost.residual.reference = stateTarget

mpcSolver.solve(ddp.xs[:Tmpc+1],ddp.us[:Tmpc],10,isFeasible=True)
x = mpcSolver.xs[1].copy()

hxs = [ np.array(ddp.xs) ]
hx = [ x ]
hiter = [ mpcSolver.iter ]
for t in range(1,500):
    stateTarget = x0.copy()
    stateTarget[:3] = x0[:3] + p.VCOM_TARGET*(t+Tmpc)*p.DT
    mpcProblem.terminalModel.differential.costs.costs['stateReg'].cost.residual.reference = stateTarget
    #tlast = t+Tmpc
    tlast = T_START+1+((t+Tmpc-T_START-1)%(2*T_SINGLE+2*T_DOUBLE))
    #print(f't={t} ... last is {tlast}')
    mpcProblem.circularAppend(problem.runningModels[tlast],problem.runningDatas[tlast])
    mpcProblem.x0 = x.copy()
    #assert(mpcProblem.runningModels[0] == problem.runningModels[t])
    xg = list(mpcSolver.xs)[1:]+[mpcSolver.xs[-1]]
    ug = list(mpcSolver.us)[1:]+[mpcSolver.us[-1]]
    #if t==100: stophere
    mpcSolver.solve(xg,ug,maxiter=1)
    print(f'{t:4d} {miscdisp.dispocp(mpcProblem,contactIds)} {stateTarget[0]:.03} {mpcSolver.iter:4d}')
    x = mpcSolver.xs[1].copy()
    hx.append(x)
    hxs.append(np.array(mpcSolver.xs))
    hiter.append(mpcSolver.iter)
    if not t%10:
        viz.display(x[:model.nq])
        time.sleep(p.DT)
        
# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

xs_sol = np.array(ddp.xs)
us_sol = np.array(ddp.us)
acs_sol = np.array([ d.differential.xout for d in problem.runningDatas ])
fs_sol = [ [  (cd.data().jMf.inverse()*cd.data().f).vector for cd in d.differential.multibody.contacts.contacts ]
           for d in problem.runningDatas ]
fs_sol0 = [ np.concatenate([
    (d.differential.multibody.contacts.contacts[f'{model.frames[cid].name}_contact'].jMf.inverse()*
    d.differential.multibody.contacts.contacts[f'{model.frames[cid].name}_contact'].f).vector
    if f'{model.frames[cid].name}_contact' in d.differential.multibody.contacts.contacts
    else np.zeros(6)
    for cid in contactIds   ])
            for m,d in zip(problem.runningModels,problem.runningDatas) ]

from walk_plotter import WalkPlotter

plotter = WalkPlotter(model,contactIds)
plotter.setData(contactPattern,np.array(hx),None,None)

target=mpcProblem.terminalModel.differential.costs.costs['stateReg'].cost.residual.reference
plotter.plotBasis(target)
#plotter.plotTimeCop()
#plotter.plotCopAndFeet(p.FOOT_SIZE,.6)
#plotter.plotForces(referenceForces)
plotter.plotCom(com0)
plotter.plotFeet()
plotter.plotFootCollision(p.footMinimalDistance)
print('Run ```plt.ion(); plt.show()``` to display the plots.')
# ### SAVE #####################################################################

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
t=90; cid=48 # impact
try:
    xs=guess['xs']
    us=guess['us']
    fs0=guess['fs']
    acs=guess['acs']
except:
    xs=xs_sol
    us=us_sol
    fs0=fs_sol0
    acs=acs_sol
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
cosname='impactRefJoint'
cosdata = dadata.costs.costs[cosname]
cosmodel = damodel.costs.costs[cosname].cost
np.set_printoptions(precision=6, linewidth=300, suppress=True,threshold=10000)
