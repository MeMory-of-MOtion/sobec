"""
Simple numdiff test of the COP cost.
COP is a complex function as it depends on the contact forces.
Hence, build a complete DAM with contact, and assert its gradient WRT numdiff.
"""

import pinocchio as pin
import crocoddyl as croc
import numpy as np
import example_robot_data as robex
from numpy.linalg import norm

# Local imports
import sobec

np.random.seed(0)

# ## LOAD AND DISPLAY SOLO
robot = robex.load("talos_legs")
feetIds = [ i for i,f in enumerate(robot.model.frames) if "sole_link" in f.name ]

# The pinocchio model is what we are really interested by.
model = robot.model
model.q0 = pin.integrate(model,robot.q0,np.random.rand(model.nv)*2-1)
del robot.q0

# Initial config, also used for warm start
x0 = np.concatenate([model.q0, np.zeros(model.nv)])

# #####################################################################################

state = croc.StateMultibody(model)
actuation = croc.ActuationModelFull(state)

# Costs
costs = croc.CostModelSum(state, actuation.nu)

feetColResidual = sobec.ResidualModelFeetCollision(state, feetIds[0],feetIds[1], actuation.nu)
feetColCost = croc.CostModelResidual(state, feetColResidual)
costs.addCost("feetcol", feetColCost, 1)

# Action
damodel = croc.DifferentialActionModelFreeFwdDynamics(state, actuation, costs)
dadata = damodel.createData()

# #####################################################################################

cid1 = feetIds[0]
cid2 = feetIds[1]
jid1 = model.frames[cid1].parent
jid2 = model.frames[cid2].parent

u0 = np.random.rand(actuation.nu) * 20 - 10
v0 = np.random.rand(actuation.nu) * 2 - 1
x0[model.nq :] = v0

# For easier manipulation at debug time
np.set_printoptions(precision=3, linewidth=300, suppress=True, threshold=10000)
cosname = "feetcol"
cosdata = dadata.costs.costs[cosname]
cosmodel = damodel.costs.costs[cosname].cost
data = cosdata.shared.pinocchio
print('Cost model ready')

# Calc!
damodel.calc(dadata, x0, u0)
damodel.calcDiff(dadata, x0, u0)

# ## MANUAL CHECK
# Recompute the residual manually in python and assert it
p1 = data.oMf[cid1].translation #[:2]
p2 = data.oMf[cid2].translation #[:2]
dist = norm(p1[:2]-p2[:2])
assert(abs(cosdata.residual.r[0]-dist)<1e-10)

J1 = pin.getFrameJacobian(model,data,cid1,pin.LOCAL_WORLD_ALIGNED)
J2 = pin.getFrameJacobian(model,data,cid2,pin.LOCAL_WORLD_ALIGNED)
J = (p1[:2]-p2[:2]).T/dist @ (J1[:2]-J2[:2])

# Finite-diff for the residual jacobian
def fun2(q):
    pin.framesForwardKinematics(model,data,q)
    p1 = data.oMf[cid1].translation[:2]
    p2 = data.oMf[cid2].translation[:2]
    dist = np.array([sum((p1-p2)**2)])
    dist = np.array([norm(p1-p2)])
    return dist
 
def nd(f,q,h=1e-6):
    v0 = f(q).copy()
    dq = np.zeros(model.nv)
    J = []
    for i in range(model.nv):
        dq[i] = h
        qplus = pin.integrate(model,q,dq)
        J.append( (f(qplus)-v0)/h)
        dq[i] = 0
    return np.array(J).T

Jnd = nd(fun2,model.q0)
assert(norm(Jnd-cosdata.residual.Rx[:model.nv])<1e-6)
assert(norm(Jnd-J)<1e-6)
assert(norm(J-cosdata.residual.Rx[:model.nv])<1e-6)

# ### NUMDIFF TEST using croco tools (assert gradient, not jacobian residual)
damnd = croc.DifferentialActionModelNumDiff(damodel)
dadnd = damnd.createData()
damnd.calc(dadnd, x0, u0)
damnd.calcDiff(dadnd, x0, u0)

assert norm(dadnd.Lx - dadata.Lx) / norm(dadata.Lx) < 1e-5
assert norm(dadnd.Lu - dadata.Lu) / norm(dadata.Lx) < 1e-5
