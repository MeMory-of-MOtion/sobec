"""
Simple numdiff test of the COP cost.
COP is a complex function as it depends on the contact forces.
Hence, build a complete DAM with contact, and assert its gradient WRT numdiff.
"""

import pinocchio as pin
import crocoddyl as croc
import numpy as np
import example_robot_data as robex
from numpy.linalg import norm, inv, pinv, svd, eig

# Local imports
import sobec

np.random.seed(0)

### LOAD AND DISPLAY SOLO
# Load the robot model from example robot data and display it if possible in Gepetto-viewer
# robot = talos_low.load()
# contactIds = [ i for i,f in enumerate(robot.model.frames) if "sole_link" in f.name ]
# contactIds = [ contactIds[0] ]

robot = robex.load("talos_arm")
contactIds = [len(robot.model.frames) - 1]

# The pinocchio model is what we are really interested by.
model = robot.model
model.q0 = robot.q0
data = model.createData()

# Initial config, also used for warm start
x0 = np.concatenate([model.q0, np.zeros(model.nv)])


# #################################################################################################

state = croc.StateMultibody(model)
actuation = croc.ActuationModelFloatingBase(state)

# Contacts
contacts = croc.ContactModelMultiple(state, actuation.nu)
for cid in contactIds:
    contact = croc.ContactModel6D(
        state, cid, pin.SE3.Identity(), actuation.nu, np.zeros(2)
    )
    contacts.addContact(model.frames[cid].name + "_contact", contact)

# Costs
costs = croc.CostModelSum(state, actuation.nu)

for cid in contactIds:
    copResidual = sobec.ResidualModelCenterOfPressure(state, cid, actuation.nu)
    copAct = croc.ActivationModelWeightedQuad(np.array([1.1, 0.9]))
    copCost = croc.CostModelResidual(state, copAct, copResidual)
    costs.addCost(f"{model.frames[cid].name}_cop", copCost, 1.5)

    transResidual = croc.ResidualModelFrameTranslation(
        state, cid, np.array([1, 2, 3]), actuation.nu
    )
    transCost = croc.CostModelResidual(state, transResidual)
    # costs.addCost('trans',transCost,1)


# Action
damodel = croc.DifferentialActionModelContactFwdDynamics(
    state, actuation, contacts, costs, 0, True
)

# #################################################################################################

# ### For easier manipulation at debug time
dadata = damodel.createData()
u0 = np.random.rand(actuation.nu) * 20 - 10

damodel.calc(dadata, x0, u0)
damodel.calcDiff(dadata, x0, u0)

try:
    fname = "left_sole_link"
    conname = f"{fname}_contact"
    condata = dadata.multibody.contacts.contacts[conname]
    cosname = f"{fname}_cop"
    cosdata = dadata.costs.costs[cosname]
    cosmodel = damodel.costs.costs[cosname].cost
except:
    pass

# ### NUMDIFF TEST
damnd = croc.DifferentialActionModelNumDiff(damodel, gaussApprox=True)
dadnd = damnd.createData()
damnd.calc(dadnd, x0, u0)
damnd.calcDiff(dadnd, x0, u0)

assert norm(dadnd.Lx - dadata.Lx) / norm(dadata.Lx) < 1e-5
assert norm(dadnd.Lu - dadata.Lu) / norm(dadata.Lx) < 1e-5
