import example_robot_data as robex
import crocoddyl as croc
import numpy as np
import pinocchio as pin
import sobec

Tstart = 10
Tdouble = 10
Tsingle = 10
Tend = 10
T = Tstart+(Tdouble+Tsingle)*2+Tend
Tmpc = 20

robot = robex.load('talos_legs')
model = robot.model
cid = 16

runmodels = []

state = croc.StateMultibody(model)
actuation = croc.ActuationModelFloatingBase(state)

# Contact
contacts = croc.ContactModelMultiple(state, actuation.nu)
contact = croc.ContactModel6D(state, cid, pin.SE3.Identity(), actuation.nu,np.zeros(2))
contacts.addContact('contact',contact)

# Costs
costs = croc.CostModelSum(state, actuation.nu)

model.x0 = np.concatenate([ pin.neutral(model), np.zeros(model.nv) ])
xRegResidual = croc.ResidualModelState(state, model.x0, actuation.nu)
xRegCost = croc.CostModelResidual(state,xRegResidual)
costs.addCost("stateReg", xRegCost, 1)


damodel = croc.DifferentialActionModelContactFwdDynamics( state, actuation,contacts,costs,1e-3,True )
amodel = croc.IntegratedActionModelEuler(damodel, .1)

problem = croc.ShootingProblem(model.x0,[ amodel for t in range(T) ],amodel)
ddp = croc.SolverFDDP(problem)
ddp.solve()

mpc = sobec.MPCWalk(problem)
mpc.Tmpc = Tmpc
mpc.Tstart = Tstart
mpc.Tdouble = Tdouble
mpc.Tsingle = Tsingle
mpc.Tend = Tend
mpc.vcomRef = np.array([.33,.1,.0])
mpc.initialize([ x for x in ddp.xs[:Tmpc+1]],[ u for u in ddp.us[:Tmpc]])

mpc.calc(model.x0,10)
mpc.solver.setCallbacks([croc.CallbackVerbose() ])
mpc.solver.solve()
mpc.calc(model.x0,10)


