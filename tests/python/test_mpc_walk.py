import example_robot_data as robex
import crocoddyl as croc
import numpy as np
import pinocchio as pin
import sobec

T_START = 10
T_DOUBLE = 10
T_SINGLE = 10
T_END = 10
T = T_START+(T_DOUBLE+T_SINGLE)*2+T_END
Tmpc = 20

robot = robex.load('ur5')
model = robot.model

runmodels = []

state = croc.StateMultibody(model)
actuation = croc.ActuationModelFloatingBase(state)

# Costs
costs = croc.CostModelSum(state, actuation.nu)

model.x0 = np.concatenate([ pin.neutral(model), np.zeros(model.nv) ])
xRegResidual = croc.ResidualModelState(state, model.x0, actuation.nu)
xRegCost = croc.CostModelResidual(state,xRegResidual)
costs.addCost("stateReg", xRegCost, 1)

damodel = croc.DifferentialActionModelFreeFwdDynamics( state, actuation,costs )
amodel = croc.IntegratedActionModelEuler(damodel, .1)

problem = croc.ShootingProblem(model.x0,[ amodel for t in range(T) ],amodel)
ddp = croc.SolverFDDP(problem)
ddp.solve()

mpc = sobec.MPCWalk(problem)
mpc.Tmpc = Tmpc
mpc.T_START = T_START
mpc.T_DOUBLE = T_DOUBLE
mpc.T_SINGLE = T_SINGLE
mpc.T_END = T_END
#mpc.initialize([ x for x in ddp.xs[:Tmpc+1]],[ u for u in ddp.us[:Tmpc]])
#mpc.calc(model.x0,0)

