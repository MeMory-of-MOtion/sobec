import example_robot_data as robex
import crocoddyl as croc
import numpy as np
import pinocchio as pin
import sobec

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

problem = croc.ShootingProblem(model.x0,[ amodel for t in range(20) ],amodel)

mpc = sobec.MPCWalk(problem)
mpc.Tmpc = 10 #len(problem.runningModels)
mpc.initialize([ model.x0,model.x0 ],[model.x0])
mpc.calc(model.x0,0)

