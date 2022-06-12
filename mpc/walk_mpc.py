import pinocchio as pin
import crocoddyl as croc
import numpy as np

import walk_ocp
import miscdisp

class WalkMPC:

    def updateTerminalStateTarget(self,t):
        '''t is the current time, the target should be computed at t+Tmpc'''
        costmodel = self.problem.terminalModel.differential.costs.costs["stateReg"].cost

        stateTarget = self.robotWrapper.x0.copy()
        termtime = (t + self.walkParams.Tmpc) * self.walkParams.DT
        stateTarget[:3] = stateTarget[:3] + self.walkParams.VCOM_TARGET * termtime
        self.basisRef = stateTarget[:3].copy() # For debug mostly
        
        costmodel.residual.reference = stateTarget
    
    def __init__(self,robotWrapper,storage,walkParams,xs_init=None,us_init=None):

        robot = self.robotWrapper = robotWrapper
        p = self.walkParams = walkParams
        self.storage = storage
        runmodels = self.storage.runningModels
        termmodel = self.storage.terminalModel
        Tmpc = p.Tmpc

        self.state = termmodel.state
        self.problem = croc.ShootingProblem(robot.x0, runmodels[:Tmpc], termmodel)
        self.solver = croc.SolverFDDP(self.problem)
        self.solver.th_stop = p.solver_th_stop
        #self.solver.setCallbacks([croc.CallbackVerbose()])

        self.updateTerminalStateTarget(0)

        if xs_init is not None and us_init is not None:
            x0s = xs_init[:Tmpc+1]
            u0s = us_init[:Tmpc]
        else:
            x0s,u0s = walk_ocp.buildInitialGuess(self.problem,self.walkParams)
        
        self.hxs = [np.array(self.solver.xs)]
        self.hx = [self.problem.x0]
        self.hiter = [self.solver.iter]

        self.solver.solve(x0s,u0s,10*p.maxiter)
        print(
            f"{0:4d} {miscdisp.dispocp(self.problem,robot.contactIds)} "
            f"{self.basisRef[0]:.03} {self.solver.iter:4d}"
        )
        
    def run(self,x,t):
        
        robot = self.robotWrapper
        p = self.walkParams
        runmodels = self.storage.runningModels
        rundatas = self.storage.runningDatas
        termmodel = self.storage.terminalModel
        Tmpc = p.Tmpc
   
        self.updateTerminalStateTarget(t)

        tlast = p.T_START + 1 + ((t + p.Tmpc - p.T_START - 1) % (2 * p.T_SINGLE + 2 * p.T_DOUBLE))
        self.problem.circularAppend(runmodels[tlast], rundatas[tlast])
        self.problem.x0 = x.copy()

        xg = list(self.solver.xs)[1:] + [self.solver.xs[-1]]
        ug = list(self.solver.us)[1:] + [self.solver.us[-1]]
        self.solver.solve(xg, ug, maxiter=p.maxiter)
        print(
            f"{t:4d} {miscdisp.dispocp(self.problem,robot.contactIds)} "
            f"{self.basisRef[0]:.03} {self.solver.iter:4d}"
        )
        x = self.solver.xs[1].copy()
        self.hx.append(x)
        self.hxs.append(np.array(self.solver.xs))
        self.hiter.append(self.solver.iter)

