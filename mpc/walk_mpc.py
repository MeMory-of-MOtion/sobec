import crocoddyl as croc
import numpy as np
import time
import warnings

import walk_ocp
import miscdisp
import sobec


class WalkMPC:
    def updateTerminalStateTarget(self, t):
        """t is the current time, the target should be computed at t+Tmpc"""
        costmodel = self.problem.terminalModel.differential.costs.costs["stateReg"].cost

        stateTarget = self.robotWrapper.x0.copy()
        termtime = (t + self.walkParams.Tmpc) * self.walkParams.DT
        stateTarget[:3] = stateTarget[:3] + self.walkParams.vcomRef * termtime
        self.basisRef = stateTarget[:3].copy()  # For debug mostly

        costmodel.residual.reference = stateTarget

    def __init__(self, robotWrapper, storage, walkParams, xs_init=None, us_init=None):
        warnings.warn("Class WalkMPC is deprecated. Now use sobec.MPCWalk.")

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
        # self.solver.setCallbacks([croc.CallbackVerbose()])

        self.updateTerminalStateTarget(0)

        if xs_init is not None and us_init is not None:
            x0s = xs_init[: Tmpc + 1]
            u0s = us_init[:Tmpc]
        else:
            x0s, u0s = walk_ocp.buildInitialGuess(self.problem, self.walkParams)

        self.hxs = [np.array(self.solver.xs)]
        self.hx = [self.problem.x0]
        self.hiter = [self.solver.iter]

        self.reg = p.solver_reg_min
        self.solver.reg_min = p.solver_reg_min

        self.solver.solve(x0s, u0s)
        print(
            "{:4d} {} {:.03} {:4d}".format(
                0,
                miscdisp.dispocp(self.problem, robot.contactIds),
                self.basisRef[0],
                self.solver.iter,
            )
        )

    def run(self, x, t):

        robot = self.robotWrapper
        p = self.walkParams
        runmodels = self.storage.runningModels
        rundatas = self.storage.runningDatas
        # termmodel = self.storage.terminalModel
        # Tmpc = p.Tmpc

        self.updateTerminalStateTarget(t)

        tlast = (
            p.Tstart
            + 1
            + ((t + p.Tmpc - p.Tstart - 1) % (2 * p.Tsingle + 2 * p.Tdouble))
        )
        self.problem.circularAppend(runmodels[tlast], rundatas[tlast])
        self.problem.x0 = x.copy()

        xg = list(self.solver.xs)[1:] + [self.solver.xs[-1]]
        ug = list(self.solver.us)[1:] + [self.solver.us[-1]]
        start_time = time.time()
        solved = self.solver.solve(
            xg, ug, maxiter=p.maxiter, isFeasible=False, regInit=self.reg
        )
        solve_time = time.time() - start_time

        self.ref = self.solver.x_reg
        print(
            f"{t:4d} {miscdisp.dispocp(self.problem,robot.contactIds)} "
            # f"{self.basisRef[0]:.03} "
            f"{self.solver.iter:4d} "
            f"reg={self.solver.x_reg:.3} "
            f"a={self.solver.stepLength:.3} "
            f"solveTime={solve_time:.3}"
        )
        x = self.solver.xs[1].copy()
        self.hx.append(x)
        self.hxs.append(np.array(self.solver.xs))
        self.hiter.append(self.solver.iter)

        return solved

    def moreIterations(self, mult):
        self.solver.solve(
            self.solver.xs, self.solver.us, maxiter=self.walkParams.maxiter * mult
        )


def configureMPCWalk(mpc, params):
    mpc.Tmpc = params.Tmpc
    mpc.Tstart = params.Tstart
    mpc.Tdouble = params.Tdouble
    mpc.Tsingle = params.Tsingle
    mpc.Tend = params.Tend
    mpc.DT = params.DT
    mpc.solver_th_stop = params.solver_th_stop
    mpc.vcomRef = params.vcomRef
    mpc.solver_reg_min = params.solver_reg_min
    mpc.solver_maxiter = params.maxiter


if __name__ == "__main__":
    import pinocchio as pin
    import crocoddyl as croc
    import numpy as np
    import matplotlib.pylab as plt  # noqa: F401
    from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
    import time

    # Local imports
    from save_traj import save_traj
    import walk_plotter
    from robot_wrapper import RobotWrapper
    import walk_ocp as walk
    from mpcparams import WalkParams
    import talos_low
    from walk_mpc import WalkMPC
    import viewer_multiple

    urdf = talos_low.load()
    robot = RobotWrapper(urdf.model, contactKey="sole_link")
    p = walkParams = WalkParams()
    contactPattern = (
        []
        + [[1, 1]] * walkParams.Tstart
        + [[1, 1]] * walkParams.Tdouble
        + [[0, 1]] * walkParams.Tsingle
        + [[1, 1]] * walkParams.Tdouble
        + [[1, 0]] * walkParams.Tsingle
        + [[1, 1]] * walkParams.Tdouble
        + [[1, 1]] * walkParams.Tend
        + [[1, 1]]
    )
    # ### DDP #############################################################################
    ddp = walk.buildSolver(robot, contactPattern, walkParams)
    problem = ddp.problem
    x0s, u0s = walk.buildInitialGuess(ddp.problem, walkParams)
    ddp.setCallbacks([croc.CallbackVerbose()])
    ddp.solve(x0s, u0s, 200)

    # ### MPC #############################################################################
    problem1 = walk.buildSolver(robot, contactPattern, walkParams).problem
    problem2 = walk.buildSolver(robot, contactPattern, walkParams).problem

    mpc = WalkMPC(robot, problem1, walkParams, xs_init=ddp.xs, us_init=ddp.us)
    mpccpp = sobec.MPCWalk(problem2)
    configureMPCWalk(mpccpp, walkParams)
    x = robot.x0

    mpccpp.initialize(ddp.xs[: p.Tmpc + 1], ddp.us[: p.Tmpc])
    mpccpp.solver.setCallbacks([croc.CallbackVerbose()])
    mpc.solver.setCallbacks([croc.CallbackVerbose()])
    assert norm(mpc.solver.xs[10] - mpccpp.solver.xs[10]) < 1e-9

    for t in range(1, 200):
        print(f"\n\n\n *** ITER {t} \n\n")
        x = mpc.solver.xs[1]
        mpc.run(x, t)

        mpccpp.calc(x, t)

        assert norm(mpc.solver.xs[10] - mpccpp.solver.xs[10]) < 1e-6

    # ### DEBUG ##########################################################################
    pin.SE3.__repr__ = pin.SE3.__str__
    np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)
