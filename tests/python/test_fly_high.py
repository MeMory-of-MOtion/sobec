"""
Simple numdiff test of the COP cost.
COP is a complex function as it depends on the contact forces.
Hence, build a complete DAM with contact, and assert its gradient WRT numdiff.
"""

import unittest

import pinocchio as pin
import crocoddyl as croc
import numpy as np
import example_robot_data as robex
from numpy.linalg import norm

# Local imports
import sobec


class TestFlyHigh(unittest.TestCase):
    def test_fly_high(self):

        np.random.seed(0)

        # ## LOAD AND DISPLAY SOLO
        # Load the robot model from example robot data and display it if possible in
        # Gepetto-viewer
        # robot = talos_low.load()
        # contactIds = [
        # i for i, f in enumerate(robot.model.frames) if "sole_link" in f.name
        # ]
        # contactIds = [ contactIds[0] ]

        robot = robex.load("talos_arm")
        contactIds = [len(robot.model.frames) - 1]

        # The pinocchio model is what we are really interested by.
        model = robot.model
        model.q0 = robot.q0

        # Initial config, also used for warm start
        x0 = np.concatenate([model.q0, np.zeros(model.nv)])

        # #############################################################################

        state = croc.StateMultibody(model)
        actuation = croc.ActuationModelFull(state)

        # Costs
        costs = croc.CostModelSum(state, actuation.nu)

        for cid in contactIds:
            flyHighResidual = sobec.ResidualModelFlyHigh(
                state, cid, 1. / 2, actuation.nu
            )
            flyHighCost = croc.CostModelResidual(state, flyHighResidual)
            costs.addCost("fly", flyHighCost, 1)

            # velRes = croc.ResidualModelFrameVelocity(
            # state,
            # cid,
            # pin.Motion.Zero(),
            # pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            # actuation.nu,
            # )
            # velCost = croc.CostModelResidual(state, velRes)
            # costs.addCost('fly',velCost,1)

        # Action
        damodel = croc.DifferentialActionModelFreeFwdDynamics(state, actuation, costs)

        # #############################################################################

        # jid = model.frames[cid].parent

        # ### For easier manipulation at debug time
        dadata = damodel.createData()
        u0 = np.random.rand(actuation.nu) * 20 - 10
        u0 *= 0
        v0 = np.random.rand(actuation.nu) * 2 - 1
        x0[model.nq :] = v0

        try:
            cosname = "fly"
            cosdata = dadata.costs.costs[cosname]
            # cosmodel = damodel.costs.costs[cosname].cost
            data = cosdata.shared.pinocchio
        except KeyError:
            pass

        damodel.calc(dadata, x0, u0)
        damodel.calcDiff(dadata, x0, u0)

        # ## MANUAL CHECK
        np.set_printoptions(precision=3, linewidth=300, suppress=True, threshold=10000)
        self.assertGreater(norm(cosdata.residual.r), 0)
        v = pin.getFrameVelocity(
            model, data, cid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        ez = np.exp(-data.oMf[cid].translation[2] / 2)
        # ez=1
        r = v.linear[:2] * ez
        self.assertLess(norm(cosdata.residual.r - r), 1e-6)

        # Compute LWA velocity derivatives
        dq, dv = pin.getFrameVelocityDerivatives(
            model, data, cid, pin.ReferenceFrame.LOCAL
        )
        J = pin.getFrameJacobian(
            model, data, cid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        R = data.oMf[cid].rotation
        o_dv = np.dot(R, dv[:3])
        o_dq = np.dot(R, dq[:3]) - np.dot(np.dot(pin.skew(v.linear), R), dv[3:])
        self.assertLess(norm(J[3:] - np.dot(R, dv[3:])), 1e-6)

        # Compute residual derivatives, first due to the velocity ...
        Rv = o_dv[:2] * ez
        Rq = o_dq[:2] * ez
        # ... second due to the altitude
        Rq[0] -= r[0] / 2 * J[2]
        Rq[1] -= r[1] / 2 * J[2]
        self.assertLess(norm(Rv - cosdata.residual.Rx[:, model.nv :]), 1e-6)
        self.assertLess(norm(Rq - cosdata.residual.Rx[:, : model.nv]), 1e-6)

        # ### NUMDIFF TEST
        damnd = croc.DifferentialActionModelNumDiff(damodel, gaussApprox=True)
        dadnd = damnd.createData()
        damnd.calc(dadnd, x0, u0)
        damnd.calcDiff(dadnd, x0, u0)

        self.assertLess(norm(dadnd.Lx - dadata.Lx) / norm(dadata.Lx), 1e-5)
        self.assertLess(norm(dadnd.Lu - dadata.Lu) / norm(dadata.Lx), 1e-5)


if __name__ == "__main__":
    unittest.main()
