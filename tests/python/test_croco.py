import unittest

import crocoddyl
import numpy as np
np.random.seed(2)

import sobec


class TestAdderPin(unittest.TestCase):
    def test_uniex(self):
        model = sobec.ActionModelUniEx()
        problem = crocoddyl.ShootingProblem(np.random.rand(3), [model] * 20, model)
        solver = crocoddyl.SolverDDP(problem)

        self.assertEqual(solver.xs[-1][0], 0)
        self.assertEqual(solver.xs[-1][1], 0)
        self.assertEqual(solver.xs[-1][2], 0)

        solver.solve()

        self.assertNotEqual(solver.xs[-1][0], 0)
        self.assertNotEqual(solver.xs[-1][1], 0)
        self.assertNotEqual(solver.xs[-1][2], 0)
        self.assertAlmostEqual(solver.xs[-1][0], 0, places=6)
        self.assertAlmostEqual(solver.xs[-1][1], 0, delta=1e-1)
        self.assertAlmostEqual(solver.xs[-1][2], 0)


if __name__ == '__main__':
    unittest.main()
