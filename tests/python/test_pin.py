import unittest

import sobec
from pinocchio import SE3


class TestAdderPin(unittest.TestCase):
    def test_adder_se3(self):
        a = SE3.Random()
        b = SE3.Identity()
        self.assertNotEqual(sobec.adds(a, a), a)
        self.assertEqual(sobec.adds(b, b), b)
        self.assertEqual(sobec.adds(a, b), a)
        self.assertEqual(sobec.adds(b, a), a)
        self.assertEqual(sobec.subs(a, b), a.inverse())


if __name__ == '__main__':
    unittest.main()
