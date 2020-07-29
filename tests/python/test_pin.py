import unittest

import example_adder as exa
from pinocchio import SE3


class TestAdderPin(unittest.TestCase):
    def test_adder_se3(self):
        a = SE3.Random()
        b = SE3.Identity()
        self.assertNotEqual(exa.adds(a, a), a)
        self.assertEqual(exa.adds(b, b), b)
        self.assertEqual(exa.adds(a, b), a)
        self.assertEqual(exa.adds(b, a), a)
        self.assertEqual(exa.subs(a, b), a.inverse())


if __name__ == '__main__':
    unittest.main()
