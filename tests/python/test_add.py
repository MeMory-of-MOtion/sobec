import unittest

import sobec


class TestAdder(unittest.TestCase):
    def test_adder_integers(self):
        self.assertEqual(sobec.add(4, 3), 7)
        self.assertEqual(sobec.sub(4, 3), 1)


if __name__ == "__main__":
    unittest.main()
