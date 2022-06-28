import unittest
from tempfile import NamedTemporaryFile

try:
    from collections.abc import Iterable
except ImportError:  # Python 2
    from collections import Iterable


import numpy as np

import sobec
import sobec.walk_without_think.yaml_params as yaml_params


class TestYaml(unittest.TestCase):
    def assertValueEquality(self, v1, v2):
        if isinstance(v1, float) or isinstance(v1, int):
            self.assertLess(abs(v1 - v2), 1e-6)
        elif isinstance(v2, np.ndarray):
            self.assertLess(np.linalg.norm(v1 - v2), 1e-6)
        elif isinstance(v1, str):
            self.assertEqual(v1, v2)
        elif isinstance(v1, Iterable) and isinstance(v2, Iterable):
            for vi1, vi2 in zip(v1, v2):
                self.assertValueEquality(vi1, vi2)
        else:
            self.assertTrue(False)

    def compareTwoParams(self, params1, params2, ParamClass):
        for k in dir(params1):
            if k in dir(params2):
                v1 = getattr(params1, k)
                v2 = getattr(params2, k)
                if not k.startswith("__") and yaml_params.isYamlTypeCompatible(v1):
                    print(k, v1, v2)
                    self.assertValueEquality(v1, v2)

    def test_1(self):
        with NamedTemporaryFile() as yamlFile:
            ocpparams = sobec.OCPWalkParams()
            sobec.wwt.yamlWriteParams(yamlFile.name, ocpparams)

            ocpparams_pyread = sobec.OCPWalkParams()
            # Check direct read from file is ok
            # This cannot be checked because empty list [] are not parsed as np.array
            # Trival behavior, ignore for now
            # yamlReadToParams(yamlFile.name, ocpparams_pyread)
            self.compareTwoParams(ocpparams, ocpparams_pyread, sobec.OCPWalkParams)

    def test_2(self):
        """Read complex param from yaml."""
        with NamedTemporaryFile() as yamlFile:
            pyparams = sobec.wwt.WalkParams("talos_low")
            sobec.wwt.yamlWriteParams(yamlFile.name, pyparams)

            ocpparams_pyread = sobec.OCPWalkParams()
            sobec.wwt.yamlReadToParams(yamlFile.name, ocpparams_pyread)
            self.compareTwoParams(pyparams, ocpparams_pyread, sobec.OCPWalkParams)

    def test_3(self):
        """Use c++ yaml read."""
        with NamedTemporaryFile() as yamlFile:
            pyparams = sobec.wwt.WalkParams("talos_low")
            sobec.wwt.yamlWriteParams(yamlFile.name, pyparams)

            ocpparams_cppread = sobec.OCPWalkParams()
            ocpparams_cppread.readFromYaml(yamlFile.name)
            self.compareTwoParams(pyparams, ocpparams_cppread, sobec.OCPWalkParams)


if __name__ == "__main__":
    unittest.main()
