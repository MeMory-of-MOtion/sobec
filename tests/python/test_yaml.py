import numpy as np
import sobec
from sobec.walk.yaml_params import yamlWriteParams, yamlReadToParams
from sobec.walk.params import WalkParams
import sobec.walk.yaml_params as yaml_params
from collections.abc import Iterable
import sys

if sys.version_info.major == 3:

    def checkValueEquality(v1, v2):
        if isinstance(v1, float) or isinstance(v1, int):
            return abs(v1 - v2) < 1e-6
        elif isinstance(v2, np.ndarray):
            return np.linalg.norm(v1 - v2) < 1e-6
        elif isinstance(v1, str):
            return v1 == v2
        elif isinstance(v1, Iterable) and isinstance(v2, Iterable):
            return all([checkValueEquality(vi1, vi2) for vi1, vi2 in zip(v1, v2)])
        else:
            return False

    def compareTwoParams(params1, params2, ParamClass):
        for k in dir(params1):
            if k in dir(params2):
                v1 = getattr(params1, k)
                v2 = getattr(params2, k)
                if k[:2] != "__" and yaml_params.isYamlTypeCompatible(v1):
                    print(k, v1, v2)
                    assert checkValueEquality(v1, v2)

    # ### TEST 1
    ocpparams = sobec.OCPWalkParams()
    yamlWriteParams("/tmp/test_sobec_yaml_1.yml", ocpparams)

    ocpparams_pyread = sobec.OCPWalkParams()
    # Check direct read from file is ok
    # --- This cannot be checked because empty list [] are not parsed as np.array
    # --- Trival behavior, ignore for now
    # yamlReadToParams('/tmp/test_sobec_yaml_1.yml',ocpparams_pyread)
    # compareTwoParams(ocpparams,ocpparams_pyread,sobec.OCPWalkParams)

    # ### TEST 2: read complex param from yaml
    pyparams = WalkParams("talos_low")
    yamlWriteParams("/tmp/test_sobec_yaml_2.yml", pyparams)

    ocpparams_pyread = sobec.OCPWalkParams()
    yamlReadToParams("/tmp/test_sobec_yaml_2.yml", ocpparams_pyread)
    compareTwoParams(pyparams, ocpparams_pyread, sobec.OCPWalkParams)

    # ### TEST 3: use c++ yaml read
    ocpparams_cppread = sobec.OCPWalkParams()
    # TODO Not working as param.readFromYaml does not accept str.
    # ocpparams_cppread.readFromYaml('/tmp/test_sobec_yaml_2.yml')
    # compareTwoParams(pyparams,ocpparams_cppread,sobec.OCPWalkParams)
