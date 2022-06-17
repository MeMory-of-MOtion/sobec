import numpy as np
import sobec
from sobec.walk.yaml_params import yamlWriteParams, yamlReadToParams
from sobec.walk.params import WalkParams
import sobec.walk.yaml_params as yaml_params


def compareTwoParams(params1, params2, ParamClass):
    for k in ParamClass.__dict__.keys():
        if k[:2] == "__":
            continue
        if not hasattr(params1, k) or not hasattr(params2, k):
            # Check that, if v1 exits, it is not yaml compatible
            assert not (
                hasattr(params1, k)
                and yaml_params.isYamlTypeCompatible(params1.__getattribute__(k))
            )
            # Check that, if v2 exits, it is not yaml compatible
            assert not (
                hasattr(params2, k)
                and yaml_params.isYamlTypeCompatible(params2.__getattribute__(k))
            )
            continue
        v1 = params1.__getattribute__(k)
        v2 = params2.__getattribute__(k)
        # print(f'Compare {k}: {v1} vs {v2} ')
        if isinstance(v1, float) or isinstance(v1, int):
            assert abs(v1 - v2) < 1e-6
        elif isinstance(v2, np.ndarray):
            assert np.linalg.norm(v1 - v2) < 1e-6


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
