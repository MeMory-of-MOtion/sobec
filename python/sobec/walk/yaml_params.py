"""
This module provides functions to write paams into a yamlfile. Params can
be either python params from sobec.walk.params, or c++ params from
sobec.OCPWalkParams

- yamlWriteParams

"""

import yaml
import numpy as np
import sobec

# ################################################################################
# ################################################################################
# ################################################################################


def paramsToDict(params):
    # res = params.__dict__
    # for k in params.__class__.__dict__.keys():
    #     if k[:2] != "__" and k not in res:
    #         res[k] = getattr(params, k)
    # if len(params.__class__.__bases__) > 0:
    #     for k in params.__class__.__bases__[0].__dict__.keys():
    #         if k[:2] != "__" and k not in res:
    #             res[k] = getattr(params, k)
    # return res
    return {k: getattr(params, k) for k in dir(params) if k[:2] != "__"}


def isYamlTypeCompatible(v):
    if isinstance(v, float) or isinstance(v, int) or isinstance(v, str):
        return True
    elif isinstance(v, np.ndarray):
        return len(v.shape) == 1
    elif isinstance(v, list):
        return all([isYamlTypeCompatible(vi) for vi in v])
    else:
        return False


def flattenDictArrayValues(paramsAsDict):
    """
    Taking a dict containing array vector values, flatten them to list
    """
    todel = []
    for k, v in paramsAsDict.items():
        if not isYamlTypeCompatible(v):
            print("Field <%s> is not a serializable type for yaml ... ignore" % k)
            todel.append(k)
        if isinstance(v, np.ndarray):
            paramsAsDict[k] = v.tolist()
        elif isinstance(v, sobec.sobec_pywrap.StdVectorStdStringIndex_):
            paramsAsDict[k] = [str(vi) for vi in v]
    for k in todel:
        del paramsAsDict[k]


def yamlWrite(paramsAsDict, filename):
    with open(filename, "w") as outfile:
        yaml.dump(paramsAsDict, outfile, default_flow_style=None)


def yamlWriteParams(filename, *args):
    # pdict = { **paramsToDict(params) for params in args }
    pdict = {k: v for params in args for k, v in paramsToDict(params).items()}
    flattenDictArrayValues(pdict)
    yamlWrite(pdict, filename)


# ################################################################################
# ################################################################################
# ################################################################################


def yamlRead(filename):
    with open(filename, "r") as infile:
        return yaml.load(infile, Loader=yaml.FullLoader)


def dictToNpValue(paramsAsDict):
    for k, v in paramsAsDict.items():
        if isinstance(v, list) and len(v) > 0:
            if isinstance(v[0], float) or isinstance(v[0], int):
                paramsAsDict[k] = np.array(v, np.float64)


def yamlReadToParams(filename, params):
    pdict = yamlRead(filename)
    dictToNpValue(pdict)
    for k, v in pdict.items():
        if hasattr(params, k):
            setattr(params, k, v)


# ################################################################################
# ################################################################################
# ################################################################################

if __name__ == "__main__":
    import sobec.walk.params

    params = sobec.walk.params.WalkParams("talos_low")
    pdict = paramsToDict(params)
    flattenDictArrayValues(pdict)
    yamlWrite(pdict, "/tmp/walk.yaml")

    ocpparams = sobec.OCPWalkParams()
    ocpdict = paramsToDict(ocpparams)
    mpcparams = sobec.MPCWalkParams()
    mpcdict = paramsToDict(mpcparams)
    cppdict = {
        k: v
        for params in [ocpparams, mpcparams]
        for k, v in paramsToDict(params).items()
    }
    flattenDictArrayValues(cppdict)

    yamlWriteParams("/tmp/swp_full.yml", params)
    yamlWriteParams("/tmp/sobec_full.yml", ocpparams, mpcparams)

    yamlReadToParams("/tmp/walk.yaml", ocpparams)
    yamlReadToParams("/tmp/walk.yaml", mpcparams)

    for paramClass in [sobec.OCPWalkParams, sobec.MPCWalkParams]:
        for k in paramClass.__class__.__dict__.keys():
            if k[:2] == "__":
                continue
            if isinstance(ocpparams[k], float) or isinstance(ocpparams[k], int):
                assert abs(ocpparams[k] - params[k]) < 1e-6
            elif isinstance(ocpparams[k], np.ndarray):
                assert np.linalg.norm(ocpparams[k] - params[k]) < 1e-6
