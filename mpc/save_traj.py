import numpy as np


def save_traj(xs,us=None,fs=None,acs=None,filename="/tmp/ddp.npy"):
    data = {  "xs": xs }
    if us is not None: data["us"] =  us
    if acs is not None: data["acs"] = acs
    if fs is not None: data["fs"] = fs
    np.save(open(filename, "wb"),data)
    print(f'Save "{filename}"!')

