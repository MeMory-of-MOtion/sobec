import numpy as np


def save_traj(xs,us=None,fs=None,acs=None,filename="/tmp/ddp.npy"):
    data = {  "xs": xs }
    if us is not None: data["us"] =  us
    if acs is not None: data["acs"] = acs
    if fs is not None: data["fs"] = fs
    with open(filename, "wb") as f:
        np.save(f, data)
    print(f'Save "{filename}"!')

def getContactActivationFromAModel(contactIds,action):
    '''Return a list of binary values representing the contact activation of an action movel'''
    model = action.differential.pinocchio
    return [ int(f'{model.frames[cid].name}_contact' in action.differential.contacts.contacts) for cid in contactIds ]

def saveProblemConfig(contactIds,problem,filename='/tmp/ddp.config'):
    data = {
        'contactPattern': [ getContactActivationFromAModel(contactIds,r) for r in problem.runningModels ] \
        + [ getContactActivationFromAModel(contactIds,problem.terminalModel) ],
        'x0': problem.x0,
        'stateTerminalTarget': problem.terminalModel.differential.costs.costs['stateReg'].cost.residual.reference,
    }
    with open(filename, "wb") as f:
        np.save(f, data)
    print(f'Save OCP config in "{filename}"!')

def loadProblemConfig(filename='/tmp/ddp.config'):
    data = np.load(filename,allow_pickle=True)[()]
    print(f'Load OCP config from "{filename}"!')

    return data
