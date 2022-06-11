'''
Some simple methods to display an OCP problem as string.
'''

import pinocchio as pin
from numpy.linalg import norm,inv,pinv,svd,eig
import crocoddyl as croc

### Save all the action details in a text file.
def reprAction(amodel):
    str = ''
    dmodel = amodel.differential
    str+='=== Contact\n'
    for citem in dmodel.contacts.contacts:
        str+=f'  - {citem.key()}: {citem.data()}\n'
    str+='=== Cost\n'
    for citem in dmodel.costs.costs:
        str+=f'  - {citem.key()}: {citem.data()}\n'
        cost = citem.data().cost
        if isinstance(cost.activation,croc.ActivationModelWeightedQuad):
            str+=f'\t\twact = {cost.activation.weights}\n'
        if isinstance(cost.activation,croc.ActivationModelQuadraticBarrier):
            str+=f'\t\tlower = {cost.activation.bounds.lb}\n'
            str+=f'\t\tupper = {cost.activation.bounds.lb}\n'
    return str

def reprProblem(problem):
    str = ''
    for t,r in enumerate(problem.runningModels):
        str+=f'*t={t}\n'
        str+=reprAction(r)
    str+=f'*TERMINAL\n'
    str+=reprAction(problem.terminalModel)
    return str

### Represent an ocp as one line of ASCII caracters for contacts and impacts
def contact2car(model,contactIds,contacts,costs):
    left = '⎻'
    right = '_'
    leftplus = '┘'
    rightplus = '┌'
    double = '='
    res = ''
    if len(contacts)==2:
        if f'{model.frames[contactIds[1]].name}_altitudeimpact' in costs: res += ","
        if f'{model.frames[contactIds[0]].name}_altitudeimpact' in costs: res += "'"
        res += '='
    elif len(contacts)==0: res += ' '
    elif f'{model.frames[contactIds[1]].name}' in next(iter(contacts)).key():
        if f'{model.frames[contactIds[1]].name}_altitudeimpact' in costs: res += rightplus
        else: res += right
    elif f'{model.frames[contactIds[0]].name}' in next(iter(contacts)).key():
        if f'{model.frames[contactIds[0]].name}_altitudeimpact' in costs: res += leftplus
        else: res += left
    else: res += '*' 
    return res
  
def dispocp(pb,contactIds):
    return ''.join([contact2car(r.differential.pinocchio,contactIds,r.differential.contacts.contacts,r.differential.costs.costs) for r in pb.runningModels ])

