"""
Some simple methods to display an OCP problem as string.
"""

import crocoddyl as croc


def reprAction(amodel):
    """Save all the action details in a text file."""
    str = ""
    dmodel = amodel.differential
    str += "=== Contact\n"
    for citem in dmodel.contacts.contacts:
        str += "  - %s: %s\n" % (citem.key(), citem.data())
    str += "=== Cost\n"
    for citem in dmodel.costs.costs:
        str += "  - %s: %s\n" % (citem.key(), citem.data())
        cost = citem.data().cost
        if isinstance(cost.activation, croc.ActivationModelWeightedQuad):
            str += "\t\twact = %s\n" % cost.activation.weights
        if isinstance(cost.activation, croc.ActivationModelQuadraticBarrier):
            str += "\t\tlower = %s\n" % cost.activation.bounds.lb
            str += "\t\tupper = %s\n" % cost.activation.bounds.lb
        try:
            str += "\t\tref = %s\n" % cost.residual.reference
        except AttributeError:
            pass
    return str


def reprProblem(problem):
    return "".join(
        "*t=%s\n%s" % (t, reprAction(r)) for t, r in enumerate(problem.runningModels)
    ) + "*TERMINAL\n%s" % {reprAction(problem.terminalModel)}


def contact2car(model, contactIds, contacts, costs):
    """Represent an ocp as one line of ASCII caracters for contacts and impacts"""
    left = "⎻"
    right = "_"
    leftplus = "┘"
    rightplus = "┌"
    res = ""
    if len(contacts) == 2:
        if "%s_altitudeimpact" % model.frames[contactIds[1]].name in costs:
            res += ","
        if "%s_altitudeimpact" % model.frames[contactIds[0]].name in costs:
            res += "'"
        res += "="
    elif len(contacts) == 0:
        res += " "
    elif model.frames[contactIds[1]].name in next(iter(contacts)).key():
        if "%s_altitudeimpact" % model.frames[contactIds[1]].name in costs:
            res += rightplus
        else:
            res += right
    elif model.frames[contactIds[0]].name in next(iter(contacts)).key():
        if "%s_altitudeimpact" % model.frames[contactIds[0]].name in costs:
            res += leftplus
        else:
            res += left
    else:
        res += "*"
    return res


def dispocp(pb, contactIds):
    return "".join(
        [
            contact2car(
                r.differential.pinocchio,
                contactIds,
                r.differential.contacts.contacts,
                r.differential.costs.costs,
            )
            for r in pb.runningModels
        ]
    )

class CallbackMPCWalk(croc.CallbackAbstract):
    def __init__(self,contactIds):
        croc.CallbackAbstract.__init__(self)
        self.contactIds = contactIds
    def __call__(self, solver):
        print(dispocp(solver.problem,self.contactIds))
