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
        str += f"  - {citem.key()}: {citem.data()}\n"
    str += "=== Cost\n"
    for citem in dmodel.costs.costs:
        str += f"  - {citem.key()}: {citem.data()}\n"
        cost = citem.data().cost
        if isinstance(cost.activation, croc.ActivationModelWeightedQuad):
            str += f"\t\twact = {cost.activation.weights}\n"
        if isinstance(cost.activation, croc.ActivationModelQuadraticBarrier):
            str += f"\t\tlower = {cost.activation.bounds.lb}\n"
            str += f"\t\tupper = {cost.activation.bounds.lb}\n"
        try:
            str += f"\t\tref = {cost.residual.reference}\n"
        except AttributeError:
            pass
    return str


def reprProblem(problem):
    return (
        "".join(f"*t={t}\n{reprAction(r)}" for t, r in enumerate(problem.runningModels))
        + f"*TERMINAL\n{reprAction(problem.terminalModel)}"
    )


def contact2car(model, contactIds, contacts, costs):
    """Represent an ocp as one line of ASCII caracters for contacts and impacts"""
    left = "⎻"
    right = "_"
    leftplus = "┘"
    rightplus = "┌"
    res = ""
    if len(contacts) == 2:
        if f"{model.frames[contactIds[1]].name}_altitudeimpact" in costs:
            res += ","
        if f"{model.frames[contactIds[0]].name}_altitudeimpact" in costs:
            res += "'"
        res += "="
    elif len(contacts) == 0:
        res += " "
    elif f"{model.frames[contactIds[1]].name}" in next(iter(contacts)).key():
        if f"{model.frames[contactIds[1]].name}_altitudeimpact" in costs:
            res += rightplus
        else:
            res += right
    elif f"{model.frames[contactIds[0]].name}" in next(iter(contacts)).key():
        if f"{model.frames[contactIds[0]].name}_altitudeimpact" in costs:
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
