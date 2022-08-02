"""
Display any Crocoddyl OCP problem as a string.
Use it for debug or to assert the similarity of two problems.
"""

from __future__ import print_function

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
    """
    Given a Crocoddyl shooting problem, return a string fully describing it.
    """
    return "".join(
        "*t=%s\n%s" % (t, reprAction(r)) for t, r in enumerate(problem.runningModels)
    ) + "*TERMINAL\n%s" % reprAction(problem.terminalModel)


def printReprProblem(problem, out="/tmp/disp-pb-from-cpp.txt"):
    """
    Same as reprProblem(.) but print the string and write it in a file.
    """
    ret = reprProblem(problem)
    print(ret)
    with open(out, "w") as f:
        print(ret, file=f)
