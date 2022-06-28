# -*- coding: utf-8 -*-
"""
Some simple methods to display an OCP problem as string.
"""

from __future__ import print_function

import crocoddyl as croc


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
    def __init__(self, contactIds):
        croc.CallbackAbstract.__init__(self)
        self.contactIds = contactIds

    def __call__(self, solver):
        print(dispocp(solver.problem, self.contactIds))
