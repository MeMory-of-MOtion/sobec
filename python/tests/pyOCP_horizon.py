#!/usr/bin/env python3
"""
Created on Wed May 25 11:51:23 2022

@author: nvilla
"""
import crocoddyl
import numpy as np


class ReceidingHorizon:
    def __init__(self, conf, design, x0, modeller, N, *forms):
        """
        ARGUMENTS:

            *forms : is a list of dictionaries with arguments for the modeller
            design : manages all what is needed from pinnochio in the modeller

        """
        if len(forms) == 0:
            forms = [{}] * N
        elif len(forms) == 1:
            forms = forms * N

        # Data structure
        state = crocoddyl.StateMultibody(design.rmodel)
        actuation = crocoddyl.ActuationModelFloatingBase(state)

        ia_models = [
            modeller(conf, design, state, actuation, **(forms[i])) for i in range(N)
        ]

        problem = crocoddyl.ShootingProblem(x0, ia_models, ia_models[-1])

        self.ddp = crocoddyl.SolverFDDP(problem)

        self.conf = conf
        self.state = state
        self.actuation = actuation

    def IAM(self, time):
        return self.ddp.problem.runningModels[time]

    def DAM(self, time):
        return self.IAM(time).differential

    def costs(self, time):
        return self.DAM(time).costs

    def contacts(self, time):
        return self.DAM(time).contacts

    def data(self, time):
        return self.ddp.problem.runningDatas[time]

    def set_residual_reference(self, time, name, new_value):
        self.costs(time).costs[name].cost.residual.reference = new_value

    def set_residual_references(self, name, new_value, set_times=None):
        """Times is a range or a list of times"""

        times = range(self.ddp.problem.T) if set_times is None else set_times
        for time in times:
            self.set_residual_reference(time, name, new_value)

    def set_gravity_compensation(self, time, x=None):
        if x is None:
            x = self.costs(time).costs["state"].cost.residual.reference
        gravity_compensation = self.IAM(time).quasiStatic(self.data(time), x)
        self.set_residual_reference(time, "control", gravity_compensation)

    def activate_contact_LF(self, time):
        self.contacts(time).changeContactStatus(self.conf.leftFoot, True)

    def activate_contact_RF(self, time):
        self.contacts(time).changeContactStatus(self.conf.rightFoot, True)

    def remove_contact_LF(self, time):
        self.contacts(time).changeContactStatus(self.conf.leftFoot, False)

    def remove_contact_RF(self, time):
        self.contacts(time).changeContactStatus(self.conf.rightFoot, False)

    def set_force_reference_LF(self, time, ref_wrench):
        cone = self.costs(time).costs["left_wrench_cone"]
        new_ref = np.dot(cone.cost.residual.reference.A, ref_wrench)
        cone.cost.activation.reference = new_ref

    def set_force_reference_RF(self, time, ref_wrench):
        cone = self.costs(time).costs["right_wrench_cone"]
        new_ref = np.dot(cone.cost.residual.reference.A, ref_wrench)
        cone.cost.activation.reference = new_ref

    def set_swinging_LF(self, time):
        self.remove_contact_LF(time)
        self.set_force_reference_LF(time, np.zeros(6))

    def set_swinging_RF(self, time):
        self.remove_contact_RF(time)
        self.set_force_reference_RF(time, np.zeros(6))

    def set_supporting_LF(self, time):
        self.activate_contact_LF(time)

    def set_supporting_RF(self, time):
        self.activate_contact_RF(time)

    def get_contacts(self, time):
        active = self.contacts(time).active_set
        return {
            self.conf.leftFoot: self.conf.leftFoot in active,
            self.conf.rightFoot: self.conf.rightFoot in active,
        }

    def preview_state(self):
        return self.ddp.xs

    def preview_actions(self):
        return self.ddp.us

    def recede(self, new_model=None, new_data=None):
        aux_model = new_model if new_model else self.IAM(0)
        aux_data = (
            new_data
            if new_data
            else (new_model.createData() if new_model else self.data(0))
        )
        self.ddp.problem.circularAppend(aux_model, aux_data)

    @property
    def size(self):
        return self.ddp.problem.T

        # printers

    def print_cost_names(self, time):
        print(self.costs(time).costs.todict().keys())

    def print_contacts(self):
        for i in range(self.ddp.problem.T):
            print("model ", i, " contacts : \n", self.get_contacts(i), "\n")

    def print_res_reference(self, name):
        for i in range(self.ddp.problem.T):
            print(
                "model ",
                i,
                " contacts : \n",
                self.costs(i).costs[name].cost.residual.reference,
                "\n",
            )


if __name__ == "__main__":
    import configuration as config
    from pyRobotWrapper import PinTalos
    from pyModelMaker import modeller

    design = PinTalos(config)
    x0 = np.hstack([design.q0, np.zeros(design.rmodel.nv)])
    o = ReceidingHorizon(config, design, x0, modeller, config.T)
