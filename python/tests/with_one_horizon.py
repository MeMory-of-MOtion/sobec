#!/usr/bin/env python3
"""
Created on Thu May 19 18:47:13 2022

@author: nvilla
"""

import crocoddyl

import numpy as np
import pinocchio as pin

from formulation_step_tracker import modeller
from horizon_manager import ReceidingHorizon


class CrocoWBC:
    def __init__(self, conf, design):

        x0 = np.hstack([design.q0, np.zeros(design.rmodel.nv)])

        self.horizon = ReceidingHorizon(conf, design, x0, modeller, conf.T)
        for time in range(self.horizon.ddp.problem.T):
            self.horizon.set_gravity_compensation(time)

        self.design = design
        self.conf = conf

        self.x0 = x0
        self.delta = self.delta_dot = np.zeros(4)

        self.fz_ref = -design.get_robot_mass() * conf.gravity[2]

        # timing
        self.current_time = 0

        ds_start = 100
        self.t_takeoff_RF = np.arange(0, 6 * conf.Tstep, 2 * conf.Tstep) + ds_start
        self.t_takeoff_LF = self.t_takeoff_RF + conf.Tstep
        self.t_land_RF = self.t_takeoff_RF + conf.T1contact
        self.t_land_LF = self.t_takeoff_LF + conf.T1contact

        self.xForward = conf.xForward

        self.design.update_reduced(x0[: self.design.rmodel.nq])
        self.LF_sample = design.get_LF_frame().copy()
        self.RF_sample = design.get_RF_frame().copy()

        self.initialize_horizon_contacts()
        self.initialize_horizon_foot_trajectory()

    def shapeState(self, q_current, v_current):
        qc = q_current[[i + 5 for i in self.design.pinocchioControlledJoints[1:]]]
        vc = v_current[[i + 4 for i in self.design.pinocchioControlledJoints[1:]]]

        return np.hstack((q_current[:7], qc, v_current[:6], vc))

    def get_current_contact(self):
        return self.horizon.get_contacts(0)

    def update_step_cycle_timing(self):
        self.t_land_LF -= 1
        self.t_land_RF -= 1
        self.t_takeoff_LF -= 1
        self.t_takeoff_RF -= 1

        if self.t_land_LF[0] < 0:
            self.t_land_LF += 2 * self.conf.Tstep
        if self.t_land_RF[0] < 0:
            self.t_land_RF += 2 * self.conf.Tstep
        if self.t_takeoff_LF[0] < 0:
            self.t_takeoff_LF += 2 * self.conf.Tstep
        if self.t_takeoff_RF[0] < 0:
            self.t_takeoff_RF += 2 * self.conf.Tstep

    def time_for_croco_solve(self, time):  # Note, it is a discrete time.
        return not time % self.conf.Nc

    def decide_actions(self, x0, is_feasible):
        warm_xs = self.horizon.ddp.xs[1:].tolist() + [self.horizon.ddp.xs[-1]]
        warm_xs[0] = x0
        warm_us = self.horizon.ddp.us[1:].tolist() + [self.horizon.ddp.us[-1]]

        self.horizon.ddp.problem.x0 = x0
        self.horizon.ddp.solve(warm_xs, warm_us, self.conf.ddpIteration, is_feasible)

    def iterate(self, time, x0, OL_MPC=False):

        if self.time_for_croco_solve(time):
            # ~~ timing ~~#
            self.update_step_cycle_timing()
            self.horizon.recede()

            # ~~ Update references ~~ #
            self.design.update_reduced(x0[: self.design.rmodel.nq])
            self.set_contacts(self.conf.T - 1)
            #            self.update_feet_ref_frame()
            self.set_desired_feet_placement(self.conf.T - 1)

            # ~~ decide ~~ #
            self.decide_actions(x0, OL_MPC)

        # ~~ funtion to obtain x_ref by integrating xs[0] with us[0] during one tracking
        # period.
        torques = self.horizon.ddp.us[0] + self.horizon.ddp.K[
            0
        ] @ self.horizon.state.diff(x0, self.horizon.ddp.xs[0])

        return torques

    def planned_contacts_at(self, preview_time):

        contacts = self.horizon.get_contacts(preview_time - 1)
        if preview_time in self.t_land_LF:
            contacts[self.conf.leftFoot] = True

        if preview_time in self.t_land_RF:
            contacts[self.conf.rightFoot] = True

        if preview_time in self.t_takeoff_LF:
            contacts[self.conf.leftFoot] = False

        if preview_time in self.t_takeoff_RF:
            contacts[self.conf.rightFoot] = False

        return contacts

    def set_contacts(self, preview_time):

        contacts = self.planned_contacts_at(preview_time)

        if contacts[self.conf.leftFoot] and contacts[self.conf.rightFoot]:
            self.horizon.set_supporting_LF(preview_time)
            self.horizon.set_supporting_RF(preview_time)

            self.horizon.set_force_reference_LF(
                preview_time, np.array([0, 0, self.fz_ref / 2, 0, 0, 0])
            )
            self.horizon.set_force_reference_RF(
                preview_time, np.array([0, 0, self.fz_ref / 2, 0, 0, 0])
            )
            print("update to double contact model ", preview_time)

        elif contacts[self.conf.leftFoot]:
            self.horizon.set_supporting_LF(preview_time)
            self.horizon.set_swinging_RF(preview_time)

            self.horizon.set_force_reference_LF(
                preview_time, np.array([0, 0, self.fz_ref, 0, 0, 0])
            )
            print("update to supporting left contact model ", preview_time)

        elif contacts[self.conf.rightFoot]:
            self.horizon.set_swinging_LF(preview_time)
            self.horizon.set_supporting_RF(preview_time)

            self.horizon.set_force_reference_RF(
                preview_time, np.array([0, 0, self.fz_ref, 0, 0, 0])
            )
            print("update to supporting right contact model ", preview_time)

    def initialize_horizon_contacts(self):
        for i in range(1, self.conf.T):
            self.set_contacts(i)

    def update_feet_ref_frame(self):
        if self.t_takeoff_LF[0] == 0:
            orientationLF = self.design.get_LF_frame().copy().rotation.T
            frame_LF = crocoddyl.WrenchCone(
                orientationLF,
                self.conf.mu,
                self.conf.cone_box,
                4,
                True,
                self.conf.minNforce,
                self.conf.maxNforce,
            )

            if self.t_takeoff_LF[0] < self.conf.T:
                support_end = self.t_takeoff_LF[0]
            else:
                support_end = self.conf.T

            self.horizon.set_residual_references(
                "left_wrench_cone", frame_LF, range(0, support_end)
            )

        if self.t_takeoff_RF[0] == 0:
            orientationRF = self.design.get_RF_frame().copy().rotation.T
            frame_RF = crocoddyl.WrenchCone(
                orientationRF,
                self.conf.mu,
                self.conf.cone_box,
                4,
                True,
                self.conf.minNforce,
                self.conf.maxNforce,
            )

            if self.t_takeoff_RF[0] < self.conf.T:
                support_end = self.t_takeoff_RF[0]
            else:
                support_end = self.conf.T

            self.horizon.set_residual_references(
                "right_wrench_cone", frame_RF, range(0, support_end)
            )

    def initialize_horizon_foot_trajectory(self):
        for i in range(1, self.conf.T):
            self.set_desired_feet_placement(i)

    def set_desired_feet_placement(self, preview_time):

        planned_land_LFs = self.t_land_LF[self.t_land_LF > preview_time]
        planned_land_RFs = self.t_land_RF[self.t_land_RF > preview_time]

        if planned_land_LFs.size == 0:
            next_land_LF = self.conf.T1contact + 1
        else:
            next_land_LF = planned_land_LFs[0]

        if planned_land_RFs.size == 0:
            next_land_RF = self.conf.T1contact + 1
        else:
            next_land_RF = planned_land_RFs[0]

        LF = self.LF_sample.copy()
        RF = self.RF_sample.copy()

        LF.translation = self.foot_trajectory(next_land_LF - preview_time, LF)
        RF.translation = self.foot_trajectory(next_land_RF - preview_time, RF)
        #        print(self.RF_sample.translation)
        self.horizon.set_residual_reference(preview_time, "left_foot_place", LF.copy())
        self.horizon.set_residual_reference(preview_time, "right_foot_place", RF.copy())

    def foot_trajectory(self, time_to_land, pose, trajectory="sine"):
        tmax = self.conf.T1contact
        t = time_to_land - 4  # minus landing_advance

        if trajectory == "sine":
            # SINE
            z = 0 if t < 0 or t > tmax - 8 else (np.sin(t * np.pi / (tmax - 8))) * 0.04
        else:
            # SMOTHER
            z = (
                0
                if t < 0 or t > tmax - 8
                else (1 - np.cos(2 * t * np.pi / (tmax - 8))) * 0.02
            )

        result = np.array([pose.translation[0], pose.translation[1], z])
        return result

    # ##### FLEXIBILITY
    def estimate_deflection(self, command, hip_yawls=[0, 0]):
        flexing_idx = [2, 1, 8, 7]
        L_yawl = pin.utils.rotate("z", hip_yawls[0])[0:2, 0:2].T
        R_yawl = pin.utils.rotate("z", hip_yawls[1])[0:2, 0:2].T

        self.flex_torque = np.hstack(
            [
                L_yawl @ command["tau"][flexing_idx[0:2]],
                R_yawl @ command["tau"][flexing_idx[2:4]],
            ]
        )
        tau = self.flex_torque

        delta = compute_deflection(
            tau, self.delta, self.conf.H_stiff, self.conf.H_damp, self.conf.simu_period
        )
        delta_dot = (delta - self.delta) / self.conf.simu_period

        #        if self.conf.flex_esti_delay:
        #            delta_dot = self.movAverage(delta_dot, 0.01, "flex_rate")

        return delta, delta_dot

    def take_deflection(self, device):
        pass

    def joint_estimation(self, real_state, command=None, device=None):
        if self.conf.model_name == "talos" or not self.conf.compensate_deflections:
            return real_state.copy()

        if self.conf.exact_deflection:
            self.delta, self.delta_dot = self.take_deflection(device)
        else:
            hip_yawl = real_state["q"][[7, 13]]
            self.delta, self.delta_dot = self.estimate_deflection(command, hip_yawl)

        if self.conf.model_name == "talos_flex":
            esti_q, esti_dq = translate_hips(
                real_state["q"].copy(),
                real_state["dq"].copy(),
                self.delta,
                self.delta_dot,
            )
        return {"q": esti_q, "dq": esti_dq}


def compute_deflection(tau, delta0, k, d, Dt):
    return (delta0 * d / Dt - tau) / (k + d / Dt)


def translate_hips(q, dq, delta, delta_dot):
    rigid_q = q.copy()
    rigid_dq = dq.copy()
    # Left hip
    left_flex_Y = pin.utils.rotate("y", delta[0])
    left_flex_X = pin.utils.rotate("x", delta[1])

    left_Z = pin.utils.rotate("z", q[7])
    left_X = pin.utils.rotate("x", q[8])
    left_Y = pin.utils.rotate("y", q[9])

    Rla = left_flex_Y
    Rlb = Rla @ left_flex_X
    Rlc = Rlb @ left_Z
    Rld = Rlc @ left_X
    Rle = Rld @ left_Y

    rigid_q[[7, 8, 9]] = solve_hip_joints(Rle)

    # velocity
    w_l = [0, delta_dot[0], 0]
    w_l += Rla @ [delta_dot[1], 0, 0]
    w_l += Rlb @ [0, 0, dq[6]]
    w_l += Rlc @ [dq[7], 0, 0]
    w_l += Rld @ [0, dq[8], 0]

    Rela = pin.utils.rotate("z", rigid_q[7])
    Relb = Rela @ pin.utils.rotate("x", rigid_q[8])

    M_l = np.hstack(
        [np.array([0, 0, 1])[:, None], Rela[:, 0][:, None], Relb[:, 1][:, None]]
    )
    rigid_dq[[6, 7, 8]] = np.linalg.inv(M_l) @ w_l

    # Right hip
    right_flex_Y = pin.utils.rotate("y", delta[2])
    right_flex_X = pin.utils.rotate("x", delta[3])

    right_Z = pin.utils.rotate("z", q[13])
    right_X = pin.utils.rotate("x", q[14])
    right_Y = pin.utils.rotate("y", q[15])

    Rra = right_flex_Y
    Rrb = Rra @ right_flex_X
    Rrc = Rrb @ right_Z
    Rrd = Rrc @ right_X
    Rre = Rrd @ right_Y

    rigid_q[[13, 14, 15]] = solve_hip_joints(Rre)

    # velocity
    w_r = [0, delta_dot[2], 0]
    w_r += Rra @ [delta_dot[3], 0, 0]
    w_r += Rrb @ [0, 0, dq[12]]
    w_r += Rrc @ [dq[13], 0, 0]
    w_r += Rrd @ [0, dq[14], 0]

    Rera = pin.utils.rotate("z", rigid_q[13])
    Rerb = Rera @ pin.utils.rotate("x", rigid_q[14])

    M_r = np.hstack(
        [np.array([0, 0, 1])[:, None], Rera[:, 0][:, None], Rerb[:, 1][:, None]]
    )
    rigid_dq[[12, 13, 14]] = np.linalg.inv(M_r) @ w_r

    return rigid_q, rigid_dq


def solve_hip_joints(equivalence):
    """Solves the problem:
                R(q2)R(q3)R(q4) = equivalence, with shape([3, 3])
    and returns q2, q3, q4
    """
    q2 = np.arctan2(-equivalence[0, 1], equivalence[1, 1])
    q3 = np.arctan2(
        equivalence[2, 1],
        -equivalence[0, 1] * np.sin(q2) + equivalence[1, 1] * np.cos(q2),
    )
    q4 = np.arctan2(-equivalence[2, 0], equivalence[2, 2])
    return q2, q3, q4


# ########### END FLEXIBILITY ##########


if __name__ == "__main__":

    import configuration as config
    from pin_Talos import PinTalos

    design = PinTalos(config)
    wbc = CrocoWBC(config, design)
