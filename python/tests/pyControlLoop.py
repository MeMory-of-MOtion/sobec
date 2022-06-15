#!/usr/bin/env python3
"""
Created on Mon May 23 21:30:51 2022

@author: nvilla
"""

# import time # Time module to sleep()

import configuration as conf

from bullet_Talos import BulletTalos
from cricket.virtual_talos import VirtualPhysics
from pyRobotWrapper import PinTalos
from pyMPC import CrocoWBC

design = PinTalos(conf)
design.update_reduced(design.q0)

wbc = CrocoWBC(conf, design)

if conf.simulator == "bullet":
    device = BulletTalos(conf, design.rmodelComplete)
    device.initializeJoints(design.q0Complete.copy())
    device.showTargetToTrack(wbc.LF_sample, wbc.RF_sample)
    q_current, v_current = device.measureState()

elif conf.simulator == "pinocchio":

    device = VirtualPhysics(conf, view=True, block_joints=conf.blocked_joints)
    device.initialize(design.rmodelComplete)
    q_current, v_current = device.Cq0, device.Cv0

x0_meas = wbc.shapeState(q_current, v_current)

for s in range(conf.T_total * conf.Nc):
    #    time.sleep(0.001)
    torques = wbc.iterate(s, x0_meas)

    if conf.simulator == "bullet":
        device.execute(torques)
        q_current, v_current = device.measureState()
        device.moveMarkers(wbc.LF_sample, wbc.RF_sample)
        x0_meas = wbc.shapeState(q_current, v_current)

    elif conf.simulator == "pinocchio":

        correct_contacts = wbc.get_current_contact()
        command = {"tau": torques}
        real_state, _ = device.execute(command, correct_contacts, s)
        esti_state = wbc.joint_estimation(real_state, command)
        x0_meas = wbc.shapeState(esti_state["q"], esti_state["dq"])

#    if s == 900:stop


if conf.simulator == "bullet":
    device.close()
