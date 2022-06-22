#!/usr/bin/env python3
"""
Created on Mon May 23 21:30:51 2022

@author: nvilla
"""

# import time # Time module to sleep()

from sobec import Flex

import configuration as conf

from bullet_Talos import BulletTalos
from cricket.virtual_talos import VirtualPhysics
from pyRobotWrapper import PinTalos
from pyMPC import CrocoWBC
import numpy as np

design = PinTalos(conf)
design.update_reduced(design.q0)

flex = Flex()
flex.initialize(dict(left_stiffness = np.array(conf.H_stiff[:2]),
                     right_stiffness = np.array(conf.H_stiff[2:]),
                     left_damping = np.array(conf.H_damp[:2]),
                     right_damping = np.array(conf.H_damp[2:]),
                     dt = conf.simu_period,
                     MA_duration = 0.01,
                     left_hip_indices = np.array([0, 1, 2]),
                     right_hip_indices = np.array([6, 7, 8])
                     )
                )



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
#        esti_state = wbc.joint_estimation(real_state, command)
        
        qc, dqc = flex.correctEstimatedDeflections(torques, real_state["q"][7:], real_state["dq"][6:])
        
        q_cur = np.hstack([real_state["q"][:7], qc])
        v_cur = np.hstack([real_state["dq"][:6], dqc])
        
        x0_meas = wbc.shapeState(q_cur, v_cur)

#    if s == 900:stop


if conf.simulator == "bullet":
    device.close()
