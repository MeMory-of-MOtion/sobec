#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  8 15:14:55 2022

@author: nvilla
"""


import sobec as so
import configuration as conf
import crocoddyl 

## Checking robot Wrapper.
design_conf = dict(urdfPath= conf.modelPath + conf.URDF_SUBPATH,
                srdfPath= conf.modelPath + conf.SRDF_SUBPATH,
                leftFootName = conf.lf_frame_name,
                rightFootName = conf.rf_frame_name,
                robotDescription="",
                controlledJointsNames= ["root_joint",
                                        "leg_left_1_joint",
                                        "leg_left_2_joint",
                                        "leg_left_3_joint",
                                        "leg_left_4_joint",
                                        "leg_left_5_joint",
                                        "leg_left_6_joint",
                                        "leg_right_1_joint",
                                        "leg_right_2_joint",
                                        "leg_right_3_joint",
                                        "leg_right_4_joint",
                                        "leg_right_5_joint",
                                        "leg_right_6_joint",
                                        "torso_1_joint",
                                        "torso_2_joint",
                                        "arm_left_1_joint",
                                        "arm_left_2_joint",
                                        "arm_left_3_joint",
                                        "arm_left_4_joint",
                                        "arm_right_1_joint",
                                        "arm_right_2_joint",
                                        "arm_right_3_joint",
                                        "arm_right_4_joint"])

design = so.RobotDesigner()
design.initialize(design_conf)

## Checking model formulation
MM_conf = dict(timeStep = conf.DT,
               gravity = conf.gravity,
               mu = conf.mu,
               coneBox = conf.cone_box,
               minNforce = conf.minNforce,
               maxNforce = conf.maxNforce,
               comHeight = conf.normal_height,
               omega = conf.omega,
               wFootPlacement = conf.wFootPlacement,
               wStateReg = conf.wStateReg,
               wControlReg = conf.wControlReg,
               wLimit = conf.wLimit,
               wVCoM = conf.wVCoM,
               wWrenchCone = conf.wWrenchCone,
               wFootTrans = conf.wFootTrans,
               wFootXYTrans = conf.wFootXYTrans,
               wFootRot = conf.wFootRot,
               wGroundCol = conf.wGroundCol,
               stateWeights = conf.stateWeights,
               controlWeights = conf.controlWeight,
               th_grad = conf.th_grad,
               th_stop = conf.th_stop)

modeller = so.ModelMaker()
modeller.initialize(MM_conf, design)

ds_iam = modeller.formulateStepTracker()

supports = [so.Support.DOUBLE]*100
all_models = modeller.formulateHorizon(supports)

# Checking individual costs and contacts:
state = crocoddyl.StateMultibody(design.get_rModel())
actuation = crocoddyl.ActuationModelFloatingBase(state)

contacts = crocoddyl.ContactModelMultiple(state, actuation.nu)
costs = crocoddyl.CostModelSum(state, actuation.nu)

modeller.defineActuationTask(costs)
modeller.defineFeetTracking(costs)

modeller.defineFeetContact(contacts)


# Checking horizon_manager
H_conf = dict(leftFootName = conf.lf_frame_name, 
              rightFootName = conf.rf_frame_name)
horizon = so.HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, all_models[-1])

# Checking the WBC
import numpy as np
wbc_conf = dict(horizonSteps = conf.preview_steps,
                totalSteps = conf.total_steps,
                T = conf.T,
                TdoubleSupport = conf.T2contact,
                TsingleSupport = conf.T1contact,
                Tstep = conf.Tstep,
                ddpIteration = conf.ddpIteration,
                Dt = conf.DT,
                simu_step = conf.simu_period,
                Nc = conf.Nc)

wbc = so.WBC()
wbc.initialize(wbc_conf, 
               design, 
               horizon, 
               design.get_q0Complete(), 
               design.get_v0Complete())
