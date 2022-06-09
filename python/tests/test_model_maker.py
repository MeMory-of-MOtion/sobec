#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  8 15:14:55 2022

@author: nvilla
"""


import sobec as so
import configuration as conf


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




