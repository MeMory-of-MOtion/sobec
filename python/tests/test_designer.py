#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  7 13:07:30 2022

@author: nvilla
"""


import sobec as so
import configuration as conf

settings = dict(urdfPath= conf.modelPath + conf.URDF_SUBPATH,
                srdfPath= conf.modelPath + conf.SRDF_SUBPATH,
                leftFootName = conf.lf_frame_name,
                rightFootName = conf.rf_frame_name,
                controlledJointsNames= ["root_joint",
                                        "leg_left_1_joint",
                                        "leg_left_2_joint",
                                        "leg_left_3_joint",
                                        "leg_left_4_joint",
                                        "leg_left_5_joint",
                                        "leg_right_1_joint",
                                        "leg_right_2_joint",
                                        "leg_right_3_joint",
                                        "leg_right_4_joint",
                                        "leg_right_5_joint",
                                        ], robotDescription="")

designer = so.RobotDesigner()
designer.initialize(settings)

