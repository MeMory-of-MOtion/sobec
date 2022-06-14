#!/usr/bin/env python3
"""
Created on Sat Jun 11 14:44:53 2022

@author: nvilla
"""


import unittest

# import numpy as np

from pyRobotWrapper import PinTalos
from pyOCP_horizon import ReceidingHorizon
from pyMPC import CrocoWBC
from pyModelMaker import modeller
import numpy as np

# bindings
from sobec import RobotDesigner, HorizonManager, WBC, ModelMaker, Support

import configuration as config


class ClasesTestCase(unittest.TestCase):
    def setUp(self):

        py_design = PinTalos(config)
        temp_design = PinTalos(config)
        py_mpc = CrocoWBC(config, temp_design)
        x0 = py_mpc.shapeState(py_design.rmodelComplete.q0, py_design.rmodelComplete.v0)
        py_horizon = ReceidingHorizon(config, py_design, x0, modeller, config.T)

        # ############# Bindings ##############
        design_conf = dict(
            urdfPath=config.modelPath + config.URDF_SUBPATH,
            srdfPath=config.modelPath + config.SRDF_SUBPATH,
            leftFootName=config.lf_frame_name,
            rightFootName=config.rf_frame_name,
            robotDescription="",
            controlledJointsNames=[
                "root_joint",
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
                "arm_right_4_joint",
            ],
        )
        design = RobotDesigner()
        design.initialize(design_conf)

        # model formulation -- to create Horizon
        MM_conf = dict(
            timeStep=config.DT,
            gravity=config.gravity,
            mu=config.mu,
            coneBox=config.cone_box,
            minNforce=config.minNforce,
            maxNforce=config.maxNforce,
            comHeight=config.normal_height,
            omega=config.omega,
            wFootPlacement=config.wFootPlacement,
            wStateReg=config.wStateReg,
            wControlReg=config.wControlReg,
            wLimit=config.wLimit,
            wVCoM=config.wVCoM,
            wWrenchCone=config.wWrenchCone,
            wFootTrans=config.wFootTrans,
            wFootXYTrans=config.wFootXYTrans,
            wFootRot=config.wFootRot,
            wGroundCol=config.wGroundCol,
            stateWeights=config.stateWeights,
            controlWeights=config.controlWeight,
            th_grad=config.th_grad,
            th_stop=config.th_stop,
        )

        formuler = ModelMaker()
        formuler.initialize(MM_conf, design)

        supports = [Support.DOUBLE] * config.T
        all_models = formuler.formulateHorizon(supports)

        # Horizon

        # Checking horizon_manager
        H_conf = dict(
            leftFootName=config.lf_frame_name, rightFootName=config.rf_frame_name
        )
        horizon = HorizonManager()
        horizon.initialize(H_conf, design.get_x0(), all_models, all_models[-1])

        # MPC

        wbc_conf = dict(
            horizonSteps=config.preview_steps,
            totalSteps=config.total_steps,
            T=config.T,
            TdoubleSupport=config.T2contact,
            TsingleSupport=config.T1contact,
            Tstep=config.Tstep,
            ddpIteration=config.ddpIteration,
            Dt=config.DT,
            simu_step=config.simu_period,
            Nc=config.Nc,
        )

        mpc = WBC()
        mpc.initialize(
            wbc_conf,
            design,
            horizon,
            design.get_q0Complete(),
            design.get_v0Complete(),
            "actuationTask",
        )

        self.design = design
        self.py_design = py_design
        self.horizon = horizon
        self.py_horizon = py_horizon
        self.mpc = mpc
        self.py_mpc = py_mpc
        self.formuler = formuler
        self.Tstep = wbc_conf["Tstep"]

    def test_RobotWrapper(self):
        self.py_design.update_reduced(self.py_design.rmodel.q0)
        self.design.updateReducedModel(self.py_design.rmodel.q0)

        self.assertEqual(self.design.getRobotMass(), self.py_design.get_robot_mass())
        self.assertTrue(
            (
                self.py_design.get_LF_frame().homogeneous
                == self.design.get_LF_frame().homogeneous
            ).all()
        )
        self.assertTrue(
            (
                self.py_design.get_RF_frame().homogeneous
                == self.design.get_RF_frame().homogeneous
            ).all()
        )
        self.assertEqual(
            self.py_design.pinocchioControlledJoints,
            self.design.get_controlledJointsIDs().tolist(),
        )
        self.assertEqual(self.py_design.leftFootId, self.design.get_LF_id())
        self.assertEqual(self.py_design.rightFootId, self.design.get_RF_id())
        self.assertTrue((self.py_design.q0 == self.design.get_q0()).all())
        self.assertTrue(
            (self.py_design.q0Complete == self.design.get_q0Complete()).all()
        )
        self.assertEqual(self.py_design.contactNames[0], self.design.get_LF_name())
        self.assertEqual(self.py_design.contactNames[1], self.design.get_RF_name())
        self.assertTrue(
            (self.py_design.rmodel.defaultState == self.design.get_x0()).all()
        )
        self.assertEqual(self.py_design.rmodel.nv, self.design.get_rModel().nv)
        self.assertEqual(self.py_design.rmodel.nq, self.design.get_rModel().nq)
        self.assertEqual(
            self.py_design.rmodelComplete.nv, self.design.get_rModelComplete().nv
        )
        self.assertEqual(
            self.py_design.rmodelComplete.nq, self.design.get_rModelComplete().nq
        )

        self.py_design.update_reduced(self.py_design.rmodel.q0)
        self.design.updateReducedModel(self.py_design.rmodel.q0 * 0)

        self.assertTrue(
            not (
                self.py_design.get_LF_frame().homogeneous
                == self.design.get_LF_frame().homogeneous
            ).all()
        )
        self.assertTrue(
            not (
                self.py_design.get_RF_frame().homogeneous
                == self.design.get_RF_frame().homogeneous
            ).all()
        )

    def test_OCP(self):

        self.assertEqual(self.horizon.iam(0).dt, self.py_horizon.IAM(0).dt)
        self.assertEqual(self.horizon.iam(0).nr, self.py_horizon.IAM(0).nr)
        self.assertEqual(self.horizon.iam(0).nu, self.py_horizon.IAM(0).nu)
        self.assertEqual(self.horizon.ddp.problem.T, self.py_horizon.ddp.problem.T)
        self.assertEqual(self.horizon.size(), self.py_horizon.ddp.problem.T)

        model0_0 = self.horizon.ama(0)
        model0_1 = self.horizon.ama(1)
        model0_end = self.horizon.ama(self.horizon.size() - 1)

        self.horizon.recede()

        model1_0 = self.horizon.ama(0)
        model1_1 = self.horizon.ama(1)
        model1_end = self.horizon.ama(self.horizon.size() - 1)

        self.assertTrue(model0_0 is not model0_1)
        self.assertTrue(model0_0 is not model0_end)
        self.assertTrue(model0_end is not model1_end)
        self.assertTrue(model0_1 is not model1_1)
        self.assertTrue(model0_0 is model1_end)
        self.assertTrue(model1_0 is model0_1)

    #        self.assertTrue(self.horizon.ddp.solve())

    def test_MPC(self):

        nq = self.design.get_rModelComplete().nq
        nv = self.design.get_rModelComplete().nv
        q = np.random.rand(nq)
        v = np.random.rand(nv)
        self.assertTrue(
            (self.mpc.shapeState(q, v) == self.py_mpc.shapeState(q, v)).all()
        )

        self.mpc.generateFullCycle(self.formuler)
        self.assertTrue(isinstance(self.mpc.walkingCycle, HorizonManager))
        self.assertEqual(self.mpc.walkingCycle.ddp.problem.T, 2 * self.Tstep)


if __name__ == "__main__":
    unittest.main()
