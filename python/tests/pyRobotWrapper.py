#!/usr/bin/env python3
"""
Created on Tue May 10 15:44:50 2022

@author: nvilla
"""

import pinocchio as pin

import numpy as np
import hppfcl


class PinTalos:
    def __init__(self, conf):

        # ## COMPLETE MODEL ##

        rmodelComplete = pin.buildModelsFromUrdf(
            conf.modelPath + conf.URDF_SUBPATH,
            conf.modelPath,
            pin.JointModelFreeFlyer(),
        )[0]

        # Take rotor inertia and gear ratio into account
        pin.loadRotorParameters(
            rmodelComplete, conf.modelPath + conf.SRDF_SUBPATH, False
        )
        rmodelComplete.armature = np.multiply(
            rmodelComplete.rotorInertia.flat,
            np.square(rmodelComplete.rotorGearRatio.flat),
        )
        pin.loadReferenceConfigurations(
            rmodelComplete, conf.modelPath + conf.SRDF_SUBPATH, False
        )

        # Add free flyer joint limits
        rmodelComplete.upperPositionLimit[:7] = 1
        rmodelComplete.lowerPositionLimit[:7] = -1

        q0Complete = rmodelComplete.referenceConfigurations["half_sitting"]
        rdataComplete = rmodelComplete.createData()

        # ## REDUCED MODEL ##

        pinocchioControlledJoints = [
            i
            for (i, n) in enumerate(rmodelComplete.names)
            if n not in conf.blocked_joints
        ]
        # 1-6 leg_left, 7-12 leg_right, 13-14 torso, 15-21 arm_left, 22 gripper_left,
        # 23-29 arm_right, 30 gripper_right, 31-32 head if using talos_reduced
        JointsToLockId = [
            i
            for i in range(1, rmodelComplete.njoints)
            if i not in pinocchioControlledJoints
        ]

        # Create reduced model
        rmodel = pin.buildReducedModel(rmodelComplete, JointsToLockId, q0Complete)
        rdata = rmodel.createData()
        pin.loadRotorParameters(rmodel, conf.modelPath + conf.SRDF_SUBPATH, False)
        rmodel.armature = np.multiply(
            rmodel.rotorInertia.flat, np.square(rmodel.rotorGearRatio.flat)
        )

        # Load reference configuration
        # pin.loadReferenceConfigurations(rmodel, conf.modelPath + conf.SRDF_SUBPATH)
        q0 = rmodel.referenceConfigurations["half_sitting"]

        rmodel.defaultState = np.hstack(
            [q0, np.zeros(rmodel.nv)]
        )  # TODO: remove this default state and use a varing state.

        self.pinocchioControlledJoints = pinocchioControlledJoints
        self.rightFootId = rmodel.getFrameId(conf.rightFoot)
        self.leftFootId = rmodel.getFrameId(conf.leftFoot)

        self.contactNames = [conf.leftFoot, conf.rightFoot]

        self.rmodelComplete = rmodelComplete
        self.rdataComplete = rdataComplete
        self.q0Complete = q0Complete

        self.rmodel = rmodel
        self.rdata = rdata
        self.q0 = q0
        self.rmodel.q0 = q0
        self.rmodel.v0 = np.zeros(rmodel.nv)
        self.rmodelComplete.v0 = np.zeros(rmodelComplete.nv)
        self.rmodelComplete.q0 = q0Complete

        # some pinochio attributes:
        self.WORLD = pin.WORLD
        self.LOCAL = pin.LOCAL

    def load_ground_collision_model(self):
        self.geomModel = pin.GeometryModel()

        se3ObsPose = pin.SE3.Identity()
        se3ObsPose.translation = np.array([0, 0, -0.05])  # [0,0,-0.05]

        # Defining ground
        ig_obs_ground = self.geomModel.addGeometryObject(
            pin.GeometryObject(
                "ground",
                self.rmodel.getFrameId("universe"),
                self.rmodel.frames[self.rmodel.getFrameId("universe")].parent,
                hppfcl.Box(3, 3, 0.0),
                se3ObsPose,
            ),
            self.rmodel,
        )

        # Defining feet
        ig_foot_left = self.geomModel.addGeometryObject(
            pin.GeometryObject(
                "left_foot",
                self.leftFootId,
                self.rmodel.frames[self.leftFootId].parent,
                hppfcl.Sphere(0),
                self.get_LF_frame().copy(),
            ),
            self.rmodel,
        )

        ig_foot_right = self.geomModel.addGeometryObject(
            pin.GeometryObject(
                "right_foot",
                self.rightFootId,
                self.rmodel.frames[self.rightFootId].parent,
                hppfcl.Sphere(0),
                self.get_RF_frame().copy(),
            ),
            self.rmodel,
        )

        self.geomModel.addCollisionPair(pin.CollisionPair(ig_foot_right, ig_obs_ground))
        self.geomModel.addCollisionPair(pin.CollisionPair(ig_foot_left, ig_obs_ground))

    def get_LF_frame(self):
        return self.rdata.oMf[self.leftFootId]

    def get_RF_frame(self):
        return self.rdata.oMf[self.rightFootId]

    def update_reduced(self, q):

        pin.forwardKinematics(self.rmodel, self.rdata, q)
        pin.updateFramePlacements(self.rmodel, self.rdata)

    def update_complete(self, q):

        pin.forwardKinematics(self.rmodelComplete, self.rdataComplete, q)
        pin.updateFramePlacements(self.rmodelComplete, self.rdataComplete)

    def get_robot_mass(self):
        M = sum(iner.mass for iner in self.rmodel.inertias.tolist())
        return M


if __name__ == "__main__":
    import configuration as config

    u = PinTalos(config)
