#!/usr/bin/env python3
"""
Created on Mon May  9 18:22:56 2022

@author: nvilla
"""

import pybullet_data
import pybullet as p  # PyBullet simulator
import numpy as np


class BulletTalos:
    def __init__(self, conf, rmodelComplete):
        p.connect(p.GUI)  # Start the client for PyBullet
        p.setTimeStep(conf.simu_step)
        p.setGravity(*conf.gravity.tolist())  # Set gravity (disabled by default)

        # place CoM of root link ## TODO: check placement
        robotStartPosition = [0.0, 0.0, 1.01927]
        robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        p.setAdditionalSearchPath(conf.modelPath + "/talos_data/robots/")

        self.robotId = p.loadURDF(
            conf.URDF_FILENAME,
            robotStartPosition,
            robotStartOrientation,
            useFixedBase=False,
        )

        # Load horizontal plane
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        self.localInertiaPos = p.getDynamicsInfo(self.robotId, -1)[
            3
        ]  # of the base link

        # leg_left (45-50), leg_right (52-57), torso (0-1), arm_left (11-17),
        # gripper_left (21), arm_right (28-34), gripper_right (38), head (3,4).
        bulletJointNames = [
            p.getJointInfo(self.robotId, i)[1].decode()
            for i in range(p.getNumJoints(self.robotId))
        ]
        self.JointIndicesComplete = [
            bulletJointNames.index(rmodelComplete.names[i])
            for i in range(2, rmodelComplete.njoints)
        ]

        # Joints controlled with crocoddyl
        self.bulletControlledJoints = [
            i
            for i in self.JointIndicesComplete
            if p.getJointInfo(self.robotId, i)[1].decode() not in conf.blocked_joints
        ]

        # Disable default position controler in torque controlled joints
        # Default controller will take care of other joints
        p.setJointMotorControlArray(
            self.robotId,
            jointIndices=self.bulletControlledJoints,
            controlMode=p.VELOCITY_CONTROL,
            forces=[0.0 for m in self.bulletControlledJoints],
        )

        # Augment friction to forbid feet sliding
        p.changeDynamics(self.robotId, 50, lateralFriction=100, spinningFriction=30)
        p.changeDynamics(self.robotId, 57, lateralFriction=100, spinningFriction=30)

    def initializeJoints(self, q0CompleteStart):
        # Initialize position in pyBullet
        initial_joint_positions = np.array(q0CompleteStart[7:].flat).tolist()
        for i in range(len(initial_joint_positions)):
            p.enableJointForceTorqueSensor(self.robotId, i, True)
            p.resetJointState(
                self.robotId, self.JointIndicesComplete[i], initial_joint_positions[i]
            )

    def execute(self, torques):
        p.setJointMotorControlArray(
            self.robotId,
            self.bulletControlledJoints,
            controlMode=p.TORQUE_CONTROL,
            forces=torques,
        )
        p.stepSimulation()

    def measureState(self):
        jointStates = p.getJointStates(
            self.robotId, self.JointIndicesComplete
        )  # State of all joints
        baseState = p.getBasePositionAndOrientation(self.robotId)
        baseVel = p.getBaseVelocity(self.robotId)

        # Joint vector for Pinocchio
        q = np.hstack(
            [
                baseState[0],
                baseState[1],
                [jointStates[i_joint][0] for i_joint in range(len(jointStates))],
            ]
        )
        v = np.hstack(
            [
                baseVel[0],
                baseVel[1],
                [jointStates[i_joint][1] for i_joint in range(len(jointStates))],
            ]
        )

        q[:3] -= self.localInertiaPos
        return q, v

    def showTargetToTrack(self, LF_pose, RF_pose):
        visualShapeTarget = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.1, 0.075, 0.001],
            rgbaColor=[0.0, 0.0, 1.0, 1.0],
            specularColor=[0.4, 0.4, 0],
            visualFramePosition=[0.0, 0.0, 0.0],
        )

        self.sphereIdRight = p.createMultiBody(
            baseMass=0.0,
            baseInertialFramePosition=[0, 0, 0],
            baseVisualShapeIndex=visualShapeTarget,
            basePosition=[
                RF_pose.translation[0],
                RF_pose.translation[1],
                RF_pose.translation[2],
            ],
            useMaximalCoordinates=True,
        )

        self.sphereIdLeft = p.createMultiBody(
            baseMass=0.0,
            baseInertialFramePosition=[0, 0, 0],
            baseVisualShapeIndex=visualShapeTarget,
            basePosition=[
                LF_pose.translation[0],
                LF_pose.translation[1],
                LF_pose.translation[2],
            ],
            useMaximalCoordinates=True,
        )

        self.visualShapeTargetCom = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.05, 0.05, 0.05],
            rgbaColor=[0.0, 1.0, 0.0, 1.0],
            specularColor=[0.4, 0.4, 0],
            visualFramePosition=[0.0, 0.0, 0.0],
        )

    def moveMarkers(self, LF_pose, RF_pose):

        p.resetBasePositionAndOrientation(
            self.sphereIdRight,
            posObj=[RF_pose.translation[0], RF_pose.translation[1], 0],
            ornObj=np.array([0.0, 0.0, 0.0, 1.0]),
        )
        p.resetBasePositionAndOrientation(
            self.sphereIdLeft,
            posObj=[LF_pose.translation[0], LF_pose.translation[1], 0],
            ornObj=np.array([0.0, 0.0, 0.0, 1.0]),
        )

    def close(self):
        p.disconnect()


if __name__ == "__main__":

    import configuration as config
    from pin_Talos import PinTalos

    design = PinTalos(config)

    o = BulletTalos(config, design.rmodelComplete)
    o.initializeJoints(design.q0Complete)

    o.close()
