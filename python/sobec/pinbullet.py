"""
Wrapper class for load a URDF model in both Pinocchio and Bullet.

Example of use:

models = SimuProxy()
models.loadExampleRobot('talos')
models.loadBulletModel()
models.freeze([
    "arm_left_7_joint",
    "arm_right_7_joint",
    "gripper_left_joint",
    "gripper_right_joint",
    "head_1_joint",
    "head_2_joint",
])
models.setTorqueControlMode()
models.setTalosDefaultFriction()

for i in range(100):
  x = models.getState()
  tau = mycontroller(x)
  models.step(tau)
"""

import pinocchio as pin
import pybullet as pyb
import pybullet_data
import numpy as np
import example_robot_data as robex
import os


class SimuProxy:
    def __init__(self):
        self.readyForSimu = False
        pass

    def loadExampleRobot(self, name):
        inst = robex.ROBOTS[name]()

        self.rmodel = inst.robot.model
        self.gmodel_col = inst.robot.collision_model
        self.gmodel_vis = inst.robot.visual_model

        self.model_rootpath = inst.model_path
        self.urdf_subpath = os.path.join(inst.path, inst.urdf_subpath)
        self.urdf_filename = inst.urdf_filename
        self.srdf_subpath = os.path.join(inst.path, inst.srdf_subpath)
        self.srdf_filename = inst.srdf_filename

        self.setPinocchioFinalizationTricks()

    def setPinocchioFinalizationTricks(self):
        # Add free flyers joint limits ... WHY?
        self.rmodel.upperPositionLimit[:7] = 1
        self.rmodel.lowerPositionLimit[:7] = -1

        self.rmodel.armature = self.rmodel.rotorInertia * self.rmodel.rotorGearRatio**2
        self.rmodel.q0 = self.rmodel.referenceConfigurations["half_sitting"]
        self.rdata = self.rmodel.createData()

    def loadPinocchio(
        self, model_rootpath, urdf_subpath, urdf_filename, srdf_subpath, srdf_filename
    ):
        self.model_rootpath = model_rootpath
        self.urdf_subpath = urdf_subpath
        self.urdf_filename = urdf_filename
        self.srdf_subpath = srdf_subpath
        self.srdf_filename = srdf_filename

        # Load model from URDF
        urdf_path = os.path.join([model_rootpath, urdf_subpath, urdf_filename])
        self.rmodel, self.gmodel_col, self.gmodel_vis = pin.buildModelsFromUrdf(
            urdf_path, model_rootpath, pin.JointModelFreeFlyer()
        )

        # Take rotor inertia and gear ratio into account
        srdf_path = os.path.join([model_rootpath, srdf_subpath, srdf_filename])
        pin.loadRotorParameters(self.rmodel, srdf_path, False)
        pin.loadReferenceConfigurations(self.rmodel, srdf_path, False)

        self.pinocchioFinalizationTricks()

    ################################################################################
    ################################################################################
    # Load bullet model
    def loadBulletModel(self, guiOpt=pyb.DIRECT):
        self.bulletClient = pyb.connect(guiOpt)

        pyb.setTimeStep(1e-3)

        # Set gravity (disabled by default in Bullet)
        pyb.setGravity(*(self.rmodel.gravity.linear.tolist()))

        # Load horizontal plane
        pyb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeId = pyb.loadURDF("plane.urdf")

        pyb.setAdditionalSearchPath(
            os.path.join(self.model_rootpath, self.urdf_subpath)
        )
        self.robotId = pyb.loadURDF(
            self.urdf_filename,
            list(self.rmodel.q0[:3]),
            list(self.rmodel.q0[3:7]),
            useFixedBase=False,
        )

        # Magic translation from bullet where the basis center is shifted
        self.localInertiaPos = pyb.getDynamicsInfo(self.robotId, -1)[3]

        self.setBulletFinalizationTrics()

        for ipin, ibul in enumerate(self.bulletCtrlJointsInPinOrder):
            pyb.enableJointForceTorqueSensor(1, ipin, True)
            pyb.resetJointState(self.robotId, ibul, self.rmodel.q0[ipin + 7])

    def setBulletFinalizationTrics(self):
        self.bullet_names2indices = {
            pyb.getJointInfo(1, i)[1].decode(): i for i in range(pyb.getNumJoints(1))
        }
        self.bulletCtrlJointsInPinOrder = [
            self.bullet_names2indices[n] for n in self.rmodel.names[2:]
        ]

    ################################################################################

    def setTalosDefaultFriction(self):
        self.changeFriction(["leg_left_6_joint", "leg_right_6_joint"], 100, 30)

    #        self.changeFriction(["leg_left_5_joint", "leg_right_5_joint"], 100, 30)

    def changeFriction(self, names, lateralFriction=100, spinningFriction=30):
        for n in names:
            idx = self.bullet_names2indices[n]
            pyb.changeDynamics(
                self.robotId,
                idx,
                lateralFriction=lateralFriction,
                spinningFriction=spinningFriction,
            )

    def freeze(self, jointNames):
        jointIds = [i for (i, n) in enumerate(self.rmodel.names) if n in jointNames]

        self.rmodel_full = self.rmodel
        self.rmodel, [self.gmodel_col, self.gmodel_vis] = pin.buildReducedModel(
            self.rmodel_full,
            [self.gmodel_col, self.gmodel_vis],
            jointIds,
            self.rmodel.q0,
        )

        self.setPinocchioFinalizationTricks()
        self.setBulletFinalizationTrics()

    def setTorqueControlMode(self):
        """
        Disable default position controler in torque controlled joints
        Default controller will take care of other joints
        """
        pyb.setJointMotorControlArray(
            self.robotId,
            jointIndices=self.bulletCtrlJointsInPinOrder,
            controlMode=pyb.VELOCITY_CONTROL,
            forces=[0.0 for _ in self.bulletCtrlJointsInPinOrder],
        )
        self.readyForSimu = True

    ################################################################################
    ################################################################################

    def step(self, torques):
        assert self.readyForSimu
        pyb.setJointMotorControlArray(
            self.robotId,
            self.bulletCtrlJointsInPinOrder,
            controlMode=pyb.TORQUE_CONTROL,
            forces=torques,
        )
        pyb.stepSimulation()

    def getState(self):
        # Get articulated joint pos and vel
        xbullet = pyb.getJointStates(self.robotId, self.bulletCtrlJointsInPinOrder)
        q = [x[0] for x in xbullet]
        vq = [x[1] for x in xbullet]

        # Get basis pose
        p, quat = pyb.getBasePositionAndOrientation(self.robotId)
        # Get basis vel
        v, w = pyb.getBaseVelocity(self.robotId)

        # Concatenate into a single x vector
        x = np.concatenate([p, quat, q, v, w, vq])

        # Magic transformation of the basis translation, as classical in Bullet.
        x[:3] -= self.localInertiaPos

        return x


# ##############################################################################
# ##############################################################################
# ############   ##     ##    ###    #### ##    ##   ###########################
# ############   ###   ###   ## ##    ##  ###   ##   ###########################
# ############   #### ####  ##   ##   ##  ####  ##   ###########################
# ############   ## ### ## ##     ##  ##  ## ## ##   ###########################
# ############   ##     ## #########  ##  ##  ####   ###########################
# ############   ##     ## ##     ##  ##  ##   ###   ###########################
# ############   ##     ## ##     ## #### ##    ##   ###########################
# ##############################################################################
# ##############################################################################

if __name__ == "__main__":
    print(" Lets try with Talos")
    models = SimuProxy()
    models.loadExampleRobot("talos")
    models.loadBulletModel()
    models.freeze(
        [
            # "universe",
            # "arm_left_1_joint",
            # "arm_left_2_joint",
            # "arm_left_3_joint",
            # "arm_left_4_joint",
            "arm_left_5_joint",
            "arm_left_6_joint",
            "arm_left_7_joint",
            # "arm_right_1_joint",
            # "arm_right_2_joint",
            # "arm_right_3_joint",
            # "arm_right_4_joint",
            "arm_right_5_joint",
            "arm_right_6_joint",
            "arm_right_7_joint",
            "gripper_left_joint",
            "gripper_right_joint",
            "head_1_joint",
            "head_2_joint",
        ]
    )
