import example_robot_data as robex
import pinocchio as pin

# Collection of list of joint names, to be locked when loading the full talos urdf
# model.
# See talos_low for an example of use.
jointToLockCollection = {
    "talos_low": [
        # "universe",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        # "arm_left_4_joint",
        "arm_left_5_joint",
        "arm_left_6_joint",
        "arm_left_7_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        # "arm_right_4_joint",
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "gripper_left_joint",
        "gripper_right_joint",
        "head_1_joint",
        "head_2_joint",
        "torso_1_joint",
        "torso_2_joint",
    ],
    "talos_legs": [
        # "universe",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_left_5_joint",
        "arm_left_6_joint",
        "arm_left_7_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "gripper_left_joint",
        "gripper_right_joint",
        "head_1_joint",
        "head_2_joint",
        "torso_1_joint",
        "torso_2_joint",
    ],
    "pyrene_legs": [
        # "universe",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_left_5_joint",
        "arm_left_6_joint",
        "arm_left_7_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "gripper_left_joint",
        "gripper_right_joint",
        "head_1_joint",
        "head_2_joint",
        "torso_1_joint",
        "torso_2_joint",
        "gripper_left_inner_double_joint",
        "gripper_left_fingertip_1_joint",
        "gripper_left_fingertip_2_joint",
        "gripper_left_inner_single_joint",
        "gripper_left_fingertip_3_joint",
        "gripper_left_motor_single_joint",
        "gripper_right_inner_double_joint",
        "gripper_right_fingertip_1_joint",
        "gripper_right_fingertip_2_joint",
        "gripper_right_inner_single_joint",
        "gripper_right_fingertip_3_joint",
        "gripper_right_motor_single_joint",
    ],
}


def jointNamesToIds(names, model):
    return [i for (i, n) in enumerate(model.names) if n in names]


def robexLoadAndReduce(urdfName, jointToLock_key):
    """Load a robot model from example robot data, using the urdfName
    (e.g 'talos') and reduce it by locking the joints, following the names
    chosend with jointToLock_key (e.g. 'talos_low').
    """
    robot = robex.load(urdfName)

    jointToLockNames = jointToLockCollection[jointToLock_key]
    jointToLockIds = jointNamesToIds(jointToLockNames, robot.model)
    robot.model, [robot.collision_model, robot.visual_model] = pin.buildReducedModel(
        robot.model,
        [robot.collision_model, robot.visual_model],
        jointToLockIds,
        robot.q0,
    )
    robot.q0 = robot.model.referenceConfigurations["half_sitting"]
    robot.rebuildData()

    return robot


if __name__ == "__main__":
    robot = robexLoadAndReduce("talos", "talos_low")
