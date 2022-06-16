import example_robot_data as robex
import pinocchio as pin
from sobec.walk.talos_collections import jointToLockCollection

jointToLockNames = jointToLockCollection['talos_low']

def load():
    robot = robex.load("talos")

    jointToLockIds = [
        i for (i, n) in enumerate(robot.model.names) if n in jointToLockNames
    ]
    robot.model, [robot.collision_model, robot.visual_model] = pin.buildReducedModel(
        robot.model,
        [robot.collision_model, robot.visual_model],
        jointToLockIds,
        robot.q0,
    )
    robot.q0 = robot.model.referenceConfigurations["half_sitting"]
    robot.rebuildData()

    return robot
