# import numpy as np
from numpy.linalg import norm
import example_robot_data as robex
import sobec
from sobec.walk.robot_wrapper import RobotWrapper as pyRobotWrapper

urdf = robex.load("talos_legs")
pyrobot = pyRobotWrapper(urdf.model, contactKey="sole_link")


robot = sobec.OCPRobotWrapper(urdf.model, "sole_line", "half_sitting")
robot.model
assert norm(pyrobot.com0 - robot.com0) < 1e-9
assert norm(pyrobot.x0 - robot.x0) < 1e-9

params = sobec.OCPWalkParams()
params.DT

ocp = sobec.OCPWalk(robot, params)
ocp.problem
