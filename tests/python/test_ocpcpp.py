import numpy as np
from numpy.linalg import norm
import example_robot_data as robex
import sobec
from sobec.walk.robot_wrapper import RobotWrapper as pyRobotWrapper
from sobec.walk.params import WalkParams as pyWalkParams
import sobec.walk.ocp as pyOCPWalk
from sobec.walk.miscdisp import reprProblem

# --- ROBOT WRAPPER
print("*** Py robot")
pyurdf = robex.load("talos_legs")
pyurdf.model.name = "talos"
pyrobot = pyRobotWrapper(pyurdf.model, contactKey="sole_link")

print("*** C++ robot")
urdf = robex.load("talos_legs")
urdf.model.name = "talos"
robot = sobec.OCPRobotWrapper(urdf.model, "sole_link", "half_sitting")
robot.model
assert norm(pyrobot.com0 - robot.com0) < 1e-9
assert norm(pyrobot.x0 - robot.x0) < 1e-9

# --- PARAMS
print("*** Py params")
pyparams = pyWalkParams(pyrobot.name)

print("*** C++ params")
params = sobec.OCPWalkParams()
params.DT

for k,v in pyparams.__dict__.items():
    if hasattr(params,k):
        try:
            if k[:2]!='__':
                params.__setattr__(k,v)
        except ArgumentError:
            print('*** ', k,' cannot be allocated to ',v)
    else:
        print(k,' is not a field of params')
for k,v in pyparams.__class__.__dict__.items():
    if hasattr(params,k):
        try:
            if k[:2]!='__':
                params.__setattr__(k,v)
        except ArgumentError:
            print('*** ', k,' cannot be allocated to ',v)
    else:
        print(k,' is not a field of params')



# --- CONTACT PATTERN
print("*** Contacts")
pycontactPattern = (
    []
    + [[1, 1]] * 10
    + [[1, 0]] * 20
    + [[1, 1]] * 11
    + [[1, 1]]
)
contactPattern = np.array(pycontactPattern).T


# --- OCP
print("*** Py OCP")
pyddp = pyOCPWalk.buildSolver(pyrobot,pycontactPattern,pyparams)

print("*** C++ OCP")
ocp = sobec.OCPWalk(robot, params, contactPattern)
ocp.buildSolver()

with open('/tmp/py.txt', 'w') as f: f.write(reprProblem(pyddp.problem))
with open('/tmp/cpp.txt', 'w') as f: f.write(reprProblem(ocp.problem))

print('*** You can now run: \n\t\tdiff /tmp/py.txt /tmp/cpp.txt')
