import numpy as np
from numpy.linalg import norm
import example_robot_data as robex
import sobec
import sobec.walk
from sobec.walk.robot_wrapper import RobotWrapper as pyRobotWrapper
from sobec.walk.params import WalkParams as pyWalkParams
import sobec.walk.ocp as pyOCPWalk
from sobec.walk.miscdisp import reprProblem
from sobec.walk.yaml_params import yamlWriteParams

# --- ROBOT WRAPPER
pyurdf = robex.load("talos_legs")
pyurdf.model.name = "talos"
pyrobot = pyRobotWrapper(pyurdf.model, contactKey="sole_link")
urdf = robex.load("talos_legs")
urdf.model.name = "talos"
robot = sobec.OCPRobotWrapper(urdf.model, "sole_link", "half_sitting")

assert norm(pyrobot.com0 - robot.com0) < 1e-9
assert norm(pyrobot.x0 - robot.x0) < 1e-9

# --- PARAMS

pyparams = pyWalkParams(pyrobot.name)
params = sobec.OCPWalkParams()

pyparams.minimalNormalForce = 50

# TODO. The following option is deactivated. When activated, the cone-penalty,
# containing infinite bounds, is anormally detected as not matching. Here is an
# example of a false-negative detection:

"""
  - right_sole_link_cone: {w=0.1, CostModelResidual {ResidualModelContactWrenchCone {frame=right_sole_link, mu=1000, box=[0.1, 0.1]}, ActivationModelQuadraticBarrier {nr=17}}}
		lower = [-1.79769313e+308 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
  5.00000000e+001 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
 -1.79769313e+308]
		upper = [-1.79769313e+308 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
  5.00000000e+001 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308 -1.79769313e+308
 -1.79769313e+308]
		ref =          R: 1 0 0
0 1 0
0 0 1
"""  # noqa E501

"""
# Force all 0 cost to be activated to a small value, to strengthen this test
# (otherwise, the cost is not added to the OCP hence not tested).
for k in dir(pyparams):
    if k[-6:]=='Weight' and getattr(pyparams,k)==0.:
        setattr(pyparams,k,.1)
"""


# TODO: this could be added as another test. I mean: the following commented
# code is converted the pyparam into c++ param by manual copy (which is ugly coded
# btw). I now rewrite it by going through yaml file.
"""
for k, v in pyparams.__dict__.items():
    if hasattr(params, k):
        # TODO: "ArgumentError" doesn't exist, does it ? In argparse maybe ?
        # try:
        if k[:2] != "__":
            params.__setattr__(k, v)
        # except ArgumentError:
        # print("*** ", k, " cannot be allocated to ", v)
    else:
        print(k, " is not a field of params")
for k, v in pyparams.__class__.__dict__.items():
    if hasattr(params, k):
        # try:
        if k[:2] != "__":
            params.__setattr__(k, v)
        # except ArgumentError:
        # print("*** ", k, " cannot be allocated to ", v)
    else:
        print(k, " is not a field of params")
"""
yamlWriteParams("/tmp/test_ocpcpp.yml", pyparams)
params.readFromYaml("/tmp/test_ocpcpp.yml")

# --- CONTACT PATTERN
pycontactPattern = (
    []
    + [[1, 1]] * pyparams.Tdouble
    + [[1, 0]] * pyparams.Tsingle
    + [[1, 1]] * pyparams.Tdouble
    + [[1, 1]]
)
contactPattern = np.array(pycontactPattern).T

# --- OCP
pyocp = pyOCPWalk.buildSolver(pyrobot, pycontactPattern, pyparams)
ocp = sobec.OCPWalk(robot, params, contactPattern)
ocp.buildSolver()

# with open('/tmp/py.txt', 'w') as f: f.write(reprProblem(pyocp.problem))
# with open('/tmp/cpp.txt', 'w') as f: f.write(reprProblem(ocp.problem))
# print('*** You can now run: \n\t\tdiff /tmp/py.txt /tmp/cpp.txt')

pyserial = reprProblem(pyocp.problem)
cppserial = reprProblem(ocp.problem)

fs = np.array([np.concatenate(f) for f in ocp.referenceForces])
import matplotlib.pylab as plt  # noqa: E402,F401

if pyserial != cppserial:
    with open("/tmp/cpp.txt", "w") as f:
        f.write(reprProblem(ocp.problem))
    with open("/tmp/py.txt", "w") as f:
        f.write(reprProblem(pyocp.problem))
assert pyserial == cppserial
