import numpy as np
import sobec

np.set_printoptions(precision=3, linewidth=300, suppress=True, threshold=10000)

contact = [[0, 1]] * 10 + [[1, 1]] * 10
pyprof = sobec.walk_without_think.weight_share.weightShareSmoothProfile(contact, 5)
cppprof = sobec.computeWeightShareSmoothProfile(np.array(contact).T, 5, 0.1)
assert np.linalg.norm(cppprof.T - pyprof) < 1e-6

contact = [[0, 1]] * 10 + [[1, 1]] * 10
pyprof = sobec.walk_without_think.weight_share.weightShareSmoothProfile(
    contact, 5, sat=0
)
cppprof = sobec.computeWeightShareSmoothProfile(np.array(contact).T, 5, 0)
assert np.linalg.norm(cppprof.T - pyprof) < 1e-6

contact = [[1, 1]] * 10 + [[0, 1]] * 10
pyprof = sobec.walk_without_think.weight_share.weightShareSmoothProfile(contact, 5)
cppprof = sobec.computeWeightShareSmoothProfile(np.array(contact).T, 5, 0.1)
assert np.linalg.norm(cppprof.T - pyprof) < 1e-6

contact = [[1, 1]] * 10 + [[0, 1]] * 10
pyprof = sobec.walk_without_think.weight_share.weightShareSmoothProfile(
    contact, 5, sat=0
)
cppprof = sobec.computeWeightShareSmoothProfile(np.array(contact).T, 5, 0.0)
assert np.linalg.norm(cppprof.T - pyprof) < 1e-6

contact = [[1, 0]] * 10 + [[1, 1]] * 10 + [[0, 1]] * 10
pyprof = sobec.walk_without_think.weight_share.weightShareSmoothProfile(contact, 5)
cppprof = sobec.computeWeightShareSmoothProfile(np.array(contact).T, 5, 0.1)
assert np.linalg.norm(cppprof.T - pyprof) < 1e-6

contact = [[1, 0]] * 10 + [[1, 1]] * 10 + [[0, 1]] * 10
pyprof = sobec.walk_without_think.weight_share.weightShareSmoothProfile(
    contact, 5, sat=0
)
cppprof = sobec.computeWeightShareSmoothProfile(np.array(contact).T, 5, 0.0)
assert np.linalg.norm(cppprof.T - pyprof) < 1e-6
