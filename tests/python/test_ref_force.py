import numpy as np
import sobec
from sobec.walk_without_think import weight_share as ws

np.set_printoptions(precision=3, linewidth=300, suppress=True, threshold=10000)

contact = [[0, 1]] * 10 + [[1, 1]] * 10
contact = [[1, 1]] * 10 + [[0, 1]] * 10
pyprof = ws.weightShareSmoothProfile(contact, 4)

cppprof = sobec.computeWeightShareSmoothProfile(np.array(contact).T, 4)
