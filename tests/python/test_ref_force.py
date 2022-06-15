import sobec
from sobec.walk import weight_share as ws



contact = [ [ 0, 1] * 10  + [1,1]*10 ]
pyprof = ws.weightShareSmoothProfile(contact,5)

sobec.weightShareSmoothProfile

