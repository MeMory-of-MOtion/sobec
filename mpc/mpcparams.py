import numpy as np
import params

class WalkParams(params.WalkParams):

    # Inherited from params.WalkParams
    stateTerminalWeight = 20
    saveFile = '/tmp/mpc.npy'

    basisQWeight = [0, 0, 0, 50, 50, 0]
    legQWeight = [5, 5, 1, 2, 1, 1]
    torsoQWeight = [10, 10]
    armQWeight = [3, 3]
    basisVWeight = [0, 0, 0, 3, 3, 1]  # ## was 003331
    legVWeight = [1] * 6
    torsoVWeight = [20] * 2
    armVWeight = [2] * 2

    stateImportance = np.array(
        basisQWeight
        + legQWeight
        + legQWeight
        + armQWeight
        + basisVWeight
        + legVWeight
        + legVWeight
        + armVWeight
    )

    # New parameters
    T_START = 30
    T_SINGLE = 50
    T_DOUBLE = 11
    T_END = 30
    Tmpc = 100
    
    maxiter = 200
