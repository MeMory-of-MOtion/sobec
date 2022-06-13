import numpy as np
import params


class WalkParams(params.WalkParams):

    # Inherited from params.WalkParams
    stateTerminalWeight = 20
    saveFile = "/tmp/mpc.npy"

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

    # impactAltitudeWeight = 1000  #100
    impactVelocityWeight = 1000  # 10
    # impactRotationWeight = 50    #5

    # groundColWeight = 0
    conePenaltyWeight = 0

    # feetCollisionWeight = 200
    # footMinimalDistance = 0.3  # (.17 is the max value wrt initial config)
    # copWeight = .5

    # New parameters
    T_START = 30
    T_SINGLE = 60
    T_DOUBLE = 11
    T_END = 30
    Tmpc = 120

    refFootFlyingAltitude = 7e-2
    flyHighSlope = 3 / refFootFlyingAltitude
    baumgartGains = np.array([0, 50])

    VCOM_TARGET = np.array([0.05, 0, 0])
    # vcomWeight = 2
    # comWeight = 1000  # 20
    vcomImportance = np.array([0.0, 0, 1])

    maxiter = 1
    solver_reg_min = 1e-6
