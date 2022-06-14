import numpy as np
import params


class WalkParams(params.WalkParams):
    DT = 0.010

    # Inherited from params.WalkParams
    stateTerminalWeight = 20
    #    saveFile = '/tmp/mpc.npy'
    saveFile = None

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
    impactVelocityWeight = 10 * 1000  # 10
    # impactRotationWeight = 50    #5

    # groundColWeight = 0
    conePenaltyWeight = 0

    # feetCollisionWeight = 200
    # footMinimalDistance = 0.3  # (.17 is the max value wrt initial config)
    # copWeight = .5

    # New parameters
    Tstart = int(0.3 / DT)
    Tsingle = int(0.8 / DT)  # 60
    Tdouble = int(0.11 / DT)  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.6 / DT)  # 120

    refFootFlyingAltitude = 7e-2
    flyHighSlope = 3 / refFootFlyingAltitude
    flyWeight = 10 * 20
    baumgartGains = np.array([0, 100])

    vcomRef = np.array([0.05, 0, 0])
    # vcomWeight = 2
    # comWeight = 1000  # 20
    vcomImportance = np.array([0.0, 0, 1])

    maxiter = 2
    solver_reg_min = 1e-6

    torque_noise = 0.0  # max magnitude of the multiplicative joint torque noise, expressed as a percentage (i.e. 1=100%)

    ### DEBUG
    showPreview = False
