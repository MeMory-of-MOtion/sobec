import sobec.walk.params as swparams
import numpy as np


class WalkParams(swparams.WalkParams):
    DT = 0.015
    Tstart = int(0.3 / DT)
    Tsingle = int(0.8 / DT)  # 60
    # I prefer an even number for Tdouble
    Tdouble = 2 * int(np.round(0.11 / DT / 2 - 0.75)) + 1  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.4 / DT)  # 1.6

    vcomRef = np.array([0.05, 0, 0])

    def __init__(self, name="talos_low"):
        swparams.WalkParams.__init__(self, name)
        # super(swparams.WalkParams, self).__init__(self, name)


class StandParams(swparams.WalkParams):
    """MPC Params with not single support, hence standing straight"""

    DT = 0.01
    Tsimu = 500
    Tstart = 50
    Tsingle = 1  # int(0.8 / DT)
    # I prefer an even number for Tdouble
    Tdouble = 51  # 2 * int(np.round(0.11 / DT / 2 - 0.75)) + 1  # 11
    Tend = 50
    Tmpc = 60
    transitionDuration = (Tdouble - 1) // 2

    vcomRef = np.array([0, 0, 0])
    vcomImportance = np.array([0.0, 0, 1])
    vcomWeight = 2

    refStateWeight = 1e-1
    forceImportance = np.array([1, 1, 0.1, 10, 10, 2])
    coneAxisWeight = 5e-5  # 2e-4
    copWeight = 2  # 2
    refForceWeight = 20  # 10

    minimalNormalForce = 50
    withNormalForceBoundOnly = True
    conePenaltyWeight = 1

    refTorqueWeight = 0
    comWeight = 10  # 20
    verticalFootVelWeight = 0
    flyHighWeight = 0
    groundColWeight = 0
    conePenaltyWeight = 0
    feetCollisionWeight = 0
    impactAltitudeWeight = 2000  # 20000
    impactVelocityWeight = 200  # 10000
    impactRotationWeight = 100  # 200
    refMainJointsAtImpactWeight = 0  # 2e2 # For avoinding crossing legs

    stateTerminalWeight = 20  # 2000
    maxiter = 10

    def __init__(self, name="talos_legs"):
        swparams.WalkParams.__init__(self, name)
        # super(swparams.WalkParams, self).__init__(self, name)
