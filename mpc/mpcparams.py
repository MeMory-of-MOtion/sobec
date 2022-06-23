import sobec.walk.params as swparams
import numpy as np


def roundToEven(xfloat):
    return 2 * int(np.round(xfloat / 2 - 0.5001)) + 1


class WalkParams(swparams.WalkParams):
    DT = 0.015
    Tstart = int(0.3 / DT)
    Tsingle = int(0.8 / DT)  # 60
    # I prefer an even number for Tdouble
    Tdouble = roundToEven(0.11 / DT)  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.4 / DT)  # 1.6
    vcomRef = np.array([0.05, 0, 0])
    Tsimu = 1500

    def __init__(self, name="talos_low"):
        swparams.WalkParams.__init__(self, name)
        # super(swparams.WalkParams, self).__init__(self, name)


class PushParams(swparams.WalkParams):
    DT = 0.015
    Tstart = int(0.3 / DT)
    Tsingle = int(0.6 / DT)  # 60
    # I prefer an even number for Tdouble
    Tdouble = roundToEven(0.11 / DT)  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.4 / DT)  # 1.6

    vcomRef = np.array([0.0, 0, 0])
    solver_maxiter = 2
    comWeight = 100
    solver_th_stop = 1e-2

    def __init__(self, name="talos_low"):
        swparams.WalkParams.__init__(self, name)
        # super(swparams.WalkParams, self).__init__(self, name)


class StandParams(swparams.WalkParams):
    """MPC Params with not single support, hence standing straight"""

    DT = 0.01
    Tsimu = 200
    Tstart = 50
    Tsingle = 1  # int(0.8 / DT)
    # I prefer an even number for Tdouble
    Tdouble = roundToEven(71)
    Tend = 50
    Tmpc = 80
    transitionDuration = (Tdouble - 1) // 2

    vcomRef = np.array([0, 0, 0])
    vcomImportance = np.array([0.0, 3, 1])
    vcomWeight = 20

    refStateWeight = 1e-1
    forceImportance = np.array([1, 1, 0.1, 10, 10, 2])
    coneAxisWeight = 5e-5  # 2e-4
    copWeight = 3  # 2
    centerOfFrictionWeight = 1  # 2
    refForceWeight = 10  # 10

    minimalNormalForce = 50
    withNormalForceBoundOnly = True
    conePenaltyWeight = 0

    refTorqueWeight = 0
    comWeight = 1  # 20
    verticalFootVelWeight = 10
    flyHighWeight = 0
    groundColWeight = 0

    feetCollisionWeight = 0
    impactAltitudeWeight = 2000  # 20000
    impactVelocityWeight = 200  # 10000
    impactRotationWeight = 100  # 200
    refMainJointsAtImpactWeight = 0  # 2e2 # For avoinding crossing legs

    stateTerminalWeight = 20  # 2000
    solver_maxiter = 10

    def __init__(self, name="talos_legs"):
        swparams.WalkParams.__init__(self, name)
        # super(swparams.WalkParams, self).__init__(self, name)
