import sobec.walk_without_think.params as swparams
import numpy as np


def roundToOdd(xfloat):
    """
    For double support, it is better to take an odd integer
    (due to the particular way reference forces are smoothed).
    """
    return 2 * int(np.round(xfloat / 2 - 0.5001)) + 1


class WalkOCPParams(swparams.WalkParams):
    """
    Params quickly tune for the OCP example (ie single traj optim,
    no MPC). I guess we could discard this set and only use the next
    one (but I am lazy to do that now -- yeah, I know, but it is friday
    night and I am tired).
    """

    conePenaltyWeight = 20
    flyHighWeight = 20
    impactVelocityWeight = 200
    refFootFlyingAltitude = 0.03
    flyHighSlope = 5 / refFootFlyingAltitude
    vcomRef = np.array([0.1, 0, 0])
    baumgartGains = np.array([0, 50])
    minimalNormalForce = 0.0

    def __init__(self, name="talos_low"):
        swparams.WalkParams.__init__(self, name)


class WalkParams(swparams.WalkParams):
    """
    Specialization of the basic parameters for the MPC example of this folder.
    With that, the robot should walk nicely in open loop MPC (receding horizon)
    but also in closed-loop in bullet.
    """

    DT = 0.015
    Tstart = int(0.3 / DT)
    Tsingle = int(0.8 / DT)  # 60
    # I prefer an odd number for Tdouble
    Tdouble = roundToOdd(0.11 / DT)  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.4 / DT)  # 1.6
    vcomRef = np.array([0.05, 0, 0])
    Tsimu = 15

    def __init__(self, name="talos_low"):
        swparams.WalkParams.__init__(self, name)
        # super(swparams.WalkParams, self).__init__(self, name)


class StandParams(swparams.WalkParams):
    """MPC Params with not single support, hence standing straight"""

    DT = 0.01
    Tsimu = 200
    Tstart = 50
    Tsingle = 1  # int(0.8 / DT)
    # I prefer an odd number for Tdouble
    Tdouble = roundToOdd(71)
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


class PushParams(swparams.WalkParams):
    """
    Roughly tuned example for the push-and-recover demo.
    These parameters should not be kept on the long range.
    """

    DT = 0.015
    Tstart = int(0.3 / DT)
    Tsingle = int(0.6 / DT)  # 60
    # I prefer an odd number for Tdouble
    Tdouble = roundToOdd(0.11 / DT)  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.4 / DT)  # 1.6
    Tsimu = 500

    vcomRef = np.array([0.0, 0, 0])
    solver_maxiter = 2
    comWeight = 100
    solver_th_stop = 1e-2

    def __init__(self, name="talos_low"):
        swparams.WalkParams.__init__(self, name)
        # super(swparams.WalkParams, self).__init__(self, name)
