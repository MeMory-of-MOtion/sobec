import numpy as np


class WalkParams:
    DT = 0.010

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

    stateTerminalImportance = np.array([3, 3, 0, 0, 0, 30] + [0] * 14 + [1] * 20)

    legUWeight = [1, 1, 1, 1, 10, 10]
    torsoUWeight = [1, 1]
    armUWeight = [10, 10]
    controlImportance = np.array(legUWeight * 2 + armUWeight)

    # ## Gains for force continuity: wfref for tracking the reference, wfcont for time
    # difference
    refTorqueWeight = 0
    refStateWeight = 1e-1
    flatBaseWeight = 0  # 20
    forceImportance = np.array([1, 1, 0.1, 10, 10, 2])
    coneAxisWeight = 2e-4
    comWeight = 0  # 20
    vcomImportance = np.array([0.0, 0, 1])
    vcomWeight = 1
    acomWeight = 0  # 16*DT
    copWeight = 2
    verticalFootVelWeight = 20
    footVelWeight = 0  # 20
    footAccWeight = 0  # 2
    flyWeight = 20
    groundColWeight = 200
    conePenaltyWeight = 20
    feetCollisionWeight = 1000

    lowbandwidthweight = 0  # 2e-1
    minTorqueDiffWeight = 0  # 2e-2

    refForceWeight = 10
    contiForceWeight = 0

    impactAltitudeWeight = 20000
    impactVelocityWeight = 200
    impactRotationWeight = 200
    refMainJointsAtImpactWeight = 0  # 2e2 # For avoinding crossing legs

    stateTerminalWeight = 20  # 2000
    terminalNoVelocityWeight = 2000
    terminalXTargetWeight = 0  # ##DDP## 2000

    enforceMinimalFootDistance = False

    refFootFlyingAltitude = 3e-2
    flyHighSlope = 5 / refFootFlyingAltitude
    footMinimalDistance = 0.2  # (.17 is the max value wrt initial config)
    soleCollision = True
    towCollision = False
    heelCollision = False
    MAIN_JOINTS = [
        f"leg_{side}_{idx}_joint" for side in ["left", "right"] for idx in [1, 2, 4]
    ]

    vcomRef = np.array([0.1, 0, 0])
    vcomSelection = [0, 1, 2]
    vcomImportance = np.array([0.0, 0, 1])
    FOOT_SIZE = 0.05

    kktDamping = 0  # 1e-6
    baumgartGains = np.array([0, 50])
    solver_th_stop = 1e-3
    guessFile = "/tmp/ddp.npy"
    saveFile = "/tmp/ddp.npy"
