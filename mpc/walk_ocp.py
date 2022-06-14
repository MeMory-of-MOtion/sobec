import pinocchio as pin
import crocoddyl as croc
import numpy as np

# Local imports
import sobec
from weight_share import computeReferenceForces


def buildRunningModels(robotWrapper, contactPattern, params):

    p = params
    robot = robotWrapper

    referenceForces = computeReferenceForces(contactPattern, robot.gravForce)
    models = []

    # #################################################################################
    for t, pattern in enumerate(contactPattern[:-1]):
        # print(f'time t={t} {pattern}')

        # Basics
        state = croc.StateMultibody(robot.model)
        actuation = croc.ActuationModelFloatingBase(state)

        # Contacts
        contacts = croc.ContactModelMultiple(state, actuation.nu)
        for k, cid in enumerate(robot.contactIds):
            if not pattern[k]:
                continue
            contact = croc.ContactModel6D(
                state, cid, pin.SE3.Identity(), actuation.nu, p.baumgartGains
            )
            contacts.addContact(robot.model.frames[cid].name + "_contact", contact)

        # Costs
        costs = croc.CostModelSum(state, actuation.nu)

        xRegResidual = croc.ResidualModelState(state, robot.x0, actuation.nu)
        xRegCost = croc.CostModelResidual(
            state,
            croc.ActivationModelWeightedQuad(p.stateImportance**2),
            xRegResidual,
        )
        costs.addCost("stateReg", xRegCost, p.refStateWeight)

        uResidual = croc.ResidualModelControl(state, actuation.nu)
        uRegCost = croc.CostModelResidual(
            state,
            croc.ActivationModelWeightedQuad(np.array(p.controlImportance**2)),
            uResidual,
        )
        if p.refTorqueWeight > 0:
            costs.addCost("ctrlReg", uRegCost, p.refTorqueWeight)

        comResidual = croc.ResidualModelCoMPosition(state, robot.com0, actuation.nu)
        comAct = croc.ActivationModelWeightedQuad(np.array([0, 0, 1]))
        comCost = croc.CostModelResidual(state, comAct, comResidual)
        if p.comWeight > 0:
            costs.addCost("com", comCost, p.comWeight)

        comVelResidual = sobec.ResidualModelCoMVelocity(state, p.vcomRef, actuation.nu)
        comVelAct = croc.ActivationModelWeightedQuad(p.vcomImportance)
        comVelCost = croc.CostModelResidual(state, comVelAct, comVelResidual)
        costs.addCost("comVelCost", comVelCost, p.vcomWeight)

        # Contact costs
        for k, cid in enumerate(robot.contactIds):
            if not pattern[k]:
                continue

            copResidual = sobec.ResidualModelCenterOfPressure(state, cid, actuation.nu)
            copAct = croc.ActivationModelWeightedQuad(
                np.array([1 / p.FOOT_SIZE**2] * 2)
            )
            copCost = croc.CostModelResidual(state, copAct, copResidual)
            costs.addCost(f"{robot.model.frames[cid].name}_cop", copCost, p.copWeight)

            # Cone with enormous friction (Assuming the robot will barely ever slide).
            # p.FOOT_SIZE is the allowed area size, while cone expects the corner
            # coordinates => x2
            cone = croc.WrenchCone(
                np.eye(3), 1000, np.array([p.FOOT_SIZE * 2] * 2), 4, True, 1, 10000
            )
            coneCost = croc.ResidualModelContactWrenchCone(
                state, cid, cone, actuation.nu
            )
            ub = cone.ub.copy()
            ub[:4] = np.inf
            # ub[5:] = np.inf  ### DEBUG
            ub[-8:] = np.inf
            coneAct = croc.ActivationModelQuadraticBarrier(
                croc.ActivationBounds(cone.lb, ub)
            )
            coneCost = croc.CostModelResidual(state, coneAct, coneCost)
            costs.addCost(
                f"{robot.model.frames[cid].name}_cone", coneCost, p.conePenaltyWeight
            )

            # Penalize the distance to the central axis of the cone ...
            #  ... using normalization weights depending on the axis.
            # The weights are squared to match the tuning of the CASADI formulation.
            coneAxisResidual = croc.ResidualModelContactForce(
                state, cid, pin.Force.Zero(), 6, actuation.nu
            )
            w = np.array(p.forceImportance**2)
            w[2] = 0
            coneAxisAct = croc.ActivationModelWeightedQuad(w)
            coneAxisCost = croc.CostModelResidual(state, coneAxisAct, coneAxisResidual)
            costs.addCost(
                f"{robot.model.frames[cid].name}_coneaxis",
                coneAxisCost,
                p.coneAxisWeight,
            )

            # Follow reference (smooth) contact forces
            forceRefResidual = croc.ResidualModelContactForce(
                state, cid, pin.Force(referenceForces[t][k]), 6, actuation.nu
            )
            forceRefCost = croc.CostModelResidual(state, forceRefResidual)
            costs.addCost(
                f"{robot.model.frames[cid].name}_forceref",
                forceRefCost,
                p.refForceWeight / robot.gravForce**2,
            )

        # IMPACT
        for k, cid in enumerate(robot.contactIds):
            if t > 0 and not contactPattern[t - 1][k] and pattern[k]:
                # REMEMBER TO divide the weight by p.DT, as impact should be independant
                # of the node duration (at least, that s how weights are tuned in
                # casadi).

                print(f"Impact {cid} at time {t}")
                impactResidual = croc.ResidualModelFrameTranslation(
                    state, cid, np.zeros(3), actuation.nu
                )
                impactAct = croc.ActivationModelWeightedQuad(np.array([0, 0, 1]))
                impactCost = croc.CostModelResidual(state, impactAct, impactResidual)
                costs.addCost(
                    f"{robot.model.frames[cid].name}_altitudeimpact",
                    impactCost,
                    p.impactAltitudeWeight / p.DT,
                )

                impactVelResidual = croc.ResidualModelFrameVelocity(
                    state,
                    cid,
                    pin.Motion.Zero(),
                    pin.ReferenceFrame.LOCAL,
                    actuation.nu,
                )
                impactVelCost = croc.CostModelResidual(state, impactVelResidual)
                costs.addCost(
                    f"{robot.model.frames[cid].name}_velimpact",
                    impactVelCost,
                    p.impactVelocityWeight / p.DT,
                )

                impactRotResidual = croc.ResidualModelFrameRotation(
                    state, cid, np.eye(3), actuation.nu
                )
                impactRotAct = croc.ActivationModelWeightedQuad(np.array([1, 1, 0]))
                impactRotCost = croc.CostModelResidual(
                    state, impactRotAct, impactRotResidual
                )
                costs.addCost(
                    f"{robot.model.frames[cid].name}_rotimpact",
                    impactRotCost,
                    p.impactRotationWeight / p.DT,
                )

                impactRefJointsResidual = croc.ResidualModelState(
                    state, robot.x0, actuation.nu
                )
                jselec = np.zeros(robot.model.nv * 2)
                jselec[
                    [
                        robot.model.joints[robot.model.getJointId(name)].idx_v
                        for name in p.MAIN_JOINTS
                    ]
                ] = 1
                impactRefJointsAct = croc.ActivationModelWeightedQuad(jselec)
                impactRefJointCost = croc.CostModelResidual(
                    state, impactRefJointsAct, impactRefJointsResidual
                )
                costs.addCost(
                    "impactRefJoint",
                    impactRefJointCost,
                    p.refMainJointsAtImpactWeight / p.DT,
                )

        # Flying foot
        for k, fid in enumerate(robot.contactIds):
            if pattern[k]:
                continue
            verticalFootVelResidual = croc.ResidualModelFrameVelocity(
                state,
                fid,
                pin.Motion.Zero(),
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                actuation.nu,
            )
            verticalFootVelAct = croc.ActivationModelWeightedQuad(
                np.array([0, 0, 1, 0, 0, 0])
            )
            verticalFootVelCost = croc.CostModelResidual(
                state, verticalFootVelAct, verticalFootVelResidual
            )
            costs.addCost(
                f"{robot.model.frames[fid].name}_vfoot_vel",
                verticalFootVelCost,
                p.verticalFootVelWeight,
            )

            # Slope is /2 since it is squared in casadi (je me comprends)
            flyHighResidual = sobec.ResidualModelFlyHigh(
                state, fid, p.flyHighSlope / 2, actuation.nu
            )
            flyHighCost = croc.CostModelResidual(state, flyHighResidual)
            costs.addCost(
                f"{robot.model.frames[fid].name}_flyhigh", flyHighCost, p.flyWeight
            )

            groundColRes = croc.ResidualModelFrameTranslation(
                state, fid, np.zeros(3), actuation.nu
            )
            # groundColBounds = croc.ActivationBounds(
            # np.array([-np.inf, -np.inf, 0.01]), np.array([np.inf, np.inf, np.inf])
            # )
            # np.inf introduces an error on lb[2] ... why? TODO ... patch by replacing
            # np.inf with 1000
            groundColBounds = croc.ActivationBounds(
                np.array([-1000, -1000, 0.0]), np.array([1000, 1000, 1000])
            )
            groundColAct = croc.ActivationModelQuadraticBarrier(groundColBounds)
            groundColCost = croc.CostModelResidual(state, groundColAct, groundColRes)
            costs.addCost(
                f"{robot.model.frames[fid].name}_groundcol",
                groundColCost,
                p.groundColWeight,
            )

            for kc, cid in enumerate(robot.contactIds):
                if not pattern[kc]:
                    continue
                assert fid != cid
                # print(f'At t={t} add collision between {cid} and {fid}')

                for id1, id2 in [
                    (i, j)
                    for i in [cid, robot.towIds[cid], robot.heelIds[cid]]
                    for j in [fid, robot.towIds[fid], robot.heelIds[fid]]
                ]:
                    feetColResidual = sobec.ResidualModelFeetCollision(
                        state, id1, id2, actuation.nu
                    )
                    feetColBounds = croc.ActivationBounds(
                        np.array([p.footMinimalDistance]), np.array([1000])
                    )
                    feetColAct = croc.ActivationModelQuadraticBarrier(feetColBounds)
                    feetColCost = croc.CostModelResidual(
                        state, feetColAct, feetColResidual
                    )
                    costs.addCost(
                        (
                            f"feetcol_{robot.model.frames[id1].name}_"
                            f"VS_{robot.model.frames[id2].name}"
                        ),
                        feetColCost,
                        p.feetCollisionWeight,
                    )

        # Action
        damodel = croc.DifferentialActionModelContactFwdDynamics(
            state, actuation, contacts, costs, p.kktDamping, True
        )
        amodel = croc.IntegratedActionModelEuler(damodel, p.DT)

        models.append(amodel)

    return models


# ### TERMINAL MODEL ##################################################################
def buildTerminalModel(robotWrapper, contactPattern, params):

    robot = robotWrapper
    p = params
    pattern = contactPattern[-1]

    # Horizon length
    T = len(contactPattern) - 1

    state = croc.StateMultibody(robot.model)
    actuation = croc.ActuationModelFloatingBase(state)

    # Contacts
    contacts = croc.ContactModelMultiple(state, actuation.nu)
    for k, cid in enumerate(robot.contactIds):
        if not pattern[k]:
            continue
        contact = croc.ContactModel6D(
            state, cid, pin.SE3.Identity(), actuation.nu, p.baumgartGains
        )
        contacts.addContact(robot.model.frames[cid].name + "_contact", contact)

    # Costs
    costs = croc.CostModelSum(state, actuation.nu)

    if "stateTerminalTarget" not in locals():
        stateTerminalTarget = robot.x0.copy()
        stateTerminalTarget[:3] += p.vcomRef * T * p.DT
    stateTerminalResidual = croc.ResidualModelState(
        state, stateTerminalTarget, actuation.nu
    )
    stateTerminalAct = croc.ActivationModelWeightedQuad(p.stateTerminalImportance**2)
    stateTerminalCost = croc.CostModelResidual(
        state, stateTerminalAct, stateTerminalResidual
    )
    costs.addCost("stateReg", stateTerminalCost, p.stateTerminalWeight)

    damodel = croc.DifferentialActionModelContactFwdDynamics(
        state, actuation, contacts, costs, p.kktDamping, True
    )
    termmodel = croc.IntegratedActionModelEuler(damodel, p.DT)

    return termmodel


# ### SOLVER ########################################################################


def buildSolver(robotWrapper, contactPattern, walkParams):
    models = buildRunningModels(robotWrapper, contactPattern, walkParams)
    termmodel = buildTerminalModel(robotWrapper, contactPattern, walkParams)

    problem = croc.ShootingProblem(robotWrapper.x0, models, termmodel)
    ddp = croc.SolverFDDP(problem)
    ddp.th_stop = walkParams.solver_th_stop
    return ddp


def buildInitialGuess(problem, walkParams):
    if walkParams.guessFile is not None:
        try:
            guess = np.load(walkParams.guessFile, allow_pickle=True)[()]
            print(f'Load "{walkParams.guessFile}"!')
            x0s = [x for x in guess["xs"]]
            u0s = [u for u in guess["us"]]
        except (FileNotFoundError, KeyError):
            x0s = []
            u0s = []

    if len(x0s) != problem.T + 1 or len(u0s) != problem.T:
        print("No valid solution file, build quasistatic initial guess!")
        x0s = [problem.x0.copy() for _ in range(problem.T + 1)]
        u0s = [
            m.quasiStatic(d, x)
            for m, d, x in zip(problem.runningModels, problem.runningDatas, x0s)
        ]

    return x0s, u0s


# ### SOLUTION ######################################################################


class Solution:
    def __init__(self, robotWrapper, ddp):
        # model = ddp.problem.terminalModel.differential.pinocchio
        self.xs = np.array(ddp.xs)
        self.us = np.array(ddp.us)
        self.acs = np.array([d.differential.xout for d in ddp.problem.runningDatas])
        self.fs = [
            [
                (cd.data().jMf.inverse() * cd.data().f).vector
                for cd in d.differential.multibody.contacts.contacts
            ]
            for d in ddp.problem.runningDatas
        ]
        self.fs0 = [
            np.concatenate(
                [
                    (
                        d.differential.multibody.contacts.contacts[
                            f"{robotWrapper.model.frames[cid].name}_contact"
                        ].jMf.inverse()
                        * d.differential.multibody.contacts.contacts[
                            f"{robotWrapper.model.frames[cid].name}_contact"
                        ].f
                    ).vector
                    if f"{robotWrapper.model.frames[cid].name}_contact"
                    in d.differential.multibody.contacts.contacts
                    else np.zeros(6)
                    for cid in robotWrapper.contactIds
                ]
            )
            for m, d in zip(ddp.problem.runningModels, ddp.problem.runningDatas)
        ]
