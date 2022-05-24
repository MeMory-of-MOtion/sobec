import pinocchio as pin
import crocoddyl as croc
import sobec
import numpy as np
from sys import float_info

# ##############################################################################
# # HYPER PARAMS
# ##############################################################################

rightFoot = "right_sole_link"
leftFoot = "left_sole_link"

# z coordinate of robot base when standing on ground
# zBase_reference = 1.01927

# Total number of nodes of the simulation
# T_total = 2000

# Time step of the DDP
DT = 1e-2

# Time horizon of the DDP (number of node)
T = 100

# Double support time
T2contact = 50

# Single support time
T1contact = 100

# Number of DDP iterations
ddpIteration = 1

# step size
xForward = 0.1

# foot height
foot_height = 0.03

# Foot depth in ground
TFootDepth = 220

# Correction in y to push the feet away from each other
yCorrection = 0.005

# Friction
mu = 0.1
cone_box = np.array([0.1, 0.075])
minNforce = 200
maxNforce = 1200

# Weights for all costs

wFootPlacement = 1000
wStateReg = 0.1
wControlReg = 0.001
wLimit = 1e3
wVCoM = 0
wWrenchCone = 0.005

# ###############################################################################
# ##############################################################################
# #############################################################################


class OCP:
    def __init__(self, rmodel, verbose=True):

        self.verbose = verbose

        self.rmodel = rmodel
        rdata = self.rdata = rmodel.createData()

        pin.forwardKinematics(self.rmodel, self.rdata, self.rmodel.q0)
        pin.updateFramePlacements(self.rmodel, self.rdata)

        rightFootId = self.rightFootId = self.rmodel.getFrameId(rightFoot)
        leftFootId = self.leftFootId = self.rmodel.getFrameId(leftFoot)

        self.defaultState = np.concatenate([self.rmodel.q0, np.zeros(self.rmodel.nv)])

        # Data structure
        state = croc.StateMultibody(self.rmodel)
        actuation = croc.ActuationModelFloatingBase(state)

        # Add contact to the model
        contactModelDouble = croc.ContactModelMultiple(state, actuation.nu)
        framePlacementLeft = croc.FramePlacement(leftFootId, rdata.oMf[leftFootId])
        supportContactModelLeft = croc.ContactModel6D(
            state, framePlacementLeft, actuation.nu, np.array([0, 50])
        )
        contactModelDouble.addContact(
            self.rmodel.frames[leftFootId].name + "_contact", supportContactModelLeft
        )
        framePlacementRight = croc.FramePlacement(
            rightFootId, rdata.oMf[rightFootId]
        )  # rdata.oMi[1:][rightFootId]
        supportContactModelRight = croc.ContactModel6D(
            state, framePlacementRight, actuation.nu, np.array([0, 50])
        )
        contactModelDouble.addContact(
            self.rmodel.frames[rightFootId].name + "_contact", supportContactModelRight
        )

        contactModelLeft = croc.ContactModelMultiple(state, actuation.nu)
        contactModelLeft.addContact(
            self.rmodel.frames[leftFootId].name + "_contact", supportContactModelLeft
        )

        contactModelRight = croc.ContactModelMultiple(state, actuation.nu)
        contactModelRight.addContact(
            self.rmodel.frames[rightFootId].name + "_contact", supportContactModelRight
        )

        # #####  Create the cost functions

        # #####  Cost for self-collision
        xlb = np.concatenate(
            [
                -float_info.max * np.ones(6),  # dimension of the SE(3) manifold
                self.rmodel.lowerPositionLimit[7:],
                -float_info.max * np.ones(state.nv),
            ]
        )
        xub = np.concatenate(
            [
                float_info.max * np.ones(6),  # dimension of the SE(3) manifold
                self.rmodel.upperPositionLimit[7:],
                float_info.max * np.ones(state.nv),
            ]
        )
        bounds = croc.ActivationBounds(xlb, xub, 1.0)

        limitCost = croc.CostModelResidual(
            state,
            croc.ActivationModelQuadraticBarrier(bounds),
            croc.ResidualModelState(state, np.zeros(state.nx), actuation.nu),
        )

        # Wrench cone cost
        coneRotationLeft = rdata.oMf[leftFootId].copy().rotation.T
        coneRotationRight = rdata.oMf[rightFootId].copy().rotation.T
        wrenchConeFrameLeft = croc.WrenchCone(
            coneRotationLeft, mu, cone_box, 4, True, minNforce, maxNforce
        )
        wrenchConeFrameRight = croc.WrenchCone(
            coneRotationRight, mu, cone_box, 4, True, minNforce, maxNforce
        )
        boundsFrictionLeft = croc.ActivationBounds(
            wrenchConeFrameLeft.lb, wrenchConeFrameLeft.ub, 1.0
        )
        boundsFrictionRight = croc.ActivationBounds(
            wrenchConeFrameRight.lb, wrenchConeFrameRight.ub, 1.0
        )

        wrenchRefTwoSupports = np.zeros(len(wrenchConeFrameLeft.ub))
        fz_ref2 = 400
        wrenchRefTwoSupports[4] = fz_ref2
        wrenchRefTwoSupports[5] = -cone_box[1] * fz_ref2
        wrenchRefTwoSupports[6] = -cone_box[1] * fz_ref2
        wrenchRefTwoSupports[7] = -cone_box[0] * fz_ref2
        wrenchRefTwoSupports[8] = -cone_box[0] * fz_ref2
        for i in range(4):
            wrenchRefTwoSupports[i] = -fz_ref2 * mu
        for i in range(8):
            wrenchRefTwoSupports[i + 9] = -fz_ref2 * mu * (cone_box[0] + cone_box[1])

        wrenchRef1 = np.zeros(len(wrenchConeFrameLeft.ub))
        fz_ref1 = 800
        wrenchRef1[4] = fz_ref1
        wrenchRef1[5] = -cone_box[1] * fz_ref1
        wrenchRef1[6] = -cone_box[1] * fz_ref1
        wrenchRef1[7] = -cone_box[0] * fz_ref1
        wrenchRef1[8] = -cone_box[0] * fz_ref1
        for i in range(4):
            wrenchRef1[i] = -fz_ref1 * mu
        for i in range(8):
            wrenchRef1[i + 9] = -fz_ref1 * mu * (cone_box[0] + cone_box[1])

        wrenchConeResidualLeft = croc.ResidualModelContactWrenchCone(
            state, leftFootId, wrenchConeFrameLeft, actuation.nu
        )
        wrenchConeResidualRight = croc.ResidualModelContactWrenchCone(
            state, rightFootId, wrenchConeFrameRight, actuation.nu
        )
        wrenchConeCostLeft = croc.CostModelResidual(
            state, sobec.ActivationModelQuadRef(wrenchRef1), wrenchConeResidualLeft
        )
        wrenchConeCostRight = croc.CostModelResidual(
            state, sobec.ActivationModelQuadRef(wrenchRef1), wrenchConeResidualRight
        )

        wrenchConeResidualLeft2 = croc.ResidualModelContactWrenchCone(
            state, leftFootId, wrenchConeFrameLeft, actuation.nu
        )
        wrenchConeResidualRight2 = croc.ResidualModelContactWrenchCone(
            state, rightFootId, wrenchConeFrameRight, actuation.nu
        )
        wrenchConeCostLeft2 = croc.CostModelResidual(
            state,
            sobec.ActivationModelQuadRef(wrenchRefTwoSupports),
            wrenchConeResidualLeft2,
        )
        wrenchConeCostRight2 = croc.CostModelResidual(
            state,
            sobec.ActivationModelQuadRef(wrenchRefTwoSupports),
            wrenchConeResidualRight2,
        )

        # ####  Cost for state and control

        runningCosts = np.array(
            [1000.0, 0.1, 0.001, 0, 1e3, 0.005, 100]
        )  # [1.,0.02,0.0004,0.0,1e3]
        # GoalTracking cost, State regularization cost, control cost, limit cost
        terminalCosts = np.array([8000.0, 0.02, 0.0, 0, 0.0])  # [10.0,0.02,0.0,0.0,0.0]

        weightBasePos = [0, 0, 0, 100000, 100000, 100000]
        weightBaseVel = [0, 0, 0, 1000, 1000, 1000]
        weightLegPos = [100, 100, 100, 10000, 100, 100]
        weightLegVel = [1000, 1000, 1000, 1000, 1000, 1000]
        weightArmRightPos = [
            10000,
            10000,
            10000,
            10000,
        ]  # ,100,100,100 for 3 last arm joint
        weightArmRightVel = [
            1000,
            1000,
            1000,
            1000,
        ]  # ,100,100,100 for 3 last arm joint
        weightArmLeftPos = [
            10000,
            10000,
            10000,
            10000,
        ]  # ,100,100,100 for 3 last arm joint
        weightArmLeftVel = [100, 100, 100, 100]  # ,100,100,100 for 3 last arm joint
        weightTorsoPos = [500, 500]
        weightTorsoVel = [500, 500]

        stateWeights = np.array(
            weightBasePos
            + weightLegPos * 2
            + weightTorsoPos
            + weightArmLeftPos
            + weightArmRightPos
            + weightBaseVel
            + weightLegVel * 2
            + weightTorsoVel
            + weightArmLeftVel
            + weightArmRightVel
        )

        weightuBase = [0, 0, 0, 0, 0, 0]
        weightuLeg = [1, 1, 1, 1, 10, 10]
        weightuArm = [10, 10, 10, 10]
        weightuTorso = [1, 1]
        controlWeight = np.array(weightuLeg * 2 + weightuTorso + weightuArm * 2)

        xRegResidual = croc.ResidualModelState(state, self.defaultState, actuation.nu)
        xRegCost = croc.CostModelResidual(
            state,
            croc.ActivationModelWeightedQuad(np.array(stateWeights)),
            xRegResidual,
        )

        uResidual = croc.ResidualModelControl(state, actuation.nu)
        uRegCost = croc.CostModelResidual(
            state, croc.ActivationModelWeightedQuad(np.array(controlWeight)), uResidual
        )

        # ### Foot placement cost
        startingPosLeftFoot = rdata.oMf[leftFootId].copy()
        startingPosRightFoot = rdata.oMf[rightFootId].copy()

        residualPlacementRight = croc.ResidualModelFramePlacement(
            state, rightFootId, startingPosRightFoot, actuation.nu
        )
        residualPlacementLeft = croc.ResidualModelFramePlacement(
            state, leftFootId, startingPosLeftFoot, actuation.nu
        )

        goalTrackingCostRight = croc.CostModelResidual(
            state, croc.ActivationModelQuadFlatLog(6, 0.002), residualPlacementRight
        )
        goalTrackingCostLeft = croc.CostModelResidual(
            state, croc.ActivationModelQuadFlatLog(6, 0.002), residualPlacementLeft
        )

        # ### CoM velocity regularization
        residualCoMVelocity = sobec.ResidualModelCoMVelocity(
            state, np.array([0, 0, 0]), actuation.nu
        )
        comVelCost = croc.CostModelResidual(state, residualCoMVelocity)

        # #############################################################################
        #
        # Initialize cost model, action model and DDP
        #
        # #############################################################################

        # Create cost model per each action model
        runningCostModel = croc.CostModelSum(state, actuation.nu)
        runningCostModel.addCost("stateReg", xRegCost, wStateReg)
        runningCostModel.addCost("ctrlReg", uRegCost, wControlReg)
        runningCostModel.addCost("limitcost", limitCost, wLimit)
        runningCostModel.addCost("wrenchConeRight", wrenchConeCostRight2, wWrenchCone)
        runningCostModel.addCost("wrenchConeLeft", wrenchConeCostLeft2, wWrenchCone)
        runningCostModel.addCost("comVelCost", comVelCost, wVCoM)

        dmodelRunning = croc.DifferentialActionModelContactFwdDynamics(
            state, actuation, contactModelDouble, runningCostModel, 0, True
        )
        runningModel = croc.IntegratedActionModelEuler(dmodelRunning, DT)
        self.DT = DT

        # Update control reference to gravity compensating torque
        # in half sitting position
        temp_data = runningModel.createData()
        uResidual.reference = runningModel.quasiStatic(temp_data, self.defaultState)

        # Create running cost model for simple support
        runningCostModelLeftSwing = croc.CostModelSum(state, actuation.nu)
        runningCostModelRightSwing = croc.CostModelSum(state, actuation.nu)

        runningCostModelLeftSwing.addCost(
            "gripperPoseLeft", goalTrackingCostLeft, wFootPlacement
        )
        runningCostModelLeftSwing.addCost("stateReg", xRegCost, wStateReg)
        runningCostModelLeftSwing.addCost("ctrlReg", uRegCost, wControlReg)
        runningCostModelLeftSwing.addCost("limitcost", limitCost, wLimit)
        runningCostModelLeftSwing.addCost(
            "wrenchConeRight", wrenchConeCostRight, wWrenchCone
        )
        runningCostModelLeftSwing.addCost("comVelCost", comVelCost, wVCoM)

        dmodelRunningLeftSwing = croc.DifferentialActionModelContactFwdDynamics(
            state, actuation, contactModelRight, runningCostModelLeftSwing, 0, True
        )
        runningModelLeftSwing = croc.IntegratedActionModelEuler(
            dmodelRunningLeftSwing, DT
        )

        runningCostModelRightSwing = croc.CostModelSum(state, actuation.nu)
        runningCostModelRightSwing.addCost(
            "gripperPoseRight", goalTrackingCostRight, wFootPlacement
        )
        runningCostModelRightSwing.addCost("stateReg", xRegCost, wStateReg)
        runningCostModelRightSwing.addCost("ctrlReg", uRegCost, wControlReg)
        runningCostModelRightSwing.addCost("limitcost", limitCost, wLimit)
        runningCostModelRightSwing.addCost(
            "wrenchConeLeft", wrenchConeCostLeft, wWrenchCone
        )
        runningCostModelRightSwing.addCost("comVelCost", comVelCost, wVCoM)

        dmodelRunningRightSwing = croc.DifferentialActionModelContactFwdDynamics(
            state, actuation, contactModelLeft, runningCostModelRightSwing, 0, True
        )
        runningModelRightSwing = croc.IntegratedActionModelEuler(
            dmodelRunningRightSwing, DT
        )

        problem = croc.ShootingProblem(
            self.defaultState, [runningModel] * T, runningModel
        )

        # Creating the DDP solver for this OC problem, defining a logger

        ddp = croc.SolverFDDP(problem)
        # ddp.setCallbacks([croc.CallbackVerbose()])
        ddp.th_stop = 1e-6
        ddp.th_grad = 1e-9
        # Solving it with the DDP algorithm
        xs0 = [self.defaultState] * (T + 1)
        us0 = [
            ddp.problem.runningModels[0].quasiStatic(
                ddp.problem.runningDatas[0], self.defaultState
            )
        ] * T

        ddp.solve(xs0, us0, 500, False)

        self.ddp = ddp
        self.runningModelRightSwing = runningModelRightSwing
        self.runningModelLeftSwing = runningModelLeftSwing
        self.runningModel = runningModel
        self.residualPlacementRight = residualPlacementRight
        self.residualPlacementLeft = residualPlacementLeft
        self.coneRotationLeft = coneRotationLeft
        self.coneRotationRight = coneRotationRight
        # self.wrenchConeFrameLeft = wrenchConeFrameLeft
        # self.wrenchConeFrameRight = wrenchConeFrameRight
        self.wrenchConeResidualLeft = wrenchConeResidualLeft
        self.wrenchConeResidualLeft2 = wrenchConeResidualLeft2
        self.wrenchConeResidualRight = wrenchConeResidualRight
        self.wrenchConeResidualRight2 = wrenchConeResidualRight2
        self.startingPosLeftFoot = startingPosLeftFoot
        self.startingPosRightFoot = startingPosRightFoot
        self.state = state

    def initLocomotionPattern(self):

        # Counter until beginning of single phase
        self.Tswitch2single = T + T2contact
        # Counter until beginning of double phase
        self.Tswitch2double = T + T2contact + T1contact
        self.swingRightPhaseDouble = True
        self.swingRightPhaseSimple = True
        self.firstStep = True
        self.xForward = xForward

    def updateOCP(self, x):

        # Initialize MPC with current state of simulation
        pin.forwardKinematics(self.rmodel, self.rdata, x[: self.rmodel.nq])
        pin.updateFramePlacements(self.rmodel, self.rdata)

        if self.Tswitch2single >= 0 and self.Tswitch2single <= T:
            if self.swingRightPhaseSimple:
                self.ddp.problem.updateModel(
                    self.Tswitch2single, self.runningModelRightSwing
                )
                if self.verbose:
                    print("update to swing right contact model ", self.Tswitch2single)
            else:
                self.ddp.problem.updateModel(
                    self.Tswitch2single, self.runningModelLeftSwing
                )
                if self.verbose:
                    print("update to swing left contact model ", self.Tswitch2single)
        if self.Tswitch2double >= 0 and self.Tswitch2double <= T:
            self.ddp.problem.updateModel(self.Tswitch2double, self.runningModel)
            if self.verbose:
                print("update to double contact model ", self.Tswitch2double)
        if self.Tswitch2double >= 0 and self.Tswitch2double <= T1contact:
            if self.swingRightPhaseDouble:
                # Update right foot desired placement
                targetFrameNow = self.startingPosRightFoot.copy()
                targetFrameNow.translation[0] = self.startingPosRightFoot.translation[
                    0
                ] + self.xForward * float(T1contact - self.Tswitch2double) / float(
                    T1contact
                )
                targetFrameNow.translation[1] = self.startingPosRightFoot.translation[
                    1
                ] - yCorrection * float(T1contact - self.Tswitch2double) / float(
                    T1contact
                )
                targetFrameNow.translation[2] = self.startingPosRightFoot.translation[
                    2
                ] + foot_height * np.sin(
                    float(T1contact - self.Tswitch2double)
                    / float(T1contact)
                    * TFootDepth
                    * 3.14
                    / 180
                )
                self.residualPlacementRight.reference = targetFrameNow
                if self.verbose:
                    print("Change placement right")
            else:
                # Update left foot desired placement
                targetFrameNow = self.startingPosLeftFoot.copy()
                targetFrameNow.translation[0] = self.startingPosLeftFoot.translation[
                    0
                ] + self.xForward * float(T1contact - self.Tswitch2double) / float(
                    T1contact
                )
                targetFrameNow.translation[1] = self.startingPosLeftFoot.translation[
                    1
                ] + yCorrection * float(T1contact - self.Tswitch2double) / float(
                    T1contact
                )
                targetFrameNow.translation[2] = self.startingPosLeftFoot.translation[
                    2
                ] + foot_height * np.sin(
                    float(T1contact - self.Tswitch2double)
                    / float(T1contact)
                    * TFootDepth
                    * 3.14
                    / 180
                )
                self.residualPlacementLeft.reference = targetFrameNow
                if self.verbose:
                    print("Change placement left")

        self.Tswitch2single -= 1
        self.Tswitch2double -= 1

        if self.Tswitch2single < 0:
            self.Tswitch2single = T1contact + T2contact
            self.swingRightPhaseSimple = not (self.swingRightPhaseSimple)
            if self.verbose:
                print("Next swight is right? ", self.swingRightPhaseSimple)
        if self.Tswitch2double < 0:
            if self.firstStep:
                self.xForward *= 2
                self.firstStep = False

            self.startingPosLeftFoot = self.rdata.oMf[self.leftFootId].copy()
            self.startingPosRightFoot = self.rdata.oMf[self.rightFootId].copy()

            self.Tswitch2double = T1contact + T2contact
            self.swingRightPhaseDouble = not (self.swingRightPhaseDouble)

            if self.verbose:
                print("switch to double: Update wrench cone orientation")
            # Update wrench cone orientation
            self.coneRotationLeft = self.rdata.oMf[self.leftFootId].copy().rotation.T
            self.coneRotationRight = self.rdata.oMf[self.rightFootId].copy().rotation.T
            wrenchConeFrameLeft = croc.WrenchCone(
                self.coneRotationLeft, mu, cone_box, 4, True, minNforce, maxNforce
            )
            wrenchConeFrameRight = croc.WrenchCone(
                self.coneRotationRight, mu, cone_box, 4, True, minNforce, maxNforce
            )

            self.wrenchConeResidualLeft.reference = wrenchConeFrameLeft
            self.wrenchConeResidualLeft2.reference = wrenchConeFrameLeft
            self.wrenchConeResidualRight.reference = wrenchConeFrameRight
            self.wrenchConeResidualRight2.reference = wrenchConeFrameRight

    def solve(self, x):
        xs = list(self.ddp.xs[1:]) + [self.ddp.xs[-1]]
        xs[0] = x
        us = list(self.ddp.us[1:]) + [self.ddp.us[-1]]
        self.ddp.problem.x0 = xs[0]
        self.ddp.solve(xs, us, ddpIteration, False)


# ##############################################################################
# ##############################################################################
# ############   ##     ##    ###    #### ##    ##   ###########################
# ############   ###   ###   ## ##    ##  ###   ##   ###########################
# ############   #### ####  ##   ##   ##  ####  ##   ###########################
# ############   ## ### ## ##     ##  ##  ## ## ##   ###########################
# ############   ##     ## #########  ##  ##  ####   ###########################
# ############   ##     ## ##     ##  ##  ##   ###   ###########################
# ############   ##     ## ##     ## #### ##    ##   ###########################
# ##############################################################################
# ##############################################################################

if __name__ == "__main__":

    print("ASSERT receiding horizon behavior against logged files")

    import example_robot_data as robex

    # ## Load model with some frozen joints
    robot = robex.load("talos")
    robot.model.q0 = robot.model.referenceConfigurations["half_sitting"]
    blockedJointNames = [
        # "universe",
        # "arm_left_1_joint",
        # "arm_left_2_joint",
        # "arm_left_3_joint",
        # "arm_left_4_joint",
        "arm_left_5_joint",
        "arm_left_6_joint",
        "arm_left_7_joint",
        # "arm_right_1_joint",
        # "arm_right_2_joint",
        # "arm_right_3_joint",
        # "arm_right_4_joint",
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "gripper_left_joint",
        "gripper_right_joint",
        "head_1_joint",
        "head_2_joint",
    ]
    blockedJointIds = [
        i for (i, n) in enumerate(robot.model.names) if n in blockedJointNames
    ]
    rmodel, [gmodel] = pin.buildReducedModel(
        robot.model, [robot.visual_model], blockedJointIds, robot.model.q0
    )
    rmodel.q0 = rmodel.referenceConfigurations["half_sitting"]

    # ## Create OCP
    ocp = OCP(rmodel, verbose=False)
    ocp.initLocomotionPattern()

    # ## Open display
    viz = pin.visualize.GepettoVisualizer(rmodel, gmodel, gmodel)
    viz.initViewer(loadModel=True)
    viz.display(rmodel.q0)

    # ## Open reference logs.
    npy = np.load("assert_ocp_walk_feet_traj.npy", allow_pickle=True)[()]
    gtx = npy["x"]
    gtu = npy["u"]

    # ## Receiding horizon with "perfect" behavior, to be compared against logs.
    for s in range(1000):
        assert np.linalg.norm(gtx[s] - ocp.ddp.xs[0]) < 1e-6
        ocp.updateOCP(ocp.ddp.xs[0])
        ocp.solve(ocp.ddp.xs[1])
        viz.display(ocp.ddp.xs[0][: rmodel.nq])
