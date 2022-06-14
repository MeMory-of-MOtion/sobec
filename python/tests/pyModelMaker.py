#!/usr/bin/env python3
"""
Created on Wed May 25 11:49:27 2022

@author: nvilla
"""

import crocoddyl
import sobec as crocobec
import numpy as np


def modeller(conf, design, state, actuation, **form):
    if "support" in form.keys():
        support = form["support"]  # options "double", "left", "right"
    else:
        support = "double"

    contacts = crocoddyl.ContactModelMultiple(state, actuation.nu)
    costs = crocoddyl.CostModelSum(state, actuation.nu)

    # ### CONTACTS ###
    define_support_contact(conf, design, state, actuation, support, contacts)

    # ### COSTS ###
    wrench_cone(conf, design, state, actuation, support, costs)
    joint_limits(conf, design, state, actuation, costs)
    control_n_state_regularization(conf, design, state, actuation, costs)
    feet_tracking(conf, design, state, actuation, costs)
    com_velocity_tracking(conf, design, state, actuation, costs)

    # ### MODEL ###
    diff_model = crocoddyl.DifferentialActionModelContactFwdDynamics(
        state, actuation, contacts, costs, 0, True
    )
    int_model = crocoddyl.IntegratedActionModelEuler(diff_model, conf.DT)

    return int_model


def define_support_contact(conf, design, state, actuation, support, contacts):

    if support == "left" or support == "double":
        frameLeftFoot = crocoddyl.FramePlacement(
            design.leftFootId, design.get_LF_frame().copy()
        )
        left6Dcontact = crocoddyl.ContactModel6D(
            state, frameLeftFoot, actuation.nu, np.array([0, 50])
        )
        contacts.addContact(conf.leftFoot, left6Dcontact)

    if support == "right" or support == "double":
        frameRightFoot = crocoddyl.FramePlacement(
            design.rightFootId, design.get_RF_frame().copy()
        )
        right6Dcontact = crocoddyl.ContactModel6D(
            state, frameRightFoot, actuation.nu, np.array([0, 50])
        )
        contacts.addContact(conf.rightFoot, right6Dcontact)

    return contacts


def wrench_cone(conf, design, state, actuation, support, costs):
    Mg = -design.get_robot_mass() * conf.gravity[2]
    fz_ref = Mg / 2 if support == "double" else Mg

    if support == "left" or support == "double":
        LF_orientation = design.get_LF_frame().copy().rotation.T
        wrenchConeFrameLeft = crocoddyl.WrenchCone(
            LF_orientation,
            conf.mu,
            conf.cone_box,
            4,
            True,
            conf.minNforce,
            conf.maxNforce,
        )

        lf_wrench_ref = np.array([0, 0, fz_ref, 0, 0, 0])
        ds_wrenchRef_lf = wrenchConeFrameLeft.A @ lf_wrench_ref
        wrenchConeResidualLeft = crocoddyl.ResidualModelContactWrenchCone(
            state, design.leftFootId, wrenchConeFrameLeft, actuation.nu
        )
        wrenchConeCostLeft = crocoddyl.CostModelResidual(
            state,
            crocobec.ActivationModelQuadRef(ds_wrenchRef_lf),
            wrenchConeResidualLeft,
        )
        costs.addCost("left_wrench_cone", wrenchConeCostLeft, conf.wWrenchCone)

    if support == "right" or support == "double":
        RF_orientation = design.get_RF_frame().copy().rotation.T
        wrenchConeFrameRight = crocoddyl.WrenchCone(
            RF_orientation,
            conf.mu,
            conf.cone_box,
            4,
            True,
            conf.minNforce,
            conf.maxNforce,
        )

        rf_wrench_ref = np.array([0, 0, fz_ref, 0, 0, 0])
        ds_wrenchRef_rf = wrenchConeFrameRight.A @ rf_wrench_ref
        wrenchConeResidualRight = crocoddyl.ResidualModelContactWrenchCone(
            state, design.rightFootId, wrenchConeFrameRight, actuation.nu
        )
        wrenchConeCostRight = crocoddyl.CostModelResidual(
            state,
            crocobec.ActivationModelQuadRef(ds_wrenchRef_rf),
            wrenchConeResidualRight,
        )
        costs.addCost("right_wrench_cone", wrenchConeCostRight, conf.wWrenchCone)

    # boundsFrictionLeft = crocoddyl.ActivationBounds(
    # wrenchConeFrameLeft.lb, wrenchConeFrameLeft.ub, 1.0
    # )
    # boundsFrictionRight = crocoddyl.ActivationBounds(
    # wrenchConeFrameRight.lb, wrenchConeFrameRight.ub, 1.0
    # )


def joint_limits(conf, design, state, actuation, costs):

    xlb = np.hstack(
        [
            -np.Inf * np.ones(6),  # dimension of the SE(3) manifold
            design.rmodel.lowerPositionLimit[7:],
            -np.Inf * np.ones(state.nv),
        ]
    )
    xub = np.hstack(
        [
            np.Inf * np.ones(6),  # dimension of the SE(3) manifold
            design.rmodel.upperPositionLimit[7:],
            np.Inf * np.ones(state.nv),
        ]
    )
    bounds = crocoddyl.ActivationBounds(xlb, xub, 1.0)

    jointLimitResiduals = crocoddyl.ResidualModelState(
        state, np.zeros(state.nx), actuation.nu
    )
    limitCost = crocoddyl.CostModelResidual(
        state, crocoddyl.ActivationModelQuadraticBarrier(bounds), jointLimitResiduals
    )
    costs.addCost("joint_limits", limitCost, conf.wLimit)


def control_n_state_regularization(conf, design, state, actuation, costs):

    xRegResidual = crocoddyl.ResidualModelState(
        state, design.rmodel.defaultState, actuation.nu
    )
    xRegCost = crocoddyl.CostModelResidual(
        state, crocoddyl.ActivationModelWeightedQuad(conf.stateWeights), xRegResidual
    )
    uResidual = crocoddyl.ResidualModelControl(state, actuation.nu)
    uRegCost = crocoddyl.CostModelResidual(
        state, crocoddyl.ActivationModelWeightedQuad(conf.controlWeight), uResidual
    )
    costs.addCost("state", xRegCost, conf.wStateReg)
    costs.addCost("control", uRegCost, conf.wControlReg)


def feet_tracking(conf, design, state, actuation, costs):

    residualPlacementRight = crocoddyl.ResidualModelFramePlacement(
        state, design.rightFootId, design.get_RF_frame().copy(), actuation.nu
    )
    residualPlacementLeft = crocoddyl.ResidualModelFramePlacement(
        state, design.leftFootId, design.get_LF_frame().copy(), actuation.nu
    )

    LF_tracking_cost = crocoddyl.CostModelResidual(
        state, crocoddyl.ActivationModelQuadFlatLog(6, 0.002), residualPlacementLeft
    )
    RF_tracking_cost = crocoddyl.CostModelResidual(
        state, crocoddyl.ActivationModelQuadFlatLog(6, 0.002), residualPlacementRight
    )
    costs.addCost("left_foot_place", LF_tracking_cost, conf.wFootPlacement)
    costs.addCost("right_foot_place", RF_tracking_cost, conf.wFootPlacement)


def com_velocity_tracking(conf, design, state, actuation, costs):

    residualCoMVelocity = crocobec.ResidualModelCoMVelocity(
        state, np.array([0, 0, 0]), actuation.nu
    )
    comVelCost = crocoddyl.CostModelResidual(state, residualCoMVelocity)
    costs.addCost("com_velocity", comVelCost, conf.wVCoM)


def swinging_foot_translation(conf, design, state, actuation, costs):

    #    deepFootHeight = 0.04
    #    yCorrection = 0.02
    goalTranslationRight = design.get_RF_frame().copy().translation
    goalTranslationLeft = design.get_LF_frame().copy().translation
    #    goalTranslationRight[1] -= yCorrection
    #    goalTranslationLeft[1] += yCorrection
    #    goalTranslationRight[2] -= deepFootHeight
    #    goalTranslationLeft[2] -= deepFootHeight

    residualTranslationZRight = crocoddyl.ResidualModelFrameTranslation(
        state, design.rightFootId, goalTranslationRight, actuation.nu
    )
    residualTranslationZLeft = crocoddyl.ResidualModelFrameTranslation(
        state, design.leftFootId, goalTranslationLeft, actuation.nu
    )

    lowYleft = np.array([-np.inf, 0.05, -np.inf])
    upperYleft = np.array([np.inf, np.inf, np.inf])
    lowYright = np.array([-np.inf, -np.inf, -np.inf])
    upperYright = np.array([np.inf, -0.05, np.inf])
    boundsXYleft = crocoddyl.ActivationBounds(lowYleft, upperYleft, 1.0)
    boundsXYright = crocoddyl.ActivationBounds(lowYright, upperYright, 1.0)

    residualTranslationXYRight = crocoddyl.ResidualModelFrameTranslation(
        state, design.rightFootId, goalTranslationRight, actuation.nu
    )
    residualTranslationXYLeft = crocoddyl.ResidualModelFrameTranslation(
        state, design.leftFootId, goalTranslationLeft, actuation.nu
    )

    # Not regulating X motion
    transZWeights = np.array([0, 0, 1])
    translationZCostRight = crocoddyl.CostModelResidual(
        state,
        crocoddyl.ActivationModelWeightedQuad(transZWeights),
        residualTranslationZRight,
    )
    translationZCostLeft = crocoddyl.CostModelResidual(
        state,
        crocoddyl.ActivationModelWeightedQuad(transZWeights),
        residualTranslationZLeft,
    )

    translationXYCostRight = crocoddyl.CostModelResidual(
        state,
        crocoddyl.ActivationModelQuadraticBarrier(boundsXYright),
        residualTranslationXYRight,
    )
    translationXYCostLeft = crocoddyl.CostModelResidual(
        state,
        crocoddyl.ActivationModelQuadraticBarrier(boundsXYleft),
        residualTranslationXYLeft,
    )

    costs.addCost("left_foot_XY_Translation", translationXYCostLeft, conf.wFootXYTrans)
    costs.addCost(
        "right_foot_XY_Translation", translationXYCostRight, conf.wFootXYTrans
    )
    costs.addCost("left_foot_translation", translationZCostLeft, conf.wFootTrans)
    costs.addCost("right_foot_translation", translationZCostRight, conf.wFootTrans)


def feet_orientation(conf, design, state, actuation, costs):
    residualRotationRight = crocoddyl.ResidualModelFrameRotation(
        state, design.rightFootId, np.eye(3), actuation.nu
    )
    residualRotationLeft = crocoddyl.ResidualModelFrameRotation(
        state, design.leftFootId, np.eye(3), actuation.nu
    )

    rotationCostRight = crocoddyl.CostModelResidual(state, residualRotationRight)
    rotationCostLeft = crocoddyl.CostModelResidual(state, residualRotationLeft)

    costs.addCost("left_foot_orientation", rotationCostLeft, conf.wFootTrans)
    costs.addCost("left_foot_orientation", rotationCostRight, conf.wFootTrans)


def ground_contact(conf, design, state, actuation, costs):
    design.load_ground_collision_model()

    residualPairCollisionLeft = crocoddyl.ResidualModelVelCollision(
        state, actuation.nu, design.geomModel, 1, design.leftFootId, design.WORLD, 0.01
    )
    leftGroundPenetrationCost = crocoddyl.CostModelResidual(
        state, residualPairCollisionLeft
    )

    residualPairCollisionRight = crocoddyl.ResidualModelVelCollision(
        state, actuation.nu, design.geomModel, 0, design.rightFootId, design.WORLD, 0.01
    )
    rightGroundPenetrationCost = crocoddyl.CostModelResidual(
        state, residualPairCollisionRight
    )

    costs.addCost("left_ground_collision", leftGroundPenetrationCost, conf.wGroundCol)
    costs.addCost("right_ground_collision", rightGroundPenetrationCost, conf.wGroundCol)
