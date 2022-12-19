#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
import matplotlib.pyplot as plt

from bullet_Talos import BulletTalos

# from cricket.virtual_talos import VirtualPhysics

# from pyRobotWrapper import PinTalos
# from pyMPC import CrocoWBC
# from pyModelMaker import modeller
import pinocchio as pin
from sobec import RobotDesigner, WBC, HorizonManager, ModelMaker, Flex, Support
import ndcurves
import numpy as np
import time
import example_robot_data
import crocoddyl
import os

DEFAULT_SAVE_DIR = os.path.dirname(os.path.realpath(__file__))


class conf:
    # PATHS

    URDF_FILENAME = "talos_reduced.urdf"
    SRDF_FILENAME = "talos.srdf"
    SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
    modelPath = example_robot_data.getModelPath(URDF_SUBPATH)

    # Joint settings

    blocked_joints = [
        "universe",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_left_5_joint",
        "arm_left_6_joint",
        "arm_left_7_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
        "arm_right_5_joint",
        "arm_right_6_joint",
        "arm_right_7_joint",
        "gripper_left_joint",
        "gripper_right_joint",
        "head_1_joint",
        "head_2_joint",
    ]

    # #### TIMING #####
    total_steps = 4
    DT = 1e-2  # Time step of the DDP
    T = 100  # Time horizon of the DDP (number of nodes)
    TdoubleSupport = 20  # Double support time  # TODO: (check with 20)
    simu_step = simu_period = 1e-3  #

    Nc = int(
        round(DT / simu_step)
    )  # Number of control knots per planification timestep

    # TODO: landing_advance and takeoff_delay are missing

    TsingleSupport = 50  # Single support time
    ddpIteration = 1  # Number of DDP iterations

    Tstep = TsingleSupport + TdoubleSupport

    gravity = np.array([0, 0, -9.81])

    mu = 0.3
    footSize = 0.05
    cone_box = np.array([0.1, 0.05])
    minNforce = 200
    maxNforce = 1200  # This may be still too low

    # ###### WALKING GEOMETRY #########
    xForward = 0.1  # step size
    swingApex = 0.2  # foot height
    footSeparation = (
        0.2  # 0.005 # Correction in y to push the feet away from each other
    )

    normal_height = 0.87
    omega = np.sqrt(-gravity[2] / normal_height)

    # ##### CROCO - CONFIGURATION ########
    # relevant frame names

    rightFoot = rf_frame_name = "leg_right_sole_fix_joint"
    leftFoot = lf_frame_name = "leg_left_sole_fix_joint"

    # Weights for all costs

    wFootPlacement = 10000
    wStateReg = 100
    wControlReg = 0.001
    wLimit = 1e3
    wWrenchCone = 0.05
    wCoP = 100

    weightBasePos = [0, 0, 0, 1000, 1000, 10]  # [x, y, z| x, y, z]
    weightBaseVel = [0, 0, 10, 100, 100, 10]  # [x, y, z| x, y, z]
    weightLegPos = [1, 100, 100, 0.01, 0.1, 1]  # [z, x, y, y, y, x]
    weightLegVel = [10, 10, 1, 0.1, 1, 10]  # [z, x, y, y, y, x]
    weightArmPos = [10, 10, 10, 10]  # [z, x, z, y, z, x, y]
    weightArmVel = [100, 100, 100, 100]  # [z, x, z, y, z, x, y]
    weightTorsoPos = [5, 5]  # [z, y]
    weightTorsoVel = [5, 5]  # [z, y]
    stateWeights = np.array(
        weightBasePos
        + weightLegPos * 2
        + weightTorsoPos
        # + weightArmPos * 2
        + weightBaseVel
        + weightLegVel * 2
        + weightTorsoVel
        # + weightArmVel * 2
    )

    weightuBase = "not actuated"
    weightuLeg = [1, 1, 1, 1, 1, 1]
    weightuArm = [10, 10, 10, 10]
    weightuTorso = [1, 1]
    controlWeight = np.array(
        weightuLeg * 2
        + weightuTorso
        # + weightuArm * 2
    )
    lowKinematicLimits = np.array(
        [
            -0.35,
            -0.52,
            -2.10,
            0.0,
            -1.31,
            -0.52,  # left leg
            -1.57,
            -0.52,
            -2.10,
            0.0,
            -1.31,
            -0.52,  # right leg
            -1.3,
            -0.1,
        ]
    )  # torso
    highKinematicLimits = np.array(
        [
            1.57,
            0.52,
            0.7,
            2.62,
            0.77,
            0.52,  # left leg
            0.35,
            0.52,
            0.7,
            2.62,
            0.77,
            0.52,  # right leg
            1.3,
            0.2,
        ]
    )  # torso

    th_stop = 1e-6  # threshold for stopping criterion
    th_grad = 1e-9  # threshold for zero gradient.


def save_derivatives(Lxx, Ks, xss, uss, save_name=None, save_dir=DEFAULT_SAVE_DIR):
    """
    Saves data to a compressed npz file (binary)
    """
    simu_data = {}
    simu_data["Lxx"] = Lxx
    simu_data["Ks"] = Ks
    simu_data["xss"] = xss
    simu_data["uss"] = uss
    print("Compressing & saving data...")
    if save_name is None:
        save_name = "sim_data_NO_NAME" + str(time.time())
    if save_dir is None:
        save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../data"))
    save_path = save_dir + "/" + save_name + ".npz"
    np.savez_compressed(save_path, data=simu_data)
    print("Saved data to " + str(save_path) + " !")


def load_data(npz_file):
    """
    Loads a npz archive of sim_data into a dict
    """
    d = np.load(npz_file, allow_pickle=True, encoding="latin1")
    return d["data"][()]


# ####### CONFIGURATION  ############
# ### RobotWrapper
design_conf = dict(
    urdfPath=conf.modelPath + conf.URDF_SUBPATH,
    srdfPath=conf.modelPath + conf.SRDF_SUBPATH,
    leftFootName=conf.lf_frame_name,
    rightFootName=conf.rf_frame_name,
    robotDescription="",
    controlledJointsNames=[
        "root_joint",
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "torso_1_joint",
        "torso_2_joint",
    ],
)
design = RobotDesigner()
design.initialize(design_conf)

# Vector of Formulations
MM_conf = dict(
    timeStep=conf.DT,
    gravity=conf.gravity,
    mu=conf.mu,
    coneBox=conf.cone_box,
    minNforce=conf.minNforce,
    maxNforce=conf.maxNforce,
    comHeight=conf.normal_height,
    omega=conf.omega,
    footSize=conf.footSize,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wWrenchCone=conf.wWrenchCone,
    wCoP=conf.wCoP,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    lowKinematicLimits=conf.lowKinematicLimits,
    highKinematicLimits=conf.highKinematicLimits,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMaker()
formuler.initialize(MM_conf, design)
all_models = formuler.formulateHorizon(length=conf.T)
ter_model = formuler.formulateTerminalStepTracker(Support.DOUBLE)

# Horizon

H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, ter_model)

x_init = [design.get_x0()] * (conf.T + 1)
u_init = [np.zeros(horizon.ddp.problem.runningModels[0].nu)] * conf.T
horizon.ddp.setCallbacks([crocoddyl.CallbackVerbose()])

horizon.ddp.solve(x_init, u_init, 1)

# False: Compare one iteration of DDP with stored trajectory
# True: store current trajectory in npz file
save_derivative = False


if not (save_derivative):
    data2 = load_data(DEFAULT_SAVE_DIR + "/trajectory_croco1_7.npz")
    Lxx2 = data2["Lxx"]
    Ks2 = data2["Ks"]
    xss = data2["xss"]
    uss = data2["uss"]

    for j in range(conf.T):
        print("node t = " + str(j))
        print(np.linalg.norm(horizon.ddp.problem.runningDatas[j].Lxx - Lxx2[j]))
        assert (
            np.linalg.norm(horizon.ddp.problem.runningDatas[j].Lxx - Lxx2[j])
            < conf.th_stop
        )
        print("Ks")
        print(np.linalg.norm(horizon.ddp.K[j] - Ks2[j]))
        assert np.linalg.norm(horizon.ddp.K[j] - Ks2[j]) < conf.th_stop
        print("xs")
        print(np.linalg.norm(horizon.ddp.xs[j] - xss[j]))
        assert np.linalg.norm(horizon.ddp.xs[j] - xss[j]) < conf.th_stop
        print("us")
        print(np.linalg.norm(horizon.ddp.us[j] - uss[j]))
        assert np.linalg.norm(horizon.ddp.us[j] - uss[j]) < conf.th_stop
    print("j = " + str(conf.T))
    print(np.linalg.norm(horizon.ddp.problem.terminalData.Lxx - Lxx2[conf.T]))
else:
    Lxx = [horizon.ddp.problem.runningDatas[i].Lxx for i in range(conf.T)]
    Ks = [horizon.ddp.K[i] for i in range(conf.T)]
    xss = [horizon.ddp.xs[i] for i in range(conf.T)]
    uss = [horizon.ddp.us[i] for i in range(conf.T)]
    Lxx.append(horizon.ddp.problem.terminalData.Lxx)
    save_derivatives(Lxx, Ks, xss, uss, save_name="new_trajectory")
