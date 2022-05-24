import pinocchio as pin
import numpy as np
from numpy.linalg import norm, inv, pinv, svd, eig
from pinbullet import SimuProxy
from ocp_walk_feet_traj import OCP


################################################################################
### HYPER PARAMS
################################################################################

# Total number of nodes of the simulation
T_total = 2000

### SIMU #############################################################################################
### Load urdf model in pinocchio and bullet
models = SimuProxy()
models.loadExampleRobot("talos")
models.loadBulletModel()
models.freeze(
    [
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
)
models.setTorqueControlMode()
models.setTalosDefaultFriction()


### OCP ########################################################################
### OCP ########################################################################
# Init crocoddyl
ocp = OCP(models.rmodel)
ocp.initLocomotionPattern()

### DISPLAY ####################################################################
# Init Gepetto viewer
viz = pin.visualize.GepettoVisualizer(
    models.rmodel, models.gmodel_col, models.gmodel_vis
)
viz.initViewer(loadModel=True)
viz.display(models.rmodel.q0)

### MAIN LOOP ##################################################################

hx = []
hu = []

# FOR LOOP
for s in range(T_total):

    ################################################################################
    ## Log and display
    viz.display(ocp.ddp.xs[0][: models.rmodel.nq])
    hx.append(ocp.ddp.xs[0].copy())
    hu.append(ocp.ddp.us[0].copy())

    ################################################################################
    ## For timesteps without MPC updates
    for k in range(int(ocp.DT / 1e-3)):
        # Get simulation state
        x = models.getState()

        # Compute Ricatti feedback
        torques = ocp.ddp.us[0] + ocp.ddp.K[0] @ (ocp.state.diff(x, ocp.ddp.xs[0]))

        # Run one step of simu
        models.step(torques)

    ################################################################################
    ## If mpc update
    ### Change OCP and build warmstart
    ocp.updateOCP(ocp.ddp.xs[0])
    ### Solve
    ocp.solve(x)
