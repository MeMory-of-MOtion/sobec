"""
This file define the class WalkParam, containing all the weights needed for the
OCP.
"""

import numpy as np
import sobec

class StateRelatedParams:
    """These params are only acceptable for some robots"""

    def __init__(self, stateImportance, stateTerminalImportance, controlImportance):
        self.stateImportance = stateImportance
        self.stateTerminalImportance = stateTerminalImportance
        self.controlImportance = controlImportance


# ### TALOS INFO ##################################################
# ### TALOS INFO ##################################################
# ### TALOS INFO ##################################################


class TalosInfo:
    """Store here the main info we need for Talos,
    to be use in the robot_2_state map for convenience."""

    basisQWeights = [0, 0, 0, 50, 50, 0]
    legQWeights = [5, 5, 1, 2, 1, 1]
    torsoQWeights = [10, 10]
    armQWeight = 3
    basisVWeights = [0, 0, 0, 3, 3, 1]  # ## was 003331
    legVWeights = [1] * 6
    torsoVWeights = [20] * 2
    armVWeight = 2


# ### STATE FOR EACH ROBOT ##################################################
# ### STATE FOR EACH ROBOT ##################################################
# ### STATE FOR EACH ROBOT ##################################################

Robot_2_StateMap = {
    "talos_14": StateRelatedParams(
        stateImportance=np.array(
            TalosInfo.basisQWeights
            + TalosInfo.legQWeights * 2
            + [TalosInfo.armQWeight] * 2
            + TalosInfo.basisVWeights
            + TalosInfo.legVWeights * 2
            + [TalosInfo.armVWeight] * 2
        ),
        stateTerminalImportance=np.array([3, 3, 0, 0, 0, 30] + [0] * 14 + [1] * 20),
        controlImportance=np.array([1] * 14),
    ),
    "talos_12": StateRelatedParams(
        stateImportance=np.array(
            TalosInfo.basisQWeights
            + TalosInfo.legQWeights * 2
            + TalosInfo.basisVWeights
            + TalosInfo.legVWeights * 2
        ),
        stateTerminalImportance=np.array([3, 3, 0, 0, 0, 30] + [0] * 12 + [1] * 18),
        controlImportance=np.array([1] * 12),
    ),
}

# ### MAIN PARAM CLASS ##################################################
# ### MAIN PARAM CLASS ##################################################
# ### MAIN PARAM CLASS ##################################################


class WalkParams:

    # ### WEIGHTS
    # Weights for the importance of cost functions.  Weights are multiply to
    # the residual squared Importance terms are multiplied during the
    # activation (hence are not squared).

    refTorqueWeight = 0
    refStateWeight = 1e-1
    # [DEPREC] flatBaseWeight = 0  # 20
    forceImportance = np.array([1, 1, 0.1, 10, 10, 2])
    coneAxisWeight = 2e-4
    comWeight = 0  # 20
    vcomImportance = np.array([0.0, 0, 1])
    vcomWeight = 1
    # [DEPREC] acomWeight = 0  # 16*DT
    copWeight = 2
    verticalFootVelWeight = 20
    # [DEPREC] footVelWeight = 0  # 20
    # [DEPREC] footAccWeight = 0  # 2
    flyHighWeight = 200
    groundColWeight = 200
    conePenaltyWeight = 0
    feetCollisionWeight = 1000

    # [DEPREC] lowbandwidthweight = 0  # 2e-1
    # [DEPREC] minTorqueDiffWeight = 0  # 2e-2

    refForceWeight = 10
    # [DEPREC] contiForceWeight = 0

    impactAltitudeWeight = 20000
    impactVelocityWeight = 10000
    impactRotationWeight = 200
    refMainJointsAtImpactWeight = 0  # 2e2 # For avoinding crossing legs

    stateTerminalWeight = 20  # 2000
    # [DEPREC] terminalNoVelocityWeight = 2000
    # [DEPREC] terminalXTargetWeight = 0  # ##DDP## 2000

    # ## Other terms related to the cost functions
    # [DEPREC] enforceMinimalFootDistance = False

    refFootFlyingAltitude = 7e-2
    flyHighSlope = 3 / refFootFlyingAltitude
    footMinimalDistance = 0.2  # (.17 is the max value wrt initial config)
    # [DEPREC] soleCollision = True
    # [DEPREC] towCollision = False
    # [DEPREC] heelCollision = False
    mainJointIds = [
        "leg_%s_%s_joint" % (side, idx)
        for side in ["left", "right"]
        for idx in [1, 2, 4]
    ]
    vcomRef = np.array([0.05, 0, 0])

    footSize = 0.05

    # ## Contact parameters for the kkt dynamics
    kktDamping = 0  # 1e-6
    baumgartGains = np.array([0, 100])
    transitionDuration = 4 ### Beware, this is not used by the Py ocp, only by the c++ ocp
    
    # ## Parameters related to the solver
    solver_th_stop = 1e-3
    solver_maxiter = 2
    solver_reg_min = 1e-6

    # ## Parameter related to the time lines
    DT = 0.010
    Tstart = int(0.3 / DT)
    Tsingle = int(0.8 / DT)  # 60
    # I prefer an even number for Tdouble
    Tdouble = 2 * int(np.round(0.11 / DT / 2 - 0.75)) + 1  # 11
    Tend = int(0.3 / DT)
    Tmpc = int(1.6 / DT)  # 120

    # ## Parameters related to the IO file (load and save)
    guessFile = None
    saveFile = "/tmp/sobec.npy"
    showPreview = False

    # ## Parameters related to the control environment
    # max magnitude of the multiplicative joint torque noise, expressed as a percentage
    # (i.e. 1=100%)
    torque_noise = 0.0

    def __init__(self, robotName):
        """
        Init from the robot name used as a key to
        selec the info related to the state dimension.
        """
        w = Robot_2_StateMap[robotName]
        self.stateImportance = w.stateImportance
        self.stateTerminalImportance = w.stateTerminalImportance
        self.controlImportance = w.controlImportance



# ### AD HOC CODE GENERATION ############################################
# The method to call is generateParamFileForTheRobot. See main for an example.
def nparrayToCpp(v):
    res = ' '
    first = ''
    for vi in v:
        res += '%s %.10f' % (first,vi)
        first = ','
    return res

def keyValueToCpp(objName,k,v):
    if isinstance(v,int):
        res ='  %s->%s = %d; ' % ( objName, k, v )
    elif isinstance(v,float):
        res ='  %s->%s = %.10f; ' % ( objName,k, v )
    elif isinstance(v,np.ndarray):
        res ='  %s->%s.resize(%d);' % ( objName,k,len(v) )
        res += '%s->%s << %s;' % ( objName,k, nparrayToCpp(v))
    elif isinstance(v,list) and isinstance(v[0],str):
        res = ' %s->%s = { "%s" ' % (objName,k,v[0])
        for vi in v[1:]:
            res += ', "%s"' % vi
        res += ' };'
    else:
        print(' *** Error, the type of <%s> is not implemented (v=%s) ' %(k,str(v)))
        #raise TypeError
    return res

def generateParamsFromCppClass(pyobj,cppName,cppClass,verbose=True):
    res = ''
    for k in cppClass.__dict__.keys():
        if k[:2] == '__': continue
        if not hasattr(pyobj,k):
            if verbose:
                res += '  // *** Cannot find field <%s> in python params object.\n' % k
            continue
        v = pyobj.__getattribute__(k)
        res += keyValueToCpp(cppName,k,v) + '\n'
    return res


def generateParamFileForTheRobot(params,robot = None):
    '''
    From a param object, generate the c++ code for initializing the c++ ocp/mpc

    '''
    res = '''
#include <sobec/ocp-walk.hpp>
#include <sobec/mpc-walk.hpp>

void initParamsFromAutomaticallyGeneratedCode(boost::shared_ptr<sobec::OCPWalkParams> params)
{
%s
}
void initMPCFromAutomaticallyGeneratedCode(boost::shared_ptr<sobec::MPCWalkParams> mpcparams)
{
%s
}
''' % (
   
    generateParamsFromCppClass(params,'params',sobec.OCPWalkParams),
    generateParamsFromCppClass(params,'mpcparams',sobec.MPCWalkParams),
    )

    if robot is not None:
        res += '''
bool checkAutomaticallyGeneratedCodeCompatibility(boost::shared_ptr<sobec::OCPRobotWrapper> robot)
{
  bool res = true;
  res &= (robot->model->nv == %d);
  res &= (robot->model->nq == %d);
  return res;
}
''' % (robot.model.nq, robot.model.nv)
    
    return res


if __name__ == "__main__":
    import example_robot_data as robex
    
    params = WalkParams('talos_14')
    robot = robex.load('talos_legs')
    print(generateParamFileForTheRobot(params,robot))

    
