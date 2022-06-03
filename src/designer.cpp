#include "sobec/designer.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace sobec {

RobotDesigner::RobotDesigner(){}

RobotDesigner::RobotDesigner(RobotDesignerSettings settings){ initialize(settings); }

void RobotDesigner::initialize(RobotDesignerSettings settings){ 
    settings_ = settings;

    // Initialization of all atributes:

    //pinocchioControlledJoints_ =;
    //leftFootId_ =
    //rightFootId_ = 

    //ModelComplete = 
    //rModel = 
    //rdataComplete = 
    //rdata =

    //q0Complete =
    //q0 = 
    //v0Complete = 
    //v0 =

}
void RobotDesigner::updateReducedModel(Eigen::VectorXd x){
    /** x is the reduced posture, or contains the reduced posture in the first elements */
    pinocchio::forwardKinematics(rModel_, rData_, x.head(rModel_.nq));
    pinocchio::updateFramePlacements(rModel_, rData_);
}

void RobotDesigner::updateCompleteModel(Eigen::VectorXd x){
    /** x is the complete posture, or contains the complete posture in the first elements */
    pinocchio::forwardKinematics(rModelComplete_, rDataComplete_, x.head(rModelComplete_.nq));
    pinocchio::updateFramePlacements(rModelComplete_, rDataComplete_);
}

pinocchio::SE3 RobotDesigner::get_LF_frame(){
    return rData_.oMf[leftFootId_];
}

pinocchio::SE3 RobotDesigner::get_RF_frame(){
    return rData_.oMf[rightFootId_];
}

double RobotDesigner::getRobotMass(){
    double mass = 0;
    for (pinocchio::Inertia& I : rModel_.inertias)
        mass += I.mass();    
    return mass;
}

}