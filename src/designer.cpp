#include "sobec/designer.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

namespace sobec {

RobotDesigner::RobotDesigner(){}

RobotDesigner::RobotDesigner(const RobotDesignerSettings &settings){ initialize(settings); }

void RobotDesigner::initialize(const RobotDesignerSettings &settings){ 
    
    if (settings.controlled_joints_names[0] != "root_joint")
	{
        throw std::invalid_argument("the joint at index 0 must be called 'root_joint' ");
	}
    settings_ = settings;
    //@TODO: check that the urdf file and srdf file exist, as done in aig.

    // COMPLETE MODEL //
    pinocchio::Model pin_model_complete;
    pinocchio::urdf::buildModel(settings_.urdf_path, pinocchio::JointModelFreeFlyer(), rModelComplete_);
    rDataComplete_ = pinocchio::Data(rModelComplete_);

    pinocchio::srdf::loadReferenceConfigurations(rModelComplete_, settings_.srdf_path, false);
    q0Complete_ = rModelComplete_.referenceConfigurations["half_sitting"];
    v0Complete_ = Eigen::VectorXd::Zero(rModelComplete_.nv);

    // REDUCED MODEL //


    // Initialization of all atributes:

    //pinocchioControlledJoints_ =;
    //leftFootId_ =
    //rightFootId_ = 

    //rModelComplete_ = done
    //rModel_ = 
    //rdataComplete_ = done
    //rdata_ =

    //q0Complete_ = done
    //q0_ = 
    //v0Complete_ = done
    //v0_ =

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