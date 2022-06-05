#include "sobec/designer.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

namespace sobec {

RobotDesigner::RobotDesigner(){}

RobotDesigner::RobotDesigner(const RobotDesignerSettings &settings){ initialize(settings); }

void RobotDesigner::initialize(const RobotDesignerSettings &settings){ 
    settings_ = settings;

    // COMPLETE MODEL //

	if (settings_.robotDescription.size() > 0)
	{
		pinocchio::urdf::buildModelFromXML(settings_.robotDescription, pinocchio::JointModelFreeFlyer(),rModelComplete_);
		std::cout<<"### Build pinocchio model from rosparam robot_description."<<std::endl;
	}
	else if (settings_.urdfPath.size() > 0)
	{
		pinocchio::urdf::buildModel(settings_.urdfPath, pinocchio::JointModelFreeFlyer(),rModelComplete_);
		std::cout<<"### Build pinocchio model from urdf file."<<std::endl;
	}
    else
    {
        throw std::invalid_argument("the urdf file, or robotDescription must be specified.");
    }
    rDataComplete_ = pinocchio::Data(rModelComplete_);

    pinocchio::srdf::loadReferenceConfigurations(rModelComplete_, settings_.srdfPath, false);
    pinocchio::srdf::loadRotorParameters(rModelComplete_, settings_.srdfPath, false);
    q0Complete_ = rModelComplete_.referenceConfigurations["half_sitting"];
    v0Complete_ = Eigen::VectorXd::Zero(rModelComplete_.nv);

    // REDUCED MODEL //

    if (settings_.controlledJointsNames[0] != "root_joint")
	{
        throw std::invalid_argument("the joint at index 0 must be called 'root_joint' ");
	}

    // Check if listed joints belong to model
    for(std::vector<std::string>::const_iterator it = settings_.controlledJointsNames.begin(); it != settings_.controlledJointsNames.end(); ++it)
	{
		const std::string &joint_name = *it;
		std::cout << joint_name << std::endl;
		std::cout << rModelComplete_.getJointId(joint_name) << std::endl;
		if(not(rModelComplete_.existJointName(joint_name)))
		{
			std::cout << "joint: " << joint_name << " does not belong to the model" << std::endl;
		}
	}

    // making list of blocked joints
    std::vector<unsigned long> locked_joints_id;
	for(std::vector<std::string>::const_iterator it = rModelComplete_.names.begin() + 1;it != rModelComplete_.names.end(); ++it)
	{
		const std::string &joint_name = *it;
		if(std::find(settings_.controlledJointsNames.begin(), settings_.controlledJointsNames.end(), joint_name) == settings_.controlledJointsNames.end())
		{
			locked_joints_id.push_back(rModelComplete_.getJointId(joint_name));
		}
	}

	rModel_ = pinocchio::buildReducedModel(rModelComplete_, locked_joints_id, q0Complete_);
    rData_ = pinocchio::Data(rModel_);

    pinocchio::srdf::loadReferenceConfigurations(rModel_,settings_.srdfPath, false);
	pinocchio::srdf::loadRotorParameters(rModel_, settings_.srdfPath, false);
    q0_ = rModel_.referenceConfigurations["half_sitting"];
    v0_ = Eigen::VectorXd::Zero(rModel_.nv);
    x0_ << q0_, v0_;
    
    // Generating list of indices for controlled joints //
    for(std::vector<std::string>::const_iterator it = rModel_.names.begin()+1;it != rModel_.names.end(); ++it)
	{
		const std::string & joint_name = *it;
		if(std::find(settings_.controlledJointsNames.begin(), settings_.controlledJointsNames.end(), joint_name) != settings_.controlledJointsNames.end())
		{
			controlled_joints_id_.push_back((long)rModel_.getJointId(joint_name) - 2);
		}
	}

    leftFootId_ = rModel_.getFrameId(settings_.leftFootName);
    rightFootId_ = rModel_.getFrameId(settings_.rightFootName);

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
