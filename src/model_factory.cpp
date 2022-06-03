#include "sobec/model_factory.hpp"
#include "sobec/designer.hpp"
#include <crocoddyl/multibody/fwd.hpp>

namespace sobec {

ModelMaker::ModelMaker(){}

ModelMaker::ModelMaker(const ModelMakerSettings &settings, const RobotDesigner &design){ 
    initialize(settings, design); 
    }

void ModelMaker::initialize(const ModelMakerSettings &settings, const RobotDesigner &design){ 
    settings_ = settings;
    design_ = design;

    //state_ = // initialize state with  pinocchio::Model>(design.get_rModel());
    //actuation_ = 

}

AMA ModelMaker::formulate_flat_walker(const Support &support){

//     // Here goes the full definition of contacts and costs to get the IAM
//     // Use the support enumeration to set the appropriate contact and wrench cone.
    
    return AMA();
}

AMA ModelMaker::formulate_stair_climber(const Support &support){

    // Here goes the full definition of contacts and costs to get the IAM
    // Use the support enumeration to set the appropriate contact and wrench cone.

    return AMA();
}

std::vector<AMA> ModelMaker::formulateHorizon(const std::vector<Support> &suports, const double &T){

    // for loop to generate a vector of IAMs

    std::vector<AMA> models = { AMA(), AMA() };

    return models;
}


}

