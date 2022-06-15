///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////


#include "sobec/ocp-walk.hpp"

namespace sobec {

  OCPRobotWrapper::
  OCPRobotWrapper( boost::shared_ptr<pinocchio::Model> model_,
                   const std::string & contactKey,
                   const std::string & referencePosture)
  {
    this->model = model_;
      
    // Search contact Ids using key name (eg all frames containing "sole_link")
    for( pinocchio::FrameIndex idx = 0; idx<model->frames.size(); ++idx )
      {
        if(model->frames[idx].name.find(contactKey) != std::string::npos)
          {
            std::cout << "Found contact " << idx << std::endl;
            contactIds.push_back(idx);
          }
      }

    // Add tow and heel frames ... TODO
    for( pinocchio::FrameIndex cid: contactIds )
      {
        towIds[cid] = cid;
        heelIds[cid] = cid;
      }

    //   boost::shared_ptr<pinocchio::Data> data;
    data = boost::make_shared<pinocchio::Data>( *model );

    // Get ref config
    Eigen::VectorXd q0 = model->referenceConfigurations[referencePosture];

    // eval x0
    x0.resize(model->nq+model->nv);
    x0.head(model->nq) = q0;
    x0.tail(model->nv).fill(0);
    
    // eval COM
    com0 = pinocchio::centerOfMass(*model,*data,q0,false);

    // eval mass
    robotGravityForce = pinocchio::computeTotalMass(*model) * model->gravity.linear()[2];
    
  }


  
  boost::shared_ptr<IntegratedActionModelEuler> OCPWalk::buildRunningModel()
  {
    return NULL;
  }
  boost::shared_ptr<IntegratedActionModelEuler> OCPWalk::buildTerminalModel()
  {
    return NULL;
  }



  
} // namespace sobec
