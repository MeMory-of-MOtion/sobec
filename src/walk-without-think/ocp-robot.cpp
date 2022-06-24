///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2022 LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "sobec/walk-without-think/ocp.hpp"

namespace sobec {

OCPRobotWrapper::OCPRobotWrapper(boost::shared_ptr<pinocchio::Model> model_,
                                 const std::string& contactKey,
                                 const std::string& referencePosture) {
  this->model = model_;

  // Search contact Ids using key name (eg all frames containing "sole_link")
  for (pinocchio::FrameIndex idx = 0; idx < model->frames.size(); ++idx) {
    if (model->frames[idx].name.find(contactKey) != std::string::npos) {
      std::cout << "Found contact " << idx << std::endl;
      contactIds.push_back(idx);
    }
  }

  // Add tow and heel frames ... TODO
  for (pinocchio::FrameIndex cid : contactIds) {
    pinocchio::SE3 towPlacement = pinocchio::SE3::Identity();
    towPlacement.translation()[0] = 0.1;
    auto cframe = model->frames[cid];
    pinocchio::Frame towFrame(
        cframe.name + "_" + "tow", cframe.parent, cframe.previousFrame,
        cframe.placement * towPlacement, pinocchio::OP_FRAME);
    towIds[cid] = model->addFrame(towFrame);

    pinocchio::SE3 heelPlacement = pinocchio::SE3::Identity();
    heelPlacement.translation()[0] = -0.1;
    pinocchio::Frame heelFrame(
        cframe.name + "_" + "heel", cframe.parent, cframe.previousFrame,
        cframe.placement * heelPlacement, pinocchio::OP_FRAME);
    heelIds[cid] = model->addFrame(heelFrame);
  }

  //   boost::shared_ptr<pinocchio::Data> data;
  data = boost::make_shared<pinocchio::Data>(*model);

  // Get ref config
  Eigen::VectorXd q0 = model->referenceConfigurations[referencePosture];

  // eval x0
  x0.resize(model->nq + model->nv);
  x0.head(model->nq) = q0;
  x0.tail(model->nv).fill(0);

  // eval COM
  com0 = pinocchio::centerOfMass(*model, *data, q0, false);

  // eval mass
  robotGravityForce =
      -pinocchio::computeTotalMass(*model) * model->gravity.linear()[2];
}

}  // namespace sobec
