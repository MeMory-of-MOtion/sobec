#ifndef SOBEC_WBC_NOTHINKING
#define SOBEC_WBC_NOTHINKING

#include <pinocchio/fwd.hpp>

// include pinocchio first

#include <ndcurves/bezier_curve.h>
#include <ndcurves/fwd.h>
#include <ndcurves/piecewise_curve.h>
#include <ndcurves/polynomial.h>
#include <ndcurves/se3_curve.h>

#include "sobec/walk-with-traj/wbc.hpp"
#include "sobec/walk-with-traj/designer.hpp"
#include "sobec/walk-with-traj/horizon_manager.hpp"
#include "sobec/walk-with-traj/model_factory_nothinking.hpp"

namespace sobec {

/// @todo: in order to switch between locomotions safely, incorporate a terminal
/// constraint
/// @todo: bind these enumerations

class WBCNoThinking : public WBC {
  /**
   * Form to use this class:
   * 1) The function iterate produces the torques to command.
   * 2) All cost references must be updated separtely in the control loop.
   *
   */

 protected:

  // timings
  void updateNonThinkingReferences();

  // References for costs:
  eVector3 ref_com_vel_;
  eVector3 ref_com_;

 public:
  WBCNoThinking();
  WBCNoThinking(const WBCSettings &settings, const RobotDesigner &design,
      const HorizonManager &horizon, const Eigen::VectorXd &q0,
      const Eigen::VectorXd &v0, const std::string &actuationCostName);

  void initialize(const WBCSettings &settings, const RobotDesigner &design,
                  const HorizonManager &horizon, const Eigen::VectorXd &q0,
                  const Eigen::VectorXd &v0,
                  const std::string &actuationCostName);

  void generateWalkingCycle(ModelMakerNoThinking &mm);

  void generateStandingCycle(ModelMakerNoThinking &mm);

  void iterate(const Eigen::VectorXd &q_current,
               const Eigen::VectorXd &v_current, bool is_feasible);


  // USER REFERENCE SETTERS AND GETTERS
  
  const eVector3 &getCoMRef() { return ref_com_;}
  void setCoMRef(eVector3 ref_com) { ref_com_ = ref_com;}
  
  const eVector3 &getVelRef_COM() {
    return ref_com_vel_;
  }
  void setVelRef_COM(eVector3 ref_com_vel) {
    ref_com_vel_ = ref_com_vel;
  }
  // For the python bindings:
  eVector3 &ref_com() { return ref_com_; }
  eVector3 &ref_com_vel() { return ref_com_vel_; }
};
}  // namespace sobec

#endif  // SOBEC_OCP
