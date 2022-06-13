#ifndef SOBEC_HORIZON_MANAGER
#define SOBEC_HORIZON_MANAGER

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "sobec/model_factory.hpp"
#include "sobec/designer.hpp"
#include "sobec/fwd.hpp"

namespace sobec{
	struct HorizonManagerSettings{
        public:
            std::string leftFootName = "left_sole_link";
            std::string rightFootName = "right_sole_link";
    };
			
    class HorizonManager{
        
        private:
            HorizonManagerSettings settings_;
            DDP ddp_;

            //prealocated memory:
            boost::shared_ptr<crocoddyl::CostModelResidual> cone_;
            
            //NEW ~ ocp related ~
            std::vector<Eigen::VectorXd> xs_; //using it
            std::vector<Eigen::VectorXd> us_; //using it
            
            ResidualModelFramePlacementPtr goalTrackingResidual_;
            ResidualModelContactWrenchConePtr wrenchConeResidual_;
            crocoddyl::WrenchCone wrench_cone_;
            ActivationModelQuadRefPtr quadRefActivationPtr_;
            IAM IAM_;
            IAD IAD_;
            DAM DAM_;
            //end NEW ~ ocp related ~

        public:
            HorizonManager();

            HorizonManager(const HorizonManagerSettings &horizonSettings, 
						   const Eigen::VectorXd &x0, 
						   const std::vector<AMA> &runningModels,
						   const AMA &terminalModel);

            void initialize(const HorizonManagerSettings &horizonSettings, 
						    const Eigen::VectorXd &x0, 
						    const std::vector<AMA> &runningModels,
						    const AMA &terminalModel);
            
            // OLD
            AMA ama(const unsigned long &time);
            IAM iam(const unsigned long &time);
            DAM dam(const unsigned long &time);
            Cost costs(const unsigned long &time);
            Contact contacts(const unsigned long &time);
            IAD data(const unsigned long &time);
            
            boost::shared_ptr<crocoddyl::StateMultibody> state(const unsigned long &time);
            // // void setResidualReference(unsigned long time, const std::string &name,  const auto &new_value);
            // // Try to avoid the "auto"
            // // void setResidualReferences(unsigned long time, const std::string &name);

            void setActuationReference(const unsigned long &time, const Eigen::VectorXd &reference);
            void setBalancingTorque(const unsigned long &time, const Eigen::VectorXd x);
            void setBalancingTorque(const unsigned long &time);
            void setPoseReferenceLF(const unsigned long &time, const pinocchio::SE3 &ref_placement);
            void setPoseReferenceRF(const unsigned long &time, const pinocchio::SE3 &ref_placement);
            void activateContactLF(const unsigned long &time);
            void activateContactRF(const unsigned long &time);
            void removeContactLF(const unsigned long &time);
            void removeContactRF(const unsigned long &time);
            void setForceReferenceLF(const unsigned long &time, const eVector6 &reference);
            void setForceReferenceRF(const unsigned long &time, const eVector6 &reference);
            void setSwingingLF(const unsigned long &time);
            void setSwingingRF(const unsigned long &time);
            void setSupportingLF(const unsigned long &time);
            void setSupportingRF(const unsigned long &time);
            //end OLD

            // NEW ~ ocp related ~
            void updateIAM(const unsigned long &time);
            void updateDAM(const unsigned long &time);
            Cost costs();
            Contact contacts();
            // IAD data(const unsigned long &time);
            
            void setPlacementReferenceRF(const pinocchio::SE3 &new_ref);
            void setPlacementReferenceLF(const pinocchio::SE3 &new_ref);

            void activateContactLF();
            void activateContactRF();
            void removeContactLF();
            void removeContactRF();
            void setForceReferenceLF(const eVector6 &reference);
            void setForceReferenceRF(const eVector6 &reference);
            void setSwingingLF(const unsigned long &time, 
							   const pinocchio::SE3 &right_placement,
							   const pinocchio::SE3 &left_placement,
							   const eVector6 &ref_wrench);
            void setSwingingRF(const unsigned long &time, 
                               const pinocchio::SE3 &right_placement,
                               const pinocchio::SE3 &left_placement,
                               const eVector6 &ref_wrench);
            void setSupportingFeet(const unsigned long &time,
								   const pinocchio::SE3 &right_placement,
								   const pinocchio::SE3 &left_placement,
								   const eVector6 &ref_wrench);
            //end NEW ~ ocp related ~

            //std::vector<std::string> get_contacts(const unsigned long &time);
            //std::vector<Eigen::VectorXd> preview_states();
            //std::vector<Eigen::VectorXd> preview_actions();
            void recede(const IAM &new_model, const IAD &new_data);
            void recede(const IAM &new_model);
            void recede();

            unsigned long get_size();

            //NEW ~ ocp related ~
            void solve(const std::vector<Eigen::VectorXd> xs, 
                       const std::vector<Eigen::VectorXd> us, 
                       const std::size_t &ddpIteration,
                       const bool &is_feasible=false);
            void solve(const Eigen::VectorXd &measured_x,           //Using it
                       const std::size_t &ddpIteration,
                       const bool &is_feasible=false);
            Eigen::VectorXd currentTorques(const Eigen::VectorXd &measured_x);
            
            Eigen::VectorXd get_xs0(){return xs_[0];}
            Eigen::VectorXd get_us0(){return us_[0];}
            Eigen::MatrixXd get_K0(){return ddp_->get_K()[0];}  
            DDP get_ddp(){return ddp_;}
            void set_ddp(const DDP &ddp){ddp_ = ddp;}
            
            //end NEW ~ ocp related ~
    };
}  // namespace
#endif // SOBEC_HORIZON_MANAGER
