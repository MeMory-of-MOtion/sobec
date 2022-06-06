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
            std::string leftFootName = "leftSoleContact";
            std::string rightFootName = "rightSoleContact";
    };
			
    class HorizonManager{
        
        private:
            HorizonManagerSettings horizonSettings_;
            DDP ddp_;
            
            std::vector<Eigen::VectorXd> xs_;
            std::vector<Eigen::VectorXd> us_;
            
            sobec::RobotDesigner designer_;
            sobec::ModelMaker modelMaker_;
            
            ResidualModelFramePlacementPtr goalTrackingResidual_;
            ResidualModelContactWrenchConePtr wrenchConeResidual_;
            crocoddyl::WrenchCone wrench_cone_;
            ActivationModelQuadRefPtr quadRefActivationPtr_;
            IAM IAM_;
            IAD IAD_;
            DAM DAM_;

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

            void updateIAM(const unsigned long &time);
            void updateDAM(const unsigned long &time);
            Cost costs();
            Contact contacts();
            IAD data(const unsigned long &time);
            
            void setPlacementReferenceRF(const pinocchio::SE3 &new_ref);
            void setPlacementReferenceLF(const pinocchio::SE3 &new_ref);

            void activateContactLF();
            void activateContactRF();
            void removeContactLF();
            void removeContactRF();
            void setForceReferenceLF(const eVector6 &reference);
            void setForceReferenceRF(const eVector6 &reference);
            void setSwingingLF(const unsigned long &time, const pinocchio::SE3 &ref_placement,  const eVector6 &ref_wrench);
            void setSwingingRF(const unsigned long &time, const pinocchio::SE3 &ref_placement,  const eVector6 &ref_wrench);
            void setSupportingFeet(const unsigned long &time, const eVector6 &ref_wrench);

            //std::vector<std::string> get_contacts(const unsigned long &time);
            //std::vector<Eigen::VectorXd> preview_states();
            //std::vector<Eigen::VectorXd> preview_actions();
            void recede(IAM new_model, IAD new_data);
            void recede(IAM new_model);
            void recede();

            unsigned long get_size();
            
            void solve_ddp(const std::vector<Eigen::VectorXd> xs, 
                           const std::vector<Eigen::VectorXd> us, 
                           const std::size_t &ddpIteration);
            void solveControlCycle(const Eigen::VectorXd &measured_x,
                                   const std::size_t &ddpIteration);
            
            Eigen::VectorXd get_xs0(){return xs_[0];}
            Eigen::VectorXd get_us0(){return us_[0];}
            Eigen::VectorXd get_K0(){return ddp_->get_K()[0];}
            
            
    };

}  // namespace
#endif // SOBEC_HORIZON_MANAGER
