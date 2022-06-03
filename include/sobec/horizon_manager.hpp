#ifndef SOBEC_HORIZON_MANAGER
#define SOBEC_HORIZON_MANAGER

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "sobec/fwd.hpp"

namespace sobec{

    struct HorizonManagerSettings{
        public:
            std::string leftFootName = "";
            std::string rightFootName = "";
    };

    class HorizonManager{
        
        private:
            DDP ddp_;
            HorizonManagerSettings settings_;

        public:
            HorizonManager();

            HorizonManager(const HorizonManagerSettings &settings, 
                           const Eigen::VectorXd &x0, 
                           const std::vector<AMA> &runningModels,
                           const IAM &terminalModel);

            void initialize(const HorizonManagerSettings &settings, 
                            const Eigen::VectorXd &x0, 
                            const std::vector<AMA> &runningModels,
                            const IAM &terminalModel);

            AMA ama(const unsigned long &time);
            IAM iam(const unsigned long &time);
            DAM dam(const unsigned long &time);
            Cost costs(const unsigned long &time);
            Contact contacts(const unsigned long &time);
            IAD data(const unsigned long &time);
            
            // void setResidualReference(unsigned long time, const std::string &name,  const auto &new_value);
            // Try to avoid the "auto"
            // void setResidualReferences(unsigned long time, const std::string &name);

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

            std::vector<std::string> get_contacts(const unsigned long &time);
            std::vector<Eigen::VectorXd> preview_states();
            std::vector<Eigen::VectorXd> preview_actions();

            void recede(IAM new_model, IAD new_data);
            void recede(IAM new_model);
            void recede();

            unsigned long get_size();
            
    };

}  // namespace
#endif // SOBEC_HORIZON_MANAGER