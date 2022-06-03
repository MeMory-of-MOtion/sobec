#ifndef SOBEC_OCP
#define SOBEC_OCP	

#include "sobec/designer.hpp"
#include "sobec/model_factory.hpp"
#include "sobec/horizon_manager.hpp"

namespace sobec{

    struct OCPSettings {
        public:
            
            // timing
            unsigned long T_total = 2000;
            unsigned long T = 100;
            unsigned long T2contact = 50;
            unsigned long T1contact = 100;
            unsigned long Tstep = T1contact + T2contact;
            unsigned long ddpIteration = 1;
            
            double Dt = 1e-2;
            double simu_step = 1e-3;

            unsigned long Nc = round(Dt / simu_step);
    };

    class OCP{

        private:

            OCPSettings settings_;
            RobotDesigner design_;
            ModelMaker model_settings_; 
            HorizonManager horizon_;
            
            Eigen::VectorXd x0_;

            Eigen::VectorXd shapeState(Eigen::VectorXd q, Eigen::VectorXd v);

            void update(unsigned long time);
            void solve(unsigned long time);

        public:
            OCP();
            OCP(OCPSettings settings);
            void initialize(OCPSettings settings);

            Eigen::VectorXd iterate(unsigned long time,
                                    Eigen::VectorXd meassured_q,
                                    Eigen::VectorXd meassured_v);

    };
}

#endif  // SOBEC_OCP