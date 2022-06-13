#ifndef SOBEC_MODEL_FACTORY
#define SOBEC_MODEL_FACTORY

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "sobec/designer.hpp"
#include "sobec/fwd.hpp"

namespace sobec{

    enum Support {LEFT, RIGHT, DOUBLE};

    struct ModelMakerSettings{
        public:
            
            // Timing
            double timeStep = 0.01;

            // physics
            eVector3 gravity = eVector3(0, 0, -9.81);

            double mu = 0.3;
            eVector2 coneBox = eVector2(0.1, 0.05); // half lenght and width
            double minNforce = 200.0;
            double maxNforce = 1200;
            
            double comHeight = 0.87;
            double omega = -comHeight/gravity(2);

            // Croco configuration
            double wFootPlacement = 0;//1000;
            double wStateReg = 0;//100;
            double wControlReg = 0;//0.001;
            double wLimit = 0;//1e3;
            double wVCoM = 0;//0;
            double wWrenchCone = 0;//0.05;
            double wFootTrans = 0;//100;
            double wFootXYTrans = 0;//0;
            double wFootRot = 0;// 100;
            double wGroundCol = 0;// 0.05;

            Eigen::VectorXd stateWeights;
            Eigen::VectorXd controlWeights;

            double th_stop = 1e-6; // threshold for stopping criterion
            double th_grad = 1e-9; // threshold for zero gradient.
    };
    class ModelMaker{

        private:
            ModelMakerSettings settings_;
            RobotDesigner designer_;

            boost::shared_ptr<crocoddyl::StateMultibody> state_;
            boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> actuation_;
            Eigen::VectorXd x0_;

            

        public:
            ModelMaker();
            ModelMaker(const ModelMakerSettings &settings, const RobotDesigner &design);
            void initialize(const ModelMakerSettings &settings, const RobotDesigner &design);
            bool initialized_ = false;

            AMA formulateStepTracker(const Support &support = Support::DOUBLE);
            // AMA formulate_flat_walker(const Support &support = Support::DOUBLE);
            AMA formulate_stair_climber(const Support &support = Support::DOUBLE);

            std::vector<AMA> formulateHorizon(const std::vector<Support> &supports);
            std::vector<AMA> formulateHorizon(const int &T);
            ModelMakerSettings &get_settings(){return settings_;}

            // formulation parts: 
            void defineFeetContact(Contact &contactCollector, const Support &support = Support::DOUBLE);
            void defineFeetWrenchCost(Cost &costCollector, const Support &support = Support::DOUBLE);
            void defineFeetTracking(Cost &costCollector);
            void definePostureTask(Cost &costCollector);
            void defineActuationTask(Cost &costCollector);
            void defineJointLimits(Cost &costCollector);
            void defineCoMVelocity(Cost &costCollector);

            boost::shared_ptr<crocoddyl::StateMultibody> getState(){return state_;}
            void setState(const boost::shared_ptr<crocoddyl::StateMultibody> &new_state){state_ = new_state;} 
            boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> getActuation(){return actuation_;}
            void setActuation(const boost::shared_ptr<crocoddyl::ActuationModelFloatingBase> &new_actuation){actuation_ = new_actuation;}
    };

}  // namespace
#endif // SOBEC_MODEL_FACTORY
