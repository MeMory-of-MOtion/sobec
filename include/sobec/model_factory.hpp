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
            
            // Timestep
            double timeStep_ = 0.01;

            // physics
            eVector3 gravity = eVector3(0, 0, -9.81);

            double mu = 0.3;
            eVector2 coneBox = eVector2(0.1, 0.05); // half lenght and width
            double minNforce = 200.0;
            double maxNforce = 1200;
            
            double fzRef1Contact = 1000.;
            double fzRef2Contact = 500.;
            
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
            double wFootFix = 0;//100;
            double wFootXYTrans = 0;//0;
            double wFootRot = 0;// 100;
            double wGroundCol = 0;// 0.05;

            // eVector6 weightBasePos = eVector6::Zero(); //by default it should be (0, 0, 1000, 1000, 1000, 10);    // [x, y, z| x, y, z]
            // eVector6 weightBaseVel = eVector6::Zero(); //by default it should be (0, 0, 10, 1000, 1000, 10);      // [x, y, z| x, y, z]
            // eVector6 weightLegPos = eVector6::Zero(); //by default it should be (1, 0.1, 0.01, 0.01, 0.1, 1);    // [z, x, y, y, y, x]
            // eVector6 weightLegVel = eVector6::Zero(); //by default it should be (10, 10, 1, 0.1, 1, 10);               //[z, x, y, y, y, x]
            // eVector4 weightArmPos = eVector4::Zero();//(10, 10, 10, 10);      // [z, x, z, y, z, x, y]
            // eVector4 weightArmVel = eVector4::Zero();//(100, 100, 100, 100);  // [z, x, z, y, z, x, y]
            // eVector2 weightTorsoPos = eVector2::Zero(); //(5, 5);       //[z, y]
            // eVector2 weightTorsoVel = eVector2::Zero(); //(5, 5);       //[z, y]
            Eigen::VectorXd stateWeights;
            
            // eVector6 weightuLeg = eVector6::Zero(); // [1, 1, 1, 1, 1, 1]
            // eVector6 weightuArm = eVector6::Zero(); // [10, 10, 10, 10]
            // eVector2 weightuTorso = eVector2::Zero(); //(1, 1); // [1, 1]
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

            AMA formulate_flat_walker(const Support &support);
            AMA formulate_stair_climber(const Support &support);

            std::vector<AMA> formulateHorizon(const std::vector<Support> &supports);

    };

}  // namespace
#endif // SOBEC_MODEL_FACTORY
