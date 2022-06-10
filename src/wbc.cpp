#include "sobec/wbc.hpp"

namespace sobec {

	
	WBC::WBC(){}

	WBC::WBC(const WBCSettings &settings,
             const RobotDesigner &design,
             const HorizonManager &horizon,
             const Eigen::VectorXd &q0,
             const Eigen::VectorXd &v0){ 
		initialize(settings,design, horizon, q0, v0);
	}

	void WBC::initialize(const WBCSettings &settings,
					     const RobotDesigner &design,
                         const HorizonManager &horizon,
					     const Eigen::VectorXd &q0,
						 const Eigen::VectorXd &v0){
							 
		settings_ = settings;
		designer_ = design;
        horizon_ = horizon;
		x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
        x0_ << shapeState(q0, v0);

		t_takeoff_RF_ = Eigen::ArrayXi::LinSpaced(settings_.horizonSteps, 0,
												   2*settings_.horizonSteps*settings_.Tstep);
		t_takeoff_RF_ += (int)settings_.T;
        t_takeoff_LF_ = t_takeoff_RF_ + settings_.Tstep;
        t_land_RF_ = t_takeoff_RF_ + settings_.TsimpleSupport;
        t_land_LF_ = t_takeoff_LF_ + settings_.TsimpleSupport;
	}

	void WBC::generateFullCycle(ModelMaker &mm){
		
		std::vector<Support> cycle;
		unsigned long takeoff_RF, land_RF, takeoff_LF, land_LF;
		takeoff_RF = 0;
		land_RF = takeoff_RF + settings_.TsimpleSupport;
		takeoff_LF = takeoff_RF + settings_.Tstep;
		land_LF = takeoff_LF + settings_.TsimpleSupport;

		for (int i; i<2*settings_.Tstep; i++){

			if (i < land_RF) 			cycle.push_back(LEFT);
			else if (i < takeoff_LF) 	cycle.push_back(DOUBLE);
			else if (i < land_LF) 		cycle.push_back(RIGHT);
			else 						cycle.push_back(DOUBLE);
		}
		std::vector<AMA> cyclicModels = mm.formulateHorizon(cycle);
		HorizonManagerSettings names = {designer_.get_LF_name(), designer_.get_RF_name()};
		fullCycle_ = HorizonManager(names, x0_, cyclicModels, cyclicModels[2*settings_.Tstep - 1]);
	}

	void WBC::updateStepCycleTiming() {
		t_takeoff_RF_ -= 1;
		t_takeoff_LF_ -= 1;
		t_land_RF_ -= 1;
		t_land_LF_ -= 1;

		if(t_land_LF_(0) < 0) t_land_LF_ += 2*settings_.Tstep;
		if(t_land_RF_(0) < 0) t_land_RF_ += 2*settings_.Tstep;
		if(t_takeoff_LF_(0) < 0) t_takeoff_LF_ += 2*settings_.Tstep; 
		if(t_takeoff_LF_(0) < 0) t_takeoff_LF_ += 2*settings_.Tstep;
	}

	bool WBC::timeToSolveDDP(const unsigned long &iteration){
		return !(iteration % settings_.Nc);
	}

	Eigen::VectorXd WBC::iterate(const unsigned long &iteration, 
								 const Eigen::VectorXd &q_current,
								 const Eigen::VectorXd &v_current,
								 const bool &is_feasible){

		x_current_ = shapeState(q_current, v_current);
		if (timeToSolveDDP(iteration)){
			// ~~TIMING~~ //
			updateStepCycleTiming();
			recedeWithFullCycle();

			// ~~REFERENCES~~ //
			designer_.updateReducedModel(x_current_);
			//setDesiredFeetPose(iteration, settings_.T - 1);
		}
		return horizon_.currentTorques(x_current_);
	}

	// void WBC::setDesiredFeetPoses(unsigned long iteration, unsigned long time){


	// }

	void WBC::recedeWithFullCycle() {
		horizon_.recede(fullCycle_.iam(0), fullCycle_.data(0));
		fullCycle_.recede();
	}

    Eigen::VectorXd WBC::shapeState(Eigen::VectorXd q, Eigen::VectorXd v){ // TEST IT
        x_current_.head<7>() = q.head<7>();
        x_current_.segment<6>(designer_.get_rModel().nq);
		
        for(unsigned long joint_id : designer_.get_controlledJointsIDs()){
			if(joint_id > 1){
				x_current_(joint_id + 7) = q(joint_id + 5);
				x_current_(designer_.get_rModel().nq + joint_id + 6) = v(joint_id + 4);
			}
        }

        // qc = q[[i + 5 for i in self.design.pinocchioControlledJoints[1:]]]
        // vc = v[[i + 4 for i in self.design.pinocchioControlledJoints[1:]]]
    
        return x_current_;
    }

	
}