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
		/** The posture required here is the full robot posture in the order of pinicchio*/
							 
		settings_ = settings;
		designer_ = design;
        horizon_ = horizon;

		controlled_joints_id_ = designer_.get_controlledJointsIDs();
		x_internal_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);

		x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
        x0_ << shapeState(q0, v0);

		t_takeoff_RF_ = Eigen::ArrayXi::LinSpaced(settings_.horizonSteps, 0,
												   2*settings_.horizonSteps*settings_.Tstep);
		t_takeoff_RF_ += (int)settings_.T;
        t_takeoff_LF_ = t_takeoff_RF_ + settings_.Tstep;
        t_land_RF_ = t_takeoff_RF_ + settings_.TsingleSupport;
        t_land_LF_ = t_takeoff_LF_ + settings_.TsingleSupport;
	}

	void WBC::generateFullCycle(ModelMaker &mm){
		
		std::vector<Support> cycle;
		int takeoff_RF, land_RF, takeoff_LF, land_LF;
		takeoff_RF = 0;
		land_RF = takeoff_RF + settings_.TsingleSupport;
		takeoff_LF = takeoff_RF + settings_.Tstep;
		land_LF = takeoff_LF + settings_.TsingleSupport;

		for (int i=0; i<2*settings_.Tstep; i++){

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

	bool WBC::timeToSolveDDP(const int &iteration){
		return !(iteration % settings_.Nc);
	}

	Eigen::VectorXd WBC::iterate(const int &iteration, 
								 const Eigen::VectorXd &q_current,
								 const Eigen::VectorXd &v_current,
								 const bool &is_feasible){

		x0_ = shapeState(q_current, v_current);
		if (timeToSolveDDP(iteration)){
			// ~~TIMING~~ //
			updateStepCycleTiming();
			recedeWithFullCycle();

			// ~~REFERENCES~~ //
			designer_.updateReducedModel(x0_);
			//setDesiredFeetPose(iteration, settings_.T - 1);

			// ~~SOLVER~~ //
			horizon_.solve(x0_, settings_.ddpIteration, is_feasible);
		}
		return horizon_.currentTorques(x0_);
	}

	// void WBC::setDesiredFeetPoses(unsigned long iteration, unsigned long time){

	// }

	void WBC::recedeWithFullCycle() {
		horizon_.recede(fullCycle_.iam(0), fullCycle_.data(0));
		fullCycle_.recede();
	}

    Eigen::VectorXd WBC::shapeState(Eigen::VectorXd q, Eigen::VectorXd v){ 
		if (q.size() != designer_.get_rModelComplete().nq || v.size() != designer_.get_rModelComplete().nv){
			throw std::runtime_error("The full posture must be provided to shape the state.");
		}
        x_internal_.head<7>() = q.head<7>();
        x_internal_.segment<6>(designer_.get_rModel().nq) = v.head<6>();
		
		int i = 0;
		for (unsigned long jointID : controlled_joints_id_)
			if(jointID > 1){
				x_internal_(i + 7) = q(jointID + 5);
				x_internal_(designer_.get_rModel().nq + i + 6) = v(jointID + 4);
				i++;}
		
        return x_internal_;
    }
}