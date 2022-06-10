#include "sobec/ocp.hpp"

namespace sobec {

	OCP::OCP(){}

	OCP::OCP(const OCPSettings &settings,
             const ModelMakerSettings &model_settings,
             const RobotDesignerSettings &design,
             const Eigen::VectorXd &q0,
             const Eigen::VectorXd &v0){ 
		initialize(settings,model_settings, design, q0, v0);
	}

	void OCP::initialize(const OCPSettings &settings,
					     const ModelMakerSettings &model_settings,
					     const RobotDesigner &design,
					     const Eigen::VectorXd &q0,
					     const Eigen::VectorXd &v0){ 
							 
		OCP_settings_ = settings;
		designer_ = sobec::RobotDesigner(design); 
		modelMaker_ = sobec::ModelMaker(model_settings, designer_);
		
		std::vector<Support> supports(OCP_settings_.T, Support::DOUBLE);
        std::vector<AMA> runningModels = modelMaker_.formulateHorizon(supports);
        AMA terminalModel = modelMaker_.formulateStepTracker(Support::DOUBLE);
		
		x0_.resize(designer_.get_rModel().nq + designer_.get_rModel().nv);
		x0_ << q0, v0;
		
		sobec::HorizonManagerSettings horizonSettings = {designer_.get_LF_name(),designer_.get_RF_name()};
		horizon_ = sobec::HorizonManager(horizonSettings, x0_, runningModels, terminalModel);
		
		std::vector<Eigen::VectorXd> x_init;
        std::vector<Eigen::VectorXd> u_init;
        Eigen::VectorXd zero_u = Eigen::VectorXd::Zero(designer_.get_rModel().nv - 6);
        
        for(std::size_t i = 0; i < OCP_settings_.T ;i++){
			x_init.push_back(x0_);
			u_init.push_back(zero_u);
		}
		x_init.push_back(x0_);
		
		horizon_.solve(x_init,u_init,500);
		
		designer_.updateReducedModel(q0);
		
		// Initialize first foot trajectory
		starting_position_right_ = designer_.get_rData().oMf[designer_.get_RF_id()];
		final_position_right_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_RF_id()]);
		final_position_right_.translation()[0] += OCP_settings_.stepSize;
		final_position_right_.translation()[1] -= OCP_settings_.stepYCorrection;
		final_position_right_.translation()[2] -= OCP_settings_.stepDepth;
		transBezierRight_ = defineBezier(OCP_settings_.stepHeight,0,1,starting_position_right_,final_position_right_);
		
		starting_position_left_ = designer_.get_rData().oMf[designer_.get_LF_id()];
		final_position_left_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_LF_id()]);
		final_position_left_.translation()[0] += OCP_settings_.stepSize;
		final_position_left_.translation()[1] += OCP_settings_.stepYCorrection;
		final_position_left_.translation()[2] -= OCP_settings_.stepDepth;
		transBezierLeft_ = defineBezier(OCP_settings_.stepHeight,0,1,starting_position_left_,final_position_left_);
		
		frame_placement_next_ = starting_position_right_;
		
		// Initialize varying parameters
		TswitchPhase_ =  OCP_settings_.Tstep;
		TswitchTraj_ =  OCP_settings_.Tstep + OCP_settings_.T;
		swingRightPhase_ = true;
		swingRightTraj_ = false;
		steps_ = 0;

		wrench_reference_double_ << 0,0,500,0,0,0;
		wrench_reference_simple_ << 0,0,1000,0,0,0;
		
		// Initialize the whole sequence of contacts
		std::vector<unsigned long> simple_contacts;
		for(std::size_t i = 0; i < OCP_settings_.TsimpleSupport ;i++){
			simple_contacts.push_back(i + 1);
		}
        
		std::vector<unsigned long> double_contacts(OCP_settings_.TdoubleSupport, 0);

        for(std::size_t i = 0; i < OCP_settings_.totalSteps ;i++){
			contacts_sequence_.insert(contacts_sequence_.end(),double_contacts.begin(),double_contacts.end());
			contacts_sequence_.insert(contacts_sequence_.end(),simple_contacts.begin(),simple_contacts.end());
		}
		std::vector<unsigned long> end_contacts(OCP_settings_.T, 0);
		contacts_sequence_.insert(contacts_sequence_.end(),end_contacts.begin(),end_contacts.end());
	}
	
	ndcurves::piecewise_SE3_t OCP::defineBezier(const double &height, const double &time_init,const double &time_final, 
                                           const pinocchio::SE3 &placement_init,const pinocchio::SE3 &placement_final){
		ndcurves::point3_t a1(placement_init.translation()[0],
		                      placement_init.translation()[1],
		                      placement_init.translation()[2]);
		Eigen::Vector3d placement_mid = (placement_init.translation() + placement_final.translation()) / 2.;
		ndcurves::point3_t a2(placement_init.translation()[0],
		                      placement_init.translation()[1],
		                      placement_init.translation()[2] + height);
		ndcurves::point3_t a3(placement_mid[0],
		                      placement_mid[1],
		                      placement_mid[2] + height);
		ndcurves::point3_t a4(placement_final.translation()[0],
		                      placement_final.translation()[1],
		                      placement_final.translation()[2]);
		std::vector<ndcurves::point3_t> params;
		params.push_back(a1);
		params.push_back(a1);
		params.push_back(a2);
		params.push_back(a2);
		params.push_back(a3);
		params.push_back(a3);
		for (size_t i = 0; i < 3; i++)
		{
			params.push_back(a4);
		}
		boost::shared_ptr<ndcurves::bezier_t> translation_bezier(
            new ndcurves::bezier_t(params.begin(), params.end(),time_init,time_final));
        ndcurves::curve_SE3_ptr_t se3curve(new ndcurves::SE3Curve_t(translation_bezier,placement_init.rotation(),placement_final.rotation()));
		ndcurves::piecewise_SE3_t piecewise(se3curve);
		return piecewise;
	}	
	
	void OCP::updateEndPhase(){
		steps_ += 1;
		// If this is the end of a phase, update phase
		if (TswitchPhase_ == 0){
			TswitchPhase_ = OCP_settings_.Tstep;
			swingRightPhase_ = not(swingRightPhase_);
			std::cout << "TswitchPhase = 0"  << std::endl;
		}
		// If this is the end of a step, update next foot trajectory
		if (TswitchTraj_ == 0){
			steps_ += 1;
			if (swingRightTraj_){
				starting_position_left_ = designer_.get_rData().oMf[designer_.get_LF_id()];
				final_position_left_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_LF_id()]);
				final_position_left_.translation()[0] += OCP_settings_.stepSize;
				final_position_left_.translation()[1] += OCP_settings_.stepYCorrection;
				final_position_left_.translation()[2] -= OCP_settings_.stepDepth;
				transBezierLeft_ = defineBezier(OCP_settings_.stepHeight,0,1,starting_position_left_,final_position_left_);
				std::cout << "TswitchTraj = 0 update left traj"  << std::endl;
				frame_placement_next_ = starting_position_left_;
			}
			else{
				starting_position_left_ = designer_.get_rData().oMf[designer_.get_LF_id()];
				final_position_left_ = pinocchio::SE3(designer_.get_rData().oMf[designer_.get_LF_id()]);
				final_position_left_.translation()[0] += OCP_settings_.stepSize;
				final_position_left_.translation()[1] += OCP_settings_.stepYCorrection;
				final_position_left_.translation()[2] -= OCP_settings_.stepDepth;
				transBezierLeft_ = defineBezier(OCP_settings_.stepHeight,0,1,starting_position_left_,final_position_left_);
				std::cout << "TswitchTraj = 0 update right traj"  << std::endl;
				frame_placement_next_ = starting_position_right_;
			}
					
			TswitchTraj_ = OCP_settings_.Tstep;
			swingRightTraj_ = not(swingRightTraj_);
		}
	}
	
	void OCP::updateOCP(const Eigen::VectorXd &measured_x){
		designer_.updateReducedModel(measured_x.head(designer_.get_rModel().nq));
		
		if (!contacts_sequence_.empty()){
			TswitchTraj_ --;
			TswitchPhase_ --;
			// Take first action model
			//std::cout << "Contact sequence " << contacts_sequence_[0] << " at Tswitch " << Tswitch_ << " and iteration " << iteration_ << std::endl;
			// If contacts_sequence[0] > 0 , this is a swing phase
			if (contacts_sequence_[0] > 0){
				// Get desired foot reference for the end of the horizon
				if (swingRightPhase_){
					point_now_ = transBezierRight_(float(contacts_sequence_[0]) / float(OCP_settings_.TsimpleSupport)).translation();
					frame_placement_next_.translation()[0] = point_now_[0];
					frame_placement_next_.translation()[1] = point_now_[1];
					frame_placement_next_.translation()[2] = point_now_[2];
					horizon_.setSwingingRF(0,frame_placement_next_,wrench_reference_simple_);
				}
				else{
					point_now_ = transBezierLeft_(float(contacts_sequence_[0]) / float(OCP_settings_.TsimpleSupport)).translation();
					frame_placement_next_.translation()[0] = point_now_[0];
					frame_placement_next_.translation()[1] = point_now_[1];
					frame_placement_next_.translation()[2] = point_now_[2];
					horizon_.setSwingingLF(0,frame_placement_next_,wrench_reference_simple_);
				}
			}
			// else, this is a double support phase
			else{
				horizon_.setSupportingFeet(0,wrench_reference_double_);
			}
			updateEndPhase();
			// Put first model in last position
			horizon_.recede();
			
			// Update contact sequence
			contacts_sequence_.erase(contacts_sequence_.begin());
			// Solve ddp
			horizon_.solve(measured_x, OCP_settings_.ddpIteration);
			
		}
		
	}
}
