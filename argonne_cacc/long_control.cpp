#include "long_control.hpp"

long_control::long_control(setup_struct* ss, int _veh_id, int _control_mode, double i_long_speed, int _preceding_vehicle_id, bool _is_with_hmi, bool _is_cacc_perf,
								bool _v2i_engaged, bool _manual_control_requested, int _my_pip, int _preferred_topology){
	// Constructor
	this->cfg = (setup_struct*)  malloc(sizeof(setup_struct));
	this->cfg = ss; print_cfg_parameters();
	this->vehicle_id = _veh_id;
	this->Ts = this->cfg->SAMPLING_TIME;
	this->desired_control_mode = _control_mode;
	this->desired_cruise_speed = i_long_speed;
	this->expected_preceding_vehicle_id = _preceding_vehicle_id;
	this->is_with_hmi = _is_with_hmi;
	this->is_cacc_performance_flag = _is_cacc_perf;
	this->manual_control_requested = _manual_control_requested;
	this->my_pip = _my_pip;
	this->preferred_topology = _preferred_topology;
	this->preceding_veh_id = 0;

	this->speed_tracking_P_gain = this->cfg->SPEED_TRACKING_P_GAIN;
	this->ACC_gap_control_Kp_comf = this->cfg->ACC_KP_COMFORT;
	this->ACC_gap_control_Kd_comf = this->cfg->ACC_KD_COMFORT;
	this->ACC_gap_control_Kp_perf = this->cfg->ACC_KD_PERFORMANCE;
	this->ACC_gap_control_Kd_perf = this->cfg->ACC_KD_PERFORMANCE;

	this->CACC_gap_control_Kp_comf = this->cfg->HIGHWAY_CACC_KP_COMFORT;
	this->CACC_gap_control_Kd_comf = this->cfg->HIGHWAY_CACC_KD_COMFORT;
	this->CACC_gap_control_Kp_perf = this->cfg->HIGHWAY_CACC_KP_PERFORMANCE;
	this->CACC_gap_control_Kd_perf = this->cfg->HIGHWAY_CACC_KD_PERFORMANCE;

	if(_is_cacc_perf){
		this->CACC_gap_control_Kp_comf = this->cfg->TRACK_CACC_KP_COMFORT;
		this->CACC_gap_control_Kd_comf = this->cfg->TRACK_CACC_KD_COMFORT;
		this->CACC_gap_control_Kp_perf = this->cfg->TRACK_CACC_KP_PERFORMANCE;
		this->CACC_gap_control_Kd_perf = this->cfg->TRACK_CACC_KD_PERFORMANCE;
	}

	this->beta_relative_speed_gain = this->cfg->RELATIVE_SPEED_GAIN;

	this->LPF_ego_preceding_gain = this->cfg->LPF_EGO_PRECEDING_FF_GAIN;
	this->LPF_ego_leader_gain = this->cfg->LPF_EGO_LEADER_FF_GAIN;

	this->current_control_mode = CRUISE;
	this->first_time = true;
	this->long_acceleration = 0.;
	this->deceleration_command = 0.;
	this->desired_acceleration = 0.;
	this->throttle_level_command = 0.;
	this->brake_level_command = 0.;
	this->setpoint_speed = 0.;
	this->speed_error_integral = 0;
	this->ACC_gap_error = 0.;
	this->ACC_gap_error_deriv = 0.;
	this->last_ACC_gap_error =0;

	this->standstill_distance = cfg->STANDSTILL_DISTANCE;
	this->ACC_time_gap = cfg->ACC_TARGET_TIME_GAP;
	this->cooperative_ACC_time_gap = cfg->CACC_TARGET_TIME_GAP;
	this->ACC_ref_speed = 0;
	this->cooperative_ACC_ref_speed = 0;
	this->ACC_fb_out = 0;
	this->cooperative_ACC_fb_out = 0;
	this->ACC_gap_error = 0;
	this->last_CACC_gap_error = 0;
	this->CACC_gap_error = 0;
	this->CACC_gap_error_deriv = 0;
	this->CACC_ff_out = 0;
	this->remaining_transition_time = 0;
	this->previous_state_last_speed = 0;
	this->preceding_ref_speed = 0.;
	this->virtual_leader_info.relative_speed = 0;
	this->virtual_leader_info.leader_x_coordinate = 0.;
	this->leader_ref_speed = 0;
	this->desired_distance_gap = 0;
	this->performance_factor_from_user = cfg->DEFAULT_PERFORMANCE_FACTOR;

	this->user_time_gap_change_flag = false;
	this->user_control_mode_change_flag = false;
	this->user_requested_control_mode = this->desired_control_mode;
	this->monitoring_counter = 0;
	this->v2v_fault_timer = 10;

	this->ACC_fb_out_lp_filter = new signal_filter(this->Ts, 1, cfg->ACC_ERROR_FILTER_FREQ/this->ACC_time_gap, 0);
	this->ego_speed_filter = new signal_filter(this->Ts, 1, 1.5*2*PI, 0);

	this->CACC_fb_out_lp_filter = new signal_filter(this->Ts, 1, cfg->CACC_ERROR_FILTER_FREQ/this->cooperative_ACC_time_gap, 0);

	this->ACC_derror_filter = new signal_filter(this->Ts, 1, cfg->ACC_ERROR_FILTER_FREQ/this->ACC_time_gap, 0);
	this->CACC_derror_filter = new signal_filter(this->Ts, 1, cfg->CACC_ERROR_FILTER_FREQ/this->cooperative_ACC_time_gap, 0);

	this->is_preceding_valid = false;
	this->preceding_vehicle.ID = -1;
	this->preceding_vehicle.relative_distance = 200;
	this->preceding_vehicle.relative_position = 0;
	this->preceding_vehicle.relative_speed = -10;

	this->full_cut_event_flag = false;

	double ego_wn, ego_df, prec_wn, prec_df;
	switch(this->vehicle_id){
		case(PRIUS):
			ego_wn = cfg->PRIUS_BANDWIDTH;
			ego_df = cfg->PRIUS_DAMPING_FACTOR;
		break;
		case(LEAF):
			ego_wn = cfg->LEAF_BANDWIDTH;
			ego_df = cfg->LEAF_DAMPING_FACTOR;
		break;
		case(CAMRY):
			ego_wn = cfg->CAMRY_BANDWIDTH;
			ego_df = cfg->CAMRY_DAMPING_FACTOR;
		break;

	}
	switch(this->expected_preceding_vehicle_id){
		case(PRIUS):
			prec_wn = cfg->PRIUS_BANDWIDTH;
			prec_df = cfg->PRIUS_DAMPING_FACTOR;
		break;
		case(LEAF):
			prec_wn = cfg->LEAF_BANDWIDTH;
			prec_df = cfg->LEAF_DAMPING_FACTOR;
		break;
		case(CAMRY):
			prec_wn = cfg->CAMRY_BANDWIDTH;
			prec_df = cfg->CAMRY_DAMPING_FACTOR;
		break;

	}

	// LPF-FF variables
	this->ego_prec_FF_dynamics_filter = new signal_filter(this->Ts, ego_df, ego_wn, prec_df, prec_wn, 0);
	this->ego_prec_CACC_FF_1_H = new signal_filter(this->Ts, 1, cfg->CACC_FF_EGO_PREC_FILTER_FREQ/this->cooperative_ACC_time_gap, 0);
	this->ego_leader_FF_dynamics_filter = new signal_filter(this->Ts, ego_df, ego_wn, cfg->CAMRY_DAMPING_FACTOR, cfg->CAMRY_BANDWIDTH, 0);
	this->ego_leader_CACC_FF_1_H = new signal_filter(this->Ts, 1, cfg->CACC_FF_EGO_LEADER_FILTER_FREQ/this->cooperative_ACC_time_gap, 0);
	this->is_LPF_active= false;
	this->LPF_ego_preceding_gain = cfg->LPF_EGO_PRECEDING_FF_GAIN;
	this->LPF_ego_leader_gain = cfg->LPF_EGO_LEADER_FF_GAIN;

	// Spacing policy variables

	this->cuts_counter = 0;
	this->ACC_ctg_policy = new spacing_policy(CONSTANT_TIME_GAP, ACC, i_long_speed, this->ACC_time_gap, this->Ts, cfg);
	this->current_spacing_policy = new spacing_policy(CONSTANT_TIME_GAP, ACC, i_long_speed, this->ACC_time_gap, this->Ts, cfg);
	this->CACC_ctg_policy = new spacing_policy(CONSTANT_TIME_GAP, CACC, i_long_speed, this->cooperative_ACC_time_gap, this->Ts, cfg);
	this->fr_policy = new spacing_policy(FULL_RANGE_POLICY, CACC, i_long_speed, this->cooperative_ACC_time_gap, this->Ts, cfg);
	this->test_ACC_policy = new spacing_policy(FULL_RANGE_POLICY, ACC, i_long_speed, this->ACC_time_gap, this->Ts, cfg);


	this->current_spacing_policy = this->ACC_ctg_policy;
	// Com related variables
	//this->v2v_fault_timer = 0;
	this->v2x = new v2x_struct();

	// For virtual platooning
	this->v2i_fault_timer = 0;

	// For virtual platooning
	this->is_v2i_engaged = _v2i_engaged;
	this->is_virtual_car_following = _v2i_engaged && (this->my_pip == 1) && (this->desired_control_mode == ACC);
	if(this->is_virtual_car_following){
		this->ACC_gap_control_Kp_comf = this->cfg->KP_VIRTUAL_FOLLOWING_HIGH;
		this->ACC_gap_control_Kd_comf = this->cfg->KD_VIRTUAL_FOLLOWING_HIGH;
		this->ACC_gap_control_Kp_perf = this->cfg->KP_VIRTUAL_FOLLOWING_LOW;
		this->ACC_gap_control_Kd_perf = this->cfg->KD_VIRTUAL_FOLLOWING_LOW;
	}
	if(this->desired_control_mode == CACC){
		if(this->is_with_hmi)
			this->user_requested_control_mode = ACC;
		else
			this->user_requested_control_mode = CACC;

	}

	double init_coordinate = -this->cfg->STANDSTILL_DISTANCE * (this->my_pip-1) - 3;
	this->circuit_loc = new localization(this->Ts, 37.91530345847134, -122.3329937600211, 0, init_coordinate, cfg, this->is_v2i_engaged);

}

long_control::~long_control() {
	// Destructor

}

void long_control::Update_Cruise_inputs(double i_long_speed, double i_setpoint_speed, double current_time, double _long_acceleration) {
	this->long_speed = i_long_speed;
	this->long_acceleration = _long_acceleration;
	this->desired_cruise_speed = i_setpoint_speed;
	this->current_time = current_time;
	circuit_loc->update_states(fmax(0, this->ego_speed_filter->low_pass_order_1(i_long_speed)));
	return;
}

void long_control::Update_ACC_inputs(double i_long_speed, double i_long_acceleration, int nb_targets, double _current_time) {
	
	this->long_speed = i_long_speed;
	this->long_acceleration = i_long_acceleration;
	this->current_time = _current_time;
	switch(this->vehicle_id){
		case(LEAF):
			Get_preceding_vehicle_from_leaf_list(nb_targets, this->long_speed);
		break;
		case(PRIUS):
			Update_preceding_vehicle_from_prius_target(this->long_speed);
		break;
		case(CAMRY):
			 Get_preceding_vehicle_from_camry_list(nb_targets, this->long_speed);
		break;
	}

	this->current_spacing_policy = this->ACC_ctg_policy;

	return;
}

void long_control::Update_ACC_inputs_V2I_engaged(double i_long_speed, double _current_time) {

	this->long_speed = i_long_speed;
	this->current_time = _current_time;

	this->virtual_leader_info = this->v2x->v2i;
	this->v2v_fault_timer = Verify_v2v_communication(&(this->v2x->v2v[1]));
	Verify_v2i_communication(this->virtual_leader_info.sequence_nb);

	circuit_loc->update_states(fmax(0, this->ego_speed_filter->low_pass_order_1(i_long_speed)));
	circuit_loc->get_trajectory_planning_ref_accel(this->virtual_leader_info.dist_to_intersection - this->circuit_loc->get_x_coordinate(), this->virtual_leader_info.time_remaining, i_long_speed);
	this->trajectory_planning_requested = this->circuit_loc->trajectory_planning_engaged;

	if(this->virtual_leader_info.leader_x_coordinate < (this->circuit_loc->get_x_coordinate()))
		this->virtual_leader_info.leader_x_coordinate = 1195;

	if((fabs(this->virtual_leader_info.leader_x_coordinate - 758.6 ) < 0.5) && (fabs(this->virtual_leader_info.relative_speed + this->long_speed) < 1.1))
		this->virtual_leader_info.leader_x_coordinate = 1195;
	this->preceding_vehicle.relative_speed = this->virtual_leader_info.relative_speed;
	if((this->virtual_leader_info.leader_x_coordinate - this->circuit_loc->get_x_coordinate()) > 80) // )  // If virtual vehicle is too far away
		this->preceding_vehicle.relative_distance = 80;
	else{
		this->preceding_vehicle.relative_distance = this->virtual_leader_info.leader_x_coordinate  - this->circuit_loc->get_x_coordinate(); // Apply car-following only if within 50m horizon and not from red light
	}
	return;
}

void long_control::Update_CACC_inputs(double i_long_speed, double i_long_acceleration, int nb_targets, double _current_time) {

	this->long_speed = i_long_speed;
	this->long_acceleration = i_long_acceleration;
	this->current_time = _current_time;

	Verify_v2v_communication(&(v2x->v2v[0]));
	Verify_v2v_communication(&(v2x->v2v[1]));
	Verify_v2v_communication(&(v2x->v2v[2]));
	switch(this->my_pip){
		case(2):
			this->preceding_comm_data = this->v2x->v2v[0];
			this->leader_comm_data = this->v2x->v2v[0];
			break;
		case(3):
			this->preceding_comm_data = this->v2x->v2v[1];
			this->leader_comm_data = this->v2x->v2v[0];
			break;
	}
	this->preceding_ref_speed = (this->is_cacc_performance_flag) ? this->preceding_comm_data.ref_speed : this->preceding_comm_data.long_speed;
	this->leader_ref_speed =    (this->is_cacc_performance_flag) ? this->leader_comm_data.ref_speed    : this->leader_comm_data.long_speed;

	circuit_loc->update_states(fmax(0, this->ego_speed_filter->low_pass_order_1(i_long_speed)));

	switch(this->vehicle_id){
		case(LEAF):
			Get_preceding_vehicle_from_leaf_list(nb_targets, this->long_speed);
		break;
		case(PRIUS):
			this->long_speed = i_long_speed;
			Update_preceding_vehicle_from_prius_target(this->long_speed);
		break;
		case(CAMRY):
		 Get_preceding_vehicle_from_camry_list(nb_targets, this->long_speed);
		break;
	}

	this->v2v_fault_timer = this->preceding_comm_data.v2v_fault_timer;
	return;
}

double long_control::Verify_v2v_communication(v2v_comm_data *v2v){
	(v2v->sequence_nb ==  v2v->last_sequence_nb) ? v2v->v2v_fault_timer += this->Ts : v2v->v2v_fault_timer = 0;
	v2v->last_sequence_nb = v2v->sequence_nb;
	return v2v->v2v_fault_timer;
}

void long_control::Verify_v2i_communication(int _v2i_sequence_number){
	(this->v2i_sequence_number ==  _v2i_sequence_number) ? this->v2i_fault_timer += this->Ts : this->v2i_fault_timer = 0;
	this->v2i_sequence_number = _v2i_sequence_number;
	return;
}

void long_control::Update_Emergency_braking_inputs(double i_long_speed, int nb_targets, double _current_time, bool is_trigger) {

	this->long_speed = i_long_speed;
	this->current_time = _current_time;

	if(is_trigger){
		Init_braking_maneuver();
	}
	Verify_v2v_communication(&(v2x->v2v[0]));
	Verify_v2v_communication(&(v2x->v2v[1]));
	Verify_v2v_communication(&(v2x->v2v[2]));

	switch(this->my_pip){
		case(2):
			this->preceding_comm_data = this->v2x->v2v[0];
			this->leader_comm_data = this->v2x->v2v[0];
			break;
		case(3):
			this->preceding_comm_data = this->v2x->v2v[1];
			this->leader_comm_data = this->v2x->v2v[0];
			break;
	}

	switch(this->vehicle_id){
		case(LEAF):
			Get_preceding_vehicle_from_leaf_list(nb_targets, this->long_speed);
		break;
		case(PRIUS):
			this->long_speed = fmax(0.000001,this->ego_speed_filter->low_pass_order_1(i_long_speed/0.992));
			Update_preceding_vehicle_from_prius_target(this->long_speed);
		break;
		case(CAMRY):
			 Get_preceding_vehicle_from_camry_list(nb_targets, this->long_speed);
		break;
	}
	this->v2v_fault_timer = this->preceding_comm_data.v2v_fault_timer;
	return;
}

void long_control::Get_preceding_vehicle_from_leaf_list(int nb_targets, double ego_speed){
	double closest_distance = 120;
	if (targets[0].relative_distance > 0.01 && targets[0].relative_distance <= fmax(25,(this->long_speed * cfg->MAXIMUM_ACC_TIME_GAP_HORIZON + this->standstill_distance))){
		this->preceding_vehicle.relative_speed = this->targets[0].relative_speed;
		this->preceding_vehicle.relative_distance = this->targets[0].relative_distance;
		closest_distance = this->preceding_vehicle.relative_distance;
		this->missing_target_elapsed_time = 0.;
		this->is_preceding_valid = true;
	}
	if(closest_distance == 120){
		this->missing_target_elapsed_time += this->Ts;
		this->is_preceding_valid = false;
	}
	if(this->is_cacc_performance_flag || this->is_v2i_engaged)
		return;

	if(this->is_preceding_valid){
		Verify_possible_cut_in_out();
	}
	if(this->is_with_hmi && (this->current_spacing_policy->cut_detection_frames_counter == -1) && this->user_time_gap_change_flag){
		Change_target_time_gap_user_requested();
	}

}

void long_control::Update_preceding_vehicle_from_prius_target(double ego_speed){
	double closest_distance = 120;
	if (targets[0].relative_distance > 0.01 && targets[0].relative_distance <= fmax(25,(this->long_speed * cfg->MAXIMUM_ACC_TIME_GAP_HORIZON + this->standstill_distance))){
		this->preceding_vehicle.relative_speed = this->targets[0].relative_speed;
		this->preceding_vehicle.relative_distance = this->targets[0].relative_distance;
		closest_distance = this->preceding_vehicle.relative_distance;
		this->missing_target_elapsed_time = 0.;
		this->is_preceding_valid = true;
	}
	if(closest_distance == 120){
		this->missing_target_elapsed_time += this->Ts;
		this->is_preceding_valid = false;
	}
	if(this->is_cacc_performance_flag || this->is_v2i_engaged)
		return;

	if(this->is_preceding_valid){
		Verify_possible_cut_in_out();
	}
	if(this->is_with_hmi && (this->current_spacing_policy->cut_detection_frames_counter == -1) && this->user_time_gap_change_flag){
		Change_target_time_gap_user_requested();
	}
}

void long_control::Get_preceding_vehicle_from_camry_list(int nb_targets, double ego_speed){
	double closest_distance = 120;
	if (targets[0].relative_distance > 0.01 && targets[0].relative_distance <= fmax(25,(this->long_speed * cfg->MAXIMUM_ACC_TIME_GAP_HORIZON + this->standstill_distance))){
		this->preceding_vehicle.relative_speed = this->targets[0].relative_speed;
		this->preceding_vehicle.relative_distance = this->targets[0].relative_distance;
		closest_distance = this->preceding_vehicle.relative_distance;
		this->missing_target_elapsed_time = 0.;
		this->is_preceding_valid = true;
	}
	if(closest_distance == 120){
		this->missing_target_elapsed_time += this->Ts;
		this->is_preceding_valid = false;
	}
	if(this->is_cacc_performance_flag || this->is_v2i_engaged)
		return;

	if(this->is_preceding_valid){
		Verify_possible_cut_in_out();
	}
	if(this->is_with_hmi && (this->current_spacing_policy->cut_detection_frames_counter == -1) && this->user_time_gap_change_flag){
		Change_target_time_gap_user_requested();
	}

}

void long_control::Verify_possible_cut_in_out(){

	if(first_time){
		this->previous_preceding_vehicle = this->preceding_vehicle;
		this->first_time = false;
		return;
	}
	double threshold_gap_change = 0.1*this->long_speed + 2.5;
	double delta_distance =  this->preceding_vehicle.relative_distance - this->previous_preceding_vehicle.relative_distance;
	this->full_cut_event_flag = false;
	if(fabs(delta_distance) > threshold_gap_change){
		if(!this->cut_detection_flag){
			this->cut_detection_flag = true;
			this->cut_detection_timer = this->Ts;
		} else{
			this->cut_detection_timer += this->Ts;
			printf("\n cut detection timer=%.2f \n",this->cut_detection_timer);
		}
		if(this->cut_detection_timer >= cfg->CUT_DETECTION_TIMER){
			this->cut_detection_flag = false;
			this->full_cut_event_flag = true;
			this->previous_preceding_vehicle = this->preceding_vehicle;
			this->cut_detection_timer = 0;
			double speed_range_for_profile = 0.5 + (3.5)*this->performance_factor_from_user;
			double hinit = (this->preceding_vehicle.relative_distance - this->standstill_distance) / fmax(0.001, this->long_speed);
			double htarg = (this->is_with_hmi) ? fmax(0.2, this->user_requested_time_gap) :
					cfg->ACC_TARGET_TIME_GAP;
			if(hinit > htarg){
				this->cuts_counter++;
				if(this->long_speed < cfg->MAXIMUM_SPEED){
					this->ACC_fb_out_lp_filter->init_values(0);
					this->ACC_ctg_policy->InitGapClosing(hinit, htarg, cfg->CUT_OUT_A_MAX, cfg->CUT_OUT_J_MAX, this->long_speed, fmin(cfg->MAXIMUM_SPEED, this->long_speed+speed_range_for_profile));
					this->last_ACC_gap_error = Estimate_gap_error(this->ACC_ctg_policy->get_current_time_gap(), this->preceding_vehicle.relative_distance, this->long_speed, this->standstill_distance);
				}
			} else {
				this->cuts_counter--;
				if(this->long_speed > 0){
					this->ACC_fb_out_lp_filter->init_values(0);
					this->ACC_ctg_policy->InitGapOpening(fmax(hinit, 0.2), htarg, cfg->CUT_IN_A_MAX, cfg->CUT_IN_J_MAX, this->long_speed, fmax(0, this->long_speed-speed_range_for_profile));
					this->last_ACC_gap_error = Estimate_gap_error(this->ACC_ctg_policy->get_current_time_gap(), this->preceding_vehicle.relative_distance, this->long_speed, this->standstill_distance);
				}
			}
		}
	} else{
		this->cut_detection_flag = false;
		this->previous_preceding_vehicle = this->preceding_vehicle;
	}

	return;
}

void long_control::Change_target_time_gap_user_requested(){
	printf("\n Request to change target time gap to %.2f, for %s mode\n",
							this->user_requested_time_gap,
							(this->current_spacing_policy->control_mode == ACC) ? "ACC" : "CACC");
	double hinit = this->current_spacing_policy->htarg;
	this->current_spacing_policy->htarg = fmax((this->current_spacing_policy->control_mode == ACC) ?
						cfg->MINIMUM_ACC_TIME_GAP : cfg->MINIMUM_CACC_TIME_GAP,
						this->user_requested_time_gap);
	this->user_time_gap_change_flag = false;

	if(this->current_control_mode == CRUISE || this->current_control_mode == ACC_2_CRUISE)
		return;

	double speed_range_for_profile = 0.5 + (3.5)*this->performance_factor_from_user;

	if(hinit > this->current_spacing_policy->htarg && this->long_speed > 0.1){
		if(this->long_speed < cfg->MAXIMUM_SPEED){
			this->ACC_fb_out_lp_filter->init_values(0);
			this->CACC_fb_out_lp_filter->init_values(0);
			this->current_spacing_policy->InitGapClosing(hinit, this->current_spacing_policy->htarg, cfg->CUT_OUT_A_MAX,
				cfg->CUT_OUT_J_MAX, this->long_speed, fmin(cfg->MAXIMUM_SPEED, this->long_speed+speed_range_for_profile));
		}
	} else {
		if(this->long_speed > 0.1){
			this->ACC_fb_out_lp_filter->init_values(0);
			this->CACC_fb_out_lp_filter->init_values(0);
			this->current_spacing_policy->InitGapOpening(hinit, this->current_spacing_policy->htarg, cfg->CUT_IN_A_MAX,
				cfg->CUT_IN_J_MAX, this->long_speed, fmax(0, this->long_speed-speed_range_for_profile));
		}
	}
	return;
}

void long_control::Execute_control() {
	switch(this->desired_control_mode){
		case(CRUISE):
			this->setpoint_speed = this->desired_cruise_speed;
			break;
		case(ACC):
			if(this->is_virtual_car_following){
				Virtual_ACC_Control();
				break;
			}
			if(this->is_preceding_valid && !this->cut_detection_flag){
				ACC_Control();
			}
			this->setpoint_speed = fmin(cfg->MAXIMUM_SPEED, fmax(cfg->MINIMUM_SPEED, Reference_speed_decision()));
			break;
		case(CACC):
			if(this->is_preceding_valid || this->cut_detection_flag){
				this->is_cacc_performance_flag ? Performance_CACC_Control() : CACC_Control();
			}
			this->setpoint_speed = fmin(cfg->MAXIMUM_SPEED, fmax(cfg->MINIMUM_SPEED, Reference_speed_decision()));
			break;
		case(EMERGENCY_BRAKING):
			(my_pip == 1) ? Regulate_stopping_distance_leader() : Apply_Emergency_brake_control_followers(this->v2x->v2v[0].acceleration);
			if(!manual_control_requested)
				Actuators_signal_from_reference_acceleration(this->long_speed, this->desired_acceleration);
			print_monitoring_data(this->desired_control_mode, this->vehicle_id, this->current_control_mode);
			return;
		break;
	}

	this->desired_acceleration = Apply_speed_tracking_control(this->setpoint_speed, this->long_speed, this->long_acceleration);
	Acceleration_decision_for_trajectory_planning();
	if(!manual_control_requested)
		Actuators_signal_from_reference_acceleration(this->long_speed, this->desired_acceleration);

	print_monitoring_data(this->desired_control_mode, this->vehicle_id, this->current_control_mode);

	return;
}

void long_control::Virtual_ACC_Control(){
	double offset_distance = 9;

	this->ACC_fb_out = Apply_ACC_feedback(Estimate_gap_error(this->ACC_time_gap, this->preceding_vehicle.relative_distance,
											this->long_speed, this->standstill_distance+offset_distance));
	this->ACC_ref_speed = this->long_speed + fmin(cfg->TOP_ACCELERATION_V2I, fmax(cfg->TOP_DECELERATION, this->ACC_fb_out));

	if(!this->virtual_leader_info.sync_flag){
		this->setpoint_speed = 3.5;
		return;
	}
	this->setpoint_speed = fmin(this->circuit_loc->get_top_speed_from_curvature(), fmax(cfg->MINIMUM_SPEED, this->ACC_ref_speed));

	if(this->preceding_vehicle.relative_distance < 80 && this->preceding_vehicle.relative_distance > 50)
		this->setpoint_speed = fmin(this->circuit_loc->get_top_speed_from_curvature(),
							   fmax(cfg->MINIMUM_SPEED,
							   this->setpoint_speed + fmin(0,
							   this->beta_relative_speed_gain * this->preceding_vehicle.relative_speed)));
	return;
}

void long_control::ACC_Control(){
	this->ACC_time_gap = this->ACC_ctg_policy->get_current_time_gap();
	this->ACC_fb_out = Apply_ACC_feedback(Estimate_gap_error(this->ACC_time_gap, this->preceding_vehicle.relative_distance, this->long_speed, this->standstill_distance));
	this->ACC_fb_out *= 1 + (cfg->GAIN_REDUCTION_AT_MAX_TIME_GAP - 1)/(cfg->MAXIMUM_ACC_TIME_GAP_HORIZON - cfg->ACC_TARGET_TIME_GAP)
									* (this->ACC_time_gap - cfg->ACC_TARGET_TIME_GAP);
	this->ACC_ref_speed = this->long_speed + fmin(Get_limit_accel(),fmax(cfg->TOP_DECELERATION,this->ACC_fb_out));
	return;
}

void long_control::CACC_Control(){
	this->cooperative_ACC_time_gap = this->is_v2i_engaged ?
										this->cfg->CACC_TARGET_TIME_GAP :
										this->CACC_ctg_policy->get_current_time_gap();
	this->cooperative_ACC_fb_out = Apply_CACC_feedback(Estimate_gap_error(this->cooperative_ACC_time_gap, this->preceding_vehicle.relative_distance, this->long_speed, this->standstill_distance));
	this->cooperative_ACC_fb_out *= 1 + (cfg->GAIN_REDUCTION_AT_MAX_TIME_GAP - 1)/(cfg->MAXIMUM_CACC_TIME_GAP_HORIZON - cfg->MINIMUM_CACC_TIME_GAP) * (this->cooperative_ACC_time_gap - cfg->MINIMUM_CACC_TIME_GAP);

	this->preceding_ref_speed = this->ego_prec_CACC_FF_1_H->low_pass_order_1(this->preceding_ref_speed, cfg->CACC_FF_EGO_PREC_FILTER_FREQ/this->cooperative_ACC_time_gap);
	this->leader_ref_speed    = this->ego_leader_CACC_FF_1_H->low_pass_order_1(this->leader_ref_speed, cfg->CACC_FF_EGO_LEADER_FILTER_FREQ/this->cooperative_ACC_time_gap);

	if(this->my_pip == 3 && this->preferred_topology == LEADER_PREDECESSOR && this->leader_comm_data.v2v_fault_timer < cfg->MAX_V2V_MISSING_TIME_TOLERANCE){
		this->CACC_ff_out = this->LPF_ego_preceding_gain * this->preceding_ref_speed + this->LPF_ego_leader_gain * this->leader_ref_speed;
		this->is_LPF_active = true;
	} else {
		this->CACC_ff_out = this->preceding_ref_speed;
		this->is_LPF_active = false;
	}

	this->cooperative_ACC_ref_speed = this->CACC_ff_out + fmin(Get_limit_accel()*1.5 , fmax(cfg->TOP_DECELERATION*1.5, this->cooperative_ACC_fb_out));
	this->cooperative_ACC_ref_speed = fmin(cfg->MAXIMUM_SPEED, fmax(cfg->MINIMUM_SPEED, this->cooperative_ACC_ref_speed));

	this->ACC_time_gap = this->is_v2i_engaged ?
									this->cfg->ACC_TARGET_TIME_GAP :
									this->ACC_ctg_policy->get_current_time_gap();
	this->ACC_fb_out = Apply_ACC_feedback(Estimate_gap_error(this->ACC_time_gap, this->preceding_vehicle.relative_distance, this->long_speed, this->standstill_distance));
	this->ACC_fb_out *= 1 + (cfg->GAIN_REDUCTION_AT_MAX_TIME_GAP - 1)/(cfg->MAXIMUM_ACC_TIME_GAP_HORIZON - cfg->ACC_TARGET_TIME_GAP) * (this->ACC_time_gap - cfg->ACC_TARGET_TIME_GAP);
	this->ACC_ref_speed = this->long_speed + fmin(Get_limit_accel(),fmax(cfg->TOP_DECELERATION,this->ACC_fb_out));
	this->ACC_ref_speed = fmin(cfg->MAXIMUM_SPEED, fmax(cfg->MINIMUM_SPEED, this->ACC_ref_speed));

	return;
}

void long_control::Performance_CACC_Control(){
	this->cooperative_ACC_time_gap = this->fr_policy->get_current_time_gap(this->long_speed);
	double _error = Estimate_gap_error(this->preceding_vehicle.relative_distance, this->fr_policy->get_desired_distance(this->long_speed));
	this->cooperative_ACC_fb_out = Apply_Performance_CACC_feedback(_error);
	this->cooperative_ACC_fb_out *= 1 + (cfg->GAIN_REDUCTION_AT_MAX_TIME_GAP - 1)/(cfg->MAXIMUM_CACC_TIME_GAP_HORIZON - cfg->MINIMUM_CACC_TIME_GAP) * (this->cooperative_ACC_time_gap - cfg->MINIMUM_CACC_TIME_GAP);

	this->preceding_ref_speed = this->ego_prec_FF_dynamics_filter->FF_filter(this->preceding_ref_speed);
	this->leader_ref_speed    = this->ego_leader_FF_dynamics_filter->FF_filter(this->leader_ref_speed);

	this->preceding_ref_speed = this->ego_prec_CACC_FF_1_H->low_pass_order_1(this->preceding_ref_speed, cfg->CACC_FF_EGO_PREC_FILTER_FREQ/this->cooperative_ACC_time_gap);
	this->leader_ref_speed    = this->ego_leader_CACC_FF_1_H->low_pass_order_1(this->leader_ref_speed, cfg->CACC_FF_EGO_LEADER_FILTER_FREQ/this->cooperative_ACC_time_gap);

	if(this->my_pip == 3 && this->preferred_topology == LEADER_PREDECESSOR && this->leader_comm_data.v2v_fault_timer < cfg->MAX_V2V_MISSING_TIME_TOLERANCE){
		this->CACC_ff_out = this->LPF_ego_preceding_gain * this->preceding_ref_speed + this->LPF_ego_leader_gain * this->leader_ref_speed;
		this->is_LPF_active = true;
	} else {
		this->CACC_ff_out = this->preceding_ref_speed;
		this->is_LPF_active = false;
	}

	this->cooperative_ACC_ref_speed = this->CACC_ff_out + fmin(Get_limit_accel()*1.5 , fmax(cfg->TOP_DECELERATION*1.5, this->cooperative_ACC_fb_out));
	this->cooperative_ACC_ref_speed = fmin(cfg->MAXIMUM_SPEED, fmax(cfg->MINIMUM_SPEED, this->cooperative_ACC_ref_speed));

	this->ACC_time_gap = this->ACC_ctg_policy->get_current_time_gap();
	this->ACC_fb_out = Apply_ACC_feedback(Estimate_gap_error(this->ACC_time_gap, this->preceding_vehicle.relative_distance, this->long_speed, this->standstill_distance));
	this->ACC_fb_out *= 1 + (cfg->GAIN_REDUCTION_AT_MAX_TIME_GAP - 1)/(cfg->MAXIMUM_ACC_TIME_GAP_HORIZON - cfg->ACC_TARGET_TIME_GAP) * (this->ACC_time_gap - cfg->ACC_TARGET_TIME_GAP);
	this->ACC_ref_speed = this->long_speed + fmin(Get_limit_accel(),fmax(cfg->TOP_DECELERATION,this->ACC_fb_out));
	this->ACC_ref_speed = fmin(cfg->MAXIMUM_SPEED, fmax(cfg->MINIMUM_SPEED, this->ACC_ref_speed));

	return;
}

double long_control::Estimate_gap_error(double _time_gap, double _relative_distance, double _long_speed, double _standstill_distance) {
	this->desired_distance_gap = (_time_gap * _long_speed) + _standstill_distance;
	double error = _relative_distance - (_time_gap * _long_speed) - _standstill_distance;
	return error;
}

double long_control::Estimate_gap_error(double _relative_distance, double _ref_spacing) {
	this->desired_distance_gap = _ref_spacing;
	double error = _relative_distance - _ref_spacing;
	return error;
}

double long_control::Apply_ACC_feedback(double _gap_error) {
	double fb_out_comf=0, fb_out_perf = 0, performance_factor;

	this->ACC_gap_error = _gap_error;
	if(this->is_v2i_engaged){
		this->ACC_gap_error = this->ACC_fb_out_lp_filter->low_pass_order_1(_gap_error, cfg->ACC_ERROR_FILTER_FREQ/this->ACC_time_gap);
		this->ACC_gap_error_deriv = 0.1*(this->ACC_gap_error - this->last_ACC_gap_error) / this->Ts + 0.9*this->ACC_gap_error_deriv;
		fb_out_comf = this->ACC_gap_control_Kp_comf * this->ACC_gap_error + this->ACC_gap_control_Kd_comf * this->ACC_gap_error_deriv;
		fb_out_perf = this->ACC_gap_control_Kp_perf * this->ACC_gap_error + this->ACC_gap_control_Kd_perf * this->ACC_gap_error_deriv;
	}else{
		this->ACC_gap_error = this->ACC_fb_out_lp_filter->low_pass_order_1(_gap_error, cfg->ACC_ERROR_FILTER_FREQ/this->ACC_time_gap);
		this->ACC_gap_error_deriv = 0.1*(this->ACC_gap_error - this->last_ACC_gap_error) / this->Ts + 0.9*this->ACC_gap_error_deriv;
		fb_out_comf = this->ACC_gap_control_Kp_comf * ((1-cfg->FULL_GAP_ERROR_FILTER_FLAG)*_gap_error + cfg->FULL_GAP_ERROR_FILTER_FLAG*this->ACC_gap_error) +
				this->ACC_gap_control_Kd_comf * this->ACC_gap_error_deriv;
		fb_out_perf = this->ACC_gap_control_Kp_perf * ((1-cfg->FULL_GAP_ERROR_FILTER_FLAG)*_gap_error + cfg->FULL_GAP_ERROR_FILTER_FLAG*this->ACC_gap_error) +
				this->ACC_gap_control_Kd_perf * this->ACC_gap_error_deriv;
	}

	this->last_ACC_gap_error = this->ACC_gap_error;
	if(!this->is_with_hmi)
		performance_factor = (fmin(cfg->MAXIMUM_SPEED, fmax(this->long_speed, 0))) / (cfg->MAXIMUM_SPEED);
	else
		performance_factor = 1-this->performance_factor_from_user;

	this->ACC_fb_out = performance_factor * fb_out_comf + (1-performance_factor) * fb_out_perf;

	return this->ACC_fb_out;
}

double long_control::Apply_CACC_feedback(double _gap_error){
	double fb_out_comf=0, fb_out_perf = 0, performance_factor;

	this->CACC_gap_error = _gap_error;
	this->CACC_gap_error = this->CACC_fb_out_lp_filter->low_pass_order_1(_gap_error, cfg->CACC_ERROR_FILTER_FREQ/this->cooperative_ACC_time_gap);

	this->CACC_gap_error_deriv = 0.1*(this->CACC_gap_error - this->last_CACC_gap_error) / this->Ts + 0.9*this->CACC_gap_error_deriv;

	fb_out_comf = this->CACC_gap_control_Kp_comf * ((1-cfg->FULL_GAP_ERROR_FILTER_FLAG)*_gap_error + cfg->FULL_GAP_ERROR_FILTER_FLAG*this->CACC_gap_error) +
			this->CACC_gap_control_Kd_comf * this->CACC_gap_error_deriv;
	fb_out_perf = this->CACC_gap_control_Kp_perf * ((1-cfg->FULL_GAP_ERROR_FILTER_FLAG)*_gap_error + cfg->FULL_GAP_ERROR_FILTER_FLAG*this->CACC_gap_error) +
			this->CACC_gap_control_Kd_perf * this->CACC_gap_error_deriv;

	this->last_CACC_gap_error = this->CACC_gap_error;

	performance_factor = (fmin(1.0, fmax(this->performance_factor_from_user, 0.0)));
	this->cooperative_ACC_fb_out = performance_factor * fb_out_perf + (1-performance_factor) * fb_out_comf;

	return this->cooperative_ACC_fb_out;
}

double long_control::Apply_Performance_CACC_feedback(double _gap_error){
	double fb_out_comf=0, fb_out_perf = 0, performance_factor;

	this->CACC_gap_error = _gap_error;
	this->CACC_gap_error = this->CACC_fb_out_lp_filter->low_pass_order_1(_gap_error, cfg->PERF_CACC_ERROR_FILTER_FREQ/this->cooperative_ACC_time_gap);

	this->CACC_gap_error_deriv = 0.1*(this->CACC_gap_error - this->last_CACC_gap_error) / this->Ts + 0.9*this->CACC_gap_error_deriv;

	fb_out_comf = this->CACC_gap_control_Kp_comf * ((1-cfg->FULL_GAP_ERROR_FILTER_FLAG)*_gap_error + cfg->FULL_GAP_ERROR_FILTER_FLAG*this->CACC_gap_error) +
			this->CACC_gap_control_Kd_comf * this->CACC_gap_error_deriv;
	fb_out_perf = this->CACC_gap_control_Kp_perf * ((1-cfg->FULL_GAP_ERROR_FILTER_FLAG)*_gap_error + cfg->FULL_GAP_ERROR_FILTER_FLAG*this->CACC_gap_error) +
			this->CACC_gap_control_Kd_perf * this->CACC_gap_error_deriv;

	this->last_CACC_gap_error = this->CACC_gap_error;

	performance_factor = (fmin(1.0, fmax(this->performance_factor_from_user, 0.0)));
	this->cooperative_ACC_fb_out = performance_factor * fb_out_perf + (1-performance_factor) * fb_out_comf;

	return this->cooperative_ACC_fb_out;
}

double long_control::Compute_ACC_fb_out_prius(){
	double out=0;
	return out;
}

double long_control::Compute_CACC_fb_out_prius(){
	double out=0;
	return out;
}

void long_control::Acceleration_decision_for_trajectory_planning(){
	if(this->trajectory_planning_requested && this->is_virtual_car_following){
		this->trajectory_planning_requested = (this->desired_acceleration > this->circuit_loc->ref_accel_traj_plan);
		this->desired_acceleration = fmax(cfg->TOP_DECELERATION, fmin(this->desired_acceleration, this->circuit_loc->ref_accel_traj_plan));
	}
	return;
}

double long_control::Reference_speed_decision(){

	double ref_vel, elapsed_time;
	this->speed_reduction_zone_flag = false;
	this->current_spacing_policy = this->ACC_ctg_policy;
	switch(this->current_control_mode){
		case(CRUISE):
			this->desired_cruise_speed = fmin((this->is_with_hmi) ? this->user_requested_cruise_speed :
					cfg->DESIRED_CC_SPEED,
					this->desired_cruise_speed + this->Ts*Get_limit_accel()/3);
			ref_vel = this->desired_cruise_speed;
			if(this->is_preceding_valid && (this->preceding_vehicle.relative_distance <=
					(this->cfg->MAXIMUM_ACC_TIME_GAP_HORIZON*this->long_speed + this->standstill_distance)) ){
				ref_vel += this->beta_relative_speed_gain*this->preceding_vehicle.relative_speed;
				this->speed_reduction_zone_flag = true;
			}
			if(this->is_preceding_valid && (this->ACC_ref_speed <= this->desired_cruise_speed)){
				this->previous_state_last_speed = ref_vel;
				this->transition_time = this->current_time;
				this->current_control_mode = CRUISE_2_ACC;
				this->desired_cruise_speed = ref_vel;
				this->transition_period = fmax(1,fmin(5, fabs(this->desired_cruise_speed - this->ACC_ref_speed) / cfg->TRANSITION_DECEL_CRUISE_2_ACC));
			}
		break;
		case(ACC_2_CRUISE):
			elapsed_time = this->current_time - this->transition_time;
			ref_vel = this->desired_cruise_speed * (elapsed_time)/this->transition_period + this->previous_state_last_speed * (this->transition_period - elapsed_time)/this->transition_period;

			if(ref_vel == this->desired_cruise_speed || elapsed_time >= this->transition_period){
				this->current_control_mode = CRUISE;
				break;
			}
			if(this->is_preceding_valid && (this->preceding_vehicle.relative_distance <= (cfg->MAXIMUM_ACC_TIME_GAP_HORIZON*this->long_speed + this->standstill_distance)) ){
				ref_vel += this->beta_relative_speed_gain*this->preceding_vehicle.relative_speed;
				this->speed_reduction_zone_flag = true;
			}
			if(this->is_preceding_valid && (this->ACC_ref_speed <= ref_vel)){
				this->current_control_mode = ACC;
				this->transition_time = this->current_time;
				this->previous_state_last_speed = ref_vel;
			}
			this->remaining_transition_time = (this->transition_period-elapsed_time);
		break;
		case(CRUISE_2_ACC):
			elapsed_time = this->current_time - this->transition_time;
			ref_vel = this->ACC_ref_speed * (elapsed_time)/this->transition_period + this->previous_state_last_speed * (this->transition_period - elapsed_time)/this->transition_period;
			if(this->missing_target_elapsed_time < cfg->MAX_MISSING_TARGET_TIME && (elapsed_time >= this->transition_period)){
				this->transition_time = this->current_time;
				this->current_control_mode = ACC;
			}
			if(this->missing_target_elapsed_time > cfg->MAX_MISSING_TARGET_TIME )
				Switch2Cruise(ref_vel);

			this->remaining_transition_time = (this->transition_period-elapsed_time);
		break;
		case(ACC):
			ref_vel = this->ACC_ref_speed;
			if(this->missing_target_elapsed_time > cfg->MAX_MISSING_TARGET_TIME)
				Switch2Cruise(ref_vel);

			if(this->user_requested_control_mode == CACC && this->is_preceding_valid &&
					((this->current_time - this->transition_time) > 4) &&
					this->v2v_fault_timer < cfg->MAX_V2V_MISSING_TIME_TOLERANCE &&
					this->ACC_ctg_policy->cut_detection_frames_counter == -1){
				this->transition_time = this->current_time;
				this->current_control_mode = ACC_2_CACC;   // may have to be changed
				this->previous_state_last_speed = ref_vel;
				this->transition_period = fmin((fabs(this->ACC_ref_speed - this->cooperative_ACC_ref_speed)) / cfg->TRANSITION_ACCEL_ACC_2_CACC, 5);
			}
		break;
		case(ACC_2_CACC):
			this->current_spacing_policy = (this->is_cacc_performance_flag) ? this->fr_policy : this->CACC_ctg_policy;
			elapsed_time = this->current_time - this->transition_time;
			ref_vel = this->cooperative_ACC_ref_speed * (elapsed_time)/this->transition_period + this->ACC_ref_speed * (this->transition_period - elapsed_time)/this->transition_period;
			if(elapsed_time >= this->transition_period && this->is_preceding_valid &&
					this->v2v_fault_timer <= cfg->MAX_V2V_MISSING_TIME_TOLERANCE){
				this->current_control_mode = CACC;
			} else if(this->v2v_fault_timer > cfg->MAX_V2V_MISSING_TIME_TOLERANCE || this->user_requested_control_mode == ACC){
				this->current_control_mode = CACC_2_ACC;
				this->transition_time = this->current_time;
				this->previous_state_last_speed = ref_vel;
				this->transition_period = fmin((fabs(this->ACC_ref_speed - this->cooperative_ACC_ref_speed))/ cfg->TRANSITION_ACCEL_CACC_2_ACC, 5);
			}
			if(this->missing_target_elapsed_time > cfg->MAX_MISSING_TARGET_TIME)
				Switch2Cruise(ref_vel);

			this->remaining_transition_time = (this->transition_period-elapsed_time);
			break;
		case(CACC_2_ACC):
			elapsed_time = this->current_time - this->transition_time;
			ref_vel = this->ACC_ref_speed * (elapsed_time)/this->transition_period + this->previous_state_last_speed * (this->transition_period - elapsed_time)/this->transition_period;
			if(elapsed_time >= this->transition_period && this->is_preceding_valid){
				this->current_control_mode = ACC;
			}
			if(this->missing_target_elapsed_time > cfg->MAX_MISSING_TARGET_TIME)
				Switch2Cruise(ref_vel);

			this->remaining_transition_time = (this->transition_period-elapsed_time);
		break;
		case(CACC):
			this->current_spacing_policy = (this->is_cacc_performance_flag) ? this->fr_policy : this->CACC_ctg_policy;
			ref_vel = this->cooperative_ACC_ref_speed;
			if(this->v2v_fault_timer > cfg->MAX_V2V_MISSING_TIME_TOLERANCE || this->user_requested_control_mode == ACC){
				this->current_control_mode = CACC_2_ACC;
				if(!this->is_v2i_engaged && this->is_with_hmi)
					this->user_requested_control_mode = ACC;
				this->transition_time = this->current_time;
				this->previous_state_last_speed = ref_vel;
				this->transition_period = fmin((fabs(this->ACC_ref_speed - this->previous_state_last_speed)) / cfg->TRANSITION_ACCEL_CACC_2_ACC, 4);
			}
			if(this->missing_target_elapsed_time > cfg->MAX_MISSING_TARGET_TIME)
				Switch2Cruise(ref_vel);

			if(this->full_cut_event_flag && !this->is_v2i_engaged && !this->is_cacc_performance_flag){ // If there is a cut in/out in a platoon, degrade to ACC
				this->current_control_mode = ACC;
				if(this->is_with_hmi)
					this->user_requested_control_mode = ACC;
				this->previous_state_last_speed = ref_vel;
				this->transition_time = this->current_time;
				this->transition_period = (fabs(this->ACC_ref_speed - this->previous_state_last_speed)/ cfg->TRANSITION_ACCEL_CACC_2_ACC);
			}
		break;
	}
	this->desired_distance_gap = this->current_spacing_policy->get_desired_distance(this->long_speed);
	return ref_vel;
}

void long_control::Switch2Cruise(double last_velocity){
	this->current_control_mode = ACC_2_CRUISE;
	if(!this->is_v2i_engaged && this->is_with_hmi)
		this->user_requested_control_mode = ACC;
	this->transition_time = this->current_time;
	this->desired_cruise_speed = (this->is_with_hmi) ? this->user_requested_cruise_speed : cfg->DESIRED_CC_SPEED;
	this->ACC_ctg_policy->cut_detection_frames_counter = -1;
	this->CACC_ctg_policy->cut_detection_frames_counter = -1;
	this->previous_state_last_speed = last_velocity;
	this->transition_period = fabs(this->desired_cruise_speed - this->previous_state_last_speed) / cfg->TRANSITION_ACCEL_ACC_2_CRUISE;
	return;
}

void long_control::Init_braking_maneuver(){
	if(this->my_pip == 1){
		this->circuit_loc = new localization(this->Ts, 37.91530345847134, -122.3329937600211, this->long_speed, 0, cfg, false);
		this->nominal_deceleration_EB = (this->long_speed*this->long_speed) / (2*(cfg->E_BRAKING_STOPPING_DISTANCE-5));
		this->init_speed_EB = this->long_speed;
		return;
	}
	this->ACC_fb_out_lp_filter->init_values(0);
	this->last_ACC_gap_error = Estimate_gap_error(this->ACC_ctg_policy->htarg, this->preceding_vehicle.relative_distance, this->long_speed, cfg->STANDSTILL_DISTANCE);
	this->ACC_gap_error_deriv = 0;
	this->init_speed_EB = this->long_speed;

	return;
}

void long_control::Regulate_stopping_distance_leader(){
	this->circuit_loc->update_states(this->long_speed);
	this->remaining_distance_EB = fmax(0.00, this->cfg->E_BRAKING_STOPPING_DISTANCE - this->circuit_loc->get_x_coordinate());
	double factor=this->init_speed_EB*this->init_speed_EB;
	factor -= 2*this->nominal_deceleration_EB*(this->circuit_loc->get_x_coordinate()+3);
	this->setpoint_speed = sqrt(fmax(0.000, factor));
	this->setpoint_speed += cfg->LEADER_EMERGENCY_BRAKING_GAIN*(this->setpoint_speed - this->long_speed);
	this->desired_acceleration = //(this->remaining_distance_EB < 0.01) ? -1 :
			Apply_speed_tracking_control(this->setpoint_speed, this->long_speed, this->long_acceleration);
	return;
}

void long_control::Apply_Emergency_brake_control_followers(double leader_ref_acc){
	double error = Estimate_gap_error(this->ACC_ctg_policy->htarg, this->preceding_vehicle.relative_distance, this->long_speed, cfg->STANDSTILL_DISTANCE);

	//this->ACC_gap_error = this->ACC_fb_out_lp_filter->low_pass_order_1(error, cfg->ACC_ERROR_FILTER_FREQ/this->ACC_time_gap);
	this->ACC_fb_out = this->ACC_gap_control_Kp_perf*error; //this->ACC_gap_control_Kp_perf* (this->cfg->FULL_GAP_ERROR_FILTER_FLAG)*this->ACC_gap_error +
								//(1-this->cfg->FULL_GAP_ERROR_FILTER_FLAG)*error;

	//this->ACC_gap_error_deriv = 0.1*(this->ACC_gap_error - this->last_ACC_gap_error) / this->Ts + 0.9*this->ACC_gap_error_deriv;
	//this->ACC_fb_out += this->ACC_gap_control_Kd_perf * this->ACC_gap_error_deriv;

	this->desired_acceleration = fmin(0.0, fmax(this->cfg->TOP_DECELERATION, leader_ref_acc + this->ACC_fb_out));
	return;
}

double long_control::Apply_speed_tracking_control(double _setpoint_speed, double _long_speed, double _long_acceleration) {
	double speedError = (_setpoint_speed - _long_speed);
	//double gain = (this->is_cacc_performance_flag) ? this->speed_tracking_P_gain_perf : this->speed_tracking_P_gain;
	double accControlOut = speedError * this->speed_tracking_P_gain;
	accControlOut = fmin(Get_limit_accel(), fmax(cfg->TOP_DECELERATION, accControlOut));

	return accControlOut;
}

void long_control::Actuators_signal_from_reference_acceleration(double _veh_speed, double _desired_acceleration) {

	if (_desired_acceleration >= -0.5) {
		this->throttle_level_command = Throttle_map_search(_veh_speed, _desired_acceleration);
		this->deceleration_command = (this->vehicle_id==ACCORD) ? 0.00000000 : fmin(0.0000000, _desired_acceleration);
		this->brake_level_command = 0.00000000;
	}
	if (_desired_acceleration < -0.5 || this->throttle_level_command < 0.00001) {
		this->throttle_level_command = 0.0000000;
		this->deceleration_command = _desired_acceleration;
//		this->brake_level_command = fmax(0, -_desired_acceleration / 0.1534); // Brake map first order fitting, only for Accord
		this->brake_level_command = (this->vehicle_id==LEAF) ? fmax(0, desired_acceleration) : fmax(0, -_desired_acceleration / 0.1534); // Brake map first order fitting, only for Accord
	}
	return;
}

double long_control::Throttle_map_search(double _veh_speed, double _desired_acceleration) {
	double speedIndex = fmax(0, fmin(129, _veh_speed * 3.6 - 1));
	double accelerationIndex = fmax(0, fmin(3.5, (_desired_acceleration + 0.5) / 3.5)) * 70;
	double out=0.;
	int speed_floor = floor(speedIndex), speed_ceil = ceil(speedIndex), accel_floor = floor(accelerationIndex), accel_ceil = ceil(accelerationIndex);
	switch (vehicle_id) {
		case(PRIUS):
			if (speed_floor == speed_ceil && accel_floor == accel_ceil)
				out = prius_throttle_map[speed_ceil][accel_floor];
			else if (speed_floor == speed_ceil && accel_floor != accel_ceil)
				out = prius_throttle_map[speed_ceil][accel_ceil] * (accelerationIndex - accel_floor) + prius_throttle_map[speed_ceil][accel_floor] * (accel_ceil - accelerationIndex);
			else if (speed_floor != speed_ceil && accel_floor == accel_ceil)
				out = prius_throttle_map[speed_ceil][accel_ceil] * (speedIndex - speed_floor) + prius_throttle_map[speed_floor][accel_ceil] * (speed_ceil - speedIndex);
			else
				out = (prius_throttle_map[speed_ceil][accel_ceil] * (speedIndex - speed_floor) + prius_throttle_map[speed_floor][accel_ceil] * (speed_ceil - speedIndex))* (accelerationIndex - accel_floor) +
				(prius_throttle_map[speed_ceil][accel_floor] * (speedIndex - speed_floor) + prius_throttle_map[speed_floor][accel_floor] * (speed_ceil - speedIndex)) * (accel_ceil - accelerationIndex);
			break;
		case(LEAF):
			if (speed_floor == speed_ceil && accel_floor == accel_ceil)
				out = leaf_throttle_map[speed_ceil][accel_floor];
			else if (speed_floor == speed_ceil && accel_floor != accel_ceil)
				out = leaf_throttle_map[speed_ceil][accel_ceil] * (accelerationIndex - accel_floor) + leaf_throttle_map[speed_ceil][accel_floor] * (accel_ceil - accelerationIndex);
			else if (speed_floor != speed_ceil && accel_floor == accel_ceil)
				out = leaf_throttle_map[speed_ceil][accel_ceil] * (speedIndex - speed_floor) + leaf_throttle_map[speed_floor][accel_ceil] * (speed_ceil - speedIndex);
			else
				out = (leaf_throttle_map[speed_ceil][accel_ceil] * (speedIndex - speed_floor) + leaf_throttle_map[speed_floor][accel_ceil] * (speed_ceil - speedIndex)) * (accelerationIndex - accel_floor) +
				(leaf_throttle_map[speed_ceil][accel_floor] * (speedIndex - speed_floor) + leaf_throttle_map[speed_floor][accel_floor] * (speed_ceil - speedIndex)) * (accel_ceil - accelerationIndex);
			break;
		case(CAMRY):
			if (speed_floor == speed_ceil && accel_floor == accel_ceil)
				out = camry_throttle_map[speed_ceil][accel_floor];
			else if (speed_floor == speed_ceil && accel_floor != accel_ceil)
				out = camry_throttle_map[speed_ceil][accel_ceil] * (accelerationIndex - accel_floor) + camry_throttle_map[speed_ceil][accel_floor] * (accel_ceil - accelerationIndex);
			else if (speed_floor != speed_ceil && accel_floor == accel_ceil)
				out = camry_throttle_map[speed_ceil][accel_ceil] * (speedIndex - speed_floor) + camry_throttle_map[speed_floor][accel_ceil] * (speed_ceil - speedIndex);
			else
				out = (camry_throttle_map[speed_ceil][accel_ceil] * (speedIndex - speed_floor) + camry_throttle_map[speed_floor][accel_ceil] * (speed_ceil - speedIndex)) * (accelerationIndex - accel_floor) +
				(camry_throttle_map[speed_ceil][accel_floor] * (speedIndex - speed_floor) + camry_throttle_map[speed_floor][accel_floor] * (speed_ceil - speedIndex)) * (accel_ceil - accelerationIndex);
			break;
		default:
			break;
	}
	
	return out;
}

double long_control::Get_throttle_command() {
	return throttle_level_command;
}

double long_control::Get_deceleration_command() {
	return this->deceleration_command;
}

double long_control::Get_brake_command() {
	return this->brake_level_command;
}

double long_control::Get_desired_acceleration() {
	return this->desired_acceleration;
}

double long_control::Get_setpoint_speed() {
	return this->setpoint_speed;
}

double long_control::Get_limit_accel(){
	double a_max = (cfg->TOP_ACCELERATION_HIGH_SPEED - cfg->TOP_ACCELERATION_LOW_SPEED)/ cfg->MAXIMUM_SPEED * fmax(0, this->long_speed) + cfg->TOP_ACCELERATION_LOW_SPEED;
	return a_max;
}

void long_control::print_monitoring_data(int desired_control_mode, int vehicle_id, int current_mode){
	this->monitoring_counter++;
	if((this->monitoring_counter * this->Ts) < (1/cfg->MONITORING_FREQUENCY))
		return;
	this->monitoring_counter=0;
	if(vehicle_id==LEAF){
		printf("a=%2.1f b=%2.1f A-",this->throttle_level_command, this->brake_level_command);
	}
	if(vehicle_id==PRIUS){
		printf("a=%2.2f P-",this->desired_acceleration);
	}
	if(vehicle_id==CAMRY){
		printf("a=%2.1f b=%2.2f T-", this->throttle_level_command, this->deceleration_command);
	}
	switch(desired_control_mode){
		case(CRUISE):
			printf("Cruise:   t=%.3f       Speed=%.5f,     SetPointSpeed= %.3f,         throttle=%.2f,       brake=%.2f,      A_des=%.2f \n",
							this->current_time,
							this->long_speed,
							this->setpoint_speed,
							this->throttle_level_command,
							this->deceleration_command,
							this->desired_acceleration);
			break;

		case(ACC):
			if(this->is_v2i_engaged)
				printf("V2I-ACC t=%.2f: sync=%s | leadX:%.2f rel_v:%.2f  Vi_1=%.2f  |  egoX:%.2f  d=%.2f | Aref:%.2f  t_rem=%.2f  t_estim=%.2f  dist_rem=%.2f  | Vref:%.1f  E:%.1f v2vt:%.2f  v2i_t:%.2f | Vel:%.1f | Mode:%s\n ",
								this->current_time,
								this->virtual_leader_info.sync_flag ? "true": "false",
								this->virtual_leader_info.leader_x_coordinate,
								this->virtual_leader_info.relative_speed,
								this->virtual_leader_info.leader_speed,
								this->circuit_loc->x_coordinate,
								this->preceding_vehicle.relative_distance,
								this->circuit_loc->ref_accel_traj_plan,
								this->circuit_loc->time_remaining,
								this->circuit_loc->time_estimated,
								this->virtual_leader_info.dist_to_intersection,
								this->setpoint_speed,
								this->ACC_gap_error,
								this->v2v_fault_timer,
								this->v2i_fault_timer,
								this->long_speed,
								this->trajectory_planning_requested ? "TP" : "CF");
			else{
				printf("ACC: t=%.2f  DIST=%.2f  isTV=%s |  V_REF=%.2f  |   CC=%.2f   |  CF_SPEED=%.2f   fb=%.2f   ERROR=%.2f   |   cuts=%d   |   tgap=%.3f   SPEED=%.3f  |  FR_h=%.2f  FR_d=%.2f   Mode: %s %s",
								this->current_time,
								this->preceding_vehicle.relative_distance,
								this->is_preceding_valid ? "true " : "false",
								this->setpoint_speed,
								this->desired_cruise_speed,
								this->ACC_ref_speed,
								this->ACC_fb_out,
								this->ACC_gap_error,
								this->cuts_counter,
								this->ACC_time_gap,
								this->long_speed,
								this->test_ACC_policy->get_current_time_gap(this->long_speed),
								this->test_ACC_policy->get_desired_distance(this->long_speed),
								MODE_NAMES[current_mode],
								this->speed_reduction_zone_flag ? "REDUCING": " ");
				if(current_mode == ACC_2_CRUISE || current_mode == CRUISE_2_ACC || current_mode == ACC_2_CACC || current_mode == CACC_2_ACC)
					printf(" tc:%.1f \n",this->remaining_transition_time);
				else
					printf("\n");
			}
			break;
		case(CACC):
			if(this->is_cacc_performance_flag) printf("Perf-");
			if(this->is_LPF_active) printf("LPF-");
			printf("CACC:t=%.2f d=%.1f isTV=%s  V_REF=%.2f | CC=%.1f | ACC_SP=%.2f E_A=%.1f h_A:%.2f  A_fb=%.2f  |  FF=%.2f  CACC_SP=%.2f  E_C=%.1f h_C:%.2f  C_fb=%.2f |  v2vt:%.2f | Vel=%.3f cut=%d  Mode: %s %s",
							this->current_time,
							this->preceding_vehicle.relative_distance,
							this->is_preceding_valid ? "1" : "0",
							this->setpoint_speed,
							this->desired_cruise_speed,
							this->ACC_ref_speed,
							this->ACC_gap_error,
							this->ACC_time_gap,
							this->ACC_fb_out,
							this->CACC_ff_out,
							this->cooperative_ACC_ref_speed,
							this->CACC_gap_error,
							this->cooperative_ACC_time_gap,
							this->cooperative_ACC_fb_out,
							this->v2v_fault_timer,
							this->long_speed,
							this->cuts_counter,
							MODE_NAMES[current_mode],
							this->speed_reduction_zone_flag ? "REDUCING": "");
			if(current_mode == ACC_2_CRUISE || current_mode == CRUISE_2_ACC || current_mode == ACC_2_CACC || current_mode == CACC_2_ACC)
				printf(" tc:%.1f ",this->remaining_transition_time);

			this->is_v2i_engaged ? printf(" egoX:%.1f\n", this->circuit_loc->get_x_coordinate()) : printf("\n");
			break;
		case(EMERGENCY_BRAKING):
			if(this->my_pip == 1)
				printf("EBRAKE: t=%.2f    d_remain=%.2f   traveled_dist=%.2f    nom_decel=%.2f    a_des=%.2f    Vref=%.2f    Vel=%.2f   accel=%.2f",
							this->current_time,
							this->remaining_distance_EB,
							this->circuit_loc->get_x_coordinate(),
							this->nominal_deceleration_EB,
							this->desired_acceleration,
							this->setpoint_speed,
							this->long_speed,
							this->long_acceleration);
			else
				printf("EBRAKE: t=%.2f    leader_dec=%.2f  d=%.2f   fb=%.2f  a_des=%.2f      ERROR=%.2f    VEL=%.2f",
							this->current_time,
							this->v2x->v2v[0].ref_acceleration,
							this->preceding_vehicle.relative_distance,
							this->ACC_fb_out,
							this->desired_acceleration,
							this->ACC_gap_error,
							this->long_speed);

			printf("\n");
			break;
	}

	return;
}

void long_control::print_cfg_parameters(){
	printf("%.2f \n", cfg->SAMPLING_TIME);
	printf("%.2f \n", cfg->SPEED_TRACKING_P_GAIN);
	printf("%.2f \n", cfg->SPEED_TRACKING_P_GAIN_PERF);
	printf("%.2f \n", cfg->ACC_KP_COMFORT);
	printf("%.2f \n", cfg->ACC_KD_COMFORT);
	printf("%.2f \n", cfg->ACC_KP_PERFORMANCE);
	printf("%.2f \n", cfg->ACC_KD_PERFORMANCE);
	printf("%.2f \n", cfg->HIGHWAY_CACC_KP_COMFORT);
	printf("%.2f \n", cfg->HIGHWAY_CACC_KD_COMFORT);
	printf("%.2f \n", cfg->HIGHWAY_CACC_KP_PERFORMANCE);
	printf("%.2f \n", cfg->HIGHWAY_CACC_KD_PERFORMANCE);
	printf("%.2f \n", cfg->TRACK_CACC_KP_COMFORT);
	printf("%.2f \n", cfg->TRACK_CACC_KD_COMFORT);
	printf("%.2f \n", cfg->TRACK_CACC_KP_PERFORMANCE);
	printf("%.2f \n", cfg->TRACK_CACC_KD_PERFORMANCE);
	printf("%.2f \n", cfg->RELATIVE_SPEED_GAIN);

	printf("%.2f \n", cfg->MAXIMUM_ACC_TIME_GAP_HORIZON);
	printf("%.2f \n", cfg->MINIMUM_ACC_TIME_GAP);
	printf("%.2f \n", cfg->MAXIMUM_CACC_TIME_GAP_HORIZON);
	printf("%.2f \n", cfg->MINIMUM_CACC_TIME_GAP);
	printf("%.2f \n", cfg->ACC_TARGET_TIME_GAP);
	printf("%.2f \n", cfg->CACC_TARGET_TIME_GAP);

	printf("%.2f \n", cfg->STANDSTILL_DISTANCE);
	printf("%.2f \n", cfg->ACC_ERROR_FILTER_FREQ);
	printf("%.2f \n", cfg->CACC_ERROR_FILTER_FREQ);
	printf("%.2f \n", cfg->PERF_CACC_ERROR_FILTER_FREQ);
	printf("%.2f \n", cfg->CUT_IN_A_MAX);
	printf("%.2f \n", cfg->CUT_IN_J_MAX);
	printf("%.2f \n", cfg->CUT_OUT_A_MAX);
	printf("%.2f \n", cfg->CUT_OUT_J_MAX);
	printf("%.2f \n", cfg->TOP_ACCELERATION_LOW_SPEED);
	printf("%.2f \n", cfg->TOP_ACCELERATION_V2I);
	printf("%.2f \n", cfg->TOP_ACCELERATION_HIGH_SPEED);
	printf("%.2f \n", cfg->TOP_DECELERATION);
	printf("%.2f \n", cfg->DESIRED_CC_SPEED);

	printf("%.2f \n", cfg->CUT_DETECTION_TIMER);
	printf("%.2f \n", cfg->MAXIMUM_SPEED);
	printf("%.2f \n", cfg->MINIMUM_SPEED);
	printf("%.2f \n", cfg->GAIN_REDUCTION_AT_MAX_TIME_GAP);
	printf("%.2f \n", cfg->MAX_V2V_MISSING_TIME_TOLERANCE);

	printf("%.2f \n", cfg->KP_VIRTUAL_FOLLOWING_LOW);
	printf("%.2f \n", cfg->KD_VIRTUAL_FOLLOWING_LOW);
	printf("%.2f \n", cfg->KP_VIRTUAL_FOLLOWING_HIGH);
	printf("%.2f \n", cfg->KD_VIRTUAL_FOLLOWING_HIGH);

	printf("%.2f \n", cfg->TRANSITION_DECEL_CRUISE_2_ACC);
	printf("%.2f \n", cfg->TRANSITION_ACCEL_ACC_2_CRUISE);
	printf("%.2f \n", cfg->TRANSITION_ACCEL_ACC_2_CACC);
	printf("%.2f \n", cfg->TRANSITION_ACCEL_CACC_2_ACC);
	printf("%.2f \n", cfg->LATERAL_LANE_TARGET_SPACE);
	printf("%.2f \n", cfg->MAX_MISSING_TARGET_TIME);
	printf("%.2f \n", cfg->MONITORING_FREQUENCY);

	printf("%.2f \n", cfg->LPF_EGO_PRECEDING_FF_GAIN);
	printf("%.2f \n", cfg->LPF_EGO_LEADER_FF_GAIN);
	printf("%.2f \n", cfg->H_MIN_FR_POLICY);
	printf("%.2f \n", cfg->H_TARGET_FR_POLICY);
	printf("%.2f \n", cfg->A_MAX_FR_POLICY);
	printf("%.2f \n", cfg->J_MAX_FR_POLICY);

	printf("%.2f \n", cfg->A_MAX_SPEED_PROFILE);
	printf("%.2f \n", cfg->D_MAX_SPEED_PROFILE);
	printf("%.2f \n", cfg->V_MAX_SPEED_PROFILE);
	printf("%.2f \n", cfg->V_MIN_SPEED_PROFILE);

	printf("%.2f \n", cfg->E_BRAKING_STOPPING_DISTANCE);
	printf("%.2f \n", cfg->MANEUVER_TRIGGER_TIME);
	printf("%.2f \n", cfg->FULL_GAP_ERROR_FILTER_FLAG);
	printf("%.2f \n", cfg->CACC_FF_EGO_PREC_FILTER_FREQ);
	printf("%.2f \n", cfg->CACC_FF_EGO_LEADER_FILTER_FREQ);
	printf("%.2f \n", cfg->DEFAULT_PERFORMANCE_FACTOR);

	printf("%.2f \n", cfg->SWEEP_FREQUENCY_1);
	printf("%.2f \n", cfg->SWEEP_FREQUENCY_2);
	printf("%.2f \n", cfg->SWEEP_AMPLITUDE);
	printf("%.2f \n", cfg->SWEEP_DURATION);
	printf("%.2f \n", cfg->TRAJECTORY_PLANNING_GAIN);
	printf("%.2f \n", cfg->TRAJECTORY_PLANNING_BUFFER_DISTANCE);
	printf("%.2f \n", cfg->LEADER_EMERGENCY_BRAKING_GAIN);

	printf("%.2f \n", cfg->PRIUS_DAMPING_FACTOR);
	printf("%.2f \n", cfg->PRIUS_BANDWIDTH);
	printf("%.2f \n", cfg->LEAF_DAMPING_FACTOR);
	printf("%.2f \n", cfg->LEAF_BANDWIDTH);
	printf("%.2f \n", cfg->CAMRY_DAMPING_FACTOR);
	printf("%.2f \n", cfg->CAMRY_BANDWIDTH);

	return;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////         OTHER		CLASSES 						/////////////////////////////////////////////////////

localization::localization(double _Ts, double _init_lat, double _init_long, double _init_speed, double _init_coordinate, setup_struct* ss, bool _is_v2i){
	this->Ts = _Ts;
	this->x_coordinate = 0;//_init_coordinate;
	this->observed_x = 0;//_init_coordinate;
	this->predicted_x = 0;//_init_coordinate;
	this->speed_integrated_x = 0;//_init_coordinate;
	this->long_speed = _init_speed;
	this->steering = 0;
	this->latitude = _init_lat;
	this->longitude = _init_long;
	this->current_segment_index = 0;
	this->Q = 0.02;
	this->R_obs = 1.0;
	this->cov = this->Q;
	this->a_priori_cov = this->Q;
	this->K=0;
	this->observation_updated = false;
	this->speed_planning_segment_index = 1;
	this->trajectory_planning_gain = ss->TRAJECTORY_PLANNING_GAIN;
	this->buffer_distance = ss->TRAJECTORY_PLANNING_BUFFER_DISTANCE;
	this->trajectory_planning_engaged = false;
	this->init_coordinate = _init_coordinate;
	this->top_acceleration_v2i = ss->TOP_ACCELERATION_V2I;
	this->top_deceleration = ss->TOP_DECELERATION;
	this->ref_accel_traj_plan = 0.;
	this->time_estimated = 0.;
	this->time_remaining = 0.;
	this->is_v2i = _is_v2i;

	this->segments[0] = *(new circuit_segment(-122.3329937600211, 37.91530345847134, -122.3318019205247, 37.91785384630239,        0, 302.3758, 0));
	this->segments[1] = *(new circuit_segment(-122.3318019205247, 37.91785384630239, -122.3331786429029, 37.91826998398804, 302.3758, 431.7708, 1));
	this->segments[2] = *(new circuit_segment(-122.3331786429029, 37.91826998398804, -122.3353741651216, 37.91452521332643, 431.7708, 890.8078, 2));
	this->segments[3] = *(new circuit_segment(-122.3353741651216, 37.91452521332643, -122.3335868682308, 37.91401612059229, 890.8078, 1057.508, 3));
	this->segments[4] = *(new circuit_segment(-122.3335868682308, 37.91401612059229, -122.3329937600211, 37.91530345847134, 1057.508, 1196.470, 4));

}

void localization::update_states(double _speed){
	this->long_speed = _speed;
	predict_from_speed(this->long_speed);
	double end = (this->is_v2i) ? this->segments[4].x_end : 1000000;
	this->x_coordinate = fmod(this->predicted_x, end);
	return;
}

double localization::get_x_coordinate(){
	return (this->x_coordinate + this->init_coordinate);
}

double localization::get_trajectory_planning_ref_accel(double dist_to_intersection, double _time_remaining, double long_speed){
	this->time_estimated = (dist_to_intersection - this->buffer_distance)/ fmax(0.01, long_speed);
	this->time_remaining = _time_remaining;
	if((this->x_coordinate > INTERSECTION_COORDINATE) || (this->x_coordinate < (INTERSECTION_COORDINATE - 300)) || dist_to_intersection > 300 || this->time_remaining > 90){
		this->ref_accel_traj_plan = this->top_acceleration_v2i;
		this->trajectory_planning_engaged = false;
	}else{
		double prorrated_gain = ((this->trajectory_planning_gain / 10) + (9*this->trajectory_planning_gain/10)/(150) * (this->x_coordinate - INTERSECTION_COORDINATE + 300));
		this->trajectory_planning_engaged = true;
		this->ref_accel_traj_plan = fmin(prorrated_gain, this->trajectory_planning_gain) * (this->time_estimated - this->time_remaining);
	}
	return fmin( this->top_acceleration_v2i, fmax( this->top_deceleration, this->ref_accel_traj_plan));
}

double localization::get_top_speed_from_curvature(){
	double top_speed;
	double Vinit, Vfinal, Xinit, Xfinal;
	double current_x = get_x_coordinate();
	if(current_x > circuit_coordinates[this->speed_planning_segment_index]){
		this->speed_planning_segment_index = fmax(1,((this->speed_planning_segment_index+1) % NB_CIRCUIT_WAYPOINTS));
	}else if(this->speed_planning_segment_index == (NB_CIRCUIT_WAYPOINTS-1) && current_x < circuit_coordinates[1])
		this->speed_planning_segment_index = 1;

	Vinit = circuit_velocities[this->speed_planning_segment_index-1];
	Vfinal = circuit_velocities[this->speed_planning_segment_index];
	Xinit = circuit_coordinates[this->speed_planning_segment_index-1];
	Xfinal = circuit_coordinates[this->speed_planning_segment_index];

	this->a0 = (Vfinal*(Xinit-Xfinal)*(Xinit-Xfinal)*(Xinit-Xfinal) - 3*(Vfinal-Vinit)*Xinit*Xfinal*Xfinal + (Vfinal-Vinit)*Xfinal*Xfinal*Xfinal) / ((Xinit-Xfinal)*(Xinit-Xfinal)*(Xinit-Xfinal));
	this->a1 = 6*(Vfinal-Vinit)*Xinit*Xfinal / ((Xinit-Xfinal)*(Xinit-Xfinal)*(Xinit-Xfinal));
	this->a2 = 3*(Xfinal+Xinit)*(Vinit-Vfinal) / ((Xinit-Xfinal)*(Xinit-Xfinal)*(Xinit-Xfinal));
	this->a3 = 2*(Vfinal-Vinit) / ((Xinit-Xfinal)*(Xinit-Xfinal)*(Xinit-Xfinal));

	top_speed = this->a0 + this->a1 * current_x + this->a2 * current_x * current_x + this->a3 * current_x * current_x * current_x;

	return top_speed;
}

void localization::check_if_end_of_segment(double _x){
	if(get_distance_2_points(this->latitude, this->longitude, segments[current_segment_index].latitude_end, segments[current_segment_index].latitude_end) < THRESHOLD_END_POINT){
		if(fabs(this->steering) > THRESHOLD_STEERING_END_POINT){
			current_segment_index = (current_segment_index+1) % NB_CIRCUIT_SEGMENTS;
			this->x_coordinate = segments[current_segment_index].x_init;
			this->latitude = segments[current_segment_index].latitude_init;
			this->longitude = segments[current_segment_index].longitude_init;
			this->speed_integrated_x = this->x_coordinate;
		}
	}
	return;
}

void localization::predict_from_speed(double speed){
	this->speed_integrated_x += this->Ts * speed;
	this->predicted_x += this->Ts * speed;
	this->a_priori_cov += Q;
	return;
}

void localization::correct_with_observation(double obs, double pred){
	this->K = this->a_priori_cov*(this->R_obs + this->a_priori_cov);
	this->x_coordinate = pred + K*(obs - pred);
	this->cov = (1-this->K) * this->a_priori_cov;
	this->a_priori_cov = this->cov;
	this->predicted_x = this->x_coordinate;

	return;
}

double localization::get_distance_2_points(double lat1, double long1, double lat2, double long2){
	double distance = 2*R*asin(pow(sin((lat2-lat1)/2),2) + cos(lat2)*cos(lat1)*pow(sin((long2-long1)/2),2) );
	return distance;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

spacing_policy::spacing_policy(int _spacing_policy_type, int _control_mode, double _long_speed, double _htarg, double _Ts, setup_struct *ss){

	this->spacing_policy_type = _spacing_policy_type;
	this->control_mode = _control_mode;
	this->long_speed = _long_speed;
	this->Ts = _Ts;
	this->desired_distance = get_desired_distance(this->long_speed);
	this->time_gap_profile_length = MAX_LENGTH_TIME_GAP_PROFILE;
	this->Amax = ss->A_MAX_FR_POLICY;
	this->Jmax = ss->J_MAX_FR_POLICY;
	this->cut_detection_frames_counter = -1;
	this->Vlim = 40/3.6*1.6;
	this->ego_speed_filter = new signal_filter(this->Ts, 1, 2*PI*2, 0);
	switch(this->control_mode){
		case(ACC):
			this->htarg  = (this->spacing_policy_type == CONSTANT_TIME_GAP) ? ss->ACC_TARGET_TIME_GAP : ss->H_TARGET_FR_POLICY;
			this->reaction_time = 1/2.25;
			this->hmin = ss->H_MIN_FR_POLICY;
		break;
		case(CACC):
			this->htarg  = (this->spacing_policy_type == CONSTANT_TIME_GAP) ? ss->CACC_TARGET_TIME_GAP : ss->H_TARGET_FR_POLICY;
			this->reaction_time = 0.05;
			this->hmin = ss->H_MIN_FR_POLICY;
		break;
	}
	this->standstill_distance = ss->STANDSTILL_DISTANCE;

	printf("Htarg=%.2f, Hmin=%.2f, Tau=%.2f, std_dist=%.2f\n", this->htarg, this->hmin, this->reaction_time, this->standstill_distance);
}

double spacing_policy::get_desired_distance(double _long_speed){
	//this->long_speed = //this->ego_speed_filter->low_pass_order_1(
		//	_long_speed;
	if(this->spacing_policy_type == CONSTANT_TIME_GAP){
		this->desired_distance = this->standstill_distance + get_current_time_gap()*this->long_speed;
	} else {
		double c22 = ( this->htarg - this->hmin)/(2*this->Vlim);
		if(this->long_speed <= this->Vlim){
			this->desired_distance = this->standstill_distance + this->hmin*this->long_speed + c22*this->long_speed*this->long_speed;
			this->time_gap = this->hmin + (2*c22*this->long_speed);
		}
		else {
			this->desired_distance = (this->htarg*this->long_speed) - (this->htarg*this->Vlim) + (this->standstill_distance + this->hmin*this->Vlim + c22*this->Vlim*this->Vlim);
			this->time_gap = this->htarg;
		}
	}
	return desired_distance;
}

double spacing_policy::get_current_time_gap(double _long_speed){
	this->long_speed = this->ego_speed_filter->low_pass_order_1(_long_speed);
	if(this->spacing_policy_type == CONSTANT_TIME_GAP){
		this->desired_distance = this->standstill_distance + get_current_time_gap()*this->long_speed;
	} else {
		double c22 = ( this->htarg - this->hmin)/(2*this->Vlim);
		if(this->long_speed <= this->Vlim){
			this->desired_distance = this->standstill_distance + this->hmin*this->long_speed + c22*this->long_speed*this->long_speed;
			this->time_gap = this->hmin + (2*c22*this->long_speed);
		}
		else {
			this->desired_distance = (this->htarg*this->long_speed) - (this->htarg*this->Vlim) + (this->standstill_distance + this->hmin*this->Vlim + c22*this->Vlim*this->Vlim);
			this->time_gap = this->htarg;
		}
	}
	return this->time_gap;
}

double spacing_policy::get_current_time_gap(){
	if(this->cut_detection_frames_counter == -1 || this->cut_detection_frames_counter >= this->time_gap_profile_length ){
		this->cut_detection_frames_counter = -1;
		this->time_gap = this->htarg;
		return this->htarg;
	}
	this->time_gap = this->time_gap_profile[this->cut_detection_frames_counter];
	this->cut_detection_frames_counter++;
	return this->time_gap;
}

void spacing_policy::InitGapClosing(float hinit, float htarg, float Amax, float _Jmax, float Veq, float Vmax) {
	printf("\n\nhin %.2f  htar %.2f   Amax %.2f, Jmax %.2f,  Veq %.2f, Vmax %.2f\n\n", hinit, htarg, Amax, _Jmax, Veq, Vmax);
	float Jmax = _Jmax;
	float t1 = Amax / Jmax;
	float requiredDeltaGap = (hinit - htarg) * Veq;
	if (2 * pow(Amax, 3) / pow(Jmax, 2) - requiredDeltaGap >= 0) {
		Jmax = sqrt(2 * pow(Amax,3) / requiredDeltaGap) * 1.1;
		t1 = Amax / Jmax;
	}
	float t2 = -1.5 * (Amax / Jmax) + sqrt(pow(Amax, 4) / (Jmax * Jmax) + 4 * requiredDeltaGap * Amax) / (2 * Amax);
	float v_top = Amax * (t1 + t2) + Veq;
	this->time_gap_profile_length = 0;
	if (v_top > Vmax) {
		// Maximum speed exceeded
		if (((Vmax - Veq) / Amax) < t1) {
			// Not enough time to reach Amax with given Jmax
			EstimateTimeGapProfile_GapClosingCase3(Veq, Amax, Jmax, hinit, requiredDeltaGap, Vmax);
		}
		else {
			EstimateTimeGapProfile_GapClosingCase2(t1, Veq, Amax, Jmax, hinit, requiredDeltaGap, Vmax);
		}
	}
	else {
		EstimateTimeGapProfile_GapClosingCase1(t1, t2, Veq, Amax, Jmax, hinit);
	}
	this->cut_detection_frames_counter = 0;
	return;
}

void spacing_policy::InitGapOpening(float hinit, float htarg, float Amax, float _Jmax, float Veq, float Vmin) {
	printf("\n\nhin %.2f  htar %.2f   Amax %.2f, Jmax %.2f,  Veq %.2f, Vmin %.2f\n\n",hinit, htarg, Amax, _Jmax, Veq, Vmin);
	float Jmax = _Jmax;
	float t1 = Amax / Jmax;
	float requiredDeltaGap = (htarg - hinit) * Veq;
	if (2*pow(Amax, 3) / pow(Jmax, 2) - requiredDeltaGap >= 0) {
		Jmax = sqrt(2 * pow(Amax, 3) / requiredDeltaGap) * 1.1;
		t1 = Amax / Jmax;
	}
	float t2 = -1.5 * (Amax / Jmax) + sqrt(pow(Amax, 4) / (Jmax * Jmax) + 4 * requiredDeltaGap * Amax) / (2 * Amax);
	float v_bottom = - Amax * (t1 + t2) + Veq;
	this->time_gap_profile_length=0;
	if (v_bottom < Vmin) {
		// Maximum speed exceeded
		if (((Veq - Vmin) / Amax) < t1) {
			// Not enough time to reach Amax with given Jmax
			EstimateTimeGapProfile_GapOpeningCase3(Veq, Amax, Jmax, hinit, requiredDeltaGap, Vmin);
		}
		else {
			EstimateTimeGapProfile_GapOpeningCase2(t1, Veq, Amax, Jmax, hinit, requiredDeltaGap, Vmin);
		}
	}
	else {
		EstimateTimeGapProfile_GapOpeningCase1(t1, t2, Veq, Amax, Jmax, hinit);
	}
	this->cut_detection_frames_counter = 0;
	return;
}

void spacing_policy::EstimateTimeGapProfile_GapClosingCase1(float t1, float t2, float Veq, float Amax, float Jmax, float hinit)
{
	printf("GC case1 ");
	int t1_length = (int) (t1 * SAMPLE_FREQ);
	int t2_length = (int) (t2 * SAMPLE_FREQ);
	int t3_length = t1_length;

	this->time_gap_profile_length  = 2*t1_length + 2*t2_length + 2*t3_length;
	printf("2*%d + 2*%d + %d =  %d\n", t1_length, t2_length, t3_length, this->time_gap_profile_length);

	float int_vs = 0;
	float vs = 0.0;
	float t = 0;

	int indA = t1_length;
	int indB = t1_length + t2_length;
	int indC = (t1_length + t2_length + t3_length);
	int indD = (t1_length + t2_length + 2 * t3_length);
	int indE = (t1_length + 2 * t2_length + 2 * t3_length) ;

	int i;
	for (i = 0; i < this->time_gap_profile_length; i++)
        {
		if (i < indA)
        {
            t = i * this->Ts;
            vs = Veq + Jmax * t * t / 2;
        }
		else if(i >= indA && i < indB)
        {
            t = (i-indA)*this->Ts;
            vs = Veq + Amax * t + Jmax * t1 * t1 / 2;
        }
		else if(i >= indB && i < indC)
        {
            t = (i-indB)*this->Ts;
            vs = Veq + Amax * t2 + Jmax * t1 * t1 / 2 + Amax * t - Jmax * t * t / 2;
        }
		else if(i >= indC && i < indD)
        {
            t = (i-indC)*this->Ts;
            vs = Veq + Amax * t2 + Jmax * t1 * t1 / 2 + Amax * (t1 - t) - Jmax * pow(t1 - t, 2) / 2;
        }
		else if(i >= indD && i < indE)
        {
            t = (i-indD)*this->Ts;
            vs = Veq + Amax * (t2 - t) + Jmax * t1 * t1 / 2;
        }
		else  //(i >= indE)
        {
            t = (i-indE)*this->Ts;
            vs = Veq + Jmax * pow(t1 - t, 2) / 2;
        }

		if(i >= 1)
        {
            int_vs = int_vs + (vs - Veq) * this->Ts;
        }

		time_gap_profile[i] = (Veq * hinit - int_vs) / vs;
		printf("  %.3f",time_gap_profile[i]);
	}
	printf("\n");
    return;
}

void spacing_policy::EstimateTimeGapProfile_GapClosingCase2(float t1, float Veq, float Amax, float Jmax, float hinit, float requiredDeltaGap, float Vmax) {
	printf("GC case2 ");

	float t2 = (Vmax - Veq) / Amax - t1;
	float deltaDistance = Amax * t2 * t2 + (3 * pow(Amax, 2) / Jmax) * t2 + 2 * pow(Amax, 3) / (Jmax * Jmax);
	float t4 = (requiredDeltaGap - deltaDistance) / (Vmax - Veq);

	int t1_length = t1 * SAMPLE_FREQ;
	int t2_length = t2 * SAMPLE_FREQ;
	int t4_length = t4 * SAMPLE_FREQ;
	int t3_length = t1_length;

	this->time_gap_profile_length = 2*t1_length+2*t2_length+2*t3_length+t4_length;
	printf("2*%d + 2*%d + %d +2*%d =  %d\n", t1_length, t2_length, t4_length, t3_length, this->time_gap_profile_length);

	float int_vs = 0;
        float vs = 0.0;
        float t = 0;

	int indA = t1_length;
	int indB = t1_length + t2_length;
	int indC = (t1_length + t2_length + t3_length);
	int indD = (t1_length + t2_length + t3_length + t4_length);
	int indE = (t1_length + t2_length + 2 * t3_length + t4_length);
	int indF = (t1_length + 2 * t2_length + 2 * t3_length + t4_length);
	int i;
	for (i = 0; i < this->time_gap_profile_length; i++)
    {
		if (i < indA)
        {
            t = i * this->Ts;
            vs = Veq + Jmax * t * t / 2;
        }
		else if(i >= indA && i < indB)
        {
            t = (i-indA) * this->Ts;
            vs = Veq + Amax * t + Jmax * t1 * t1 / 2;
        }
		else if(i >= indB && i < indC)
        {
            t = (i-indB)* this->Ts;
            vs = Veq + Amax * t2 + Jmax * t1 * t1 / 2 + Amax * t - Jmax * t * t / 2;
        }
		else if(i >= indC && i < indD)
        {
            vs = Vmax;
        }
		else if(i >= indD && i < indE)
        {
            t = (i -indD)*this->Ts;
            vs = Veq + Amax * t2 + Jmax * t1 * t1 / 2 + Amax * (t1 - t) - Jmax * pow(t1 - t,2) / 2;
        }
		else if(i >= indE && i < indF)
        {
            t = (i - indE)*this->Ts;
            vs = Veq + Amax * (t2 - t) + Jmax * t1 * t1 / 2;
        }
		else
        {
            t = (i - indF)*this->Ts;
            vs = Veq + Jmax * pow(t1 - t,2) / 2;
        }

		if (i >= 1)
        {
            int_vs = int_vs + (vs - Veq) * this->Ts;
        }
		time_gap_profile[i] = (Veq * hinit - int_vs) / vs;
		printf("  %.3f",time_gap_profile[i]);
	}
	printf("\n");

    return;
}

void spacing_policy::EstimateTimeGapProfile_GapClosingCase3(float Veq, float Amax, float Jmax, float hinit, float requiredDeltaGap, float Vmax) {

	printf("GC case3 ");

	float t1 = sqrt((Vmax - Veq) / Jmax);
	float deltaDistanceGap = 2 * Jmax * t1 * t1 * t1;
	float t3 = (requiredDeltaGap - deltaDistanceGap) / (Vmax - Veq);
	int t1_length = t1 * SAMPLE_FREQ;
	int t3_length = t3 * SAMPLE_FREQ;

	this->time_gap_profile_length  = 4*t1_length+t3_length;

	printf("4*%d + %d =  %d\n", t1_length, t3_length, this->time_gap_profile_length);

	float int_vs = 0;
    float vs = 0.0;
    float t = 0;

	int indA = t1_length;
	int indB = 2*t1_length;
	int indC = (2*t1_length + t3_length);
	int indD = (3*t1_length + t3_length);
	int i;
	for (i = 0; i < this->time_gap_profile_length; i++) {
		if (i < indA) // Increase acceleration
        {
            t = i * this->Ts;
            vs = Veq + Jmax * t * t / 2;
        }
		else if (i >= indA && i < indB) // decrease acceleration
		{
            t = (i - indA) * this->Ts;
            vs = Veq + Jmax * t1 * t1 / 2 + Jmax * t1 * t - Jmax * t * t / 2;
        }
		else if (i >= indB && i < indC) // maintain Vmax
        {
            vs = Vmax;
        }
		else if (i >= indC && i < indD)
        {
            t = (i - indC) * this->Ts;
            vs = Veq + Jmax * t1 * t1 / 2 + Jmax * t1 * (t1 - t) - Jmax * pow(t1 - t, 2) / 2;
        }
		else
        {
            t = (i - indD) * this->Ts;
            vs = Veq + Jmax * pow(t1 - t, 2) / 2;
        }

		if (i >= 1)
        {
            int_vs += (vs - Veq) * this->Ts;
        }
		time_gap_profile[i] = (Veq * hinit - int_vs) / vs;
		printf("  %.3f",time_gap_profile[i]);
	}
	printf("\n");
    return;
}

void spacing_policy::EstimateTimeGapProfile_GapOpeningCase1(float t1, float t2, float Veq, float Amax, float Jmax, float hinit) {
	printf("GO case1 ");

	int t1_length = t1* SAMPLE_FREQ;
	int t2_length = t2* SAMPLE_FREQ;
	int t3_length = t1_length;

	this->time_gap_profile_length  = 2*t1_length+2*t2_length+2*t3_length;
	printf("2*%d + 2*%d + %d =  %d\n", t1_length, t2_length, t3_length, this->time_gap_profile_length);

    float int_vs = 0;
    float vs = 0.0;
    float t = 0;

	int indA = t1_length;
	int indB = t1_length + t2_length;
	int indC = (t1_length + t2_length + t3_length);
	int indD = (t1_length + t2_length + 2 * t3_length);
	int indE = (t1_length + 2 * t2_length + 2 * t3_length);
	int i;
	for (i = 0; i < this->time_gap_profile_length; i++)
    {
		if (i < indA)
        {
            t = i * this->Ts;
            vs = Veq - Jmax * t * t / 2;
        }
		else if (i >= indA && i < indB)
		{
            t = (i - indA)*this->Ts;
            vs = Veq - (Amax * t + Jmax * t1 * t1 / 2);
        }
		else if (i >= indB && i < indC)
        {
            t = (i-indB)*this->Ts;
            vs = Veq - (Amax * t2 + Jmax * t1 * t1 / 2 + Amax * t - Jmax * t * t / 2);
        }
		else if (i >= indC && i < indD)
        {
            t = (i - indC)*this->Ts;
            vs = Veq - (Amax * t2 + Jmax * t1 * t1 / 2 + Amax * (t1 - t) - Jmax * pow(t1 - t, 2) / 2);
        }
		else if (i >= indD && i < indE)
        {
            t = (i - indD)*this->Ts;
            vs = Veq - (Amax * (t2 - t) + Jmax * t1 * t1 / 2);
        }
		else
        {
            t = (i - indE)*this->Ts;
            vs = Veq - Jmax * pow(t1 - t, 2) / 2;
        }

		if (i >= 1)
        {
            int_vs = int_vs + (vs - Veq) * this->Ts;
        }
		time_gap_profile[i] = (Veq * hinit - int_vs) / vs;
		printf("  %.3f",time_gap_profile[i]);
	}
	printf("\n");
    return;
}

void spacing_policy::EstimateTimeGapProfile_GapOpeningCase2(float t1, float Veq, float Amax, float Jmax, float hinit, float requiredDeltaGap, float Vmin) {
	printf("GO case2 ");

	float t2 = (Veq - Vmin) / Amax - t1;
	float deltaDistance = Amax * t2 * t2 + (3 * pow(Amax, 2) / Jmax) * t2 + 2 * pow(Amax, 3) / (Jmax * Jmax);
	float t4 = (requiredDeltaGap - deltaDistance) / (Veq - Vmin);

	int t1_length = t1* SAMPLE_FREQ;
	int t2_length = t2* SAMPLE_FREQ;
	int t4_length = t4* SAMPLE_FREQ;
	int t3_length = t1_length;
	this->time_gap_profile_length  = 2*t1_length+2*t2_length+t4_length+2*t3_length;

	printf("2*%d + 2*%d + %d +2*%d =  %d\n", t1_length, t2_length, t4_length, t3_length, this->time_gap_profile_length);

    float int_vs = 0;
    float vs = 0.0;
    float t = 0;

	int indA = t1_length;
	int indB = t1_length + t2_length;
	int indC = (t1_length + t2_length + t3_length);
	int indD = (t1_length + t2_length + t3_length + t4_length);
	int indE = (t1_length + t2_length + 2 * t3_length + t4_length);
	int indF = (t1_length + 2 * t2_length + 2 * t3_length + t4_length);
	int i;
	for (i = 0; i < this->time_gap_profile_length; i++)
    {
		if (i < indA)
        {
            t = i * this->Ts;
            vs = Veq - Jmax * t * t / 2;
        }
		else if (i >= indA && i < indB)
        {
            t = (i - indA) * this->Ts;
            vs = Veq - (Amax * t + Jmax * t1 * t1 / 2);
        }
		else if (i >= indB && i < indC)
        {
            t = (i - indB)* this->Ts;
            vs = Veq - (Amax * t2 + Jmax * t1 * t1 / 2 + Amax * t - Jmax * t * t / 2);
        }
		else if (i >= indC && i < indD)
        {
            vs = Vmin;
        }
		else if (i >= indD && i < indE)
        {
            t = (i - indD)* this->Ts;
            vs = Veq - (Amax * t2 + Jmax * t1 * t1 / 2 + Amax * (t1 - t) - Jmax * pow(t1 - t, 2) / 2);
        }
		else if (i >= indE && i < indF)
        {
            t = (i - indE)*this->Ts;
            vs = Veq - (Amax * (t2 - t) + Jmax * t1 * t1 / 2);
        }
		else
        {
            t = (i - indF)*this->Ts;
            vs = Veq - Jmax * pow(t1 - t, 2) / 2;
        }

		if (i >= 1)
        {
            int_vs = int_vs + (vs - Veq) * this->Ts;
        }
		time_gap_profile[i] = (Veq * hinit - int_vs) / vs;
		printf("  %.3f",time_gap_profile[i]);
	}
	printf("\n");
    return;
}

void spacing_policy::EstimateTimeGapProfile_GapOpeningCase3(float Veq, float Amax, float Jmax, float hinit, float requiredDeltaGap, float Vmin) {
	printf("GO case3 ");

	float t1 = sqrt((Veq - Vmin) / Jmax);
	float deltaDistanceGap = 2 * Jmax * t1 * t1 * t1;
	float t3 = (requiredDeltaGap - deltaDistanceGap) / (Veq - Vmin);
	int t1_length = t1* SAMPLE_FREQ;
	int t3_length = t3* SAMPLE_FREQ;

	this->time_gap_profile_length  = 4*t1_length+2*t3_length;
	printf("4*%d + %d =  %d\n", t1_length, t3_length, this->time_gap_profile_length);

    float int_vs = 0;
    float vs = 0.0;
    float t = 0;

	int indA = t1_length;
	int indB = 2 * t1_length;
	int indC = (2 * t1_length + t3_length);
	int indD = (3 * t1_length + t3_length);

	int i;
	for (i = 0; i < this->time_gap_profile_length; i++)
    {
		if (i < indA) // Increase acceleration
        {
            t = i * this->Ts;
            vs = Veq - Jmax * t * t / 2;
        }
		else if (i >= indA && i < indB) // decrease acceleration
		{
            t = (i - indA)*this->Ts;
            vs = Veq - (Jmax * t1 * t1 / 2 + Jmax * t1 * t - Jmax * t * t / 2);
        }
		else if (i >= indB && i < indC) // maintain Vmax
        {
            vs = Vmin;
        }
		else if (i >= indC && i < indD)
        {
            t = (i - indC)*this->Ts;
            vs = Veq - (Jmax * t1 * t1 / 2 + Jmax * t1 * (t1 - t) - Jmax * pow(t1 - t, 2) / 2);
        }
		else
        {
            t = (i - indD)*this->Ts;
            vs = Veq -( Jmax * pow(t1 - t, 2) / 2);
        }

		if (i >= 1)
        {
            int_vs += (vs - Veq) * this->Ts;
        }

		time_gap_profile[i] = (Veq * hinit - int_vs) / vs;
		printf("  %.3f",time_gap_profile[i]);
	}
	printf("\n");
    return;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

signal_filter::signal_filter(double _Ts, int _filter_order, double _w1, double _w2){
	this->Ts = _Ts;
	this->w1 = _w1;
	this->w2 = _w2;
	this->filter_order = _filter_order;

	this->in_z_1 = 0;
	this->in_z_2 = 0;
	this->in_z_3 = 0;
	this->out_z_1 = 0;
	this->out_z_2 = 0;
	this->out_z_3 = 0;
	this->out = 0.;

}

signal_filter::signal_filter(double _Ts, double _ego_xi, double _ego_wn, double _prec_xi, double _prec_wn, double _init){
	this->Ts = _Ts;
	this->filter_order = 2;
	this->w1 = 0;
	this->w2 = 0;
	this->ego_xi = _ego_xi;
	this->ego_wn = _ego_wn;
	this->prec_xi = _prec_xi;
	this->prec_wn = _prec_wn;
	this->in_z_1 = _init;
	this->in_z_2 = _init;
	this->in_z_3 = _init;
	this->out_z_1 = _init;
	this->out_z_2 = _init;
	this->out_z_3 = _init;
	this->out=_init;
	printf("signal_filter: Getting wn_i=%.2f, xi_i=%.2f and wn_i-1=%.2f, xi_i-1=%.2f \n", this->ego_wn, this->ego_xi, this->prec_wn, this->prec_xi);
}

double signal_filter::low_pass_order_1(double in) {
	double n0 = this->Ts;
	double n1 = this->Ts;
	double d0 = 2 * (1 / this->w1) + this->Ts;
	double d1 = this->Ts - 2 * (1 / this->w1);
	this->out = (1 / d0) * (n0 * in + n1 * this->in_z_1 - d1 * this->out_z_1);
	this->in_z_1 = in;
	this->out_z_1 = this->out;
	return this->out;
}

double signal_filter::low_pass_order_2(double in) {
	double n0 = w1*w2*Ts*Ts;
	double n1 = 2 * w1*w2*Ts*Ts;
	double n2 = w1*w2*Ts*Ts;

	double d0 = 4+2*Ts*(w1+w2)+w1*w2*Ts*Ts;
	double d1 = 2*Ts*Ts*w1*w2-8;
	double d2 = 4-2*(w1+w2)*Ts+w1*w2*Ts*Ts;

	this->out = (1 / d0) * (n0 * in + n1 * this->in_z_1 + n2 * this->in_z_2	- d1 * this->out_z_1 - d2 * this->out_z_2);

	this->in_z_2 = this->in_z_1;
	this->in_z_1 = in;
	this->out_z_2 = this->out_z_1;
	this->out_z_1 = this->out;

	return this->out;
}

double signal_filter::low_pass_order_1(double in, double _w1) {
	double n0 = this->Ts;
	double n1 = this->Ts;
	double d0 = 2 * (1 / _w1) + this->Ts;
	double d1 = this->Ts - 2 * (1 / _w1);
	this->out = (1 / d0) * (n0 * in + n1 * this->in_z_1 - d1 * this->out_z_1);
	this->in_z_1 = in;
	this->out_z_1 = this->out;
	return this->out;
}

double signal_filter::low_pass_order_2(double in, double _w1, double _w2) {
	double n0 = _w1*_w2*Ts*Ts;
	double n1 = 2 * _w1*_w2*Ts*Ts;
	double n2 = _w1*_w2*Ts*Ts;

	double d0 = 4+2*Ts*(_w1+_w2)+_w1*_w2*Ts*Ts;
	double d1 = 2*Ts*Ts*_w1*_w2-8;
	double d2 = 4-2*(_w1+_w2)*Ts+_w1*_w2*Ts*Ts;

	this->out = (1 / d0) * (n0 * in + n1 * this->in_z_1 + n2 * this->in_z_2	- d1 * this->out_z_1 - d2 * this->out_z_2);

	this->in_z_2 = this->in_z_1;
	this->in_z_1 = in;
	this->out_z_2 = this->out_z_1;
	this->out_z_1 = this->out;

	return this->out;
}

double signal_filter::FF_filter(double in){
	double Ts = this->Ts;
	double Twi =  1/this->ego_wn;
	double DFi = this->ego_xi;
	double Twi_1 = 1/this->prec_wn;
	double DFi_1 = this->prec_xi;



	this->out = (4*Twi*Twi+4*DFi*Twi*Ts+Ts*Ts)*in + (2*Ts*Ts-8*Twi*Twi)*this->in_z_1 + (4*Twi*Twi-4*DFi*Twi*Ts+Ts*Ts)*this->in_z_2;
	this->out = this->out - (2*Ts*Ts-8*Twi_1*Twi_1)*this->out_z_1 -	(4*Twi_1*Twi_1-4*DFi_1*Twi_1*Ts+Ts*Ts)*this->out_z_2;
	this->out /= (4*Twi_1*Twi_1+4*DFi_1*Twi_1*Ts+Ts*Ts);

/*
	this->out = ((4*Twi*Twi+4*DFi*Twi*Ts+Ts*Ts)  *in +
					(2*Ts*Ts-8*Twi*Twi)           *this->in_z_1 +
					(4*Twi*Twi-4*DFi*Twi*Ts+Ts*Ts)*this->in_z_2 -

					(2*Ts*Ts-8*Twi_1*Twi_1)       *this->out_z_1 -
			(4*Twi_1*Twi_1-4*DFi_1*Twi_1*Ts+Ts*Ts)*this->out_z_2) /
					(4*Twi_1*Twi_1+4*DFi_1*Twi_1*Ts+Ts*Ts);
*/
	/*printf("signal_filter: prec_wn=%.2f       o_i=%.2f  o_i-1=%.2f   o_i-2=%.2f   i_i=%.2f   i_i-1=%.2f   i_i-2=%.2f \n",
				this->prec_wn,
				this->out,
				this->out_z_1,
				this->out_z_2,
				in,
				this->in_z_1,
				this->in_z_2);
*/
	this->in_z_2 = this->in_z_1;
	this->in_z_1 = in;
	this->out_z_2 = this->out_z_1;
	this->out_z_1 = this->out;

	return this->out;
}


void signal_filter::init_values(double in){
	this->in_z_1 = in;
	this->in_z_2 = in;
	this->in_z_3 = in;
	this->out_z_1 = in;
	this->out_z_2 = in;
	this->out_z_3 = in;
	this->out = in;

	return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

speed_profile::speed_profile(double _Ts, double _v0, int _scenario_id, setup_struct ss){
	this->ss = ss;
	this->scenario_id = _scenario_id;
	set_profile_parameters(_scenario_id);
	this->Ts = _Ts;
	this->v0 = _v0;
	this->state = 0;
	this->down_step = true;
}

void speed_profile::set_profile_parameters(int _scenario_id){
	switch(_scenario_id){
		case(CONSTANT_SPEED):
			this->profile_nb = CONSTANT_SPEED_PROFILE;
			this->init_speed = this->ss.DESIRED_CC_SPEED;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(CUSTOM_SPEED_PLANNING):
			this->profile_nb = SPEED_STEPS_FOR_TESTS;
			this->Dmax = ss.D_MAX_SPEED_PROFILE;
			this->Amax = ss.A_MAX_SPEED_PROFILE;
			this->Atop = -Dmax;
			this->Vmax = ss.V_MAX_SPEED_PROFILE * 1.6 / 3.6;
			this->Vmin = ss.V_MIN_SPEED_PROFILE * 1.6 / 3.6;
			this->init_speed = this->Vmax;
			deltaV = (this->Vmax - this->Vmin);
			this->T = 4 * deltaV / (3*this->Dmax);
			this->Jmax = 4*this->Atop/this->T;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			printf("Custom speed profile;   T=%.2f   Jmax=%.2f   Atop=%.2f", this->T, this->Jmax, this->Atop);
			break;
		case(STEPS_HIGH_SPEED_LOW_ACCEL):
			this->profile_nb = SPEED_STEPS_FOR_TESTS;
			this->Dmax = 0.6;
			this->Amax = 0.4;
			this->Atop = -Dmax;
			this->Vmax = 65 * 1.6 / 3.6;
			this->Vmin = 55 * 1.6 / 3.6;
			this->init_speed = this->Vmax;
			deltaV = (this->Vmax - this->Vmin);
			this->T = 4 * deltaV / (3*this->Dmax);
			this->Jmax = 4*this->Atop/this->T;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			printf("Case Steps high speed low accel;   T=%.2f   Jmax=%.2f   Atop=%.2f", this->T, this->Jmax, this->Atop);
			break;
		case(STEPS_HIGH_SPEED_HIGH_ACCEL):
			this->profile_nb = SPEED_STEPS_FOR_TESTS;
			this->Dmax = 1.2;
			this->Amax = 2.0;
			this->Atop = -Dmax;
			this->Vmax = 70 * 1.6 / 3.6;
			this->Vmin = 50 * 1.6 / 3.6;
			this->init_speed = this->Vmax;
			deltaV = (this->Vmax - this->Vmin);
			this->T = 4 * deltaV / (3*this->Dmax);
			this->Jmax = 4*this->Atop/this->T;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(STEPS_LOW_SPEED_LOW_ACCEL):
			this->profile_nb = SPEED_STEPS_FOR_TESTS;
			this->Dmax = 1.2;
			this->Amax = 1.0;
			this->Atop = -Dmax;
			this->Vmax = 35 * 1.6 / 3.6;
			this->Vmin = 20 * 1.6 / 3.6;
			this->init_speed = this->Vmax;
			deltaV = (this->Vmax - this->Vmin);
			this->T = 4 * deltaV / (3*this->Dmax);
			this->Jmax = 4*this->Atop/this->T;
			printf("Case Steps low speed low accel;   T=%.2f   Jmax=%.2f   Atop=%.2f", this->T, this->Jmax, this->Atop);
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(STEPS_LOW_SPEED_HIGH_ACCEL):
			this->profile_nb = SPEED_STEPS_FOR_TESTS;
			this->Dmax = 4.0;
			this->Amax = 2.0;
			this->Atop = -Dmax;
			this->Vmax = 35 * 1.6 / 3.6;
			this->Vmin = 20 * 1.6 / 3.6;
			this->init_speed = this->Vmax;
			deltaV = (this->Vmax - this->Vmin);
			this->T = 4 * deltaV / (3*this->Dmax);
			this->Jmax = 4*this->Atop/this->T;
			printf("Case Steps low speed high accel;   T=%.2f   Jmax=%.2f   Atop=%.2f", this->T, this->Jmax, this->Atop);
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(STEPS_VARYING_RATE_PROFILE):
			this->profile_nb = STEPS_VARYING_RATE_PROFILE;
			this->Dmax = ss.D_MAX_SPEED_PROFILE*0.25;
			this->Amax = ss.A_MAX_SPEED_PROFILE*0.25;
			this->Atop = -Dmax;
			this->Vmax = ss.V_MAX_SPEED_PROFILE * 1.6 / 3.6;
			this->Vmin = ss.V_MIN_SPEED_PROFILE * 1.6 / 3.6;
			this->init_speed = this->Vmax;
			deltaV = (this->Vmax - this->Vmin);
			this->T = 4 * deltaV / (3*this->Dmax);
			this->Jmax = 4*this->Atop/this->T;
			printf("Custom speed profile;   T=%.2f   Jmax=%.2f   Atop=%.2f", this->T, this->Jmax, this->Atop);
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(MULTISINE_LOW_SPEED_LOW_ACCEL):
			this->profile_nb = MULTI_SINE;
			this->init_speed = multisine_low_speed[0];
			this->profile_index=0;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(MULTISINE_HIGH_SPEED_LOW_ACCEL):
			this->profile_nb = MULTI_SINE;
			this->init_speed = multisine_high_speed[0];
			this->profile_index=0;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(MULTISINE_LOW_SPEED_HIGH_ACCEL):
			this->profile_nb = MULTI_SINE;
			this->init_speed = multisine_low_speed_2[0];
			this->profile_index=0;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(MULTISINE_HIGH_SPEED_HIGH_ACCEL):
			this->profile_nb = MULTI_SINE;
			this->init_speed = multisine_high_speed_2[0];
			this->profile_index=0;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(EMERGENCY_STOP):
			this->init_speed = ss.DESIRED_CC_SPEED;
			this->profile_nb = EMERGENCY_STOP;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
			break;
		case(SINE_SWEEP):
			this->profile_nb = SINE_SWEEP;
			this->sweep_freq1 = ss.SWEEP_FREQUENCY_1;
			this->sweep_freq2 = ss.SWEEP_FREQUENCY_2;
			this->sweep_amplitude = ss.SWEEP_AMPLITUDE;
			this->sweep_duration = ss.SWEEP_DURATION;
			this->init_speed = ss.DESIRED_CC_SPEED;
			this->t0 = ss.MANEUVER_TRIGGER_TIME;
		}
}

double speed_profile::Get_x_position(){
	return this->x_pose;
}

void speed_profile::GetReferenceSpeed(double t){
	double speed=0;
	switch(this->profile_nb){
		case(CONSTANT_SPEED):
			if(t < 30){
				speed = this->v0; // initial standstill
				break;
			}
			if(t >=30 && t < this->t0){
				speed = ((this->init_speed - this->v0)/(this->t0-30))*(t-30) + this->v0;	// accelerate to init speed
				this->state = 0;
			} if(t>= t0){
				speed = this->init_speed;
				this->state = 1;
			}
			break;
		case(SPEED_STEPS_FOR_TESTS):
			if(t < 30){
				speed = this->v0; // initial standstill
				break;
			}
			if(t >=30 && t < this->t0){
				speed = ((this->init_speed - this->v0)/(this->t0-30))*(t-30) + this->v0;	// accelerate to init speed
				this->state = 0;
			} if(t>= t0 && t < this->t0 + this->T){
				speed = this->init_speed;	// keep init speed for T time
				this->state = 1;
			} if(t >= this->t0 + this->T && t < this->t0 + 2*this->T){				// Speed planning
				this->state = 2;
				if(t >= this->t0 + this->T && t < this->t0 + this->T + this->T/4){
					speed = this->init_speed + this->Jmax * pow(t - this->t0 - this->T, 2)/2;
				}
				if(t >= this->t0 + this->T + this->T/4 && t < this->t0 + this->T + this->T*3/4){
					speed = this->init_speed + this->Jmax * pow(this->T/4, 2)/2 + this->Atop * (t - this->t0 - this->T - this->T/4);
				}
				if(t >= this->t0 + this->T + this->T*3/4 && t < this->t0 + 2*this->T){
					speed = this->init_speed + this->Jmax * pow(this->T/4, 2)/2 + this->Atop * (this->T/2)
								- this->Jmax * (t - this->t0 - this->T - this->T*3/4)*(t - this->t0 - this->T - this->T*3/4) / 2
								+ this->Atop * (t - this->t0 - this->T - this->T*3/4);
				}
			} if(t >= this->t0 + 2*this->T && this->state == 2){
				this->state = 1;
				this->down_step = !(this->down_step || this->down_step);
				this->init_speed += this->Atop * (this->T*3/4);
				if(!this->down_step){
					this->Atop = Amax;
					deltaV = (this->Vmax - this->Vmin);
					this->T = (4 * deltaV) / (3*this->Amax);
				} else {
					this->Atop = -Dmax;
					deltaV = (this->Vmax - this->Vmin);
					this->T = (4 * deltaV) / (3*this->Dmax);
				}
				this->Jmax = 4*this->Atop/this->T;
				this->t0 = t;
				speed = this->init_speed;
				printf("downstep=%s  T=%.2f deltaV=%.2f  Jmax=%.2f   Atop=%.2f, init_speed=%.2f \n", this->down_step ? "yes" : "no", this->T, this->Vmax-this->Vmin, this->Jmax, this->Atop, this->init_speed);
			}
			break;
		case(MULTI_SINE):
			if(t < 30){
				speed = this->v0; // initial standstill
				break;
			}
			if(t >=30 && t < this->t0){
				speed = ((this->init_speed - this->v0)/(this->t0-30))*(t-30) + this->v0;	// accelerate to init speed
				this->state = 0;
			} if(t>= t0){
				if(this->profile_index >= MULTISINE_PROFILE_LENGTH){
					return;
				}
				switch(this->scenario_id){
					case(MULTISINE_LOW_SPEED_LOW_ACCEL):
						speed = multisine_low_speed[this->profile_index];
						break;
					case(MULTISINE_HIGH_SPEED_LOW_ACCEL):
						speed = multisine_high_speed[this->profile_index];
						break;
					case(MULTISINE_LOW_SPEED_HIGH_ACCEL):
						speed = multisine_low_speed_2[this->profile_index];
						break;
					case(MULTISINE_HIGH_SPEED_HIGH_ACCEL):
						speed = multisine_high_speed_2[this->profile_index];
						break;
				}
				this->profile_index++;
			}
			break;
		case(STEPS_VARYING_RATE_PROFILE):
			if(t < 30){
				speed = this->v0; // initial standstill
				break;
			}
			if(t >=30 && t < this->t0){
				speed = ((this->init_speed - this->v0)/(this->t0-30))*(t-30) + this->v0;	// accelerate to init speed
				this->state = 0;
			} if(t>= t0 && t < this->t0 + this->T){
				speed = this->init_speed;	// keep init speed for T time
				this->state = 1;
			} if(t >= this->t0 + this->T && t < this->t0 + 2*this->T){				// Speed planning
				this->state = 2;

				if(t >= this->t0 + this->T && t < this->t0 + this->T + this->T/4){
					speed = this->init_speed + this->Jmax * pow(t - this->t0 - this->T, 2)/2;
				}
				if(t >= this->t0 + this->T + this->T/4 && t < this->t0 + this->T + this->T*3/4){
					speed = this->init_speed + this->Jmax * pow(this->T/4, 2)/2 + this->Atop * (t - this->t0 - this->T - this->T/4);
				}
				if(t >= this->t0 + this->T + this->T*3/4 && t < this->t0 + 2*this->T){
					speed = this->init_speed + this->Jmax * pow(this->T/4, 2)/2 + this->Atop * (this->T/2)
								- this->Jmax * (t - this->t0 - this->T - this->T*3/4)*(t - this->t0 - this->T - this->T*3/4) / 2
								+ this->Atop * (t - this->t0 - this->T - this->T*3/4);
				}
			} if(t >= this->t0 + 2*this->T && this->state == 2){
				this->state = 1;
				this->down_step = !(this->down_step || this->down_step);
				this->init_speed += this->Atop * (this->T*3/4);
				if(!this->down_step){
					this->Amax += ss.A_MAX_SPEED_PROFILE/4;
					this->Dmax += ss.D_MAX_SPEED_PROFILE/4;
					this->Atop = Amax;
					deltaV = (this->Vmax - this->Vmin);
					this->T = (4 * deltaV) / (3*this->Amax);
				} else {
					this->Atop = -Dmax;
					deltaV = (this->Vmax - this->Vmin);
					this->T = (4 * deltaV) / (3*this->Dmax);
				}
				this->Jmax = 4*this->Atop/this->T;
				this->t0 = t;
				speed = this->init_speed;
				printf("downstep=%s  T=%.2f deltaV=%.2f  Jmax=%.2f   Atop=%.2f, init_speed=%.2f \n", this->down_step ? "yes" : "no", this->T, this->Vmax-this->Vmin, this->Jmax, this->Atop, this->init_speed);
			}
			break;
		case(EMERGENCY_STOP):
			if(t < 30){
				speed = this->v0; // initial standstill
				break;
			}
			if((t >=30) && (t < this->t0)){
				speed = ((this->init_speed - this->v0)/(this->t0-30))*(t-30) + this->v0;	// accelerate to init speed
				this->state = 0;
			}
			if((t>= this->t0) && (t < (this->t0 + 10))){
				speed = this->init_speed;
			}
			if(t>= (this->t0 + 10)){
				speed = -1;
			}
			break;
		case(SINE_SWEEP):
			if(t < 30){
				speed = this->v0; // initial standstill
				break;
			}
			if((t >=30) && (t < this->t0)){
				speed = ((this->init_speed - this->v0)/(this->t0-30))*(t-30) + this->v0;	// accelerate to init speed
				this->state = 0;
			}
			if((t>= this->t0) && (t < (this->t0 + this->sweep_duration))){
				double current_freq = this->sweep_freq1 + (t-this->t0)*(this->sweep_freq2 - this->sweep_freq1)/(this->sweep_duration);
				speed = this->init_speed + this->sweep_amplitude*sin(current_freq*2*PI*(t-this->t0));
			}
			if(t>= (this->t0 + this->sweep_duration)){
				speed = outSpeed;
			}
			break;
	}
	this->outSpeed = speed;
	this->x_pose += speed*this->Ts;
	return;

}

