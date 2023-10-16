#include "argonne_cacc.hpp"
using namespace std;

struct radar_object target[MAX_TARGET_NUM];
db_clt_typ *pclt;
static jmp_buf exit_env;
static int sig_list[] = {
        SIGINT,
        SIGQUIT,
        SIGTERM,
        SIGALRM,
        ERROR,
};
static void sig_hand(int code)
{
        if (code == SIGALRM)
			return;
        else
			longjmp(exit_env, code);
}

int main(int argc, char *argv[])
{
	Initialization_task(argc, argv);

	while(true) {
		if (!Read_inputs()){
			break;
		}
		if (!Process_data()) {
			break;
		}			
		if (!Write_outputs()) {
			break;
		}
		usleep(config->SAMPLING_TIME * 1000000 - 2000); // Makes system work at 50Hz, clock tested
	}

	End_tasks();
	return 0;
}

void Initialization_task(int argc, char *argv[]) {

	printf("\nargonne_cacc started \n");

	char hostname[MAXHOSTNAMELEN];
	char *domain; /// on Linux sets DB q-file directory
	int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h

	get_local_name(hostname, MAXHOSTNAMELEN);
	if ( (pclt = db_list_init(argv[0], hostname, domain, xport, (db_id_t *)NULL, (int)0, (int *)NULL, (int)0))	== NULL) {
		exit(EXIT_FAILURE);
	}
	if (setjmp(exit_env) != 0) {
		output_t output;
		output.torque_level = 0; output.accel_decel_request = 0;
		db_clt_write(pclt,DB_OUTPUT_VAR, sizeof(output_t), &output);
		db_list_done(pclt, (db_id_t *)NULL, (int)0, (int *)NULL, (int)0);
		delete sProfile;
		delete control_structure;
		exit(EXIT_SUCCESS);
	} else
	    sig_ign(sig_list, sig_hand);

	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	// TO CHANGE IN FUNCTION OF MODE, STRING POSITION AND VEHICLE TYPE

	int opt;

	desired_control_mode = ACC;
	ego_vehicle_id = PRIUS;
	preceding_vehicle_id = CAMRY;
	my_pip = 1;
	v2i_engaged = false;
	with_hmi = 0;
	cacc_flag_performance_test = 0;
	test_scenario_id = 0;
	manual_control_requested = false;
	preferred_topology = PREDECESSOR_ONLY;
	char* file_name;
	config = new setup_struct(ego_vehicle_id);
	comm_verbosity = false;
	camry_verbosity = false;
	fuel_verbosity = false;
	gps_verbosity = false;
	enable_flag_from_hmi = 0;
	int verbosity = 0;

	while((opt = getopt(argc, argv, "i:c:e:p:s:f:t:b:xhm")) != -1){
		switch(opt) {
			case 'i':
				my_pip = atoi(optarg);
				if(my_pip < 1 || my_pip > 3){
					printf("Pip must be between 1 and 3 \n");
					exit(EXIT_FAILURE);
				}
				break;
			case 'c':
				desired_control_mode = atoi(optarg);
				if(desired_control_mode < 0 || desired_control_mode > 3){
					printf("Control mode must be 0-cruise, 1-acc, 2-cacc, 3-cacc_perf \n");
					exit(EXIT_FAILURE);
					break;
				}
				if(desired_control_mode == CACC_PERF){
					desired_control_mode = CACC;
					printf("argonne_cacc: Performance CACC requested\n");
					cacc_flag_performance_test = true;
				}
				break;
			case 'e':
				ego_vehicle_id = atoi(optarg);
				if(ego_vehicle_id < 0 || ego_vehicle_id > 2){
					printf("Ego ID must be between 0 and 2");
					exit(EXIT_FAILURE);
				}
				config = new setup_struct(ego_vehicle_id);
				printf("Loading parameters for %d\n",ego_vehicle_id);
				break;
			case 'p':
				preceding_vehicle_id = atoi(optarg);
				if(preceding_vehicle_id < 0 || preceding_vehicle_id > 2){
					printf("Preceding ID must be between 0 and 2");
					exit(EXIT_FAILURE);
				}
				break;
			case 's':
				test_scenario_id = atoi(optarg);
				if(test_scenario_id < -1 || test_scenario_id > 12 ){
					printf("Preceding ID must be between -1 and 12");
					exit(EXIT_FAILURE);
				}
				break;
			case 't':
				preferred_topology = atoi(optarg);
				if(preferred_topology < 0 || preferred_topology > 1 ){
					printf("Topology has to be either PF:0 or LPF:1 \n");
					exit(EXIT_FAILURE);
				}
				break;
			case 'f':
				file_name = optarg;
				printf("Requested file name: %s \n", file_name);
				if(!Load_configuration_file(file_name)){
					printf("Something is wrong with the configuration file, exiting.\n");
					exit(EXIT_FAILURE);
				}
				printf("Configuration file loaded.\n");
				break;
			case 'x':
				v2i_engaged = true;
				printf("V2I_requested\n");
				break;
			case 'h':
				with_hmi = 1;
				printf("HMI requested\n");
				break;
			case 'm':
				manual_control_requested = true;
				printf("Manual control requested, actuation is not applied.\n");
				break;
			case 'b':
				verbosity = atoi(optarg);
				if(verbosity == 1)
					comm_verbosity = true;
				if(verbosity == 2)
					camry_verbosity = true;
				if(verbosity == 3)
					fuel_verbosity = true;
				if(verbosity == 4)
					gps_verbosity = true;
				printf("verbosity %d\n", verbosity);
				break;
			default:
				printf("Not recognized: %s %s\n", argv[0], opt);
				exit(EXIT_FAILURE);
				break;
		}
	}
	// BAD SETTINGS VERIFICATIONS
	if((desired_control_mode == CACC) && (my_pip == 1)){
		printf("Leader vehicle cannot be in CACC mode, exiting\n");
		exit(EXIT_FAILURE);
	}
	if(((desired_control_mode == ACC) || (desired_control_mode == CRUISE)) && (my_pip > 1)){
		printf("Followers on platoon should be set in CACC mode, exiting\n");
		exit(EXIT_FAILURE);
	}

	// END
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////

	executions = 0;
	i_setpoint_speed = 0;
	trigger_EB = false;

	db_2_hmi_data = (tx_data*)  malloc(sizeof(tx_data));
	hmi_2_db_data = (rx_data*)  malloc(sizeof(rx_data));

	while(!enable_flag_from_hmi && with_hmi){
		db_clt_read(pclt, CACC_CONTROL_ENABLE, sizeof(int), &enable_flag_from_hmi);
		usleep(100000);
	}
	printf("\n argonne_cacc: enabled for control! \n");

	InitOutputFile();

	if(with_hmi)
		Init_db_variables_from_hmi();

	input_t input;
	db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
	i_long_speed = input.vehicle_speed_mps;

	clock_gettime( CLOCK_REALTIME, &t_start);
	control_structure = new long_control(config, ego_vehicle_id, desired_control_mode, i_long_speed, preceding_vehicle_id, (with_hmi==1),
						cacc_flag_performance_test, v2i_engaged, manual_control_requested, my_pip, preferred_topology);

	sProfile = new speed_profile(config->SAMPLING_TIME, i_long_speed, test_scenario_id, *config);

	return;
}

bool Read_inputs() {
	input_t input;
	db_steinhoff_out_t db_steinhoff_brake_out;
	db_steinhoff_out_t db_steinhoff_accel_out;

	db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
	i_long_speed = input.vehicle_speed_mps;
	i_long_acceleration = input.vehicle_accel_mps2;
	i_fuel_rate = input.fuel_rate;
	i_steering = input.steering_angle_deg;

	db_clt_read(pclt,DB_GPS_GGA_VAR, sizeof(path_gps_point_t), &gps_data);
	printf("Read_inputs: i_long_speed %.3f input.vehicle_speed_mps %.3f\n",
			i_long_speed,
			input.vehicle_speed_mps
	);

	if(gps_verbosity)
		printf("\n    Lat = %.4f  Long = %.4f  Head = %.4f\n", gps_data.latitude, gps_data.longitude, gps_data.heading);

	if(fuel_verbosity)
		printf("\n    Fuel rate = %.4f \n", i_fuel_rate);

	Read_db_v2x_comm_data();
	control_structure->v2x->v2i.leader_x_coordinate = input.distance_from_start;

	db_clt_read(pclt, 4000, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);
	db_clt_read(pclt, 5000, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);

	clock_gettime( CLOCK_REALTIME, &t_current);
	current_time = ( t_current.tv_sec - t_start.tv_sec ) + (double)( t_current.tv_nsec - t_start.tv_nsec ) / (double) BILLION;

	int nb_radar_objectives;
	if(control_structure->v2x->v2v[0].control_mode == EMERGENCY_BRAKING && my_pip > 1){
		desired_control_mode = EMERGENCY_BRAKING;
		control_structure->desired_control_mode = EMERGENCY_BRAKING;
		control_structure->current_control_mode = EMERGENCY_BRAKING;
		trigger_EB = true;
	}
	switch (desired_control_mode) {
		case(CRUISE):
			sProfile->GetReferenceSpeed(current_time);
			i_setpoint_speed = sProfile->outSpeed;
			if(with_hmi && test_scenario_id == -1 ){
				i_setpoint_speed = hmi_2_db_data->desired_cruise_speed;
			}
			control_structure->Update_Cruise_inputs(i_long_speed, i_setpoint_speed, current_time, i_long_acceleration);
			if((i_setpoint_speed < 0.00) && test_scenario_id == EMERGENCY_STOP){
				desired_control_mode = EMERGENCY_BRAKING;
				control_structure->desired_control_mode = EMERGENCY_BRAKING;
				control_structure->current_control_mode = EMERGENCY_BRAKING;
				trigger_EB = true;
			}
			break;
		case(ACC):
			if(v2i_engaged && my_pip == 1){
				control_structure->Update_ACC_inputs_V2I_engaged(i_long_speed, current_time);
				break;
			}
			switch (ego_vehicle_id) {
				case(PRIUS):
					get_prius_targets();
					nb_radar_objectives=1;
					break;
				case(LEAF):
					get_leaf_targets();
					nb_radar_objectives=1;
					break;
				case(CAMRY):
					get_camry_targets();
					nb_radar_objectives=1;
					break;
			}
			control_structure->Update_ACC_inputs(i_long_speed, i_long_acceleration, nb_radar_objectives, current_time);
			if(with_hmi){
				control_structure->user_time_gap_change_flag = ((control_structure->user_requested_time_gap != hmi_2_db_data->desired_time_gap) && !control_structure->user_time_gap_change_flag);
				control_structure->user_requested_time_gap = hmi_2_db_data->desired_time_gap;
				control_structure->performance_factor_from_user = hmi_2_db_data->performance_factor;
				control_structure->user_requested_control_mode = (v2i_engaged) ? desired_control_mode : hmi_2_db_data->desired_control_mode;
				control_structure->user_control_mode_change_flag = ((control_structure->user_requested_control_mode != hmi_2_db_data->desired_control_mode) && !control_structure->user_control_mode_change_flag);
				control_structure->user_requested_cruise_speed = (hmi_2_db_data->desired_cruise_speed < 0.01) ? config->DESIRED_CC_SPEED : hmi_2_db_data->desired_cruise_speed;
			}
			break;
		case(CACC):
			switch (ego_vehicle_id) {
				case(PRIUS):
					get_prius_targets();
					nb_radar_objectives = 1;
					break;
				case(CAMRY):
					get_camry_targets();
					nb_radar_objectives = 1;
					break;
				case(LEAF):
					get_leaf_targets();
					nb_radar_objectives = 1;
					break;

			}
			control_structure->Update_CACC_inputs(i_long_speed, i_long_acceleration, nb_radar_objectives, current_time);
			if(with_hmi){
				control_structure->user_time_gap_change_flag = ((control_structure->user_requested_time_gap != hmi_2_db_data->desired_time_gap) && !control_structure->user_time_gap_change_flag);
				control_structure->user_requested_time_gap = hmi_2_db_data->desired_time_gap;
				control_structure->performance_factor_from_user = hmi_2_db_data->performance_factor;
				control_structure->user_requested_cruise_speed = (hmi_2_db_data->desired_cruise_speed < 0.01) ? config->DESIRED_CC_SPEED : hmi_2_db_data->desired_cruise_speed;
				control_structure->user_requested_control_mode = (v2i_engaged) ? desired_control_mode : hmi_2_db_data->desired_control_mode;
				control_structure->user_control_mode_change_flag = ((control_structure->user_requested_control_mode != hmi_2_db_data->desired_control_mode) && !control_structure->user_control_mode_change_flag);
			}
			break;
		case(EMERGENCY_BRAKING):
			switch (ego_vehicle_id) {
				case(PRIUS):
					get_prius_targets();
					nb_radar_objectives = 1;
					break;
				case(CAMRY):
					get_camry_targets();
					nb_radar_objectives = 1;
					break;
				case(LEAF):
					get_leaf_targets();
					nb_radar_objectives = 1;
					break;
			}
			control_structure->Update_Emergency_braking_inputs(i_long_speed, nb_radar_objectives, current_time, trigger_EB);
			trigger_EB = false;
			break;
	}
	return true;
}

bool Process_data() {

	control_structure->Execute_control();

	switch (ego_vehicle_id) {
		case(PRIUS):
		case(LEAF):
		case(CAMRY):
			o_deceleration_command = control_structure->Get_deceleration_command();
			o_throttle_command = fmax(0, control_structure->desired_acceleration);
			break;
//		case(CAMRY):
//			o_deceleration_command = control_structure->Get_deceleration_command();
//			o_throttle_command = (o_deceleration_command > -0.00001) ? control_structure->Get_throttle_command() : 0.0000;
//			break;
	}

	return true;
}

bool Write_outputs() {
	output_t output;
	veh_comm_packet_t comm_pkt;
	timestamp_t ts;

	get_current_timestamp(&ts);

//	(ego_vehicle_id == ACCORD) ? output.accel_decel_request = o_brake_command : output.accel_decel_request = o_deceleration_command;
//	output.accel_decel_request = o_throttle_command;
	(ego_vehicle_id == ACCORD) ? output.brake_level = o_brake_command : output.brake_level = o_deceleration_command;
	output.throttle_pct = o_throttle_command;

	db_clt_write(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
	db_clt_read(pclt, DB_COMM_TX_VAR, sizeof(comm_pkt), &comm_pkt);

	comm_pkt.vel_traj = (float) (comm_verbosity) ? 10*(my_pip+0.5*sin(2*PI*0.5*current_time)) : fmax(0, control_structure->setpoint_speed);
	comm_pkt.velocity = (float) (comm_verbosity) ? 0.2*sin(2*PI*0.5*current_time) + my_pip : fmax(0,control_structure->long_speed);
	comm_pkt.accel = (float) i_long_acceleration;
	comm_pkt.acc_traj = (float) control_structure->desired_acceleration;
	comm_pkt.my_pip = (unsigned char) my_pip;
	comm_pkt.user_float1 = (float) fmax(0, control_structure->circuit_loc->get_x_coordinate());
	comm_pkt.maneuver_id = (unsigned char) control_structure->current_control_mode;

	db_clt_write(pclt, DB_COMM_TX_VAR, sizeof(comm_pkt), &comm_pkt);

	if(with_hmi)
		Write_read_hmi_variables_to_database();

	UpdateStandardLogFile();

	return true;
}

void End_tasks() {
	printf("\n End tasks \n");
	delete sProfile;
	delete control_structure;
	//WriteLogFile();
	return;
}

int get_prius_targets() {
	prius_radar_forward_vehicle_t prius_radar_forward_vehicle;
	int verbose = 0;

	db_clt_read(pclt,DB_PRIUS_MSG2E6_VAR, sizeof(prius_radar_forward_vehicle_t), &prius_radar_forward_vehicle);
	target[0].relative_distance = prius_radar_forward_vehicle.Radar_forward_veh_distance_CAN1__m;
	target[0].relative_speed = prius_radar_forward_vehicle.Radar_forward_veh_relative_spd_CAN1__mps;
	control_structure->targets[0].relative_distance = prius_radar_forward_vehicle.Radar_forward_veh_distance_CAN1__m;
	control_structure->targets[0].relative_speed = prius_radar_forward_vehicle.Radar_forward_veh_relative_spd_CAN1__mps;
	control_structure->targets[0].ID = 0;
	if(verbose)
		printf("\nget_prius_target: t=%.4f i %d 	DISTANCE %.4f 		REL SPEED %.4f\n",
				current_time,
				0,
				target[0].relative_distance,
				target[0].relative_speed
		);
	return 0;
}

int get_leaf_targets() {
	leaf_target_object_distance_t leaf_target_object_distance;
	leaf_target_relative_speed_mps_t leaf_target_relative_speed_mps;
	int verbose = 1;

	db_clt_read(pclt, DB_LEAF_OBD2MSG107_VAR, sizeof(leaf_target_object_distance_t), &leaf_target_object_distance);
	db_clt_read(pclt, DB_LEAF_OBD2MSG108_VAR, sizeof(leaf_target_relative_speed_mps_t), &leaf_target_relative_speed_mps);


	target[0].relative_distance = leaf_target_object_distance.object_distance_Radar;
	target[0].relative_speed = leaf_target_relative_speed_mps.object_relative_spd_Radar__mps;
	control_structure->targets[0].relative_distance = leaf_target_object_distance.object_distance_Radar;
	control_structure->targets[0].relative_speed = leaf_target_relative_speed_mps.object_relative_spd_Radar__mps;
	control_structure->targets[0].ID = 0;
	if(verbose)
		printf("\get_leaf_targets: t=%.4f i %d 	DISTANCE %.4f 		REL SPEED %.4f\n",
				current_time,
				0,
				target[0].relative_distance,
				target[0].relative_speed
		);
	return 0;
}


int get_camry_targets(){
	camry_prius_radar_forward_vehicle_t a;

	for(int i=0; i<1; i++){
	db_clt_read(pclt, DB_CAMRY_MSG680_VAR + i, sizeof(camry_prius_radar_forward_vehicle_t), &a);
	control_structure->targets[i].relative_distance = a.LONG_DIST_CAN1__m;
	control_structure->targets[i].relative_speed = a.LONG_SPEED_CAN1__mps;
	control_structure->targets[i].relative_position = a.LAT_DIST_CAN1__m;
	control_structure->targets[i].ID = i;
		if(camry_verbosity){
			printf("Camry target %d distance:%.5f | ",i, a.LONG_DIST_CAN1__m);
			printf("%d relative speed ", i);
			printf("%.5f ", a.LONG_SPEED_CAN1__mps);
			fprintf(fp_t, "%d d:%.5f | ",i, a.LONG_DIST_CAN1__m);
			fprintf(fp_t,"%d ", i);
			fprintf(fp_t,"%.5f ", a.LONG_SPEED_CAN1__mps);
		}


	if(camry_verbosity){
		fprintf(fp_t,"\n");
		printf("\n");
	}
	}
	return 1;
}

void Read_db_v2x_comm_data(){
	db_clt_read(pclt, DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt_Virtual_vehicle);
	db_clt_read(pclt, DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt1);
	db_clt_read(pclt, DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt2);
	db_clt_read(pclt, DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t), &comm_pkt3);

	v2i_sequence_number = (int) comm_pkt_Virtual_vehicle.sequence_no;
	control_structure->v2x->v2i.sequence_nb = v2i_sequence_number;
    control_structure->v2x->v2i.cruise_speed = comm_pkt_Virtual_vehicle.vel_traj;
    control_structure->v2x->v2i.relative_speed = comm_pkt_Virtual_vehicle.rate;
    control_structure->v2x->v2i.leader_speed = comm_pkt_Virtual_vehicle.velocity;
    control_structure->v2x->v2i.sequence_nb = comm_pkt_Virtual_vehicle.sequence_no;
    control_structure->v2x->v2i.distance_to_leader = comm_pkt_Virtual_vehicle.range;
    control_structure->v2x->v2i.relative_acceleration = comm_pkt_Virtual_vehicle.accel;
    control_structure->v2x->v2i.sync_flag = ((comm_pkt_Virtual_vehicle.user_ushort_1 == 1) ? true: false);
    control_structure->v2x->v2i.time_remaining = comm_pkt_Virtual_vehicle.user_float1;
    control_structure->v2x->v2i.dist_to_intersection = comm_pkt_Virtual_vehicle.user_float;

    control_structure->v2x->v2v[0].ref_speed = comm_pkt1.vel_traj;
    control_structure->v2x->v2v[0].long_speed = comm_pkt1.velocity;
    control_structure->v2x->v2v[0].sequence_nb = comm_pkt1.sequence_no;
    control_structure->v2x->v2v[0].pip = comm_pkt1.my_pip;
    control_structure->v2x->v2v[0].ref_acceleration = comm_pkt1.acc_traj;
    control_structure->v2x->v2v[0].ID = Check_vehicle_id(comm_pkt1.object_id);
    control_structure->v2x->v2v[0].control_mode = comm_pkt1.maneuver_id;
    control_structure->v2x->v2v[0].coordinate = comm_pkt1.user_float1;
	control_structure->v2x->v2v[0].acceleration = comm_pkt1.acc_traj;

    control_structure->v2x->v2v[1].ref_speed = comm_pkt2.vel_traj;
    control_structure->v2x->v2v[1].long_speed = comm_pkt2.velocity;
    control_structure->v2x->v2v[1].sequence_nb = comm_pkt2.sequence_no;
    control_structure->v2x->v2v[1].pip = comm_pkt2.my_pip;
    control_structure->v2x->v2v[1].ref_acceleration = comm_pkt2.acc_traj;
    control_structure->v2x->v2v[1].ID = Check_vehicle_id(comm_pkt2.object_id);
    control_structure->v2x->v2v[1].control_mode = comm_pkt2.maneuver_id;
    control_structure->v2x->v2v[1].coordinate = comm_pkt2.user_float1;
	control_structure->v2x->v2v[1].acceleration = comm_pkt2.acc_traj;

    control_structure->v2x->v2v[2].ref_speed = comm_pkt3.vel_traj;
    control_structure->v2x->v2v[2].long_speed = comm_pkt3.velocity;
    control_structure->v2x->v2v[2].sequence_nb = comm_pkt3.sequence_no;
    control_structure->v2x->v2v[2].pip = comm_pkt3.my_pip;
    control_structure->v2x->v2v[2].ref_acceleration = comm_pkt3.acc_traj;
    control_structure->v2x->v2v[2].ID = Check_vehicle_id(comm_pkt3.object_id);
    control_structure->v2x->v2v[2].control_mode = comm_pkt3.maneuver_id;
    control_structure->v2x->v2v[2].coordinate = comm_pkt3.user_float1;
	control_structure->v2x->v2v[2].acceleration = comm_pkt3.acc_traj;

    control_structure->v2x->v2v[my_pip-1].ref_speed = control_structure->setpoint_speed;
	control_structure->v2x->v2v[my_pip-1].long_speed = control_structure->long_speed;
	control_structure->v2x->v2v[my_pip-1].sequence_nb = 99;
	control_structure->v2x->v2v[my_pip-1].pip = my_pip;
	control_structure->v2x->v2v[my_pip-1].ref_acceleration = control_structure->desired_acceleration;
	control_structure->v2x->v2v[my_pip-1].ID = ego_vehicle_id;
	control_structure->v2x->v2v[my_pip-1].control_mode = control_structure->current_control_mode;
	control_structure->v2x->v2v[my_pip-1].coordinate = control_structure->circuit_loc->get_x_coordinate();
	control_structure->v2x->v2v[my_pip-1].acceleration = control_structure->long_acceleration;

    if(comm_verbosity)
    	printf("v1=%.4f  sqnb1=%d  v1_t=%.2f  v_ref_1=%.1f   pip1=%d ID1=%d  mode1=%d    |    v2=%.4f  sqnb2=%d  v2_t=%.2f  v_ref_2=%.1f pip2=%d ID2=%d  mode2=%d  |   v3=%.4f sqnb3=%d v3_t=%.2f  v_ref_3=%.1f pip3=%d ID3=%d  mode3=%d\n",
    				control_structure->v2x->v2v[0].long_speed,
					control_structure->v2x->v2v[0].sequence_nb,
					control_structure->v2x->v2v[0].v2v_fault_timer,
					control_structure->v2x->v2v[0].ref_speed,
					control_structure->v2x->v2v[0].pip,
					control_structure->v2x->v2v[0].ID,
					control_structure->v2x->v2v[0].control_mode,
					control_structure->v2x->v2v[1].long_speed,
					control_structure->v2x->v2v[1].sequence_nb,
					control_structure->v2x->v2v[1].v2v_fault_timer,
					control_structure->v2x->v2v[1].ref_speed,
					control_structure->v2x->v2v[1].pip,
					control_structure->v2x->v2v[1].ID,
					control_structure->v2x->v2v[1].control_mode,
					control_structure->v2x->v2v[2].long_speed,
					control_structure->v2x->v2v[2].sequence_nb,
					control_structure->v2x->v2v[2].v2v_fault_timer,
					control_structure->v2x->v2v[2].ref_speed,
					control_structure->v2x->v2v[2].pip,
					control_structure->v2x->v2v[2].ID,
					control_structure->v2x->v2v[2].control_mode);

	return;
}

void InitOutputFile(){
	time_t rawtime;
	struct tm * timeinfo;
	time_t t;
	time(&t);
	struct tm tm = *localtime(&t);
	char file_name[200]={0};
	time( &rawtime );
	timeinfo = localtime( &rawtime );

	sprintf(file_name,"/home/qnxuser/leaf_binaries_and_scripts/path_can_bin/data/control_test_%s_log_%02d%02d%d__%02d_%02d_%02d.dat",VEHICLE_NAMES[ego_vehicle_id], tm.tm_mon+1,tm.tm_mday,tm.tm_year+1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

	fp = fopen(file_name, "w");
	usleep(100000);
	if(ego_vehicle_id == CAMRY && camry_verbosity){
		char file_name_t[200]={0};
		printf("Trying to create file \n");
		sprintf(file_name_t,"/home/qnxuser/leaf_binaries_and_scripts/path_can_bin/data/targets%02d%02d%d__%02d_%02d_%02d.dat", tm.tm_mon+1,tm.tm_mday,tm.tm_year+1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);//%s_targets_%02d%02d%d__%02d_%02d_%02d.dat",VEHICLE_NAMES[ego_vehicle_id], tm.tm_mon+1,tm.tm_mday,tm.tm_year+1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
		printf("\nFile name %s \n", file_name_t);
		fp_t = fopen(file_name_t, "w");
		if (fp_t == NULL) {
			printf("Open camry targets file failed \n");
		} else{
			printf("\ncamry file created\n");
		}
	}

	if (fp == NULL) {
		printf("Open output file fail");
		return;
	} else{
		printf("\nFile created\n");
		return;
	}

}

void UpdateStandardLogFile(){
	time_t rawtime;
	struct tm * timeinfo;
	time_t t;
	time(&t);
	struct tm tm = *localtime(&t);

	time( &rawtime );
	timeinfo = localtime( &rawtime );
	timestamp_t ts;
	get_current_timestamp(&ts);
	fprintf(fp,"%02d", tm.tm_mon+1);//1
	fprintf(fp," %02d", tm.tm_mday);
	fprintf(fp," %02d", tm.tm_year+1900);
	fprintf(fp," %02d", ts.hour);
	fprintf(fp," %02d", ts.min);			//5

	fprintf(fp," %02d", ts.sec);
	fprintf(fp," %03d", ts.millisec);
	fprintf(fp," %.6f", current_time);
	if(manual_control_requested)
		fprintf(fp," %d", 1);				//9
	else{
		switch(control_structure->current_control_mode){
			case(CRUISE):
				fprintf(fp," %d", 2);		//9
				break;
			case(ACC):
				fprintf(fp," %d", 3);		//9
				break;
			case(CACC):
				fprintf(fp," %d", 4);		//9
				break;
			case(ACC_2_CRUISE):
				fprintf(fp," %d", 2);		//9
				break;
			case(CRUISE_2_ACC):
				fprintf(fp," %d", 3);		//9
				break;
			case(CACC_2_ACC):
				fprintf(fp," %d", 3);		//9
				break;
			case(ACC_2_CACC):
				fprintf(fp," %d", 4);		//9
				break;
			case(EMERGENCY_BRAKING):
				fprintf(fp," %d", 0);		//9
				break;
		}
	}
	fprintf(fp," %d",   v2i_engaged);		//10

	fprintf(fp," %d", control_structure->current_control_mode);  // 11
	fprintf(fp," %d", control_structure->my_pip);
	fprintf(fp," %d", ego_vehicle_id);
	fprintf(fp," %.3f", control_structure->long_speed);
	fprintf(fp," %.3f", i_fuel_rate);		//15

	fprintf(fp," %.4f", control_structure->long_acceleration);
	fprintf(fp," %.4f", gps_data.latitude);
	fprintf(fp," %.4f", gps_data.longitude);
	fprintf(fp," %.4f", gps_data.altitude);
	fprintf(fp," %d",   control_structure->is_preceding_valid ? 1 : 0); //20

	fprintf(fp," %.3f", control_structure->preceding_vehicle.relative_distance); // 21
	fprintf(fp," %.3f", control_structure->preceding_vehicle.relative_speed);
	fprintf(fp," %.3f", control_structure->preceding_vehicle.relative_position);
	fprintf(fp," %.3f", control_structure->throttle_level_command);
	fprintf(fp," %.3f", control_structure->brake_level_command);

	fprintf(fp," %.3f", control_structure->deceleration_command);
	fprintf(fp," %.3f", control_structure->setpoint_speed);
	fprintf(fp," %.3f", control_structure->desired_acceleration);
	fprintf(fp," %.3f", control_structure->desired_distance_gap);
	fprintf(fp," %.3f", control_structure->ACC_time_gap);

	fprintf(fp," %.3f", control_structure->ACC_gap_error);  // 31
	fprintf(fp," %.3f", control_structure->ACC_fb_out);
	fprintf(fp," %.3f", control_structure->ACC_ref_speed);
	fprintf(fp," %.3f", control_structure->v2v_fault_timer);
	fprintf(fp," %.3f", control_structure->cooperative_ACC_time_gap);

	fprintf(fp," %.3f", control_structure->CACC_gap_error);
	fprintf(fp," %.3f", control_structure->cooperative_ACC_fb_out);
	fprintf(fp," %.3f", control_structure->cooperative_ACC_ref_speed);
	fprintf(fp," %.3f", control_structure->preceding_ref_speed);
	fprintf(fp," %.3f", control_structure->leader_ref_speed);

	fprintf(fp," %d", control_structure->is_LPF_active ? 1 : 0);  // 41
	fprintf(fp," %.3f", control_structure->CACC_ff_out);
	fprintf(fp," %d", control_structure->v2x->v2v[0].pip);
	fprintf(fp," %d", control_structure->v2x->v2v[0].ID);
	fprintf(fp," %.3f", control_structure->v2x->v2v[0].ref_speed);

	fprintf(fp," %.3f", control_structure->v2x->v2v[0].ref_acceleration);
	fprintf(fp," %.3f", control_structure->v2x->v2v[0].long_speed);
	fprintf(fp," %.3f", control_structure->v2x->v2v[0].acceleration);
	fprintf(fp," %.3f", control_structure->v2x->v2v[0].coordinate);
	fprintf(fp," %d", control_structure->v2x->v2v[0].control_mode);

	fprintf(fp," %d", control_structure->v2x->v2v[1].pip);  // 51
	fprintf(fp," %d", control_structure->v2x->v2v[1].ID);
	fprintf(fp," %.3f", control_structure->v2x->v2v[1].ref_speed);
	fprintf(fp," %.3f", control_structure->v2x->v2v[1].ref_acceleration);
	fprintf(fp," %.3f", control_structure->v2x->v2v[1].long_speed);

	fprintf(fp," %.3f", control_structure->v2x->v2v[1].acceleration);
	fprintf(fp," %.3f", control_structure->v2x->v2v[1].coordinate);
	fprintf(fp," %d", control_structure->v2x->v2v[1].control_mode);
	fprintf(fp," %d", control_structure->v2x->v2v[2].pip);
	fprintf(fp," %d", control_structure->v2x->v2v[2].ID);

	fprintf(fp," %.3f", control_structure->v2x->v2v[2].ref_speed);  // 61
	fprintf(fp," %.3f", control_structure->v2x->v2v[2].ref_acceleration);
	fprintf(fp," %.3f", control_structure->v2x->v2v[2].long_speed);
	fprintf(fp," %.3f", control_structure->v2x->v2v[2].acceleration);
	fprintf(fp," %.3f", control_structure->v2x->v2v[2].coordinate);

	fprintf(fp," %d", control_structure->v2x->v2v[2].control_mode);
	fprintf(fp," %.2f", control_structure->v2i_fault_timer);
	fprintf(fp," %d"  , control_structure->virtual_leader_info.sync_flag ? 1 : 0);
	fprintf(fp," %.3f", control_structure->circuit_loc->x_coordinate);
	fprintf(fp," %.3f", control_structure->virtual_leader_info.leader_x_coordinate);

	fprintf(fp," %.3f", control_structure->virtual_leader_info.relative_speed);  // 71
	fprintf(fp," %d", control_structure->circuit_loc->trajectory_planning_engaged ? 1 : 0);
	fprintf(fp," %.3f", control_structure->circuit_loc->time_estimated);
	fprintf(fp," %.3f", control_structure->circuit_loc->time_remaining);
	fprintf(fp," %.3f", control_structure->circuit_loc->ref_accel_traj_plan);

	fprintf(fp, "\n");
}

bool Load_configuration_file(char* file_name){
	FILE* fp = fopen(file_name, "r");
	if (fp == NULL) {
		printf("Could not read the file");
		return false;
	}
	while (!feof(fp)) {
		char name[100];
		float value;
		while (fscanf(fp, "%s %f\n", name, &value) != EOF) {
			if (!Fill_config_structure(config, name, value)) {
				return false;
			}
		}
		fclose(fp);
	}
	return true;
}

int Check_vehicle_id(char *name){
	if(!strcmp(VEHICLE_NAMES[0], name)){
		return 0;
	}
	if(!strcmp(VEHICLE_NAMES[1], name)){
		return 1;
	}
	if(!strcmp(VEHICLE_NAMES[2], name)){
		return 2;
	}
	return -1;
}

bool Fill_config_structure(setup_struct* ss, char* name, float value){
	printf("Reading %s = %.2f\n", name, value);
	if (!strcmp(name, "SAMPLING_TIME")) {
		ss->SAMPLING_TIME = value;
		return true;
	}
	if (!strcmp(name, "SPEED_TRACKING_P_GAIN")) {
		ss->SPEED_TRACKING_P_GAIN = value;
		return true;
	}
	if (!strcmp(name, "SPEED_TRACKING_P_GAIN_PERF")) {
		ss->SPEED_TRACKING_P_GAIN = value;
		return true;
	}
	if (!strcmp(name, "ACC_KP_COMFORT")) {
		ss->ACC_KP_COMFORT = value;
		return true;
	}
	if (!strcmp(name, "ACC_KD_COMFORT")) {
		ss->ACC_KD_COMFORT =  value;
		return true;
	}
	if (!strcmp(name, "ACC_KP_PERFORMANCE")) {
		ss->ACC_KP_PERFORMANCE =  value;
		return true;
	}
	if (!strcmp(name, "ACC_KD_PERFORMANCE")) {
		ss->ACC_KD_PERFORMANCE = value;
		return true;
	}
	if (!strcmp(name, "HIGHWAY_CACC_KP_COMFORT")) {
		ss->HIGHWAY_CACC_KP_COMFORT =  value;
		return true;
	}
	if (!strcmp(name, "HIGHWAY_CACC_KD_COMFORT")) {
		ss->HIGHWAY_CACC_KD_COMFORT =  value;
		return true;
	}
	if (!strcmp(name, "HIGHWAY_CACC_KP_PERFORMANCE")) {
		ss->HIGHWAY_CACC_KP_PERFORMANCE =  value;
		return true;
	}
	if (!strcmp(name, "HIGHWAY_CACC_KD_PERFORMANCE")) {
		ss->HIGHWAY_CACC_KD_PERFORMANCE = value;
		return true;
	}
	if (!strcmp(name, "TRACK_CACC_KP_COMFORT")) {
		ss->TRACK_CACC_KP_COMFORT =  value;
		return true;
	}
	if (!strcmp(name, "TRACK_CACC_KD_COMFORT")) {
		ss->TRACK_CACC_KD_COMFORT =  value;
		return true;
	}
	if (!strcmp(name, "TRACK_CACC_KP_PERFORMANCE")) {
		ss->TRACK_CACC_KP_PERFORMANCE =  value;
		return true;
	}
	if (!strcmp(name, "TRACK_CACC_KD_PERFORMANCE")) {
		ss->TRACK_CACC_KD_PERFORMANCE = value;
		return true;
	}
	if (!strcmp(name, "RELATIVE_SPEED_GAIN")) {
		ss->RELATIVE_SPEED_GAIN =  value;
		return true;
	}
	if (!strcmp(name, "MAXIMUM_ACC_TIME_GAP_HORIZON")) {
		ss->MAXIMUM_ACC_TIME_GAP_HORIZON =  value;
		return true;
	}
	if (!strcmp(name, "MINIMUM_ACC_TIME_GAP")) {
		ss->MINIMUM_ACC_TIME_GAP = value;
		return true;
	}
	if (!strcmp(name, "MAXIMUM_CACC_TIME_GAP_HORIZON")) {
		ss->MAXIMUM_CACC_TIME_GAP_HORIZON =  value;
		return true;
	}
	if (!strcmp(name, "MINIMUM_CACC_TIME_GAP")) {
		ss->MINIMUM_CACC_TIME_GAP = value;
		return true;
	}
	if (!strcmp(name, "ACC_TARGET_TIME_GAP")) {
		ss->ACC_TARGET_TIME_GAP = value;
		return true;
	}
	if (!strcmp(name, "CACC_TARGET_TIME_GAP")) {
		ss->CACC_TARGET_TIME_GAP = value;
		return true;
	}
	if (!strcmp(name, "STANDSTILL_DISTANCE")) {
		ss->STANDSTILL_DISTANCE = value;
		return true;
	}
	if (!strcmp(name, "ACC_ERROR_FILTER_FREQ")) {
		ss->ACC_ERROR_FILTER_FREQ = value;
		return true;
	}
	if (!strcmp(name, "CACC_ERROR_FILTER_FREQ")) {
		ss->CACC_ERROR_FILTER_FREQ = value;
		return true;
	}
	if (!strcmp(name, "PERF_CACC_ERROR_FILTER_FREQ")) {
		ss->PERF_CACC_ERROR_FILTER_FREQ = value;
		return true;
	}
	if (!strcmp(name, "CUT_IN_A_MAX")) {
		ss->CUT_IN_A_MAX =  value;
		return true;
	}
	if (!strcmp(name, "CUT_IN_J_MAX")) {
		ss->CUT_IN_J_MAX = value;
		return true;
	}
	if (!strcmp(name, "CUT_OUT_A_MAX")) {
		ss->CUT_OUT_A_MAX =  value;
		return true;
	}
	if (!strcmp(name, "CUT_OUT_J_MAX")) {
		ss->CUT_OUT_J_MAX =  value;
		return true;
	}
	if (!strcmp(name, "TOP_ACCELERATION_LOW_SPEED")) {
		ss->TOP_ACCELERATION_LOW_SPEED = value;
		return true;
	}
	if (!strcmp(name, "TOP_ACCELERATION_V2I")) {
		ss->TOP_ACCELERATION_V2I =  value;
		return true;
	}
	if (!strcmp(name, "TOP_ACCELERATION_HIGH_SPEED")) {
		ss->TOP_ACCELERATION_HIGH_SPEED = value;
		return true;
	}
	if (!strcmp(name, "TOP_DECELERATION")) {
		ss->TOP_DECELERATION = value;
		return true;
	}
	if (!strcmp(name, "DESIRED_CC_SPEED")) {
		ss->DESIRED_CC_SPEED = value;
		return true;
	}
	if (!strcmp(name, "CUT_IN_DETECTION_TIMER")) {
		ss->CUT_DETECTION_TIMER = value;
		return true;
	}
	if (!strcmp(name, "MAXIMUM_SPEED")) {
		ss->MAXIMUM_SPEED = value;
		return true;
	}
	if (!strcmp(name, "MINIMUM_SPEED")) {
		ss->MINIMUM_SPEED = value;
		return true;
	}
	if (!strcmp(name, "GAIN_REDUCTION_AT_MAX_TIME_GAP")) {
		ss->GAIN_REDUCTION_AT_MAX_TIME_GAP = value;
		return true;
	}
	if (!strcmp(name, "MAX_V2V_MISSING_TIME_TOLERANCE")) {
		ss->MAX_V2V_MISSING_TIME_TOLERANCE = value;
		return true;
	}
	if (!strcmp(name, "KP_VIRTUAL_FOLLOWING_LOW")) {
		ss->KP_VIRTUAL_FOLLOWING_LOW = value;
		return true;
	}
	if (!strcmp(name, "KD_VIRTUAL_FOLLOWING_LOW")) {
		ss->KD_VIRTUAL_FOLLOWING_LOW = value;
		return true;
	}
	if (!strcmp(name, "KP_VIRTUAL_FOLLOWING_HIGH")) {
		ss->KP_VIRTUAL_FOLLOWING_HIGH = value;
		return true;
	}
	if (!strcmp(name, "KD_VIRTUAL_FOLLOWING_HIGH")) {
		ss->KD_VIRTUAL_FOLLOWING_HIGH = value;
		return true;
	}
	if (!strcmp(name, "TRANSITION_DECEL_CRUISE_2_ACC")) {
		ss->TRANSITION_DECEL_CRUISE_2_ACC = value;
		return true;
	}
	if (!strcmp(name, "TRANSITION_ACCEL_ACC_2_CRUISE")) {
		ss->TRANSITION_ACCEL_ACC_2_CRUISE = value;
		return true;
	}
	if (!strcmp(name, "TRANSITION_ACCEL_ACC_2_CACC")) {
		ss->TRANSITION_ACCEL_ACC_2_CACC = value;
		return true;
	}
	if (!strcmp(name, "TRANSITION_ACCEL_CACC_2_ACC")) {
		ss->TRANSITION_ACCEL_CACC_2_ACC = value;
		return true;
	}
	if (!strcmp(name, "LATERAL_LANE_TARGET_SPACE")) {
		ss->LATERAL_LANE_TARGET_SPACE = value;
		return true;
	}
	if (!strcmp(name, "LATERAL_LANE_TARGET_SPACE")) {
		ss->LATERAL_LANE_TARGET_SPACE = value;
		return true;
	}
	if (!strcmp(name, "MAX_MISSING_TARGET_TIME")) {
		ss->MAX_MISSING_TARGET_TIME = value;
		return true;
	}
	if (!strcmp(name, "MONITORING_FREQUENCY")) {
		ss->MONITORING_FREQUENCY = value;
		return true;
	}
	if (!strcmp(name, "LPF_EGO_LEADER_FF_GAIN")) {
		ss->LPF_EGO_LEADER_FF_GAIN = value;
		return true;
	}
	if (!strcmp(name, "LPF_EGO_PRECEDING_FF_GAIN")) {
		ss->LPF_EGO_PRECEDING_FF_GAIN = value;
		return true;
	}
	if (!strcmp(name, "H_MIN_FR_POLICY")) {
		ss->H_MIN_FR_POLICY = value;
		return true;
	}
	if (!strcmp(name, "H_TARGET_FR_POLICY")) {
		ss->H_TARGET_FR_POLICY = value;
		return true;
	}
	if (!strcmp(name, "A_MAX_FR_POLICY")) {
		ss->A_MAX_FR_POLICY = value;
		return true;
	}
	if (!strcmp(name, "J_MAX_FR_POLICY")) {
		ss->J_MAX_FR_POLICY = value;
		return true;
	}
	if (!strcmp(name, "A_MAX_SPEED_PROFILE")) {
		ss->A_MAX_SPEED_PROFILE = value;
		return true;
	}
	if (!strcmp(name, "D_MAX_SPEED_PROFILE")) {
		ss->D_MAX_SPEED_PROFILE = value;
		return true;
	}
	if (!strcmp(name, "V_MAX_SPEED_PROFILE")) {
		ss->V_MAX_SPEED_PROFILE = value;
		return true;
	}
	if (!strcmp(name, "V_MIN_SPEED_PROFILE")) {
		ss->V_MIN_SPEED_PROFILE = value;
		return true;
	}
	if (!strcmp(name, "E_BRAKING_STOPPING_DISTANCE")) {
		ss->E_BRAKING_STOPPING_DISTANCE = value;
		return true;
	}
	if (!strcmp(name, "MANEUVER_TRIGGER_TIME")) {
		ss->MANEUVER_TRIGGER_TIME = value;
		return true;
	}
	if (!strcmp(name, "FULL_GAP_ERROR_FILTER_FLAG")) {
		ss->FULL_GAP_ERROR_FILTER_FLAG = value;
		return true;
	}
	if (!strcmp(name, "CACC_FF_EGO_PREC_FILTER_FREQ")) {
		ss->CACC_FF_EGO_PREC_FILTER_FREQ = value;
		return true;
	}
	if (!strcmp(name, "CACC_FF_EGO_LEADER_FILTER_FREQ")) {
		ss->CACC_FF_EGO_LEADER_FILTER_FREQ = value;
		return true;
	}
	if (!strcmp(name, "DEFAULT_PERFORMANCE_FACTOR")) {
		ss->DEFAULT_PERFORMANCE_FACTOR = value;
		return true;
	}
	if (!strcmp(name, "SWEEP_FREQUENCY_1")) {
		ss->SWEEP_FREQUENCY_1 = value;
		return true;
	}
	if (!strcmp(name, "SWEEP_FREQUENCY_2")) {
		ss->SWEEP_FREQUENCY_2 = value;
		return true;
	}
	if (!strcmp(name, "SWEEP_AMPLITUDE")) {
		ss->SWEEP_AMPLITUDE = value;
		return true;
	}
	if (!strcmp(name, "SWEEP_DURATION")) {
		ss->SWEEP_DURATION = value;
		return true;
	}
	if (!strcmp(name, "TRAJECTORY_PLANNING_GAIN")) {
		ss->TRAJECTORY_PLANNING_GAIN = value;
		return true;
	}
	if (!strcmp(name, "TRAJECTORY_PLANNING_BUFFER_DISTANCE")) {
		ss->TRAJECTORY_PLANNING_BUFFER_DISTANCE = value;
		return true;
	}
	if (!strcmp(name, "LEADER_EMERGENCY_BRAKING_GAIN")) {
		ss->LEADER_EMERGENCY_BRAKING_GAIN = value;
		return true;
	}
	if (!strcmp(name, "PRIUS_DAMPING_FACTOR")) {
		ss->PRIUS_DAMPING_FACTOR = value;
		return true;
	}
	if (!strcmp(name, "PRIUS_BANDWIDTH")) {
		ss->PRIUS_BANDWIDTH = value;
		return true;
	}
	if (!strcmp(name, "LEAF_BANDWIDTH")) {
		ss->LEAF_BANDWIDTH = value;
		return true;
	}
	if (!strcmp(name, "LEAF_DAMPING_FACTOR")) {
		ss->LEAF_DAMPING_FACTOR = value;
		return true;
	}
	if (!strcmp(name, "CAMRY_DAMPING_FACTOR")) {
		ss->CAMRY_DAMPING_FACTOR = value;
		return true;
	}
	if (!strcmp(name, "CAMRY_BANDWIDTH")) {
		ss->CAMRY_BANDWIDTH = value;
		return true;
	}
	return false;
}


void Write_read_hmi_variables_to_database(){

	db_2_hmi_data->my_pip = my_pip;
	db_2_hmi_data->current_control_mode = control_structure->current_control_mode;
	db_2_hmi_data->ego_speed = control_structure->long_speed;
	db_2_hmi_data->measured_time_gap = ((control_structure->preceding_vehicle.relative_distance - config->STANDSTILL_DISTANCE) / fmax(0.01, control_structure->long_speed));
	db_2_hmi_data->ACC_set_time_gap = control_structure->ACC_time_gap;
	db_2_hmi_data->CACC_set_time_gap = control_structure->cooperative_ACC_time_gap;
	db_2_hmi_data->target_vehicle_valid = control_structure->is_preceding_valid ? 1 : 0;
	db_2_hmi_data->veh_1_speed = control_structure->v2x->v2v[0].long_speed;
	db_2_hmi_data->veh_2_speed = control_structure->v2x->v2v[1].long_speed;
	db_2_hmi_data->veh_3_speed = control_structure->v2x->v2v[2].long_speed;
	db_2_hmi_data->v2v_available = (control_structure->v2v_fault_timer > config->MAX_V2V_MISSING_TIME_TOLERANCE) ? 0 : 1;
	db_2_hmi_data->timestamp = current_time;

	printf("Write_read_hmi_variables_to_database: db_2_hmi_data->ego_speed %.2f control_structure->long_speed %.2f current_time %.2f\n ",
			db_2_hmi_data->ego_speed,
			control_structure->long_speed,
			db_2_hmi_data->timestamp
			);
	db_clt_write(pclt, DB_2_HMI_DATA, sizeof(tx_data), db_2_hmi_data);
	db_clt_read(pclt, HMI_2_DB_DATA, sizeof(rx_data), hmi_2_db_data);

	return;

}

void Init_db_variables_from_hmi(){
	hmi_2_db_data->desired_control_mode = desired_control_mode;
	hmi_2_db_data->desired_time_gap = config->ACC_TARGET_TIME_GAP;
	hmi_2_db_data->gap_choice = 0;
	hmi_2_db_data->desired_cruise_speed = config->DESIRED_CC_SPEED;
	hmi_2_db_data->performance_factor = 0.;

	db_clt_write(pclt, HMI_2_DB_DATA, sizeof(rx_data), hmi_2_db_data);
}
