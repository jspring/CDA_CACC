#include "db_include.h"
#include "leaf_can.h"
#include "clt_vars_leaf.h"
#include <can_dbvars.h>

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

const char *usage = "-v verbose -a <acceleration> -b <braking pct>";

int steinhoff_trig_list[] =  {
        DB_STEINHOFF_MSG_VAR,
		DB_OUTPUT_VAR,
		DB_STEINHOFF_OBD2_IN_3003_VAR
};

int num_steinhoff_trig_variables = sizeof(steinhoff_trig_list)/sizeof(int);

int main(int argc, char *argv[]) {
	int verbose = 0;
	int veryverbose = 0;
	int option;
	int count = 0;
	float acceleration = 0;
	short torque = 0;

	char hostname[MAXHOSTNAMELEN];
    char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
    db_clt_typ *pclt;               /// data server client pointer
    int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
	posix_timer_typ *ptimer;
	int ipc_message_error_ctr = 0;
	trig_info_typ trig_info;

	db_steinhoff_msg_t db_steinhoff_msg;
	db_steinhoff_out_t db_steinhoff_torque_out;
	db_steinhoff_out_t db_steinhoff_accel_out;
	db_steinhoff_out_t db_steinhoff_obd2;

	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms;
	int ts_now;
	int ts_sav;
	char obd2_ctr = 0;
	unsigned int obd2ID;
	int toggle_flag = 0;

	leaf_accel_cmd_t leaf_accel_cmd;
	leaf_torque_cmd_t leaf_torque_cmd;
	leaf_vehicle_speed_t leaf_vehicle_speed;
	leaf_torque_t  leaf_torque;
	leaf_steering_t leaf_steering;
	leaf_fuel_rate_t leaf_fuel_rate;
	leaf_target_object_distance_t leaf_target_object_distance;
	leaf_target_relative_speed_mps_t leaf_target_relative_speed_mps;
	veh_comm_packet_t virtual_car_comm_packet;
	leaf_Veh_Accel_CAN4_t leaf_Veh_Accel_CAN4;
	leaf_Torq_brake_ACC_t leaf_Torq_brake_ACC;
	int use_98_command = 0;
	int send_tx_commands = 1;

        while ((option = getopt(argc, argv, "vwa:t:T")) != EOF) {
                switch(option) {
                case 'v':
                        verbose = 1;
                        break;
                case 'w':
                        verbose = 1;
                        veryverbose = 1;
                        break;
                case 'a':
                        acceleration = atof(optarg);
                        break;
                case 'T':
                        use_98_command = 1;
                        break;
                case 't':
                        torque = (short)atoi(optarg);
                        printf("torque1: %d\n", torque);
                        break;
                default:
                        printf("Usage: %s %s\n", argv[0], usage);
                        exit(EXIT_FAILURE);
                        break;
                }
        }

        printf("Got to 1\n");

	get_local_name(hostname, MAXHOSTNAMELEN);
	if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
			NULL, 0, NULL, 0))
		== NULL) {
			exit(EXIT_FAILURE);
	}
    printf("Got to 1.5\n");

	if (clt_trig_set( pclt, DB_STEINHOFF_MSG_VAR, DB_STEINHOFF_MSG_TYPE) == FALSE )
		exit(EXIT_FAILURE);
//	if (clt_trig_set( pclt, DB_STEINHOFF_CHASSIS_3004_VAR, DB_STEINHOFF_CHASSIS_3004_TYPE) == FALSE )
//		exit(EXIT_FAILURE);

printf("leaf_can: clt_trig_set OK for DB_STEINHOFF_MSG_VAR %d\n", DB_STEINHOFF_MSG_VAR);
        if (setjmp(exit_env) != 0) {
//        	leaf_torque_cmd.torque_cmd = 0;
//			db_steinhoff_torque_out.port = BRAKE_PORT;
//			db_steinhoff_torque_out.id = 0x98;
//			db_steinhoff_torque_out.size = 2;
//			set_leaf_torque_cmd(db_steinhoff_torque_out.data, &leaf_torque_cmd);
//			db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_torque_out);

//			sleep(0.02);

//			leaf_accel_cmd.accel_cmd = 0;
//			db_steinhoff_accel_out.port = ACCEL_PORT;
//			db_steinhoff_accel_out.id = 0x99;
//			db_steinhoff_accel_out.size = 2;
//			set_leaf_accel_cmd(db_steinhoff_accel_out.data, &leaf_accel_cmd);

        	db_list_done(pclt, NULL, 0, steinhoff_trig_list, num_steinhoff_trig_variables);
                printf("%s: received %d CAN messages %d IPC message errors\n", 
			argv[0], count, ipc_message_error_ctr);
                exit(EXIT_SUCCESS);
        } else
               sig_ign(sig_list, sig_hand);

        if ((ptimer = timer_init( 20, ChannelCreate(0) )) == NULL) {
                fprintf(stderr, "Unable to initialize delay timer\n");
                exit(EXIT_FAILURE);
        }

	/* Zero data structures */
	memset(&leaf_accel_cmd, 0, sizeof(leaf_accel_cmd));
	memset(&leaf_torque_cmd, 0, sizeof(leaf_torque_cmd));
	memset(&leaf_vehicle_speed, 0, sizeof(leaf_vehicle_speed_t));
	memset(&leaf_torque, 0, sizeof(leaf_torque_t));
	memset(&leaf_steering, 0, sizeof(leaf_steering_t));
	memset(&leaf_Torq_brake_ACC, 0, sizeof(leaf_Torq_brake_ACC_t));

	leaf_accel_cmd.two_message_periods = 40; 		// 2*20 msec
	leaf_torque_cmd.two_message_periods = 40; 		// 2*20 msec
	leaf_vehicle_speed.two_message_periods = 80;
	leaf_torque.two_message_periods = 80;
	leaf_steering.two_message_periods = 80;
	leaf_Torq_brake_ACC.two_message_periods = 80;

	db_clt_write(pclt, DB_LEAF_MSG99_VAR, sizeof(leaf_accel_cmd_t), &leaf_accel_cmd);
	db_clt_write(pclt, DB_LEAF_MSG98_VAR, sizeof(leaf_torque_cmd_t), &leaf_torque_cmd);
	db_clt_write(pclt, DB_LEAF_MSG292_VAR, sizeof(leaf_vehicle_speed_t), &leaf_Veh_Accel_CAN4);
	db_clt_write(pclt, DB_LEAF_MSG1DA_VAR, sizeof(leaf_torque_t), &leaf_torque);
	db_clt_write(pclt, DB_LEAF_MSG2_VAR, sizeof(leaf_steering_t), &leaf_steering);
	db_clt_write(pclt, DB_LEAF_MSG2B0_VAR, sizeof(leaf_Torq_brake_ACC_t), &leaf_Torq_brake_ACC);

	if(acceleration == 0)
		leaf_accel_cmd.accel_cmd = 0;
	if(torque == 0)
		leaf_torque_cmd.torque_cmd = 0;
	get_current_timestamp(&ts);
	ts_now = TS_TO_MS(&ts);
	ts_sav = ts_now;
    printf("Got to 2\n");

	for(;;) {

//NOTE: PC104 CAN1=ADAS CAN2=EV BUS CAN3=OBD2 CAN4=CHASSIS

		clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
		if( DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_MSG_VAR ) {
			count++;
			db_clt_read(pclt, DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);
//			printf("DB_STEINHOFF_MSG_VAR: ID %#x ", db_steinhoff_msg.id);
//			for(i=0; i< db_steinhoff_msg.size;i++)
//				printf("data[%d]: %#hhx ", i, data[i]);
//			printf("\n");

			switch(db_steinhoff_msg.id) {
				case 0x355:
					get_leaf_vehicle_speed(db_steinhoff_msg.data, &leaf_vehicle_speed);
					check_msg_timeout(ts_ms, &leaf_vehicle_speed.ts_ms, 
						&leaf_vehicle_speed.two_message_periods, 
						&leaf_vehicle_speed.message_timeout_counter); 
					input.vehicle_speed_mps = leaf_vehicle_speed.veh_speed1_CAN6__mps;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);

					if(veryverbose)
						printf("Leaf input.vehicle speed %.2f vehicle speed1 %.2f vehicle speed2 %.2f output.torque_level %.2f output.accel_decel_request %.2f mph data[0] %#hhx data[1] %#hhx data[2] %#hhx data[3] %#hhx\n",
								input.vehicle_speed_mps,
								leaf_vehicle_speed.veh_speed1_CAN6__mps,
								leaf_vehicle_speed.veh_speed2_CAN6__mps,
								output.torque_level,
								output.accel_decel_request,
								db_steinhoff_msg.data[0],
								db_steinhoff_msg.data[1],
								db_steinhoff_msg.data[2],
								db_steinhoff_msg.data[3]
						);
					break;
				case 0x1DA:
					get_leaf_torque(db_steinhoff_msg.data, &leaf_torque);
					check_msg_timeout(ts_ms, &leaf_torque.ts_ms, 
						&leaf_torque.two_message_periods, 
						&leaf_torque.message_timeout_counter); 
					input.torque = leaf_torque.Motor_Torque_CAN5__Nm;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					if(veryverbose)
						printf("Leaf motor torque %.3f\n",
							leaf_torque.Motor_Torque_CAN5__Nm
						);
					break;
				case 0x2:
					get_leaf_steering(db_steinhoff_msg.data, &leaf_steering);
					check_msg_timeout(ts_ms, &leaf_steering.ts_ms, 
						&leaf_steering.two_message_periods, 
						&leaf_steering.message_timeout_counter); 
					input.steering_angle_deg = leaf_steering.SteeringAngle;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					if(veryverbose)
						printf("Leaf steering position %.2f rate %.2f\n",
							leaf_steering.SteeringAngle,
							leaf_steering.SteeringAngleChangeRate
						);
					break;
				case 0x292:
					get_leaf_accel(db_steinhoff_msg.data, &leaf_Veh_Accel_CAN4);
					check_msg_timeout(ts_ms, &leaf_Veh_Accel_CAN4.ts_ms,
						&leaf_Veh_Accel_CAN4.two_message_periods,
						&leaf_Veh_Accel_CAN4.message_timeout_counter);
					input.vehicle_accel_mps2 = G2MPS2 * leaf_Veh_Accel_CAN4.Long_Accel_CAN4__G;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					if(veryverbose)
						printf("leaf_can.c: Long_Accel_CAN4__G %.3f Lateral_Accel_CAN4__mps2 %.3f Yaw_Rate_CAN4__degps %.3f Veh_brake_press_CAN4__bar %d\n",
								leaf_Veh_Accel_CAN4.Long_Accel_CAN4__G,
								leaf_Veh_Accel_CAN4.Lateral_Accel_CAN4__mps2,
								leaf_Veh_Accel_CAN4.Yaw_Rate_CAN4__degps,
								leaf_Veh_Accel_CAN4.Veh_brake_press_CAN4__bar
						);
					break;
				case 804:
					get_leaf_fuel_rate(db_steinhoff_msg.data, &leaf_fuel_rate);
					check_msg_timeout(ts_ms, &leaf_fuel_rate.ts_ms, 
						&leaf_fuel_rate.two_message_periods, 
						&leaf_fuel_rate.message_timeout_counter); 
					input.fuel_rate = leaf_fuel_rate.fuel_integrated_flow;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					db_clt_write(pclt,DB_LEAF_MSG804_VAR, sizeof(leaf_fuel_rate_t), &leaf_fuel_rate); 
					break;
				default:
//					if(veryverbose)
//						printf("Unknown message %#lX received\n", db_steinhoff_msg.id);
					break;
			}
		}
	else if(( DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_CHASSIS_3004_VAR )) {
		count++;
		db_clt_read(pclt, DB_STEINHOFF_CHASSIS_3004_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
		get_current_timestamp(&ts);
		ts_ms = TS_TO_MS(&ts);

		switch(db_steinhoff_msg.id) {
			case 0x2B0:
				get_leaf_torq_brake_acc(db_steinhoff_msg.data, &leaf_Torq_brake_ACC);

<<<<<<< Updated upstream
				if(leaf_Torq_brake_ACC.Brake_pedal_state == 0)
=======
				if( (leaf_Torq_brake_ACC.Brake_pedal_state == 0) && (leaf_Torq_brake_ACC.green_cruise_icon != 0))
>>>>>>> Stashed changes
					send_tx_commands = 1;

				check_msg_timeout(ts_ms, &leaf_steering.ts_ms,
					&leaf_Torq_brake_ACC.two_message_periods,
					&leaf_Torq_brake_ACC.message_timeout_counter);

				if(veryverbose){
					print_timestamp(stdout, &ts);
					printf("leaf_can: Left_side_Torque_cmd %d Right_side_Torque_cmd %d ACC_button_state %#hhx Brake_pedal_state %#hhx send_tx_commands %d Cruise_info %d counter %d d[0]: %#hhx d[1]: %#hhx d[2]: %#hhx d[3]: %#hhx d[6]: %#hhx \n",
						leaf_Torq_brake_ACC.Left_side_Torque_cmd,
						leaf_Torq_brake_ACC.Right_side_Torque_cmd,
						leaf_Torq_brake_ACC.ACC_button_state,
						leaf_Torq_brake_ACC.Brake_pedal_state,
						send_tx_commands,
						leaf_Torq_brake_ACC.Cruise_info,
						leaf_Torq_brake_ACC.counter,
						db_steinhoff_msg.data[0],
						db_steinhoff_msg.data[1],
						db_steinhoff_msg.data[2],
						db_steinhoff_msg.data[3],
						db_steinhoff_msg.data[6]
					);
				}
				break;
		}
	}
			else if(( DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_OBD2_IN_3003_VAR )) {
				count++;
				memset(&db_steinhoff_msg.data[0], 0, 8);
				db_clt_read(pclt, DB_STEINHOFF_OBD2_IN_3003_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
				get_current_timestamp(&ts);
				ts_ms = TS_TO_MS(&ts);
		//		printf("ID %hX: 0: %hhx 1: %hhx 2: %hhx 3: %hhx 4: %hhx 5: %hhx 6: %hhx 7: %hhx",
		//				db_steinhoff_msg.id,
		//				db_steinhoff_msg.data[0],
		//				db_steinhoff_msg.data[1],
		//				db_steinhoff_msg.data[2],
		//				db_steinhoff_msg.data[3],
		//				db_steinhoff_msg.data[4],
		//				db_steinhoff_msg.data[5],
		//				db_steinhoff_msg.data[6],
		//				db_steinhoff_msg.data[7]
		//		);
				obd2ID = (unsigned int ) (((db_steinhoff_msg.data[0] <<24) & 0xFF000000) +
						((db_steinhoff_msg.data[1] <<16) & 0x00FF0000) +
						((db_steinhoff_msg.data[2] <<8) & 0x0000FF00) +
						((db_steinhoff_msg.data[3]) & 0x000000FF));
		//		printf(" %#X\n", obd2ID);

				get_current_timestamp(&ts);
				ts_ms = TS_TO_MS(&ts);
				switch(db_steinhoff_msg.id) {
					case 0x735:
					switch(obd2ID){
						case 0x05620107: //object distance
							get_leaf_target_object_distance(db_steinhoff_msg.data, &leaf_target_object_distance);
							check_msg_timeout(ts_ms, &leaf_target_object_distance.ts_ms,
								&leaf_target_object_distance.two_message_periods,
								&leaf_target_object_distance.message_timeout_counter);
							db_clt_write(pclt, DB_LEAF_OBD2MSG107_VAR, sizeof(leaf_target_object_distance_t), &leaf_target_object_distance);
							if(verbose){
								print_timestamp(stdout, &ts);
								printf("Leaf target distance %.3f\n", leaf_target_object_distance.object_distance_Radar);
							}
							break;
						case 0x05620108: //object relative speed
							get_leaf_target_relative_speed(db_steinhoff_msg.data, &leaf_target_relative_speed_mps);
							check_msg_timeout(ts_ms, &leaf_target_relative_speed_mps.ts_ms,
								&leaf_target_relative_speed_mps.two_message_periods,
								&leaf_target_relative_speed_mps.message_timeout_counter);
							db_clt_write(pclt, DB_LEAF_OBD2MSG108_VAR, sizeof(leaf_target_relative_speed_mps_t), &leaf_target_relative_speed_mps);
							if(verbose){
								print_timestamp(stdout, &ts);
								printf("Leaf target relative speed %.3f\n", leaf_target_relative_speed_mps.object_relative_spd_Radar__mps);
							}
							break;
						default:
			//				if(veryverbose)
			//					printf("Unknown message %#lX received\n", db_steinhoff_msg.id);
							break;
					}
				}
			}
		get_current_timestamp(&ts);
		ts_now = TS_TO_MS(&ts);

		if(ts_now - ts_sav >= 20){
			ts_sav = ts_now;

			db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
			db_clt_read(pclt, DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t), &virtual_car_comm_packet);
			input.distance_from_start = virtual_car_comm_packet.user_float;
			db_clt_write(pclt, DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t), &virtual_car_comm_packet);

			if(verbose && ((count % 5) == 0) )
			print_timestamp(stdout, &ts);

			if(send_tx_commands == 1){
				if(use_98_command){
					memset(&db_steinhoff_torque_out, 0, sizeof(db_steinhoff_out_t));
					if(torque != 0){
						printf("torque2: torque %d\n", torque);
						leaf_torque_cmd.torque_cmd = torque;
					}
					else
						leaf_torque_cmd.torque_cmd = output.torque_level;
					if(leaf_torque_cmd.torque_cmd != 0) {
						output.accel_decel_request = 0;		//if braking is requested, set throttle to 0
					}
					db_steinhoff_torque_out.port = PORT_1;
					db_steinhoff_torque_out.id = 0x98;
					db_steinhoff_torque_out.size = 2;
					set_leaf_torque_cmd(db_steinhoff_torque_out.data, &leaf_torque_cmd);
					//			if(verbose && ((count % 5) == 0) )
					if(verbose)
						printf("leaf_can: torque %.2f  %hhx\n", leaf_torque_cmd.torque_cmd, db_steinhoff_torque_out.data[2]);
					db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_torque_out);
				}

				else
					if(!use_98_command){
			//			//Practical note: For the 99 message, the car will respond to both accel and decel messages as long as you're above the 21 mph
			//			//engagement limit.  below the engagement limit, the car will respond to decel requests but not to accel requests.
			//			memset(&db_steinhoff_accel_out, 0, sizeof(db_steinhoff_out_t));
						if(acceleration < 0)
							leaf_accel_cmd.accel_cmd = acceleration;
						else
							leaf_accel_cmd.accel_cmd = output.brake_level;
						if(leaf_accel_cmd.accel_cmd < 0){
							output.throttle_pct = 0;		//if braking is requested, set throttle to 0
						db_steinhoff_accel_out.port = BRAKE_PORT;
						db_steinhoff_accel_out.id = 0x99;
						db_steinhoff_accel_out.size = 2;
						set_leaf_accel_cmd(db_steinhoff_accel_out.data, &leaf_accel_cmd);
						if(verbose)
							printf("leaf_can: brake %hhx %#hhx %.2f\n",
								db_steinhoff_accel_out.data[0],
								db_steinhoff_accel_out.data[1],
								leaf_accel_cmd.accel_cmd
							);
						db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);
						}

						if(acceleration > 0) {
							leaf_accel_cmd.accel_cmd = acceleration;
						}
						else
							leaf_accel_cmd.accel_cmd = output.throttle_pct;
						if(leaf_accel_cmd.accel_cmd > 0) {
							db_steinhoff_accel_out.port = BRAKE_PORT;
							db_steinhoff_accel_out.id = 0x99;
							db_steinhoff_accel_out.size = 2;
							set_leaf_accel_cmd(db_steinhoff_accel_out.data, &leaf_accel_cmd);
							if(verbose)
								printf("leaf_can: accel %hhx %#hhx %.2f\n",
									db_steinhoff_accel_out.data[0],
									db_steinhoff_accel_out.data[1],
									leaf_accel_cmd.accel_cmd
								);
							db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);
						}


						memset(&db_steinhoff_accel_out, 0, sizeof(db_steinhoff_out_t));
						if(acceleration > 0) {
							leaf_accel_cmd.accel_cmd = acceleration;
							leaf_torque_cmd.torque_cmd = 0;
						}
						else
							leaf_accel_cmd.accel_cmd = output.throttle_pct;
						if(leaf_accel_cmd.accel_cmd > 0) {
							db_steinhoff_accel_out.port = PORT_1;
							db_steinhoff_accel_out.id = 0x99;
							db_steinhoff_accel_out.size = 2;
							set_leaf_accel_cmd(db_steinhoff_accel_out.data, &leaf_accel_cmd);
							if(verbose && ((count % 5) == 0) )
								printf("leaf_can: acceleration %hhx %.2f\n", db_steinhoff_accel_out.data[0], leaf_accel_cmd.accel_cmd);
						}
						db_clt_write(pclt, DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);
					}
			}

			if((++obd2_ctr % 3) == 0) {
				if(toggle_flag == 0) {
					//Send OBD2 target distance poll
					db_steinhoff_obd2.port = PORT_3;
					db_steinhoff_obd2.id = 0x723;
					db_steinhoff_obd2.size = 8;
					db_steinhoff_obd2.data[0] = 0x03;
					db_steinhoff_obd2.data[1] = 0x22;
					db_steinhoff_obd2.data[2] = 0x01;
					db_steinhoff_obd2.data[3] = 0x07;
					db_steinhoff_obd2.data[4] = 0x00;
					db_steinhoff_obd2.data[5] = 0x00;
					db_steinhoff_obd2.data[6] = 0x00;
					db_steinhoff_obd2.data[7] = 0x00;
					db_clt_write(pclt, DB_STEINHOFF_OBD2_OUT_4003_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_obd2);
					toggle_flag = 1;
				}
				else{
					//Send OBD2 object relative speed poll
					db_steinhoff_obd2.port = PORT_3;
					db_steinhoff_obd2.id = 0x723;
					db_steinhoff_obd2.size = 8;
					db_steinhoff_obd2.data[0] = 0x03;
					db_steinhoff_obd2.data[1] = 0x22;
					db_steinhoff_obd2.data[2] = 0x01;
					db_steinhoff_obd2.data[3] = 0x08;
					db_steinhoff_obd2.data[4] = 0x00;
					db_steinhoff_obd2.data[5] = 0x00;
					db_steinhoff_obd2.data[6] = 0x00;
					db_steinhoff_obd2.data[7] = 0x00;
					db_clt_write(pclt, DB_STEINHOFF_OBD2_OUT_4003_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_obd2);
					toggle_flag = 0;
				}
			}
			}
			usleep(10);
	}
	return 0;
}

void check_msg_timeout(int curr_ts_ms, int *prev_ts_ms, 
	unsigned char *two_message_periods, 
	unsigned int *message_timeout_counter) {
	if( (curr_ts_ms - *prev_ts_ms) > *two_message_periods ) {
	   ++*message_timeout_counter;
	   *prev_ts_ms = curr_ts_ms;
	}
}
