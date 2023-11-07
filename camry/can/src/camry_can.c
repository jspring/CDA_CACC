#include "db_include.h"
#include "camry_can.h"
#include "clt_vars_camry.h"
#include "can_dbvars.h"

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

const char *usage = "-v verbose -a <acceleration>";

int steinhoff_trig_list[] =  {
        DB_STEINHOFF_MSG_VAR,
        DB_STEINHOFF_MSG2_VAR,
		DB_OUTPUT_VAR
};

int num_steinhoff_trig_variables = sizeof(steinhoff_trig_list)/sizeof(int);

int main(int argc, char *argv[]) {
	int verbose = 0;
	int option;
	int count = 0;
	float acceleration = 0;

    char hostname[MAXHOSTNAMELEN];
    char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
    db_clt_typ *pclt;               /// data server client pointer
    int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
	posix_timer_typ *ptimer;
	int ipc_message_error_ctr = 0;
	trig_info_typ trig_info;
	int index;

	db_steinhoff_msg_t db_steinhoff_msg;
	db_steinhoff_out_t db_steinhoff_brake_out;
	db_steinhoff_out_t db_steinhoff_accel_out;
	db_steinhoff_out_t db_steinhoff_obd2;
	veh_comm_packet_t virtual_car_comm_packet;
	camry_fuel_rate_t camry_fuel_rate;
	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms = 0;
	int ts_now = 0;
	int ts_sav = 0;
	char obd2_ctr = 0;

	camry_accel_cmd_status_t camry_accel_cmd_status;
	camry_accel_cmd_t camry_accel_cmd;
	camry_accel_cmd_t camry_brake_cmd;
	camry_cruise_control_state_t camry_cruise_control_state;
	camry_vehicle_speed_t camry_vehicle_speed;
	camry_wheel_speed_t camry_wheel_speed;
	camry_long_lat_accel_t camry_long_lat_accel;
	camry_radar_forward_vehicle_t camry_radar_forward_vehicle[12];

        while ((option = getopt(argc, argv, "va:")) != EOF) {
                switch(option) {
                case 'v':
                        verbose = 1;
                        break;
                case 'a':
                        acceleration = atof(optarg);
                        break;
                default:
                        printf("Usage: %s %s\n", argv[0], usage);
                        exit(EXIT_FAILURE);
                        break;
                }
        }

    	get_local_name(hostname, MAXHOSTNAMELEN);
    	if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
    //			NULL, 0, steinhoff_trig_list, num_steinhoff_trig_variables))
    			NULL, 0, NULL, 0))
    		== NULL) {
    			exit(EXIT_FAILURE);
    	}

    	if (clt_trig_set( pclt, DB_STEINHOFF_MSG_VAR, DB_STEINHOFF_MSG_TYPE) == FALSE ){
    		printf("Could not set trigger for DB_STEINHOFF_MSG_VAR %d\n ", DB_STEINHOFF_MSG_VAR);
    		exit(EXIT_FAILURE);
    	}
    	else
    		printf("camry_can: clt_trig_set OK for DB_STEINHOFF_MSG_VAR %d\n", DB_STEINHOFF_MSG_VAR);

    	if (clt_trig_set( pclt, DB_STEINHOFF_MSG2_VAR, DB_STEINHOFF_MSG2_TYPE) == FALSE ){
    		printf("Could not set trigger for DB_STEINHOFF_MSG2_VAR %d\n ", DB_STEINHOFF_MSG2_VAR);
    		exit(EXIT_FAILURE);
    	}
    	else
    		printf("camry_can: clt_trig_set OK for DB_STEINHOFF_MSG2_VAR %d\n", DB_STEINHOFF_MSG2_VAR);

    	if (setjmp(exit_env) != 0) {
    		memset(&db_steinhoff_brake_out, 0, sizeof(db_steinhoff_out_t));
    		db_steinhoff_brake_out.port = BRAKE_PORT;
    		db_steinhoff_brake_out.id = 0x99;
    		db_steinhoff_brake_out.size = 2;
 //   		set_camry_brake_cmd(db_steinhoff_brake_out.data, &camry_brake_cmd);
    		db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);
        	db_list_done(pclt, NULL, 0, NULL, 0);
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
	memset(&camry_accel_cmd, 0, sizeof(camry_accel_cmd));
	memset(&camry_accel_cmd_status, 0, sizeof(camry_accel_cmd_status_t));
	memset(&camry_cruise_control_state, 0, sizeof(camry_cruise_control_state_t));
	memset(&camry_vehicle_speed, 0, sizeof(camry_vehicle_speed_t));
	memset(&camry_wheel_speed, 0, sizeof(camry_wheel_speed_t));
	memset(&camry_long_lat_accel, 0, sizeof(camry_long_lat_accel_t));

	camry_accel_cmd.two_message_periods = 80; 		// 2*10 msec
	camry_accel_cmd_status.two_message_periods = 80; 		// 2*10 msec
	camry_cruise_control_state.two_message_periods = 80;
	camry_vehicle_speed.two_message_periods = 80;
	camry_wheel_speed.two_message_periods = 80;
	camry_long_lat_accel.two_message_periods = 80;


	db_clt_write(pclt, DB_CAMRY_MSG99_VAR, sizeof(camry_accel_cmd_t), &camry_accel_cmd); 
	db_clt_write(pclt, DB_CAMRY_MSG343_VAR, sizeof(camry_accel_cmd_status_t), &camry_accel_cmd_status); 
	db_clt_write(pclt, DB_CAMRY_MSG399_VAR, sizeof(camry_cruise_control_state_t), &camry_cruise_control_state);
	db_clt_write(pclt, DB_CAMRY_MSGB4_VAR, sizeof(camry_vehicle_speed_t), &camry_vehicle_speed);
	db_clt_write(pclt, DB_CAMRY_MSGAA_VAR, sizeof(camry_wheel_speed_t), &camry_wheel_speed);
	db_clt_write(pclt, DB_CAMRY_MSG228_VAR, sizeof(camry_long_lat_accel_t), &camry_long_lat_accel);

	get_current_timestamp(&ts);
	ts_sav = ts_now = TS_TO_MS(&ts);

	for(;;) {
	
		clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
		if(( DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_MSG_VAR ) || ( DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_MSG2_VAR )) {
			count++;
			memset(&db_steinhoff_msg.data[0], 0, 8);
			if(DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_MSG_VAR)
				db_clt_read(pclt, DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
			else{
				db_clt_read(pclt, DB_STEINHOFF_MSG2_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
//				printf("Reading DB_STEINHOFF_MSG2_VAR id %#X \n", db_steinhoff_msg.id);
			}

			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);
			switch(db_steinhoff_msg.id) {
				case 0xAA: //170
					get_camry_wheel_speed(db_steinhoff_msg.data, &camry_wheel_speed);
					check_msg_timeout(ts_ms, &camry_wheel_speed.ts_ms, 
						&camry_wheel_speed.two_message_periods, 
						&camry_wheel_speed.message_timeout_counter); 
					db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					input.vehicle_speed_mps = camry_wheel_speed.veh_wheel_spd_FR_CAN1_mps;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					if(verbose)
						printf("Camry wheel speed %.2f input.vehicle_speed_mps %.2f d[0] %#hhx d[1] %#hhx %.2f %.2f %.2f instantaneous_acceleration %.3f\n",
							camry_wheel_speed.veh_wheel_spd_FR_CAN1_mps,
							input.vehicle_speed_mps,
							db_steinhoff_msg.data[0],
							db_steinhoff_msg.data[1],
							camry_wheel_speed.veh_wheel_spd_FL_CAN1_mps,
							camry_wheel_speed.veh_wheel_spd_RR_CAN1_mps,
							camry_wheel_speed.veh_wheel_spd_RL_CAN1_mps,
							camry_wheel_speed.instantaneous_acceleration
							);
					break;
				case 0xB4: //180
					get_camry_vehicle_speed(db_steinhoff_msg.data, &camry_vehicle_speed);
					check_msg_timeout(ts_ms, &camry_vehicle_speed.ts_ms, 
						&camry_vehicle_speed.two_message_periods, 
						&camry_vehicle_speed.message_timeout_counter); 
					db_clt_write(pclt,DB_CAMRY_MSGB4_VAR, sizeof(camry_vehicle_speed_t), &camry_vehicle_speed); 
//					if(verbose)
//				        printf("camry_vehicle_speed:  vehicle speed %.3f\n",
//				                camry_vehicle_speed.veh_spd_CAN1_kph
//				        );
					break;
				case 0x228:
					get_camry_long_lat_accel(db_steinhoff_msg.data, &camry_long_lat_accel);
					check_msg_timeout(ts_ms, &camry_long_lat_accel.ts_ms, 
						&camry_long_lat_accel.two_message_periods, 
						&camry_long_lat_accel.message_timeout_counter); 
					db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					input.vehicle_accel_mps2 = camry_long_lat_accel.long_accel;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
//					if(verbose)
//						print_long_lat_accel(&camry_long_lat_accel);
					break;
//				case 0x301: //769
//				case 0x303: //771
//				case 0x305: //769
//				case 0x307: //769
//				case 0x309: //769
//				case 0x30B: //769
//				case 0x30D: //769
//				case 0x30F: //769
//				case 0x311: //769
//				case 0x313: //769
//				case 0x315: //769
//				case 0x317: //769
				case 0x680: //769
//				case 0x681: //769
//				case 0x682: //769
//				case 0x683: //769
//				case 0x684: //769				case 0x683: //769
//				case 0x685: //769

//					if(index < 0x680)
//					index = (db_steinhoff_msg.id - 0x301) / 2;
//					if(index >= 0x680)
//						index = db_steinhoff_msg.id - 0x680;
					index=0;
printf("ID %d index %d\n", db_steinhoff_msg.id, index);
					get_camry_radar_forward_vehicle(db_steinhoff_msg.data, &camry_radar_forward_vehicle[index], db_steinhoff_msg.id);
					check_msg_timeout(ts_ms, &camry_radar_forward_vehicle[index].ts_ms,
						&camry_radar_forward_vehicle[index].two_message_periods,
						&camry_radar_forward_vehicle[index].message_timeout_counter);
						db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
						db_clt_write(pclt, DB_CAMRY_MSG680_VAR + index, sizeof(camry_radar_forward_vehicle_t), &camry_radar_forward_vehicle[index]);
						if(verbose){
						     print_timestamp(stdout, &ts);
						     printf("Camry %d target: dist %.5f  d[0] %#hhx d[1] %#hhx speed %.5f d[2] %#hhx d[3] %#hhx\n",
						    		 db_steinhoff_msg.id,
					             camry_radar_forward_vehicle[index].LONG_DIST_CAN1__m,
						             db_steinhoff_msg.data[0],
						             db_steinhoff_msg.data[1],
									 camry_radar_forward_vehicle[index].LONG_SPEED_CAN1__mps,
						             db_steinhoff_msg.data[2],
						             db_steinhoff_msg.data[3]

						    );
						}
						break;
//				case 1664: //769
//				case 1665: //769
//				case 1666: //769
//				case 1667: //769
//				case 1668: //769
//				case 1669: //769
////				case 0x680: //769
////				case 0x681: //769
////				case 0x682: //769
////				case 0x683: //769
////				case 0x684: //769
////				case 0x685: //769
//					index = db_steinhoff_msg.id - 0x680;
//					get_camry_radar_forward_vehicle(db_steinhoff_msg.data, &camry_radar_forward_vehicle[index]);
//					check_msg_timeout(ts_ms, &camry_radar_forward_vehicle[index].ts_ms,
//						&camry_radar_forward_vehicle[index].two_message_periods,
//						&camry_radar_forward_vehicle[index].message_timeout_counter);
//						db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
////						char ID;
////						float LONG_DIST_CAN1__m;
////						float LAT_DIST_CAN1__m;
////						float LONG_SPEED_CAN1__kph;
////						float LONG_SPEED_CAN1__mps;
////						float LAT_SPEED_CAN1__mps;
////						int RCS;
//						db_clt_write(pclt, DB_CAMRY_MSG680_VAR + index, sizeof(camry_radar_forward_vehicle_t), &camry_radar_forward_vehicle[index]);
//						if(verbose){
//						     print_timestamp(stdout, &ts);
//						     printf("Camry %#hX target: dist %.5f  d[0] %#hhx d[1] %#hhx speed %.5f d[2] %#hhx d[3] %#hhx\n",
//						    		 db_steinhoff_msg.id,
//						             camry_radar_forward_vehicle[index].LONG_DIST_CAN1__m,
//						             db_steinhoff_msg.data[0],
//						             db_steinhoff_msg.data[1],
//									 camry_radar_forward_vehicle[index].LONG_SPEED_CAN1__mps,
//						             db_steinhoff_msg.data[2],
//						             db_steinhoff_msg.data[3]
//
//						    );
//						}
//						break;
				case 0x7E9:
//					get_camry_fuel_rate(db_steinhoff_msg.data, &camry_fuel_rate);
//					check_msg_timeout(ts_ms, &camry_fuel_rate.ts_ms,
//						&camry_fuel_rate.two_message_periods,
//						&camry_fuel_rate.message_timeout_counter);
//					input.fuel_rate = camry_fuel_rate.fuel_rate;
//					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					if(verbose)
						print_timestamp(stdout, &ts);
					if(verbose){
						printf("camry OBD2 targets: ");
						for(int i=0; i<8; i++)
							printf("%#2.2hhX", db_steinhoff_msg.data[i]);
						printf("\n");
					}
					break;

//				case 0x7E8:
//					get_camry_fuel_rate(db_steinhoff_msg.data, &camry_fuel_rate);
//					check_msg_timeout(ts_ms, &camry_fuel_rate.ts_ms,
//						&camry_fuel_rate.two_message_periods,
//						&camry_fuel_rate.message_timeout_counter);
//					input.fuel_rate = camry_fuel_rate.fuel_rate;
//					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
//					if(verbose)
//						print_timestamp(stdout, &ts);
//					if(verbose)
//						printf("camry_fuel_rate %.3f\n", camry_fuel_rate.fuel_rate);
//					break;
				default:
//					printf("Unknown message %#hX received\n", db_steinhoff_msg);
					break;
			}
//			if(print_msg)
//				printmsg(&db_kom);
//			if(veryverbose) {
//				printcan(&db_kom);
//				printmsg(&db_kom);
//			}
		}	
		get_current_timestamp(&ts);
		ts_now = TS_TO_MS(&ts);

		if(ts_now - ts_sav >= 20){
			ts_sav = ts_now;

                        db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
                        db_clt_read(pclt, DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t), &virtual_car_comm_packet);
/*
                        printf("Virtual car message: seq no %d rate %.2f range %.2f accel %.2f distance from start %.4f\n",
                                virtual_car_comm_packet.sequence_no,
                                virtual_car_comm_packet.rate,
                                virtual_car_comm_packet.range,
                                virtual_car_comm_packet.accel,
                                virtual_car_comm_packet.user_float
                        );
*/
                        input.distance_from_start = virtual_car_comm_packet.user_float;
                        db_clt_write(pclt, DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t), &virtual_car_comm_packet);

			memset(&db_steinhoff_brake_out, 0, sizeof(db_steinhoff_out_t));
			if(acceleration < 0)
				camry_brake_cmd.accel_cmd = acceleration;
			else
				camry_brake_cmd.accel_cmd = output.accel_decel_request;
			if(camry_brake_cmd.accel_cmd < 0){
				output.throttle_pct = 0;		//if braking is requested, set throttle to 0
				db_steinhoff_brake_out.port = BRAKE_PORT;
				db_steinhoff_brake_out.id = 0x99;
				db_steinhoff_brake_out.size = 2;
				set_camry_accel_cmd(db_steinhoff_brake_out.data, &camry_brake_cmd);
				if(verbose)
					printf("camry_can: brake %hhx %#hhx %.2f\n",
						db_steinhoff_brake_out.data[0],
						db_steinhoff_brake_out.data[1],
						camry_brake_cmd.accel_cmd
					);
				db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);
			}

			memset(&db_steinhoff_accel_out, 0, sizeof(db_steinhoff_out_t));
			if(acceleration > 0) {
				camry_accel_cmd.accel_cmd = acceleration;
				camry_brake_cmd.accel_cmd = 0;
			}
			else
				camry_accel_cmd.accel_cmd = output.throttle_pct;
			if(camry_accel_cmd.accel_cmd > 0) {
				db_steinhoff_accel_out.port = BRAKE_PORT;
				db_steinhoff_accel_out.id = 0x99;
				db_steinhoff_accel_out.size = 2;
				set_camry_accel_cmd(db_steinhoff_accel_out.data, &camry_accel_cmd);
				if(verbose)
					printf("camry_can: accel %hhx %#hhx %.2f\n", 
						db_steinhoff_accel_out.data[0],
						db_steinhoff_accel_out.data[1],
						camry_accel_cmd.accel_cmd
					);
				db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);
			}
			if((++obd2_ctr % 3) == 0) { //Send OBD2 targets poll
				db_steinhoff_obd2.port = ACCEL_PORT;
				db_steinhoff_obd2.id = 0x790;
				db_steinhoff_obd2.size = 8;
				db_steinhoff_obd2.data[0] = 2;
				db_steinhoff_obd2.data[1] = 0x21;
				db_steinhoff_obd2.data[2] = 0x0B;
				db_steinhoff_obd2.data[3] = 0x00;
				db_steinhoff_obd2.data[4] = 0x00;
				db_steinhoff_obd2.data[5] = 0x00;
				db_steinhoff_obd2.data[6] = 0x00;
				db_steinhoff_obd2.data[7] = 0x00;
				db_clt_write(pclt, DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_obd2);
//				if((++obd2_ctr % 3) == 0) { //Send OBD2 fuel level poll
//					db_steinhoff_obd2.port = ACCEL_PORT;
//					db_steinhoff_obd2.id = 0x7DF;
//					db_steinhoff_obd2.size = 8;
//					db_steinhoff_obd2.data[0] = 2;
//					db_steinhoff_obd2.data[1] = 1;
//					db_steinhoff_obd2.data[2] = 0x10;
//					db_steinhoff_obd2.data[3] = 0xCC;
//					db_steinhoff_obd2.data[4] = 0xCC;
//					db_steinhoff_obd2.data[5] = 0xCC;
//					db_steinhoff_obd2.data[6] = 0xCC;
//					db_steinhoff_obd2.data[7] = 0xCC;
//					db_clt_write(pclt, DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_obd2);
//					if(verbose)
//						print_timestamp(stdout, &ts);
//					if(verbose)
//						printf("Sending OBD2 fuel poll\n");
			}
		}
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
