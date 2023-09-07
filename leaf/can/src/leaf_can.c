#include "db_include.h"
#include "leaf_can.h"
#include "clt_vars_leaf.h"
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

const char *usage = "-v verbose -a <acceleration> -b <braking pct>";

int steinhoff_trig_list[] =  {
        DB_STEINHOFF_MSG_VAR,
		DB_OUTPUT_VAR
};

int num_steinhoff_trig_variables = sizeof(steinhoff_trig_list)/sizeof(int);

int main(int argc, char *argv[]) {
	int verbose = 0;
	int veryverbose = 0;
	int option;
	int count = 0;
	float acceleration = 0;
	float torque = 0;

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

	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms;
	int ts_now;
	int ts_sav;

	leaf_accel_cmd_t leaf_accel_cmd;
	leaf_torque_cmd_t leaf_torque_cmd;
	leaf_vehicle_speed_t leaf_vehicle_speed;
	leaf_torque_t  leaf_torque;
	leaf_steering_t leaf_steering;
	leaf_fuel_rate_t leaf_fuel_rate;
	veh_comm_packet_t virtual_car_comm_packet;
	leaf_Veh_Accel_CAN4_t leaf_Veh_Accel_CAN4;

        while ((option = getopt(argc, argv, "vwa:b:")) != EOF) {
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
                case 't':
                        torque = atof(optarg);
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
//			NULL, 0, steinhoff_trig_list, num_steinhoff_trig_variables))
			NULL, 0, NULL, 0))
		== NULL) {
			exit(EXIT_FAILURE);
	}
    printf("Got to 1.5\n");

	if (clt_trig_set( pclt, DB_STEINHOFF_MSG_VAR, DB_STEINHOFF_MSG_TYPE) == FALSE )
		exit(EXIT_FAILURE);

printf("leaf_can: clt_trig_set OK for DB_STEINHOFF_MSG_VAR %d\n", DB_STEINHOFF_MSG_VAR);
        if (setjmp(exit_env) != 0) {
        	leaf_torque_cmd.torque_cmd = 0;
		db_steinhoff_torque_out.port = BRAKE_PORT;
		db_steinhoff_torque_out.id = 0x98;
		db_steinhoff_torque_out.size = 2;
		set_leaf_torque_cmd(db_steinhoff_torque_out.data, &leaf_torque_cmd);
		db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_torque_out);

		sleep(0.02);

		leaf_accel_cmd.accel_cmd = 0;
		db_steinhoff_accel_out.port = ACCEL_PORT;
		db_steinhoff_accel_out.id = 0x99;
		db_steinhoff_accel_out.size = 2;
		set_leaf_accel_cmd(db_steinhoff_accel_out.data, &leaf_accel_cmd);
			db_clt_write(pclt, DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);

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

	leaf_accel_cmd.two_message_periods = 40; 		// 2*20 msec
	leaf_torque_cmd.two_message_periods = 40; 		// 2*20 msec
	leaf_vehicle_speed.two_message_periods = 80;
	leaf_torque.two_message_periods = 80;
	leaf_steering.two_message_periods = 80;


	db_clt_write(pclt, DB_LEAF_MSG98_VAR, sizeof(leaf_torque_cmd_t), &leaf_torque_cmd);
	db_clt_write(pclt, DB_LEAF_MSG99_VAR, sizeof(leaf_accel_cmd_t), &leaf_accel_cmd);
	db_clt_write(pclt, DB_LEAF_MSG292_VAR, sizeof(leaf_vehicle_speed_t), &leaf_Veh_Accel_CAN4);

	if(acceleration == 0)
		leaf_accel_cmd.accel_cmd = 0;
	if(torque == 0)
		leaf_torque_cmd.torque_cmd = 0;
	get_current_timestamp(&ts);
	ts_now = TS_TO_MS(&ts);
	ts_sav = ts_now;
    printf("Got to 2\n");

	for(;;) {
	
		clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
		if( DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_MSG_VAR ) {
			count++;
			memset(&db_steinhoff_msg.data[0], 0, 8);
			db_clt_read(pclt, DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);
			switch(db_steinhoff_msg.id) {
				case 0x158:
					get_leaf_vehicle_speed(db_steinhoff_msg.data, &leaf_vehicle_speed);
					check_msg_timeout(ts_ms, &leaf_vehicle_speed.ts_ms, 
						&leaf_vehicle_speed.two_message_periods, 
						&leaf_vehicle_speed.message_timeout_counter); 
					input.vehicle_speed_mps = leaf_vehicle_speed.veh_speed1_CAN6__mps;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);

					if(veryverbose)
						printf("Leaf vehicle speed %.2f output.torque_level %.2f output.accel_decel_request %.2f\n",
								input.vehicle_speed_mps,
								output.torque_level,
								output.accel_decel_request
						);
					break;
				case 0x530:
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
				case 342:
					get_leaf_steering(db_steinhoff_msg.data, &leaf_steering);
					check_msg_timeout(ts_ms, &leaf_steering.ts_ms, 
						&leaf_steering.two_message_periods, 
						&leaf_steering.message_timeout_counter); 
					input.steering_angle_deg = leaf_steering.Steering_Wheel_Pos_CAN_deg;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					if(veryverbose)
						printf("Leaf steering position %.2f rate %.2f\n",
							leaf_steering.Steering_Wheel_Pos_CAN_deg,
							leaf_steering.Steering_Wheel_Rate_CAN_degps
						);
					break;
				case 0x292:
					get_leaf_accel(db_steinhoff_msg.data, &leaf_Veh_Accel_CAN4);
					check_msg_timeout(ts_ms, &leaf_Veh_Accel_CAN4.ts_ms,
						&leaf_Veh_Accel_CAN4.two_message_periods,
						&leaf_Veh_Accel_CAN4.message_timeout_counter);
					input.vehicle_accel_mps2 = G2MPS * leaf_Veh_Accel_CAN4.Long_Accel_CAN4__G;
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
//					if(veryverbose)
//print_timestamp(stdout, &ts);
//printf("Leaf fuel rate %.2f\n",
//	leaf_fuel_rate.fuel_integrated_flow
//);
					break;
				default:
//					if(veryverbose)
//						printf("Unknown message %#lX received\n", db_steinhoff_msg.id);
					break;
			}
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

			if(verbose && ((count % 5) == 0) )
			print_timestamp(stdout, &ts);

			memset(&db_steinhoff_torque_out, 0, sizeof(db_steinhoff_out_t));
			if(torque > 0)
				leaf_torque_cmd.torque_cmd = torque;
			else
				leaf_torque_cmd.torque_cmd = output.torque_level;
			if(leaf_torque_cmd.torque_cmd > 0) {
				output.accel_decel_request = 0;		//if braking is requested, set throttle to 0
			}
			db_steinhoff_torque_out.port = BRAKE_PORT;
			db_steinhoff_torque_out.id = 0x98;
			db_steinhoff_torque_out.size = 2;
			set_leaf_torque_cmd(db_steinhoff_torque_out.data, &leaf_torque_cmd);
			if(verbose && ((count % 5) == 0) )
				printf("leaf_can: torque %.2f  %hhx\n", leaf_torque_cmd.torque_cmd, db_steinhoff_torque_out.data[2]);
			db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_torque_out);

			// N.B. No delay is necessary between sending torque & acceleration commands because
			// on the Leaf, these two ECUs are on different CAN buses.
			memset(&db_steinhoff_accel_out, 0, sizeof(db_steinhoff_out_t));
			if(acceleration > 0) {
				leaf_accel_cmd.accel_cmd = acceleration;
				leaf_torque_cmd.torque_cmd = 0;
			}
			else
				leaf_accel_cmd.accel_cmd = output.accel_decel_request;
			if(leaf_accel_cmd.accel_cmd > 0) {
				db_steinhoff_accel_out.port = ACCEL_PORT;
				db_steinhoff_accel_out.id = 0x99;
				db_steinhoff_accel_out.size = 2;
				set_leaf_accel_cmd(db_steinhoff_accel_out.data, &leaf_accel_cmd);
				if(verbose && ((count % 5) == 0) )
					printf("leaf_can: acceleration %hhx %.2f\n", db_steinhoff_accel_out.data[0], leaf_accel_cmd.accel_cmd);
			}
			db_clt_write(pclt, DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);
			usleep(10);
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
