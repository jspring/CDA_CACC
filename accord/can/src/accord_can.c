#include "db_include.h"
#include "accord_can.h"
#include "clt_vars_accord.h"
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
	float deceleration = 0;

        char hostname[MAXHOSTNAMELEN];
        char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
        db_clt_typ *pclt;               /// data server client pointer
        int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
	posix_timer_typ *ptimer;
	int ipc_message_error_ctr = 0;
	trig_info_typ trig_info;

	db_steinhoff_msg_t db_steinhoff_msg;
	db_steinhoff_out_t db_steinhoff_brake_out;
	db_steinhoff_out_t db_steinhoff_accel_out;

	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms;
	int ts_now;
	int ts_sav;

	accord_accel_cmd_t accord_accel_cmd;
	accord_brake_cmd_t accord_brake_cmd;
	accord_vehicle_speed_t accord_vehicle_speed;
	accord_torque_t  accord_torque;
	accord_PRNDL_Pos_t  accord_PRNDL_Pos;
	accord_steering_t accord_steering;
	accord_fuel_rate_t accord_fuel_rate;
	veh_comm_packet_t virtual_car_comm_packet;

	accord_target_object_t accord_target_object_410;
	accord_target_object_t accord_target_object_411;
	accord_target_object_t accord_target_object_412;
	accord_target_object_t accord_target_object_413;
	accord_target_object_t accord_target_object_414;
	accord_target_object_t accord_target_object_415;
	accord_target_object_t accord_target_object_416;
	accord_target_object_t accord_target_object_417;
	accord_target_object_t accord_target_object_420;
	accord_target_object_t accord_target_object_421;
	accord_target_object_t accord_target_object_422;
	accord_target_object_t accord_target_object_423;
	accord_target_object_t accord_target_object_424;

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
                case 'b':
                        deceleration = atof(optarg);
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

	if (clt_trig_set( pclt, DB_STEINHOFF_MSG_VAR, DB_STEINHOFF_MSG_TYPE) == FALSE )
		exit(EXIT_FAILURE);

printf("accord_can: clt_trig_set OK for DB_STEINHOFF_MSG_VAR %d\n", DB_STEINHOFF_MSG_VAR);
        if (setjmp(exit_env) != 0) {
        	accord_brake_cmd.brake_cmd = 0;
		db_steinhoff_brake_out.port = BRAKE_PORT;
		db_steinhoff_brake_out.id = 0x99;
		db_steinhoff_brake_out.size = 6;
		set_accord_brake_cmd(db_steinhoff_brake_out.data, &accord_brake_cmd);
		db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);

		sleep(0.02);

		accord_accel_cmd.accel_cmd = 0;
		db_steinhoff_accel_out.port = ACCEL_PORT;
		db_steinhoff_accel_out.id = 0x98;
		db_steinhoff_accel_out.size = 1;
		set_accord_accel_cmd(db_steinhoff_accel_out.data, &accord_accel_cmd);
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
	memset(&accord_accel_cmd, 0, sizeof(accord_accel_cmd));
	memset(&accord_brake_cmd, 0, sizeof(accord_brake_cmd));
	memset(&accord_vehicle_speed, 0, sizeof(accord_vehicle_speed_t));
	memset(&accord_torque, 0, sizeof(accord_torque_t));
	memset(&accord_PRNDL_Pos, 0, sizeof(accord_PRNDL_Pos_t));
	memset(&accord_steering, 0, sizeof(accord_steering_t));

	accord_accel_cmd.two_message_periods = 40; 		// 2*20 msec
	accord_brake_cmd.two_message_periods = 40; 		// 2*20 msec
	accord_vehicle_speed.two_message_periods = 80;
	accord_torque.two_message_periods = 80;
	accord_PRNDL_Pos.two_message_periods = 80;
	accord_steering.two_message_periods = 80;


	db_clt_write(pclt, DB_ACCORD_MSG98_VAR, sizeof(accord_accel_cmd_t), &accord_accel_cmd); 
	db_clt_write(pclt, DB_ACCORD_MSG99_VAR, sizeof(accord_brake_cmd_t), &accord_brake_cmd); 
	db_clt_write(pclt, DB_ACCORD_MSG158_VAR, sizeof(accord_vehicle_speed_t), &accord_vehicle_speed);
	db_clt_write(pclt, DB_ACCORD_MSG530_VAR, sizeof(accord_torque_t), &accord_torque);
	db_clt_write(pclt, DB_ACCORD_MSG342_VAR, sizeof(accord_steering_t), &accord_steering);
	db_clt_write(pclt, DB_ACCORD_MSG392_VAR, sizeof(accord_PRNDL_Pos_t), &accord_PRNDL_Pos);
	if(acceleration == 0)
		accord_accel_cmd.accel_cmd = 0;
	if(deceleration == 0)
		accord_brake_cmd.brake_cmd = 0;
	get_current_timestamp(&ts);
	ts_now = TS_TO_MS(&ts);
	ts_sav = ts_now;

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
					get_accord_vehicle_speed(db_steinhoff_msg.data, &accord_vehicle_speed);
					check_msg_timeout(ts_ms, &accord_vehicle_speed.ts_ms, 
						&accord_vehicle_speed.two_message_periods, 
						&accord_vehicle_speed.message_timeout_counter); 
					input.vehicle_speed_mps = accord_vehicle_speed.vehicle_speed_CAN2_MPS;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);

					if(veryverbose)
						printf("Accord vehicle speed %.2f output.brake_level %.2f output.throttle_pct %.2f\n", input.vehicle_speed_mps, output.brake_level,output.throttle_pct);
					break;
				case 0x530:
					get_accord_torque(db_steinhoff_msg.data, &accord_torque);
					check_msg_timeout(ts_ms, &accord_torque.ts_ms, 
						&accord_torque.two_message_periods, 
						&accord_torque.message_timeout_counter); 
					input.torque = accord_torque.generator_torque_CAN2_nm;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					if(veryverbose)
						printf("Accord generator torque %.2f motor torque %.2f\n",
							accord_torque.generator_torque_CAN2_nm,
							accord_torque.motor_torque_CAN2_nm
						);
					break;
				case 342:
					get_accord_steering(db_steinhoff_msg.data, &accord_steering);
					check_msg_timeout(ts_ms, &accord_steering.ts_ms, 
						&accord_steering.two_message_periods, 
						&accord_steering.message_timeout_counter); 
					input.steering_angle_deg = accord_steering.Steering_Wheel_Pos_CAN_deg;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					if(veryverbose)
						printf("Accord steering position %.2f rate %.2f\n",
							accord_steering.Steering_Wheel_Pos_CAN_deg,
							accord_steering.Steering_Wheel_Rate_CAN_degps
						);
					break;
				case 0x392:
					get_accord_PRNDL_Pos(db_steinhoff_msg.data, &accord_PRNDL_Pos);
					check_msg_timeout(ts_ms, &accord_PRNDL_Pos.ts_ms, 
						&accord_PRNDL_Pos.two_message_periods, 
						&accord_PRNDL_Pos.message_timeout_counter); 
					input.gear = accord_PRNDL_Pos.prndl;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					if(veryverbose)
						printf("Accord PRNDL position %hhu\n",
							accord_PRNDL_Pos.prndl
						);
					break;
				case 804:
					get_accord_fuel_rate(db_steinhoff_msg.data, &accord_fuel_rate);
					check_msg_timeout(ts_ms, &accord_fuel_rate.ts_ms, 
						&accord_fuel_rate.two_message_periods, 
						&accord_fuel_rate.message_timeout_counter); 
					input.fuel_rate = accord_fuel_rate.fuel_integrated_flow;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					db_clt_write(pclt,DB_ACCORD_MSG804_VAR, sizeof(accord_fuel_rate_t), &accord_fuel_rate); 
//					if(veryverbose)
//print_timestamp(stdout, &ts);
//printf("Accord fuel rate %.2f\n",
//	accord_fuel_rate.fuel_integrated_flow
//);
					break;
				case 0x410:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_410);
					check_msg_timeout(ts_ms, &accord_target_object_410.ts_ms, 
						&accord_target_object_410.two_message_periods, 
						&accord_target_object_410.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG410_VAR, sizeof(accord_target_object_t), &accord_target_object_410); 
					if(veryverbose)
						printf("410: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_410.data_valid,
							accord_target_object_410.object_relative_position_CAN5_M,
							accord_target_object_410.object_relative_distance_CAN5_M,
							accord_target_object_410.object_relative_velocity_CAN5_MPS,
							accord_target_object_410.object_counter_CAN5 
						);
					break;
				case 0x411:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_411);
					check_msg_timeout(ts_ms, &accord_target_object_411.ts_ms, 
						&accord_target_object_411.two_message_periods, 
						&accord_target_object_411.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG411_VAR, sizeof(accord_target_object_t), &accord_target_object_411); 
					if(veryverbose)
						printf("411: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_411.data_valid,
							accord_target_object_411.object_relative_position_CAN5_M,
							accord_target_object_411.object_relative_distance_CAN5_M,
							accord_target_object_411.object_relative_velocity_CAN5_MPS,
							accord_target_object_411.object_counter_CAN5 
						);
					break;
				case 0x412:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_412);
					check_msg_timeout(ts_ms, &accord_target_object_412.ts_ms, 
						&accord_target_object_412.two_message_periods, 
						&accord_target_object_412.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG412_VAR, sizeof(accord_target_object_t), &accord_target_object_412); 
					if(veryverbose)
						printf("412: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_412.data_valid,
							accord_target_object_412.object_relative_position_CAN5_M,
							accord_target_object_412.object_relative_distance_CAN5_M,
							accord_target_object_412.object_relative_velocity_CAN5_MPS,
							accord_target_object_412.object_counter_CAN5 
						);
					break;
				case 0x413:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_413);
					check_msg_timeout(ts_ms, &accord_target_object_413.ts_ms, 
						&accord_target_object_413.two_message_periods, 
						&accord_target_object_413.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG413_VAR, sizeof(accord_target_object_t), &accord_target_object_413); 
					if(veryverbose)
						printf("413: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_413.data_valid,
							accord_target_object_413.object_relative_position_CAN5_M,
							accord_target_object_413.object_relative_distance_CAN5_M,
							accord_target_object_413.object_relative_velocity_CAN5_MPS,
							accord_target_object_413.object_counter_CAN5 
						);
					break;
				case 0x414:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_414);
					check_msg_timeout(ts_ms, &accord_target_object_414.ts_ms, 
						&accord_target_object_414.two_message_periods, 
						&accord_target_object_414.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG414_VAR, sizeof(accord_target_object_t), &accord_target_object_414); 
					if(veryverbose)
						printf("414: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_414.data_valid,
							accord_target_object_414.object_relative_position_CAN5_M,
							accord_target_object_414.object_relative_distance_CAN5_M,
							accord_target_object_414.object_relative_velocity_CAN5_MPS,
							accord_target_object_414.object_counter_CAN5 
						);
					break;
				case 0x415:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_415);
					check_msg_timeout(ts_ms, &accord_target_object_415.ts_ms, 
						&accord_target_object_415.two_message_periods, 
						&accord_target_object_415.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG415_VAR, sizeof(accord_target_object_t), &accord_target_object_415); 
					if(veryverbose)
						printf("415: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_415.data_valid,
							accord_target_object_415,
							accord_target_object_415.object_relative_distance_CAN5_M,
							accord_target_object_415.object_relative_velocity_CAN5_MPS,
							accord_target_object_415.object_counter_CAN5 
						);
					break;
				case 0x416:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_416);
					check_msg_timeout(ts_ms, &accord_target_object_416.ts_ms, 
						&accord_target_object_416.two_message_periods, 
						&accord_target_object_416.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG416_VAR, sizeof(accord_target_object_t), &accord_target_object_416); 
					if(veryverbose)
						printf("416: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_416.data_valid,
							accord_target_object_416.object_relative_position_CAN5_M,
							accord_target_object_416.object_relative_distance_CAN5_M,
							accord_target_object_416.object_relative_velocity_CAN5_MPS,
							accord_target_object_416.object_counter_CAN5 
						);
					break;
				case 0x417:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_417);
					check_msg_timeout(ts_ms, &accord_target_object_417.ts_ms, 
						&accord_target_object_417.two_message_periods, 
						&accord_target_object_417.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG417_VAR, sizeof(accord_target_object_t), &accord_target_object_417); 
					if(veryverbose)
						printf("417: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_417.data_valid,
							accord_target_object_417.object_relative_position_CAN5_M,
							accord_target_object_417.object_relative_distance_CAN5_M,
							accord_target_object_417.object_relative_velocity_CAN5_MPS,
							accord_target_object_417.object_counter_CAN5 
						);
					break;
				case 0x420:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_420);
					check_msg_timeout(ts_ms, &accord_target_object_420.ts_ms, 
						&accord_target_object_420.two_message_periods, 
						&accord_target_object_420.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG420_VAR, sizeof(accord_target_object_t), &accord_target_object_420); 
					if(veryverbose)
						printf("420: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_420.data_valid,
							accord_target_object_420.object_relative_position_CAN5_M,
							accord_target_object_420.object_relative_distance_CAN5_M,
							accord_target_object_420.object_relative_velocity_CAN5_MPS,
							accord_target_object_420.object_counter_CAN5 
						);
					break;
				case 0x421:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_421);
					check_msg_timeout(ts_ms, &accord_target_object_421.ts_ms, 
						&accord_target_object_421.two_message_periods, 
						&accord_target_object_421.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG421_VAR, sizeof(accord_target_object_t), &accord_target_object_421); 
					if(veryverbose)
						printf("421: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_421.data_valid,
							accord_target_object_421.object_relative_position_CAN5_M,
							accord_target_object_421.object_relative_distance_CAN5_M,
							accord_target_object_421.object_relative_velocity_CAN5_MPS,
							accord_target_object_421.object_counter_CAN5 
						);
					break;
				case 0x422:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_422);
					check_msg_timeout(ts_ms, &accord_target_object_422.ts_ms, 
						&accord_target_object_422.two_message_periods, 
						&accord_target_object_422.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG422_VAR, sizeof(accord_target_object_t), &accord_target_object_422); 
					if(veryverbose)
						printf("422: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_422.data_valid,
							accord_target_object_422.object_relative_position_CAN5_M,
							accord_target_object_422.object_relative_distance_CAN5_M,
							accord_target_object_422.object_relative_velocity_CAN5_MPS,
							accord_target_object_422.object_counter_CAN5 
						);
					break;
				case 0x423:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_423);
					check_msg_timeout(ts_ms, &accord_target_object_423.ts_ms, 
						&accord_target_object_423.two_message_periods, 
						&accord_target_object_423.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG423_VAR, sizeof(accord_target_object_t), &accord_target_object_423); 
					if(veryverbose)
						printf("423: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_423.data_valid,
							accord_target_object_423.object_relative_position_CAN5_M,
							accord_target_object_423.object_relative_distance_CAN5_M,
							accord_target_object_423.object_relative_velocity_CAN5_MPS,
							accord_target_object_423.object_counter_CAN5 
						);
					break;
				case 0x424:
					get_accord_target_object(db_steinhoff_msg.data, &accord_target_object_424);
					check_msg_timeout(ts_ms, &accord_target_object_424.ts_ms, 
						&accord_target_object_424.two_message_periods, 
						&accord_target_object_424.message_timeout_counter); 
					db_clt_write(pclt,DB_ACCORD_MSG424_VAR, sizeof(accord_target_object_t), &accord_target_object_424); 
					if(veryverbose)
						printf("424: data_valid %d pos %.2f dist %.2f vel %.2f count %d\n",
							accord_target_object_424.data_valid,
							accord_target_object_424.object_relative_position_CAN5_M,
							accord_target_object_424.object_relative_distance_CAN5_M,
							accord_target_object_424.object_relative_velocity_CAN5_MPS,
							accord_target_object_424.object_counter_CAN5 
						);
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

			memset(&db_steinhoff_brake_out, 0, sizeof(db_steinhoff_out_t));
			if(deceleration > 0)//Braking is a nonnegative char
				accord_brake_cmd.brake_cmd = deceleration;
			else
				accord_brake_cmd.brake_cmd = output.brake_level;
			if(accord_brake_cmd.brake_cmd > 0) {
				output.throttle_pct = 0;		//if braking is requested, set throttle to 0
			}
			if(accord_brake_cmd.brake_cmd < 0)
				accord_brake_cmd.brake_cmd = 0;
			db_steinhoff_brake_out.port = BRAKE_PORT;
			db_steinhoff_brake_out.id = 0x99;
			db_steinhoff_brake_out.size = 6;
			set_accord_brake_cmd(db_steinhoff_brake_out.data, &accord_brake_cmd);
			if(verbose && ((count % 5) == 0) )
				printf("accord_can: brake %.2f  %hhx\n", accord_brake_cmd.brake_cmd, db_steinhoff_brake_out.data[2]);
			db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);

			// N.B. No delay is necessary between sending brake & acceleration commands because
			// on the Accord, these two ECUs are on different CAN buses.
			memset(&db_steinhoff_accel_out, 0, sizeof(db_steinhoff_out_t));
			if(acceleration > 0) {
				accord_accel_cmd.accel_cmd = acceleration;
				accord_brake_cmd.brake_cmd = 0;
			}
			else
				accord_accel_cmd.accel_cmd = output.throttle_pct;
			if(accord_accel_cmd.accel_cmd > 0) {
				db_steinhoff_accel_out.port = ACCEL_PORT;
				db_steinhoff_accel_out.id = 0x98;
				db_steinhoff_accel_out.size = 1;
				set_accord_accel_cmd(db_steinhoff_accel_out.data, &accord_accel_cmd);
				if(verbose && ((count % 5) == 0) )
					printf("accord_can: acceleration %hhx %.2f\n", db_steinhoff_accel_out.data[0], accord_accel_cmd.accel_cmd);
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
