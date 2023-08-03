#include "db_include.h"
#include "taurus_can.h"
#include "clt_vars_taurus.h"
#include "can_dbvars.h"
#include "long_comm.h"
#include "Leddar.h"

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
	int option;
	int count = 0;
	float acceleration = 0;
	float deceleration = 0;
	int i;

        char hostname[MAXHOSTNAMELEN];
        char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
        db_clt_typ *pclt;               /// data server client pointer
        int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
	posix_timer_typ *ptimer;
	int ipc_message_error_ctr = 0;
	trig_info_typ trig_info;
	int recv_type;

	db_steinhoff_msg_t db_steinhoff_msg;
	db_steinhoff_out_t db_steinhoff_brake_out;
	db_steinhoff_out_t db_steinhoff_accel_out;
	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms;
	int ts_now;
	int ts_sav;

	taurus_accel_cmd_t taurus_accel_cmd;
	taurus_brake_cmd_t taurus_brake_cmd;
	taurus_wheel_speed_t taurus_wheel_speed;
	taurus_vehicle_speed_513_t taurus_vehicle_speed_513;
	taurus_long_lat_accel_t taurus_long_lat_accel;
	taurus_acc_389_t taurus_acc_389;
	taurus_fuel_level_t taurus_fuel_level;
	leddar_t	leddar;
	char obd2_ctr = 0;



        while ((option = getopt(argc, argv, "va:b:")) != EOF) {
                switch(option) {
                case 'v':
                        verbose = 1;
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

printf("taurus_can: clt_trig_set OK for DB_STEINHOFF_MSG_VAR %d\n", DB_STEINHOFF_MSG_VAR);
        if (setjmp(exit_env) != 0) {
        	taurus_brake_cmd.brake_cmd = -0.01;
        	db_steinhoff_brake_out.port = BRAKE_PORT;
			db_steinhoff_brake_out.id = 0x99;
			db_steinhoff_brake_out.size = 2;
			set_taurus_brake_cmd(db_steinhoff_brake_out.data, &taurus_brake_cmd);
			db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);

			sleep(0.02);

			taurus_accel_cmd.accel_cmd = 0;
			db_steinhoff_accel_out.port = ACCEL_PORT;
			db_steinhoff_accel_out.id = 0x98;
			db_steinhoff_accel_out.size = 1;
			set_taurus_accel_cmd(db_steinhoff_accel_out.data, &taurus_accel_cmd);
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
	memset(&taurus_accel_cmd, 0, sizeof(taurus_accel_cmd));
	memset(&taurus_brake_cmd, 0, sizeof(taurus_brake_cmd));
	memset(&taurus_wheel_speed, 0, sizeof(taurus_wheel_speed_t));

	taurus_accel_cmd.two_message_periods = 40; 		// 2*20 msec
	taurus_brake_cmd.two_message_periods = 40; 		// 2*20 msec
	taurus_wheel_speed.two_message_periods = 80;


	db_clt_write(pclt, DB_TAURUS_MSG98_VAR, sizeof(taurus_accel_cmd_t), &taurus_accel_cmd); 
	db_clt_write(pclt, DB_TAURUS_MSG99_VAR, sizeof(taurus_brake_cmd_t), &taurus_brake_cmd); 
	db_clt_write(pclt, DB_TAURUS_MSG215_VAR, sizeof(taurus_wheel_speed_t), &taurus_wheel_speed);

	if(acceleration == 0)
		taurus_accel_cmd.accel_cmd = 0;
	if(deceleration == 0)
		taurus_brake_cmd.brake_cmd = 0;
	get_current_timestamp(&ts);
	ts_now = TS_TO_MS(&ts);
	ts_sav = ts_now;

	for(;;) {
	
		/* Now wait for a trigger. If the trigger is from the Komodo, read and parse the message. 
		** If the trigger is from the timer, it's time to send the acceleration message */
	
		recv_type = clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
		if( DB_TRIG_VAR(&trig_info) == DB_STEINHOFF_MSG_VAR ) {
			count++;
			memset(&db_steinhoff_msg.data[0], 0, 8);
			db_clt_read(pclt, DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);
			switch(db_steinhoff_msg.id) {
				case 0x92:
					get_taurus_long_lat_accel(db_steinhoff_msg.data, &taurus_long_lat_accel);
					check_msg_timeout(ts_ms, &taurus_long_lat_accel.ts_ms, 
						&taurus_long_lat_accel.two_message_periods, 
						&taurus_long_lat_accel.message_timeout_counter); 
					db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					input.vehicle_accel_mps2 = taurus_long_lat_accel.longitudinal_acceleration;
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					if(verbose)
						print_timestamp(stdout, &ts);
					if(verbose)
						printf("Taurus longitudinal acceleration %.2f lateral %.2f vertical %.2f\n", 
							input.vehicle_accel_mps2, output.brake_level,output.throttle_pct);
					break;
				case 0x7E8:
					get_taurus_fuel_level(db_steinhoff_msg.data, &taurus_fuel_level);
					check_msg_timeout(ts_ms, &taurus_fuel_level.ts_ms,
						&taurus_fuel_level.two_message_periods,
						&taurus_fuel_level.message_timeout_counter);
					input.fuel_rate = taurus_fuel_level.fuel_level;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					if(verbose)
						print_timestamp(stdout, &ts);
					if(verbose)
						printf("Taurus fuel level %.1f\n", taurus_fuel_level.fuel_level);
					break;

				case 513:
					get_taurus_vehicle_speed_513(db_steinhoff_msg.data, &taurus_vehicle_speed_513);
					check_msg_timeout(ts_ms, &taurus_vehicle_speed_513.ts_ms,
						&taurus_vehicle_speed_513.two_message_periods,
						&taurus_vehicle_speed_513.message_timeout_counter);
					db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					input.vehicle_speed_mps = taurus_vehicle_speed_513.vehicle_speed_kph/3.6;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					if(verbose)
						print_timestamp(stdout, &ts);
					if(verbose)
						printf("Taurus vehicle speed %.5f\n", input.vehicle_speed_mps);
					break;
				case 0x215:
					get_taurus_wheel_speed(db_steinhoff_msg.data, &taurus_wheel_speed);
					check_msg_timeout(ts_ms, &taurus_wheel_speed.ts_ms, 
						&taurus_wheel_speed.two_message_periods, 
						&taurus_wheel_speed.message_timeout_counter); 
//					input.vehicle_speed_mps = taurus_wheel_speed.average_speed;
//					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
//					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					if(verbose)
						print_timestamp(stdout, &ts);
					if(verbose)
						printf("Taurus average speed %.5f input.vehicle_speed_mps %.4f output.brake_level %.2f output.throttle_pct %.2f\n", taurus_wheel_speed.average_speed, input.vehicle_speed_mps, output.brake_level,output.throttle_pct);
					break;
				case 389:
					get_taurus_acc_389(db_steinhoff_msg.data, &taurus_acc_389);
					check_msg_timeout(ts_ms, &taurus_acc_389.ts_ms, 
						&taurus_acc_389.two_message_periods, 
						&taurus_acc_389.message_timeout_counter); 
//					db_clt_write(pclt,DB_TAURUS_MSG48_VAR, sizeof(taurus_acc_389_t), &taurus_acc_389);
					if(verbose)
						print_timestamp(stdout, &ts);
					if(verbose)
						printf("389: target speed kph %.2f mph %.2f desired acceleration %.2f desired braking %.2f Cmbb_B_Enbl %d CmbbDeny_B_Actl %d CmbbOvrrd_B_RqDrv %d AccDeny_B_Rq %d AccBrkDecel_B_Rq %d\n",
						taurus_acc_389.target_speed_kph,
						taurus_acc_389.target_speed_kph * 0.621371192,
						taurus_acc_389.AccPrpl_A_Rq_desired_accel,
						taurus_acc_389.AccBrkTot_A_Rq_desired_decel,
						taurus_acc_389.Cmbb_B_Enbl,
						taurus_acc_389.CmbbDeny_B_Actl,
						taurus_acc_389.CmbbOvrrd_B_RqDrv,
						taurus_acc_389.AccDeny_B_Rq,
						taurus_acc_389.AccBrkDecel_B_Rq
						);
					break;
				default:
//					if(verbose)
//						printf("Unknown message %#lX received\n", db_steinhoff_msg.id);
					break;
			}
		}

		get_current_timestamp(&ts);
		ts_now = TS_TO_MS(&ts);

		if(ts_now - ts_sav >= 15){
			ts_sav = ts_now;

			db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);

			memset(&db_steinhoff_brake_out, 0, sizeof(db_steinhoff_out_t));
			if(deceleration < 0)
				taurus_brake_cmd.brake_cmd = deceleration;
			else
				taurus_brake_cmd.brake_cmd = output.brake_level;
			if(taurus_brake_cmd.brake_cmd < 0)
				output.throttle_pct = 0;		//if braking is requested, set throttle to 0
			if((++obd2_ctr % 3) == 0) { //Send OBD2 fuel level poll
				db_steinhoff_brake_out.port = BRAKE_PORT;
				db_steinhoff_brake_out.id = 0x7DF;
				db_steinhoff_brake_out.size = 8;
				db_steinhoff_brake_out.data[0] = 2;
				db_steinhoff_brake_out.data[1] = 1;
				db_steinhoff_brake_out.data[2] = 0x2F;
				db_steinhoff_brake_out.data[3] = 0xCC;
				db_steinhoff_brake_out.data[4] = 0xCC;
				db_steinhoff_brake_out.data[5] = 0xCC;
				db_steinhoff_brake_out.data[6] = 0xCC;
				db_steinhoff_brake_out.data[7] = 0xCC;
				db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);
				if(verbose)
					print_timestamp(stdout, &ts);
				if(verbose)
					printf("Sending OBD2 fuel poll\n");
				usleep(1000);
				usleep(1000);
			}
			if((obd2_ctr % 40000) == 0) {//Send OBD2 clear errors command
				db_steinhoff_brake_out.port = BRAKE_PORT;
				db_steinhoff_brake_out.id = 0x7DF;
				db_steinhoff_brake_out.size = 8;
				db_steinhoff_brake_out.data[0] = 1;
				db_steinhoff_brake_out.data[1] = 4;
				db_steinhoff_brake_out.data[2] = 0xCC;
				db_steinhoff_brake_out.data[3] = 0xCC;
				db_steinhoff_brake_out.data[4] = 0xCC;
				db_steinhoff_brake_out.data[5] = 0xCC;
				db_steinhoff_brake_out.data[6] = 0xCC;
				db_steinhoff_brake_out.data[7] = 0xCC;
				db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);

				if(verbose)
					print_timestamp(stdout, &ts);
				if(verbose)
					printf("Sending OBD2 clear errors command obd2_ctr %d\n", obd2_ctr);
				usleep(1000);
			}
			db_steinhoff_brake_out.id = 0x99;
			db_steinhoff_brake_out.size = 2;
			set_taurus_brake_cmd(db_steinhoff_brake_out.data, &taurus_brake_cmd);
			if(verbose)
				print_timestamp(stdout, &ts);
			if(verbose)
				printf("taurus_can: taurus_brake_cmd.brake_cmd %.2f db_steinhoff_out.data[0] %hhx db_steinhoff_out.data[1] %hhx\n", taurus_brake_cmd.brake_cmd, db_steinhoff_brake_out.data[0], db_steinhoff_brake_out.data[1]);
			db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);

			// N.B. No delay is necessary between sending brake & acceleration commands because
			// on the Accord, these two ECUs are on different CAN buses.
			memset(&db_steinhoff_accel_out, 0, sizeof(db_steinhoff_out_t));
			if(acceleration > 0) {
				taurus_accel_cmd.accel_cmd = acceleration;
				taurus_brake_cmd.brake_cmd = 0;
			}
			else
				taurus_accel_cmd.accel_cmd = output.throttle_pct;
			db_steinhoff_accel_out.port = ACCEL_PORT;
			db_steinhoff_accel_out.id = 0x98;
			db_steinhoff_accel_out.size = 1;
			set_taurus_accel_cmd(db_steinhoff_accel_out.data, &taurus_accel_cmd);
			db_clt_write(pclt, DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);

			if(verbose){
				print_timestamp(stdout, &ts);
				printf("taurus_can: db_steinhoff_out.data[0] %hhx acceleration %.2f\n", db_steinhoff_accel_out.data[0], taurus_accel_cmd.accel_cmd);
				db_clt_read(pclt, DB_LEDDAR_2_VAR, sizeof(leddar_t), &leddar);
				printf("leddar: ");
				for(i=0; i<16; i++)
					printf("%.2f ", leddar.distance[i]);
				printf("\n");
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
