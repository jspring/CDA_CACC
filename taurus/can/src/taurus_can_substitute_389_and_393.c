#include "db_include.h"
#include "taurus_can.h"
#include "clt_vars_taurus.h"
#include "can_dbvars.h"
#include "long_comm.h"

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
        DB_STEINHOFF_MSG_VAR, DB_OUTPUT_VAR
};

int num_steinhoff_trig_variables = sizeof(steinhoff_trig_list)/sizeof(int);

int main(int argc, char *argv[]) {
	int verbose = 0;
	int option;
	int count = 0;
	float acceleration = 0;
	int i;

        char hostname[MAXHOSTNAMELEN];
        char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
        db_clt_typ *pclt;               /// data server client pointer
        int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
	posix_timer_typ *ptimer;
	int ipc_message_error_ctr = 0;
	trig_info_typ trig_info;
	int recv_type;

	db_steinhoff_msg_t db_car_can_in;
	db_steinhoff_out_t db_car_can_out;
//	db_steinhoff_out_t db_ccm_can_out;
	db_steinhoff_msg_t db_ccm_can_in;
	db_steinhoff_out_t db_steinhoff_accel_out;
	taurus_target_data_t taurus_target_data_1;
	taurus_target_data_t taurus_target_data_2;
	taurus_vehicle_speed_513_t taurus_vehicle_speed_513;
	input_t input;
	output_t output;
	veh_comm_packet_t comm_pkt;
	timestamp_t ts;
	int ts_ms;
	int ts_now;
	int ts_sav;

	taurus_accel_cmd_t taurus_accel_cmd;
	taurus_wheel_speed_t taurus_wheel_speed;
	taurus_long_lat_accel_t taurus_long_lat_accel;
	taurus_acc_389_t taurus_acc_389;
	taurus_acc_393_t taurus_acc_393;
	memset(&output, 0, sizeof(output_t));

        while ((option = getopt(argc, argv, "va:b:")) != EOF) {
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
        printf("Got to 1\n");
        printf("Got to 2\n");
	get_local_name(hostname, MAXHOSTNAMELEN);
	if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
//			NULL, 0, steinhoff_trig_list, num_steinhoff_trig_variables))
			NULL, 0, NULL, 0))
		== NULL) {
			exit(EXIT_FAILURE);
	}

	if (clt_trig_set( pclt, DB_CCM_CAN_IN_VAR, DB_CCM_CAN_IN_TYPE) == FALSE )
		exit(EXIT_FAILURE);
	if (clt_trig_set( pclt, DB_CAR_CAN_OUT_VAR, DB_CAR_CAN_OUT_TYPE) == FALSE )
		exit(EXIT_FAILURE);
	if (clt_trig_set( pclt, DB_CAR_CAN_IN_VAR, DB_CAR_CAN_IN_TYPE) == FALSE )
		exit(EXIT_FAILURE);
	if (clt_trig_set( pclt, DB_STEINHOFF_MSG_VAR, DB_STEINHOFF_MSG_TYPE) == FALSE )
		exit(EXIT_FAILURE);

printf("taurus_can: clt_trig_set OK for DB_STEINHOFF_MSG_VAR %d\n", DB_STEINHOFF_MSG_VAR);
        if (setjmp(exit_env) != 0) {
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

//        if ((ptimer = timer_init( 20, ChannelCreate(0) )) == NULL) {
//                fprintf(stderr, "Unable to initialize delay timer\n");
//                exit(EXIT_FAILURE);
//        }

	/* Zero data structures */
	memset(&taurus_accel_cmd, 0, sizeof(taurus_accel_cmd));
	memset(&taurus_wheel_speed, 0, sizeof(taurus_wheel_speed_t));

	taurus_accel_cmd.two_message_periods = 40; 		// 2*20 msec
	taurus_wheel_speed.two_message_periods = 80;


	db_clt_write(pclt, DB_TAURUS_MSG98_VAR, sizeof(taurus_accel_cmd_t), &taurus_accel_cmd); 
	db_clt_write(pclt, DB_TAURUS_MSG215_VAR, sizeof(taurus_wheel_speed_t), &taurus_wheel_speed);

	if(acceleration == 0)
		taurus_accel_cmd.accel_cmd = 0;
	get_current_timestamp(&ts);
	ts_now = TS_TO_MS(&ts);
	ts_sav = ts_now;

	for(;;) {
	
		/* Now wait for a trigger. If the trigger is from the Komodo, read and parse the message. 
		** If the trigger is from the timer, it's time to send the acceleration message */
	
		recv_type= clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
		if( DB_TRIG_VAR(&trig_info) == DB_CAR_CAN_IN_VAR) {
			count++;
//			memset(&db_ccm_can_out, 0, sizeof(db_steinhoff_out_t));
			db_clt_read(pclt, DB_CAR_CAN_IN_VAR, sizeof(db_steinhoff_msg_t), &db_car_can_in);
			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);

			//Zero out the output variable and then set its id and size from the input variable
			//The data payload will be set below
//			db_ccm_can_out.size = db_car_can_in.size;
//			db_ccm_can_out.id = db_car_can_in.id;

			switch(db_car_can_in.id) {
				case 0x47:
//					get_taurus_target_data(db_car_can_in.data, &taurus_target_data_1);
					check_msg_timeout(ts_ms, &taurus_target_data_1.ts_ms, 
						&taurus_target_data_1.two_message_periods, 
						&taurus_target_data_1.message_timeout_counter); 
					db_clt_write(pclt,DB_TAURUS_MSG47_VAR, sizeof(taurus_target_data_t), &taurus_target_data_1);
					if(verbose)
						printf("47: target1_data %.2f\n",
							taurus_target_data_1.target1_data
						);
					break;
				case 0x48:
//					get_taurus_target_data(db_car_can_in.data, &taurus_target_data_2);
					check_msg_timeout(ts_ms, &taurus_target_data_2.ts_ms, 
						&taurus_target_data_2.two_message_periods, 
						&taurus_target_data_2.message_timeout_counter); 
					db_clt_write(pclt,DB_TAURUS_MSG48_VAR, sizeof(taurus_target_data_t), &taurus_target_data_2);
					if(verbose)
						printf("48: target2_data %.2f\n",
							taurus_target_data_2.target1_data
						);
					break;
				case 0x92:
					get_taurus_long_lat_accel(db_car_can_in.data, &taurus_long_lat_accel);
					check_msg_timeout(ts_ms, &taurus_long_lat_accel.ts_ms, 
						&taurus_long_lat_accel.two_message_periods, 
						&taurus_long_lat_accel.message_timeout_counter); 
					input.vehicle_accel_mps2 = taurus_long_lat_accel.longitudinal_acceleration;
					db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					db_clt_read(pclt, DB_COMM_TX_VAR, sizeof(veh_comm_packet_t), &comm_pkt);
					comm_pkt.accel = taurus_long_lat_accel.longitudinal_acceleration;
					db_clt_write(pclt, DB_COMM_TX_VAR, sizeof(veh_comm_packet_t), &comm_pkt);
					if(verbose)
						printf("Taurus longitudinal acceleration %.2f lateral %.2f vertical %.2f\n", 
							input.vehicle_accel_mps2, output.brake_level,output.throttle_pct);
					break;
				case 0x215:
					get_taurus_wheel_speed(db_car_can_in.data, &taurus_wheel_speed);
					check_msg_timeout(ts_ms, &taurus_wheel_speed.ts_ms, 
						&taurus_wheel_speed.two_message_periods, 
						&taurus_wheel_speed.message_timeout_counter); 
					input.vehicle_speed_mps = taurus_wheel_speed.wheel_speed_FL;
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input); 
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					db_clt_read(pclt, DB_COMM_TX_VAR, sizeof(veh_comm_packet_t), &comm_pkt);
					comm_pkt.velocity = taurus_wheel_speed.wheel_speed_FL;
					db_clt_write(pclt, DB_COMM_TX_VAR, sizeof(veh_comm_packet_t), &comm_pkt);
					if(verbose)
						printf("Taurus wheel speed FL %.2f FR %.2f RL %.2f RR %.2f input.vehicle_speed_mps %.2f output.brake_level %.2f output.throttle_pct %.2f\n", taurus_wheel_speed.wheel_speed_FL, taurus_wheel_speed.wheel_speed_FR, taurus_wheel_speed.wheel_speed_RL, taurus_wheel_speed.wheel_speed_RR, input.vehicle_speed_mps, output.brake_level,output.throttle_pct);
					break;
				case 513:
					get_taurus_vehicle_speed_513(db_car_can_in.data, &taurus_vehicle_speed_513);
					if(verbose)
						printf("Taurus vehicle speed %.2f\n",taurus_vehicle_speed_513.vehicle_speed_kph);
				default:
//					if(verbose)
//						printf("Unknown message %#lX received\n", db_car_can_in.id);
					break;
			}
//			memcpy(&db_ccm_can_out.data[0], &db_car_can_in.data[0], 8);
//			db_clt_write(pclt, DB_CCM_CAN_OUT_VAR, sizeof(db_steinhoff_out_t), &db_ccm_can_out);
/*
			if(verbose){
				printf("CCM CAN output: dbnum %d id %d size %d ", 
					DB_CCM_CAN_OUT_VAR,
					db_ccm_can_out.id,
					db_ccm_can_out.size);
				for(i=0 ; i<8; i++)
					printf("%#hhx ", db_ccm_can_out.data[i]);
				for(i=0 ; i<8; i++)
					printf("%#hhx ", db_car_can_in.data[i]);
				printf("\n");
			}
*/
		}
		if( DB_TRIG_VAR(&trig_info) == DB_CCM_CAN_IN_VAR) {
			count++;
			db_clt_read(pclt, DB_CCM_CAN_IN_VAR, sizeof(db_steinhoff_msg_t), &db_ccm_can_in);
			db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);

			//Zero out the output variable and then set its id and size from the input variable
			//The data payload will be set below
			memset(&db_car_can_out, 0, sizeof(db_steinhoff_out_t));
			db_car_can_out.size = db_ccm_can_in.size;
			db_car_can_out.id = db_ccm_can_in.id;

			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);
			ts_sav = ts_now;

			switch(db_ccm_can_in.id) {
				case 389:
					get_taurus_acc_389(db_ccm_can_in.data, &taurus_acc_389);
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					check_msg_timeout(ts_ms, &taurus_acc_389.ts_ms, 
						&taurus_acc_389.two_message_periods, 
						&taurus_acc_389.message_timeout_counter); 
//					db_clt_write(pclt,DB_TAURUS_MSG389_VAR, sizeof(taurus_acc_389_t), &taurus_acc_389);
					if(verbose)
						printf("389: target speed kph %.2f desired acceleration %.2f desired braking %.2f Cmbb_B_Enbl %d CmbbDeny_B_Actl %d CmbbOvrrd_B_RqDrv %d AccDeny_B_Rq %d AccBrkDecel_B_Rq %d\n",
						taurus_acc_389.target_speed_kph,
						taurus_acc_389.AccPrpl_A_Rq_desired_accel,
						taurus_acc_389.AccBrkTot_A_Rq_desired_decel,
						taurus_acc_389.Cmbb_B_Enbl,
						taurus_acc_389.CmbbDeny_B_Actl,
						taurus_acc_389.CmbbOvrrd_B_RqDrv,
						taurus_acc_389.AccDeny_B_Rq,
						taurus_acc_389.AccBrkDecel_B_Rq
						);
					if(output.brake_level < 0) {
						output.throttle_pct = 0;		//if braking is requested, set throttle to 0
						set_taurus_acc_389(&db_car_can_in.data[0], output.brake_level, taurus_vehicle_speed_513.vehicle_speed_kph);
					}
					break;
				case 393:
					get_taurus_acc_393(db_ccm_can_in.data, &taurus_acc_393);
					db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
					check_msg_timeout(ts_ms, &taurus_acc_393.ts_ms, 
						&taurus_acc_393.two_message_periods, 
						&taurus_acc_393.message_timeout_counter); 
//					db_clt_write(pclt,DB_TAURUS_MSG393_VAR, sizeof(taurus_acc_393_t), &taurus_acc_393);
					if(verbose)
						printf("393: AccPrpl_V_Rq_veh_speed kph %.2f\n",
						taurus_acc_393.AccPrpl_V_Rq_veh_speed
						);
					if(output.brake_level < 0) {
						output.throttle_pct = 0;		//if braking is requested, set throttle to 0
						set_taurus_acc_393(&db_car_can_in.data[0], output.brake_level, taurus_vehicle_speed_513.vehicle_speed_kph);
					}
					break;
			}
			memcpy(&db_car_can_out.data[0], &db_ccm_can_in.data[0], 8);
			db_clt_write(pclt, DB_CAR_CAN_OUT_VAR, sizeof(db_steinhoff_out_t), &db_car_can_out);
			if(verbose){
				printf("CAR CAN output: dbnum %d id %d size %d ", 
					DB_CAR_CAN_OUT_VAR,
					db_car_can_out.id,
					db_car_can_out.size);
				for(i=0 ; i<8; i++)
					printf("%#hhx ", db_car_can_out.data[i]);
				for(i=0 ; i<8; i++)
					printf("%#hhx ", db_ccm_can_in.data[i]);
				printf("\n");
			}
		}



/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Pseudo Code for Panda Man-in-the-Middle

% variable description:
% accel = acceleration command in m/s^2, This is the CAN acceleration override command we would send in a message with Arb ID 99 and a 2 byte signed 2's complement integer signal with scale: {Raw Value}*0.001
% veh_spd = current vehicle speed from CAN bus in kph (Veh_V_ActlEng in dbc file)
% decel_calc = calculated coastdown deceleration rate [m/s^2] based on function of current vehicle speed. decel_calc = f(veh_spd)
% decel_calc = -0.003713 * veh_spd - 0.032923

               - set 'AccDeny_B_Rq' = 0
               - set 'AccCancl_B_Rq' = 0

if accel >= 0        % positive acceleration requested
               389 AccPrpl_A_Rq = accel
               389 AccBrkTot_A_Rq = 0
               389 AccBrkPrchg_B_Rq = 0
               389 AccBrkDecel_B_Rq = 0
               393 Acc_Resume_Active = 0
               393 Acc_Autobrake_Cancel = 0
               % gateway all other signals

elseif accel < 0 && accel > decel_calc       % negative acceleration requested AND requested acceleration rate is higher (less negative) than coastdown deceleration rate at current vehicle speed
               % use AccPrpl_A_Rq to command negative acceleration which requires some accelerator input to achieve
               % example: very low deceleration rate at 70 mph requires only backing off of the accelerator but not completely taking foot off which will cause a greater deceleration rate
               389 AccPrpl_A_Rq = accel
               389 AccBrkTot_A_Rq = 0
               389 AccBrkPrchg_B_Rq = 0
               389 AccBrkDecel_B_Rq = 0
               393 Acc_Resume_Active = 0
               393 Acc_Autobrake_Cancel = 0
               % gateway all other signals

elseif accel < 0 && accel < decel_calc % negative acceleration requested AND requested acceleration rate is lower (more negative) than coastdown deceleration rate at current vehicle speed
               % brake application is required to slow the car down quicker than coastdown only.
               389 AccBrkTot_A_Rq = accel
               389 AccPrpl_A_Rq = -5
               389 AccBrkPrchg_B_Rq = 1
               389 AccBrkDecel_B_Rq = 1
               393 AccPrpl_V_Rq = veh_spd
               393 Acc_Resume_Active = 0
               393 Acc_Autobrake_Cancel = 0
               % gateway all other signals
end

** Also set AccDeny_B_Rq and AccCancl_B_Rq both to zero as well to avoid not being able to turn on ACC if fault codes are present on the car.
*/
		get_current_timestamp(&ts);
		ts_ms = TS_TO_MS(&ts);
		if(ts_sav - ts_now > 15) {
			ts_sav = ts_now;
			// N.B. No delay is necessary between sending brake & acceleration commands because
			// on the Accord, these two ECUs are on different CAN buses.
			memset(&db_steinhoff_accel_out, 0, sizeof(db_steinhoff_out_t));
			if(acceleration > 0) {
				taurus_accel_cmd.accel_cmd = acceleration;
			}
			else
				taurus_accel_cmd.accel_cmd = output.throttle_pct;
			db_steinhoff_accel_out.port = ACCEL_PORT;
			db_steinhoff_accel_out.id = 0x98;
			db_steinhoff_accel_out.size = 1;
			set_taurus_accel_cmd(db_steinhoff_accel_out.data, &taurus_accel_cmd);
			if(verbose)
				printf("taurus_can: db_steinhoff_out.data[0] %hhx acceleration %.2f\n", db_steinhoff_accel_out.data[0], taurus_accel_cmd.accel_cmd);
			db_clt_write(pclt, DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_accel_out);
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
