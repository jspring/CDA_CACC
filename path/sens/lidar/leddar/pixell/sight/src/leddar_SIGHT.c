/* neweagle_steering.c - db_slv client that passes sensor data to the database
**	and forwards control signals to the CAN driver 
**
**
**
*/

#include "db_include.h"
#include "neweagle_steering.h"
#include "leddar_SIGHT.h"
#include "clt_vars_neweagle.h"
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

const char *usage = "-v verbose -a <angle> -w veryverbose";

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
	int i;

	char hostname[MAXHOSTNAMELEN];
	char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
	db_clt_typ *pclt;               /// data server client pointer
	int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
	posix_timer_typ *ptimer;
	int ipc_message_error_ctr = 0;
	trig_info_typ trig_info;

	db_steinhoff_msg_t db_steinhoff_msg;
	db_steinhoff_out_t db_steinhoff_steering_out;
	neweagle_steering_report_t neweagle_steering_report;
	neweagle_steering_request_t neweagle_steering_request = {0};
	neweagle_dbw_misc_t dbw_misc;
	char SteerCtrlEnblReqCounter = 0; //Counter for toggling SteerCtrlEnblReq in case there's a fault
	int steering_angle_or_torque_request_sav = 0;
	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms = 0;
	int ts_now = 0;
	int ts_sav = 0;
	int ts_sav2 = 0;
	int readonly = 0;
	int int_msg_id = 0;
	int clrq = 0;
	unsigned char counter_sav = 0;
	int read_CAN_dbnum = DB_STEINHOFF_MSG_VAR;
	int print_low_level_text = 0;
	leddar_SIGHT_detection_t sight_detection;
printf("Got to 1\n");
	// Set output steering request to large negative number
	memset(&output, 0, sizeof(output_t));

        while ((option = getopt(argc, argv, "vwa:t:rn:x")) != EOF) {
                switch(option) {
                case 'v':
                        verbose = 1;
                        break;
                case 'w':
                        veryverbose = 1;
                        verbose = 1;
                        break;
                case 'a':
                		output.steering_angle_request = atof(optarg);
                		output.steering_angle_or_torque_request = 1;
                		clrq = 1;
                        break;
                case 't':
                	output.steering_torque_pct_request = atof(optarg);
                	output.steering_angle_or_torque_request = -1;
                	clrq = 1;
                	break;
                case 'r':
                	readonly = 1;
                	break;
                case 'n':
                	read_CAN_dbnum = atoi(optarg);
                	break;
                case 'x':
                	print_low_level_text = 1;
                	break;
                default:
                	printf("Usage: %s %s\n", argv[0], usage);
                	exit(EXIT_FAILURE);
                	break;
                }
        }
        printf("Got to 2\n");

    	get_local_name(hostname, MAXHOSTNAMELEN);
    	if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
    			NULL, 0, NULL, 0))
    		== NULL) {
    			printf("Could not db_list_init\n");
    			exit(EXIT_FAILURE);
    	}
    	printf("Got to 3\n");

    	if (clt_trig_set( pclt, read_CAN_dbnum, read_CAN_dbnum) == FALSE )
    		exit(EXIT_FAILURE);
    	printf("%s: clt_trig_set OK for read_CAN_dbnum %d\n", argv[0], read_CAN_dbnum);

    	if (setjmp(exit_env) != 0) {

			neweagle_steering_report.SteeringEnabled = 0;
			neweagle_steering_report.MiscByWireReady = 1;
			neweagle_steering_report.SteeringDriverActivity = 1;
			neweagle_steering_report.SteeringFault = 1;
			input.overheat_prevent = neweagle_steering_report.OverheatPreventMode;
			input.overheat_warning = neweagle_steering_report.SteeringOverheatWarning;
			input.heartbeat= (neweagle_steering_report.SteeringRollingCntr == ((counter_sav + 1) % 16 )) ? 1 : 0;
			counter_sav = neweagle_steering_report.SteeringRollingCntr;
                                db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
			db_clt_write(pclt, DB_STEERING_REPORT_VAR, sizeof(neweagle_steering_report_t), &neweagle_steering_report);

		//Disable steering control
    		memset(&db_steinhoff_steering_out, 0, sizeof(db_steinhoff_out_t));
    		db_steinhoff_steering_out.id = 0x2F03;
    		db_steinhoff_steering_out.size = 8;
    		db_clt_write(pclt, DB_STEINHOFF_STEERING_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_steering_out);
/*

		neweagle_steering_request.SteerCtrlEnblReq = 0;
		set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, 0);
		neweagle_steering_request.SteeringChecksum = Compute_CRC8(7, &db_steinhoff_steering_out.data[0]);
		set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, veryverbose);
    		db_clt_write(pclt, DB_STEINHOFF_STEERING_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_steering_out);
    		*/
			input.misc_comm_fault = 1;
			input.misc_by_wire_ready = 0;
			input.driver_override = 1;
			input.steering_fault = 1;
            db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);

            printf("Got to 4\n");

		//Sleep for 100 ms to give CAN driver enough time to send steering disable
		usleep(100000);

        	db_list_done(pclt, NULL, 0, NULL, 0);
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
	memset(&neweagle_steering_report, 0, sizeof(neweagle_steering_report_t));
	memset(&neweagle_steering_request, 0, sizeof(neweagle_steering_request_t));
	memset(&input, 0, sizeof(input_t));
	input.steering_enabled = -1;
	input.driver_override = -1;
	input.steering_fault = -1;
	input.overheat_prevent = -1;
	input.overheat_warning = -1;
	input.misc_by_wire_ready = -1;
	input.misc_comm_fault = -1;
	input.heartbeat= -1;


	neweagle_steering_report.two_message_periods = 80; 		// 2*10 msec
	neweagle_steering_request.two_message_periods = 80; 		// 2*10 msec

	//Initialize database structs
	db_clt_write(pclt, DB_STEERING_REPORT_VAR, sizeof(neweagle_steering_report_t), &neweagle_steering_report); 
	db_clt_write(pclt, DB_STEERING_REQUEST_VAR, sizeof(neweagle_steering_request_t), &neweagle_steering_request); 
	db_clt_write(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
	db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);

	get_current_timestamp(&ts);
	ts_sav2 = ts_sav = ts_now = TS_TO_MS(&ts);
	printf("Got to 5\n");

	for(;;) {
		printf("Got to 6\n");

		clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
		if( DB_TRIG_VAR(&trig_info) == read_CAN_dbnum) {
			count++;

			memset(&db_steinhoff_msg.data[0], 0, 8);
			db_clt_read(pclt, read_CAN_dbnum, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);
//			printf("Got to 7 db_steinhoff_msg.id %#hx verbose %d veryverbose %d print_low_level_text %d\n", 
//					db_steinhoff_msg.id,
//					verbose,
//					veryverbose,
//					print_low_level_text
//					);
			int_msg_id = (int)db_steinhoff_msg.id;

			switch(int_msg_id) {
			  case 0x0751:
				  if(print_low_level_text) {
					print_timestamp(stdout, &ts);
					if(db_steinhoff_msg.data[0] == 0) {
						printf("Fault text id %d num pages %d status %d\n",
//                  	printf("0: %d %d %d\n",
								(db_steinhoff_msg.data[1] << 8) + db_steinhoff_msg.data[2],
								db_steinhoff_msg.data[5],
								db_steinhoff_msg.data[6]
							);
					}
					else
						if(db_steinhoff_msg.data[0] < 254)
							printf("Num detections %d\n",db_steinhoff_msg.data[0]);
					break;
			  case 0x0750:
			  case 0x0752:
			  case 0x0753:
			  case 0x0754:
			  case 0x0755:
			  case 0x0756:
			  case 0x0757:
				  parse_SIGHT_detections(db_steinhoff_msg.data, &sight_detection, veryverbose);
						check_msg_timeout(ts_ms, &sight_detection.ts_ms,
								&sight_detection.two_message_periods,
								&sight_detection.message_timeout_counter);
//                                    db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					if(verbose != 0) {
						get_current_timestamp(&ts);
						print_timestamp(stdout, &ts);
						if(veryverbose != 0) {
							for(i=0; i<8 ; i++)
								printf("%#hhx ", db_steinhoff_msg.data[i]);
							printf("\n");
						}
						printf("main:parse_SIGHT_detections: distance1 %.2f amplitude1 %.2f channel1 %hhu distance2 %.2f amplitude2 %.2f channel2 %hhu\n",
							sight_detection.distance1,
							sight_detection.amplitude1,
							sight_detection.channel1,
							sight_detection.distance2,
							sight_detection.amplitude2,
							sight_detection.channel2
						);
					   }

//                                        db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
//                                        db_clt_write(pclt, DB_LEDDAR_SIGHT_DETECTION_VAR, sizeof(leddar_SIGHT_detection_t), &sight_detection);
						break;
				default:
//					printf("Unknown message %#hx received\n", db_steinhoff_msg.id);
						break;
		}

		}
//		printf("Got to 8\n");

		get_current_timestamp(&ts);
//		ts_now = TS_TO_MS(&ts);
		ts_ms = TS_TO_MS(&ts);

//printf("trig %d\n", DB_TRIG_VAR(&trig_info));

//		if((ts_ms - ts_sav >= 20) && (readonly == 0)){
////			printf("1:ts_ms %d ts_sav %d diff %d\n", ts_ms, ts_sav, ts_ms-ts_sav);
//			ts_sav = ts_ms;
////			printf("2:ts_ms %d ts_sav %d diff %d\n", ts_ms, ts_sav, ts_ms-ts_sav);
//			if(clrq == 0) {
//				if( db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output) == FALSE) {
//					printf("neweagle_steering: clt_read of output variable failed. Exiting....");
//					exit(EXIT_FAILURE);
//				}
//			}
//
//			if( (output.steering_angle_or_torque_request == 1) ||(output.steering_angle_or_torque_request == -1)) {
//				steering_angle_or_torque_request_sav = output.steering_angle_or_torque_request;
//				if(output.steering_angle_or_torque_request == 1) {
//				    if((output.steering_angle_request > -820) && (output.steering_angle_request < 820)) {
//					neweagle_steering_request.SteerRollingCtr++;
//					if(neweagle_steering_request.SteerRollingCtr > 15)
//						neweagle_steering_request.SteerRollingCtr = 0;
//					neweagle_steering_request.SteeringWhlAngleVelocityLim = 100;
//					neweagle_steering_request.SteeringReqType = 1;
//					neweagle_steering_request.SteeringWhlIgnoreDriverOvrd = 0;
//					neweagle_steering_request.SteeringWhlAngleReq = output.steering_angle_request;
//					memset(&db_steinhoff_steering_out, 0, sizeof(db_steinhoff_out_t));
//					db_steinhoff_steering_out.id = 0x2F03;
//					db_steinhoff_steering_out.size = 8;
////					db_steinhoff_steering_out.ts = ts;
//				    }
//				}
//				else
//				if(output.steering_angle_or_torque_request == -1) {
//				    if((output.steering_torque_pct_request > -100) && (output.steering_torque_pct_request < 100)) {
//					neweagle_steering_request.SteerRollingCtr++;
//					if(neweagle_steering_request.SteerRollingCtr > 15)
//						neweagle_steering_request.SteerRollingCtr = 0;
//					neweagle_steering_request.SteeringWhlAngleVelocityLim = 100;
//					neweagle_steering_request.SteeringReqType = 0;
//					neweagle_steering_request.SteeringWhlIgnoreDriverOvrd = 0;
//					neweagle_steering_request.SteeringWhlPcntTrqReq = output.steering_torque_pct_request;
//					memset(&db_steinhoff_steering_out, 0, sizeof(db_steinhoff_out_t));
//					db_steinhoff_steering_out.id = 0x2F03;
//					db_steinhoff_steering_out.size = 8;
////					db_steinhoff_steering_out.ts = ts;
//				    }
//				}
//				if(neweagle_steering_report.SteeringEnabled != 0) {
//					db_steinhoff_steering_out.id = 0x2F03;
//					db_steinhoff_steering_out.size = 8;
//						neweagle_steering_request.SteerCtrlEnblReq = 1;
//						set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, 0);
//						neweagle_steering_request.SteeringChecksum = Compute_CRC8(7, &db_steinhoff_steering_out.data[0]);
//						set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, veryverbose);
//						db_clt_write(pclt, DB_STEINHOFF_STEERING_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_steering_out);
//						usleep(1000);
//				 }
//				 else {
//					if(SteerCtrlEnblReqCounter-- > 0) {
//						neweagle_steering_request.SteerCtrlEnblReq = 1;
//						printf("neweagle_steering:Got to 9: SteerCtrlEnblReqCounter %d\n", SteerCtrlEnblReqCounter);
//
//					}
//					else {
//						neweagle_steering_request.SteerCtrlEnblReq = 0;
//						if(SteerCtrlEnblReqCounter <= -5) {
//							SteerCtrlEnblReqCounter = 5;
//							printf("neweagle_steering:Got to 10: SteerCtrlEnblReqCounter %d\n", SteerCtrlEnblReqCounter);
//
//						}
//					}
//					printf("neweagle_steering:Got to 11: SteerCtrlEnblReqCounter %d\n", SteerCtrlEnblReqCounter);
//					db_steinhoff_steering_out.id = 0x2F03;
//					db_steinhoff_steering_out.size = 8;
//
//					set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, 0);
//					neweagle_steering_request.SteeringChecksum = Compute_CRC8(7, &db_steinhoff_steering_out.data[0]);
//					set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, veryverbose);
//					db_clt_write(pclt, DB_STEINHOFF_STEERING_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_steering_out);
//					usleep(1000);
//				}
//			}
//			else {
////				if(steering_angle_or_torque_request_sav != 0){
//				neweagle_steering_request.SteerRollingCtr++;
//				if(neweagle_steering_request.SteerRollingCtr > 15)
//					neweagle_steering_request.SteerRollingCtr = 0;
//				neweagle_steering_request.SteeringWhlAngleVelocityLim = 0;
//				neweagle_steering_request.SteeringReqType = 1;
//				neweagle_steering_request.SteeringWhlIgnoreDriverOvrd = 0;
//				memset(&db_steinhoff_steering_out, 0, sizeof(db_steinhoff_out_t));
//				db_steinhoff_steering_out.id = 0x2F03;
//				db_steinhoff_steering_out.size = 8;
//				neweagle_steering_request.SteerCtrlEnblReq = 0;
////					steering_angle_or_torque_request_sav = 0;
//				set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, 0);
//				neweagle_steering_request.SteeringChecksum = Compute_CRC8(7, &db_steinhoff_steering_out.data[0]);
//				set_neweagle_steering_request(db_steinhoff_steering_out.data, &neweagle_steering_request, veryverbose);
//				db_clt_write(pclt, DB_STEINHOFF_STEERING_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_steering_out);
//			}
//			if(verbose){
//				print_timestamp(stdout, &ts);
//				printf("%s: steering request %hhx %#hhx %.2f steering_angle_or_torque_request %d\n",
//					argv[0],
//					db_steinhoff_steering_out.data[0],
//					db_steinhoff_steering_out.data[1],
//					output.steering_angle_request,
//					output.steering_angle_or_torque_request
//				);
//				usleep(1000);
//
//			}
//		}
//#define DELAY_DEBUG 1
//#ifdef	DELAY_DEBUG
//		if(ts_ms - ts_sav2 >= 20){
//        db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
//		db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
//		print_timestamp(stdout, &ts);
////        printf("DELAY DEBUG: steering_torque_pct_request %f input.percent_torque %f %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
//          printf("DELAY DEBUG:Req %f Report %f\n",
//				output.steering_torque_pct_request,
//				input.percent_torque
////				neweagle_steering_request.SteerCtrlEnblReq,
////    			dbw_misc.MiscByWireEnbl,
////    			dbw_misc.MiscFault,
////    			dbw_misc.MiscAkitCommFault,
////    			dbw_misc.MiscByWireReady,
////    			dbw_misc.MiscDriverActivity,
////    			dbw_misc.DBW_VehReadyToDrive,
////				input.steering_enabled,
////				neweagle_steering_report.SteeringEnabled,
////				input.driver_override,
////				neweagle_steering_report.SteeringDriverActivity,
////				input.steering_fault,
////				neweagle_steering_report.SteeringFault,
////				neweagle_steering_report.MiscAKitCommFault,
////				neweagle_steering_report.SteeringRollingCntr,
////				output.steering_angle_or_torque_request,
////				neweagle_steering_request.SteeringReqType,
////                                neweagle_steering_report.SteeringCtrlType,
////                       SteerCtrlEnblReqCounter
//				);
//          ts_sav2 = ts_sav;
//	}
//#endif

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
