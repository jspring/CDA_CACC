/* neweagle_steering.c - db_slv client that passes sensor data to the database
**	and forwards control signals to the CAN driver 
**
**
**
*/

#include "db_include.h"
#include "neweagle_steering.h"
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
	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms = 0;
	int i;
	int read_CAN_dbnum = DB_STEINHOFF_MSG_VAR;

	// Set output steering request to large negative number
	memset(&output, 0, sizeof(output_t));

        while ((option = getopt(argc, argv, "vwn:")) != EOF) {
                switch(option) {
                case 'v':
                        verbose = 1;
                        break;
                case 'w':
                        veryverbose = 1;
                        verbose = 1;
                        break;
                case 'n':
                        read_CAN_dbnum = atoi(optarg);
                        break;
                default:
                        printf("Usage: %s %s\n", argv[0], usage);
                        exit(EXIT_FAILURE);
                        break;
                }
        }

    	get_local_name(hostname, MAXHOSTNAMELEN);
    	if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
    			NULL, 0, NULL, 0))
    		== NULL) {
    			printf("Could not db_list_init\n");
    			exit(EXIT_FAILURE);
    	}

    	if (clt_trig_set( pclt, read_CAN_dbnum, read_CAN_dbnum) == FALSE )
    		exit(EXIT_FAILURE);
    	printf("%s: clt_trig_set OK for read_CAN_dbnum %d\n", argv[0], read_CAN_dbnum);

    	if (setjmp(exit_env) != 0) {
    		//Sleep for 100 ms to give CAN driver enough time to send steering disable
    		usleep(100000);

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

	for(;;) {
	
		clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
		if( DB_TRIG_VAR(&trig_info) == read_CAN_dbnum) {
			count++;
			memset(&db_steinhoff_msg.data[0], 0, 8);
			db_clt_read(pclt, read_CAN_dbnum, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
			get_current_timestamp(&ts);
			ts_ms = TS_TO_MS(&ts);
			switch(db_steinhoff_msg.id) {
				case 0X18FEF111:
					check_msg_timeout(ts_ms, &neweagle_steering_report.ts_ms,
							&neweagle_steering_report.two_message_periods,
							&neweagle_steering_report.message_timeout_counter);
					db_clt_read(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					input.vehicle_speed_mps = (float)(((db_steinhoff_msg.data[2] << 8 ) & 0XFF00) + db_steinhoff_msg.data[1]) / (256.0 * 3.6);
					db_clt_write(pclt,DB_INPUT_VAR, sizeof(input_t), &input);
					if(verbose){
						printf("%s CCVS: speed %.2f kph ", argv[0], input.vehicle_speed_mps);
						for(i=0; i<8; i++)
							printf("%#hhX ", db_steinhoff_msg.data[i]);
						printf("\n");
					}
					break;
				default:
//					printf("Unknown message %#hx received\n");
					break;
			}
		}
		get_current_timestamp(&ts);
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
