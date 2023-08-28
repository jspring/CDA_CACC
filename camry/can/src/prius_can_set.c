#include "db_include.h"
#include "prius_can.h"
#include "clt_vars_prius.h"
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

	db_steinhoff_msg_t db_steinhoff_msg;
	db_steinhoff_out_t db_steinhoff_brake_out;
	db_steinhoff_out_t db_steinhoff_accel_out;
	db_steinhoff_out_t db_steinhoff_obd2;
	veh_comm_packet_t virtual_car_comm_packet;
	prius_fuel_rate_t prius_fuel_rate;
	input_t input;
	output_t output;
	timestamp_t ts;
	int ts_ms = 0;
	int ts_now = 0;
	int ts_sav = 0;
	char obd2_ctr = 0;

	prius_accel_cmd_status_t prius_accel_cmd_status;
	prius_accel_cmd_t prius_accel_cmd;
	prius_accel_cmd_t prius_brake_cmd;
	prius_cruise_control_state_t prius_cruise_control_state;
	prius_vehicle_speed_t prius_vehicle_speed;
	prius_wheel_speed_t prius_wheel_speed;
	prius_long_lat_accel_t prius_long_lat_accel;
	prius_radar_forward_vehicle_t prius_radar_forward_vehicle;

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


    	if (setjmp(exit_env) != 0) {
    		memset(&db_steinhoff_brake_out, 0, sizeof(db_steinhoff_out_t));
    		db_steinhoff_brake_out.port = BRAKE_PORT;
    		db_steinhoff_brake_out.id = 0x99;
    		db_steinhoff_brake_out.size = 2;
 //   		set_prius_brake_cmd(db_steinhoff_brake_out.data, &prius_brake_cmd);
    		db_clt_write(pclt, DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t), &db_steinhoff_brake_out);
        	db_list_done(pclt, NULL, 0, NULL, 0);
            printf("%s: received %d CAN messages %d IPC message errors\n",
                		argv[0], count, ipc_message_error_ctr);
            exit(EXIT_SUCCESS);
        } else
               sig_ign(sig_list, sig_hand);

    db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
	if(acceleration > 0) {
		output.brake_level = 0;
		output.throttle_pct = acceleration;
		printf("Setting output.throttle_pct to %.2f\n", output.throttle_pct);
	}
	else {
		output.brake_level = acceleration;
		output.throttle_pct = 0;
		printf("Setting output.brake_level to %.2f\n", output.brake_level);
	}
    db_clt_write(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
	return 0;
}
