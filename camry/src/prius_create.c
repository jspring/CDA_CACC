/* \file 
 *
 * 	Copyright (c) 2010   Regents of the University of California
 *
 */

#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include <db_sync.h>
#include <m56_can.h>
#include "das3.h"

path_gps_point_t local_gps;              // on-vehicle GPS
sync_record_typ video;

/** Output variables for the long_trk process are created with this list. 
 */
static db_id_t db_vars_list[] = {
        {DB_GPS_PT_LCL_VAR, sizeof(path_gps_point_t)},
        {DB_M56_VCAN2_MSG002_VAR, sizeof(m56_steering_t)},
        {DB_M56_VCAN2_MSG174_VAR, sizeof(m56_gear_t)},
        {DB_M56_VCAN2_MSG180_VAR, sizeof(m56_engine_rpm_t)},
        {DB_M56_VCAN2_MSG1c3_VAR, sizeof(m56_its_alive_t)},
        {DB_M56_VCAN2_MSG239_VAR, sizeof(m56_pedal_position_t)},
        {DB_M56_VCAN2_MSG284_VAR, sizeof(m56_wheel_speed_front_t)},
        {DB_M56_VCAN2_MSG285_VAR, sizeof(m56_wheel_speed_rear_t)},
        {DB_M56_VCAN2_MSG292_VAR, sizeof(m56_acceleration_t)},
        {DB_M56_VCAN2_MSG2aa_VAR, sizeof(m56_acc_status_t)},
        {DB_M56_VCAN2_MSG2b0_VAR, sizeof(m56_eng_tq_acc_and_brake_flags_t)},
        {DB_M56_VCAN2_MSG2b3_VAR, sizeof(m56_dashboard_indicators_t)},
        {DB_M56_VCAN2_MSG354_VAR, sizeof(m56_abs_status_t)},
        {DB_M56_VCAN2_MSG358_VAR, sizeof(m56_turn_switch_status_t)},
        {DB_M56_VCAN2_MSG421_VAR, sizeof(m56_atcvt_t)},
        {DB_M56_VCAN2_MSG5b0_VAR, sizeof(m56_transmission_mode_t)},
        {DB_M56_VCAN2_MSG625_VAR, sizeof(m56_front_wiper_status_t)},
        {DB_M56_ITSCAN_MSG52b_VAR, sizeof(m56_lidar_target_t)},
        {DB_M56_ITSCAN_MSG52c_VAR, sizeof(m56_yaw_rate_t)},
        {DB_M56_ITSCAN_MSG52d_VAR, sizeof(m56_lidar_status_t)},
        {DB_M56_IGNITION_VAR, sizeof(m56_ignition_status_t)},
        {DB_DAS3_IGNITION_VAR, sizeof(das3_ignition_status_t)},
        {DB_M56_VCAN2_MSG210_VAR, sizeof(m56_vdne491_m210_t)},
        {DB_M56_VCAN2_MSG211_VAR, sizeof(m56_vdne491_m211_t)},
        {DB_M56_VCAN2_MSG212_VAR, sizeof(m56_vdne491_m212_t)},
        {DB_M56_VCAN2_MSG213_VAR, sizeof(m56_vdne491_m213_t)},
        {DB_M56_VCAN2_MSG214_VAR, sizeof(m56_vpro4_m214_t)},
        {DB_M56_VCAN2_MSG215_VAR, sizeof(m56_vpro4_m215_t)},
        {DB_M56_VCAN2_MSG216_VAR, sizeof(m56_vpro4_m216_t)},
        {DB_M56_VCAN2_MSG217_VAR, sizeof(m56_vpro4_m217_t)},
        {DB_M56_VCAN2_MSG218_VAR, sizeof(m56_vdnc304_m218_t)},
        {DB_M56_VCAN2_MSG219_VAR, sizeof(m56_vdnc304_m219_t)},
        {DB_M56_VCAN2_MSG21a_VAR, sizeof(m56_vdnc304_m21a_t)},
        {DB_M56_VCAN2_MSG21b_VAR, sizeof(m56_vdnc304_m21b_t)},
        {DB_M56_VCAN2_MSG21c_VAR, sizeof(m56_vpro4_m21c_t)},
        {DB_M56_VCAN2_MSG21d_VAR, sizeof(m56_vpro4_m21d_t)},
        {DB_M56_VCAN2_MSG21e_VAR, sizeof(m56_vpro4_m21e_t)},
        {DB_M56_VCAN2_MSG21f_VAR, sizeof(m56_vpro4_m21f_t)},
        {DB_M56_VCAN2_MSG289_VAR, sizeof(m56_adas_289_t)},
        {DB_M56_VCAN2_MSG28a_VAR, sizeof(m56_adas_28a_t)},

        {DB_M56_ITSCAN_MSG200_VAR, sizeof(m56_m200_t)},
        {DB_M56_ITSCAN_MSG201_VAR, sizeof(m56_m201_t)},
        {DB_M56_ITSCAN_MSG202_VAR, sizeof(m56_m202_t)},
        {DB_M56_ITSCAN_MSG203_VAR, sizeof(m56_m203_t)},
        {DB_M56_ITSCAN_MSG650_VAR, sizeof(m56_m650_t)},

        {DB_M56_ITSCAN_MSG651_VAR, sizeof(m56_m651_t)},
        {DB_M56_ITSCAN_MSG652_VAR, sizeof(m56_m652_t)},
        {DB_M56_ITSCAN_MSG653_VAR, sizeof(m56_m653_t)},
        {DB_M56_ITSCAN_MSG654_VAR, sizeof(m56_m654_t)},
        {DB_M56_ITSCAN_MSG655_VAR, sizeof(m56_m655_t)},

        {DB_M56_ITSCAN_MSG656_VAR, sizeof(m56_m656_t)},
        {DB_M56_ITSCAN_MSG657_VAR, sizeof(m56_m657_t)},
        {DB_M56_ITSCAN_MSG658_VAR, sizeof(m56_m658_t)},
        {DB_M56_ITSCAN_MSG710_VAR, sizeof(m56_m710_t)},
        {DB_M56_ITSCAN_MSG711_VAR, sizeof(m56_m711_t)},

        {DB_M56_ITSCAN_MSG430_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG440_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG450_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG460_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG470_VAR, sizeof(m56_m4n0_t)},

        {DB_M56_ITSCAN_MSG431_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG441_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG451_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG461_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG471_VAR, sizeof(m56_m4n1_t)},

        {DB_M56_ITSCAN_MSG432_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG442_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG452_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG462_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG472_VAR, sizeof(m56_m4n2_t)},

        {DB_M56_ITSCAN_MSG433_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG443_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG453_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG463_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG473_VAR, sizeof(m56_m4n3_t)},

        {DB_M56_ITSCAN_MSG434_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG444_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG454_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG464_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG474_VAR, sizeof(m56_m4n4_t)},

        {DB_M56_ITSCAN_MSG435_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG445_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG455_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG465_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG475_VAR, sizeof(m56_m4n5_t)},

        {DB_M56_ITSCAN_MSG436_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG446_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG456_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG466_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG476_VAR, sizeof(m56_m4n6_t)},

        {DB_M56_ITSCAN_MSG437_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG447_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG457_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG467_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG477_VAR, sizeof(m56_m4n7_t)},

        {DB_M56_ITSCAN_MSG438_VAR, sizeof(m56_m4n8_t)},
        {DB_M56_ITSCAN_MSG448_VAR, sizeof(m56_m4n8_t)},
        {DB_M56_ITSCAN_MSG458_VAR, sizeof(m56_m4n8_t)},
        {DB_M56_ITSCAN_MSG468_VAR, sizeof(m56_m4n8_t)},
        {DB_M56_ITSCAN_MSG478_VAR, sizeof(m56_m4n8_t)},

        {DB_M56_ITSCAN_MSG480_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG490_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG4A0_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG4B0_VAR, sizeof(m56_m4n0_t)},
        {DB_M56_ITSCAN_MSG4C0_VAR, sizeof(m56_m4n0_t)},

        {DB_M56_ITSCAN_MSG481_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG491_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG4A1_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG4B1_VAR, sizeof(m56_m4n1_t)},
        {DB_M56_ITSCAN_MSG4C1_VAR, sizeof(m56_m4n1_t)},

        {DB_M56_ITSCAN_MSG482_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG492_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG4A2_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG4B2_VAR, sizeof(m56_m4n2_t)},
        {DB_M56_ITSCAN_MSG4C2_VAR, sizeof(m56_m4n2_t)},

        {DB_M56_ITSCAN_MSG483_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG493_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG4A3_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG4B3_VAR, sizeof(m56_m4n3_t)},
        {DB_M56_ITSCAN_MSG4C3_VAR, sizeof(m56_m4n3_t)},

        {DB_M56_ITSCAN_MSG484_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG494_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG4A4_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG4B4_VAR, sizeof(m56_m4n4_t)},
        {DB_M56_ITSCAN_MSG4C4_VAR, sizeof(m56_m4n4_t)},

        {DB_M56_ITSCAN_MSG485_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG495_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG4A5_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG4B5_VAR, sizeof(m56_m4n5_t)},
        {DB_M56_ITSCAN_MSG4C5_VAR, sizeof(m56_m4n5_t)},

        {DB_M56_ITSCAN_MSG486_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG496_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG4A6_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG4B6_VAR, sizeof(m56_m4n6_t)},
        {DB_M56_ITSCAN_MSG4C6_VAR, sizeof(m56_m4n6_t)},

        {DB_M56_ITSCAN_MSG487_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG497_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG4A7_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG4B7_VAR, sizeof(m56_m4n7_t)},
        {DB_M56_ITSCAN_MSG4C7_VAR, sizeof(m56_m4n7_t)},

        {DB_SYNC_RECORD_VAR, sizeof(sync_record_typ)},
};

#define NUM_DB_VARS	sizeof(db_vars_list)/sizeof(db_id_t)

static int sig_list[] = 
{
	SIGINT,
	SIGQUIT,
	SIGTERM,
	SIGALRM,
	ERROR,
};

static jmp_buf exit_env;

static void sig_hand(int code)
{
	if (code == SIGALRM)
		return;
	else
		longjmp(exit_env, code);
}


int main(int argc, char *argv[])
{
	db_clt_typ *pclt;              /* Database client pointer */
	char hostname[MAXHOSTNAMELEN+1];
        char *domain=DEFAULT_SERVICE;
        int xport = COMM_OS_XPORT;
	int option;
	posix_timer_typ *ptimer;
	int delay_ms = 1000;	// wake up once a second
	char dummy[MAX_DATA_SIZE];	// used when initializing variables
	int i;
	int id;
	int size;
	int verbose = 0;
	timestamp_t current_ts;
        while ((option = getopt(argc, argv, "t:v")) != EOF) {
                switch(option) {
		case 't':
			delay_ms = atoi(optarg);
			break;
		case 'v':
			verbose = 1;
			break;
		default:
			fprintf(stderr, "Usage %s -t <millisec> -v (verbose)\n",				argv[0]);
			exit(1);
			break;
                }
	}

	get_local_name(hostname, MAXHOSTNAMELEN);
	pclt = db_list_init(argv[0], hostname, domain, xport, db_vars_list, 
		NUM_DB_VARS, NULL, 0);

	/* Catch the signals SIGINT, SIGQUIT, and SIGTERM.  If signal occurs,
	 * log out from database and exit. */
	if (setjmp(exit_env) != 0) {
		if (pclt != NULL)
			db_list_done(pclt, db_vars_list, NUM_DB_VARS, NULL, 0);
		exit(EXIT_SUCCESS);
	} else
		sig_ign(sig_list, sig_hand);

        if ((ptimer = timer_init( delay_ms, ChannelCreate(0) )) == NULL)
        {
                fprintf(stderr, "Unable to initialize delay timer\n");
                exit(EXIT_FAILURE);
        }
	// Create and initialize additional variables  to 0
        for (i = 0; i < NUM_DB_VARS; i++) {
		printf("db_vars_list[%d].id %d\n", i, db_vars_list[i].id);
                id = db_vars_list[i].id;
		size = db_vars_list[i].size;
		clt_create(pclt, id, id, size);
        }
	memset(dummy, 0, MAX_DATA_SIZE);

        for (i = 0; i < NUM_DB_VARS; i++) {
                int id = db_vars_list[i].id;
		int size = db_vars_list[i].size;
		db_clt_write(pclt, id, size, (void *) dummy); 
        }

	// Loop forever doing nothing until terminated with signal
	// In verbose mode print timestamp to track that process is alive 
	while (TRUE) {
		if (verbose) {
                       get_current_timestamp(&current_ts);
                       print_timestamp(stdout,&current_ts);
                       printf("%s \n", argv[0]);
		}
		TIMER_WAIT (ptimer);
	}
}

