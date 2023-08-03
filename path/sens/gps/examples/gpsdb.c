/**\file 
 *
 * gpsdb.c     Read GPS stream from stdin and write to a local heartbeat
 *		structure. Can use gpsd with tcp_client -q r typed
 *		to stdin. Or can cat serial port directly into stdin. 
 *
 *		Writes GPS data to data bucket (DB).
 *		In verbose mode prints on stdout.
 *	
 *		DB shared variables are assumed to be created elsewhere. 
 *		DB variable to use to write data can be specified 
 *		on command line. 	
 *	
 *  Copyright (c) 2008   Regents of the University of California
 *
 *  The signals SIGINT, SIGQUIT, and SIGTERM are trapped, and cause the
 *  process to terminate. SIGALRM is used for Posix timers on Linux.
 *
 */


#include <sys_os.h>
#include <sys_rt.h>
#include <local.h>
#include <timing.h>
#include <timestamp.h>
#include <udp_utils.h>
#include <db_clt.h>
#include <db_utils.h>
#include "path_gps_lib.h"
#undef DO_TRACE

static int sig_list[] = {
    SIGINT,
    SIGQUIT,
    SIGTERM,
    SIGALRM,	/// for timer
    ERROR,
};

jmp_buf env;
static void sig_hand(int code )
{
    if (code == SIGALRM)
	return;
    else
	longjmp(env, code );
}

#define MAX_BUFFERED_GPS_DATA 20

int main(int argc, char *argv[])
{
    int status;
    char hostname[MAXHOSTNAMELEN];
    char *domain = DEFAULT_SERVICE;	/// on Linux sets DB q-file directory
    db_clt_typ *pclt;       /// data server client pointer
    int xport = COMM_OS_XPORT;	/// value set correctly in sys_os.h 
    int use_db = 1;		/// set to 0 to use data server

    char sentence[MAX_GPS_SENTENCE];
    gps_data_typ gps_data[MAX_BUFFERED_GPS_DATA];
	
    int verbose = 0;	/// if 1, print extra info for debugging	
    int veryverbose = 0;	/// if 1, print way too much info

    FILE* file = stdin;
    int use_stdin = 1;

    int option;
    path_gps_point_t hb;	/// fill in from GPS messages received
    int num_db = DB_GPS_PT_LCL_VAR;	/// or set from cmd line
    int num_buffered = 0;	/// number of raw messages passed to path_gps_get_point
    short sequence_no = 0;  /// sequence number useful when sending GPS heartbeat 	
    char *trigger_message = "GPRMC"; /// NMEA message type that triggers write to DB
    volatile int num_read = 0;
    double gps_utc_seconds = 0;
    double gps_utc_seconds_sav = 0; 
    /* Read and interpret any user switches. */
    while ((option = getopt(argc, argv, "d:n:p:vwt")) != EOF) {
	switch(option) {
	case 'd':
	    use_db = atoi(optarg);
	    break;
	case 'n':
	    num_db = atoi(optarg);
	    break;
	case 'p':
	    file = fopen(optarg, "r");
	    if (!file)
	    {
		fprintf(stderr, "gpsdb: failed to open file %s\n", optarg);
		exit(EXIT_FAILURE);
	    }
	    use_stdin = 0;
	    break;
	case 'v':
	    verbose = 1;
	    break;
	case 'w':
	    veryverbose = 1;	/// traces raw GPS messages
	    break;
	default:
	    fprintf(stderr, "Usage %s: \n"
		    " -d  <1 use DB, 0, don't use DB> \n"
		    " -v  (verbose) \n"
		    " -w  (very verbose) \n"
		    " -n  (use this DB number) \n\n", argv[0]);
	    exit(EXIT_FAILURE);
	    break;
	}
    }
    printf("PATH GPS store in db app, read GPS from stdin\n");
    fflush(stdout);

    if (use_db){
	get_local_name(hostname, MAXHOSTNAMELEN);
	if ((pclt = db_list_init(argv[0], hostname, domain, xport,
				 NULL, 0, NULL, 0)) == NULL ) {
	    printf("Database initialization error in %s\n", argv[0]);
	    exit(EXIT_FAILURE);
	}
    }

    if (setjmp(env) != 0) {
	if (use_db)
	    db_list_done(pclt, NULL, 0, NULL, 0);
	printf("gpsdb: %d total messages\n", num_read);
	if (!use_stdin)
	    fclose(file);
	exit(EXIT_SUCCESS);
    } else
	sig_ign(sig_list, sig_hand);

    memset(&hb, 0, sizeof(hb));

    while (1) {
	/// Blocks until it gets a good GPS sentence
	path_gps_get_sentence(sentence, file, 0, 0); 
	path_gps_parse_sentence(sentence, &gps_data[num_buffered]);

	if (veryverbose) {
	    path_gps_print_data(stdout, &gps_data[num_buffered]);
	    printf("\n");
	}
	num_buffered++;
	if  (strcmp(gps_data[num_buffered - 1].gps_id,
		    trigger_message) == 0 ||
	     num_buffered == MAX_BUFFERED_GPS_DATA) {
	    path_gps_get_point(gps_data, num_buffered, &hb);
	    hb.sequence_no = sequence_no++;
	    if (use_db) {
		gps_utc_seconds = TS_TO_SEC(&hb.utc_time);
               	if(gps_utc_seconds_sav  != gps_utc_seconds) {
       			gps_utc_seconds_sav = gps_utc_seconds;
			get_current_timestamp(&hb.local_time);
		}
		hb.utc_seconds_since_midnight = TS_TO_SEC(&hb.utc_time);
		db_clt_write(pclt,num_db, sizeof(path_gps_point_t), &hb);
	    }
	    num_buffered = 0;
	    if (verbose) {
		print_timestamp(stdout, &hb.local_time);
		printf(" ");
		path_gps_print_point(stdout, &hb);
		printf("\n");
	    }

	    num_read++;
	}
    }
}
