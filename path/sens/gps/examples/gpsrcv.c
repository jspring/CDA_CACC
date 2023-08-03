/**\file *
 * gpsrcv.c     Read GPS point message arriving over UDP on a specified port
 *		and write to a specified database variable number on the
 *		data server. Can also write trace to stdout.	
 *
 *		Records local timestamp when received as well as 
 *		UTC time and local timestamp when sent from the message.	
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
#include <timestamp.h>
#include <timing.h>
#include <udp_utils.h>
#include <db_clt.h>
#include <db_utils.h>
#include "path_gps_lib.h"

jmp_buf env;
static void sig_hand(int sig);

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
		longjmp(env, code);
}

int main(int argc, char *argv[])
{
	char hostname[MAXHOSTNAMELEN];
	char *domain = DEFAULT_SERVICE;	/// on Linux sets DB q-file directory
        db_clt_typ *pclt;       /// data server client pointer
        int xport = COMM_OS_XPORT;	/// OS-agnostic designation 
	int db_num = 0;		/// must be set to non-zero for DB update.

	int verbose = 0;	/// if 1, print extra info for debugging	
	int udp_port = 7015;	/// port for receiving heartbeat
	int option;

	path_gps_point_t hb;	/// fill in from GPS messages received
	char *phb = (char *)&hb;	/// fill in from GPS messages received
	gps_data_typ gps_data;
	int gps_utc_seconds;
	int gps_utc_seconds_sav;
	int sequence_no = 0;
	int use_db = 0;

	int sd_in;		/// socket descriptor for UDP receive
	int bytes_rcvd;		/// returned from recvfrom
	struct sockaddr_in src_addr;	/// used in recvfrom call
	unsigned int socklen;

	/* Read and interpret any user switches. */
	while ((option = getopt(argc, argv, "n:vu:")) != EOF) {
		switch(option) {
		case 'n':
			db_num = atoi(optarg);
			use_db = 1;
			break;
		case 'v':
			verbose = 1;	
			break;
		case 'u':
			udp_port = atoi(optarg);
			break;
		default:
			fprintf(stderr, "Usage %s: ", argv[0]); 
			fprintf(stderr, " -u  (UDP port number for input) ");
			fprintf(stderr, " -n  DB variable number ");
			fprintf(stderr, " -v  (verbose, info to stdout) ");
			fprintf(stderr, "\n");
			exit(EXIT_FAILURE);
		}
	}
	fprintf(stderr, "Receiving GPS point for DB variable %d\n", db_num);
	fprintf(stderr, "writing output to stdout\n");
	fflush(stderr);

	sd_in = udp_allow_all(udp_port);
	if (sd_in < 0) {
		printf("failure opening socket on %d\n", udp_port);
		exit(EXIT_FAILURE);
	}

	if (db_num == 0) {
		fprintf(stderr, "Must specify DB variable number to use\n");
		fprintf(stderr, "Create DB variable in another process\n");
		fflush(stderr);
		exit(EXIT_FAILURE);
	}

	get_local_name(hostname, MAXHOSTNAMELEN);
        if ((pclt = clt_login(argv[0], hostname, domain, xport)) == NULL ) {
                        printf("%s: Database initialization error\n", argv[0]);
                        exit(EXIT_FAILURE);
                
        }
	
	if (setjmp(env) != 0) {
		if(pclt)
			db_list_done(pclt, NULL, 0, NULL, 0);
		exit(EXIT_SUCCESS);
	} else
               sig_ign(sig_list, sig_hand);

	socklen = sizeof(src_addr);
	memset(&src_addr, 0, socklen);
	memset(&hb, 0, sizeof(hb));
	while (1) {
		timestamp_t ts;
		bytes_rcvd = recvfrom(sd_in, &hb, sizeof(hb), 0,
				(struct sockaddr *) &src_addr, &socklen);
		if (bytes_rcvd < 0) {
			perror("recvfrom failed\n");
			continue;
		}
		get_current_timestamp(&ts);
		path_gps_parse_sentence(phb, &gps_data);
            path_gps_get_point(&gps_data, 1, &hb);
            hb.sequence_no = sequence_no++;
                gps_utc_seconds = TS_TO_SEC(&hb.utc_time);
                if(gps_utc_seconds_sav  != gps_utc_seconds) {
                        gps_utc_seconds_sav = gps_utc_seconds;
                        get_current_timestamp(&hb.local_time);
                }
                hb.utc_seconds_since_midnight = TS_TO_SEC(&hb.utc_time);

            if (use_db)
		db_clt_write(pclt, db_num, sizeof(path_gps_point_t), &hb);
		if (verbose) {
			print_timestamp(stdout, &ts);
			path_gps_print_data(stdout, &gps_data);
			fprintf(stdout, "\n");
			fflush(stdout);
		}
	}
}
