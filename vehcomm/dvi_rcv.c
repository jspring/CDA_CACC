/**\fil
 *	veh_rcv.c 
 *		Receives a message from another vehicle and
 *		writes it to the appropriate database variable,
 *		depending on vehicle ID in the message. 
 *
 *		On initialization reads a config file that specifies
 *		the current vehicle ID, the lead vehicle ID,
 *		and the preceding vehicle ID.
 *
 * Copyright (c) 2008   Regents of the University of California
 *
 */
#include <sys_os.h>
#include <db_clt.h>
#include <db_utils.h>
#include <timestamp.h>
#include <local.h>
#include <sys_rt.h>
#include <sys_ini.h>
#include <udp_utils.h>
#include "dvi.h"
#include "long_comm.h"

static int sig_list[]=
{
	SIGINT,
	SIGQUIT,
	SIGTERM,
	SIGALRM,	// for timer
	(-1)
};

static jmp_buf exit_env;

static void sig_hand(int code)
{
        if (code == SIGALRM)
                return;
        else
                longjmp(exit_env, code);
}

static db_id_t db_vars_list[] = {
        {DB_DVI_RCV_VAR, sizeof(char)},
        {DB_COMM_TX_VAR, sizeof(veh_comm_packet_t)},
};

int main( int argc, char *argv[] )
{
	int ch;		
	db_clt_typ *pclt;  		/// data bucket pointer
	char *domain=DEFAULT_SERVICE;
	char hostname[MAXHOSTNAMELEN+1];
	int xport = COMM_OS_XPORT;
	struct sockaddr_in src_addr;
	char *remote_ipaddr = "172.16.0.177";       /// address of UDP destination
	char *local_ipaddr = "172.16.0.77";       /// address of UDP destination
	struct sockaddr_in dst_addr;
	int create_db_vars = 0;
	int use_db = 1;
	dvi_out_t dvi_out;
	float ts = 0;
	float ts_sav = 0;
	char gap_level = 3;
	veh_comm_packet_t comm_pkt;

	int sd;				/// socket descriptor
	int udp_port = 8003;

	char buf[10] = {0};
	int i;

	int bytes_received;     // received from a call to recv
	int verbose = 0;
	short msg_count = 0;
	int socklen = sizeof(src_addr);

	while ((ch = getopt(argc, argv, "A:a:cdu:v")) != EOF) {
			switch (ch) {
			case 'A': local_ipaddr = strdup(optarg);
					  break;
			case 'a': remote_ipaddr= strdup(optarg);
					  break;
	case 'c': create_db_vars = 1;
		  break;
	case 'd': use_db = 0;
		  break;
	case 'u': udp_port = atoi(optarg);
		  break;
	case 'v': verbose = 1;
		  break;
			default:  printf( "Usage: %s -v (verbose) -A <local ip, def. 172.16.0.77> -a <remote ip, def. 172.16.0.177> -u <UDP port, def. 8003> -d (Do NOT use database)", argv[0]);
		  exit(EXIT_FAILURE);
					  break;
			}
	}
	get_local_name(hostname, MAXHOSTNAMELEN);

	if(use_db != 0) {
		if(create_db_vars)
			pclt = db_list_init(argv[0], hostname, domain, xport, db_vars_list, 1, NULL,  0); 
		else
			pclt = db_list_init(argv[0], hostname, domain, xport, NULL, 0, NULL, 0); 
	}

	if( setjmp( exit_env ) != 0 ) {
		printf("Received %d messages\n", msg_count);
		if(create_db_vars)
			db_list_done(pclt, db_vars_list, 1, NULL, 0);		
		else
			db_list_done(pclt, NULL, 0, NULL, 0);		
		exit( EXIT_SUCCESS );
	} else
		sig_ign( sig_list, sig_hand );

	if ( (sd = udp_peer2peer_init(&dst_addr, remote_ipaddr, local_ipaddr, udp_port, 0)) < 0) {
//	if ( (sd = udp_allow_all(udp_port)) < 0) {
		printf("Failure to initialize socket from %s to %s on port %d\n",
			remote_ipaddr, local_ipaddr, udp_port);
		longjmp(exit_env, 2);
	}

	dvi_out.acc_cacc_request = 0;
	dvi_out.gap_request = gap_level;
	if(use_db != 0)
		db_clt_write(pclt, DB_DVI_OUT_VAR, sizeof(dvi_out_t), &dvi_out);

	while (1) {
		if ((bytes_received = recvfrom(sd, &buf, 10, 0, (struct sockaddr *) &src_addr, (socklen_t *) &socklen)) <= 0) {
				perror("recvfrom failed\n");
				break;
		}
		ts = sec_past_midnight_float();
		printf("%d bytes at %f: ", bytes_received, ts);
		for(i=0; i<bytes_received; i++)
			printf(" buf[%d] %#hhx", i, buf[i]);
		printf("\n");
		fflush(stdout);
#define	ACC_REQUESTED		5
#define	CACC_REQUESTED		6
#define	GAP_DECREASE_REQUESTED	7
#define	GAP_INCREASE_REQUESTED	8
		db_clt_read(pclt, DB_DVI_OUT_VAR, sizeof(dvi_out_t), &dvi_out);
		if(use_db != 0) {
			switch(buf[0]){
				case ACC_REQUESTED:
					dvi_out.acc_cacc_request = 1;
					break;
				case CACC_REQUESTED:
					dvi_out.acc_cacc_request = 2;
					break;
				case GAP_DECREASE_REQUESTED:
					if( (ts - ts_sav) > 0.2)
						if( (gap_level--) < 1)
							gap_level = 1;
					ts_sav = ts;
					dvi_out.gap_request = gap_level;
					break;
				case GAP_INCREASE_REQUESTED:
					if( (ts - ts_sav) > 0.2)
						if( (gap_level++) > 5)
							gap_level = 5;
					ts_sav = ts;
					dvi_out.gap_request = gap_level;
					break;
				default :
					if(buf[0] >= 20)
						dvi_out.driver_id = buf[0] - 20;
					else
						if((buf[0] >= 10) && (buf[0] < 20))
							dvi_out.driver_pos = buf[0] - 10;
			}
			db_clt_read(pclt, DB_COMM_TX_VAR, sizeof(comm_pkt), &comm_pkt); // db var number 499 veh_comm_packet_t

		 if (gap_level == 1 && (dvi_out.acc_cacc_request == 2))
		 {
			dvi_out.CACC_tGap=0.6;
			if (comm_pkt.maneuver_des_2 == 1)
				dvi_out.CACC_tGap=1.6;
		 }
		 else if (gap_level == 2 && (dvi_out.acc_cacc_request == 2))
		 {
			dvi_out.CACC_tGap=0.9;
			if (comm_pkt.maneuver_des_2 == 1)
				dvi_out.CACC_tGap=1.6;
		 }
		 else if (gap_level == 3 && (dvi_out.acc_cacc_request == 2))
		 {
			dvi_out.CACC_tGap=1.2;
			if (comm_pkt.maneuver_des_2 == 1)
				dvi_out.CACC_tGap=1.6;
		 }
		 else if (gap_level == 4 && (dvi_out.acc_cacc_request == 2))
		 {
			dvi_out.CACC_tGap=1.5;
			if (comm_pkt.maneuver_des_2 == 1)
				dvi_out.CACC_tGap=1.6;
		 }
		 else if (gap_level == 5 && (dvi_out.acc_cacc_request == 2))
			dvi_out.CACC_tGap=1.8;
		 else
		 {
			dvi_out.CACC_tGap=1.2;
			if (comm_pkt.maneuver_des_2 == 1)
				dvi_out.CACC_tGap=1.6;
		 }

		 if (gap_level == 1 && (dvi_out.acc_cacc_request == 1))
			dvi_out.ACC_tGap=1.1;
		 else if (gap_level == 2 && (dvi_out.acc_cacc_request == 1))
				dvi_out.ACC_tGap=1.3;
		 else if (gap_level == 3 && (dvi_out.acc_cacc_request == 1))
				dvi_out.ACC_tGap=1.5;
		 else if (gap_level == 4 && (dvi_out.acc_cacc_request == 1))
			dvi_out.ACC_tGap=1.7;
		 else if (gap_level == 5 && (dvi_out.acc_cacc_request == 1))
			dvi_out.ACC_tGap=1.9;
		 else
			dvi_out.ACC_tGap=1.6;

		db_clt_write(pclt, DB_DVI_OUT_VAR, sizeof(dvi_out_t), &dvi_out);
		}
//		if(buf[0] != 0)
//		{
//			printf("Byte received: %hhu at %f sec\n", buf[0], ts);
//		}
	}
	longjmp(exit_env,1);	/* go to exit code when loop terminates */
}
