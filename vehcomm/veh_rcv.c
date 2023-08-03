/**\file
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
#include "path_gps_lib.h"
#include "long_comm.h"
#include <local.h>
#include <sys_rt.h>
#include <sys_ini.h>
#include <udp_utils.h>

#include "asn_application.h"
#include "asn_internal.h" /* for _ASN_DEFAULT_STACK_MAX */
#include "MessageFrame.h"
#include "veh_lib.h"

static int sig_list[] =
	{
		SIGINT,
		SIGQUIT,
		SIGTERM,
		SIGALRM, // for timer
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

static void print_usage()
{
	printf("Usage: veh_rcv\n");
	printf("-A <local ipaddr, def. 127.0.0.1>\n");
	printf("-a <remote ipaddr, def. 10.0.1.9>\n");
	printf("-v (verbose)\n");
	printf("-u <UDP port, def. 5052>\n");
	printf("-t <vehicle string, def. Blue>\n");
	printf("-f <config file name, def. realtime.ini>\n");
}

/**  By default (if no ini file entry) sets up as head car
 */
static int get_ids(FILE *fpin, int *pself, int *plead, int *psecond, int *pthird)
{
	*plead = get_ini_long(fpin, "LeadVehicleID", 1);
	*pself = get_ini_long(fpin, "PositionInPlatoon", 1);
	*psecond = get_ini_long(fpin, "SecondVehicleID", 0);
	*pthird = get_ini_long(fpin, "ThirdVehicleID", 0);
	printf("lead %d self %d second %d third %d\n", *plead, *pself, *psecond, *pthird);
	fflush(stdout);
	return 1;
}

int main(int argc, char *argv[])
{
	int ch;
	db_clt_typ *pclt; /// data bucket pointer
	char *domain = DEFAULT_SERVICE;
	char hostname[MAXHOSTNAMELEN + 1];
	int xport = COMM_OS_XPORT;
	int self_vehicle_id;
	int lead_vehicle_id;
	int second_vehicle_id;
	int third_vehicle_id;
	//int ts1_sav = 0;
	//int ts2_sav = 0;
	//int ts3_sav = 0;


	veh_rcv_t vehs_obj[] = {
		{DB_COMM_VIRTUAL_TRK_VAR,0,0,0,0,0},
		{DB_COMM_LEAD_TRK_VAR,0,0,0,0,0},
		{DB_COMM_SECOND_TRK_VAR,0,0,0,0,0},
		{DB_COMM_THIRD_TRK_VAR,0,0,0,0,0}
	};

	int pltn_size = sizeof(vehs_obj)/sizeof(veh_rcv_t);
	struct sockaddr_in src_addr;
	char *remote_ipaddr = "10.0.1.9";
	char *local_ipaddr = "127.0.0.1"; 
	struct sockaddr_in dst_addr;

	int sd = -1; /// socket descriptor
	int udp_port = 5052;
	
	veh_comm_packet_t comm_pkt;
	veh_comm_packet_t repeat_comm_pkt;
	MessageFrame_t *BSMCACC_decode;
	BSMCACC_decode = (MessageFrame_t *)calloc(1, sizeof(MessageFrame_t));

	int BSMCACCSIZE = sizeof(MessageFrame_t);
	char BSMCACC_buf[BSMCACCSIZE];
	memset(BSMCACC_buf, 0, sizeof(MessageFrame_t));

	asn_dec_rval_t rval;

	int ret = -1;
	int i;
	int very_verbose = 0;
	int debug = 0;
	int bytes_received;			// received from a call to recv
	FILE *fpin;					/// file pointer for ini file
	char *vehicle_str = "Blue"; /// e.g., Blue, Gold, Silver
	int verbose = 0;
	short msg_count = 0;
	char *ini_fname = "realtime.ini";
	int socklen = sizeof(src_addr);
	int byte_offset = 3;
	int repeat_comm_packet = 0;

	short rcvd_sn = 0; // keeps track of the last variable received

	while ((ch = getopt(argc, argv, "A:a:t:u:f:vwdo:R")) != EOF)
	{
		switch (ch)
		{
		case 'A':
			local_ipaddr = strdup(optarg);
			break;
		case 'a':
			remote_ipaddr = strdup(optarg);
			break;
		case 't':
			vehicle_str = strdup(optarg);
			break;
		case 'f':
			ini_fname = strdup(optarg);
			break;
		case 'u':
			udp_port = atoi(optarg);
			break;
		case 'w':
			very_verbose = 1;
			verbose = 1;
			break;
		case 'o':
			byte_offset = atoi(optarg);
			break;
		case 'v':
			verbose = 1;
			break;
		case 'd':
			debug = 1;
			break;
		case 'R':
			repeat_comm_packet = 1;
			break;
		default:
			print_usage();
			exit(EXIT_FAILURE);
			break;
		}
	}
/*
	if ((fpin = get_ini_section(ini_fname, vehicle_str)) == NULL)
	{
		printf("%s: can't get ini file %s, section %s\n", argv[0], ini_fname, vehicle_str);
		fflush(stdout);
		exit(EXIT_FAILURE);
	}
*/
	//Read in .ini file and set vehicle IDs
	printf("%s: vehicle_str %s:", argv[0], vehicle_str);
//	get_ids(fpin, &self_vehicle_id, &lead_vehicle_id,
//			&second_vehicle_id, &third_vehicle_id);

	get_local_name(hostname, MAXHOSTNAMELEN);

	//assumes DB_COMM variables were aleady created by another process
	pclt = db_list_init(argv[0], hostname, domain, xport, NULL, 0, NULL, 0);
	if (setjmp(exit_env) != 0)
	{
		printf("%s: Received %d messages\n", argv[0], msg_count);
		db_list_done(pclt, NULL, 0, NULL, 0);
		exit(EXIT_SUCCESS);
	}
	else
	{
		sig_ign(sig_list, sig_hand);
	}

	if ((sd = udp_peer2peer_init(&dst_addr, remote_ipaddr, local_ipaddr, udp_port, 0)) < 0)
	{
		printf("%s: Failure to initialize socket from %s to %s on port %d\n",
			   argv[0], remote_ipaddr, local_ipaddr, udp_port);
		longjmp(exit_env, 2);
	}
	else
	{		
		printf("%s: Succeeded in initializing socket descriptor %d from %s to %s on port %d\n",
			   argv[0], sd, remote_ipaddr, local_ipaddr, udp_port);
	}

	while (1)
	{
		if ((bytes_received = recvfrom(sd, BSMCACC_buf, BSMCACCSIZE, 0, (struct sockaddr *)&src_addr, (socklen_t *)&socklen)) <= 0)
		{
			perror("recvfrom failed\n");
		}
		// decode bsm message
		rval = uper_decode(0, &asn_DEF_MessageFrame, (void **)&BSMCACC_decode, &BSMCACC_buf[byte_offset], (bytes_received - byte_offset), 0, 0);
		if (rval.code != RC_OK)
		{
			printf("%s:Cannot decode received message. Bytes received %d Bytes consumed %d byte_offset %d\n", argv[0],
					bytes_received, rval.consumed, byte_offset);
		}
		else // RC_OK is true
		{
			if (debug) 
			{
				printf("veh_rcv: Message decoded successfully\n");
				printf("veh_rcv: ");
				for (i = byte_offset; i < bytes_received; i++)
				{
					printf(" %d:%02hhx", i, BSMCACC_buf[i]);
				}
				printf("\n");
			}
			if (very_verbose)
			{
				xer_fprint(stdout, &asn_DEF_MessageFrame, BSMCACC_decode);
			}
		

			ret = BSM2vehcomm(BSMCACC_decode, &comm_pkt);

			//Print out comm_pkt if desired
			if (verbose)
			{
				print_comm_packet(&comm_pkt);
			}

			get_current_timestamp(&comm_pkt.rcv_ts);
			printf("packet received from %s\n", &comm_pkt.object_id[0]);
			//Make sure the comm_pkt is received from other trucks
			//if (comm_pkt.object_id[0] != *vehicle_str)
			if (!strncmp(&comm_pkt.object_id[0], vehicle_str,4))
			{
				msg_count++;

				if (msg_count == 1)
					rcvd_sn = comm_pkt.sequence_no; //initial value

				if ((comm_pkt.sequence_no - rcvd_sn) == 1){
					rcvd_sn++;
				}
				else {
					//printf( "%hd packets lost.", (comm_pkt.sequence_no - rcvd_sn - 1) );
					rcvd_sn = comm_pkt.sequence_no;
				}

				//  printf( "\nrcvd_sn: %hd.\n",rcvd_sn );
			}
			////////////////////////////////////////////////////////////////////////////

			/** Check "my_pip" field in packet and write to 
			 *  DB_COMM_LEAD_TRK, DB_COMM_TRK_SECOND, and DB_COMM_TRK_THIRD if relevant 
			 * For second and third vehicles, both may be written from same packet
			 */
/**
			if ((comm_pkt.my_pip == lead_vehicle_id) && ((TS_TO_MS(&comm_pkt.ts) - ts1_sav) > 0))
			{
				db_clt_write(pclt, DB_COMM_LEAD_TRK_VAR, sizeof(comm_pkt), &comm_pkt);
				ts1_sav = TS_TO_MS(&comm_pkt.ts);
				printf("lead vechle\n");
			}
			if ((comm_pkt.my_pip == second_vehicle_id) && ((TS_TO_MS(&comm_pkt.ts) - ts2_sav) > 0))
			{
				db_clt_write(pclt, DB_COMM_SECOND_TRK_VAR, sizeof(comm_pkt), &comm_pkt);
				ts2_sav = TS_TO_MS(&comm_pkt.ts);
				printf("scond vechle\n");
			}
			if ((comm_pkt.my_pip == third_vehicle_id) && ((TS_TO_MS(&comm_pkt.ts) - ts3_sav) > 0))
			{
				db_clt_write(pclt, DB_COMM_THIRD_TRK_VAR, sizeof(comm_pkt), &comm_pkt);
				ts3_sav = TS_TO_MS(&comm_pkt.ts);
				printf("thired vechle\n");
			}
**/
			printf("veh_rcv: Got to 1\n");
			for (i = 0; i < pltn_size; i++) {
				printf("veh_rcv: Got to 2\n");
				if (comm_pkt.my_pip == i) {
					printf("veh_rcv: Got to 3 i %d\n", i);

//					if ((TS_TO_MS(&comm_pkt.ts) - vehs_obj[i].msg_ts_last) > 0) {
						db_clt_write(pclt, vehs_obj[i].db_var, sizeof(veh_comm_packet_t), &comm_pkt);
						if(repeat_comm_packet != 0 && (!strncmp(&comm_pkt.object_id[0], "taur",4))){
							db_clt_write(pclt, DB_REPEAT_COMM_PACKET_VAR, sizeof(veh_comm_packet_t), &comm_pkt);
							printf("Writing to repeat db var %d seq_no %d my_pip %d\n", DB_REPEAT_COMM_PACKET_VAR, comm_pkt.sequence_no, comm_pkt.my_pip);
						}
						printf("my_pip %d sequence_no %d dbvar %d\n", comm_pkt.my_pip, comm_pkt.sequence_no,vehs_obj[i].db_var );
						if ((comm_pkt.sequence_no - vehs_obj[i].rx_msg_seq) == 1) { // ideal case
							vehs_obj[i].rx_msg_cnt++;
						} 
						else if ((vehs_obj[i].rx_msg_cnt == 0) || ((vehs_obj[i].rx_msg_seq - comm_pkt.sequence_no) == 127)) { // start and rollover
							vehs_obj[i].rx_msg_cnt++;
						}
						else {
							vehs_obj[i].rx_msg_err++;
							printf("################# Sequence error \n");
						}
						vehs_obj[i].msg_ts_last = TS_TO_MS(&comm_pkt.ts);
						vehs_obj[i].rx_msg_seq = comm_pkt.sequence_no;
//					}
					break;
				}
			}

			if (verbose)
			{
				printf("Vehicle %d received packet from vehicle %d speed %f\n",
					self_vehicle_id, comm_pkt.my_pip, comm_pkt.velocity);
				printf("%s: %f\n", comm_pkt.object_id,
					comm_pkt.global_time);
				fflush(stdout);
			}
		}
	}
	/* go to exit code when loop terminates */
	longjmp(exit_env, 1); 
}
