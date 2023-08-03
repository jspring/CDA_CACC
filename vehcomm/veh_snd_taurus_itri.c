/**\file
 *	veh_snd.c 
 *		Sends a message to another vehicle, filling in
 *		the fields by reading the appropriate database variable.
 *
 * Copyright (c) 2008   Regents of the University of California
 *
 */
/* Set the vehicle string as the object identifier in the
 * GPS point structure within the packet being sent.
 */


#include <sys_os.h>
#include <db_clt.h>
#include <db_utils.h>
#include <timestamp.h>
#include <local.h>
#include <sys_rt.h>
#include <sys_ini.h>
#include "path_gps_lib.h"
#include "long_comm.h"
#include "veh_lib.h"
#include "../include/vehicle_common.h"
#include "../steinhoff/can_dbvars.h"

#include "asn_application.h"
#include "asn_internal.h" /* for _ASN_DEFAULT_STACK_MAX */
#include "asn_SEQUENCE_OF.h"
#include "MessageFrame.h"

static int sig_list[] =
	{
		SIGINT,
		SIGQUIT,
		SIGTERM,
		SIGALRM,
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
	printf("Usage: veh_snd \n");
	printf("-A <local IP address, def. 127.0.0.1>\n");
	printf("-a <remote IP address, def. 127.0.0.1> \n");
	printf("-u <UDP port, def. 1516> \n");
	printf("-t <vehicle string, def. Blue> \n");
	printf("-i <interval, def. 20 ms>\n");
}

int main(int argc, char *argv[])
{
	db_clt_typ *pclt;
	char *domain = DEFAULT_SERVICE;
	char hostname[MAXHOSTNAMELEN + 1];
	int xport = COMM_OS_XPORT;

	veh_comm_packet_t comm_pkt;
	posix_timer_typ *ptmr;

	short msg_count = 0;

	int debug_counter = 0;
	float fcounter = 10.0;

	//Allocate memory for J2735 BSMCACC message
//	MessageFrame_t *BSMCACC = (MessageFrame_t *)calloc(1, sizeof(MessageFrame_t));

	char msg_str[1500];
	char encod_buffer[1000];
	asn_enc_rval_t enc_rval;

	const char *bsm_header = "Version=0.7\nType=BSM\nPSID=0x20\nPriority=0\nTxMode=CONT\nTxChannel=172\nTxInterval=0\nDeliveryStart=\nDeliveryStop=\nSignature=False\nEncryption=False\nPayload=";
	int bsm_header_length = strlen(bsm_header);

	int retval = 0;
	int i;

	struct sockaddr_in dst_addr;
	int bytes_sent;
	int socket;

	int udp_port = 1516;
	char *remote_ipaddr = "127.0.0.1"; // address of UDP destination
	char *local_ipaddr = "127.0.0.1";  // address of UDP destination
	int interval = 1000;
	int debug = 0;
	int use_db = 1;
	path_gps_point_t self_gps;

//	FILE *fpin;
//	char *file_ini = "realtime.ini";
//	printf("veh_snd:1");
	char *vehicle_str = NULL;
	int vehicle_pip = 0;
	int option;
	int repeat_comm_packet = 0;
	input_t input;
	float velocity_sav = 0;
	
	while ((option = getopt(argc, argv, "A:a:i:t:u:dsR")) != EOF)
	{
		switch (option)
		{
		case 'A':
			local_ipaddr = strdup(optarg);
			break;
		case 'a':
			remote_ipaddr = strdup(optarg);
			break;
		case 'i':
			interval = atoi(optarg);
			break;
		case 'u':
			udp_port = atoi(optarg);
			break;
		case 't':
			vehicle_str = strdup(optarg);
			break;
		case 's':
			use_db = 0;
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

//	if ((fpin = get_ini_section(file_ini, vehicle_str)) == NULL)
//	{
//		printf("veh_snd: can't get ini file %s, section %s\n", file_ini, vehicle_str);
//		fflush(stdout);
//		exit(EXIT_FAILURE);
//	}
//
//	vehicle_pip = get_ini_long(fpin, "PositionInPlatoon", 1);
	printf("veh_snd: Vehicle %s\n", vehicle_str);
	if( (strcmp(vehicle_str, "priu") != 0) && (strcmp(vehicle_str, "acco") != 0) && (strcmp(vehicle_str, "taur") != 0)) {
		printf("Vehicle ID must be one of priu, acco, or taur. Exiting...\n");
		exit(EXIT_FAILURE);
	}
	memset(&comm_pkt, 0, sizeof(veh_comm_packet_t));
	memcpy(&comm_pkt.object_id, vehicle_str, 4);
	
	if ((ptmr = timer_init(interval, ChannelCreate(0))) == NULL)
	{
		printf("timer_init failed\n");
		exit(EXIT_FAILURE);
	}

	get_local_name(hostname, MAXHOSTNAMELEN);

	/**  assumes DB_COMM variables were aleady created by another process */
	if (use_db != 0)
	{
		pclt = db_list_init(argv[0], hostname, domain, xport, NULL, 0, NULL, 0);
	}

	if (setjmp(exit_env) != 0)
	{
		printf("Sent %d messages\n", msg_count);
		if (use_db != 0)
		{
			db_list_done(pclt, NULL, 0, NULL, 0);
		}
		exit(EXIT_SUCCESS);
	}
	else
	{
		sig_ign(sig_list, sig_hand);
	}

	if ((socket = udp_peer2peer_init(&dst_addr, remote_ipaddr, local_ipaddr, udp_port, 0)) < 0)
	{
		printf("Failure to initialize socket from %s to %s on port %d\n", remote_ipaddr, local_ipaddr, udp_port);
		longjmp(exit_env, 2);
	} else {
		printf("Initialized socket from %s to %s on port %d\n", remote_ipaddr, local_ipaddr, udp_port);
	}


	if (use_db == 0)
	{
		comm_pkt.latitude = 37.91555;
		comm_pkt.longitude = -122.33487;
		strncpy(&comm_pkt.object_id[0], vehicle_str, 4);
	}

	while (1)
	{
		msg_count++;
		msg_count %= 128;

		db_clt_read(pclt, DB_INPUT_VAR, sizeof(input_t), &input); // dbvar 1000

		if (use_db != 0)
		{
			memset(&comm_pkt, 0, sizeof(veh_comm_packet_t));

//			if( (repeat_comm_packet != 0) && (msg_count % 2 == 0) )
//				db_clt_read(pclt, DB_REPEAT_COMM_PACKET_VAR, sizeof(comm_pkt), &comm_pkt); // db var number 499 veh_comm_packet_t
//			else
				db_clt_read(pclt, DB_COMM_TX_VAR, sizeof(comm_pkt), &comm_pkt); // db var number 499 veh_comm_packet_t
			/** 
			 * db var number 202 path_gps_point_t
			 * created and mintaned by gpsdb process
			**/
			db_clt_read(pclt, DB_GPS_GGA_VAR, sizeof(path_gps_point_t), &self_gps);
			comm_pkt.sequence_no = msg_count;
			comm_pkt.latitude = self_gps.latitude;
			comm_pkt.longitude = self_gps.longitude;
			comm_pkt.heading = self_gps.heading;
//			comm_pkt.my_pip = 3;
			// set tx time
			get_current_timestamp(&comm_pkt.ts);
			// set vehicle id
			memcpy(&comm_pkt.object_id, vehicle_str, 4);
		}
		/**
		 * don't use db, generate dummy data
		 * the genrated data should within the range enforced by the model in the file
		 * truckcontrol/src/vehcomm/BasicSafetyMessage-Volvo.txt
		 **/

		else // don't use db, generate dummy data
		{ 
			get_current_timestamp(&comm_pkt.ts);
			fcounter += 0.0001;
			debug_counter++;

			comm_pkt.node = 100;
			comm_pkt.global_time = 1;
			comm_pkt.user_float = fcounter;
			comm_pkt.user_float1 = fcounter;
			comm_pkt.user_ushort_1 = debug_counter % 256;
			comm_pkt.user_ushort_2 = debug_counter % 256;
			comm_pkt.my_pip = vehicle_pip;
			comm_pkt.maneuver_id = debug_counter % 128;
			comm_pkt.fault_mode = debug_counter % 16;
			comm_pkt.maneuver_des_1 = debug_counter % 128;
			comm_pkt.maneuver_des_2 = debug_counter % 128;
			comm_pkt.pltn_size = debug_counter % 16; //0-15
			comm_pkt.sequence_no = debug_counter % 128;
			comm_pkt.user_bit_1 = debug_counter % 128;
			comm_pkt.user_bit_2 = debug_counter % 128;
			comm_pkt.user_bit_3 = debug_counter % 128;
			comm_pkt.user_bit_4 = debug_counter % 128;
			comm_pkt.acc_traj = 0.123 + (debug_counter % 320); // 0-32767 0.01 m/s^s resolution
			comm_pkt.vel_traj = 0.123 + (debug_counter % 160); // 0-16383 0.01 m/s resolution
			comm_pkt.velocity = 0.123 + (debug_counter % 160); // 0-8191 0.02 m/s resolution
			comm_pkt.accel = 0.123 + (debug_counter % 20); // -2000-2001 0.01 m/s^s resoolution
			comm_pkt.range = 0.123 + (debug_counter % 127); // 0-32767 0.01 m resolution
			comm_pkt.rate = 0.123 + (debug_counter % 127); //0-16383 0.01 m/s resolution
			comm_pkt.latitude += 0.000001;
			comm_pkt.longitude += 0.000001;
			comm_pkt.heading = fcounter;
			
			printf("debug_counter %d\n", debug_counter);
			printf("fcounter %f\n", fcounter);
		}


		//enc_rval = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, BSMCACC, (void *)encod_buffer, sizeof(encod_buffer));
		enc_rval = vehcomm2BSM(encod_buffer, sizeof(encod_buffer), &comm_pkt);
		retval = ((enc_rval.encoded + 7) / 8);


		if (debug)
		{
			printf("veh_snd: Encoded message size: %d\n", retval);
			printf("veh_snd: Encoded message: ");
			for (i = 0; i < retval; i++)
			{
				printf("%02hhx ", encod_buffer[i]);
			}
			printf("\n");
		}
		if (enc_rval.encoded <= 0)
		{
			printf("veh_snd: Error... encoded bytes=%ld\n", enc_rval.encoded);
			input.sys_status |= SYSSTAT_VEH_SND_ENCODE_ERR;
		}
		else
		{
			input.sys_status &= (unsigned int)(!SYSSTAT_VEH_SND_ENCODE_ERR);

			if(comm_pkt.velocity == velocity_sav) {
				input.sys_status |= SYSSTAT_VEH_SND_VELOCITY_SAME;
			}
			else
				input.sys_status &= (unsigned int)(!SYSSTAT_VEH_SND_VELOCITY_SAME);
			velocity_sav = comm_pkt.velocity;

			// put the message into text file format
			memset(msg_str, 0, sizeof(msg_str));
			memcpy(&msg_str[3], encod_buffer, retval); //Offset message by 3 bytes because that's the offset
														//needed to read the messages from the accord and prius
			bytes_sent = sendto(socket, msg_str, retval+3, 0, (struct sockaddr *)&dst_addr, sizeof(dst_addr));// Added 3 bytes for above reason

			if (bytes_sent < 0)
			{
				perror("veh_snd: Error sneding data");
				input.sys_status |= SYSSTAT_VEH_SND_SEND_ERR;
			}
			else
				input.sys_status &= (unsigned int)(!SYSSTAT_VEH_SND_SEND_ERR);
			db_clt_write(pclt, DB_INPUT_VAR, sizeof(input_t), &input); // dbvar 1000

			if (debug)
			{
				xer_fprint(stdout, &asn_DEF_MessageFrame, encod_buffer);
			}

		}

		TIMER_WAIT(ptmr);
	}

	return 0;
}
