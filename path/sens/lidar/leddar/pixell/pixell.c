/* xsenx.c - db_slv client that passes sensor data to the database
**
**
**
*/

#include "db_include.h"
#include "pixell.h"
#include "clt_vars_pixell.h"
#include "pixell_defs.h"

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

int OpenTCPClient(char *controllerIP, char *port);


const char *usage = "-v verbose -a <angle>";

int main(int argc, char *argv[]) {
	int verbose = 0;
	int option;
	int count = 0;
	int pixell_error_ctr = 0;
	int i;
	int j;

	char hostname[MAXHOSTNAMELEN];
	char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
	db_clt_typ *pclt;               /// data server client pointer
	int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
	posix_timer_typ *ptimer;
	int interval = 1000;		//loop interval in ms

	request_header_t request_header;
	answer_header_t answer_header;
	element_header_t element_header;

	timestamp_t ts;
	int ts_ms = 0;

	struct sockaddr_in src_addr;
	int socklen = sizeof(src_addr);
	int BSMCACCSIZE = 20000;
	char BSMCACC_buf[BSMCACCSIZE];
	int num_bytes = 0;

	char *remote_ipaddr = "192.168.0.2";
	char *port = "48630";
	int fd;
	char get_data_request[] = {0x02, 0x00, 0x02, 0x70, 0x08, 0x00, 0x00, 0x00};


        while ((option = getopt(argc, argv, "a:p:i:v")) != EOF) {
        	switch(option) {
			case 'a':
				remote_ipaddr = strdup(optarg);
				break;
			case 'p':
				port = strdup(optarg);
				break;
			case 'i':
				interval = atoi(optarg);
				break;
        		case 'v':
        			verbose = 1;
        			break;
                default:
                	printf("Usage: %s %s\n", argv[0], usage);
                	exit(EXIT_FAILURE);
                	break;
        	}
        }

        if ( (fd = OpenTCPClient(remote_ipaddr, port)) < 0) {

                 printf("2 Failure to initialize socket from to %s on port %s our error number %d\n",
                		 remote_ipaddr, port, fd);
                 longjmp(exit_env, 2);
        }

    	get_local_name(hostname, MAXHOSTNAMELEN);
    	if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
    			NULL, 0, NULL, 0))
    		== NULL) {
    			printf("Could not db_list_init\n");
    			exit(EXIT_FAILURE);
    	}
    	printf("Got to 2\n");

    	if (setjmp(exit_env) != 0)
    	{
////    		printf("Sent %d messages\n", msg_count);
//    		if (use_db)
    		{
    			db_list_done(pclt, NULL, 0, NULL, 0);
    		}
    		exit(EXIT_SUCCESS);
    	}
    	else
    	{
    		sig_ign(sig_list, sig_hand);
    	}

    	printf("Got to 3\n");

        if ((ptimer = timer_init( interval, ChannelCreate(0) )) == NULL) {
                fprintf(stderr, "Unable to initialize delay timer\n");
                exit(EXIT_FAILURE);
        }
        printf("Got to 4\n");


	status.two_message_periods = 80; 		// 2*10 msec

	get_current_timestamp(&ts);
	printf("Got to 5\n");

	for(;;) {

	printf("Got to 6\n");
		num_bytes = sendto(fd, get_data_request, sizeof(get_data_request), 0, (struct sockaddr *)&src_addr, sizeof(src_addr));
	printf("Got to 7\n");

		if (num_bytes < 0)
			perror("pixell");

		if ((num_bytes = recvfrom(fd, BSMCACC_buf, BSMCACCSIZE, 0, (struct sockaddr *)&src_addr, (socklen_t *)&socklen)) <= 0)
			perror("pixell");
		else {
			for (i=0; i<num_bytes;) {
				printf("%2.2d ", i);
				for(j=0; j<16; j++)
					printf("%#2.2hhx ", BSMCACC_buf[i+j]);
				printf("\n");
				i += 16;
			}
			printf("\n");
			printf("num_bytes %d\n", num_bytes);
		}
	printf("Got to 8\n");

//		exit(EXIT_SUCCESS);
//		retval = send_pixell_poll(fd);
//
//		clt_ipc_receive(pclt, &trig_info, sizeof(trig_info));
//		if( DB_TRIG_VAR(&trig_info) == read_CAN_dbnum) {
//			count++;
//			memset(&db_pixell_msg.data[0], 0, 8);
//			db_clt_read(pclt, read_CAN_dbnum, sizeof(db_pixell_msg_t), &db_pixell_msg);
//			get_current_timestamp(&ts);
//			ts_ms = TS_TO_MS(&ts);
//			switch(db_pixell_msg.id) {
//                                case 0x11:
//					get_pixell_status(db_pixell_msg.data, &status);
//                                        check_msg_timeout(ts_ms, &status.ts_ms,
//                                                &status.two_message_periods,
//                                                &status.message_timeout_counter);
//					db_clt_write(pclt, DB_STATUS_VAR, sizeof(status_t), &status);
//                                        if(verbose)
//						printf("%s: selftest %d filter_valid %d GNSS_fix %d no_rotation_update %d compass_cal_mode %d clocksync_ok %d accX_OOR %d accY_OOR %d accZ_OOR %d gyrX_OOR %d gyrY_OOR %d gyrZ_OOR %d magX_OOR %d magY_OOR %d magZ_OOR %d OOR %d sync_in_detected %d sync_out_detected %d\n",
//							argv[0],
//							status.selftest,
//							status.filter_valid,
//							status.GNSS_fix,
//							status.no_rotation_update,
//							status.compass_cal_mode,
//							status.clocksync_ok,
//							status.accX_OOR,
//							status.accY_OOR,
//							status.accZ_OOR,
//							status.gyrX_OOR,
//							status.gyrY_OOR,
//							status.gyrZ_OOR,
//							status.magX_OOR,
//							status.magY_OOR,
//							status.magZ_OOR,
//							status.OOR,
//							status.sync_in_detected,
//							status.sync_out_detected
//
//						);
//                                        break;
//                                case 0x22:
//					get_pixell_orientation(db_pixell_msg.data, &orientation);
//                                        check_msg_timeout(ts_ms, &orientation.ts_ms,
//                                                &orientation.two_message_periods,
//                                                &orientation.message_timeout_counter);
//					db_clt_write(pclt, DB_ORIENTATION_VAR, sizeof(orientation_t), &orientation);
//                                        if(verbose)
//						printf("%s: roll %f pitch %f yaw %f\n",
//							argv[0],
//							orientation.roll,
//							orientation.pitch,
//							orientation.yaw
//						);
//                                        break;
//                                case 0x32:
//					get_pixell_rate_of_turn(db_pixell_msg.data, &rate_of_turn);
//                                        check_msg_timeout(ts_ms, &rate_of_turn.ts_ms,
//                                                &rate_of_turn.two_message_periods,
//                                                &rate_of_turn.message_timeout_counter);
//					db_clt_write(pclt, DB_RATE_OF_TURN_VAR, sizeof(rate_of_turn_t), &rate_of_turn);
//                                        if(verbose)
//						printf("%s: accX %f accY %f accZ %f\n",
//							argv[0],
//							rate_of_turn.gyrX,
//							rate_of_turn.gyrY,
//							rate_of_turn.gyrZ
//						);
//                                        break;
//                                case 0x34:
//					get_pixell_acceleration(db_pixell_msg.data, &acceleration);
//                                        check_msg_timeout(ts_ms, &acceleration.ts_ms,
//                                                &acceleration.two_message_periods,
//                                                &acceleration.message_timeout_counter);
//					db_clt_write(pclt, DB_ACCELERATION_VAR, sizeof(acceleration_t), &acceleration);
//                                        if(verbose)
//						printf("%s: gyrX %f gyrY %f gyrZ %f\n",
//							argv[0],
//							acceleration.accX,
//							acceleration.accY,
//							acceleration.accZ
//						);
//                                        break;
//                                default:
////					printf("Unknown message %#hx received\n");
//                                        break;
//                        }
//
//		}
	printf("Got to 9\n");
	TIMER_WAIT(ptimer);
	}
	return 0;
}

int OpenTCPClient(char *controllerIP, char *port) {
        struct addrinfo hints;
        struct addrinfo *result, *rp;
        int sfd, s;
        /* Obtain address(es) matching host/port */
        memset(&hints, 0, sizeof(struct addrinfo));
        hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
        hints.ai_socktype = SOCK_STREAM; /* TCP socket */
        hints.ai_flags = 0;
        hints.ai_protocol = 0;     /* Any protocol */
        printf("OpenTSCPConnection: Got to 1\n");
        s = getaddrinfo(controllerIP, port, &hints, &result);
        if (s != 0) {
                fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(s));
                exit(EXIT_FAILURE);
        }
        printf("OpenTCPClient: Got to 2\n");

        /* getaddrinfo() returns a list of address structures.
        Try each address until we successfully connect(2).
        If socket(2) (or connect(2)) fails, we (close the socket
        and) try the next address. */

        for (rp = result; rp != NULL; rp = rp->ai_next) {
                sfd = socket(rp->ai_family, rp->ai_socktype,
                rp->ai_protocol);
                if (sfd == -1) {
                        perror("socket");
                        continue;
                }
                printf("OpenTCPClient: Got to 2.1\n");
                if (connect(sfd, rp->ai_addr, rp->ai_addrlen) != -1) {
                        break;              /* Success */
                }
                perror("connect");
                printf("OpenTCPClient: Got to 2.2\n");
                close(sfd);
        }
        printf("OpenTCPClient: Got to 3\n");
        if (rp == NULL) {                /* No address succeeded */
                fprintf(stderr, "OpenTCPClient: Could not connect\n");
                return -1;
        }
        freeaddrinfo(result);       /* No longer needed */
        printf("OpenTCPClient succeeded sfd %d\n", sfd);
        return sfd;
}

void check_msg_timeout(int curr_ts_ms, int *prev_ts_ms, 
	unsigned char *two_message_periods, 
	unsigned int *message_timeout_counter) {
	if( (curr_ts_ms - *prev_ts_ms) > *two_message_periods ) {
	   ++*message_timeout_counter;
	   *prev_ts_ms = curr_ts_ms;
	}
}
