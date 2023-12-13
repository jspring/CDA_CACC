/* read_pixell.c - db_slv client that passes sensor data to the database
*/

#include "db_include.h"
#include "udp_utils.h"
#include "pixell.h"
//#include "pixell_defs.h"

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

static db_id_t db_pixell_list[] = {
{DB_PIXELL_VAR, sizeof(db_LidarDataFiltered_t)},
};
#define NUM_PIXELL_VARS	sizeof(db_pixell_list)/sizeof(db_id_t)

const char *usage = "-v verbose -a <remote IP address, def. 127.0.0.1> -p <UDP port, def. 16000> -i <loop interval, def. 1000 ms>  -v (verbose) -d (don't use database)";

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

	LidarDataFiltered_t LidarDataFiltered;
	db_LidarDataFiltered_t db_LidarDataFiltered;
	db_LidarDataFiltered_t db_LidarDataFiltered_readback;

	timestamp_t ts;
	int ts_ms = 0;

	struct sockaddr_in src_addr;
	int socklen = sizeof(src_addr);
	int bufsize = sizeof(LidarDataFiltered_t);
	char buf[bufsize];
	int num_bytes = 0;

	char *remote_ipaddr = "127.0.0.1";
	short port = 16000;
	char *portstr = NULL;
	int use_database = 1;
	int fd;
	int create_dbvar = 0;

        while ((option = getopt(argc, argv, "a:p:i:vdc")) != EOF) {
        	switch(option) {
			case 'a':
				remote_ipaddr = strdup(optarg);
				break;
			case 'p':
				port = atoi(optarg);
				portstr = strdup(optarg);
				break;
			case 'i':
				interval = atoi(optarg);
				break;
        	case 'v':
        		verbose = 1;
        		break;
        	case 'd':
        		use_database = 0;
        		break;
        	case 'c':
        		create_dbvar = 1;
        		break;
        	default:
               	printf("Usage: %s %s\n", argv[0], usage);
               	exit(EXIT_FAILURE);
               	break;
        	}
        }

            if ( (fd = udp_allow_all(port)) < 0) {
                 printf("2 Failure to initialize UDP socket from %s on port %d our error number %d\n",
                		 remote_ipaddr, port, fd);
                 longjmp(exit_env, 2);
        }

        if(use_database) {
			get_local_name(hostname, MAXHOSTNAMELEN);
			if(!create_dbvar) {
				if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
						NULL, 0, NULL, 0))
					== NULL) {
						printf("Could not db_list_init\n");
						exit(EXIT_FAILURE);
				}
			}
			else{
				if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
						db_pixell_list, 1, NULL, 0))
					== NULL) {
						printf("Could not db_list_init\n");
						exit(EXIT_FAILURE);
				}
			}
			printf("Got to 2\n");
        }

    	if (setjmp(exit_env) != 0)
    	{
////    		printf("Sent %d messages\n", msg_count);
//    		if (use_db)
    		{
    			db_list_done(pclt, db_pixell_list, 1, NULL, 0);
    		}
    		exit(EXIT_SUCCESS);
    	}
    	else
    	{
    		sig_ign(sig_list, sig_hand);
    	}

    	printf("Got to 3 fd %d\n", fd);

        if ((ptimer = timer_init( interval, ChannelCreate(0) )) == NULL) {
                fprintf(stderr, "Unable to initialize delay timer\n");
                exit(EXIT_FAILURE);
        }
        printf("Got to 4\n");


//	status.two_message_periods = 80; 		// 2*10 msec

	get_current_timestamp(&ts);
	printf("Got to 5\n");

	for(;;) {
		memset(buf, 0, sizeof(buf));
		memset(&LidarDataFiltered, 0, sizeof(LidarDataFiltered_t));
		memset(&db_LidarDataFiltered, 0, sizeof(db_LidarDataFiltered_t));
		memset(&db_LidarDataFiltered_readback, 0, sizeof(db_LidarDataFiltered_t));
		if ((num_bytes = recvfrom(fd, buf, 124, 0, (struct sockaddr *)&src_addr, (socklen_t *)&socklen)) <= 0)
			perror("pixell");
		else {
			memcpy(&LidarDataFiltered, buf, sizeof(LidarDataFiltered_t));
			if(verbose) {
				get_current_timestamp(&ts);
				LidarDataFiltered.ts = ts;
				print_timestamp(stdout, &ts);
				printf("num_bytes %d status %d numTargets %d\n", num_bytes, LidarDataFiltered.pixell_status, LidarDataFiltered.pixell_numTargets);
				for (i=2; i<num_bytes;) {
					printf("%2.2d ", i);
					for(j=0; j<40; j++)
						printf("%#2.2hhx ", buf[i+j]);
					printf("\n");
						i += 40;
				}
				printf("\n");
			}
			if(use_database) {
				db_LidarDataFiltered.pixell_status = LidarDataFiltered.pixell_status;
				db_LidarDataFiltered.pixell_numTargets = LidarDataFiltered.pixell_numTargets;
				db_LidarDataFiltered.ts = LidarDataFiltered.ts;
//				int swab;

				for (i=0; i<10;i++) {
//						swab = ((LidarDataFiltered.pixell_x[i] << 24) & 0xFF000000);
//						swab |= ((LidarDataFiltered.pixell_x[i] << 8) & 0xFF0000);
//						swab |= ((LidarDataFiltered.pixell_x[i] >> 8) & 0xFF00);
//						swab |= ((LidarDataFiltered.pixell_x[i] >> 24) & 0xFF);
//						LidarDataFiltered.pixell_x[i] = (int)swab;
//
//						swab = ((LidarDataFiltered.pixell_y[i] << 24) & 0xFF000000);
//						swab |= ((LidarDataFiltered.pixell_y[i] << 8) & 0xFF0000);
//						swab |= ((LidarDataFiltered.pixell_y[i] >> 8) & 0xFF00);
//						swab |= ((LidarDataFiltered.pixell_y[i] >> 24) & 0xFF);
//						LidarDataFiltered.pixell_y[i] = (int)swab;
//
//						swab = ((LidarDataFiltered.pixell_z[i] << 24) & 0xFF000000);
//						swab |= ((LidarDataFiltered.pixell_z[i] << 8) & 0xFF0000);
//						swab |= ((LidarDataFiltered.pixell_z[i] >> 8) & 0xFF00);
//						swab |= ((LidarDataFiltered.pixell_z[i] >> 24) & 0xFF);
//						LidarDataFiltered.pixell_z[i] = (int)swab;
					db_LidarDataFiltered.pixell_x[i] = (float)((signed int)LidarDataFiltered.pixell_x[i] / 1000000.0);
					db_LidarDataFiltered.pixell_y[i] = (float)((signed int)LidarDataFiltered.pixell_y[i] / 1000000.0);
					db_LidarDataFiltered.pixell_z[i] = (float)((signed int)LidarDataFiltered.pixell_z[i] / 1000000.0);
					if(verbose) {
						print_timestamp(stdout, &ts);
						printf("xf[%d] %.3f y[%d] %.3f z[%d] %.3f\n",
								i,
								db_LidarDataFiltered.pixell_x[i],
								i,
								db_LidarDataFiltered.pixell_y[i],
								i,
								db_LidarDataFiltered.pixell_z[i]
						);
						print_timestamp(stdout, &ts);
						printf("xd[%d] %#X y[%d] %#X z[%d] %#X\n",
								i,
								LidarDataFiltered.pixell_x[i],
								i,
								LidarDataFiltered.pixell_y[i],
								i,
								LidarDataFiltered.pixell_z[i]
						);
					}
				}
				if(verbose)
					printf("\n\n");
				if( (db_clt_write(pclt, DB_PIXELL_VAR, sizeof(db_LidarDataFiltered_t), (db_LidarDataFiltered_t *)&db_LidarDataFiltered)) == FALSE)
					printf("db_LidarDataFiltered failed\n");
				if (db_clt_read(pclt, DB_PIXELL_VAR, sizeof(db_LidarDataFiltered_t), (db_LidarDataFiltered_t *)&db_LidarDataFiltered_readback) == FALSE )
					printf("db_LidarDataFiltered_readback failed\n");
				if(verbose) {
					for (i=0; i<10;i++)
						print_timestamp(stdout, &ts);
					    printf("READBACK DB_PIXELL_VAR %d sizeof(db_LidarDataFiltered_t) %d status %d %d numTargets %d %d xf[%d] %.3f %.3f y[%d] %.3f %.3f z[%d] %.3f %.3f\n",
					    	DB_PIXELL_VAR,
							sizeof(db_LidarDataFiltered_t),
							db_LidarDataFiltered.pixell_status,
							db_LidarDataFiltered_readback.pixell_status,
							db_LidarDataFiltered.pixell_numTargets,
							db_LidarDataFiltered_readback.pixell_numTargets,
							i,
							db_LidarDataFiltered.pixell_x[i],
							db_LidarDataFiltered_readback.pixell_x[i],
							i,
							db_LidarDataFiltered.pixell_y[i],
							db_LidarDataFiltered_readback.pixell_y[i],
							i,
							db_LidarDataFiltered.pixell_z[i],
							db_LidarDataFiltered_readback.pixell_z[i]
					);

				}

			}


		}
	TIMER_WAIT(ptimer);
	}
	return 0;
}
