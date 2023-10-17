/**\file
 *
 * can_tx.c
 *
 * Periodically sends a message to the specified CAN port. 
 * Messages have ID sent from the command line and data that
 * is just an incrementing integer.
 *
 * For SSV_CAN or ECAN, sample usage:
 *
 * can_tx -p /dev/can1 -i <integer id>  -t <seconds (flost)>
 *	  -e (if uses extended frames) -v debugging information
 *
 * For Steinhoff driver, sample usage:
 *
 * can_tx -p 1 -i <integer id>  -t <seconds (flost)>
 *	  -e (if uses extended frames) -v debugging information
 *
 * Code is built with different libraries to match the kind of
 * CAN card on the system. You must change the Makefile flags
 * for this to happen. 
 */

#include <db_include.h>
#include <sys_os.h>
#include "db_clt.h"
#include "sys_rt.h"
#include "timestamp.h"
#include "can_defs.h"
#include "can_client.h"
#include "can_dbvars.h"
#include <candef.h>   // Steinhoff include files
#include <canstr.h>
#include <canglob.h>


/** global definitions for signal handling */
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

static db_id_t db_steinhoff_vars_list[] = {
        {0 , sizeof(db_steinhoff_out_t)},
};
#define NUM_STEINHOFF_VARS (sizeof(db_steinhoff_vars_list)/sizeof(db_id_t))

int main(int argc, char **argv)
{

	char hostname[MAXHOSTNAMELEN];
	char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
	db_clt_typ *pclt;               /// data server client pointer
	int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h

	float secs = 1.0;
	unsigned int id = 0;
	unsigned char extended = 0;
	unsigned char data[24] = {0};
	long count=0;
	char port= 0;
	db_steinhoff_out_t db_steinhoff_out;
	int opt;
	int verbose = 0;	// by default silent
	int dbnum = 0; 
	int millisecs = 1000;	// timer interval is in milliseconds
	int repetitions = 1;
	posix_timer_typ *ptmr;	
	int i;
	int use_db = 1;
	int create_db_vars = 0;
	short Acceleration = 0; //+ for acceleration, 0x98 message; - for deceleration/braking, 0x99 message
	unsigned char Brake = 0; //+ for acceleration, 0x98 message; - for deceleration/braking, 0x99 message
	int write_err = ERR_OK;
	int msg_size = 0;
	int cld = 0; //send command line data

	while ((opt = getopt(argc, argv, "A:B:ep:m:i:n:s:t:vcd")) != -1) {
		switch (opt) {
			case 'A':
				Acceleration = (short)atoi(optarg);
				use_db = 0;
				break;
			case 'B':
				Brake = (unsigned char)atoi(optarg);
				use_db = 0;
				break;
			case 'e':
				extended = 1;
				break;
			case 'p':
				port= (char)atoi(optarg);
				break;
			case 'i':
				id = (unsigned int)atoi(optarg);
				break;
			case 'n':
				dbnum = (int)strtol(optarg, NULL, 0);
				break;
			case 's':
				msg_size = atoi(optarg);
				break;
			case 't':
				secs = atof(optarg);
				millisecs = secs*1000.0;
				if (millisecs < 1) {
					millisecs = 1;
					repetitions = (1.0/(secs * 1000.0) + 0.5);
				} else {
					millisecs = secs * 1000.0 + 0.5;
				}
			   break;
			case 'v':
				verbose = 1;
				break;
			case 'c':
				create_db_vars = 1;
				break;
			case 'd':
				use_db = 0;
				break;
			default:
				printf("Usage: %s -p <port> ", argv[0]);
				printf("-i <ID> -t <seconds> -e <extended frames>\n");
				printf("example: %s -p /dev/can -i 1000 -t 0.1\n",
						argv[0]);
				exit(1);
                }
        }
        memset(data, 0, sizeof(data));
	if( (id == 0) || (msg_size == 0) || (port== 0) || ( (dbnum == 0) && ((Acceleration == 0) && (Brake == 0))) ) {
		printf("Usage: id (-i), message_size (-s), database number (-n), and port (-p) are REQUIRED options\n");
		printf("id %d message size %d database number %d port %s\n", id, msg_size,dbnum, port);
		exit(EXIT_FAILURE);
	}

    if(Brake != 0){
      	create_db_vars = 0;
       	use_db = 0;
       	db_steinhoff_out.size = msg_size;
       	db_steinhoff_out.data[2] = Brake;
    }
    if(Acceleration != 0){
       	create_db_vars = 0;
       	use_db = 0;
     }
	canhdl_t hdl;
	short resp;
	struct can_object msg;
	if((port == 1) || (port == 2))
	if((ConnectDriver(port, "CANDRV1", &hdl)) < 0) // physical channel 1
		exit(-1);
	if((port == 3) || (port == 4))
	if((ConnectDriver(port - 2, "CANDRV2", &hdl)) < 0) // physical channel 2
		exit(-1);

    if(use_db != 0){
         get_local_name(hostname, MAXHOSTNAMELEN);
         if(create_db_vars) {
			db_steinhoff_vars_list[0].id = dbnum;
                        if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
                                db_steinhoff_vars_list, NUM_STEINHOFF_VARS, NULL, 0))
                                == NULL) {
                                        exit(EXIT_FAILURE);
                        }
                        db_steinhoff_out.id = id;
                        db_steinhoff_out.size = msg_size;
                        memset(&db_steinhoff_out.data[0], 0, 8);
			if( db_clt_write(pclt, dbnum, sizeof(db_steinhoff_out_t), &db_steinhoff_out)
				== FALSE) {
				fprintf(stderr, "can_tx: db_clt_write of db_steinhoff_out returned FALSE. Exiting....\n");
				exit(EXIT_FAILURE);
            		}
                }
         else {
            if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
               NULL, 0, NULL, 0))
               == NULL) {
               exit(EXIT_FAILURE);
           }
         }
     }

	// second parameter of timer_init is no longer used
	if ((ptmr = timer_init(millisecs, 0))== NULL) {
                printf("timer_init failed in %s.\n", argv[0]);
                exit(EXIT_FAILURE);
        }

        /* exit code on receiving signal */
        if (setjmp(exit_env) != 0) {
        	if((DisConnectDriver(&hdl)) < 0) // physical channel 1
        		exit(-1);
                if(create_db_vars != 0)
                        db_list_done(pclt, db_steinhoff_vars_list, NUM_STEINHOFF_VARS, NULL, 0);
                else
                if(use_db != 0)
                        db_list_done(pclt, NULL, 0, NULL, 0);

		fprintf(stderr, "%s exiting, send count %ld\n",
			argv[0], count);
		fflush(stdout);
		exit(EXIT_SUCCESS);
        } else
                sig_ign(sig_list, sig_hand);
        //Send 0 to enable control
		// Output Frame
		msg.frame_inf.inf.DLC = 8;
		msg.frame_inf.inf.FF = StdFF; // standard frame
		msg.frame_inf.inf.RTR = 0;
		msg.id = id; // CAN ID
		msg.data[0] = 0;
		msg.data[1] = 0;
		msg.data[2] = 0;
		msg.data[3] = 0;
		msg.data[4] = 0;
		msg.data[5] = 0;
		msg.data[6] = 0;
		msg.data[7] = 0;
		resp = CanWrite(hdl, &msg );

	for(;;) {
		if(use_db){
			db_clt_read(pclt, dbnum, sizeof(db_steinhoff_out_t), &db_steinhoff_out);
			// Output Frame
			memset(&msg, 0, sizeof(struct can_object));
			msg.frame_inf.inf.DLC = 2;
			msg.frame_inf.inf.FF = StdFF; // standard frame
			msg.frame_inf.inf.RTR = 0;
			if((id == 0x98) || (id == 0x99))
				msg.id = id; // CAN ID
			else{
				printf("Bad id %#X Exiting....\n", id);
				exit(EXIT_FAILURE);
			}


			msg.data[0] = db_steinhoff_out.data[0];
			msg.data[1] = db_steinhoff_out.data[1];
			resp = CanWrite(hdl, &msg );
			printf("CAN Status: %04x resp: %04x\n", CanGetStatus(hdl, &msg), resp);
		}
		else{
			if(Brake != 0) {
				data[2] = Brake;
			}
			else {
				if(Acceleration != 0) {


					// Output Frame
					msg.frame_inf.inf.DLC = 2;
					msg.frame_inf.inf.FF = StdFF; // standard frame
					msg.frame_inf.inf.RTR = 0;
					msg.id = id; // CAN ID
					msg.data[0] = (char)((Acceleration >> 8) & 0xFF);
					msg.data[1] = (char)(Acceleration & 0xFF);

					resp = CanWrite(hdl, &msg );
					printf("CAN Status: %04x resp: %04x\n", CanGetStatus(hdl, &msg), resp);

			        memset(data, 0, sizeof(data));
					db_steinhoff_out.id = id;
					db_steinhoff_out.size = msg_size;
					db_steinhoff_out.data[0] = (char)((Acceleration >> 8) & 0xFF);
					db_steinhoff_out.data[1] = (char)(Acceleration & 0xFF);
				}
			}
		}
		if (verbose) { 
			timestamp_t ts;
			get_current_timestamp(&ts);
			print_timestamp(stdout, &ts);
			printf(" %#hX ", db_steinhoff_out.id);
			for (i = 0; i < db_steinhoff_out.size; i++)
				printf("%#hhx ", db_steinhoff_out.data[i]);
			printf("\n");
			fflush(stdout);
		}
		

		TIMER_WAIT(ptmr);
		
	}
	return 0;

}
