/**\file
 *
 * can_rx.c
 *
 * Listens and prints all the messages on the CAN port..
 *
 *	Usage for Steinhoff driver with SJA1000 chips:
 *		can_rx  -p 1 
 *
 * Links against different libraries for different CAN cards;
 * must change Makefile to match.
*/

#include <db_include.h>
#include <sys_os.h>
#include <sys/neutrino.h>
#include "local.h"
#include "sys_rt.h"
#include "timestamp.h"
#include "can_defs.h"
#include "can_client.h"
#include "can_dbvars.h"

static int sig_list[]=
{
        SIGINT,
        SIGQUIT,
        SIGTERM,
        SIGALRM,        // for timer
        ERROR
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
	{DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t)},
};
#define NUM_STEINHOFF_VARS (sizeof(db_steinhoff_vars_list)/sizeof(db_id_t))

int main(int argc, char **argv)
{
        char hostname[MAXHOSTNAMELEN];
        char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
        db_clt_typ *pclt;               /// data server client pointer
        int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h

	int size;
	int fd;
	can_msg_t can_msg;
        char *port = "1";	// use "1" to "4" for steinhoff
        int opt;
	int status;
	int read_err = 0;
	db_steinhoff_msg_t db_steinhoff_msg;
	int verbose = 0;
	int use_db = 1;
	int create_db_vars = 0;
	int dbnum = 0;
	

        while ((opt = getopt(argc, argv, "p:n:vdc")) != -1) {
                switch (opt) {
                  case 'p':
                        port = strdup(optarg);
                        break;
                  case 'n':
                        dbnum = atoi(optarg);
                        printf("dbnum %d\n", dbnum);
                        break;
                  case 'v':
                        verbose = 1;
                        break;
                  case 'd':
                        use_db = 0;
                        break;
                  case 'c':
                        create_db_vars = 1;
                        break;
                  default:
                        printf("Usage: %s -p <port>\n", argv[0]);
                        exit(1);
                }
        }

	printf("can_rx: trying to open %s\n", port); 
	fflush(stdout);

	fd = can_open(port, O_RDONLY);

	if (fd == -1)
		exit(EXIT_FAILURE);	// error message printed by can_open 

	printf("program %s, device name %s, fd: %d\n", argv[0], port, fd);
	fflush(stdout);

	(void) can_print_config(stdout, fd);
	fflush(stdout);

	(void) can_print_status(stdout, fd);
	fflush(stdout);
	if(use_db != 0){
		get_local_name(hostname, MAXHOSTNAMELEN);
		if(create_db_vars) {
			db_steinhoff_vars_list[0].id = dbnum;
			if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
				db_steinhoff_vars_list, NUM_STEINHOFF_VARS, NULL, 0))
				== NULL) {
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

	
        if(setjmp(exit_env) != 0) {
        	if(create_db_vars)
        		db_list_done(pclt, db_steinhoff_vars_list, NUM_STEINHOFF_VARS, NULL, 0);
		else
        	if(use_db != 0)
        		db_list_done(pclt, NULL, 0, NULL, 0);
        	if (read_err > 0)
        		printf(" %d read errors\n", read_err);
        	if (fd != -1)
        		status = can_close(&fd);
        	if (status != -1)
        		exit(EXIT_SUCCESS);
		else
			exit(EXIT_FAILURE);	// can_close prints error info
        } else
		sig_ign(sig_list, sig_hand);

	for(;;) {
		int i;
		timestamp_t ts;
		size = can_read(fd,&can_msg.id,(char *)NULL,can_msg.data,8);

		if (size < 0) 
			read_err++;	
		else {
			db_steinhoff_msg.size = size;
			db_steinhoff_msg.id = can_msg.id;
			memcpy(db_steinhoff_msg.data, can_msg.data, 8) ;
        		if(use_db != 0) {
				if(dbnum == 0)
        				db_clt_write(pclt, DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
				else
        				db_clt_write(pclt, dbnum, sizeof(db_steinhoff_msg_t), &db_steinhoff_msg);
			}
			get_current_timestamp(&ts);
			if(verbose) {
				print_timestamp(stdout, &ts);
				printf(" %#02lX ", db_steinhoff_msg.id);		
				printf(" %d bytes: ", size);		
				for (i = 0; i < size; i++)
					printf("%02hhx ", db_steinhoff_msg.data[i]);
				printf("\n");
				fflush(stdout);
			}
		}
	}
}

