#include "db_include.h"
#include "taurus_can.h"
#include "clt_vars_taurus.h"
#include "can_dbvars.h"
#include "long_comm.h"

static jmp_buf exit_env;
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
                longjmp(exit_env, code);
}

char *usage = "-v (verbose) -a <positive acceoeration> -b <negative deceleration> ";

int main(int argc, char *argv[]) {
        int option;
	int verbose = 0;
        int count = 0;
        float acceleration = 0;
        float deceleration = 0;
	output_t output;

        char hostname[MAXHOSTNAMELEN];
        char *domain = DEFAULT_SERVICE; /// on Linux sets DB q-file directory
        db_clt_typ *pclt;               /// data server client pointer
        int xport = COMM_OS_XPORT;      /// value set correctly in sys_os.h
        posix_timer_typ *ptimer;
        int ipc_message_error_ctr = 0;
        trig_info_typ trig_info;

        while ((option = getopt(argc, argv, "va:b:")) != EOF) {
                switch(option) {
                case 'v':
                        verbose = 1;
                        break;
                case 'a':
                        acceleration = atof(optarg);
                        break;
                case 'b':
                        deceleration = atof(optarg);
                        break;
                default:
                        printf("Usage: %s %s\n", argv[0], usage);
                        exit(EXIT_FAILURE);
                        break;
                }
        }

printf("Got to 1\n");
        get_local_name(hostname, MAXHOSTNAMELEN);
        if ( (pclt = db_list_init(argv[0], hostname, domain, xport,
                        NULL, 0, NULL, 0))
                == NULL) {
                        exit(EXIT_FAILURE);
        }

        if (setjmp(exit_env) != 0) {
		output.brake_level = 0;
		db_clt_write(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
                db_list_done(pclt, NULL, 0, NULL, 0);
                printf("%s: received %d CAN messages %d IPC message errors\n",
                        argv[0], count, ipc_message_error_ctr);
                exit(EXIT_SUCCESS);
        } else
               sig_ign(sig_list, sig_hand);


	db_clt_read(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);

	while(1) {
		output.brake_level = deceleration;
		db_clt_write(pclt, DB_OUTPUT_VAR, sizeof(output_t), &output);
		sleep(1);
	}
}
