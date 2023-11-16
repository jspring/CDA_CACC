/* Reads database to get object data from SMS radar and GPS data from mobile 
** SOBU-based GPS and saves to file.  
*/

#include <db_include.h>
#include "smsparse.h"
#include "sms_lib.h"
#include "path_gps_lib.h"
#include "gpio.h"

#define STDOUT	1
#define TRACE	2
#define USE_GPS	4

static int sig_list[] = 
{
    SIGINT,
    SIGQUIT,
    SIGTERM,
    SIGALRM,
    ERROR,
};

static jmp_buf exit_env;

static void sig_hand( int code )
{
	if(code == SIGALRM)
		return;
    longjmp( exit_env, code );
}

int main (int argc, char *argv[]) {

    db_clt_typ *pclt = NULL;
    char hostname[MAXHOSTNAMELEN+1];
    char *domain = DEFAULT_SERVICE; //for QNX6, use e.g. "ids"
    int xport = COMM_PSX_XPORT;    //for QNX6, no-op

    int output_mask = 0;    // 1=stdout, 2=trace file

    smsobj_typ obj_arr[65];  // smsobj_typ defined in smsparse
    int k;
    
    // each one has 5 objects; defined in smsparse.h
    smsobjarr_typ sms_arr[SMSARRMAX];
    int group_num;       // counts up to 13 groups of 5 each
    int obj_num;         // counts objects in a group
    objlist_typ objlist;
    int objlstidx;
    int objnum;
    smserr_t err;
    sms_id_err_t iderr;
    char errbuff[1024];

    path_gps_point_t veh_gps;
    unsigned char digin;
    int option;
    FILE *tracefd;
    char filename[MAX_LINE_LEN];
    struct tm tm;
    struct tm *ptm = &tm;
    time_t mytime;
    timestamp_t current_ts;
    posix_timer_typ *ptimer;
    int interval = 50;
    int num_cols;
    char num_saved_objs = 0;


    /* Read and interpret any user switches. */
    while ( (option = getopt( argc, argv, "o:i:n:" )) != EOF ) {
        switch( option ) {
            case 'o':
                output_mask = atoi(optarg);
                break;
            case 'i':
                interval = atoi(optarg);
                break;
            case 'n':
                num_saved_objs = atoi(optarg);
                break;
            default:
                printf("USAGE: %s -o <output mask, 1=stdout, 2=trace file, 4=use gps> -i interval -n no. of saved objects\n",
			argv[0]);
                exit( EXIT_FAILURE );
            }
        }

    get_local_name(hostname, MAXHOSTNAMELEN);

    if ((pclt = db_list_init(argv[0], hostname, domain, xport, 
        NULL, 0, NULL, 0)) == NULL) {
            printf("Database initialization error in %s.\n",  argv[0]);
            exit(EXIT_FAILURE);
            }

    if(output_mask & TRACE) {
	mytime = time(NULL);
	ptm = localtime(&mytime);
	sprintf(filename,"/big/data/readsmsgps.%02d-%02d-%02d_%02d:%02d:%02d",
		ptm->tm_mday, ptm->tm_mon, ptm->tm_year,
		ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    	tracefd = fopen(filename,"w");
    	if(tracefd == NULL) {
    		printf("Couln't open trace file. Exiting....");
    		exit(EXIT_FAILURE);
    		}
    	}
    if(output_mask & STDOUT) {
		tracefd = stdout;
		}
    if( setjmp( exit_env ) != 0 ) {
        db_list_done( pclt, NULL, 0, NULL, 0);

        if(tracefd)
        	fclose(tracefd);
        exit( EXIT_SUCCESS );
        }
    else
        sig_ign( sig_list, sig_hand );

        if ((ptimer = timer_init( interval, ChannelCreate(0) )) == NULL) {
                fprintf(stderr, "Unable to initialize delay timer\n");
                exit(EXIT_FAILURE);
        }

    //Initialize error structs
    memset(errbuff, 0, sizeof(errbuff));
    db_clt_write(pclt, DB_SMS_NUM_SAVED_OBJS_VAR, sizeof(char), &num_saved_objs);

    while(1) {
	// read list of SMS object ids
	if( db_clt_read(pclt, DB_SMS_OBJLIST_VAR, sizeof(objlist), 
		&objlist) == 0) {
		printf("READSMSGPS: Cannot read objlist\n");
		}

    	// read SMS data from data bucket and assign to flattened array
    	k = 0; // increment after each assignment
    	for (group_num = 0; group_num < SMSARRMAX; group_num++) {
    	    db_clt_read(pclt, DB_SMS_OBJARR0_VAR + group_num,
    	        sizeof(smsobjarr_typ), &sms_arr[group_num]);
    	    for (obj_num = 0; obj_num < MAXDBOBJS; obj_num++) {
    	         obj_arr[k]= sms_arr[group_num].object[obj_num];
    	         k++;
    	         }
    	    }

    	// read GPS data from mobile unit
	if(output_mask & USE_GPS)
		db_clt_read(pclt, DB_GPS_PT_RCV_VAR,
		sizeof(path_gps_point_t), &veh_gps);
//        if (db_clt_read( pclt, DB_DIG_IN_VAR,
//            sizeof(char), &digin) == FALSE)
//            fprintf( stderr, "READSMSGPS:clt_read( DB_DIG_IN_VAR ) failed.\n");



    	if(output_mask & TRACE) {
		if (!get_current_timestamp(&current_ts)) {
			printf("get_current_timestamp failed\n");
			return 0;
		}
		print_timestamp(tracefd, &current_ts);

		if(output_mask & USE_GPS) {
    			fprintf(tracefd, "%2.2d:%2.2d:%2.3f %2.2d:%2.2d:%2.3f ",
    				veh_gps.local_time.hour,
    				veh_gps.local_time.min,
    				veh_gps.local_time.sec +
    				veh_gps.local_time.millisec/1000.0,
    				veh_gps.utc_time.hour,
    				veh_gps.utc_time.min,
    				veh_gps.utc_time.sec +
    				veh_gps.utc_time.millisec/1000.0);
    			fprintf(tracefd, "%.7lf %.7lf %.7f %u ",
    				veh_gps.longitude, veh_gps.latitude, 
    				veh_gps.heading, digin & 0xFC);
		}
    		fprintf(tracefd, "%d %d ", objlist.num_obj, objlist.t_scan);
		for(objlstidx = 0; objlstidx < objlist.num_obj; objlstidx++) {
			objnum = obj_arr[objlist.arr[objlstidx]].obj & 0XFF;
			if( (0 < objnum) && (objnum < 64) ) {
				fprintf(tracefd, "%d %4.1f %4.1f %4.1f %4.1f %4.1f ",
					(char)(objnum),
					obj_arr[objlist.arr[objlstidx]].xrange,
					obj_arr[objlist.arr[objlstidx]].yrange,
					obj_arr[objlist.arr[objlstidx]].xvel,
					obj_arr[objlist.arr[objlstidx]].yvel,
					obj_arr[objlist.arr[objlstidx]].length);
			}
    	        }
		db_clt_read(pclt, DB_SMS_ERR_VAR, sizeof(smserr_t), &err);
		db_clt_read(pclt, DB_SMS_ID_ERR_VAR, sizeof(sms_id_err_t), &iderr);
		num_cols = sms_sprint_err(errbuff, &err, &iderr);
		fprintf(tracefd, "%s\n", errbuff);
    	    }
	TIMER_WAIT (ptimer);
    	}//end of while loop
}
