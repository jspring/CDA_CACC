/**\file
 * wrfiles_cacc3.c    File to save the data from the CAN and gps
 *              for the CACC selfs:
 *              Default time for saving data is
 *              every 75 msec.  
 *
 *              Every so often (according to a switch received in
 *              the startup line) the files will be closed and a new
 *              set of files opened.
 *
 *		This code has been extensivley rewritten (Sue, May 2009)
 *		to make	it easier to add and delete DB VARs and columns,
 *		and to allow data to be saved in memory between
 *		file writes if desired, instead of writing a line
 *		at a time. Specification for files that show data
 *		from many different DB VARs are now in cacc3_tables.c		
 *
 *
 * Copyright (c) 2000-2009   Regents of the University of California
 *
 *
 */
#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include <m56_can.h>
#include "data_log.h"
#include "db_sync.h"

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

#define IGNITION_OFF 255
#define IGN_CTR_MAX 100

/// Codes used in synchronization record sent to video recorder
#define VIDEO_STOP_RECORD	1
#define VIDEO_START_RECORD	2
#define VIDEO_SHUTDOWN		4


/**
 *	File pointers used for each file saved.
 */
FILE *f_comma = NULL;
FILE *f_commb = NULL;
FILE *f_commc = NULL;
FILE *f_self = NULL;
FILE *f_comme = NULL;
FILE *f_commf = NULL;
FILE *f_commg = NULL;

buff_typ *pfirst_buff;
buff_typ *pbuff_comma;
buff_typ *pbuff_commb;
buff_typ *pbuff_commc;
buff_typ *pbuff_self;
buff_typ *pbuff_comme;
buff_typ *pbuff_commf;
buff_typ *pbuff_commg;

extern timestamp_t timestamp;                 // used when reading back in
extern double utc_seconds_since_midnight;    // UTC seconds since midnight
extern double seconds_since_start;       // seconds since start of run

extern db_var_spec_t db_vars[];
extern int num_db_vars;
extern path_gps_point_t my_gps;
extern m56_ignition_status_t m56_ignition_status;

extern data_log_column_spec_t comm_speca[];
extern int num_afile_columns;
extern data_log_column_spec_t comm_specb[];
extern int num_bfile_columns;
extern data_log_column_spec_t comm_specc[];
extern int num_cfile_columns;
extern data_log_column_spec_t self_spec[];
extern int num_dfile_columns;
extern data_log_column_spec_t comm_spece[];
extern int num_efile_columns;
extern data_log_column_spec_t comm_specf[];
extern int num_ffile_columns;
extern data_log_column_spec_t comm_specg[];
extern int num_gfile_columns;

path_gps_point_t cabinet_gps; 		// GPS in cabinet

void do_buff_inits(int lines_to_save,
                int do_comma, int do_commb, int do_commc, int do_self,
                int do_comme, int do_commf, int do_commg,
                buff_typ **ppbuff_comma, buff_typ **ppbuff_commb, 
		buff_typ **ppbuff_commc, buff_typ **ppbuff_self, 
		buff_typ **ppbuff_comme, buff_typ **ppbuff_commf, 
		buff_typ **ppbuff_commg)
{
        if (do_self)
                *ppbuff_self = buff_init(f_self, lines_to_save);
        if (do_comma)
                *ppbuff_comma = buff_init(f_comma, lines_to_save);
        if (do_commb)
                *ppbuff_commb = buff_init(f_commb, lines_to_save);
        if (do_commc)
                *ppbuff_commc = buff_init(f_commc, lines_to_save);
        if (do_comme)
                *ppbuff_comme = buff_init(f_comme, lines_to_save);
        if (do_commf)
                *ppbuff_commf = buff_init(f_commf, lines_to_save);
        if (do_commg)
                *ppbuff_commg = buff_init(f_commg, lines_to_save);
	
}
 
int main(int argc, char *argv[])
{
	int option;
	int exitsig;
	double start_time;
	db_clt_typ *pclt;
	char hostname[MAXHOSTNAMELEN+1];
	int old_fileday = 99;	/// originally set to bad value to force search
	int serial_num;		/// set in open_data_log	
	char id_string[80];
	char monthday[80];
	char serialnum[80];
	char *vehicle_prefix = "m";

	int file_time = 15;     /// Number of minutes to record to a file 
	int interval = 50;	/// Number of milliseconds between saves 
	int lines_to_save = 100000;	/// when using memory; about 2 hours
	posix_timer_typ *ptimer;       /* Timing proxy */
	char *domain = DEFAULT_SERVICE; // usually no need to change this 
	int xport = COMM_OS_XPORT;	// set correct for OS in sys_os.h 
	int do_self = TRUE;		// command flag will turn off
	int do_comma = TRUE;		// command flag will turn off
	int do_commb = TRUE;		// command flag will turn off
	int do_commc = TRUE;		// command flag will turn off
	int do_comme = TRUE;		// command flag will turn off
	int do_commf = TRUE;		// command flag will turn off
	int do_commg = TRUE;		// command flag will turn off
	int do_video = TRUE;		// command flag will turn off
	int use_memory = FALSE;		// to save in memory, set to 1
	int use_ignition_status = FALSE;
	int ign_ctr = IGN_CTR_MAX;
	FILE *first_file = NULL;
	char *first_file_str;
	char tripdir[80];
	char tripstr[80];
	char apre[80];
	char bpre[80];
	char cpre[80];
	char dpre[80];
	char epre[80];
	char fpre[80];
	char gpre[80];
	int i;

	memset(id_string, 0, sizeof(id_string));
	memset(monthday, 0, sizeof(monthday));
	memset(serialnum, 0, sizeof(serialnum));
	memset(tripdir, 0, sizeof(tripdir));
	memset(tripstr, 0, sizeof(tripstr));
	memset(apre, 0, sizeof(apre));
	memset(bpre, 0, sizeof(bpre));
	memset(cpre, 0, sizeof(cpre));
	memset(dpre, 0, sizeof(dpre));
	memset(epre, 0, sizeof(epre));
	memset(fpre, 0, sizeof(fpre));
	memset(gpre, 0, sizeof(gpre));

	/* Read and interpret any user switches. */
	while ((option = getopt(argc, argv, "d:l:m:ABCDEFGt:uic:v")) != EOF) {
		switch(option) {
		case 'd':
			strcpy(tripdir, optarg);
			break;
	        case 'l':
			lines_to_save = atoi(optarg);
			break;
	        case 'm':
			file_time = atoi(optarg);
			if(file_time == 0)
			    file_time = 10000;
			break;
	        case 'A':
			do_comma = FALSE; 
			break;
	        case 'B':
			do_commb = FALSE; 
			break;
	        case 'C':
			do_commc = FALSE; 
			break;
	        case 'D':
			do_self = FALSE; 
			break;
	        case 'E':
			do_comme = FALSE; 
			break;
	        case 'F':
			do_commf = FALSE; 
			break;
	        case 'G':
			do_commg = FALSE; 
			break;
	        case 't':
			interval = atoi(optarg); 
			break;
	        case 'u':
			use_memory = TRUE; 
			break;
	        case 'i':
			use_ignition_status = TRUE; 
			break;
	        case 'c':
			vehicle_prefix = strdup(optarg);
			break;
	        case 'v':
			do_video = FALSE; 
			break;
	        default:
			printf("Usage: %s\n", argv[0]); 
			printf("    -d <trip directory> \n");
			printf("    -c <vehicle prefix> \n");
			printf("    -m <file time in minutes> \n");
			printf("    -i (use ignition status to stop file writing) \n");
			printf("    -v (don't save video) \n");
			printf("    -l <no. lines to save to file> \n");
			printf("    -t <loop time in ms> \n");
			printf("    -u (use memory to log data, then write to file at end)\n");
			printf("    -ABCDEFG (don't save abcdefg-file(s))\n");
			exit(EXIT_FAILURE);
	        }
	}
	strcpy(tripstr, tripdir+17);
	printf("tripdir %s\n", tripdir);
	printf("tripstr as string %s as decimal %d\n", tripstr, atoi(tripstr));
	printf("vehicle_prefix %s\n", vehicle_prefix);
	printf("%s:  do_comma %d  do_commb %d do_commc %d do_self %d do_comme %d do_commf %d do_commg %d do_video %d",
		 argv[0],
		do_comma,
		do_commb,
		do_commc,
		do_self, 
		do_comme,
		do_commf,
		do_commg,
		do_video);
	printf(" afile columns %d\n", num_afile_columns);
	printf(" bfile columns %d\n", num_bfile_columns);
	printf(" cfile columns %d\n", num_cfile_columns);
	printf(" dfile columns %d\n", num_dfile_columns);
	printf(" efile columns %d\n", num_efile_columns);
	printf(" ffile columns %d\n", num_ffile_columns);
	printf(" gfile columns %d\n", num_gfile_columns);

	/* Log in to the database (shared global memory).  Default to the
	 * the current host. Assumes other processes that create variables
	 * are already running, since list is NULL.
	 */
	get_local_name(hostname, MAXHOSTNAMELEN);
	if ((pclt = db_list_init(argv[0], hostname, domain, xport, 
			NULL, 0, NULL, 0)) == NULL) {
		printf("Database initialization error in %s.\n", argv[0]);
		exit(EXIT_FAILURE);
	}
        
	/* Setup a timer for every 'interval' msec. */
	if ((ptimer = timer_init(interval, DB_CHANNEL(pclt) )) == NULL) {
		printf("Unable to initialize wrfiles timer\n");
		exit(EXIT_FAILURE);
	}
       /* Open a set of files for recording data. Set up value of "firstfile"
	 * that will be used to handle timestamp and name set-up handling on
	 * reopens
	 */

        if (do_comma) {
		strcpy(apre, tripdir);
		strcat(apre, "/");
		strcat(apre, vehicle_prefix);
		strcat(apre, "a");
                if (first_file == NULL) {
                        open_data_log_infix(&f_comma, apre, ".dat",
                         &start_time, &old_fileday, &serial_num, monthday, serialnum, tripstr);
                        first_file = f_comma;
                        first_file_str = apre;
			pfirst_buff = pbuff_comma;
			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
                } else {
                        open_another_file(&f_comma, apre,
                                  id_string, ".dat");
                }
        }

        if (do_commb) {
		strcpy(bpre, tripdir);
		strcat(bpre, "/");
		strcat(bpre, vehicle_prefix);
		strcat(bpre, "b");
                if (first_file == NULL) {
                        open_data_log_infix(&f_commb, bpre, ".dat",
                         &start_time, &old_fileday, &serial_num, monthday, serialnum, tripstr);
                        first_file = f_commb;
                        first_file_str = bpre;
			pfirst_buff = pbuff_commb;
			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
                } else {
                        open_another_file(&f_commb, bpre,
                                  id_string, ".dat");
                }
        }
        if (do_commc) {
		strcpy(cpre, tripdir);
		strcat(cpre, "/");
		strcat(cpre, vehicle_prefix);
		strcat(cpre, "c");
                if (first_file == NULL) {
                        open_data_log_infix(&f_commc, cpre, ".dat",
                         &start_time, &old_fileday, &serial_num, monthday, serialnum, tripstr);
                        first_file = f_commc;
                        first_file_str = cpre;
			pfirst_buff = pbuff_commc;
			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
                } else {
                        open_another_file(&f_commc, cpre,
                                  id_string, ".dat");
                }
	}
        if (do_self) {
		strcpy(dpre, tripdir);
		strcat(dpre, "/");
		strcat(dpre, vehicle_prefix);
		strcat(dpre, "d");
                if (first_file == NULL) {
                        open_data_log_infix(&f_self, dpre, ".dat",
                         &start_time, &old_fileday, &serial_num, monthday, serialnum, tripstr);
                        first_file = f_self;
                        first_file_str = dpre;
			pfirst_buff = pbuff_self;
			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
                } else {
                        open_another_file(&f_self, dpre,
                                  id_string, ".dat");
                }
        }

        if (do_comme) {
		strcpy(epre, tripdir);
		strcat(epre, "/");
		strcat(epre, vehicle_prefix);
		strcat(epre, "e");
                if (first_file == NULL) {
                        open_data_log_infix(&f_comme, epre, ".dat",
                         &start_time, &old_fileday, &serial_num, monthday, serialnum, tripstr);
                        first_file = f_comme;
                        first_file_str = epre;
			pfirst_buff = pbuff_comme;
			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
                } else {
                        open_another_file(&f_comme, epre,
                                  id_string, ".dat");
                }
        }

        if (do_commf) {
		strcpy(fpre, tripdir);
		strcat(fpre, "/");
		strcat(fpre, vehicle_prefix);
		strcat(fpre, "f");
                if (first_file == NULL) {
                        open_data_log_infix(&f_commf, fpre, ".dat",
                         &start_time, &old_fileday, &serial_num, monthday, serialnum, tripstr);
                        first_file = f_commf;
                        first_file_str = fpre;
			pfirst_buff = pbuff_commf;
			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
                } else {
                        open_another_file(&f_commf, fpre,
                                  id_string, ".dat");
                }
        }

        if (do_commg) {
		strcpy(gpre, tripdir);
		strcat(gpre, "/");
		strcat(gpre, vehicle_prefix);
		strcat(gpre, "g");
                if (first_file == NULL) {
                        open_data_log_infix(&f_commg, gpre, ".dat",
                         &start_time, &old_fileday, &serial_num, monthday, serialnum, tripstr);
                        first_file = f_commg;
                        first_file_str = gpre;
			pfirst_buff = pbuff_commg;
			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
                } else {
                        open_another_file(&f_commg, gpre,
                                  id_string, ".dat");
                }
        }
        if (use_memory)
                do_buff_inits(lines_to_save,
			do_comma, do_commb, do_commc, do_self, do_comme, do_commf, do_commg, 
			&pbuff_comma, &pbuff_commb, &pbuff_commc, &pbuff_self, &pbuff_comme, 
			&pbuff_commf, &pbuff_commg);


	if(( exitsig = setjmp(exit_env)) != 0) {
		if (use_memory) {
			if (do_comma) 
				buff_done(pbuff_comma);
			if (do_commb) 
				buff_done(pbuff_commb);
			if (do_commc) 
				buff_done(pbuff_commc);
			if (do_self)
				buff_done(pbuff_self);
			if (do_comme) 
				buff_done(pbuff_comme);
			if (do_commf) 
				buff_done(pbuff_commf);
			if (do_commg) 
				buff_done(pbuff_commg);
		}
		if (f_comma)
			fclose(f_comma);
		if (f_commb)
			fclose(f_commb);
		if (f_commc)
			fclose(f_commc);
		if (f_self)
			fclose(f_self);
		if (f_comme)
			fclose(f_comme);
		if (f_commf)
			fclose(f_commf);
		if (f_commg)
			fclose(f_commg);

		db_list_done(pclt, NULL, 0, NULL, 0);
	    if(exitsig == IGNITION_OFF) {
	    	system("echo 'Going down!' > /tmp/lcdtemp");
		printf("Ignition off!\n");
		sleep(5);
		system("/home/cacc3/test/stop_cacc3.sh ignorewrfiles");
	    }
	    printf("%s exiting with signal %d\n", argv[0], exitsig);
	    exit(EXIT_SUCCESS);
	} else
		sig_ign(sig_list, sig_hand);

	fflush(stdout);

	for(;;) {
	/* Read the time of day.  This will be used as the "official
	 * timestamp" in all of the files. */
		get_current_timestamp(&timestamp);

                /* Read in all DB vars, whether or not the associated files
                 * are being written
                 */
                for (i = 0; i < num_db_vars; i++){
                        db_clt_read(pclt,
                                db_vars[i].db_id_num,
                                db_vars[i].size,
                                db_vars[i].var_pointer);
		}

        	utc_seconds_since_midnight = TS_TO_SEC(&my_gps.utc_time);
                if (do_comma)
                        save_to_spec(f_comma, timestamp, use_memory,
                                pbuff_comma, num_afile_columns,
                                &comm_speca[0]);

                if (do_commb) 
                        save_to_spec(f_commb, timestamp, use_memory,
                                pbuff_commb, num_bfile_columns,
                                &comm_specb[0]);

                if (do_commc) 
                        save_to_spec(f_commc, timestamp, use_memory,
                                pbuff_commc, num_cfile_columns,
                                &comm_specc[0]);

		if (do_self)
                        save_to_spec(f_self, timestamp, use_memory,
                                pbuff_self, num_dfile_columns,
                                &self_spec[0]);

                if (do_comme) 
                        save_to_spec(f_comme, timestamp, use_memory,
                                pbuff_comme, num_efile_columns,
                                &comm_spece[0]);

                if (do_commf) 
                        save_to_spec(f_commf, timestamp, use_memory,
                                pbuff_commf, num_ffile_columns,
                                &comm_specf[0]);

                if (do_commg) 
                        save_to_spec(f_commg, timestamp, use_memory,
                                pbuff_commg, num_gfile_columns,
                                &comm_specg[0]);

		/** Check if time to close and reopen data logs 
		  */

               if (reopen_data_log_infix(&first_file, file_time, first_file_str,
                                ".dat", &start_time, &old_fileday, &serial_num,
                                monthday, serialnum, tripstr, pfirst_buff))  {
                        // done for first file in reopen_data_log

			sprintf(id_string, "%s%s%s", monthday, tripstr, serialnum);
			if (do_video) {
				struct tm synctime;
				struct timespec now;
				sync_record_typ sync;

				clock_gettime(CLOCK_REALTIME, &now);
				localtime_r(&now.tv_sec, &synctime);
				sync.month = atoi(monthday)/100;
				sync.day = atoi(monthday) % 100;
				sync.flag = VIDEO_START_RECORD;
				sync.seq_num = serial_num - 1;
				sync.trip_num = atoi(tripstr);
				sync.time_duration = file_time * 60;
 printf("sync.month %d sync.day %d sync.flag %d sync.seq_num %d sync.trip_num %d sync.time_duration %d\n", sync.month, sync.day, sync.flag, sync.seq_num, sync.trip_num, sync.time_duration);
				db_clt_write(pclt, DB_SYNC_RECORD_VAR, 
					sizeof(sync_record_typ), &sync);
			}
                        if (do_comma)
                                reopen_another_file(&f_comma, apre,
                                        id_string, ".dat", pbuff_comma);
                        if (do_commb)
                                reopen_another_file(&f_commb, bpre,
                                        id_string, ".dat", pbuff_commb);
                        if (do_commc)
                                reopen_another_file(&f_commc, cpre,
                                        id_string, ".dat", pbuff_commc);
                        if (do_self) 
                                reopen_another_file(&f_self, dpre, 
					id_string, ".dat", pbuff_self);
                        if (do_comme)
                                reopen_another_file(&f_comme, epre,
                                        id_string, ".dat", pbuff_comme);
                        if (do_commf)
                                reopen_another_file(&f_commf, fpre,
                                        id_string, ".dat", pbuff_commf);
                        if (do_commg)
                                reopen_another_file(&f_commg, gpre,
                                        id_string, ".dat", pbuff_commg);

                        }
                        if (use_memory)
                                do_buff_inits(lines_to_save,
                                        do_comma, do_commb, do_commc, do_self,
					do_comme, do_commf, do_commg,
                                        &pbuff_comma, &pbuff_commb,  &pbuff_commc, 
					&pbuff_self,  &pbuff_comme,  &pbuff_commf,  
					&pbuff_commg);
		if((m56_ignition_status.ignition_status == 1) && use_ignition_status) {
			ign_ctr--;
			if(ign_ctr == 0) {
				printf("Exiting due to ignition off\n");
				longjmp(exit_env, IGNITION_OFF);
			}
		}
		else
			ign_ctr = IGN_CTR_MAX;

		TIMER_WAIT(ptimer);
	}
}
