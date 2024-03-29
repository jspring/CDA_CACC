/**\file
 * Utility functions for accessing database or configuration
 *		files to be called by longitudinal control code. 
 *
 * Copyright (c) 2002   Regents of the University of California
 *
 */

#include "long_ctl_prius.h"

/**
 * long_get_output_stream will open an output data file with a unique name in
 * the format "prefix_monxx_yyy.dat" where prefix is a parameter to the
 * function, month is a 3 letter month abbreviation. Serial numbers go from 0 to 999, then start over,
 * so at most 999 data files with the same prefix can be gathered
 * on a given day.. 
 *
 * If the value in old_fileday does not match the current date, the
 * serial number (file_serialno) will be reset to zero.  This will happen
 * the first time the function is called (since old_fileday was deliberately
 * initialized to 99, an invalid day) and if the system ever runs through
 * a day change at midnight.
 *
 * May be used either to open a single file or to open multiple files with
 * the same prefix in the same run. Tries to open files with the same
 * prefix starting from serial number 0, and opens the first one not
 * present.
FILE * long_get_output_stream(char *prefix)
{
	struct timeb timeptr_raw;
	struct tm time_converted;
	static int old_fileday = 99;	 initialized to illegal value 
	static int file_serialno;
	char filename[80];
	bool_typ file_exists = TRUE;
	FILE *fpout;

	 Get date information. 
	ftime ( &timeptr_raw );
	_localtime ( &timeptr_raw.time, &time_converted );

	 If old_fileday is not the same as time_converted.tm_mday, reset
	 * the serial number to zero.  This will happen first time through, or
	 * if we run through midnight. 
	if ( old_fileday != time_converted.tm_mday )
		file_serialno = 0;
	old_fileday = time_converted.tm_mday;

	while (file_exists == TRUE) {
		if (file_serialno > 999)
			return NULL;
		sprintf(filename, "%s_%s%2.2d_%3.3d.dat",
			prefix, month_names[time_converted.tm_mon],
			time_converted.tm_mday, file_serialno);
		fpout = fopen( filename, "r" );
		if ( fpout == NULL )
			file_exists = FALSE;
		else {
			fclose(fpout);
			file_serialno++;
	        }
	}

	 We've found a valid serial number (and filename for fpout file is
	 * still stored in the variable filename).  
	
	fpout = fopen( filename, "w" );
	file_serialno++;
	printf( "Recording data to %s\n", filename );
	return (fpout);

}
 */
	       
/**
 * long_set_params sets up parameters in the control structure
 * used when calculating the control commands. Currently 
 * parses command line arguments and reads task numbers and parameters
 * from initialization files. Design for demo configuration files still to
 * be determined, current set-up used for cmdtest. 
 */
 
int long_set_params(int argc, char **argv, long_params *pparams)

{
        int ch;
	char vehicle_str[80];
	FILE *fpin;

        /* Set up default filenames */
	strcpy(pparams->avcs_cfg_file, "/home/truckcontrol/test/realtime.ini"); 
	strcpy(pparams->long_cfg_file, "long_ctl.ini");
	strcpy(pparams->data_file, "long_ctl.out");
	strcpy(vehicle_str, "C2");

        /* Specify test number with command line argument */
        while ((ch = getopt(argc, argv, "f:o:p:r:s:v:")) != EOF) {
                switch (ch) {
                case 'f': strcpy(pparams->long_cfg_file, optarg);
			  printf("long_cfg_file %s\n", pparams->long_cfg_file);
                          break;
                case 'o': strcpy(pparams->data_file, optarg);
			  printf("data_file %s\n", pparams->data_file);
                          break;
                case 'r': pparams->cmd_test = atoi(optarg);
                          break;
//                case 'p': pparams->vehicle_position = atoi(optarg);
//                          break;
                case 'v': strcpy(vehicle_str, optarg); 
                          break;
                default:  printf("Usage: %s ", argv[0]);
                          printf("-f Command task configuration file\n"); 
                          printf("-r Command task number\n"); 
                          return 0;
                          break;
                }
        }
//	printf("pparams->vehicle_position %d\n", 
//		pparams->vehicle_position);
	if (strcmp(vehicle_str,"Prius") == 0)  
		pparams->vehicle_type = VEH_TYPE_PRIUS;

	printf("Vehicle %s,type %d\n", vehicle_str, pparams->vehicle_type);
        if ((fpin = get_ini_section("/home/truckcontrol/test/realtime.ini", vehicle_str))
								 == NULL) {
                printf("long_utils: can't get ini file %s, section %s\n",
                   "/home/truckcontrol/test/realtime.ini", vehicle_str);
                fflush(stdout);
                return 0;
        }
	else
		printf("long_utils: initialization file %s\n", "/home/truckcontrol/test/realtime.ini");

	printf("max_engine speed %.3f, engine_reference_torque %.3f\n",
		pparams->max_engine_speed, pparams->engine_reference_torque);
	printf("retarder_reference_torque %.3f\n",
		pparams->retarder_reference_torque);
	/* should these be configurable in the file? */
	pparams->max_iterations = 0;
        pparams->ctrl_interval = get_ini_long(fpin,"CtrlInterval", 20); //in ms
        pparams->vehicle_position = get_ini_long(fpin,"PositionInPlatoon", 1);
        pparams->mdl_lidar = get_ini_bool(fpin,"UseMDL", FALSE);
        pparams->denso_lidar = get_ini_bool(fpin,"UseLidar", FALSE);
	
        return 1;
}

/**
 * long_print_params prints long_params member read from realtime.ini
 * and command line arguments, for documentation or debugging. 
 *
 */
void long_print_params(FILE *fp, long_params *pparams)
{
	fprintf(fp, "avcs_cfg_file %s\n", pparams->avcs_cfg_file);
	fprintf(fp, "long_cfg_file %s\n", pparams->long_cfg_file);
	fprintf(fp, "data_file %s\n", pparams->data_file);
	fprintf(fp, "max_engine_speed %.3f\n", pparams->max_engine_speed);
	fprintf(fp, "engine_reference_torque %.3f\n", pparams->engine_reference_torque);
	fprintf(fp, "retarder_reference_torque %.3f\n", pparams->retarder_reference_torque);
	fprintf(fp, "cmd_test %d\n", pparams->cmd_test);
}


/** Input variable list for longitudinal control process on Prius */

static db_id_t prius_dbv_list[] = {
	{DB_PRIUS_MSG99_VAR, sizeof(prius_accel_cmd_t)},
	{DB_PRIUS_MSG343_VAR, sizeof(prius_accel_cmd_status_t)}
};

static int prius_dbv_list_size = sizeof(prius_dbv_list)/sizeof(db_id_t);

void long_set_dbv_list(long_ctrl *pctrl)
{
	long_params *pparams = &pctrl->params;
	fprintf(stderr, "Setting dbv list for ");
	switch (pparams->vehicle_type) {
	case VEH_TYPE_PRIUS:
		fprintf(stderr, " prius\n");
		pparams->dbv_list = &prius_dbv_list[0].id;
		pparams->dbv_list_size = sizeof(prius_dbv_list)/sizeof(int);
		break;
	default:
		fprintf(stderr, "Did not get db list for Prius!\n");
		break;
	}
}

/**
 * Create jbus database variables and make them active.
 * long_database_init logs in to the database process and makes sure the
 * variables in the jdbv_list and in the in_dbv_list for the
 * control process have been created; clt_create will return an error
 * if this variable has already been created, so we ignore the return
 * value; this is just to make sure the variable has been created
 * when we read or set a trigger on it. Normally the process that writes
 * the variable will have created it. 
 *
 * The active flag is set in the structure kept by the J1939 support
 * code for the rdj1939 process, for each variable on the vehicle list
 * of jbus variables. For now this active flag is only used by
 * long_read_vehicle_state to decide whether to read a variable from
 * the database, but later the J1939 support code may use this flag
 * to decide whether or not to write the variable to the data base.
 * So it is important that the flag is set during initialization
 * for any jbus database variable of interest. For now, the variable
 * numbers of active jbus variables for each vehicle type are kept in
 * the staticly initialized array bus40_jdbv_list, bus60_jdbv_list and
 * trk_jdbv_list -- later we may want to initialize these dynamically
 * for different applications.
 */ 
db_clt_typ *long_database_init(char *phost, char *pclient, long_ctrl *pctrl)
{
        db_clt_typ *pclt;
	long_params *pparams = &pctrl->params;
	int i;

	long_set_dbv_list(pctrl);

        if ((pclt = clt_login(pclient, phost, DEFAULT_SERVICE,
                                                 COMM_OS_XPORT)) == NULL)
                return((db_clt_typ *) NULL);

	// The following will need to be generalized if we ever use this code
	// for a project other thatn the truck project.
	for (i = 0; i < prius_dbv_list_size; i++) {
		int db_num = prius_dbv_list[i].id; 
		clt_create(pclt, db_num, db_num, prius_dbv_list[i].size);
	}
		
	return(pclt);
}

/**
 * long_trigger makes sure a variable is written before it is read. 
 * This assumes that these variables are updated frequently. 
 * Also assumes that database variable and type numbers are the same.
 */
int long_trigger(db_clt_typ *pclt, unsigned int db_num)
{
        pid_t trig_pid;
        trig_info_typ trig_msg;

        if (!clt_trig_set(pclt, db_num, db_num))
                return 0;
        while (1) {
                trig_pid = clt_ipc_receive(pclt, &trig_msg, sizeof(trig_msg));
                 if (DB_TRIG_VAR(&trig_msg) == db_num)
                        break;
        } 
        return 1;
}

/**
 * long_wait_for_init
 * Makes sure essential database variables used to set state have been
 * written before control begins. Later may interact with DVI or other vehicles.
 */
int long_wait_for_init(db_clt_typ *pclt, long_ctrl *pctrl)
{
	/* Trigger on a few frequently updated variables to make sure
	 * J1939 bus is alive.
	 */
        if (!long_trigger(pclt, DB_PRIUS_MSG99_VAR))
                return 0;
        if (!long_trigger(pclt, DB_PRIUS_MSG343_VAR))
                return 0;
	
	/* Useful fields of messages that might be missing are initialized
	 * to error values in database.
	 */
                                 
	long_read_vehicle_state(pclt, pctrl);
        return 1;
}
/**
 * Update vehicle state fields for a variable db_num, after data has
 * been read from data base into pdata_val.
 */ 
void long_update_fields_from_dbv(int db_num, long_vehicle_state *pstate,
			long_params *pparams, void *pdata_val)
{
	long_dig_in_typ *pdig_in;
	prius_accel_cmd_t *pprius_accel_cmd;
	prius_accel_cmd_status_t *pprius_accel_cmd_status;

	switch (db_num) {
	case DB_PRIUS_MSG99_VAR:
		pprius_accel_cmd = (prius_accel_cmd_t *) pdata_val;
		pstate->accel_cmd = pprius_accel_cmd->accel_cmd;
		break;
	case DB_PRIUS_MSG343_VAR:
		pprius_accel_cmd_status = (prius_accel_cmd_status_t *) pdata_val;
		pstate->accel_cmd_status= pprius_accel_cmd_status->accel_cmd_status;
		break;
	default:
		fprintf(stderr, "Unknown db_num %d in long_update_fields...\n", db_num);
		break;
	}
}

/**
 * long_read_vehicle_state
 *
 * Used by run_tasks to update vehicle state from J1939 and other sensor
 * data available in the longitudinal input variable. 
 *
 * Can also be used by other trace programs, e.g. veh_trk -v
 */ 
int long_read_vehicle_state(db_clt_typ *pclt, long_ctrl *pctrl)
{
	long_vehicle_state *pstate = &pctrl->vehicle_state;
	long_params *pparams = &pctrl->params;
        db_data_typ db_data;
	int i;
	int clt_read_error = 0;

	// Read Jbus variables
	for (i = 0; i < pparams->dbv_list_size; i++) {
		int db_num = pparams->dbv_list[i];
        	if (clt_read(pclt, db_num, db_num, &db_data))
			long_update_fields_from_dbv(db_num, pstate,
				pparams, &db_data.value.user);
		else
		    clt_read_error++;
	}

	// Read other input variables
	for (i = 0; i < prius_dbv_list_size; i++) {
		int db_num = prius_dbv_list[i].id;
        	if (clt_read(pclt, db_num, db_num, &db_data))
			long_update_fields_from_dbv(db_num, pstate,
				pparams, &db_data.value.user);
		else
		    clt_read_error++;
	}

	/* For now always returns true, later may want to return 0 if there
	 * are read errors.
	 */
        return 1;
}
