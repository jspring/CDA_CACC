/* FILE
 *   long_ctl_prius.h
 *
 * Structures and includes for longitudinal control programs.
 *
 */
#pragma once
#ifndef LONG_CTL_H
#define LONG_CTL_H

#include <sys_os.h>
#include <sys_list.h>
#include <sys_buff.h>
#include <sys/stat.h>
#include <sys_lib.h>
#include <sys_rt.h>
#include <sys_ini.h>
#include "db_comm.h"
#include "db_clt.h"
#include "db_utils.h"
#include "timestamp.h"
//#include "jbus_vars.h"
//#include "jbus_extended.h"
//#include "j1939.h"
//#include "j1939pdu_extended.h"
//#include "j1939db.h"
#include "evt300.h"
#include <timing.h>
#include "clt_vars.h"
#include "densolidar.h"
#include "mdl.h"
#include "path_gps_lib.h"
//#include "long_comm.h"
#include "veh_trk.h"
//#include "dvi.h"

// defined in long_ctl.c, used by tasks to control how much to print
extern int long_ctl_verbose;

/* Longitudinal control structures */

/* long_vehicle_state includes description of a vehicle's current state
 * as read from the database, or transmitted by vehicle-to-vehicle
 * communication from another vehicle. 
 */
typedef struct {
	float accel_cmd;
	float accel_cmd_status;
	float wheel_spd_FR;
	float wheel_spd_FL;
	float wheel_spd_RR;
	float wheel_spd_RL;
	float vehicle_speed;
	float long_accel;
	unsigned char CC_main_on;
	unsigned char CC_state;
	unsigned char CC_dash_set_speed;
	float lat_accel;
} long_vehicle_state;

/**
 * long_params 
 * Structure includes initialization conditions and parameters
 * for the current vehicle and the current test. These will not change
 * throughout the control, and are set once at the beginning, either
 * from command line arguments, or by reading an initialization file.
 *
 */

#define LONG_FILENAME_LEN 80
typedef struct {
  char avcs_cfg_file[LONG_FILENAME_LEN];	/* per vehicle file */
  char long_cfg_file[LONG_FILENAME_LEN];	/* longitudinal control */
  char data_file[LONG_FILENAME_LEN];	/* file for data gathering */
  int ctrl_interval;    /* time between command updates, millisecs */
  int max_iterations;	/* >0 maximum iterations control loop, 0 none */
  int cmd_test;		/* test to run; used only for cmdtest project! */
  /* vehicle characteristics */
  float max_engine_speed;		 /* maximum engine speed allowed */
  float engine_reference_torque; 	 /* vehicle's reference torque */
  float retarder_reference_torque;	/* retarder reference torque */
  int vehicle_type;	/* truck, 40-foot bus, 60-foot bus */
  int *dbv_list;
  int dbv_list_size;
  int vehicle_position;	/* currently also from script */
  bool_typ mdl_lidar;
  bool_typ denso_lidar;
} long_params;

/**
 * Data from last "data_size" control intervals can be saved in this
 * structure. The structure is initialized in init_tasks, and printed
 * out in exit_tasks.
 */

typedef struct {
  void *data_array;	/* circular buffer */
  int data_size;  /* number of structures malloced */
  int data_count; /* control intervals saved (<=data_size) */
  int data_start; /* index of oldest item in the circular buffer */
} long_data_buffer;

/**
 * Main longitudinal control structure. For real control, may include
 * other substructures that are common to all controllers. Data specific
 * to a particular control is passed by plong_private pointer
 */

typedef struct {
  struct timeb start_time;	
  long_vehicle_state vehicle_state;
  long_params params;
  cbuff_typ buff;
  void * plong_private;
} long_ctrl;

/* Default time interval for checking and changing control */
#define CTRL_MSECS       20

/* Vehicle type definitions */
#define VEH_TYPE_PRIUS 1

/* prototypes for functions in long_utils library */
//extern void init_circular_buffer(long_data_buffer *pbuff, int gather_data,
//				int item_size);
//extern int get_circular_index (long_data_buffer *pbuff);
extern void long_set_dbv_list(long_ctrl *pctrl);
extern FILE *long_get_output_stream (char *prefix);
extern int long_set_params(int argc, char **argv, long_params *pparams);
extern void long_print_params(FILE *fp, long_params *pparams);
extern db_clt_typ *long_database_init(char *phost, char *pclient,
				 long_ctrl *pctrl);
extern int long_trigger(db_clt_typ *pclt, unsigned int db_num);
extern int long_wait_for_init(db_clt_typ *pclt, long_ctrl *pctrl);
extern int long_read_vehicle_state(db_clt_typ *pclt, long_ctrl *pctrl);
extern int long_setled(db_clt_typ *pclt, int faultcode);
extern int long_rdswitch(long_dig_in_typ dig_in);
extern int long_rdbrake(long_dig_in_typ dig_in);

/* prototypes for functions that must be provided by individual controller */
extern int init_tasks(db_clt_typ *pclt, long_ctrl *pctrl, long_output_typ *pcmd);
extern int run_tasks(db_clt_typ *pclt, long_ctrl *pctrl, long_output_typ *pcmd);
extern int exit_tasks(db_clt_typ *pclt, long_ctrl *pctrl, long_output_typ *pcmd);

/* prototypes for reading and writing gpio */
extern int rdswitch( db_clt_typ *pclt );
extern int setled( db_clt_typ *pclt, int faultcode );

#define FLT_HI                 1
#define FLT_MED                2
#define FLT_LOW                3
#define FLT_AUTO               4
#define FLT_RDY2ROLL           5
#define LONG_CTL_ACTIVE        6
#define LONG_CTL_INACTIVE      7
#define TOGGLE_WATCHDOG        8

#define LED_RED        0x80 // Pin 9 (B7)
#define LED_AMBER      0x40 // Pin 10 (B6)
#define LED_NOT_AMBER  0xBF
#define LED_GRN        0x20 // Pin 11 (B5)
#define LED_BLUE       0x10 // Pin 12 (B4)
#define WDOGBIT        0x08 // Pin 13 (B3)
#define NWDOGBIT       0xF7

#endif /* LONG_CTL_H */
