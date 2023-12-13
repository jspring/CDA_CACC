#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "neweagle_steering.h"
#include "vehicle_common.h"

#define DB_DBW_MISC_TYPE		0x1F1
#define DB_STEERING_REPORT_TYPE		0x1F3
#define DB_STEERING_REQUEST_TYPE	0x2F3

#define DB_DBW_MISC_VAR DB_DBW_MISC_TYPE
#define DB_STEERING_REPORT_VAR DB_STEERING_REPORT_TYPE
#define DB_STEERING_REQUEST_VAR DB_STEERING_REQUEST_TYPE

db_id_t db_vars_list_neweagle_steering[] =  {
	{DB_INPUT_VAR, sizeof(input_t)},
	{DB_OUTPUT_VAR, sizeof(output_t)},
	{DB_STEINHOFF_STEERING_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_MSG_TYPE, sizeof(db_steinhoff_out_t)},
	{DB_GPS_GGA_VAR, sizeof(path_gps_point_t)},
	{DB_DBW_MISC_VAR , sizeof(neweagle_dbw_misc_t)},
	{DB_STEERING_REPORT_VAR , sizeof(neweagle_steering_report_t)},
	{DB_STEERING_REQUEST_VAR , sizeof(neweagle_steering_request_t)},
};
#define NUM_DB_VARS (sizeof(db_vars_list_neweagle_steering)/sizeof(db_id_t))
