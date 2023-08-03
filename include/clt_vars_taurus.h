#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "taurus_can.h"
#include "vehicle_common.h"
#include "long_comm.h"
#include "Leddar.h"

#define DB_TAURUS_MSG92_TYPE     0x92
#define DB_TAURUS_MSG98_TYPE     0x98
#define DB_TAURUS_MSG99_TYPE     0x99
#define DB_TAURUS_MSG215_TYPE     0x215
#define DB_LONG_OUTPUT_TYPE	8000

#define DB_TAURUS_MSG92_VAR      DB_TAURUS_MSG92_TYPE
#define DB_TAURUS_MSG98_VAR      DB_TAURUS_MSG98_TYPE
#define DB_TAURUS_MSG99_VAR      DB_TAURUS_MSG99_TYPE
#define DB_TAURUS_MSG215_VAR     DB_TAURUS_MSG215_TYPE

db_id_t db_vars_list_taurus[] =  {
	{DB_TAURUS_MSG92_VAR, sizeof(taurus_long_lat_accel_t)},
	{DB_TAURUS_MSG98_VAR, sizeof(taurus_accel_cmd_t)},
	{DB_TAURUS_MSG99_VAR, sizeof(taurus_accel_cmd_t)},
	{DB_TAURUS_MSG215_VAR, sizeof(taurus_wheel_speed_t)},
	{DB_INPUT_VAR, sizeof(input_t)},
	{DB_OUTPUT_VAR, sizeof(output_t)},
	{DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_TX_VAR, sizeof(veh_comm_packet_t)},
	{DB_LEDDAR_1_VAR, sizeof(leddar_t)},
	{DB_LEDDAR_2_VAR, sizeof(leddar_t)},
	{DB_GPS_GGA_VAR, sizeof(path_gps_point_t)},
	{DB_REPEAT_COMM_PACKET_VAR, sizeof(veh_comm_packet_t)},
};
#define NUM_DB_VARS (sizeof(db_vars_list_taurus)/sizeof(db_id_t))
