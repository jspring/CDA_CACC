#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "leaf_can.h"
#include "can_dbvars.h"
#include "vehicle_common.h"
#include "long_comm.h"

#define DB_LEAF_MSG98_TYPE     0x98
#define DB_LEAF_MSG99_TYPE     0x99
#define DB_LEAF_MSG158_TYPE	0x158
#define DB_LEAF_MSG342_TYPE	342
#define DB_LEAF_MSG292_TYPE	0x292
#define DB_LEAF_MSG392_TYPE	0x392
#define DB_LEAF_MSG410_TYPE	0x410
#define DB_LEAF_MSG411_TYPE	0x411
#define DB_LEAF_MSG412_TYPE	0x412
#define DB_LEAF_MSG413_TYPE	0x413
#define DB_LEAF_MSG414_TYPE	0x414
#define DB_LEAF_MSG415_TYPE	0x415
#define DB_LEAF_MSG416_TYPE	0x416
#define DB_LEAF_MSG417_TYPE	0x417
#define DB_LEAF_MSG420_TYPE	0x420
#define DB_LEAF_MSG421_TYPE	0x421
#define DB_LEAF_MSG422_TYPE	0x422
#define DB_LEAF_MSG423_TYPE	0x423
#define DB_LEAF_MSG424_TYPE	0x424
#define DB_LEAF_MSG530_TYPE	0x530
#define DB_LEAF_MSG804_TYPE	804

#define DB_LEAF_MSG98_VAR	DB_LEAF_MSG98_TYPE
#define DB_LEAF_MSG99_VAR	DB_LEAF_MSG99_TYPE
#define DB_LEAF_MSG158_VAR	DB_LEAF_MSG158_TYPE
#define DB_LEAF_MSG342_VAR	DB_LEAF_MSG342_TYPE
#define DB_LEAF_MSG292_VAR	DB_LEAF_MSG292_TYPE
#define DB_LEAF_MSG392_VAR	DB_LEAF_MSG392_TYPE
#define DB_LEAF_MSG410_VAR	DB_LEAF_MSG410_TYPE
#define DB_LEAF_MSG411_VAR	DB_LEAF_MSG411_TYPE
#define DB_LEAF_MSG412_VAR	DB_LEAF_MSG412_TYPE
#define DB_LEAF_MSG413_VAR	DB_LEAF_MSG413_TYPE
#define DB_LEAF_MSG414_VAR	DB_LEAF_MSG414_TYPE
#define DB_LEAF_MSG415_VAR	DB_LEAF_MSG415_TYPE
#define DB_LEAF_MSG416_VAR	DB_LEAF_MSG416_TYPE
#define DB_LEAF_MSG417_VAR	DB_LEAF_MSG417_TYPE
#define DB_LEAF_MSG420_VAR	DB_LEAF_MSG420_TYPE
#define DB_LEAF_MSG421_VAR	DB_LEAF_MSG421_TYPE
#define DB_LEAF_MSG422_VAR	DB_LEAF_MSG422_TYPE
#define DB_LEAF_MSG423_VAR	DB_LEAF_MSG423_TYPE
#define DB_LEAF_MSG424_VAR	DB_LEAF_MSG424_TYPE
#define DB_LEAF_MSG530_VAR	DB_LEAF_MSG530_TYPE
#define DB_LEAF_MSG804_VAR	DB_LEAF_MSG804_TYPE

db_id_t db_vars_list_leaf[] =  {
	{DB_LEAF_MSG98_VAR, sizeof(leaf_torque_cmd_t)},
	{DB_LEAF_MSG99_VAR, sizeof(leaf_accel_cmd_t)},
	{DB_LEAF_MSG158_VAR, sizeof(leaf_vehicle_speed_t)},
	{DB_LEAF_MSG342_VAR, sizeof(leaf_steering_t)},
	{DB_LEAF_MSG292_VAR, sizeof(leaf_Veh_Accel_CAN4_t)},
	{DB_LEAF_MSG392_VAR, sizeof(leaf_PRNDL_Pos_t)},
	{DB_LEAF_MSG530_VAR, sizeof(leaf_torque_t)},
	{DB_LEAF_MSG410_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG411_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG412_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG413_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG414_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG415_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG416_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG417_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG420_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG421_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG422_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG423_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG424_VAR, sizeof(leaf_target_object_t)},
	{DB_LEAF_MSG804_VAR, sizeof(leaf_fuel_rate_t)},
	{DB_INPUT_VAR, sizeof(input_t)},
	{DB_OUTPUT_VAR, sizeof(output_t)},
	{DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t)},
        {DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t)},
        {DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t)},
        {DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t)},
        {DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t)},
        {DB_COMM_TX_VAR, sizeof(veh_comm_packet_t)},
        {DB_GPS_GGA_VAR, sizeof(path_gps_point_t)},
	{DB_REPEAT_COMM_PACKET_VAR, sizeof(veh_comm_packet_t)},
};
#define NUM_DB_VARS (sizeof(db_vars_list_leaf)/sizeof(db_id_t))
