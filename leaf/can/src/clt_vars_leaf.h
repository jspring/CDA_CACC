#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "leaf_can.h"
#include "can_dbvars.h"
#include "vehicle_common.h"
#include "long_comm.h"

#define DB_LEAF_MSG2_TYPE	0x200
#define DB_LEAF_MSG98_TYPE	0x98
#define DB_LEAF_MSG99_TYPE	0x99
#define DB_LEAF_OBD2MSG107_TYPE	0x107
#define DB_LEAF_OBD2MSG108_TYPE	0x108
#define DB_LEAF_MSG158_TYPE	0x158
#define DB_LEAF_MSG1DA_TYPE	0x1DA
#define DB_LEAF_MSG292_TYPE	0x292
#define DB_LEAF_MSG2B0_TYPE	0x2B0
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
#define DB_LEAF_MSG804_TYPE	804

#define DB_LEAF_MSG2_VAR	DB_LEAF_MSG2_TYPE
#define DB_LEAF_MSG98_VAR	DB_LEAF_MSG98_TYPE
#define DB_LEAF_MSG99_VAR	DB_LEAF_MSG99_TYPE
#define DB_LEAF_OBD2MSG107_VAR	DB_LEAF_OBD2MSG107_TYPE
#define DB_LEAF_OBD2MSG108_VAR	DB_LEAF_OBD2MSG108_TYPE
#define DB_LEAF_MSG158_VAR	DB_LEAF_MSG158_TYPE
#define DB_LEAF_MSG1DA_VAR	DB_LEAF_MSG1DA_TYPE
#define DB_LEAF_MSG2B0_VAR	DB_LEAF_MSG2B0_TYPE
#define DB_LEAF_MSG292_VAR	DB_LEAF_MSG292_TYPE
#define DB_LEAF_MSG392_VAR	DB_LEAF_MSG392_TYPE
#define DB_LEAF_MSG804_VAR	DB_LEAF_MSG804_TYPE

db_id_t db_vars_list_leaf[] =  {
	{DB_LEAF_MSG2_VAR, sizeof(leaf_steering_t)},
	{DB_LEAF_MSG98_VAR, sizeof(leaf_torque_cmd_t)},
	{DB_LEAF_MSG99_VAR, sizeof(leaf_accel_cmd_t)},
	{DB_LEAF_MSG158_VAR, sizeof(leaf_vehicle_speed_t)},
	{DB_LEAF_MSG292_VAR, sizeof(leaf_Veh_Accel_CAN4_t)},
	{DB_LEAF_MSG2B0_VAR, sizeof(leaf_Torq_brake_ACC_t)},
	{DB_LEAF_MSG1DA_VAR, sizeof(leaf_torque_t)},
	{DB_LEAF_OBD2MSG107_VAR, sizeof(leaf_target_object_distance_t)},
	{DB_LEAF_OBD2MSG108_VAR, sizeof(leaf_target_relative_speed_mps_t)},
	{DB_LEAF_MSG804_VAR, sizeof(leaf_fuel_rate_t)},
	{DB_INPUT_VAR, sizeof(input_t)},
	{DB_OUTPUT_VAR, sizeof(output_t)},
	{DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_OBD2_OUT_4003_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_OBD2_IN_3003_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_STEINHOFF_MSG2_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_STEINHOFF_CHASSIS_3004_VAR, sizeof(db_steinhoff_msg_t)},
    {DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t)},
    {DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t)},
    {DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t)},
    {DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t)},
    {DB_COMM_TX_VAR, sizeof(veh_comm_packet_t)},
    {DB_GPS_GGA_VAR, sizeof(path_gps_point_t)},
	{DB_REPEAT_COMM_PACKET_VAR, sizeof(veh_comm_packet_t)},
};
#define NUM_DB_VARS (sizeof(db_vars_list_leaf)/sizeof(db_id_t))
