#pragma once
#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "prius_can.h"
#include "long_comm.h"
#include "vehicle_common.h"

#define DB_PRIUS_MSG99_TYPE     0x99
#define DB_PRIUS_MSGAA_TYPE     0xAA
#define DB_PRIUS_MSGB4_TYPE     0xB4
#define DB_PRIUS_MSG1D3_TYPE    0x1D3
#define DB_PRIUS_MSG226_TYPE    0x226
#define DB_PRIUS_MSG228_TYPE    0x228
#define DB_PRIUS_MSG2E6_TYPE    0x2E6
#define DB_PRIUS_MSG343_TYPE    0x343
#define DB_PRIUS_MSG399_TYPE    0x399
#define DB_CAMRY_MSG680_TYPE    0x680
#define DB_CAMRY_MSG681_TYPE    0x681
#define DB_CAMRY_MSG682_TYPE    0x682
#define DB_CAMRY_MSG683_TYPE    0x683
#define DB_CAMRY_MSG684_TYPE    0x684
#define DB_CAMRY_MSG685_TYPE    0x685

#define DB_PRIUS_MSG99_VAR      DB_PRIUS_MSG99_TYPE
#define DB_PRIUS_MSGAA_VAR      DB_PRIUS_MSGAA_TYPE
#define DB_PRIUS_MSGB4_VAR      DB_PRIUS_MSGB4_TYPE
#define DB_PRIUS_MSG1D3_VAR     DB_PRIUS_MSG1D3_TYPE
#define DB_PRIUS_MSG226_VAR     DB_PRIUS_MSG226_TYPE
#define DB_PRIUS_MSG228_VAR     DB_PRIUS_MSG228_TYPE
#define DB_PRIUS_MSG2E6_VAR     DB_PRIUS_MSG2E6_TYPE
#define DB_PRIUS_MSG343_VAR     DB_PRIUS_MSG343_TYPE
#define DB_PRIUS_MSG399_VAR     DB_PRIUS_MSG399_TYPE
#define DB_CAMRY_MSG680_VAR     DB_CAMRY_MSG680_TYPE
#define DB_CAMRY_MSG681_VAR     DB_CAMRY_MSG681_TYPE
#define DB_CAMRY_MSG682_VAR     DB_CAMRY_MSG682_TYPE
#define DB_CAMRY_MSG683_VAR     DB_CAMRY_MSG683_TYPE
#define DB_CAMRY_MSG684_VAR     DB_CAMRY_MSG684_TYPE
#define DB_CAMRY_MSG685_VAR     DB_CAMRY_MSG685_TYPE


db_id_t db_vars_list_prius[] =  {
	{DB_PRIUS_MSG99_VAR, sizeof(prius_accel_cmd_t)},
	{DB_PRIUS_MSGAA_VAR, sizeof(prius_wheel_speed_t)},
	{DB_PRIUS_MSGB4_VAR, sizeof(prius_vehicle_speed_t)},
	{DB_PRIUS_MSG1D3_VAR, sizeof(prius_cruise_control_state_1D3_t)},
	{DB_PRIUS_MSG226_VAR, sizeof(prius_brake_t)},
	{DB_PRIUS_MSG228_VAR, sizeof(prius_long_lat_accel_t)},
	{DB_PRIUS_MSG2E6_VAR, sizeof(prius_radar_forward_vehicle_t)},
	{DB_PRIUS_MSG343_VAR, sizeof(prius_accel_cmd_status_t)},
	{DB_PRIUS_MSG399_VAR, sizeof(prius_cruise_control_state_t)},
	{DB_CAMRY_MSG680_VAR, sizeof(camry_prius_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG681_VAR, sizeof(camry_prius_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG682_VAR, sizeof(camry_prius_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG683_VAR, sizeof(camry_prius_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG684_VAR, sizeof(camry_prius_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG685_VAR, sizeof(camry_prius_radar_forward_vehicle_t)},
	{DB_INPUT_VAR, sizeof(input_t)},
	{DB_OUTPUT_VAR, sizeof(output_t)},
	{DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_STEINHOFF_MSG2_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_STEINHOFF_3003_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_STEINHOFF_3004_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_COMM_TX_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_REPEAT_COMM_PACKET_VAR, sizeof(veh_comm_packet_t)},
	{DB_GPS_GGA_VAR, sizeof(path_gps_point_t)},
};
#define NUM_DB_VARS (sizeof(db_vars_list_prius)/sizeof(db_id_t))
