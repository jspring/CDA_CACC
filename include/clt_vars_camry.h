#pragma once

#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "camry_can.h"
#include "long_comm.h"
#include "vehicle_common.h"
#include "can_dbvars.h"

#define DB_CAMRY_MSG99_TYPE     0x99
#define DB_CAMRY_MSGAA_TYPE     0xAA
#define DB_CAMRY_MSGB4_TYPE     0xB4
#define DB_CAMRY_MSG228_TYPE    0x228
#define DB_CAMRY_MSG2E6_TYPE    0x2E6
#define DB_CAMRY_MSG343_TYPE    0x343
#define DB_CAMRY_MSG399_TYPE    0x399
#define DB_CAMRY_MSG680_TYPE    0x680
#define DB_CAMRY_MSG681_TYPE    0x681
#define DB_CAMRY_MSG682_TYPE    0x682
#define DB_CAMRY_MSG683_TYPE    0x683
#define DB_CAMRY_MSG684_TYPE    0x684
#define DB_CAMRY_MSG685_TYPE    0x685
#define DB_CAMRY_MSG686_TYPE    0x686
#define DB_CAMRY_MSG687_TYPE    0x687
#define DB_CAMRY_MSG688_TYPE    0x688
#define DB_CAMRY_MSG689_TYPE    0x689
#define DB_CAMRY_MSG68A_TYPE    0x68A
#define DB_CAMRY_MSG68B_TYPE    0x68B
#define DB_CAMRY_MSG769_TYPE    769
#define DB_CAMRY_MSG771_TYPE    771
#define DB_CAMRY_MSG773_TYPE    773
#define DB_CAMRY_MSG775_TYPE    775
#define DB_CAMRY_MSG777_TYPE    777
#define DB_CAMRY_MSG779_TYPE    779
#define DB_CAMRY_MSG781_TYPE    781
#define DB_CAMRY_MSG783_TYPE    783
#define DB_CAMRY_MSG785_TYPE    785
#define DB_CAMRY_MSG787_TYPE    787
#define DB_CAMRY_MSG789_TYPE    789
#define DB_CAMRY_MSG791_TYPE    791

#define DB_CAMRY_MSG99_VAR      DB_CAMRY_MSG99_TYPE
#define DB_CAMRY_MSGAA_VAR      DB_CAMRY_MSGAA_TYPE
#define DB_CAMRY_MSGB4_VAR      DB_CAMRY_MSGB4_TYPE
#define DB_CAMRY_MSG228_VAR     DB_CAMRY_MSG228_TYPE
#define DB_CAMRY_MSG2E6_VAR     DB_CAMRY_MSG2E6_TYPE
#define DB_CAMRY_MSG343_VAR     DB_CAMRY_MSG343_TYPE
#define DB_CAMRY_MSG399_VAR     DB_CAMRY_MSG399_TYPE
#define DB_CAMRY_MSG680_VAR     DB_CAMRY_MSG680_TYPE
#define DB_CAMRY_MSG681_VAR     DB_CAMRY_MSG681_TYPE
#define DB_CAMRY_MSG682_VAR     DB_CAMRY_MSG682_TYPE
#define DB_CAMRY_MSG683_VAR     DB_CAMRY_MSG683_TYPE
#define DB_CAMRY_MSG684_VAR     DB_CAMRY_MSG684_TYPE
#define DB_CAMRY_MSG685_VAR     DB_CAMRY_MSG685_TYPE
#define DB_CAMRY_MSG686_VAR     DB_CAMRY_MSG686_TYPE
#define DB_CAMRY_MSG687_VAR     DB_CAMRY_MSG687_TYPE
#define DB_CAMRY_MSG688_VAR     DB_CAMRY_MSG688_TYPE
#define DB_CAMRY_MSG689_VAR     DB_CAMRY_MSG689_TYPE
#define DB_CAMRY_MSG68A_VAR     DB_CAMRY_MSG68A_TYPE
#define DB_CAMRY_MSG68B_VAR     DB_CAMRY_MSG68B_TYPE
#define DB_CAMRY_MSG769_VAR     DB_CAMRY_MSG769_TYPE
#define DB_CAMRY_MSG771_VAR     DB_CAMRY_MSG771_TYPE
#define DB_CAMRY_MSG773_VAR     DB_CAMRY_MSG773_TYPE
#define DB_CAMRY_MSG775_VAR     DB_CAMRY_MSG775_TYPE
#define DB_CAMRY_MSG777_VAR     DB_CAMRY_MSG777_TYPE
#define DB_CAMRY_MSG779_VAR     DB_CAMRY_MSG779_TYPE
#define DB_CAMRY_MSG781_VAR     DB_CAMRY_MSG781_TYPE
#define DB_CAMRY_MSG783_VAR     DB_CAMRY_MSG783_TYPE
#define DB_CAMRY_MSG785_VAR     DB_CAMRY_MSG785_TYPE
#define DB_CAMRY_MSG787_VAR     DB_CAMRY_MSG787_TYPE
#define DB_CAMRY_MSG789_VAR     DB_CAMRY_MSG789_TYPE
#define DB_CAMRY_MSG791_VAR     DB_CAMRY_MSG791_TYPE

db_id_t db_vars_list_camry[] =  {
	{DB_CAMRY_MSG99_VAR, sizeof(camry_accel_cmd_t)},
	{DB_CAMRY_MSGAA_VAR, sizeof(camry_wheel_speed_t)},
	{DB_CAMRY_MSGB4_VAR, sizeof(camry_vehicle_speed_t)},
	{DB_CAMRY_MSG228_VAR, sizeof(camry_long_lat_accel_t)},
	{DB_CAMRY_MSG2E6_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG343_VAR, sizeof(camry_accel_cmd_status_t)},
	{DB_CAMRY_MSG399_VAR, sizeof(camry_cruise_control_state_t)},
	{DB_CAMRY_MSG680_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG681_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG682_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG683_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG684_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG685_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG686_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG687_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG688_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG689_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG68A_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG68B_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG769_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG771_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG773_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG775_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG777_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG779_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG781_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG783_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG785_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG787_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG789_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_CAMRY_MSG791_VAR, sizeof(camry_radar_forward_vehicle_t)},
	{DB_INPUT_VAR, sizeof(input_t)},
	{DB_OUTPUT_VAR, sizeof(output_t)},
	{DB_STEINHOFF_ACCEL_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_BRAKE_OUT_VAR, sizeof(db_steinhoff_out_t)},
	{DB_STEINHOFF_MSG_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_STEINHOFF_MSG2_VAR, sizeof(db_steinhoff_msg_t)},
	{DB_COMM_TX_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_VIRTUAL_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_LEAD_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_SECOND_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_COMM_THIRD_TRK_VAR, sizeof(veh_comm_packet_t)},
	{DB_REPEAT_COMM_PACKET_VAR, sizeof(veh_comm_packet_t)},
	{DB_GPS_GGA_VAR, sizeof(path_gps_point_t)},
};
#define NUM_DB_VARS (sizeof(db_vars_list_camry)/sizeof(db_id_t))
