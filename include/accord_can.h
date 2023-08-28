#pragma once

#include <can_dbvars.h>

/*******************************************************************************
 *      accord_accel_cmd
 *      Message ID      0x98
 *      Transmitted every 20 ms
 *
 *	accel_cmd
 *      Byte Position   0
 *      Bit Position    0
 *      Bit Length      8
 *
 */

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float accel_cmd;
} accord_accel_cmd_t;

#define acceleration_res  0.46698
#define acceleration_offset  3.74

static inline void set_accord_accel_cmd(unsigned char data[], accord_accel_cmd_t *p) {
	char accel_cmd_char;
	//  % acceleration = raw_value * 0.46698 - 3.74 (From documentation)
	accel_cmd_char =  (char)( (p->accel_cmd + acceleration_offset) / acceleration_res);
	data[0] = accel_cmd_char & 0xff;
//	printf("set_accord_accel_cmd: accel_cmd %.2f accel_cmd_char %hhd\n", p->accel_cmd, accel_cmd_char);
}

/*******************************************************************************
 *      accord_brake_cmd
 *      Message ID      0x99
 *      Transmitted every 20 ms
 *
 *	brake_override_cmd
 *      Byte Position   2
 *      Bit Position    16
 *      Bit Length	8 
 *
 */

#define deceleration_res  0.392157

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float brake_cmd;
} accord_brake_cmd_t;

static inline void set_accord_brake_cmd(unsigned char data[], accord_brake_cmd_t *p) {
	unsigned char brake_cmd_char;
	float temp;
	//  % deceleration = raw_value / 0.392157 (From documentation)
	temp =  p->brake_cmd / deceleration_res;
	brake_cmd_char =  (unsigned char)( p->brake_cmd / deceleration_res);
	data[0] = 0;
	data[1] = 0;
//	data[2] = brake_cmd_char & 0xff;
	data[2] = brake_cmd_char;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
//	printf("set_accord_brake_cmd: brake_cmd %.2f brake_cmd_char %u temp %.2f\n", p->brake_cmd, brake_cmd_char, temp);
}


/*******************************************************************************
 *      Vehicle_Speed_CAN2_MPH
 *      Message ID      0x158
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_ACCORD_MSG158_VAR
 *
 *	Vehicle_Speed_CAN2_MPH
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *
 */

#define WHEEL_SPEED_RES		0.00624
#define MPH_2_MPS		0.44704 

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float vehicle_speed_CAN2_MPS;
} accord_vehicle_speed_t;

static inline void get_accord_vehicle_speed(unsigned char *data, accord_vehicle_speed_t *p) {
	p->vehicle_speed_CAN2_MPS = (float)(((data[0] << 8) + data[1]) * WHEEL_SPEED_RES * MPH_2_MPS);
}

/*******************************************************************************
 *      Torque_CAN2_NM
 *      Message ID      0x530
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_ACCORD_MSG530_VAR
 *
 *	Torque_CAN2_NM
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *
 */

#define TORQUE_RES		0.02

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float generator_torque_CAN2_nm;
	float motor_torque_CAN2_nm;
} accord_torque_t;

static inline void get_accord_torque(unsigned char *data, accord_torque_t *p) {
	p->generator_torque_CAN2_nm = (float)(((data[0] << 8) + data[1]) * TORQUE_RES);
	p->motor_torque_CAN2_nm = (float)(((data[2] << 8) + data[3]) * TORQUE_RES);
}

/*******************************************************************************
 *	Steering wheel angle & rate
 *      Message ID     342 
 *      Transmitted every ?? ms
 *
 *	dbvar = DB_ACCORD_MSG342_VAR
 *
 *	Steering_Wheel_Pos_CAN_deg_
 *      Byte Position   0-1
 *      Bit Position    7-8 (Motorola ordering)
 *      Bit Length      16
 *
 *	Steering_Wheel_Rate_CAN_degps_ 
 *      Byte Position   2-3
 *      Bit Position    23-39 (Motorola ordering)
 *      Bit Length      16
 *

BO_ 342 BC__156: 8 Vector__XXX
 SG_ Steering_Wheel_Pos_CAN_deg_ : 7|16@0- (0.1,0) [-32.768|32.767] "deg" Vector__XXX
 SG_ Steering_Wheel_Rate_CAN_degps_ : 23|16@0- (1,0) [-127|127] "" Vector__XXX
 */

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float Steering_Wheel_Pos_CAN_deg;
	float Steering_Wheel_Rate_CAN_degps;
} accord_steering_t;

static inline void get_accord_steering(unsigned char *data, accord_steering_t *p) {
	short temp;

	temp = ((data[0] & 0xFF) << 8) + (data[1] & 0xFF) ;
	p->Steering_Wheel_Pos_CAN_deg = (float)(temp * 0.1);

	temp = ((data[2] & 0xFF) << 8) + (data[3] & 0xFF) ;
	p->Steering_Wheel_Rate_CAN_degps = (float)temp;
}

/*******************************************************************************
 *	Fuel_integrated_flow
 *      Message ID      804
 *      Transmitted every ?? ms
 *
 *	dbvar = DB_ACCORD_MSG804_VAR
 *
 *      Byte Position   2-3
 *      Bit Position    23-39 (Motorola ordering)
 *      Bit Length      16
 *

BO_ 804 BC_324: 8 Vector__XXX
 SG_ Fuel_Integrated_Flow_CAN__ : 23|16@0+ (1,0) [0|65535] "" Vector__XXX

 */

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float fuel_integrated_flow;
} accord_fuel_rate_t;

static inline void get_accord_fuel_rate(unsigned char *data, accord_fuel_rate_t *p) {
	short temp;

	temp = ((data[2] & 0xFF) << 8) + (data[3] & 0xFF) ;
	p->fuel_integrated_flow = (float)temp;
}


/*******************************************************************************
 *	PRNDL_Pos_CAN2 
 *      Message ID      0x392
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_ACCORD_MSG392_VAR
 *
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *
 */

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	unsigned char prndl;
} accord_PRNDL_Pos_t;

static inline void get_accord_PRNDL_Pos(unsigned char *data, accord_PRNDL_Pos_t *p) {
	p->prndl = data[5] & 0x0F;
}



/*******************************************************************************
 *	accord_target_object
 *      Message ID      0x410,411,412,413,414,415,416,417,420,421,422,423,424
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_ACCORD_MSG410_VAR
 *			DB_ACCORD_MSG411_VAR
 *			DB_ACCORD_MSG412_VAR
 *			DB_ACCORD_MSG413_VAR
 *			DB_ACCORD_MSG415_VAR
 *			DB_ACCORD_MSG416_VAR
 *			DB_ACCORD_MSG417_VAR
 *			DB_ACCORD_MSG420_VAR
 *			DB_ACCORD_MSG421_VAR
 *			DB_ACCORD_MSG422_VAR
 *			DB_ACCORD_MSG423_VAR
 *			DB_ACCORD_MSG424_VAR
 *
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *


BO_ 1040 HS_CAN5__410: 8 Vector__XXX
 SG_ ACC_relative_position_obj1_CAN5__ : 20|13@0- (1,0) [-4095|4095] "" Vector__XXX
 SG_ ACC_relative_distance_obj1_CAN5__ : 7|16@0+ (0.00775,0) [0|4095.9375] "m" Vector__XXX
 SG_ ACC_relative_velocity_obj1_CAN5_kph_ : 37|14@0- (0.056,0) [-8191|8191] "kph" Vector__XXX
 SG_ ACC_obj_1_counter_CAN5__ : 61|2@0+ (1,0) [0|3] "" Vector__XXX
 SG_ ACC_obj_1_checksum_CAN5__ : 59|4@0+ (1,0) [0|15] "" Vector__XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
*/

#define OBJ_REL_DIST_RES	0.00775
#define OBJ_REL_DIST_OFFSET	4095.9375
#define OBJ_REL_POS_OFFSET	4095
#define OBJ_REL_VEL_OFFSET	8191
#define OBJ_REL_VEL_RES		0.056
#define KPH_TO_MPS		(1.0/3.6)
typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float object_relative_position_CAN5_M;
	float object_relative_distance_CAN5_M;
	float object_relative_velocity_CAN5_MPS;
	unsigned char data_valid;
	unsigned char object_counter_CAN5;
} accord_target_object_t;

static inline void get_accord_target_object(unsigned char *data, accord_target_object_t *p) {
	short short_temp;
	float float_temp;

	if( (data[2] & 0x10) != 0)
		data[2] |= 0xE0;

	if( (data[4] & 0x20) != 0)
		data[4] |= 0xC0;
	//Calculate relative distance
	short_temp = (short)((data[0] << 8) + data[1]);
	float_temp = short_temp * 0.00775;
	if(float_temp > 250.0)
		p->data_valid = 0;
	else
		p->data_valid = 1;
	p->object_relative_distance_CAN5_M = float_temp;

	short_temp = (short)((data[2] << 8) + data[3]);
	float_temp = short_temp / 1.0;
	p->object_relative_position_CAN5_M = float_temp;

	//Calculate relative speed
	short_temp = (short)((data[4] << 8) + data[5]);
	float_temp = short_temp * 0.056;
	p->object_relative_velocity_CAN5_MPS = float_temp * KPH_TO_MPS;

	p->object_counter_CAN5 = (data[7] >> 4) & 0x03;
}


/*
** printcan.c - prints 8-byte CAN message to stdout
*/
int printcan(db_steinhoff_msg_t *steinhoff_msg);

int print_vehicle_speed_accord(accord_vehicle_speed_t *accord_vehicle_speed);

void check_msg_timeout(int curr_ts_ms, int *prev_ts_ms,
        unsigned char *two_message_periods,
        unsigned int *message_timeout_counter);
