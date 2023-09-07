#pragma once

#include <can_dbvars.h>

/*******************************************************************************
 *      leaf_accel_cmd
 *      Message ID      0x99
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
} leaf_accel_cmd_t;

#define acceleration_res  0.001

static inline void set_leaf_accel_cmd(unsigned char data[], leaf_accel_cmd_t *p) {
	short accel_cmd_short;
	//  % acceleration = raw_value * 0.46698 - 3.74 (From documentation)
	accel_cmd_short =  (short)(p->accel_cmd / acceleration_res);
	data[0] = accel_cmd_short & 0xff;
	data[1] = (accel_cmd_short & 0xff00) >> 8;
	printf("set_leaf_accel_cmd: accel_cmd %.2f accel_cmd_short %#hx\n", p->accel_cmd, accel_cmd_short);
}

/*******************************************************************************
 *      leaf_torqe_cmd
 *      Message ID      0x98
 *      Transmitted every 20 ms
 *
 *	torque_override_cmd
 *      Byte Position   2
 *      Bit Position    16
 *      Bit Length	8 
 *
 */

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float torque_cmd;
} leaf_torque_cmd_t;

static inline void set_leaf_torque_cmd(unsigned char data[], leaf_torque_cmd_t *p) {
	unsigned short torque_cmd_short;

	torque_cmd_short =  (short)(p->torque_cmd);
	data[0] = torque_cmd_short & 0xff;
	data[1] = (torque_cmd_short & 0xff00) >> 8;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	printf("set_leaf_torque_cmd: torque_cmd %.2f torque_cmd_short %#hx\n", p->torque_cmd, torque_cmd_short);
}


/*******************************************************************************
 *      Vehicle_Speed_CAN6_MPH
 *      Message ID      0x355 (853)
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_LEAF_MSG853_VAR
 *
 *	Vehicle_Speed_CAN6_MPH
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *
BO_ 853 Veh_Spd_CAN6: 4 Vector__XXX
 SG_ veh_speed1_CAN6__kph : 7|16@0+ (0.01,0) [0|655.35] "kph" Vector__XXX
 SG_ veh_speed2_CAN6__kph : 23|16@0+ (0.01,0) [0|655.35] "kph" Vector__XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
 */

#define VEHICLE_SPEED_RES	0.01
#define KPH_2_MPS		3.6 

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float veh_speed1_CAN6__mps;
	float veh_speed2_CAN6__mps;
} leaf_vehicle_speed_t;

static inline void get_leaf_vehicle_speed(unsigned char *data, leaf_vehicle_speed_t *p) {
	p->veh_speed1_CAN6__mps = (float)(((data[0] << 8) + data[1]) * VEHICLE_SPEED_RES * KPH_2_MPS);
	p->veh_speed2_CAN6__mps = (float)(((data[2] << 8) + data[3]) * VEHICLE_SPEED_RES * KPH_2_MPS);
	printf("library: vehicle speed 1 %.3f vehicle speed 2 %.3f\n",
		p->veh_speed1_CAN6__mps,
		p->veh_speed2_CAN6__mps
	);
}

/*******************************************************************************
 *      Torque_CAN2_NM
 *      Message ID      0x1DA (474)
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_LEAF_MSG530_VAR
 *
 *	Torque_CAN5_NM
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *
 *   BO_ 474 Motor_Speed_TRQ_VOLT: 6 Vector__XXX
 SG_ motor_inverter_input_voltage_CAN : 7|8@0+ (2,0) [0|510] "V" Vector__XXX
 SG_ Motor_Torque_CAN5__Nm : 18|11@0- (0.5,0) [-512|511.5] "Nm" Vector__XXX
 SG_ Motor_Speed_CAN5__rpm : 39|15@0- (1,0) [-16384|16383] "rpm" Vector__XXX
 *
 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56 *
 */

#define VOLTAGE_RES		2.0
#define TORQUE_RES		0.500

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float motor_inverter_input_voltage_CAN5;
	float Motor_Torque_CAN5__Nm;
	float Motor_Speed_CAN5__rpm;
} leaf_torque_t;

static inline void get_leaf_torque(unsigned char *data, leaf_torque_t *p) {
	short short_temp;

	p->motor_inverter_input_voltage_CAN5 = (float)(data[0] * VOLTAGE_RES);

	short_temp = (short)( ((data[2] << 8) & 0x700) + data[3]);
	p->Motor_Torque_CAN5__Nm = (float)(((data[2] << 8) + data[3]) * TORQUE_RES);
}

/*******************************************************************************
 *	Steering wheel angle & rate
 *      Message ID     342 
 *      Transmitted every ?? ms
 *
 *	dbvar = DB_LEAF_MSG342_VAR
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
} leaf_steering_t;

static inline void get_leaf_steering(unsigned char *data, leaf_steering_t *p) {
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
 *	dbvar = DB_LEAF_MSG804_VAR
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
} leaf_fuel_rate_t;

static inline void get_leaf_fuel_rate(unsigned char *data, leaf_fuel_rate_t *p) {
	short temp;

	temp = ((data[2] & 0xFF) << 8) + (data[3] & 0xFF) ;
	p->fuel_integrated_flow = (float)temp;
}


/*******************************************************************************
 *	PRNDL_Pos_CAN2 
 *      Message ID      0x392
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_LEAF_MSG392_VAR
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
} leaf_PRNDL_Pos_t;

static inline void get_leaf_PRNDL_Pos(unsigned char *data, leaf_PRNDL_Pos_t *p) {
	p->prndl = data[5] & 0x0F;
}



/*******************************************************************************
 *	leaf_target_object
 *      Message ID      0x107
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_LEAF_MSG107_VAR
 *
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *


BO_ 263 HS_CAN5__107: 8 Vector__XXX
 SG_object_distance_Radar: 7|16@0+ (0.1,0) [0|4095.9375] "m" Vector__XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
*/

#define OBJ_REL_DIST_RES	0.1

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float object_distance_Radar;
} leaf_target_object_t;

static inline void get_leaf_target_object(unsigned char *data, leaf_target_object_t *p) {
	short short_temp;
	float float_temp;

	//Calculate relative distance
	short_temp = (short)((data[0] << 8) + data[1]);
	float_temp = short_temp * OBJ_REL_DIST_RES;
	p->object_distance_Radar = float_temp;
	printf("library: target distance %.3f\n",
		p->object_distance_Radar
	);
}

/*******************************************************************************
 *	leaf_target_speed_mps
 *      Message ID      0x108
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_LEAF_MSG108_VAR
 *
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *


BO_ 263 HS_CAN5__108: 8 Vector__XXX
 SG_object_relative_spd_Radar: 7|16@0+ (0.1,0) [0|4095.9375] "m" Vector__XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
*/

#define OBJ_REL_SPEED_RES	0.1

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float object_relative_spd_Radar__mps;
} leaf_target_speed_mps_t;

static inline void get_leaf_target_speed(unsigned char *data, leaf_target_speed_mps_t *p) {
	short short_temp;
	float float_temp;

	//Calculate relative distance
	short_temp = (short)((data[0] << 8) + data[1]);
	float_temp = short_temp * OBJ_REL_DIST_RES;
	p->object_relative_spd_Radar__mps = float_temp;
	printf("library: target relative speed %.3f\n",
		p->object_relative_spd_Radar__mps
	);
}

/*******************************************************************************
 *	Veh_Accel_CAN4
 *      Message ID      0x292 (658)
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_LEAF_MSG392_VAR
 *
 *      Byte Position   0-1
 *      Bit Position    7
 *      Bit Length      12
 *
 *      BO_ 658 Veh_Accel_CAN4: 7 Vector__XXX^M
 *	SG_ Veh_brake_press_CAN4__bar : 55|8@0+ (1,0) [0|255] "" Vector__XXX^M
 *	SG_ Yaw_Rate_CAN4__degps : 30|11@0- (0.1,0) [-102.4|102.3] "deg/s" Vector__XXX^M
 *	SG_ Long_Accel_CAN4__G : 7|12@0+ (0.001,-2) [-2|2.095] "G" Vector__XXX^M
 *	SG_ Lateral_Accel_CAN4__mps2 : 11|12@0+ (0.01,-20) [-20|20.95] "m/s^2" Vector__XXX^M
 *
 */

#define LONG_ACCEL_RES 0.001
#define LONG_ACCEL_OFF -2.0
#define LAT_ACCEL_RES 0.01
#define LAT_ACCEL_OFF -20.0
#define YAW_RATE_RES 0.1
#define YAW_RATE_OFF 0
#define G2MPS	9.80665

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float Long_Accel_CAN4__G;
	float Lateral_Accel_CAN4__mps2;
	float Yaw_Rate_CAN4__degps;
	unsigned char Veh_brake_press_CAN4__bar;
} leaf_Veh_Accel_CAN4_t;

static inline void get_leaf_accel(unsigned char *data, leaf_Veh_Accel_CAN4_t *p) {
	short temp;

	temp = (short)(((data[0] << 4) & 0xFF0) + ((data[1] >> 0x04) & 0x0F));
	if((temp & 0x800) != 0)
		temp |= 0xF000;
	p->Long_Accel_CAN4__G = (float)((temp * LONG_ACCEL_RES) + LONG_ACCEL_OFF);

	temp = (short)(((data[1] << 4) & 0xFF0) + data[2]);
	if((temp & 0x800) != 0)
		temp |= 0xF000;
	p->Lateral_Accel_CAN4__mps2 = (float)((temp * LAT_ACCEL_RES) + LAT_ACCEL_OFF);

	temp = (short)(((data[3] << 4) & 0xFF0) + ((data[4] >> 0x04) & 0x0F));
	if((temp & 0x400) != 0)
		temp |= 0xF800;
	p->Yaw_Rate_CAN4__degps = (float)((temp * YAW_RATE_RES) + YAW_RATE_OFF);

	p->Veh_brake_press_CAN4__bar = (unsigned char)data[6];

	printf("library: Long_Accel_CAN4__G %.3f Lateral_Accel_CAN4__mps2 %.3f Yaw_Rate_CAN4__degps %.3f Veh_brake_press_CAN4__bar %d %#hhx %#hhx %#hhx %#hhx %#hhx %#hhx %#hhx \n",
			p->Long_Accel_CAN4__G,
			p->Lateral_Accel_CAN4__mps2,
			p->Yaw_Rate_CAN4__degps,
			p->Veh_brake_press_CAN4__bar,
			data[0],
			data[1],
			data[2],
			data[3],
			data[4],
			data[5],
			data[6]
	);
}


/*
** printcan.c - prints 8-byte CAN message to stdout
*/
int printcan(db_steinhoff_msg_t *steinhoff_msg);

int print_vehicle_speed(leaf_vehicle_speed_t *leaf_vehicle_speed);

void check_msg_timeout(int curr_ts_ms, int *prev_ts_ms,
        unsigned char *two_message_periods,
        unsigned int *message_timeout_counter);
