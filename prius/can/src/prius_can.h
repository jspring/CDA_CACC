#pragma once

#include <can_dbvars.h>

#define MASK_b0	 0x01
#define MASK_b01 0x03
#define MASK_b02 0x07
#define MASK_b03 0x0F
#define MASK_b04 0x1F
#define MASK_b07 0xFF
#define MASK_b1	 0x02
#define MASK_b12 0x06
#define MASK_b2	 0x04
#define MASK_b23 0x0C
#define MASK_b3	 0x08
#define MASK_b34 0x18_
#define MASK_b4	 0x10
#define MASK_b45 0x30
#define MASK_b47 0xF0
#define MASK_b5	 0x20
#define MASK_b6	 0x40
#define MASK_b67 0xC0
#define MASK_b7	 0x80

/*******************************************************************************
 *      prius_accel_cmd
 *      Message ID      0x99
 *      Transmitted every 20 ms
 *
 *	accel_cmd
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *
 */
#define ACCEL_RES  0.001

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float accel_cmd;
} prius_accel_cmd_t;

static inline void set_prius_accel_cmd(unsigned char data[], prius_accel_cmd_t *p) {
	short accel_cmd_short;

	accel_cmd_short =  (short)(p->accel_cmd / ACCEL_RES);
	data[0] = (accel_cmd_short & 0xFF00) >> 8;
	data[1] = accel_cmd_short & 0xFF;
}

/*******************************************************************************
 *      prius_wheel_speed
 *      Message ID      0xAA
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_PRIUS_MSGAA_VAR
 *
 *	veh_wheel_spd_FR_CAN1_kph
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      16
 *
 *	veh_wheel_spd_FL_CAN1_kph
 *      Byte Position   2-3
 *      Bit Position    0
 *      Bit Length      16
 *
 *	veh_wheel_spd_FR_CAN1_kph
 *      Byte Position   4-5
 *      Bit Position    0
 *      Bit Length      16
 *
 *	veh_wheel_spd_RL_CAN1_kph
 *      Byte Position   6-7
 *      Bit Position    0
 *      Bit Length      16
 *
 */

#define WHEEL_SPEED_RES		0.01
#define WHEEL_SPEED_OFFSET	6767
#define KPH_TO_MPS		1.0/3.6

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float veh_wheel_spd_FR_CAN1_mps;
	float veh_wheel_spd_FL_CAN1_mps;
	float veh_wheel_spd_RR_CAN1_mps;
	float veh_wheel_spd_RL_CAN1_mps;
	float veh_wheel_spd_average;
	float instantaneous_acceleration;
} prius_wheel_speed_t;

static inline void get_prius_wheel_speed(unsigned char *data, prius_wheel_speed_t *p) {
	static float previous_wheel_speed;
	struct timespec ts;
	static struct timespec ts_sav;

	p->veh_wheel_spd_FR_CAN1_mps = (float)((short)(((data[0] << 8) + data[1]) - WHEEL_SPEED_OFFSET) * WHEEL_SPEED_RES * KPH_TO_MPS);
	p->veh_wheel_spd_FL_CAN1_mps = (float)((short)(((data[2] << 8) + data[3]) - WHEEL_SPEED_OFFSET) * WHEEL_SPEED_RES * KPH_TO_MPS);
	p->veh_wheel_spd_RR_CAN1_mps = (float)((short)(((data[4] << 8) + data[5]) - WHEEL_SPEED_OFFSET) * WHEEL_SPEED_RES * KPH_TO_MPS);
	p->veh_wheel_spd_RL_CAN1_mps = (float)((short)(((data[6] << 8) + data[7]) - WHEEL_SPEED_OFFSET) * WHEEL_SPEED_RES * KPH_TO_MPS);
//	p->veh_wheel_spd_average = ((p->veh_wheel_spd_FR_CAN1_mps + p->veh_wheel_spd_FL_CAN1_mps + p->veh_wheel_spd_RR_CAN1_mps + p->veh_wheel_spd_RL_CAN1_mps) / 4.0) -0.094;
	p->veh_wheel_spd_average = p->veh_wheel_spd_FR_CAN1_mps;
//	clock_gettime(CLOCK_REALTIME, &ts);
//	p->instantaneous_acceleration = (p->veh_wheel_spd_FR_CAN1_mps = previous_wheel_speed)/(ts.tv_nsec - ts_sav.tv_nsec)/1e9;
//	ts_sav = ts;


	printf("library wheel speed: p->veh_wheel_spd_FR_CAN1_mps %.2f data[0] %#hhx data[1] %#hhx inst accel %.4f \n",
			p->veh_wheel_spd_FR_CAN1_mps,
			data[0],
			data[1],
			p->instantaneous_acceleration
			);
}

/*******************************************************************************
 *      prius_vehicle_speed
 *      Message ID      0xB4
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_PRIUS_MSGB4_VAR
 *
 *	veh_spd_CAN1_kph
 *      Byte Position   6-7
 *      Bit Position    0
 *      Bit Length      16
 *
 */

#define VEH_SPEED_RES		0.01

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float veh_spd_CAN1_kph;
} prius_vehicle_speed_t;

static inline void get_prius_vehicle_speed(unsigned char *data, prius_vehicle_speed_t *p) { 
	p->veh_spd_CAN1_kph = (float)((short)(((data[6] << 8) + data[7]) * VEH_SPEED_RES));
}


/*******************************************************************************
 *      prius_steering_angle
 *      Message ID      37
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_PRIUS_MSG228_VAR
 *
 *	Veh_steer_angle_CAN1__deg
 *      Byte Position   0-1
 *      Bit Position    3-8 (Motorola ordering, see below)
 *      Bit Length      12
 *
 *	Veh_steer_angle_fraction_CAN1__deg 
 *      Byte Position   4
 *      Bit Position    39-36
 *      Bit Length      4
 *
 *	Veh_steer_rate_CAN1__degps
 *      Byte Position   4-5
 *      Bit Position    35-40
 *      Bit Length      12
 *
 *
 * BO_ 37 HS_CAN5__25: 6 Vector__XXX
 * SG_ Veh_steer_angle_CAN1__deg : 3|12@0- (1.5,0) [-2048|2047] "deg" Vector__XXX
 * SG_ Veh_steer_angle_fraction_CAN1__deg : 39|4@0+ (0.1,0) [0|15] "" Vector__XXX
 * SG_ Veh_steer_rate_CAN1__degps : 35|12@0- (1,0) [0|4095] "" Vector__XXX
 *
 * Motorola ordering
 * 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 * 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
 *
 */

#define STEER_ANGLE_RES 1.5

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float steering_angle_deg;
	float steering_angle_rate_degps;
	float steering_angle_fraction_deg;
} prius_steering_angle_t;

static inline void get_prius_steering_angle(unsigned char *data, prius_steering_angle_t *p) {
	short short_temp;

	short_temp = ((data[0] << 12) + (data[1] << 4)) / 16; //Put the sign bit at the MSb of the short int
	p->steering_angle_deg = (float)(short_temp * STEER_ANGLE_RES);

	short_temp = ((data[4] << 12) + (data[5] << 4)) / 16; //Put the sign bit at the MSb of the short int
	p->steering_angle_rate_degps = (float)short_temp;

}


/*******************************************************************************
 *      prius_long_lat_accel
 *      Message ID      0x228
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_PRIUS_MSG228_VAR
 *
 *	long_accel
 *      Byte Position   1-2
 *      Bit Position    8
 *      Bit Length      15
 *
 *	lat_accel 
 *      Byte Position   3-4
 *      Bit Position    24
 *      Bit Length      15
 *
 *BO_ 552 Bus_1__228: 4 Vector__XXX
 SG_ Veh_long_accel_CAN1__mps2 : 6|15@0- (0.001,0) [-16.384|16.383] "mps2" Vector__XXX
 SG_ Veh_lat_accel_CAN1__mps2 : 22|15@0- (0.001,0) [-16.384|16.383] "mps2" Vector__XXX
 *
 *
 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
 *
 */

#define LONG_LAT_ACCEL_RES	0.001
#define LONG_LAT_ACCEL_OFFSET	16.384

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float long_accel;
	float lat_accel;
} prius_long_lat_accel_t;

static inline void get_prius_long_lat_accel(unsigned char *data, prius_long_lat_accel_t *p) {
	short short_temp;

	if( (data[0] & 0x40) != 0)
		data[0] |= 0xC0;
	short_temp = (data[0] << 8) + data[1];
	p->long_accel = (float)(short_temp * LONG_LAT_ACCEL_RES);

	if( (data[2] & 0x40) != 0)
		data[2] |= 0xC0;
	short_temp = (data[2] << 8) + data[3];
	p->lat_accel = (float)(short_temp * LONG_LAT_ACCEL_RES);
/*
	printf("library: long accel: data[0] %#hhx data[1] %#hhx %.2f lat accel: data[2] %#hhx data[3] %#hhx %.2f\n",
			data[0],
			data[1],
			p->long_accel,
			data[2],
			data[3],
			p->lat_accel
			);
*/
}

/*******************************************************************************
 *      prius_accel_cmd_status
 *      Message ID      0x343
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_PRIUS_MSG343_VAR
 *
 *	message_counter
 *      Byte Position   0
 *      Bit Position    0
 *      Bit Length      8
 *
 *	accel_cmd
 *      Byte Position   1-2
 *      Bit Position    8
 *      Bit Length      16
 *
 *	brake_nc_sw
 *      Byte Position   4
 *      Bit Position    6
 *      Bit Length      1
 *
 *	brake_no_sw
 *      Byte Position   4
 *      Bit Position    5
 *      Bit Length      1
 *
 *
 */

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float accel_cmd_status;
	unsigned char set_me_x3;
	unsigned char distance;
	unsigned char mini_car;
	unsigned char set_me_x01;
	unsigned char cancel_req;
	unsigned char set_me_1;
	unsigned char release_standstill;
	unsigned char checksum;
	unsigned char checksum_check;
} prius_accel_cmd_status_t;

static inline void get_prius_accel_cmd_status(unsigned char *data, prius_accel_cmd_status_t *p) {
	p->accel_cmd_status = (float)((short)((data[0] << 8) + data[1]) * ACCEL_RES);
	p->set_me_x3 = (data[2] & MASK_b03);
	p->distance = (data[2] & MASK_b4) >> 4;
	p->mini_car = (data[2] & MASK_b5) >> 5;
	p->set_me_x01 = (data[2] & MASK_b67) >> 6;
	p->cancel_req = (data[3] & MASK_b0) >> 0;
	p->set_me_1 = (data[3] & MASK_b6) >> 6;
	p->release_standstill = (data[3] & MASK_b7) >> 7;
	p->checksum = data[7];
	p->checksum_check  = (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + 0x4e) & 0xff;
}

/*******************************************************************************
 *      prius_cruise_state
 *      Message ID      0x399
 *      Transmitted every 40 ms
 *
 *	dbvar = DB_PRIUS_MSG399_VAR
 *
 *	cruise_main_on_CAN1
 *      Byte Position   0
 *      Bit Position    4
 *      Bit Length      1
 *
 *	cruise_control_state_CAN1
 *	Byte Position   1
 *      Bit Position    0-4
 *      Bit Length      4
 *
 *	cruise_dash_set_speed_CAN1
 *	Byte Position   3
 *      Bit Position    0-7
 *      Bit Length      8
 *
 */

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	unsigned char	cruise_main_on_CAN1;
	unsigned char	cruise_control_state_CAN1;
	unsigned char	cruise_dash_set_speed_CAN1;
} prius_cruise_control_state_t;

static inline void get_prius_cruise_control_state(unsigned char *data, prius_cruise_control_state_t *p) {
	p->cruise_main_on_CAN1 = (data[0] >> 4) & 0x01;
	p->cruise_control_state_CAN1 = data[1] & 0x0F;
	p->cruise_dash_set_speed_CAN1 = data[3] & 0xFF;
}


/*******************************************************************************
 *      prius_radar_forward_vehicle
 *      Message ID      0x680
 *      Transmitted every 20 ms
 *
 *	dbvar = DB_PRIUS_MSG680_VAR
 *
 *
 *
 *BO_ 742 Bus_1_DAS__680: 4 Vector__XXX
 SG_ Radar_forward_veh_relative_spd_CAN1__kph : 23|12@0- (0.09,0) [-204.8|204.7] "kph" Vector__XXX
 SG_ Radar_forward_veh_distance_CAN1__m : 7|15@0+ (0.01,0) [0|327.67] "m" Vector__XXX
 SG_ Radar_forward_veh_relative_spd_CAN1__mps : 23|12@0- (0.025,0) [-100|100] "m/s" HCU
 SG_ Signal_3 : 7|13@0+ (0.05,0) [0|8191] "" Vector__XXX

  7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
 *
 */
#define RADAR_DIST_RES		0.03
#define RADAR_SPEED_MPS_RES	0.06944444444
#define RADAR_SPEED_KPH_RES	0.09

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float Radar_forward_veh_distance_CAN1__m;
	float Radar_forward_veh_relative_spd_CAN1__kph;
	float Radar_forward_veh_relative_spd_CAN1__mps;
}prius_radar_forward_vehicle_t;

static inline void get_prius_radar_forward_vehicle(unsigned char *data, prius_radar_forward_vehicle_t *p) {
	short short_temp;
	short short_temp1;
	unsigned char char0 = data[0];
	unsigned char char1 = data[1];
	unsigned char char2 = data[2];
	unsigned char char3 = data[3];
	unsigned char char4 = data[4];
	unsigned char char5 = data[5];

	short_temp = (short)(((char0 >> 7) & 1) + ((char1 << 1) & 0x1FE) + ((char2 << 9 ) & 0XE00));
	p->Radar_forward_veh_distance_CAN1__m = short_temp * RADAR_DIST_RES;

//	short_temp1 = (short)(((char2 << 6) & 0x0300) + (char3)); //dividing by 16 shifts the short int to the right by 4
	short_temp1 = (short)(((char3 >> 7) & 1) + ((char4 << 1) & 0x1FE) + ((char5 << 9 ) & 0x200));

	if( (short_temp1 & 0x0200) != 0)
		short_temp1 |= 0xC000;							//bits. It should also drag the msb sign bit to the right.
	p->Radar_forward_veh_relative_spd_CAN1__mps  = short_temp1 * RADAR_SPEED_MPS_RES;

//	printf("library 0X680 targets: dist %.2f d[0] %#hhx d[1] %#hhx d[2] %#hhx short_temp %04hx speed %.2f d[2] %#hhx d[3] %#hhx d[4] %#hhx d[5] %#hhx short_temp1 %04hx\n",
//	p->Radar_forward_veh_distance_CAN1__m,
//	data[0],
//	data[1],
//	data[2],
//	short_temp,
//	p->Radar_forward_veh_relative_spd_CAN1__mps,
//	data[3],
//	data[4],
//	data[5],
//	short_temp1
//	);

}
/*******************************************************************************
 *      prius_fuel_rate
 *      Message ID 0x7E8
********************************************************************************/
typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	char size;
	char mode;
	char id;
	float fuel_rate;
}prius_fuel_rate_t;

static inline void get_prius_fuel_rate(unsigned char *data, prius_fuel_rate_t *p) {

	p->size = data[0];
	p->mode = data[1];
	p->id = data[2];
	p->fuel_rate = ((data[3] * 256) + data[4]) / 100.0/14.7;
//printf("library: prius fuel rate %.3f g/sec %#hhx %#hhx size %#hhx mode %#hhx id %#hhx\n",
//		p->fuel_rate,
//		data[3],
///		data[4],
//		data[0],
//		data[1],
//		data[2]
//);
}


/*
** printcan.c - prints 8-byte CAN message to stdout

int printcan(db_komodo_t *db_kom);
*/
/* TODO
** printmsg - prints parsed contents of PRIUS CAN message to stdout
int printmsg(db_komodo_t *db_kom);
*/

void check_msg_timeout(int curr_ts_ms, int *prev_ts_ms,
        unsigned char *two_message_periods,
        unsigned int *message_timeout_counter);

extern int print_wheel_speed(prius_wheel_speed_t *prius_wheel_speed);
//extern int print_vehicle_speed(prius_vehicle_speed_t *prius_vehicle_speed);
extern int print_long_lat_accel(prius_long_lat_accel_t *prius_long_lat_accel);
extern int print_accel_cmd_status(prius_accel_cmd_status_t *prius_accel_cmd_status);
extern int print_cruise_control_state(prius_cruise_control_state_t *prius_cruise_control_state);

