#pragma once

#include <can_dbvars.h>

/*******************************************************************************
 *      taurus_accel_cmd
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
} taurus_accel_cmd_t;

#define acceleration_res  0.424444
#define acceleration_offset -1.0 

static inline void set_taurus_accel_cmd(unsigned char data[], taurus_accel_cmd_t *p) {
	char accel_cmd_char;
	//  % acceleration = raw_value * 0.424444 - 1 (From documentation)
	accel_cmd_char =  (char)( (p->accel_cmd - acceleration_offset) / acceleration_res);
	data[0] = accel_cmd_char & 0xff;
//	printf("set_taurus_accel_cmd: accel_cmd %.2f accel_cmd_char %hhd\n", p->accel_cmd, accel_cmd_char);
}

/*******************************************************************************
 *      taurus_brake_cmd
 *      Message ID      0x99
 *      Transmitted every 20 ms
 *
 *	brake_override_cmd
 *      Byte Position   2
 *      Bit Position    16
 *      Bit Length	8 
 *
 */

#define deceleration_res  0.001

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float brake_cmd;
} taurus_brake_cmd_t;

static inline void set_taurus_brake_cmd(unsigned char data[], taurus_brake_cmd_t *p) {
	short brake_cmd_short;
	//  % deceleration = raw_value * 0.001 (From documentation)

	if(p->brake_cmd > 0)	  //This same command can be used for acceleration, so make sure
		p->brake_cmd = 0; // braking (negative value) is being called for

	brake_cmd_short =  (short)( p->brake_cmd / deceleration_res);
	data[0] = (brake_cmd_short & 0xFF00) >> 8;
	data[1] = brake_cmd_short & 0xFF;
//	printf("set_taurus_brake_cmd: brake_cmd %.2f brake_cmd_char %u temp %.2f\n", p->brake_cmd, brake_cmd_char, temp);
}


/*******************************************************************************
 *      wheel_speed
 *      Message ID      0x215
 *      Transmitted every 42 ms
 *
 *	dbvar = DB_TAURUS_MSG215_VAR
 *
 *	wheel_speed_FL
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      15
 *
 *	wheel_speed_FR
 *      Byte Position   2-3
 *      Bit Position    0
 *      Bit Length      15
 *
 *	wheel_speed_RL
 *      Byte Position   4-5
 *      Bit Position    0
 *      Bit Length      15
 *
 *	wheel_speed_RR
 *      Byte Position   6-7
 *      Bit Position    0
 *      Bit Length      15
 *
 */

/*
BO_ 533 WheelSpeed_CG1: 8 XXX
 SG_ WhlFl_W_Meas : 7|15@0+ (0.01,0) [0|0] "rad/s" XXX
 SG_ WhlFr_W_Meas : 23|15@0+ (0.01,0) [0|0] "rad/s" XXX
 SG_ WhlRl_W_Meas : 39|15@0+ (0.01,0) [0|0] "rad/s" XXX
 SG_ WhlRr_W_Meas : 55|15@0+ (0.01,0) [0|0] "rad/s" XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
*/
#define WHEEL_SPEED_RAD_PER_SEC	0.01
#define TIRE_CIRCUMFERENCE_M	3.00 //Estimate of 3 meters 1/20/2020 JAS
#define TIRE_RADIUS_M		0.500 //Estimate of 0.5 meter radius 1/20/2020 JAS
#define MAGIC_NUMBER	(5.54/2) //Divisor of rad/sec to grt m/sec

// 2 * PI rad/s <=> 1 circumference/s = 2 * PI * radius /s

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float wheel_speed_FL;
	float wheel_speed_FR;
	float wheel_speed_RL;
	float wheel_speed_RR;
	float average_speed;
} taurus_wheel_speed_t;

static inline void get_taurus_wheel_speed(unsigned char *data, taurus_wheel_speed_t *p) {
	p->wheel_speed_FL = (float)((((data[0] << 8) + data[1]) >>1) * WHEEL_SPEED_RAD_PER_SEC / MAGIC_NUMBER);
	p->wheel_speed_FR = (float)((((data[2] << 8) + data[3]) >>1) * WHEEL_SPEED_RAD_PER_SEC / MAGIC_NUMBER);
	p->wheel_speed_RL = (float)((((data[4] << 8) + data[5]) >>1) * WHEEL_SPEED_RAD_PER_SEC / MAGIC_NUMBER);
	p->wheel_speed_RR = (float)((((data[6] << 8) + data[7]) >>1) * WHEEL_SPEED_RAD_PER_SEC / MAGIC_NUMBER);
	p->average_speed = ((p->wheel_speed_FL + p->wheel_speed_FR + p->wheel_speed_RL + p->wheel_speed_RR ) / 4.0) - 0.094;
	//	printf("get_taurus_wheel_speed: data[0] %#hhx data[1] %#hhx \n", data[0], data[1]);
}


/*******************************************************************************
 *      taurus_acc_389
 *      Message ID      389
 *      Transmitted every 20 ms
 *
 *	dbvar = DB_TAURUS_MSG389_VAR
 *
 *	AccVeh_V_Trg (target velocity)
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      9
 *
 *	longitudinal_acceleration
 *      Byte Position   2-3
 *      Bit Position    0
 *      Bit Length      13
 *
 *	vertical_acceleration
 *      Byte Position   4-5
 *      Bit Position    0
 *      Bit Length      13

Message 389 has AccPrpl_A_Pred, AccPrpl_A_Rq, and AccBrkTot_A_Rq
-          AccPrpl_A_Rq is the desired acceleration [m/s^2] from the ACC module and is one of the signals that should be overridden when accelerating
-          AccBrkTot_A_Rq is the desired deceleration [m/s^2] from the ACC module and is one of the signals that should be overridden when braking
-          AccPrpl_A_Pred is a signal that is not overrided and should be gatewayed by the panda
-          All the signals from message 389 which are overridden by the Panda include:
o   AccDeny_B_Rq (set to 0),
o   AccCancl_B_Rq (set to 0),
o   AccPrpl_A_Rq (depends on desired accel),
o   AccBrkTot_A_Rq (depends on desired accel),
o   AccBrkPrchg_B_Rq (0 or 1 depending on desired accel),
o   AccBrkDecel_B_Rq (0 or 1 depending on desired accel),

BO_ 389 ACCDATA_CG1: 8 XXX
 SG_ AccVeh_V_Trg : 7|9@0+ (0.5,0) [0|0] "kph" XXX
 SG_ AccPrpl_A_Pred : 14|10@0+ (0.01,-5.0) [0|0] "m/s^2" XXX
 SG_ AccBrkTot_A_Rq : 39|13@0+ (0.0039,-20.0) [0|0] "m/s^2" XXX
 SG_ CmbbDeny_B_Actl : 40|1@0+ (1,0) [0|0] "" XXX
 SG_ AccBrkPrkEl_B_Rq : 41|1@0+ (1,0) [0|0] "" XXX
 SG_ AccCancl_B_Rq : 42|1@0+ (1,0) [0|0] "" XXX
 SG_ AccPrpl_A_Rq : 55|10@0+ (0.01,-5.0) [0|0] "m/s^2" XXX
 SG_ Cmbb_B_Enbl : 56|1@0+ (1,0) [0|0] "" XXX
 SG_ CmbbOvrrd_B_RqDrv : 57|1@0+ (1,0) [0|0] "" XXX
 SG_ CmbbEngTqMn_B_Rq : 58|1@0+ (1,0) [0|0] "" XXX
 SG_ AccDeny_B_Rq : 59|1@0+ (1,0) [0|0] "" XXX
 SG_ AccBrkPrchg_B_Rq : 60|1@0+ (1,0) [0|0] "" XXX
 SG_ AccBrkDecel_B_Rq : 61|1@0+ (1,0) [0|0] "" XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
*/

#define ACCEL_RES	0.01

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float target_speed_kph;
	float AccPrpl_A_Rq_desired_accel; //Desired acceleration, m/s^2
	float AccPrpl_A_Pred; //Predicted acceleration(?), m/s^2.
	float AccBrkTot_A_Rq_desired_decel; //Desired deceleration, m/s^2

 	unsigned char CmbbDeny_B_Actl;	 	// 40|1@0+ (1,0) [0|0] "" XXX
	unsigned char AccBrkPrkEl_B_Rq; 	// 41|1@0+ (1,0) [0|0] "" XXX
	unsigned char AccCancl_B_Rq; 		// 42, (1 bit, set to 0),
	unsigned char AccPrpl_A_Rq;		// 55|10@0+ (0.01,-5.0) [0|0] "m/s^2" XXX
	unsigned char Cmbb_B_Enbl;		// 56|1@0+ (1,0) [0|0] "" XXX
	unsigned char CmbbOvrrd_B_RqDrv;	// 57|1@0+ (1,0) [0|0] "" XXX
	unsigned char CmbbEngTqMn_B_Rq; 	// 58|1@0+ (1,0) [0|0] "" XXX
	unsigned char AccDeny_B_Rq; 		// 59, (1 bit, set to 0)
	unsigned char AccBrkPrchg_B_Rq; 	// 60, (1 bit, 0 or 1 depending on desired accel),
	unsigned char AccBrkDecel_B_Rq; 	// 61
	unsigned char Acc_Resume_Active; 	// (1 bit, set to 0),
	unsigned char Acc_Autobrake_Cancel;	// (1 bit, set to 0)
} taurus_acc_389_t;

static inline void get_taurus_acc_389(unsigned char *data, taurus_acc_389_t *p) {
	short short_temp;

	short_temp = (data[0] << 1) + (data[1] >> 7);
	p->target_speed_kph = short_temp * 0.5; 	//bit 7

	short_temp = (data[1] << 3) + (data[2] >> 5);
	p->AccPrpl_A_Pred = short_temp * 0.01; 		//bit 14

	short_temp = (data[4] << 5) + (data[5] >> 3);
	p->AccBrkTot_A_Rq_desired_decel = short_temp * 0.0039; //bit 39

 	p-> CmbbDeny_B_Actl = data[5] & 1; 		//bit 40
 	p-> AccBrkPrkEl_B_Rq = (data[5] & 2) >> 1;	//bit 41
 	p-> AccCancl_B_Rq  = (data[5] & 4) >> 2;	//bit 43

	short_temp = (data[6] << 2) + (data[7] >> 6);
	p->AccPrpl_A_Rq_desired_accel = short_temp * 0.01; // bits 48-62

 	p-> Cmbb_B_Enbl = data[7] & 0x1;		//bit 56
 	p-> CmbbOvrrd_B_RqDrv = (data[7] & 2) >> 1;	//bit 57
 	p-> CmbbEngTqMn_B_Rq = (data[7] & 4) >> 2;	//bit 58
 	p-> AccDeny_B_Rq = (data[7] & 8) >> 3;		//bit 59
 	p-> AccBrkPrchg_B_Rq = (data[7] & 0x10) >> 4;	//bit 60
 	p-> AccBrkDecel_B_Rq = (data[7] & 0x20 ) >> 5;
}



static inline void set_taurus_acc_389(unsigned char *data, float deceleration, float vehicle_speed_kph) {
/*
o   AccDeny_B_Rq (set to 0),
	unsigned char AccDeny_B_Rq; 		// 59, (1 bit, set to 0)
o   AccCancl_B_Rq (set to 0),
	unsigned char AccCancl_B_Rq; 		// 42, (1 bit, set to 0),
o   AccPrpl_A_Rq (depends on desired accel),
	=0 //never requested; use pedal override instead
o   AccBrkTot_A_Rq (depends on desired accel),
 	SG_ AccBrkTot_A_Rq : 39|13@0+ (0.0039,-20.0) [0|0] "m/s^2" XXX
o   AccBrkPrchg_B_Rq (0 or 1 depending on desired accel),
	unsigned char AccBrkPrchg_B_Rq; 	// 60, (1 bit, 0 or 1 depending on desired accel),
	deceleration <= 0 ? 1 : 0
o   AccBrkDecel_B_Rq (0 or 1 depending on desired accel),
	unsigned char AccBrkDecel_B_Rq; 	// 61, (1 bit, 0 or 1 depending on desired accel),
	deceleration <= 0 ? 1 : 0
*/
	short short_temp;
	float decel_calc;

	decel_calc = -0.003713 * vehicle_speed_kph - 0.032923;

//if accel >= 0        % positive acceleration requested
//OR
//if accel < 0 && accel > decel_calc
//The following conditions must be met:
//
//% negative acceleration requested AND requested acceleration rate is higher (less negative) than coastdown deceleration rate at current vehicle speed
//% use AccPrpl_A_Rq to command negative acceleration which requires some accelerator input to achieve
//% example: very low deceleration rate at 70 mph requires only backing off of the accelerator but not completely taking foot off which will cause a greater deceleration rate
	if(deceleration >= decel_calc) {
		data[7] &= 0xF7; //Clear AccDeny_B_Rq
		data[5] &= 0xFB; //Clear AccCancl_B_Rq
		data[7] &= 0xCF; //Clear AccBrkPrchg_B_Rq and AccBrkDecel_B_Rq

		//Set AccBrkTot_A_Rq to 0
		data[4] = 0;
		data[5] &= 0x03;

		//Set AccPrpl_A_Rq to deceleration
		short_temp = ((short)(deceleration * 100.0)) & 0xFFFF;
		data[6] = (short_temp >> 2) & 0xFF;
		data[7] &= ((short_temp & 0x03) << 6) | 0x3F;
	}

// if accel < 0 && accel < decel_calc % negative acceleration requested AND requested acceleration 
// rate is lower (more negative) than coastdown deceleration rate at current vehicle speed
// brake application is required to slow the car down quicker than coastdown only.
	if(deceleration < decel_calc) {
		data[7] &= 0xF7; //Clear AccDeny_B_Rq
		data[5] &= 0xFB; //Clear AccCancl_B_Rq
		data[7] |= 0x30; //Set AccBrkPrchg_B_Rq and AccBrkDecel_B_Rq

		//Set AccBrkTot_A_Rq to 0
 		// SG_ AccBrkTot_A_Rq : 39|13@0+ (0.0039,-20.0) [0|0] "m/s^2" XXX
		short_temp = ((short)(deceleration / 0.0039)) & 0xFFFF;
		data[4] = (short_temp >> 5) & 0x0F;
		data[5] &= ((short_temp & 0x1F) << 3) | 0x07;

		//Set AccPrpl_A_Rq to 0
		data[6] = 0;
		data[7] &= 0x3F;
	}
}

/*******************************************************************************
 *      taurus_acc_393
 *      Message ID      393
 *      Transmitted every 20 ms
 *
 *	dbvar = DB_TAURUS_MSG393_VAR
 *
 *	AccVeh_V_Trg (target velocity)
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      9
 *
 *	longitudinal_acceleration
 *      Byte Position   2-3
 *      Bit Position    0
 *      Bit Length      13
 *
 *	vertical_acceleration
 *      Byte Position   4-5
 *      Bit Position    0
 *      Bit Length      13

393 has CmbbBrkDecel_A_Rq
-          CmbbBrkDecel_A_Rq is a signal that should be gatewayed by the panda (CMBB stands for collision mitigation by braking) and is related to automatic emergency braking
-          All the signals from message 393 which are overridden by the Panda include:
o   Acc_Autobrake_Cancel (set to 0),
o   Acc_Resume_Active (set to 0),
o   AccPrpl_V_Rq (set to vehicle speed)

BO_ 393 ACCDATA_2_CG1: 8 XXX
 SG_ ACC_AUTOBRAKE_CANCEL : 56|1@0+ (1,0) [0|0] "" XXX
 SG_ ACC_RESUME_ACTIVE : 57|1@0+ (1,0) [0|0] "" XXX
 SG_ FcwAudioWarn_B_Rq : 58|1@0+ (1,0) [0|0] "" XXX
 SG_ CadsAudioMute_D_Rq : 61|2@0+ (1,0) [0|0] "" XXX
 SG_ AccWarn_D_Dsply : 63|2@0+ (1,0) [0|0] "" XXX
 SG_ HudDsplyIntns_No_Actl : 55|8@0+ (0.5,0) [0|0] "%" XXX
 SG_ FcwVisblWarn_B_Rq : 40|1@0+ (1,0) [0|0] "" XXX
 SG_ HudBlk3_B_Rq : 41|1@0+ (1,0) [0|0] "" XXX
 SG_ HudBlk2_B_Rq : 43|1@0+ (1,0) [0|0] "" XXX
 SG_ HudBlk1_B_Rq : 42|1@0+ (1,0) [0|0] "" XXX
 SG_ HudFlashRate_D_Actl : 45|2@0+ (1,0) [0|0] "" XXX
 SG_ CmbbBrkDecel_No_Cs : 39|8@0+ (1,0) [0|0] "" XXX
 SG_ CmbbBrkDecel_A_Rq : 23|13@0+ (0.0039,-20.0) [0|0] "m/s^2" XXX
 SG_ CmbbBrkPrchg_D_Rq : 47|2@0+ (1,0) [0|0] "" XXX
 SG_ CmbbBrkDecel_B_Rq : 26|1@0+ (1,0) [0|0] "" XXX
 SG_ CmbbBaSens_D_Rq : 25|2@0+ (1,0) [0|0] "" XXX
 SG_ AccPrpl_V_Rq : 7|16@0+ (0.01,0) [0|0] "kph" XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
*/

#define ACCEL_RES	0.01

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float AccPrpl_V_Rq_veh_speed; //Desired velocity, km/s
	float CmbbBrkDecel_A_Rq; //Desired deceleration, m/s^2
	unsigned char Acc_Resume_Active; 	// (1 bit, set to 0),
	unsigned char Acc_Autobrake_Cancel;	// (1 bit, set to 0)
} taurus_acc_393_t;

static inline void get_taurus_acc_393(unsigned char *data, taurus_acc_393_t *p) {
	short short_temp;

	p->Acc_Autobrake_Cancel = data[7] & 0x01;
	p->Acc_Autobrake_Cancel = (data[7] & 0x02) >> 1;

	short_temp = (data[0] << 1) + (data[1] & 0xFF);
	p->AccPrpl_V_Rq_veh_speed = short_temp * 0.01; 	

	short_temp = (data[2] << 5) + (data[3] >> 3);
	p->CmbbBrkDecel_A_Rq = short_temp * 0.0039; 
}

static inline void set_taurus_acc_393(unsigned char *data, float deceleration, float vehicle_speed_kph) {
	short short_temp;
	float decel_calc;

	decel_calc = -0.003713 * vehicle_speed_kph - 0.032923;

	data[7] |=  0xFC; //Set Acc_Autobrake_Cancel and Acc Resume_Active to 0

	//Set AccPrpl_V_Rq (set to vehicle speed)
	if(deceleration < decel_calc) {
		short_temp = ((short)(vehicle_speed_kph * 100)) & 0xFFFF;
		data[0] = (short_temp & 0xFF00) >> 8;
		data[1] = short_temp & 0xFF;
	}
}

/*******************************************************************************
 *      taurus_fuel_level
 *      Message ID 0x7E8
********************************************************************************/
typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	char size;
	char mode;
	char id;
	float fuel_level;
}taurus_fuel_level_t;

int get_taurus_fuel_level(unsigned char *data, taurus_fuel_level_t *p) {

	p->size = data[0];
	p->mode = data[1];
	p->id = data[2];
	p->fuel_level = data[3] * 100.0 / 255.0;
/*printf("library: Taurus fuel pct %.1f %#hhx %hhu size %#hhx mode %#hhx id %#hhx\n",
		p->fuel_level,
		data[3],
		data[3],
		data[0],
		data[1],
		data[2]
);*/
}
/*******************************************************************************
 *      taurus_vehicle_speed_513
 *      Message ID      513
 *      Transmitted every 20 ms
 *
 *	dbvar = DB_TAURUS_MSG513_VAR

BO_ 513 EngVehicleSpThrottle_CG1: 8 XXX^M
 SG_ ApedPos_PcRate_ActlArb : 63|8@0+ (0.04,-5.0) [0|0] "%/ms" XXX^M
 SG_ Veh_V_RqCcSet : 45|9@0+ (0.5,0) [0|0] "kph" XXX^M
 SG_ VehVActlEng_D_Qf : 9|2@0+ (1,0) [0|0] "" XXX^M
 SG_ reserve : 10|1@0+ (1,0) [0|0] "" XXX^M
 SG_ EngAout_N_Actl : 7|13@0+ (2.0,0) [0|0] "rpm" XXX^M
 SG_ Veh_V_ActlEng : 23|16@0+ (0.01,0) [0|0] "kph" XXX^M
 SG_ ApedPos_Pc_ActlArb : 39|10@0+ (0.1,0) [0|0] "%" XXX^M
 SG_ ApedPosPcActl_D_Qf : 52|2@0+ (1,0) [0|0] "" XXX^M
 SG_ Autostart_B_Stat : 50|1@0+ (1,0) [0|0] "" XXX^M
*/

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float EngAout_N_Actl;		// : 7|13@0+ (2.0,0) [0|0] "rpm" XXX^M
	unsigned char VehVActlEng_D_Qf; // : 9|2@0+ (1,0) [0|0] "" XXX^M
	unsigned char reserve;		// : 10|1@0+ (1,0) [0|0] "" XXX^M
	float Veh_V_ActlEng;		// : 23|16@0+ (0.01,0) [0|0] "kph" XXX^M
	float vehicle_speed_kph;		// : 23|16@0+ (0.01,0) [0|0] "kph" XXX^M
	float ApedPos_Pc_ActlArb;	// : 39|10@0+ (0.1,0) [0|0] "%" XXX^M
	float Veh_V_RqCcSet;		// : 45|9@0+ (0.5,0) [0|0] "kph" XXX^M
	unsigned char Autostart_B_Stat;	// : 50|1@0+ (1,0) [0|0] "" XXX^M
	unsigned char ApedPosPcActl_D_Qf;// : 52|2@0+ (1,0) [0|0] "" XXX^M
	float ApedPos_PcRate_ActlArb;	// : 63|8@0+ (0.04,-5.0) [0|0] "%/ms" XXX^M
}taurus_vehicle_speed_513_t;

static inline void get_taurus_vehicle_speed_513(unsigned char *data, taurus_vehicle_speed_513_t *p) {
	short short_temp;

	short_temp = (data[2] << 8) + data[3];
	p->vehicle_speed_kph  = short_temp * 0.01;
}

/*******************************************************************************
 *      taurus_long_lat_accel
 *      Message ID      0x92
 *      Transmitted every 20 ms
 *
 *	dbvar = DB_TAURUS_MSG92_VAR
 *
 *	lateral_acceleration
 *      Byte Position   0-1
 *      Bit Position    0
 *      Bit Length      13
 *
 *	longitudinal_acceleration
 *      Byte Position   2-3
 *      Bit Position    0
 *      Bit Length      13
 *
 *	vertical_acceleration
 *      Byte Position   4-5
 *      Bit Position    0
 *      Bit Length      13

BO_ 146 Accel_Data: 8 XXX
 SG_ VehLatAActl_D_Qf : 6|2@0+ (1,0) [0|0] "" XXX
 SG_ VehLat_A_Actl : 4|13@0+ (0.01,-40.0) [0|0] "m/s^2" XXX

 SG_ VehLongAActl_D_Qf : 22|2@0+ (1,0) [0|0] "" XXX
 SG_ VehLong_A_Actl : 20|13@0+ (0.01,-40.0) [0|0] "m/s^2" XXX

 SG_ VehVertAActl_D_Qf : 38|2@0+ (1,0) [0|0] "" XXX
 SG_ VehVert_A_Actl : 36|13@0+ (0.01,-40.0) [0|0] "m/s^2" XXX

 7 6 5 4 3 2 1 0   15 14 13 12 11 10 9 8   23 22 21 20 19 18 17 16   31 30 29 28 27 26 25 24
 39 38 37 36 35 34 33 32   47 46 45 44 43 42 41 40   55 54 53 52 51 50 49 48   63 62 61 60 59 58 57 56
*/

#define ACCEL_RES	0.01

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;
	float lateral_acceleration;
	float longitudinal_acceleration;
	float vertical_acceleration;
} taurus_long_lat_accel_t;

static inline void get_taurus_long_lat_accel(unsigned char *data, taurus_long_lat_accel_t *p) {
	unsigned char char_temp;
	short short_temp;

	if( (data[0] & 0x10) != 0)
		char_temp = data[0] | 0xE0;
	else
		char_temp = data[0] & 0x1F;
	short_temp = (short)((char_temp << 8) + data[1]);
	p->lateral_acceleration = short_temp * ACCEL_RES;

//	if( (data[2] & 0x10) != 0)
//		char_temp = data[2] | 0xE0;
//	else
		char_temp = data[2] & 0x1F;
	short_temp = (short)((char_temp << 8) + data[3]);
//	short_temp = ((short)((data[2] << 8) + data[3])) & 0x1FFF;
	p->longitudinal_acceleration = (short_temp * ACCEL_RES) - 39.935;
//printf("taurus_can.h: longitudinal acceleration %.2f data[2] %#hhx data[3] %#hhx short_temp %#hx\n",
//		p->longitudinal_acceleration,
//		data[2],
//		data[3],
//		short_temp);
	if( (data[4] & 0x10) != 0)
		char_temp = data[4] | 0xE0;
	else
		char_temp = data[4] & 0x1F;
	short_temp = (short)((char_temp << 8) + data[5]);
	p->vertical_acceleration = short_temp * ACCEL_RES;
}

/*
** printcan.c - prints 8-byte CAN message to stdout
*/
int printcan(db_steinhoff_msg_t *steinhoff_msg);

//int print_vehicle_speed(taurus_wheel_speed_t *taurus_wheel_speed);

void check_msg_timeout(int curr_ts_ms, int *prev_ts_ms,
        unsigned char *two_message_periods,
        unsigned int *message_timeout_counter);
