#pragma once
#define	DB_INPUT_TYPE				1000
#define	DB_OUTPUT_TYPE				2000
#define	DB_STEINHOFF_MSG_TYPE		3000
#define	DB_STEINHOFF_BRAKE_OUT_TYPE	4000
#define	DB_STEINHOFF_ACCEL_OUT_TYPE	5000
#define DB_CAR_CAN_IN_TYPE		6000
#define DB_CAR_CAN_OUT_TYPE		6100
#define DB_CCM_CAN_IN_TYPE		6200
#define DB_CCM_CAN_OUT_TYPE		6300

#define	DB_INPUT_VAR				DB_INPUT_TYPE
#define	DB_OUTPUT_VAR				DB_OUTPUT_TYPE
#define	DB_STEINHOFF_MSG_VAR		DB_STEINHOFF_MSG_TYPE
#define	DB_STEINHOFF_BRAKE_OUT_VAR	DB_STEINHOFF_BRAKE_OUT_TYPE
#define	DB_STEINHOFF_ACCEL_OUT_VAR	DB_STEINHOFF_ACCEL_OUT_TYPE
#define DB_CAR_CAN_IN_VAR		DB_CAR_CAN_IN_TYPE
#define DB_CAR_CAN_OUT_VAR		DB_CAR_CAN_OUT_TYPE
#define DB_CCM_CAN_IN_VAR		DB_CCM_CAN_IN_TYPE
#define DB_CCM_CAN_OUT_VAR		DB_CCM_CAN_OUT_TYPE
#define BRAKE_PORT	(char)1
#define ACCEL_PORT	(char)2

typedef struct{
	unsigned long id;
	unsigned char size;
	unsigned char data[8];
} IS_PACKED db_steinhoff_msg_t;

typedef struct{
	unsigned long id;
	unsigned char data[8];
	unsigned char size;
	unsigned char port;
} IS_PACKED db_steinhoff_out_t;
