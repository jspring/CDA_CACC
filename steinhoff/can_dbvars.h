#pragma once
#define	DB_INPUT_TYPE				1000
#define	DB_OUTPUT_TYPE				2000
#define	DB_STEINHOFF_MSG_TYPE		3000
#define	DB_STEINHOFF_MSG2_TYPE		3001
#define	DB_STEINHOFF_OBD2_IN_3002_TYPE	3002
#define	DB_STEINHOFF_CHASSIS_3004_TYPE	3004
#define	DB_STEINHOFF_BRAKE_OUT_TYPE	4000
#define	DB_STEINHOFF_OBD2_OUT_TYPE	4001
#define	DB_STEINHOFF_ACCEL_OUT_TYPE	5000
#define	DB_STEINHOFF_STEERING_OUT_TYPE	5300

#define	DB_INPUT_VAR				DB_INPUT_TYPE
#define	DB_OUTPUT_VAR				DB_OUTPUT_TYPE
#define	DB_STEINHOFF_MSG_VAR		DB_STEINHOFF_MSG_TYPE
#define	DB_STEINHOFF_MSG2_VAR		DB_STEINHOFF_MSG2_TYPE
#define	DB_STEINHOFF_OBD2_IN_3002_VAR	DB_STEINHOFF_OBD2_IN_3002_TYPE
#define	DB_STEINHOFF_CHASSIS_3004_VAR	DB_STEINHOFF_CHASSIS_3004_TYPE
#define	DB_STEINHOFF_BRAKE_OUT_VAR	DB_STEINHOFF_BRAKE_OUT_TYPE
#define	DB_STEINHOFF_OBD2_OUT_VAR	DB_STEINHOFF_OBD2_OUT_TYPE
#define	DB_STEINHOFF_ACCEL_OUT_VAR	DB_STEINHOFF_ACCEL_OUT_TYPE
#define	DB_STEINHOFF_STEERING_OUT_VAR	DB_STEINHOFF_STEERING_OUT_TYPE
#define BRAKE_PORT	(char)1
#define ACCEL_PORT	(char)2
#define STEERING_PORT	(char)3
#define PORT_1	(char)1
#define PORT_2	(char)2
#define PORT_3	(char)3

typedef struct{
	int id;
	unsigned char size;
	unsigned char data[8];
} __attribute__((__packed__)) db_steinhoff_msg_t;

typedef struct{
	int id;
	unsigned char data[8];
	unsigned char size;
	unsigned char port;
} __attribute__((__packed__)) db_steinhoff_out_t;
