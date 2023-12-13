#pragma once

#include <db_include.h>

#define DB_PIXELL_TYPE			7000
#define DB_PIXELL_VAR			DB_PIXELL_TYPE

void check_msg_timeout(int curr_ts_ms, int *prev_ts_ms, unsigned char *two_message_periods, unsigned int *message_timeout_counter);

typedef struct {
	timestamp_t ts;
	unsigned char pixell_status;
	unsigned char pixell_numTargets;
	int pixell_x[10]; // x coordinates of targets, in 1E-6 m precision (micrometer), default 0
	int pixell_y[10]; // y
	int pixell_z[10]; // z
} LidarDataFiltered_t;

typedef struct {
	timestamp_t ts;
	unsigned char pixell_status;
	unsigned char pixell_numTargets;
	float pixell_x[10]; // x coordinates of targets, in 1E-6 m precision (micrometer), default 0
	float pixell_y[10]; // y
	float pixell_z[10]; // z
} db_LidarDataFiltered_t;
