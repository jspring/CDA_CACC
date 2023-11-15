#pragma once
//#include <db_utils.h>
extern "C" {
#include "db_include.h"
#include "long_comm.h"
#include "can_dbvars.h"
//#include <clt_vars_camry.h>
#include <clt_vars_leaf.h>
#include <clt_vars_prius.h>
#include <clt_vars_accord.h>
#include "leddar_SIGHT.h"
#include "path_gps_lib.h"
}
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "long_control.hpp"
#include "hmi_server.h"

#define MAX_VARS_SIZE 600000
#define BILLION  1000000000L;

//#define
#define MAX_TARGET_NUM 20

void Initialization_task(int argc, char *argv[]);
bool Read_inputs();
bool Process_data();
bool Write_outputs();
void End_tasks();
int get_accord_targets();
int get_prius_targets();
int get_camry_targets();
int get_leaf_targets();
void Read_db_v2x_comm_data();
void UpdateStandardLogFile();
void InitOutputFile();
void Write_read_hmi_variables_to_database();
void Init_db_variables_from_hmi();
bool Load_configuration_file(char* file_name);
bool Fill_config_structure(setup_struct *ss, char* name, float value);
int Check_vehicle_id(char *name);

int ego_vehicle_id, preceding_vehicle_id;
double sampling_period;
int desired_control_mode;
int test_mode;
clock_t t0;
struct timespec t_start, t_current;
double current_time;
long executions;
int my_pip;
tx_data *db_2_hmi_data;
rx_data *hmi_2_db_data;
int with_hmi;
bool cacc_flag_performance_test;
int test_scenario_id;
bool manual_control_requested;
int v2v_sequence_number;
int v2i_sequence_number;
bool v2i_engaged;
bool trajectory_planning_requested;
int preferred_topology;
bool comm_verbosity;

double i_long_speed;
double i_long_acceleration;
double i_relative_distance_to_target;
double i_relative_speed_to_target;
double i_torque_applied;
int    i_transmission_gear;
double i_control_mode;
double i_time_gap;
double i_setpoint_speed;
double i_prec_veh_setpoint_speed[4];
double i_throttle_test;
double i_deceleration_test;
double i_desired_acceleration;
int    i_num_targets;
double i_preceding_ref_speed;
double i_latitude;
double i_longitude;
double i_steering;
double i_fuel_rate;
double o_throttle_command;
double o_brake_command;
double o_deceleration_command;
double o_ego_setpoint_speed;
double o_desired_acceleration;
long_control* control_structure;
control_variables vars_array[MAX_VARS_SIZE];
speed_profile* sProfile;
setup_struct* config;
FILE* fp;
FILE* fp_t;
bool trigger_EB;
int enable_flag_from_hmi;
bool camry_verbosity;
bool fuel_verbosity;
bool gps_verbosity;

veh_comm_packet_t comm_pkt_Virtual_vehicle;
veh_comm_packet_t comm_pkt1;
veh_comm_packet_t comm_pkt2;
veh_comm_packet_t comm_pkt3;
path_gps_point_t gps_data;
