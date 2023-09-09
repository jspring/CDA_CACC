#pragma once
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include "db_include.h"
#include "can_dbvars.h"
#include "long_comm.h"

#define CRUISE 0
#define ACC 1
#define CACC 2
#define CACC_PERF 3
#define CRUISE_2_ACC 4
#define ACC_2_CRUISE 5
#define CACC_2_ACC 6
#define ACC_2_CACC 7
#define EMERGENCY_BRAKING 8

#define LEAF	0
#define PRIUS  	1
#define CAMRY	2

#define PORT 4000
#define MAX_BUFFER_SIZE

#define DB_2_HMI_DATA    20000
#define HMI_2_DB_DATA    20001
#define MONTH            20010
#define DAY              20011
#define YEAR             20012
#define HOUR             20013
#define MINUTE           20014
#define SECONDS          20015
#define MILLISECONDS     20016
#define CACC_CONTROL_ENABLE   20017


#define NB_DATA_FROM_HMI 5
#define BUFF_SIZE 4096

static char VEHICLE_NAMES[3][5] = {"camr", "priu", "leaf"};

typedef struct rx_data{
	int desired_control_mode;
	int gap_choice;
	float desired_time_gap;
	float desired_cruise_speed;
	float performance_factor;
} rx_data;

typedef struct tx_data{
	int my_pip;
	int target_vehicle_valid;
	int current_control_mode;
	int v2v_available;
	int sys_status;
	float ego_speed;
	float veh_1_speed;
	float veh_2_speed;
	float veh_3_speed;
	float measured_time_gap;
	float ACC_set_time_gap;
	float CACC_set_time_gap;
	float timestamp;
} tx_data;

typedef struct init_rx_data{
	int sync_requested;
};

db_id_t db_vars_list_hmi[] =  {
	{DB_2_HMI_DATA, sizeof(tx_data)},
	{HMI_2_DB_DATA, sizeof(rx_data)},
	{CACC_CONTROL_ENABLE, sizeof(int)}
};

#define NUM_DB_VARS_HMI (sizeof(db_vars_list_hmi)/sizeof(db_id_t))

int EndTasks(int server_socket);
void Init_database(int argc, char const *argv[]);
int Receive_sync_data_from_hmi(int socket);
int Read_control_variables_from_database();
int Receive_data_from_hmi(int socket);
int Write_user_variables_to_database();
int Send_data_to_hmi(int socket);
int Send_sync_data_to_hmi(int socket, int sync_flag, int start_flag);
int Write_sync_flag_to_database(int* flag);
int Update_local_machine_time_with_ref_vehicle();

rx_data *hmi_data;
tx_data *db_data;
tx_data *db_data_dummy;
char string_received[2048];
char string_to_send[2048];
int ego_id;
int time_sync_requested;
int sync_button_state;
int start_button_state;
int sync_flag;
int ref_vehicle_id;
veh_comm_packet_t comm_pkt_1;
veh_comm_packet_t comm_pkt_2;
veh_comm_packet_t comm_pkt_3;
veh_comm_packet_t ref_veh_pkt;

db_clt_typ *pclt_hmi;
static jmp_buf exit_env_hmi;
static int sig_list_hmi[] = {
        SIGINT,
        SIGQUIT,
        SIGTERM,
        SIGALRM,
        ERROR,
};
static void sig_hand_hmi(int code)
{
	if (code == SIGALRM)
		return;
	else
		longjmp(exit_env_hmi, code);
}
