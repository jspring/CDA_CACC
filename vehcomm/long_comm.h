#ifndef LONG_COMM_H
#define LONG_COMM_H

/* This is used for the size of the array to hold 
 * veh_comm_packet_t received from other "trucks"
 */
#define MAX_PLATOON_SIZE	10

/** Use numbers from 480 to 489 for storing up to 10
 * veh_comm_packet_t variables in the DB for comm experiments.
 */
#define DB_COMM_BASE_TYPE 	480
#define DB_COMM_BASE_VAR 	480

/* Current longistudinal control code distinguishes messages
 * from the leader and from the preceding by their ID
 */
#define DB_COMM_VIRTUAL_TRK_TYPE 495		/// veh_comm_packet_t 
#define DB_COMM_VIRTUAL_TRK_VAR  DB_COMM_VIRTUAL_TRK_TYPE
#define DB_COMM_LEAD_TRK_TYPE	496		/// veh_comm_packet_t 
#define DB_COMM_LEAD_TRK_VAR	DB_COMM_LEAD_TRK_TYPE
#define DB_COMM_SECOND_TRK_TYPE	497		/// veh_comm_packet_t 
#define DB_COMM_SECOND_TRK_VAR	DB_COMM_SECOND_TRK_TYPE
#define DB_COMM_THIRD_TRK_TYPE	498		/// veh_comm_packet_t 
#define DB_COMM_THIRD_TRK_VAR	DB_COMM_THIRD_TRK_TYPE
#define DB_REPEAT_COMM_PACKET_TYPE	500	/// veh_comm_packet_t 
#define DB_REPEAT_COMM_PACKET_VAR	DB_REPEAT_COMM_PACKET_TYPE
#define BSM_FLOAT_MULT		100.0
#define LONG_LAT_MULT		1e7
#define HEADING_MULT		80.0
#define VELOCITY_MULT		50.0
#define ACCEL_MULT			100.0

/* Use this veh_comm_pkt_t variable to write the packet
 * that will be sent.
 */
#define DB_COMM_TX_TYPE		499 /// veh_comm_packet_t
#define DB_COMM_TX_VAR		DB_COMM_TX_TYPE

/* communications packet definition 
*/
typedef struct {
	int node;		// Node number of packet origin
	timestamp_t rcv_ts;     // When message is received, from veh_recv
	timestamp_t ts;         // When message is sent, from veh_send
	float global_time;	// From long_ctl or trk_comm_mgr 
	float user_float;
	float user_float1;
	unsigned char user_ushort_1;
	unsigned char user_ushort_2;
	unsigned char my_pip;  // My position-in-platoon (i.e. 1, 2, or 3)
	unsigned char maneuver_id;
	unsigned char fault_mode;
	unsigned char maneuver_des_1;
	unsigned char maneuver_des_2;
	unsigned char pltn_size;
	char sequence_no;
	unsigned :0;		//Force int boundary for next 4 bit fields
	unsigned user_bit_1 : 1;
	unsigned user_bit_2 : 1;
	unsigned user_bit_3 : 1;
	unsigned user_bit_4 : 1;
	float acc_traj;		//Desired acceleration from profile (m/s^2)
	float vel_traj;		//Desired velocity from profile (m/s)
	float velocity;		//Current velocity (m/s)
	float accel;		//Current acceleration (m/s^2)
	float range;		//Range from *dar (m)
	float rate;		//Relative velocity from *dar (m/s)
	double longitude;
	double latitude;
	float heading;
	int desired_torque;
	unsigned char brake_switch;
	unsigned char cc_active;
	char object_id[7];
} IS_PACKED veh_comm_packet_t;

// Message structure used by the test CAVs ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
typedef struct{
	// Time info
	short year;
	char month;
	char day;
	char hour;
	char minute;
	char second;
	short ms;

	// Test vehicle info
	short ID;
	short leaderID; // String leader ID
	char stringPos;
	char route; // 1: WB; 2: EB
	short v; // 0.001 m/s
	int pos; // Position from the start, 0.001 m
	char lane;
} IS_PACKED test_veh_data_t;

typedef struct {
	// Time info
	short year; //0,1
	char month; //2
	char day;	//3
	char hour;	//4
	char minute;//5
	char second;//6
	short ms;	//7,8

	// Sim veh ID, used to determine if there is a virtual leading vehicle
	int16_t simID;//9,10

	// Target test vehicle
	short targetVehID;//11,12

	// Virtual leading vehicle
	int v; // 13,14,15,16 0.001 m/s
	int pos; // 17,18,19,20 Position from the start, 0.001 m

	// Signal control
	char signalState;//21
	int endTime; // 22,23,24,25 numer of ms since 00:00:00 of the day

	// TP Info
	int refAcc; // 26,27,28,29 0.001 m/s2
	//int32_t intersectionLoc; // Location of the downstream intersection, 0.001 m
} IS_PACKED server_2_test_vehicle_t;

typedef veh_comm_packet_t long_comm_pkt;

typedef struct
{
	int db_var;
	char platoon_pos;
	int rx_msg_cnt;
	int rx_msg_err;
	char rx_msg_seq;
	int msg_ts_last;
} IS_PACKED veh_rcv_t;


#endif 
