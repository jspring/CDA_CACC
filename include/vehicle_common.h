#pragma once

typedef struct {
	float vehicle_speed_mps;	// Longitudinal speed
	float vehicle_accel_mps2;	// Longitudinal acceleration
	float torque;			// Applied torque (if available)
	char gear;			// Transmission gear
	char brake_pressure;
	char brake_switch;
	short brake_position;
	float yaw_rate_dps;		// Yaw rate (if available)
	float distance_from_start;	// Distance from the beginning of the track for virtual car
	float fuel_rate;		// Fuel rate, l/s
	char system_status;		// Any type of system status (e.g. ACC mode on)
	char vehID[4];			// Vehicle ID = priu, acco, taur
	char ctlMode;			// Control mode 0=invalid, 1=manual, 1=ACC, 2=CACC
	float desTGapPVeh;		// Desired time gap to preceding veh in 100 ms increments, range 0-3.0 s
	float desDGapPVeh;		// Desired distance gap to preceding veh in 0.01 m increments, range 0-300.00 m
	unsigned int sys_status;	// System status flags

	float steering_angle_deg;	// Steering angle (if available)
	float steering_torque;		// Steering torque (if available)
        float desired_angle;
        float percent_torque;
        char steering_enabled;
        char driver_override;
        char steering_fault;
        char overheat_prevent;
        char overheat_warning;
        char misc_by_wire_ready;
        char misc_comm_fault;
        char heartbeat;
} input_t;

typedef struct {
	float throttle_pct;			// Throttle percentage
	float decel_request;			// Desired deceleration (for braking) 
	float brake_level;			// Brake level (applies only for Accord)
	float steering_angle_request;		// Steering angle desired
	float steering_torque_pct_request;	// Steering torque percent request
	char steering_angle_or_torque_request;	//Flag to determine if steering request is for angle (=+1) or torque (=-1)
} output_t;

//System status flags
#define SYSSTAT_VEH_SND_ENCODE_ERR		((unsigned int)(0x00000001))
#define SYSSTAT_VEH_SND_SEND_ERR		((unsigned int)(0x00000002))
#define SYSSTAT_VEH_SND_VELOCITY_SAME	((unsigned int)(0x00000004))

