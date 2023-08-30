#pragma once

typedef struct {
	float vehicle_speed_mps;	// Longitudinal speed
	float vehicle_accel_mps2;	// Longitudinal acceleration
	float torque;			// Applied torque (if available)
	char gear;			// Transmission gear
	float yaw_rate_dps;		// Yaw rate (if available)
	float steering_angle_deg;	// Steering angle (if available)
	float distance_from_start;	// Distance from the beginning of the track for virtual car
	float fuel_rate;		// Fuel rate, l/s
	char system_status;		// Any type of system status (e.g. ACC mode on)
	char vehID[4];			// Vehicle ID = priu, acco, taur
	char ctlMode;			// Control mode 0=invalid, 1=manual, 1=ACC, 2=CACC
	float desTGapPVeh;			// Desired time gap to preceding veh in 100 ms increments, range 0-3.0 s
	float desDGapPVeh;			// Desired distance gap to preceding veh in 0.01 m increments, range 0-300.00 m
	unsigned int sys_status;	// System status flags
} input_t;

typedef struct {
	float torque_level;	// Throttle percentage
	float accel_decel_request;	// Desired deceleration (for braking) 
	float steering_angle_request;	// Steering angle desired
	float brake_level;
	float throttle_pct;

} output_t;

//System status flags
#define SYSSTAT_VEH_SND_ENCODE_ERR		((unsigned int)(0x00000001))
#define SYSSTAT_VEH_SND_SEND_ERR		((unsigned int)(0x00000002))
#define SYSSTAT_VEH_SND_VELOCITY_SAME	((unsigned int)(0x00000004))

