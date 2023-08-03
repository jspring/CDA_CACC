/**\file
 *
 *	Tables for DB variables and column specification for logging
 *	CACC3 vehicles
 */	
#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "taurus_can.h"
#include "das3.h"

/**  DB VARs read, declared global so that the variable addresses will
 *   be available to initialize the DB variable and column specification arrays.
 */

path_gps_point_t cabinet_gps; 		// GPS in cabinet
path_gps_point_t my_gps; 
path_gps_point_t veh1_gps; 		// sent from SV Taurus
path_gps_point_t veh2_gps;
path_gps_point_t veh3_gps;

m56_steering_t m56_steering;
m56_gear_t m56_gear;
m56_engine_rpm_t m56_engine_rpm;
m56_its_alive_t m56_its_alive;
m56_pedal_position_t m56_pedal_position;
m56_wheel_speed_front_t m56_wheel_speed_front;
m56_wheel_speed_rear_t m56_wheel_speed_rear;
m56_acceleration_t m56_acceleration;
m56_acc_status_t m56_acc_status;
m56_eng_tq_acc_and_brake_flags_t m56_eng_tq_acc_and_brake_flags;
m56_dashboard_indicators_t m56_dashboard_indicators;
m56_abs_status_t m56_abs_status;
m56_turn_switch_status_t m56_turn_switch_status;
m56_atcvt_t m56_atcvt;
m56_transmission_mode_t m56_transmission_mode;
m56_front_wiper_status_t m56_front_wiper_status;
m56_lidar_target_t m56_lidar_target;
m56_yaw_rate_t m56_yaw_rate;
m56_lidar_status_t m56_lidar_status;
m56_ignition_status_t m56_ignition_status;
das3_ignition_status_t das3_ignition_status;

// Comm variables
m56_vdne491_m210_t m56_vdne491_m210;
m56_vdne491_m211_t m56_vdne491_m211;
m56_vdne491_m212_t m56_vdne491_m212;
m56_vdne491_m213_t m56_vdne491_m213;
m56_vpro4_m214_t m56_vpro4_m214;
m56_vpro4_m215_t m56_vpro4_m215;
m56_vpro4_m216_t m56_vpro4_m216;
m56_vpro4_m217_t m56_vpro4_m217;
m56_vdnc304_m218_t m56_vdnc304_m218;
m56_vdnc304_m219_t m56_vdnc304_m219;
m56_vdnc304_m21a_t m56_vdnc304_m21a;
m56_vdnc304_m21b_t m56_vdnc304_m21b;
m56_vpro4_m21c_t m56_vpro4_m21c;
m56_vpro4_m21d_t m56_vpro4_m21d;
m56_vpro4_m21e_t m56_vpro4_m21e;
m56_vpro4_m21f_t m56_vpro4_m21f;
m56_adas_289_t m56_adas_289;
m56_adas_28a_t m56_adas_28a;

m56_m200_t m56_m200;
m56_m201_t m56_m201;
m56_m202_t m56_m202;
m56_m203_t m56_m203;

m56_m650_t m56_m650;
m56_m651_t m56_m651;
m56_m652_t m56_m652;
m56_m653_t m56_m653;
m56_m654_t m56_m654;
m56_m655_t m56_m655;
m56_m656_t m56_m656;
m56_m657_t m56_m657;
m56_m658_t m56_m658;

m56_m710_t m56_m710;
m56_m711_t m56_m711;

extern m56_m4n0_t m56_m430;
extern m56_m4n0_t m56_m440;
extern m56_m4n0_t m56_m450;
extern m56_m4n0_t m56_m460;
extern m56_m4n0_t m56_m470;

extern m56_m4n1_t m56_m431;
extern m56_m4n1_t m56_m441;
extern m56_m4n1_t m56_m451;
extern m56_m4n1_t m56_m461;
extern m56_m4n1_t m56_m471;

extern m56_m4n2_t m56_m432;
extern m56_m4n2_t m56_m442;
extern m56_m4n2_t m56_m452;
extern m56_m4n2_t m56_m462;
extern m56_m4n2_t m56_m472;

extern m56_m4n3_t m56_m433;
extern m56_m4n3_t m56_m443;
extern m56_m4n3_t m56_m453;
extern m56_m4n3_t m56_m463;
extern m56_m4n3_t m56_m473;

extern m56_m4n4_t m56_m434;
extern m56_m4n4_t m56_m444;
extern m56_m4n4_t m56_m454;
extern m56_m4n4_t m56_m464;
extern m56_m4n4_t m56_m474;

extern m56_m4n5_t m56_m435;
extern m56_m4n5_t m56_m445;
extern m56_m4n5_t m56_m455;
extern m56_m4n5_t m56_m465;
extern m56_m4n5_t m56_m475;

extern m56_m4n6_t m56_m436;
extern m56_m4n6_t m56_m446;
extern m56_m4n6_t m56_m456;
extern m56_m4n6_t m56_m466;
extern m56_m4n6_t m56_m476;

extern m56_m4n7_t m56_m437;
extern m56_m4n7_t m56_m447;
extern m56_m4n7_t m56_m457;
extern m56_m4n7_t m56_m467;
extern m56_m4n7_t m56_m477;

extern m56_m4n8_t m56_m438;
extern m56_m4n8_t m56_m448;
extern m56_m4n8_t m56_m458;
extern m56_m4n8_t m56_m468;
extern m56_m4n8_t m56_m478;

extern m56_m4n0_t m56_m480;
extern m56_m4n0_t m56_m490;
extern m56_m4n0_t m56_m4A0;
extern m56_m4n0_t m56_m4B0;
extern m56_m4n0_t m56_m4C0;

extern m56_m4n1_t m56_m481;
extern m56_m4n1_t m56_m491;
extern m56_m4n1_t m56_m4A1;
extern m56_m4n1_t m56_m4B1;
extern m56_m4n1_t m56_m4C1;

extern m56_m4n2_t m56_m482;
extern m56_m4n2_t m56_m492;
extern m56_m4n2_t m56_m4A2;
extern m56_m4n2_t m56_m4B2;
extern m56_m4n2_t m56_m4C2;

extern m56_m4n3_t m56_m483;
extern m56_m4n3_t m56_m493;
extern m56_m4n3_t m56_m4A3;
extern m56_m4n3_t m56_m4B3;
extern m56_m4n3_t m56_m4C3;

extern m56_m4n4_t m56_m484;
extern m56_m4n4_t m56_m494;
extern m56_m4n4_t m56_m4A4;
extern m56_m4n4_t m56_m4B4;
extern m56_m4n4_t m56_m4C4;

extern m56_m4n5_t m56_m485;
extern m56_m4n5_t m56_m495;
extern m56_m4n5_t m56_m4A5;
extern m56_m4n5_t m56_m4B5;
extern m56_m4n5_t m56_m4C5;

extern m56_m4n6_t m56_m486;
extern m56_m4n6_t m56_m496;
extern m56_m4n6_t m56_m4A6;
extern m56_m4n6_t m56_m4B6;
extern m56_m4n6_t m56_m4C6;

extern m56_m4n7_t m56_m487;
extern m56_m4n7_t m56_m497;
extern m56_m4n7_t m56_m4A7;
extern m56_m4n7_t m56_m4B7;
extern m56_m4n7_t m56_m4C7;



/**
 *	Additional variables derived from DB VAR fields that
 *	are part of the output column specification
 */
timestamp_t timestamp;
double utc_seconds_since_midnight = 0.0;// UTC seconds since midnight 
double seconds_since_start = 0.0;	// seconds since start of run

double veh1_utc_seconds = 0.0;
double veh2_utc_seconds = 0.0;
double veh3_utc_seconds = 0.0;

/**
 *	This array is used to specify how to read in the DB vars.
 */
db_var_spec_t db_vars[] =
{
	{DB_M56_VCAN2_MSG002_VAR, sizeof(m56_steering_t), &m56_steering},
	{DB_M56_VCAN2_MSG174_VAR, sizeof(m56_gear_t), &m56_gear},
	{DB_M56_VCAN2_MSG180_VAR, sizeof(m56_engine_rpm_t), &m56_engine_rpm},
	{DB_M56_VCAN2_MSG1c3_VAR, sizeof(m56_its_alive_t), &m56_its_alive},
	{DB_M56_VCAN2_MSG239_VAR, sizeof(m56_pedal_position_t), &m56_pedal_position},
	{DB_M56_VCAN2_MSG284_VAR, sizeof(m56_wheel_speed_front_t), &m56_wheel_speed_front},
	{DB_M56_VCAN2_MSG285_VAR, sizeof(m56_wheel_speed_rear_t), &m56_wheel_speed_rear},
	{DB_M56_VCAN2_MSG292_VAR, sizeof(m56_acceleration_t), &m56_acceleration},
	{DB_M56_VCAN2_MSG2aa_VAR, sizeof(m56_acc_status_t), &m56_acc_status},
	{DB_M56_VCAN2_MSG2b0_VAR, sizeof(m56_eng_tq_acc_and_brake_flags_t), &m56_eng_tq_acc_and_brake_flags},
	{DB_M56_VCAN2_MSG2b3_VAR, sizeof(m56_dashboard_indicators_t), &m56_dashboard_indicators},
	{DB_M56_VCAN2_MSG354_VAR, sizeof(m56_abs_status_t), &m56_abs_status},
	{DB_M56_VCAN2_MSG358_VAR, sizeof(m56_turn_switch_status_t), &m56_turn_switch_status},
	{DB_M56_VCAN2_MSG421_VAR, sizeof(m56_atcvt_t), &m56_atcvt},
	{DB_M56_VCAN2_MSG5b0_VAR, sizeof(m56_transmission_mode_t), &m56_transmission_mode},
	{DB_M56_VCAN2_MSG625_VAR, sizeof(m56_front_wiper_status_t), &m56_front_wiper_status},
	{DB_M56_ITSCAN_MSG52b_VAR, sizeof(m56_lidar_target_t), &m56_lidar_target},
	{DB_M56_ITSCAN_MSG52c_VAR, sizeof(m56_yaw_rate_t), &m56_yaw_rate},
	{DB_M56_ITSCAN_MSG52d_VAR, sizeof(m56_lidar_status_t), &m56_lidar_status},
	{DB_GPS_PT_LCL_VAR, sizeof(path_gps_point_t), &my_gps},
	{DB_M56_IGNITION_VAR, sizeof(m56_ignition_status_t), &m56_ignition_status},
	{DB_DAS3_IGNITION_VAR, sizeof(das3_ignition_status_t), &das3_ignition_status},
	{DB_M56_VCAN2_MSG210_VAR, sizeof(m56_vdne491_m210_t), &m56_vdne491_m210},
	{DB_M56_VCAN2_MSG211_VAR, sizeof(m56_vdne491_m211_t), &m56_vdne491_m211},
	{DB_M56_VCAN2_MSG212_VAR, sizeof(m56_vdne491_m212_t), &m56_vdne491_m212},
	{DB_M56_VCAN2_MSG213_VAR, sizeof(m56_vdne491_m213_t), &m56_vdne491_m213},
	{DB_M56_VCAN2_MSG214_VAR, sizeof(m56_vpro4_m214_t), &m56_vpro4_m214},
	{DB_M56_VCAN2_MSG215_VAR, sizeof(m56_vpro4_m215_t), &m56_vpro4_m215},
	{DB_M56_VCAN2_MSG216_VAR, sizeof(m56_vpro4_m216_t), &m56_vpro4_m216},
	{DB_M56_VCAN2_MSG217_VAR, sizeof(m56_vpro4_m217_t), &m56_vpro4_m217},
	{DB_M56_VCAN2_MSG218_VAR, sizeof(m56_vdnc304_m218_t), &m56_vdnc304_m218},
	{DB_M56_VCAN2_MSG219_VAR, sizeof(m56_vdnc304_m219_t), &m56_vdnc304_m219},
	{DB_M56_VCAN2_MSG21a_VAR, sizeof(m56_vdnc304_m21a_t), &m56_vdnc304_m21a},
	{DB_M56_VCAN2_MSG21b_VAR, sizeof(m56_vdnc304_m21b_t), &m56_vdnc304_m21b},
	{DB_M56_VCAN2_MSG21c_VAR, sizeof(m56_vpro4_m21c_t), &m56_vpro4_m21c},
	{DB_M56_VCAN2_MSG21d_VAR, sizeof(m56_vpro4_m21d_t), &m56_vpro4_m21d},
	{DB_M56_VCAN2_MSG21e_VAR, sizeof(m56_vpro4_m21e_t), &m56_vpro4_m21e},
	{DB_M56_VCAN2_MSG21f_VAR, sizeof(m56_vpro4_m21f_t), &m56_vpro4_m21f},
	{DB_M56_VCAN2_MSG289_VAR, sizeof(m56_adas_289_t), &m56_adas_289},
	{DB_M56_VCAN2_MSG28a_VAR, sizeof(m56_adas_28a_t), &m56_adas_28a},

        {DB_M56_ITSCAN_MSG200_VAR, sizeof(m56_m200_t), &m56_m200},
        {DB_M56_ITSCAN_MSG201_VAR, sizeof(m56_m201_t), &m56_m201  },
        {DB_M56_ITSCAN_MSG202_VAR, sizeof(m56_m202_t), &m56_m202 },
        {DB_M56_ITSCAN_MSG203_VAR, sizeof(m56_m203_t), &m56_m203 },
        {DB_M56_ITSCAN_MSG650_VAR, sizeof(m56_m650_t), &m56_m650  },

        {DB_M56_ITSCAN_MSG651_VAR, sizeof(m56_m651_t), &m56_m651  },
        {DB_M56_ITSCAN_MSG652_VAR, sizeof(m56_m652_t), &m56_m652},
        {DB_M56_ITSCAN_MSG653_VAR, sizeof(m56_m653_t), &m56_m653  },
        {DB_M56_ITSCAN_MSG654_VAR, sizeof(m56_m654_t), &m56_m654  },
        {DB_M56_ITSCAN_MSG655_VAR, sizeof(m56_m655_t), &m56_m655  },

        {DB_M56_ITSCAN_MSG656_VAR, sizeof(m56_m656_t), &m56_m656  },
        {DB_M56_ITSCAN_MSG657_VAR, sizeof(m56_m657_t), &m56_m657  },
        {DB_M56_ITSCAN_MSG658_VAR, sizeof(m56_m658_t), &m56_m658  },
        {DB_M56_ITSCAN_MSG710_VAR, sizeof(m56_m710_t), &m56_m710  },
        {DB_M56_ITSCAN_MSG711_VAR, sizeof(m56_m711_t), &m56_m711  },

        {DB_M56_ITSCAN_MSG430_VAR, sizeof(m56_m4n0_t), &m56_m430  },
        {DB_M56_ITSCAN_MSG440_VAR, sizeof(m56_m4n0_t), & m56_m440  },
        {DB_M56_ITSCAN_MSG450_VAR, sizeof(m56_m4n0_t), & m56_m450  },
        {DB_M56_ITSCAN_MSG460_VAR, sizeof(m56_m4n0_t), & m56_m460  },
        {DB_M56_ITSCAN_MSG470_VAR, sizeof(m56_m4n0_t), & m56_m470  },

        {DB_M56_ITSCAN_MSG431_VAR, sizeof(m56_m4n1_t), & m56_m431  },
        {DB_M56_ITSCAN_MSG441_VAR, sizeof(m56_m4n1_t), & m56_m441  },
        {DB_M56_ITSCAN_MSG451_VAR, sizeof(m56_m4n1_t), & m56_m451  },
        {DB_M56_ITSCAN_MSG461_VAR, sizeof(m56_m4n1_t), & m56_m461  },
        {DB_M56_ITSCAN_MSG471_VAR, sizeof(m56_m4n1_t), & m56_m471  },

        {DB_M56_ITSCAN_MSG432_VAR, sizeof(m56_m4n2_t), & m56_m432 },
        {DB_M56_ITSCAN_MSG442_VAR, sizeof(m56_m4n2_t), &  m56_m442},
        {DB_M56_ITSCAN_MSG452_VAR, sizeof(m56_m4n2_t), &  m56_m452},
        {DB_M56_ITSCAN_MSG462_VAR, sizeof(m56_m4n2_t), &  m56_m462},
        {DB_M56_ITSCAN_MSG472_VAR, sizeof(m56_m4n2_t), &  m56_m472},

        {DB_M56_ITSCAN_MSG433_VAR, sizeof(m56_m4n3_t), & m56_m433 },
        {DB_M56_ITSCAN_MSG443_VAR, sizeof(m56_m4n3_t), &  m56_m443},
        {DB_M56_ITSCAN_MSG453_VAR, sizeof(m56_m4n3_t), &  m56_m453},
        {DB_M56_ITSCAN_MSG463_VAR, sizeof(m56_m4n3_t), &  m56_m463},
        {DB_M56_ITSCAN_MSG473_VAR, sizeof(m56_m4n3_t), &  m56_m473},

        {DB_M56_ITSCAN_MSG434_VAR, sizeof(m56_m4n4_t), &  m56_m434},
        {DB_M56_ITSCAN_MSG444_VAR, sizeof(m56_m4n4_t), &  m56_m444},
        {DB_M56_ITSCAN_MSG454_VAR, sizeof(m56_m4n4_t), &  m56_m454},
        {DB_M56_ITSCAN_MSG464_VAR, sizeof(m56_m4n4_t), &  m56_m464},
        {DB_M56_ITSCAN_MSG474_VAR, sizeof(m56_m4n4_t), &  m56_m474},

        {DB_M56_ITSCAN_MSG435_VAR, sizeof(m56_m4n5_t), &  m56_m435},
        {DB_M56_ITSCAN_MSG445_VAR, sizeof(m56_m4n5_t), &  m56_m445},
        {DB_M56_ITSCAN_MSG455_VAR, sizeof(m56_m4n5_t), & m56_m455 },
        {DB_M56_ITSCAN_MSG465_VAR, sizeof(m56_m4n5_t), &  m56_m465},
        {DB_M56_ITSCAN_MSG475_VAR, sizeof(m56_m4n5_t), &  m56_m475},

        {DB_M56_ITSCAN_MSG436_VAR, sizeof(m56_m4n6_t), &  m56_m436},
        {DB_M56_ITSCAN_MSG446_VAR, sizeof(m56_m4n6_t), & m56_m446 },
        {DB_M56_ITSCAN_MSG456_VAR, sizeof(m56_m4n6_t), &  m56_m456},
        {DB_M56_ITSCAN_MSG466_VAR, sizeof(m56_m4n6_t), &  m56_m466},
        {DB_M56_ITSCAN_MSG476_VAR, sizeof(m56_m4n6_t), &  m56_m476},

        {DB_M56_ITSCAN_MSG437_VAR, sizeof(m56_m4n7_t), & m56_m437 },
        {DB_M56_ITSCAN_MSG447_VAR, sizeof(m56_m4n7_t), & m56_m447 },
        {DB_M56_ITSCAN_MSG457_VAR, sizeof(m56_m4n7_t), &  m56_m457},
        {DB_M56_ITSCAN_MSG467_VAR, sizeof(m56_m4n7_t), &  m56_m467},
        {DB_M56_ITSCAN_MSG477_VAR, sizeof(m56_m4n7_t), & m56_m477 },

        {DB_M56_ITSCAN_MSG438_VAR, sizeof(m56_m4n8_t), &  m56_m438},
        {DB_M56_ITSCAN_MSG448_VAR, sizeof(m56_m4n8_t), &  m56_m448},
        {DB_M56_ITSCAN_MSG458_VAR, sizeof(m56_m4n8_t), &  m56_m458},
        {DB_M56_ITSCAN_MSG468_VAR, sizeof(m56_m4n8_t), &  m56_m468},
        {DB_M56_ITSCAN_MSG478_VAR, sizeof(m56_m4n8_t), &  m56_m478},

        {DB_M56_ITSCAN_MSG480_VAR, sizeof(m56_m4n0_t), &  m56_m480},
        {DB_M56_ITSCAN_MSG490_VAR, sizeof(m56_m4n0_t), &  m56_m490},
        {DB_M56_ITSCAN_MSG4A0_VAR, sizeof(m56_m4n0_t), &  m56_m4A0},
        {DB_M56_ITSCAN_MSG4B0_VAR, sizeof(m56_m4n0_t), &  m56_m4B0},
        {DB_M56_ITSCAN_MSG4C0_VAR, sizeof(m56_m4n0_t), &  m56_m4C0},

        {DB_M56_ITSCAN_MSG481_VAR, sizeof(m56_m4n1_t), &  m56_m481},
        {DB_M56_ITSCAN_MSG491_VAR, sizeof(m56_m4n1_t), &  m56_m491},
        {DB_M56_ITSCAN_MSG4A1_VAR, sizeof(m56_m4n1_t), &  m56_m4A1},
        {DB_M56_ITSCAN_MSG4B1_VAR, sizeof(m56_m4n1_t), &  m56_m4B1},
        {DB_M56_ITSCAN_MSG4C1_VAR, sizeof(m56_m4n1_t), &  m56_m4C1},

        {DB_M56_ITSCAN_MSG482_VAR, sizeof(m56_m4n2_t), &  m56_m482},
        {DB_M56_ITSCAN_MSG492_VAR, sizeof(m56_m4n2_t), &  m56_m492},
        {DB_M56_ITSCAN_MSG4A2_VAR, sizeof(m56_m4n2_t), &  m56_m4A2},
        {DB_M56_ITSCAN_MSG4B2_VAR, sizeof(m56_m4n2_t), &  m56_m4B2},
        {DB_M56_ITSCAN_MSG4C2_VAR, sizeof(m56_m4n2_t), &  m56_m4C2},

        {DB_M56_ITSCAN_MSG483_VAR, sizeof(m56_m4n3_t), & m56_m483 },
        {DB_M56_ITSCAN_MSG493_VAR, sizeof(m56_m4n3_t), &  m56_m493},
        {DB_M56_ITSCAN_MSG4A3_VAR, sizeof(m56_m4n3_t), &  m56_m4A3},
        {DB_M56_ITSCAN_MSG4B3_VAR, sizeof(m56_m4n3_t), & m56_m4B3 },
        {DB_M56_ITSCAN_MSG4C3_VAR, sizeof(m56_m4n3_t), &  m56_m4C3},

        {DB_M56_ITSCAN_MSG484_VAR, sizeof(m56_m4n4_t), &  m56_m484},
        {DB_M56_ITSCAN_MSG494_VAR, sizeof(m56_m4n4_t), & m56_m494 },
        {DB_M56_ITSCAN_MSG4A4_VAR, sizeof(m56_m4n4_t), & m56_m4A4 },
        {DB_M56_ITSCAN_MSG4B4_VAR, sizeof(m56_m4n4_t), &  m56_m4B4},
        {DB_M56_ITSCAN_MSG4C4_VAR, sizeof(m56_m4n4_t), &  m56_m4C4},

        {DB_M56_ITSCAN_MSG485_VAR, sizeof(m56_m4n5_t), &  m56_m485},
        {DB_M56_ITSCAN_MSG495_VAR, sizeof(m56_m4n5_t), &  m56_m495},
        {DB_M56_ITSCAN_MSG4A5_VAR, sizeof(m56_m4n5_t), &  m56_m4A5},
        {DB_M56_ITSCAN_MSG4B5_VAR, sizeof(m56_m4n5_t), &  m56_m4B5},
        {DB_M56_ITSCAN_MSG4C5_VAR, sizeof(m56_m4n5_t), &  m56_m4C5},

        {DB_M56_ITSCAN_MSG486_VAR, sizeof(m56_m4n6_t), &  m56_m486},
        {DB_M56_ITSCAN_MSG496_VAR, sizeof(m56_m4n6_t), &  m56_m496},
        {DB_M56_ITSCAN_MSG4A6_VAR, sizeof(m56_m4n6_t), &  m56_m4A6},
        {DB_M56_ITSCAN_MSG4B6_VAR, sizeof(m56_m4n6_t), &  m56_m4B6},
        {DB_M56_ITSCAN_MSG4C6_VAR, sizeof(m56_m4n6_t), &  m56_m4C6},

        {DB_M56_ITSCAN_MSG487_VAR, sizeof(m56_m4n7_t), &  m56_m487},
        {DB_M56_ITSCAN_MSG497_VAR, sizeof(m56_m4n7_t), &  m56_m497},
        {DB_M56_ITSCAN_MSG4A7_VAR, sizeof(m56_m4n7_t), &  m56_m4A7},
        {DB_M56_ITSCAN_MSG4B7_VAR, sizeof(m56_m4n7_t), &  m56_m4B7},
        {DB_M56_ITSCAN_MSG4C7_VAR, sizeof(m56_m4n7_t), &  m56_m4C7},

};

int num_db_vars = (sizeof(db_vars)/sizeof(db_var_spec_t));

/** This array is used to specify the output format of the "[mnop]d" file.
 */
data_log_column_spec_t file_specd[] = 
{
	{"HH:MM:SS.SSS ", &timestamp, BASE_TIMESTAMP, REPLAY_TIME},                            //###1
	{"%.1f ", &m56_steering.steering_angle, BASE_FLOAT, REPLAY_USE},                        //###2
	{"%.1f ", &m56_steering.steering_velocity, BASE_FLOAT, REPLAY_USE},                     //###3
	{"%hhu ", &m56_steering.message_counter, BASE_CHAR, REPLAY_USE},                        //###4
	{"%u ", &m56_steering.message_timeout_counter, BASE_INT, REPLAY_USE},                   //###5
	{"%.1f ", &m56_engine_rpm.engine_rpm, BASE_FLOAT, REPLAY_USE},                          //###6
	{"%.1f ", &m56_engine_rpm.mean_effective_torque, BASE_FLOAT, REPLAY_USE},               //###7
	{"%u ", &m56_engine_rpm.message_timeout_counter, BASE_INT, REPLAY_USE},                 //###8
	{"%.1f ", &m56_its_alive.its_target_pressure, BASE_FLOAT, REPLAY_USE},                  //###9
	{"%hhu ", &m56_its_alive.its_alive_flag, BASE_CHAR, REPLAY_USE},                        //###10
	{"%hhu ", &m56_its_alive.acc_request_flag, BASE_CHAR, REPLAY_USE},                      //###11
	{"%hhu ", &m56_its_alive.message_counter, BASE_CHAR, REPLAY_USE},                       //###12
	{"%u ", &m56_its_alive.message_timeout_counter, BASE_INT, REPLAY_USE},                  //###13
	{"%.1f ", &m56_pedal_position.pedal_position, BASE_FLOAT, REPLAY_USE},                  //###14
	{"%hhu ", &m56_pedal_position.acc_inhibit, BASE_CHAR, REPLAY_USE},                      //###15
	{"%hhu ", &m56_pedal_position.resume_sw, BASE_CHAR, REPLAY_USE},                        //###16
	{"%hhu ", &m56_pedal_position.acc_set_sw, BASE_CHAR, REPLAY_USE},                       //###17
	{"%hhu ", &m56_pedal_position.following_dist_sw, BASE_CHAR, REPLAY_USE},                //###18
	{"%hhu ", &m56_pedal_position.cancel_sw, BASE_CHAR, REPLAY_USE},            		//###19
	{"%hhu ", &m56_pedal_position.main_sw, BASE_CHAR, REPLAY_USE},                          //###20
	{"%hhu ", &m56_pedal_position.acc_can_fail_flag, BASE_CHAR, REPLAY_USE},                //###21
	{"%hhu ", &m56_pedal_position.brake_nc_sw, BASE_CHAR, REPLAY_USE},                      //###22
	{"%hhu ", &m56_pedal_position.brake_no_sw, BASE_CHAR, REPLAY_USE},                      //###23
	{"%hhu ", &m56_pedal_position.message_counter, BASE_CHAR, REPLAY_USE},                  //###24
	{"%u ", &m56_pedal_position.message_timeout_counter, BASE_INT, REPLAY_USE},             //###25
	{"%.2f ", &m56_wheel_speed_front.wheel_speed_front_right, BASE_FLOAT, REPLAY_USE},      //###26
	{"%.2f ", &m56_wheel_speed_front.wheel_speed_front_left, BASE_FLOAT, REPLAY_USE},       //###27
	{"%.2f ", &m56_wheel_speed_front.vehicle_speed_copy, BASE_FLOAT, REPLAY_USE},           //###28
	{"%hhu ", &m56_wheel_speed_front.message_counter, BASE_CHAR, REPLAY_USE},               //###29
	{"%u ", &m56_wheel_speed_front.message_timeout_counter, BASE_INT, REPLAY_USE},          //###30
	{"%.2f ", &m56_wheel_speed_rear.wheel_speed_rear_right, BASE_FLOAT, REPLAY_USE},        //###31
	{"%.2f ", &m56_wheel_speed_rear.wheel_speed_rear_left, BASE_FLOAT, REPLAY_USE},         //###32
	{"%hhu ", &m56_wheel_speed_rear.message_counter, BASE_CHAR, REPLAY_USE},                //###33
	{"%u ", &m56_wheel_speed_rear.message_timeout_counter, BASE_INT, REPLAY_USE},           //###34
	{"%.2f ", &m56_acceleration.long_accel_proc_02, BASE_FLOAT, REPLAY_USE},                //###35
	{"%.2f ", &m56_acceleration.transverse_accel_proc_02, BASE_FLOAT, REPLAY_USE},          //###36
	{"%.2f ", &m56_acceleration.yaw_rate_02, BASE_FLOAT, REPLAY_USE},                       //###37
	{"%hhu ", &m56_acceleration.pressure_sensor_02, BASE_CHAR, REPLAY_USE},                 //###38
	{"%hhu ", &m56_acceleration.message_counter, BASE_CHAR, REPLAY_USE},                    //###39
	{"%u ", &m56_acceleration.message_timeout_counter, BASE_INT, REPLAY_USE},               //###40
	{"%.1f ", &m56_acc_status.estimated_pressure_value, BASE_FLOAT, REPLAY_USE},            //###41
	{"%hhu ", &m56_acc_status.release_sw, BASE_CHAR, REPLAY_USE},                           //###42
	{"%hhu ", &m56_acc_status.pbfs_nc, BASE_CHAR, REPLAY_USE},                              //###43
	{"%hhu ", &m56_acc_status.pbfs_no, BASE_CHAR, REPLAY_USE},                              //###44
	{"%hhu ", &m56_acc_status.acc_enable, BASE_CHAR, REPLAY_USE},                           //###45
	{"%hhu ", &m56_acc_status.acc_status, BASE_CHAR, REPLAY_USE},                           //###46
	{"%hhu ", &m56_acc_status.acc_alive, BASE_CHAR, REPLAY_USE},                            //###47
	{"%hhu ", &m56_acc_status.message_counter, BASE_INT, REPLAY_USE},                       //###48
	{"%u ", &m56_acc_status.message_timeout_counter, BASE_INT, REPLAY_USE},                 //###49
	{"%.1f ", &m56_eng_tq_acc_and_brake_flags.target_engine_torque_main_cpu, BASE_FLOAT, REPLAY_USE}, //###50
	{"%.1f ", &m56_eng_tq_acc_and_brake_flags.target_engine_torque_sub_cpu, BASE_FLOAT, REPLAY_USE}, //###51
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.driver_brake_nc, BASE_CHAR, REPLAY_USE},      //###52
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.driver_brake_no, BASE_CHAR, REPLAY_USE},      //###53
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.acc_main_sw, BASE_CHAR, REPLAY_USE},          //###54
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.od_off_flag, BASE_CHAR, REPLAY_USE},          //###55
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.acc_fail_flag, BASE_CHAR, REPLAY_USE},        //###56
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.control_autostop_flag, BASE_CHAR, REPLAY_USE},//###57
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.acc_active_flag_for_ecm, BASE_CHAR, REPLAY_USE}, //###58
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.acc_cruise_flag, BASE_CHAR, REPLAY_USE},      //###59
	{"%.2f ", &m56_eng_tq_acc_and_brake_flags.virtual_accelerator_angle, BASE_FLOAT, REPLAY_USE}, //###60
	{"%hhu ", &m56_eng_tq_acc_and_brake_flags.message_counter, BASE_CHAR, REPLAY_USE},      //###61
	{"%u ", &m56_eng_tq_acc_and_brake_flags.message_timeout_counter, BASE_INT, REPLAY_USE}, //###62
	{"%hhu ", &m56_dashboard_indicators.main_sw, BASE_CHAR, REPLAY_USE},                    //###63
	{"%hhu ", &m56_dashboard_indicators.target_lock, BASE_CHAR, REPLAY_USE},                //###64
	{"%hhu ", &m56_dashboard_indicators.target_approach_warning, BASE_CHAR, REPLAY_USE},    //###65
	{"%hhu ", &m56_dashboard_indicators.blink_at_target_icon, BASE_CHAR, REPLAY_USE},       //###66
	{"%hhu ", &m56_dashboard_indicators.segment_car_space, BASE_CHAR, REPLAY_USE},          //###67
	{"%hhu ", &m56_dashboard_indicators.oneself, BASE_CHAR, REPLAY_USE},                    //###68
	{"%hhu ", &m56_dashboard_indicators.auto_brake, BASE_CHAR, REPLAY_USE},                 //###69
	{"%hhu ", &m56_dashboard_indicators.dash_dash_indicator_signal, BASE_CHAR, REPLAY_USE}, //###70
	{"%hhu ", &m56_dashboard_indicators.kph_indicator_signal, BASE_CHAR, REPLAY_USE},       //###71
	{"%hhu ", &m56_dashboard_indicators.acc_buzzer_signal, BASE_CHAR, REPLAY_USE},          //###72
	{"%hhu ", &m56_dashboard_indicators.pbs2_warning, BASE_CHAR, REPLAY_USE},               //###73
	{"%hhu ", &m56_dashboard_indicators.speed_set_driver, BASE_CHAR, REPLAY_USE},           //###74
	{"%hhu ", &m56_dashboard_indicators.acc_buzzer_3rd, BASE_CHAR, REPLAY_USE},             //###75
	{"%hhu ", &m56_dashboard_indicators.acc_buzzer_2nd, BASE_CHAR, REPLAY_USE},             //###76
	{"%hhu ", &m56_dashboard_indicators.booster_active_reverse_bit, BASE_CHAR, REPLAY_USE}, //###77
	{"%hhu ", &m56_dashboard_indicators.booster_active, BASE_CHAR, REPLAY_USE},             //###78
	{"%hhu ", &m56_dashboard_indicators.pbs2_off, BASE_CHAR, REPLAY_USE},                   //###79
	{"%hhu ", &m56_dashboard_indicators.acc_bulb_check, BASE_CHAR, REPLAY_USE},             //###80
	{"%hhu ", &m56_dashboard_indicators.acc_system_fail_1_invert, BASE_CHAR, REPLAY_USE},   //###81
	{"%hhu ", &m56_dashboard_indicators.acc_system_fail_1, BASE_CHAR, REPLAY_USE},          //###82
	{"%hhu ", &m56_dashboard_indicators.trouble_trigger_code, BASE_CHAR, REPLAY_USE},       //###83
	{"%u ", &m56_dashboard_indicators.message_timeout_counter, BASE_INT, REPLAY_USE},       //###84
	{"%hhu ", &m56_abs_status.message_counter, BASE_CHAR, REPLAY_USE},                      //###85
	{"%hhu ", &m56_abs_status.abs_malfunction, BASE_CHAR, REPLAY_USE},                      //###86
	{"%hhu ", &m56_abs_status.abs_in_regulation, BASE_CHAR, REPLAY_USE},                    //###87
	{"%hhu ", &m56_abs_status.close_active_brake_switch_abs, BASE_CHAR, REPLAY_USE},        //###88
	{"%u ", &m56_abs_status.message_timeout_counter, BASE_INT, REPLAY_USE},                 //###89
	{"%hhu ", &m56_turn_switch_status.turn_switch_status, BASE_CHAR, REPLAY_USE},           //###90
	{"%u ", &m56_turn_switch_status.message_timeout_counter, BASE_INT, REPLAY_USE},         //###91
	{"%hhu ", &m56_transmission_mode.snow_mode_sw_status, BASE_CHAR, REPLAY_USE},           //###92
	{"%hhu ", &m56_transmission_mode.eco_mode_sw_status, BASE_CHAR, REPLAY_USE},            //###93
	{"%hhu ", &m56_transmission_mode.standard_mode_sw_status, BASE_CHAR, REPLAY_USE},       //###94
	{"%hhu ", &m56_transmission_mode.sport_mode_sw_status, BASE_CHAR, REPLAY_USE},          //###95
	{"%u ", &m56_transmission_mode.message_timeout_counter, BASE_INT, REPLAY_USE},          //###96
	{"%hhu ", &m56_front_wiper_status.front_wiper_status, BASE_CHAR, REPLAY_USE},           //###97
	{"%u ", &m56_front_wiper_status.message_timeout_counter, BASE_INT, REPLAY_USE},         //###98
	{"%.1f ", &m56_lidar_target.distance_to_target, BASE_FLOAT, REPLAY_USE},                //###99
	{"%.1f ", &m56_lidar_target.relative_speed_to_target, BASE_FLOAT, REPLAY_USE},          //###100
	{"%hhu ", &m56_lidar_target.object_type, BASE_CHAR, REPLAY_USE},                        //###101
	{"%hhu ", &m56_lidar_target.object_data_valid, BASE_CHAR, REPLAY_USE},                  //###102
	{"%u ", &m56_lidar_target.message_timeout_counter, BASE_INT, REPLAY_USE},               //###103
	{"%.1f ", &m56_yaw_rate.yaw_rate, BASE_FLOAT, REPLAY_USE},                              //###104
	{"%.1f ", &m56_yaw_rate.yaw_rate_sensor_value, BASE_FLOAT, REPLAY_USE},                 //###105
	{"%u ", &m56_yaw_rate.message_timeout_counter, BASE_INT, REPLAY_USE},                   //###106
	{"%hhu ", &m56_lidar_status.laser_operating_flag, BASE_CHAR, REPLAY_USE},               //###107
	{"%hhu ", &m56_lidar_status.laser_fail, BASE_CHAR, REPLAY_USE},                         //###108
	{"%u ", &m56_lidar_status.message_timeout_counter, BASE_INT, REPLAY_USE},               //###109
	{"%9.2f ", &utc_seconds_since_midnight, BASE_DOUBLE, REPLAY_USE},                       //###110
	{"%06d ", &my_gps.date, BASE_INT, REPLAY_USE},                                          //###111
	{"%13.8lf ", &my_gps.longitude, BASE_DOUBLE, REPLAY_USE},                               //###112
	{"%13.8lf ", &my_gps.latitude, BASE_DOUBLE, REPLAY_USE},                                //###113
	{"%6.3lf ", &my_gps.speed, BASE_DOUBLE, REPLAY_USE},                                    //###114
	{"%6.3f ", &my_gps.heading, BASE_FLOAT, REPLAY_USE},                                    //###115
	{"%6.2f ", &my_gps.altitude, BASE_FLOAT, REPLAY_USE},                                   //###116
	{"%d ", &my_gps.num_sats, BASE_INT, REPLAY_USE},                                        //###117
	{"%d ", &my_gps.pos, BASE_INT, REPLAY_USE},                                             //###118
	{"%6.3f ", &my_gps.hdop, BASE_FLOAT, REPLAY_USE},                                       //###119
	{"%hhu ", &m56_ignition_status.ignition_status, BASE_CHAR, REPLAY_USE},                 //###120
	{"%hhu ", &m56_gear.current_gear, BASE_CHAR, REPLAY_USE},                 		//###121
	{"%hhu ", &m56_gear.next_gear, BASE_CHAR, REPLAY_USE},                 			//###122
	{"%hhu ", &m56_atcvt.virtual_gear, BASE_CHAR, REPLAY_USE},                 		//###123
	{"%hhu ", &m56_pedal_position.temporary_accel_flag, BASE_CHAR, REPLAY_USE},		//###124
	{"%hhu ", &m56_pedal_position.acc_control_requested, BASE_CHAR, REPLAY_USE},            //###125
	{"%hhu ", &das3_ignition_status.ignition_status, BASE_CHAR, REPLAY_USE},                 //###120
};

#define NUM_DFILE_COLUMNS sizeof(file_specd)/sizeof(data_log_column_spec_t)
int num_dfile_columns = sizeof(file_specd)/sizeof(data_log_column_spec_t);



/** This array is used to specify the output format of the "[mnop]c" file.
 */
data_log_column_spec_t file_speca[] = 
{
//m56_m200_t
        {"%u ",   &m56_m200.message_timeout_counter, BASE_INT, REPLAY_USE},	//###1
        {"%hhu ",   &m56_m200.hosttx_msgcnt_id200, BASE_CHAR, REPLAY_USE},	//###2

        //my current speed
        {"%.1f ",   &m56_m200.hosttx_curr_spd_vsp_491, BASE_FLOAT, REPLAY_USE},	//###3

        //my target speed, no cars ahead
        {"%.1f ",   &m56_m200.hosttx_target_spd_vspcomn_491, BASE_FLOAT, REPLAY_USE},	//###4

        //my target speed, distance ahead controlled because of cars ahead
        {"%.1f ",   &m56_m200.hosttx_dist_ctl_target_spd_acvcomo_491, BASE_FLOAT, REPLAY_USE},	//###5

        //my acceleration
        {"%.2f ",   &m56_m200.hosttx_acceleration_xg_491, BASE_FLOAT, REPLAY_USE},	//###6
//m56_m201_t
        {"%hhu ",   &m56_m201.hosttx_msgcnt_id201, BASE_CHAR, REPLAY_USE},	//###7

        //distance to previous car
        {"%.1f ",   &m56_m201.hosttx_prev_spd_mdistance_491, BASE_FLOAT, REPLAY_USE},	//###8

        //relative speed to previous car
        {"%.1f ",   &m56_m201.hosttx_relative_spd_vr_491, BASE_FLOAT, REPLAY_USE},	//###9

        //Driver brake status
        {"%hhu ",   &m56_m201.hosttx_brake_sw_491, BASE_CHAR, REPLAY_USE},	//###10

        //Accelerator switch status
        {"%hhu ",   &m56_m201.hosttx_accel_sw_491, BASE_CHAR, REPLAY_USE},	//###11

        //set speed
        {"%.1f ",   &m56_m201.hosttx_setvsp_491, BASE_FLOAT, REPLAY_USE},	//###12
//m56_m202_t
        {"%u ",   &m56_m202.message_timeout_counter, BASE_INT, REPLAY_USE},	//###13
        {"%hhu ",   &m56_m202.hosttx_msgcnt_id202, BASE_INT, REPLAY_USE},	//###14
//m56_m203_t
        {"%u ",   &m56_m203.message_timeout_counter, BASE_INT, REPLAY_USE},	//###15
        {"%.2f ",   &m56_m203.yaw_rate, BASE_FLOAT, REPLAY_USE},	//###16
        {"%hhu ",   &m56_m203.couple_prec_veh, BASE_FLOAT, REPLAY_USE},	//###17
        {"%hhu ",   &m56_m203.couple_follow_veh, BASE_CHAR, REPLAY_USE},	//###18
//m56_m650_t
        {"%hhu ",   &m56_m650.hosttx_host_veh_ctr4, BASE_CHAR, REPLAY_USE},	//###19

        //GPS range to preceding vehicle
        {"%.1f ",   &m56_m650.hosttx_gps_range_prev_veh, BASE_FLOAT, REPLAY_USE},	//###20

        //Tracking status (0=idle, 1=detecting, 2=tracking, 3=coupling, 0xF=invalid)
        {"%hhu ",   &m56_m650.hosttx_tracking_status, BASE_CHAR, REPLAY_USE},	//###21

        // Vehicle position type (0=stand alone vehicle, 1=head of train,
        // 2=tail 0f train, 3=middle vehicle of train, 0xf=invalid)
        {"%hhu ",   &m56_m650.hosttx_veh_position_type, BASE_CHAR, REPLAY_USE},	//###22

        // Forward sequence number (0=head vehicle of train,
        // 0x01-0xFE=following vehicles, 0xFF=invalid)
        {"%hhu ",   &m56_m650.hosttx_veh_fwd_seq_num, BASE_CHAR, REPLAY_USE},	//###23

        // Backward sequence number (0=tail vehicle of train,
        // 0x01-0xFE=preceding vehicles, 0xFF=invalid)
        {"%hhu ",   &m56_m650.hosttx_veh_backwd_seq_num, BASE_CHAR, REPLAY_USE},	//###24
//m56_m651_t
        {"%u ",   &m56_m651.message_timeout_counter, BASE_INT, REPLAY_USE},	//###25
        {"%hhu ",   &m56_m651.hosttx_host_gps_ctr6, BASE_CHAR, REPLAY_USE},	//###26

        //GPS latitude
        {"%.6f ",   &m56_m651.hosttx_gps_latitude, BASE_FLOAT, REPLAY_USE},	//###27

        //GPS latitude accuracy
        {"%.1f ",   &m56_m651.hosttx_gps_lat_accuracy, BASE_FLOAT, REPLAY_USE},	//###28
//m56_m652_t
        {"%u ",   &m56_m652.message_timeout_counter, BASE_INT, REPLAY_USE},	//###29
        {"%hhu ",   &m56_m652.hosttx_host_gps_ctr7, BASE_CHAR, REPLAY_USE},	//###30

        //GPS longitude
        {"%.6f ",   &m56_m652.hosttx_gps_longitude, BASE_FLOAT, REPLAY_USE},	//###31

        //GPS longitude accuracy
        {"%.1f ",   &m56_m652.hosttx_gps_long_accuracy, BASE_FLOAT, REPLAY_USE},	//###32
//m56_m653_t
        {"%u ",   &m56_m653.message_timeout_counter, BASE_INT, REPLAY_USE},	//###33
        {"%hhu ",   &m56_m653.hosttx_host_gps_ctr8, BASE_CHAR, REPLAY_USE},	//###34

        //GPS altitude
        {"%.2f ",   &m56_m653.hosttx_gps_altitude, BASE_FLOAT, REPLAY_USE},	//###35

        //GPS altitude accuracy
        {"%.2f ",   &m56_m653.hosttx_gps_alt_accuracy, BASE_FLOAT, REPLAY_USE},	//###36
//m56_m654_t
        {"%u ",   &m56_m654.message_timeout_counter, BASE_INT, REPLAY_USE},	//###37
        {"%hhu ",   &m56_m654.hosttx_host_gps_ctr9, BASE_CHAR, REPLAY_USE},	//###38

        //GPS heading
        {"%.1f ",   &m56_m654.hosttx_gps_heading, BASE_FLOAT, REPLAY_USE},	//###39

        //GPS heading accuracy
        {"%.1f ",   &m56_m654.hosttx_gps_heading_accuracy, BASE_FLOAT, REPLAY_USE},	//###40

        //GPS speed
        {"%.1f ",   &m56_m654.hosttx_gps_speed, BASE_FLOAT, REPLAY_USE},	//###41
//m56_m655_t
        {"%u ",   &m56_m655.message_timeout_counter, BASE_INT, REPLAY_USE},	//###42
        {"%hhu ",   &m56_m655.hosttx_host_gps_ctr10, BASE_CHAR, REPLAY_USE},	//###43

        {"%hhu ",   &m56_m655.hosttx_gps_num_sat_in_use, BASE_CHAR, REPLAY_USE},	//###44
        {"%hhu ",   &m56_m655.hosttx_gps_num_sat_in_view, BASE_CHAR, REPLAY_USE},	//###45
        {"%.1f ",   &m56_m655.hosttx_gps_hdop, BASE_FLOAT, REPLAY_USE},	//###46
        {"%.1f ",   &m56_m655.hosttx_gps_pdop, BASE_FLOAT, REPLAY_USE},	//###47
        {"%hu ",   &m56_m655.hosttx_gps_fix_status, BASE_CHAR, REPLAY_USE},	//###48
//m56_m656_t
        {"%u ",   &m56_m656.message_timeout_counter, BASE_INT, REPLAY_USE},	//###49
        {"%hhu ",   &m56_m656.hosttx_host_gps_ctr11, BASE_CHAR, REPLAY_USE},	//###50

        {"%hu ",   &m56_m656.hosttx_gps_ts_year, BASE_SHORT, REPLAY_USE},	//###51
        {"%hhu ",   &m56_m656.hosttx_gps_ts_month, BASE_CHAR, REPLAY_USE},	//###52
        {"%hhu ",   &m56_m656.hosttx_gps_ts_day, BASE_CHAR, REPLAY_USE},	//###53
        {"%hhu ",   &m56_m656.hosttx_gps_ts_hour, BASE_CHAR, REPLAY_USE},	//###54
        {"%hhu ",   &m56_m656.hosttx_gps_ts_minute, BASE_CHAR, REPLAY_USE},	//###55
        {"%hhu ",   &m56_m656.hosttx_gps_ts_second, BASE_CHAR, REPLAY_USE},	//###56
//m56_m657_t
        {"%u ",   &m56_m657.message_timeout_counter, BASE_INT, REPLAY_USE},	//###57
        {"%hhu ",   &m56_m657.hosttx_host_gps_ctr12, BASE_CHAR, REPLAY_USE},	//###58

        {"%hu ",   &m56_m657.hosttx_gps_ts_millisecond, BASE_SHORT, REPLAY_USE},	//###59
//m56_m658_t
        {"%u ",   &m56_m658.message_timeout_counter, BASE_INT, REPLAY_USE},	//###60
        {"%hhu ",   &m56_m658.hosttx_host_gps_ctr13, BASE_CHAR, REPLAY_USE},	//###61
//m56_m710_t
        {"%u ",   &m56_m710.message_timeout_counter, BASE_INT, REPLAY_USE},	//###62
        {"%hhu ",   &m56_m710.keepalive_mabx2wsu_1, BASE_CHAR, REPLAY_USE},	//###63
        {"%hhu ",   &m56_m710.keepalive_mabx2wsu_2, BASE_CHAR, REPLAY_USE},	//###64
        {"%hhu ",   &m56_m710.keepalive_mabx2wsu_3, BASE_CHAR, REPLAY_USE},	//###65
        {"%hhu ",   &m56_m710.keepalive_mabx2wsu_4, BASE_CHAR, REPLAY_USE},	//###66
        {"%hhu ",   &m56_m710.keepalive_mabx2wsu_5, BASE_CHAR, REPLAY_USE},	//###67
        {"%hhu ",   &m56_m710.keepalive_mabx2wsu_6, BASE_CHAR, REPLAY_USE},	//###68
//m56_m711_t
        {"%u ",   &m56_m711.message_timeout_counter, BASE_INT, REPLAY_USE},	//###69
        {"%hhu ",   &m56_m711.keepalive_echo_wsu2mabx_1, BASE_CHAR, REPLAY_USE},	//###70
        {"%hhu ",   &m56_m711.keepalive_echo_wsu2mabx_2, BASE_CHAR, REPLAY_USE},	//###71
        {"%hhu ",   &m56_m711.keepalive_echo_wsu2mabx_3, BASE_CHAR, REPLAY_USE},	//###72
        {"%hhu ",   &m56_m711.keepalive_echo_wsu2mabx_4, BASE_CHAR, REPLAY_USE},	//###73
        {"%hhu ",   &m56_m711.keepalive_echo_wsu2mabx_5, BASE_CHAR, REPLAY_USE},	//###74
        {"%hhu ",   &m56_m711.keepalive_echo_wsu2mabx_6, BASE_CHAR, REPLAY_USE},	//###75
};

#define NUM_AFILE_COLUMNS sizeof(file_speca)/sizeof(data_log_column_spec_t)
int num_afile_columns = sizeof(file_speca)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]h" file.
 */
data_log_column_spec_t file_spech[] = 
{
};

int num_hfile_columns = sizeof(file_spech)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]i" file.
 */
data_log_column_spec_t file_speci[] = 
{
};

int num_ifile_columns = sizeof(file_speci)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]j" file.
 */
data_log_column_spec_t file_specj[] = 
{
};

int num_jfile_columns = sizeof(file_specj)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]k" file.
 */
data_log_column_spec_t file_speck[] = 
{
};

int num_kfile_columns = sizeof(file_speck)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]l" file.
 */
data_log_column_spec_t file_specl[] = 
{
};

int num_lfile_columns = sizeof(file_specl)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]m" file.
 */
data_log_column_spec_t file_specm[] = 
{
};

int num_mfile_columns = sizeof(file_specm)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]n" file.
 */
data_log_column_spec_t file_specn[] = 
{
};

int num_nfile_columns = sizeof(file_specn)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]o" file.
 */
data_log_column_spec_t file_speco[] = 
{
};

int num_ofile_columns = sizeof(file_speco)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]p" file.
 */
data_log_column_spec_t file_specp[] = 
{
};

int num_pfile_columns = sizeof(file_specp)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]q" file.
 */
data_log_column_spec_t file_specq[] = 
{
};

int num_qfile_columns = sizeof(file_specq)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]r" file.
 */
data_log_column_spec_t file_specr[] = 
{
};

int num_rfile_columns = sizeof(file_specr)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]s" file.
 */
data_log_column_spec_t file_specs[] = 
{
};

int num_sfile_columns = sizeof(file_specs)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]t" file.
 */
data_log_column_spec_t file_spect[] = 
{
};

int num_tfile_columns = sizeof(file_spect)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]u" file.
 */
data_log_column_spec_t file_specu[] = 
{
};

int num_ufile_columns = sizeof(file_specu)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]v" file.
 */
data_log_column_spec_t file_specv[] = 
{
};

int num_vfile_columns = sizeof(file_specv)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]w" file.
 */
data_log_column_spec_t file_specw[] = 
{
};

int num_wfile_columns = sizeof(file_specw)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]x" file.
 */
data_log_column_spec_t file_specx[] = 
{
};

int num_xfile_columns = sizeof(file_specx)/sizeof(data_log_column_spec_t);


/** This array is used to specify the output format of the "[mnop]y" file.
 */
data_log_column_spec_t file_specy[] = 
{
};

int num_yfile_columns = sizeof(file_specy)/sizeof(data_log_column_spec_t);

/** This array is used to specify the output format of the "[mnop]z" file.
 */
data_log_column_spec_t file_specz[] = 
{
};

int num_zfile_columns = sizeof(file_specz)/sizeof(data_log_column_spec_t);
