/*
** m56_lib.c - contains function definitions for m56_can.c
*/

#include <db_include.h>
#include <camry_can.h>

//camry_ignition_status_t camry_ignition_status;

//int printmsg(db_komodo_t *db_kom) {
//	int i;
//}

//int printcan(db_komodo_t *db_kom){
//	int i;
//
//	printf("CAN ID: %#03x msg: ", db_kom->id);
//	for(i=0 ; i<8; i++)
//		printf("%#02x ", db_kom->msg[i]);
//	printf("\n");
//return 0;
//}

int print_accel_cmd_camry(camry_accel_cmd_t *camry_accel_cmd) {
	printf("camry_accel_cmd:  acceleration %.3f\n",
		camry_accel_cmd->accel_cmd
	);
}

int print_wheel_speed_camry(camry_wheel_speed_t *camry_wheel_speed) {
	printf("camry_wheel_speed: front right %.2f front left %.2f rear right %.2f rear left %.2f\n",
		camry_wheel_speed->veh_wheel_spd_FR_CAN1_mps,
		camry_wheel_speed->veh_wheel_spd_FL_CAN1_mps,
		camry_wheel_speed->veh_wheel_spd_RR_CAN1_mps,
		camry_wheel_speed->veh_wheel_spd_RL_CAN1_mps
	);
}

int print_vehicle_speed_camry(camry_vehicle_speed_t *camry_vehicle_speed) {
	printf("camry_vehicle_speed:  vehicle speed %.3f\n",
		camry_vehicle_speed->veh_spd_CAN1_kph
	);
}

int print_long_lat_accel_camry(camry_long_lat_accel_t *camry_long_lat_accel) {
	printf("camry_long_lat_accel:  longitudinal acceleration %.3f latitudinal acceleration %.3f\n",
		camry_long_lat_accel->long_accel,
		camry_long_lat_accel->lat_accel
	);
}

int print_accel_cmd_status_camry(camry_accel_cmd_status_t *camry_accel_cmd_status) {
	printf("camry_accel_cmd_status:  set_me_x3  %#hhx distance %#hhx mini_car %#hhx set_me_x01 %#hhx cancel_req %#hhx set_me_1 %#hhx release_standstill %#hhx checksum %#hhx checksum_check %#hhx\n",
		camry_accel_cmd_status->set_me_x3,
		camry_accel_cmd_status->distance,
		camry_accel_cmd_status->mini_car,
		camry_accel_cmd_status->set_me_x01,
		camry_accel_cmd_status->cancel_req,
		camry_accel_cmd_status->set_me_1,
		camry_accel_cmd_status->release_standstill,
		camry_accel_cmd_status->checksum,
		camry_accel_cmd_status->checksum_check
	);
}

int print_cruise_control_state_camry(camry_cruise_control_state_t *camry_cruise_control_state) {
	printf("camry_cruise_control_state:  cruise_main_on_CAN1 %#hhx cruise_control_state_CAN1 %#hhx cruise_dash_set_speed_CAN1 %#hhx\n",
		camry_cruise_control_state->cruise_main_on_CAN1,
		camry_cruise_control_state->cruise_control_state_CAN1,
		camry_cruise_control_state->cruise_dash_set_speed_CAN1
	);
}
