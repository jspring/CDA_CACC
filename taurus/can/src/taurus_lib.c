/*
** m56_lib.c - contains function definitions for m56_can.c
*/

#include <db_include.h>
#include <taurus_can.h>

//taurus_ignition_status_t taurus_ignition_status;


int print_accel_cmd(taurus_accel_cmd_t *taurus_accel_cmd) {
	printf("taurus_accel_cmd:  acceleration %.3f\n",
		taurus_accel_cmd->accel_cmd
	);
	return 0;
}

int print_wheel_speed(taurus_wheel_speed_t *taurus_wheel_speed) {
	printf("taurus_wheel_speed: front right %.2f front left %.2f rear right %.2f rear left %.2f\n",
		taurus_wheel_speed->wheel_speed_FR,
		taurus_wheel_speed->wheel_speed_FL,
		taurus_wheel_speed->wheel_speed_RR,
		taurus_wheel_speed->wheel_speed_RL
	);
	return 0;
}

int print_long_lat_accel(taurus_long_lat_accel_t *taurus_long_lat_accel) {
	printf("taurus_long_lat_accel:  longitudinal acceleration %.3f latitudinal acceleration %.3f\n",
		taurus_long_lat_accel->longitudinal_acceleration,
		taurus_long_lat_accel->lateral_acceleration
	);
	return 0;
}
