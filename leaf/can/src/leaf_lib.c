/*
** leaf_lib.c - contains function definitions for leaf_can.c
*/

#include <db_include.h>
#include <leaf_can.h>
#include <can_dbvars.h>

//leaf_ignition_status_t leaf_ignition_status;

int printcan(db_steinhoff_msg_t *db_steinhoff_msg){
	int i;

	printf("CAN ID: %#03lx msg: ", db_steinhoff_msg->id);
	for(i=0 ; i<8; i++)
		printf("%#02x ", db_steinhoff_msg->data[i]);
	printf("\n");
	return 0;
}

int print_accel_cmd(leaf_accel_cmd_t *leaf_accel_cmd) {
	printf("leaf_accel_cmd:  acceleration %.3f\n",
		leaf_accel_cmd->accel_cmd
	);
	return 0;
}

int print_vehicle_speed(leaf_vehicle_speed_t *leaf_vehicle_speed) {
	printf("leaf_vehicle_speed:  vehicle speed %.3f\n",
		leaf_vehicle_speed->vehicle_speed_CAN2_MPS
	);
	return 0;
}
