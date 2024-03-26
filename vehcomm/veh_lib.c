/**\file
 *	veh_lib.c 
 *		Library for interconversion of old "comm_packet_t" with J2735 "BSMCACC_t"
 *
 * Copyright (c) 2015   Regents of the University of California
 *
 */
#include <sys_os.h>
#include <db_clt.h>
#include <db_utils.h>
#include <timestamp.h>
#include <local.h>
#include <sys_rt.h>
#include <sys_ini.h>
#include "path_gps_lib.h"
#include "long_comm.h"


#include "asn_application.h"
#include "asn_internal.h" /* for _ASN_DEFAULT_STACK_MAX */
#include "asn_SEQUENCE_OF.h"
#include "MessageFrame.h"
#include "BasicSafetyMessage.h"

#include "veh_lib.h"

#define UNSET 0


BIT_STRING_t Get_Bit_String(int size, char *s, int bit_unused)
{
	int i, j, p, w, c, h, byteSize;

	if (size < 8)
	{
		byteSize = 1;
	}
	else
	{
		h = size % 8;
		byteSize = h > 0 ? (size / 8) + 1 : size / 8;
	}

	BIT_STRING_t t;
	t.buf = (uint8_t *)calloc(1, sizeof(uint8_t));
	assert(t.buf);
	t.size = size;
	t.bits_unused = bit_unused;
	//printf("getBitString bytes=%d bits=%d <%s>\n", byteSize, size, s);
	for (p = 7, c = 0, w = 0, i = 0, j = strlen(s); i < j; i++, c++)
	{
		if (c > 7)
		{
			p = p + 8;
			c = 1;
			w++;
		}
		if (s[i] == '1')
		{
			t.buf[w] |= 1 << (7 - c);
		}
	}

	return (t);
}


asn_enc_rval_t vehcomm2BSM(char * buffer, size_t buffer_size, veh_comm_packet_t *comm_pkt, int verbose)
{
	asn_enc_rval_t enc_rval;
	// allocate memory to MessageFrame_t
	MessageFrame_t *BSM_CACC = (MessageFrame_t *) calloc(1, sizeof(MessageFrame_t));

	/** 
	 * fill in all needed data
	**/
	BSM_CACC->messageId = 20;
	BSM_CACC->value.present = MessageFrame__value_PR_BasicSafetyMessage;
	
	BasicSafetyMessage_t *bsm = &(BSM_CACC->value.choice.BasicSafetyMessage);
	
	/** 
	 * fill in BSM core data
	**/
	BSMcoreData_t *core_data = &(bsm->coreData);
	core_data->msgCnt = comm_pkt->sequence_no;
	core_data->secMark = comm_pkt->ts.millisec;
	core_data->Long = (long)(comm_pkt->longitude * LONG_LAT_MULT);
	core_data->lat = (long)(comm_pkt->latitude * LONG_LAT_MULT);
	core_data->heading = (long)(comm_pkt->heading * HEADING_MULT);
	core_data->speed = (long)(comm_pkt->velocity * VELOCITY_MULT);

	AccelerationSet4Way_t *acc_set = &(core_data->accelSet);
	acc_set->Long = (long)(comm_pkt->accel * ACCEL_MULT); //Current acceleration (m/s^2) "LSB bit 0.01 m/s^2"
	acc_set->lat = 0;
	acc_set->vert = 0;
	acc_set->yaw = 0;

//	// Temporary ID
	OCTET_STRING_t *id_octet_str = OCTET_STRING_new_fromBuf(&asn_DEF_TemporaryID, comm_pkt->object_id, 4);
	core_data->id = *id_octet_str;
//	core_data->id.buf = comm_pkt->object_id;
	PositionalAccuracy_t *pos_acu = &(core_data->accuracy);
	pos_acu->orientation = UNSET;
	pos_acu->semiMajor = UNSET;
	pos_acu->semiMinor = UNSET;

	BrakeSystemStatus_t *brake_status = &(core_data->brakes);
	if (comm_pkt->brake_switch != 0)
		brake_status->abs = AntiLockBrakeStatus_engaged;
	else
	brake_status->abs = AntiLockBrakeStatus_off;
	brake_status->auxBrakes = AuxiliaryBrakeStatus_unavailable;
	brake_status->brakeBoost = BrakeBoostApplied_unavailable;
	brake_status->scs = StabilityControlStatus_unavailable;
	brake_status->traction = TractionControlStatus_unavailable;
	brake_status->wheelBrakes = Get_Bit_String(1, "00000", 3);

	VehicleSize_t *veh_size = &(core_data->size);
	veh_size->length = 600;
	veh_size->width = 300;

	/** 
	 * fill in BSM cacc data
	**/
	BSM_CACC->value.choice.BasicSafetyMessage.caccData = (CaccData_t *)calloc(1, sizeof(CaccData_t));
	CaccData_t *cacc_data = bsm->caccData;
	OCTET_STRING_t *oct_str = OCTET_STRING_new_fromBuf(&asn_DEF_CACCFlags, "0", 1); // UNSET
	cacc_data->caccFlags = *oct_str;
	cacc_data->egoFlags = *oct_str;
	cacc_data->setSpeed = UNSET;
	cacc_data->throtPos = UNSET;
	cacc_data->grpID = UNSET;
	cacc_data->grpSize = (long)(comm_pkt->pltn_size);
	cacc_data->grpMode = UNSET;
	cacc_data->grpManDes = UNSET;
	cacc_data->grpManID = UNSET;
	cacc_data->vehID = *oct_str;
	cacc_data->frntCutIn = UNSET;
	cacc_data->vehGrpPos = (long)(comm_pkt->my_pip);
	cacc_data->vehFltMode = (long)(comm_pkt->fault_mode);
	cacc_data->vehManID = (long)(comm_pkt->maneuver_id);
	cacc_data->distToPVeh = (long)(comm_pkt->range * BSM_FLOAT_MULT);
	cacc_data->relSpdPVeh = (long)(comm_pkt->rate * BSM_FLOAT_MULT);
	cacc_data->disToLVeh = UNSET;
	cacc_data->relSpdLVeh = UNSET;
	cacc_data->desTGapPVeh = UNSET;
	cacc_data->desTGapLVeh = UNSET;
	cacc_data->estDisPVeh = UNSET;
	cacc_data->estDisLVeh = UNSET;
	cacc_data->desSpd = (long)(comm_pkt->vel_traj * BSM_FLOAT_MULT);
	cacc_data->desTrq = (long)(comm_pkt->desired_torque * BSM_FLOAT_MULT);
	cacc_data->desTrq = UNSET;
	cacc_data->desAcc = (long)(comm_pkt->acc_traj * BSM_FLOAT_MULT);
	cacc_data->userDI1 = comm_pkt->user_ushort_1; //veh_id
	cacc_data->userDI2 = comm_pkt->user_ushort_2; //drive_mode
	cacc_data->userDF1 = (long)(comm_pkt->user_float * BSM_FLOAT_MULT);  //brake pressure
	cacc_data->userDF2 = (long)(comm_pkt->user_float1 * BSM_FLOAT_MULT); //engine_retarder_torque
	cacc_data->userDF3 = UNSET;
//	cacc_data->laneDepartWarnRt = comm_pkt->Lane_departure_warning_right;
//	cacc_data->laneDepartWarnLt = comm_pkt->Lane_departure_warning_left;
//
//	cacc_data->handShake = comm_pkt->handshake % 8;
//	cacc_data->handShake = UNSET;

	DDateTime_t *temp_time = (DDateTime_t *)calloc(1, sizeof(DDateTime_t));

	temp_time->day = (long *)calloc(1, sizeof(long));
	temp_time->month = (long *)calloc(1, sizeof(long));
	temp_time->hour = (long *)calloc(1, sizeof(long));
	*temp_time->hour = (long) comm_pkt->ts.hour;
	temp_time->minute = (long *)calloc(1, sizeof(long));
	*temp_time->minute = (long) comm_pkt->ts.min;
	temp_time->second = (long *)calloc(1, sizeof(long));
	*temp_time->second = (long)(comm_pkt->ts.sec);
	if( (comm_pkt->global_time < 0) || (comm_pkt->global_time > 86400)) {
		printf("global_time out of spec at %.3f seconds\n", comm_pkt->global_time);
		comm_pkt->global_time = 0.0;
	}
	cacc_data->utcTime = *temp_time;
		cacc_data->globalTime = (long)(comm_pkt->global_time * 50); // From long_ctl or trk_comm_mgr

	/**
	DDateTime_t temp_time;
	*temp_time.day = 10;
	*temp_time.month = 2;
	*temp_time.year = 2010;
	*temp_time.hour = (long) comm_pkt->ts.hour;
	*temp_time.minute = (long) comm_pkt->ts.min;
	*temp_time.second = (long)(comm_pkt->ts.sec*1000)+(comm_pkt->ts.millisec);
	cacc_data->utcTime = temp_time;
	**/
	
	cacc_data->userBit1 = comm_pkt->user_bit_1;
	cacc_data->userBit2 = comm_pkt->user_bit_2;
	cacc_data->userBit3 = comm_pkt->user_bit_3;
	cacc_data->userBit4 = comm_pkt->user_bit_4;
	
	if (verbose==1) {
		xer_fprint(stdout, &asn_DEF_MessageFrame, BSM_CACC);
		fflush(stdout);
	}
	// encode data
	enc_rval = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, BSM_CACC, (void *) buffer, buffer_size);
	if (enc_rval.encoded <= 0)
	{
		printf("vehcomm2BSM: ENCODING ERROR!!... encoded bytes=%d See following xer_fprint....\n", enc_rval.encoded);
		xer_fprint(stdout, &asn_DEF_MessageFrame, BSM_CACC);
		fflush(stdout);
	}

	// free all alocated memory
	ASN_STRUCT_FREE(asn_DEF_DDateTime, temp_time);
	ASN_STRUCT_FREE(asn_DEF_CACCFlags, oct_str); 
	free(BSM_CACC->value.choice.BasicSafetyMessage.caccData);
	free(BSM_CACC);
	
	return enc_rval;
}

int BSM2vehcomm(MessageFrame_t *BSMCACC, veh_comm_packet_t *comm_pkt)
{
	printf("messageID %d\n", BSMCACC->messageId);
	// check message type
	if (BSMCACC->messageId == 20 && BSMCACC->value.present == MessageFrame__value_PR_BasicSafetyMessage) {
		
		printf("veh_lib: Found BSM safety message\n");

		BSMcoreData_t *core_data = &(BSMCACC->value.choice.BasicSafetyMessage.coreData);
		CaccData_t *cacc_data = BSMCACC->value.choice.BasicSafetyMessage.caccData;

		timestamp_t current_ts;
		get_current_timestamp(&(comm_pkt->rcv_ts));
		comm_pkt->ts.hour = (char) *cacc_data->utcTime.hour;
		comm_pkt->ts.min = (char) *cacc_data->utcTime.minute;
		comm_pkt->ts.sec = (char) (*cacc_data->utcTime.second);
		comm_pkt->ts.millisec = (short) (core_data->secMark);
//		comm_pkt->global_time = cacc_data->globalTime / 50; // From long_ctl or trk_comm_mgr
//		printf("VEH_LIB: global_time after division by integer %.3f ", comm_pkt->global_time); // From long_ctl or trk_comm_mgr)
		comm_pkt->global_time = cacc_data->globalTime / 50.0; // From long_ctl or trk_comm_mgr
//		printf("and by float %.3f\n", comm_pkt->global_time); // From long_ctl or trk_comm_mgr)
		comm_pkt->node = UNSET;
//		comm_pkt->ebs_deceleration = UNSET;
		comm_pkt->user_float = cacc_data->userDF1/BSM_FLOAT_MULT;  //brake pressure
		comm_pkt->user_float1 = cacc_data->userDF2/BSM_FLOAT_MULT; //engine_retarder_torque
		comm_pkt->user_ushort_1 = cacc_data->userDI1; //veh_id
		comm_pkt->user_ushort_2 = cacc_data->userDI2; //drive_mode
		comm_pkt->my_pip = cacc_data->vehGrpPos;
		comm_pkt->maneuver_id = cacc_data->vehManID;
		comm_pkt->fault_mode = cacc_data->vehFltMode;
//		comm_pkt->pltn_size = (unsigned char)cacc_data->grpSize;
		comm_pkt->pltn_size = cacc_data->grpSize;
		comm_pkt->sequence_no = core_data->msgCnt;
		comm_pkt->user_bit_1 = cacc_data->userBit1;
		comm_pkt->user_bit_2 = cacc_data->userBit2;
		comm_pkt->user_bit_3 = cacc_data->userBit3;
		comm_pkt->user_bit_4 = cacc_data->userBit4;
		comm_pkt->acc_traj = (float)(cacc_data->desAcc / BSM_FLOAT_MULT);
		comm_pkt->vel_traj = (float)(cacc_data->desSpd / BSM_FLOAT_MULT);
		comm_pkt->velocity = (float)core_data->speed / VELOCITY_MULT;
		comm_pkt->desired_torque = (long)(cacc_data->desTrq / BSM_FLOAT_MULT);
		comm_pkt->accel = (float)core_data->accelSet.Long / ACCEL_MULT;
		comm_pkt->range = (float)(cacc_data->distToPVeh / BSM_FLOAT_MULT);
		comm_pkt->rate = (float) (cacc_data->relSpdPVeh / BSM_FLOAT_MULT);
		comm_pkt->longitude = (double)(core_data->Long / LONG_LAT_MULT);	
		comm_pkt->latitude = (double)(core_data->lat / LONG_LAT_MULT);
		comm_pkt->heading = (float)(core_data->heading / HEADING_MULT);
//		comm_pkt->Lane_departure_warning_right = cacc_data->laneDepartWarnRt;
//		comm_pkt->Lane_departure_warning_left = cacc_data->laneDepartWarnLt;
//		comm_pkt->handshake = cacc_data->handShake;

		if (core_data->brakes.abs == AntiLockBrakeStatus_engaged)
			comm_pkt->brake_switch = 1;
		else
			comm_pkt->brake_switch = 0;

		strcpy(&comm_pkt->object_id[0], core_data->id.buf);
//		comm_pkt->object_id =  core_data->id;
printf("VEH_LIB: vehicle %s\n", comm_pkt->object_id); // From long_ctl or trk_comm_mgr)

	} else {		
		printf("veh_lib: Received message type error\n");
		return -1;
	}
	return 0; 
}

int vehcomm2server(test_veh_data_t *test_veh_data, veh_comm_packet_t *comm_pkt, int verbose)
{
		// Time info (year, month, and day are already taken care of by veh_snd)
		test_veh_data->hour = (char)comm_pkt->ts.hour;
		test_veh_data->minute = (char)comm_pkt->ts.min;
		test_veh_data->second = (char)comm_pkt->ts.sec;
		test_veh_data->ms = (short)comm_pkt->ts.millisec;

		// Test vehicle info
		test_veh_data->ID = (short)comm_pkt->user_ushort_1;
		test_veh_data->leaderID = (short)comm_pkt->ts.min; // String leader ID
		test_veh_data->stringPos = (char)comm_pkt->my_pip;
		test_veh_data->route = (char)comm_pkt->ts.min; // 1: WB; 2: EB
		test_veh_data->v = (short)(comm_pkt->velocity * 1000); // 0.001 m/s
		test_veh_data->pos = (int)comm_pkt->user_float; // Position from the start, 0.001 m
		test_veh_data->lane = (char)1; //comm_pkt->ts.min;
		return 0;
}

int server_2_comm_pkt( server_2_test_vehicle_t *server_2_test_vehicle, veh_comm_packet_t *comm_pkt)
{
	// Time info
	comm_pkt->ts.hour = server_2_test_vehicle->hour;
	comm_pkt->ts.min = server_2_test_vehicle->minute;
	comm_pkt->ts.sec = server_2_test_vehicle->second;
	comm_pkt->ts.millisec = server_2_test_vehicle->ms;

	// Virtual leading vehicle
	comm_pkt->user_float = server_2_test_vehicle->pos / 1000.0; // Position from the start, 0.001 m
	comm_pkt->velocity = server_2_test_vehicle->v / 1000.0;// 0.001 m/s

	// Sim veh ID, used to determine if there is a virtual leading vehicle
//	printf("Virtual vehicle ID %d ", server_2_test_vehicle->simID);

	// Target test vehicle
//	printf("Target test vehicle ID %d ", server_2_test_vehicle->targetVehID);

	// Signal control
//	printf("Signal State %d ", server_2_test_vehicle->signalState);
//	printf("endtime %d ", server_2_test_vehicle->endTime);

	// TP Info
	comm_pkt->acc_traj = server_2_test_vehicle->refAcc / 1000.0 ; // 0.001 m/s2

	printf("Virtual vehicle time: %d:%d:%d.%d: v %.2f m/s pos %.3f ID %d target ID %d signal state %d endtime %d ms refAcc %.2f m/s^2 \n",
			server_2_test_vehicle->hour,
			server_2_test_vehicle->minute,
			server_2_test_vehicle->second,
			server_2_test_vehicle->ms,
			server_2_test_vehicle->v / 1000.0,
			server_2_test_vehicle->pos / 1000.0,
			server_2_test_vehicle->simID,
			server_2_test_vehicle->targetVehID,
			server_2_test_vehicle->signalState,
			server_2_test_vehicle->endTime,				// number of ms since 00:00:00 of the day
			server_2_test_vehicle->refAcc / 1000.0
			);
//	printf("messageID %d\n", BSMCACC->messageId);
//	// check message type
//	if (BSMCACC->messageId == 20 && BSMCACC->value.present == MessageFrame__value_PR_BasicSafetyMessage) {
//
//		printf("veh_lib: Found BSM safety message\n");
//
//		BSMcoreData_t *core_data = &(BSMCACC->value.choice.BasicSafetyMessage.coreData);
//		CaccData_t *cacc_data = BSMCACC->value.choice.BasicSafetyMessage.caccData;
//
//		timestamp_t current_ts;
//		get_current_timestamp(&(comm_pkt->rcv_ts));
//		comm_pkt->ts.hour = (char) *cacc_data->utcTime.hour;
//		comm_pkt->ts.min = (char) *cacc_data->utcTime.minute;
//		comm_pkt->ts.sec = (char) (*cacc_data->utcTime.second);
//		comm_pkt->ts.millisec = (short) (core_data->secMark);
////		comm_pkt->global_time = cacc_data->globalTime / 50; // From long_ctl or trk_comm_mgr
////		printf("VEH_LIB: global_time after division by integer %.3f ", comm_pkt->global_time); // From long_ctl or trk_comm_mgr)
//		comm_pkt->global_time = cacc_data->globalTime / 50.0; // From long_ctl or trk_comm_mgr
////		printf("and by float %.3f\n", comm_pkt->global_time); // From long_ctl or trk_comm_mgr)
//		comm_pkt->node = UNSET;
////		comm_pkt->ebs_deceleration = UNSET;
//		comm_pkt->user_float = cacc_data->userDF1/BSM_FLOAT_MULT;  //brake pressure
//		comm_pkt->user_float1 = cacc_data->userDF2/BSM_FLOAT_MULT; //engine_retarder_torque
//		comm_pkt->user_ushort_1 = cacc_data->userDI1; //veh_id
//		comm_pkt->user_ushort_2 = cacc_data->userDI2; //drive_mode
//		comm_pkt->my_pip = cacc_data->vehGrpPos;
//		comm_pkt->maneuver_id = cacc_data->vehManID;
//		comm_pkt->fault_mode = cacc_data->vehFltMode;
////		comm_pkt->pltn_size = (unsigned char)cacc_data->grpSize;
//		comm_pkt->pltn_size = cacc_data->grpSize;
//		comm_pkt->sequence_no = core_data->msgCnt;
//		comm_pkt->user_bit_1 = cacc_data->userBit1;
//		comm_pkt->user_bit_2 = cacc_data->userBit2;
//		comm_pkt->user_bit_3 = cacc_data->userBit3;
//		comm_pkt->user_bit_4 = cacc_data->userBit4;
//		comm_pkt->acc_traj = (float)(cacc_data->desAcc / BSM_FLOAT_MULT);
//		comm_pkt->vel_traj = (float)(cacc_data->desSpd / BSM_FLOAT_MULT);
//		comm_pkt->velocity = (float)core_data->speed / VELOCITY_MULT;
//		comm_pkt->desired_torque = (long)(cacc_data->desTrq / BSM_FLOAT_MULT);
//		comm_pkt->accel = (float)core_data->accelSet.Long / ACCEL_MULT;
//		comm_pkt->range = (float)(cacc_data->distToPVeh / BSM_FLOAT_MULT);
//		comm_pkt->rate = (float) (cacc_data->relSpdPVeh / BSM_FLOAT_MULT);
//		comm_pkt->longitude = (double)(core_data->Long / LONG_LAT_MULT);
//		comm_pkt->latitude = (double)(core_data->lat / LONG_LAT_MULT);
//		comm_pkt->heading = (float)(core_data->heading / HEADING_MULT);
////		comm_pkt->Lane_departure_warning_right = cacc_data->laneDepartWarnRt;
////		comm_pkt->Lane_departure_warning_left = cacc_data->laneDepartWarnLt;
////		comm_pkt->handshake = cacc_data->handShake;
//
////		if (core_data->brakes.abs == AntiLockBrakeStatus_engaged)
////			comm_pkt->brake = 1;
////		else
////			comm_pkt->brake = 0;
//
//		strcpy(&comm_pkt->object_id[0], core_data->id.buf);
////		comm_pkt->object_id =  core_data->id;
//printf("VEH_LIB: vehicle %d\n", comm_pkt->object_id); // From long_ctl or trk_comm_mgr)
//
//	} else {
//		printf("veh_lib: Received message type error\n");
//		return -1;
//	}
	return 0;
}





int print_comm_packet(veh_comm_packet_t *comm_pkt)
{
	// printf("sequence number %d \nlatitude %lf \nlongitude %lf \ncar accel %f \ncar range %f \ncar rate %f \nglobal time %f\n",
	// 	comm_pkt->sequence_no,
	// 	comm_pkt->latitude,
	// 	comm_pkt->longitude,
	// 	comm_pkt->accel,
	// 	comm_pkt->range,
	// 	comm_pkt->rate,
	// 	comm_pkt->global_time 
	// );
	printf("object_id %s sequence number %d \naccel %.3f \nvelocity %.3f \nlatitude %lf \nlongitude %lf \n",
		comm_pkt->object_id,
		comm_pkt->sequence_no,
		comm_pkt->accel,
		comm_pkt->velocity,
		comm_pkt->latitude,
		comm_pkt->longitude);
	return 0;
}


void print_comm_packet_time(veh_comm_packet_t *comm_pkt)
{
	printf("%d:%d:%d.%d ", 
		comm_pkt->rcv_ts.hour,
		comm_pkt->rcv_ts.min,
		comm_pkt->rcv_ts.sec,
		comm_pkt->rcv_ts.millisec
	);
}
