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


asn_enc_rval_t vehcomm2BSM(char * buffer, size_t buffer_size, veh_comm_packet_t *comm_pkt)
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
	timestamp_t ts;
	int mstime;
	BSMcoreData_t *core_data = &(bsm->coreData);
	core_data->msgCnt = comm_pkt->sequence_no;

	get_current_timestamp(&ts);
	mstime = TS_TO_MS(&ts);
	core_data->secMark = mstime%1000;
//	printf("secMark %d\n", core_data->secMark);

	core_data->Long = (long)(comm_pkt->longitude * LONG_LAT_MULT);
	core_data->lat = (long)(comm_pkt->latitude * LONG_LAT_MULT);
	core_data->heading = (long)(comm_pkt->heading * HEADING_MULT);
	core_data->speed = (long)(comm_pkt->velocity * VELOCITY_MULT);

//printf("veh_lib: Long %ld comm_pkt->longitude %f lat %ld comm_pkt->latitude %f heading %ld comm_pkt->heading %f speed %ld comm_pkt->velocity %f\n ",
//		core_data->Long ,
//		comm_pkt->longitude ,
//		core_data->lat ,
//		comm_pkt->latitude ,
//		core_data->heading ,
//		comm_pkt->heading ,
//		core_data->speed ,
//		comm_pkt->velocity
//		);
	AccelerationSet4Way_t *acc_set = &(core_data->accelSet);
	acc_set->Long = (int)(comm_pkt->accel * ACCEL_MULT); //Current acceleration (m/s^2) "LSB bit 0.01 m/s^2"
	acc_set->lat = 0;
	acc_set->vert = 0;
	acc_set->yaw = 0;

	// Temporary ID
	OCTET_STRING_t *id_octet_str = OCTET_STRING_new_fromBuf(&asn_DEF_TemporaryID, comm_pkt->object_id, 4);
//	core_data->id = *id_octet_str;

	PositionalAccuracy_t *pos_acu = &(core_data->accuracy);
	pos_acu->orientation = UNSET;
	pos_acu->semiMajor = UNSET;
	pos_acu->semiMinor = UNSET;

	BrakeSystemStatus_t *brake_status = &(core_data->brakes);
	brake_status->abs = AntiLockBrakeStatus_unavailable;
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
	cacc_data->setSpeed = UNSET;
	cacc_data->throtPos = UNSET;
	cacc_data->grpID = UNSET;
	cacc_data->grpSize = (long)(comm_pkt->pltn_size);
	cacc_data->grpMode = UNSET;
	cacc_data->grpManDes = UNSET;
	cacc_data->grpManID = UNSET;
	cacc_data->vehID = *oct_str;
	cacc_data->frntCutIn = UNSET;
	cacc_data->vehGrpPos = (int)(comm_pkt->my_pip);
	cacc_data->vehFltMode = (long)(comm_pkt->fault_mode);
	cacc_data->vehManDes = (long)(comm_pkt->maneuver_des_1); 
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
	cacc_data->desTrq = UNSET;
	cacc_data->desAcc = (long)(comm_pkt->acc_traj * BSM_FLOAT_MULT);
	cacc_data->userDI1 = comm_pkt->user_ushort_1;
	cacc_data->userDI2 = comm_pkt->user_ushort_2;
	cacc_data->userDF1 = (long)(comm_pkt->user_float * BSM_FLOAT_MULT);
	cacc_data->userDF2 = (long)(comm_pkt->user_float1 * BSM_FLOAT_MULT);;
	cacc_data->userDF3 = UNSET;
//	printf("desSpd %ld  comm_pkt->vel_traj %f desAcc %ld comm_pkt->acc_traj %f\n",
//			cacc_data->desSpd,
//			comm_pkt->vel_traj,
//			cacc_data->desAcc,
//			comm_pkt->acc_traj
//			);
	
	DDateTime_t *temp_time = (DDateTime_t *)calloc(1, sizeof(DDateTime_t));
	
	temp_time->day = (long *)calloc(1, sizeof(long));
	temp_time->month = (long *)calloc(1, sizeof(long));
	temp_time->hour = (long *)calloc(1, sizeof(long));
	*temp_time->hour = (long) comm_pkt->ts.hour;
	temp_time->minute = (long *)calloc(1, sizeof(long));
	*temp_time->minute = (long) comm_pkt->ts.min;
	temp_time->second = (long *)calloc(1, sizeof(long));
	*temp_time->second = (long)(comm_pkt->ts.sec*1000)+(comm_pkt->ts.millisec);
	cacc_data->utcTime = *temp_time;
	
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
	cacc_data->globalTime = (long)(comm_pkt->global_time * 50); // From long_ctl or trk_comm_mgr
	
	cacc_data->userBit1 = comm_pkt->user_bit_1;
	cacc_data->userBit2 = comm_pkt->user_bit_2;
	cacc_data->userBit3 = comm_pkt->user_bit_3;
	cacc_data->userBit4 = comm_pkt->user_bit_4;
	
//printf("vehcomm2BSM: core_data->msgCnt %d\n", core_data->msgCnt);
	// encode data
	enc_rval = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, BSM_CACC, (void *) buffer, buffer_size);

	// free all alocated memory
	ASN_STRUCT_FREE(asn_DEF_DDateTime, temp_time);
	ASN_STRUCT_FREE(asn_DEF_CACCFlags, oct_str); 
	free(BSM_CACC->value.choice.BasicSafetyMessage.caccData);
	free(BSM_CACC);
	
	return enc_rval;
}

int BSM2vehcomm(MessageFrame_t *BSMCACC, veh_comm_packet_t *comm_pkt)
{
	// check message type
	if (BSMCACC->messageId == 20 && BSMCACC->value.present == MessageFrame__value_PR_BasicSafetyMessage) {
		
		//printf("veh_lib: Found BSM safty message\n");
		
		BSMcoreData_t *core_data = &(BSMCACC->value.choice.BasicSafetyMessage.coreData);
		CaccData_t *cacc_data = BSMCACC->value.choice.BasicSafetyMessage.caccData;
		
		timestamp_t current_ts;
		get_current_timestamp(&(comm_pkt->rcv_ts));
		comm_pkt->ts.hour = (char) *cacc_data->utcTime.hour;
		comm_pkt->ts.min = (char) *cacc_data->utcTime.minute;
		comm_pkt->ts.sec = (char) ((*cacc_data->utcTime.second)/1000.0);
		comm_pkt->ts.millisec = (short) ((*cacc_data->utcTime.second)%1000);
		comm_pkt->node = UNSET;
		comm_pkt->user_float = cacc_data->userDF1/BSM_FLOAT_MULT;
		comm_pkt->user_float1 = cacc_data->userDF2/BSM_FLOAT_MULT;
		comm_pkt->user_ushort_1 = cacc_data->userDI1;
		comm_pkt->user_ushort_2 = cacc_data->userDI2;
		comm_pkt->my_pip = cacc_data->vehGrpPos;
		comm_pkt->maneuver_id = cacc_data->vehManID;
		comm_pkt->fault_mode = cacc_data->vehFltMode;
		comm_pkt->maneuver_des_1 = cacc_data->vehManDes;
		comm_pkt->pltn_size = (unsigned char)cacc_data->grpSize;
		comm_pkt->sequence_no = core_data->msgCnt;
		comm_pkt->user_bit_1 = cacc_data->userBit1;
		comm_pkt->user_bit_2 = cacc_data->userBit2;
		comm_pkt->user_bit_3 = cacc_data->userBit3;
		comm_pkt->user_bit_4 = cacc_data->userBit4;
		comm_pkt->acc_traj = (float)(cacc_data->desAcc / BSM_FLOAT_MULT);
		comm_pkt->vel_traj = (float)(cacc_data->desSpd / BSM_FLOAT_MULT);
		comm_pkt->velocity = (float)core_data->speed / VELOCITY_MULT;
		comm_pkt->accel = (float)core_data->accelSet.Long / ACCEL_MULT;
		comm_pkt->range = (float)(cacc_data->distToPVeh / BSM_FLOAT_MULT);
		comm_pkt->rate = (float) (cacc_data->relSpdPVeh / BSM_FLOAT_MULT);
		comm_pkt->longitude = (double)(core_data->Long / LONG_LAT_MULT);	
		comm_pkt->latitude = (double)(core_data->lat / LONG_LAT_MULT);
		comm_pkt->heading = (float)(core_data->heading / HEADING_MULT);
//		strcpy(&comm_pkt->object_id[0], core_data->id.buf);
//		printf("BSM2vehcomm: comm_pkt->object_id %s\n", comm_pkt->object_id);
//		if(strcmp(core_data->id.buf, "VirC") == 0) {
//			comm_pkt->my_pip = 0; 			//Position-in-platoon for virtual car
//			comm_pkt->user_float = core_data->lat * 0.0001; //Distance from beginning of track
//			comm_pkt->accel = core_data->Long * 0.01;     //Acceleration of virtural car
//			comm_pkt->range = core_data->heading * 0.01;    //Distance from virtual car (m)
//			comm_pkt->rate = core_data->elev * 0.001;        //Relative velocity of virtual car(m/s)
//		}
  
		
	} else {		
		printf("veh_lib: Received message type error\n");
		return -1;
	}
	return 0; 
}

int print_comm_packet(veh_comm_packet_t *comm_pkt)
{
	printf("my_pip %d sequence number %d \nlatitude %lf \nuser_float %lf\nlongitude %lf \ncar accel %f \ncar range %f \ncar rate %f \nglobal time %f\n",
		comm_pkt->my_pip,
		comm_pkt->sequence_no,
		comm_pkt->latitude,
		comm_pkt->user_float,
		comm_pkt->longitude,
		comm_pkt->accel,
		comm_pkt->range,
		comm_pkt->rate,
		comm_pkt->global_time 
	);

	return 0;
}

void print_comm_packet_time(veh_comm_packet_t *comm_pkt)
{
	printf("Hours: %d\n", comm_pkt->ts.hour);
	printf("Mins %d\n", comm_pkt->ts.min);
	printf("Secs %d\n", comm_pkt->ts.sec);
	printf("Msecs %d\n", comm_pkt->ts.millisec);
}
