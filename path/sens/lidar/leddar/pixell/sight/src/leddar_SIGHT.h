#pragma once

#include <can_dbvars.h>

#define DB_LEDDAR_SIGHT_DETECTION_TYPE	3500
#define DB_LEDDAR_SIGHT_DETECTION_VAR	DB_LEDDAR_SIGHT_DETECTION_TYPE

extern unsigned char Compute_CRC8(int size, unsigned char *bytes);

/********************************************************************************************************
*********************************************************************************************************
*	DBW_FaultText
*	Identifier: 0x070F
*
*/

//typedef struct {
//	char page_no;
//	char ascii_char[7];
//}dbw_fault_text_t;

/********************************************************************************************************
*********************************************************************************************************
*	DBW_Misc
*	Identifier: 0x1F01
*	
0.0 MiscByWireEnbl 	1	0=manual, 1=DriveByWireMode
0.3 MiscFault		1	1=fault
0.4 MiscAkitCommFault	1	1=fault
0.5 MiscByWireReady	1	1=no fault
0.6 MiscDriverActivity	1	1=fault
0.7 DBW_VehReadyToDrive	1	1=fault
1.0 MiscVehicleSpeed	16	m/s, 0.00217014 m/s/bit
3.0 MiscFuelLvl		8	0-100, 0.5%
4.0 SoftwareBuildNumber 16
6.0 AmbientTemp		7	deg C, -40 offset

 * Intel ordering
 *  
 * 63 62 61 60 59 58 57 56   55 54 53 52 51 50 49 48   47 46 45 44 43 42 41 40   39 38 37 36 35 34 33 32
 * 31 30 29 28 27 26 25 24  23 22 21 20 19 18 17 16  15 14 13 12 11 10 9 8   7 6 5 4 3 2 1 0

*********************************************************************************************************
********************************************************************************************************/

typedef struct {
	int ts_ms;
	unsigned char two_message_periods;
	unsigned int message_timeout_counter;

	unsigned char num_detections;
	float distance1;
	float amplitude1;
	unsigned char channel1;
	float distance2;
	float amplitude2;
	unsigned char channel2;
} leddar_SIGHT_detection_t;

static inline void parse_SIGHT_detections(unsigned char *data, leddar_SIGHT_detection_t *p, int veryverbose) {

	timestamp_t ts;
	short short_temp;
	int i;

	short_temp = ((data[1] << 8) & 0xFF00)+ (data[0] & 0xFF);
	p->distance1 = (float)(short_temp / 100.0);

	short_temp = ((data[3] << 8) & 0x0F00)+ (data[2] & 0xFF);
	p->amplitude1 = (float)(short_temp / 4.0);

	p->channel1 = ((data[3] >> 4 ) & 0x0F);

	short_temp = ((data[5] << 8) & 0xFF00)+ (data[4] & 0xFF);
	p->distance2 = (float)(short_temp / 100.0);

	short_temp = ((data[7] << 8) & 0x0F00)+ (data[6] & 0xFF);
	p->amplitude2 = (float)(short_temp / 4.0);

	p->channel2 = ((data[7] >> 4 ) & 0x0F);

	if(veryverbose) {
		get_current_timestamp(&ts);
		print_timestamp(stdout, &ts);
 		for(i=0; i<8 ; i++)
 			printf("%#hhx ", data[i]);
 		printf("\n");
		printf("library parse_SIGHT_detections: distance1 %.2f amplitude1 %.2f channel1 %hhu distance2 %.2f amplitude2 %.2f channel2 %hhu\n",
			p->distance1,
			p->amplitude1,
			p->channel1,
			p->distance2,
			p->amplitude2,
			p->channel2
		);
	}
}
//Holding Data
//Type (Byte 1)
//0
//1
//Holding Data Description Argument
//Byte 2 Exponent for the number of accumulations
//(that is, if the content of this register is n, 2 n
//accumulations are performed)
//Byte 3 Exponent for the number of oversamplings
//(that is, if the content of this register is n, 2 n
//oversamplings are performed)
//Byte 4 Number of base samples
//Acquisition configuration
//Reserved
//--
//Byte 4
//Byte 5
//2
//Detection threshold
//Byte 6
//Byte 7
//3
//Laser power percent (%)
//Byte 2
//Byte 2
//4
//Acquisition option
//Byte 3
//5
//54A0058_V1.0_EN / Leddar Sight – User Guide
//Argument Description
//--
//Detection threshold as a fixed-point value with
//a 19-bit fractional part (that is, the threshold
//value is this register divided by 524 288)
//The range is limited to a maximum of 100.0
//and to a variable minimum determined by the
//accumulations and oversamplings (read back
//the register to know the actual value).
//Laser power as a percentage of maximum
//A value above 100 is an error. Only the laser
//intensity values defined in section 3.3 should
//be used. If a value is specified that is not one
//of the predefined values, the closest
//predefined value will be used. The register
//can be read back to know the actual value
//set. Note that this value is ignored if the
//automatic laser intensity is enabled.
//Bit field of acquisition options with 4 bits
//currently defined (all others are reserved):
//Bit 0: Automatic laser intensity enabled
//Bit 2: Object demerging enabled
//Bit 3: Crosstalk removal disabled (disabled
//if 1)
//Bit 8: Automatic laser intensity mode:
//0 = Mode 1
//1 = Mode 2
//Bit 9: Static detection threshold table usage
//disabled
//Bit 10: Interference algorithm enabled
//Byte 2
//45 | 88Holding Data
//Type (Byte 1)
//Holding Data Description Argument
//Auto-acquisition average
//frames
//6
//Smoothing
//Byte 3 Change delay in the number of
//measurements
//Byte 2 Smoothing: used to stabilize the sensor
//measurements.
//The behavior of the smoothing algorithm can
//be adjusted by a value ranging from −16 to
//16. Select the Disabled check box or set the
//value to −17 to disable smoothing.
//Byte 2
//7
//Distance units
//Byte 3
//8
//9
//Communication segment
//enabled
//Argument Description
//Byte 2
//Byte 3
//Distance units:
//mm = 1 000
//cm = 100
//dm = 10
//m = 1
//Bit field of the activated segment for
//communication
//Byte 2 Baud rate (kbps):
//0 = 1 000
//1 = 500
//2 = 250
//3 = 125
//4 = 100
//5 = 50
//6 = 20
//7 = 10
//Byte 3 Frame format:
//0 = Standard 11 bits
//1 = Extended 29 bits
//CAN port configuration 1
//Byte 4
//Byte 5
//Byte 6
//Tx base ID
//Byte 7
//Byte 4
//10
//CAN port configuration 2
//Byte 5
//Byte 6
//Rx base ID
//Byte 7
//11
//46 | 88
//CAN port configuration 3
//Byte 2
//CAN operation mode bit field:
//Bit 0: 0 = return detection in single
//message mode, 1 = return detection
//in multiple message mode
//Bit 1: Inter-message delay activation
//© 2021 LeddarTech Inc.Holding Data
//Type (Byte 1)
//Holding Data Description Argument
//Argument Description
//Bit 2:
//Bit 3:
//Byte 3
//Byte 4
//Byte 5
//Byte 6
//Byte 7
//Byte 2
//12
//Test mode
//Byte 3
//13 Reserved
//14 Acquisition segment pair
//enabled
//Inter-cycle delay activation
//Detection flag message activation
//Maximum number of detections
//(measurements) returned per CAN detection
//message transaction: 1 to 96
//Inter-message delay, 0 to 65 535 milliseconds
//Inter-cycle delay, 0 to 65 535 milliseconds
//Bit field for test mode (if the fail-safe product
//option is available):
//Bit 0: Test mode activation (0 = OFF, 1 =
//ON), volatile setting to default OFF
//state
//--
//Byte 2
//Byte 3
//--
//Bit field of segment pair enabled for
//acquisition
//The SET command execution will fail if this sensor is USB connected to a host devic
