#include <db_include.h>
#include <db_utils.h>
#include <path_gps_lib.h>
#include "data_log.h"
#include "m56_can.h"

m56_m4n0_t m56_m470;
m56_m4n1_t m56_m471;
m56_m4n2_t m56_m472;
m56_m4n3_t m56_m473;
m56_m4n4_t m56_m474;
m56_m4n5_t m56_m475;
m56_m4n6_t m56_m476;
m56_m4n7_t m56_m477;
m56_m4n8_t m56_m478;

m56_m4n0_t m56_m4C0;
m56_m4n1_t m56_m4C1;
m56_m4n2_t m56_m4C2;
m56_m4n3_t m56_m4C3;
m56_m4n4_t m56_m4C4;
m56_m4n5_t m56_m4C5;
m56_m4n6_t m56_m4C6;
m56_m4n7_t m56_m4C7;
m56_m4n8_t m56_m4C8;

timestamp_t timestamp;



/** This array is used to specify the output format of the "[mnop]c" file.
 */
data_log_column_spec_t comm_specg[] = 
{
        {"HH:MM:SS.SSS ", &timestamp, BASE_TIMESTAMP, REPLAY_TIME},                            //###1
//m56_m470_t
        {"%u ",   &m56_m470.message_timeout_counter, BASE_INT, REPLAY_USE},	//###2
        {"%hhu ",   &m56_m470.msgcnt_id200, BASE_CHAR, REPLAY_USE},		//###3

        //Current speed received from n cars ahead
        {"%.1f ",   &m56_m470.host_rx_vsp_491, BASE_FLOAT, REPLAY_USE},	//###4

        //Target speed received from n cars ahead, distance not controlled
        {"%.1f ",   &m56_m470.host_rx_vspcomn_491, BASE_FLOAT, REPLAY_USE},	//###5

        //Target speed received from n cars ahead, distance controlled
        {"%.1f ",   &m56_m470.host_rx_acvcomo_491, BASE_FLOAT, REPLAY_USE},	//###6

        //Current acceleration received from n cars ahead
        {"%.2f ",   &m56_m470.host_rx_xg_491, BASE_FLOAT, REPLAY_USE},		//###7
//m56_m471_t
        {"%u ",   &m56_m471.message_timeout_counter, BASE_INT, REPLAY_USE},	//###8
        {"%hhu ",   &m56_m471.msgcnt_id201, BASE_CHAR, REPLAY_USE},		//###9

        //Distance received from n cars ahead
        {"%.1f ",   &m56_m471.host_rx_mdistance_491, BASE_FLOAT, REPLAY_USE},	//###10

        //Speed received from n cars ahead
        {"%.1f ",   &m56_m471.host_rx_vr_491, BASE_FLOAT, REPLAY_USE},		//###11

        //Brake switch status received from n cars ahead
        {"%hhu ",   &m56_m471.host_rx_brake_sw_491, BASE_CHAR, REPLAY_USE},	//###12

        //Accelerator switch status received from n cars ahead
        {"%hhu ",   &m56_m471.host_rx_accel_sw_491, BASE_CHAR, REPLAY_USE},	//###13

        //Set speed received from n cars ahead
        {"%.1f ",   &m56_m471.host_rx_setvsp_491, BASE_FLOAT, REPLAY_USE},	//###14
//m56_m472_t
        {"%u ",   &m56_m472.message_timeout_counter, BASE_INT, REPLAY_USE},	//###15
        {"%hhu ",   &m56_m472.msgcnt_id202, BASE_CHAR, REPLAY_USE},		//###16
//m56_m473_t
        {"%u ",   &m56_m473.message_timeout_counter, BASE_INT, REPLAY_USE},	//###17
        {"%hhu ",   &m56_m473.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###18

        //GPS range to n cars ahead
        {"%.1f ",   &m56_m473.host_rx_gps_range, BASE_FLOAT, REPLAY_USE},	//###19

        //Tracking status received from n cars ahead
        //0=idle, 1=detecting, 2=tracking, 3=coupling, 0xF=invalid
        {"%hhu ",   &m56_m473.host_rx_gps_tracking_status, BASE_CHAR, REPLAY_USE},//###20

        //Vehicle position type received from n cars ahead
        //0=standalone vehicle, 1=head of train, 2=tail of train,
        //3=middle vehicle of train, 0xF=invalid
        {"%hhu ",   &m56_m473.host_rx_gps_veh_pos_type, BASE_CHAR, REPLAY_USE},	//###21

        //Forward sequence number received from n cars ahead
        //0=head vehicle of train, 1-0xFE=following vehicles,
        //0xFF=invalid
        {"%hhu ",   &m56_m473.host_rx_fwd_seq_num, BASE_CHAR, REPLAY_USE},	//###22

        //Backward sequence number received from n cars ahead
        //0=tail vehicle of train, 1-0xFE=preceding vehicles,
        //0xFF=invalid
        {"%hhu ",   &m56_m473.host_rx_backwd_seq_num, BASE_CHAR, REPLAY_USE},	//###23
//m56_m474_t
        {"%u ",   &m56_m474.message_timeout_counter, BASE_INT, REPLAY_USE},	//###24
        {"%hhu ",   &m56_m474.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###25

        //WSU ID from n cars ahead
        {"%hu ",   &m56_m474.host_rx_wsu_id, BASE_CHAR, REPLAY_USE},		//###26

        //Percent channel busy received from n cars ahead
        {"%hhu ",   &m56_m474.host_rx_chanbusy, BASE_CHAR, REPLAY_USE},		//###27

        //RSS in dBm received from n cars ahead
        {"%.1f ",   &m56_m474.host_rx_rss, BASE_FLOAT, REPLAY_USE},		//###28

        //RxPktPER% received from n cars ahead
        //I assume this is a percentage of packets received
        {"%.1f ",   &m56_m474.host_rx_pkt_pct, BASE_FLOAT, REPLAY_USE},	//###29
//m56_m475_t
        {"%u ",   &m56_m475.message_timeout_counter, BASE_INT, REPLAY_USE},	//###30
        {"%hhu ",   &m56_m475.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###31

        //GPS latitude received from n cars ahead
        {"%.6f ",   &m56_m475.host_rx_gps_latitude, BASE_FLOAT, REPLAY_USE},	//###32

        //GPS heading received from n cars ahead
        {"%.1f ",   &m56_m475.host_rx_gps_heading, BASE_FLOAT, REPLAY_USE},	//###33
//m56_m476_t
        {"%u ",   &m56_m476.message_timeout_counter, BASE_INT, REPLAY_USE},	//###34
        {"%hhu ",   &m56_m476.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###35

        //GPS longitude received from n cars ahead
        {"%.6f ",   &m56_m476.host_rx_gps_longitude, BASE_FLOAT, REPLAY_USE},	//###36

        //GPS speed received from n cars ahead
        {"%.1f ",   &m56_m476.host_rx_gps_speed, BASE_FLOAT, REPLAY_USE},	//###37
//m56_m477_t
        {"%u ",   &m56_m477.message_timeout_counter, BASE_INT, REPLAY_USE},	//###38
        {"%hhu ",   &m56_m477.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###39

        //GPS altitude received from n cars ahead
        {"%.2f ",   &m56_m477.host_rx_gps_altitude, BASE_FLOAT, REPLAY_USE},	//###40
//m56_m478_t
        {"%u ",   &m56_m478.message_timeout_counter, BASE_INT, REPLAY_USE},	//###41
        {"%hhu ",   &m56_m478.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###42

        //GPS timestamp milliseconds, received from n cars ahead
        {"%hu ",   &m56_m478.host_rx_gps_ts_millisecond, BASE_SHORT, REPLAY_USE},//###43

        //GPS range to host vehicle received from n cars ahead
        {"%.1f ",   &m56_m478.host_rx_gps_range2host, BASE_FLOAT, REPLAY_USE},	//###44

        //Relative position to host vehicle received from n cars ahead
        //0x00=unclassified, 0x01=ahead, 0x02=behind, 0x03=oncoming,
        //0x04=ahead left, 0x05=ahead right, 0x06=behind left,
        //0x07=behind right, 0x08=oncoming left, 0x09=oncoming right,
        //0x0A=ahead far left, 0X0B=ahead far right, 0x0C=behind far left,
        //0x0D=behind far right, 0x0E=oncoming far left,
        //0x0F=oncoming far right, 0x10=intersecting left
        //0x11=intersecting right
        {"%hhu ",   &m56_m478.host_rx_gps_relpos2host, BASE_CHAR, REPLAY_USE},	//###45
//m56_m4C0_t
        {"%u ",   &m56_m4C0.message_timeout_counter, BASE_INT, REPLAY_USE},	//###46
        {"%hhu ",   &m56_m4C0.msgcnt_id200, BASE_CHAR, REPLAY_USE},		//###47

        //Current speed received from n cars behind
        {"%.1f ",   &m56_m4C0.host_rx_vsp_491, BASE_FLOAT, REPLAY_USE},	//###48

        //Target speed received from n cars behind, distance not controlled
        {"%.1f ",   &m56_m4C0.host_rx_vspcomn_491, BASE_FLOAT, REPLAY_USE},	//###49

        //Target speed received from n cars behind, distance controlled
        {"%.1f ",   &m56_m4C0.host_rx_acvcomo_491, BASE_FLOAT, REPLAY_USE},	//###50

        //Current acceleration received from n cars behind
        {"%.2f ",   &m56_m4C0.host_rx_xg_491, BASE_FLOAT, REPLAY_USE},		//###51
//m56_m4C1_t
        {"%u ",   &m56_m4C1.message_timeout_counter, BASE_INT, REPLAY_USE},	//###52
        {"%hhu ",   &m56_m4C1.msgcnt_id201, BASE_CHAR, REPLAY_USE},		//###53

        //Distance received from n cars behind
        {"%.1f ",   &m56_m4C1.host_rx_mdistance_491, BASE_FLOAT, REPLAY_USE},	//###54

        //Speed received from n cars behind
        {"%.1f ",   &m56_m4C1.host_rx_vr_491, BASE_FLOAT, REPLAY_USE},		//###55

        //Brake switch status received from n cars behind
        {"%hhu ",   &m56_m4C1.host_rx_brake_sw_491, BASE_CHAR, REPLAY_USE},	//###56

        //Accelerator switch status received from n cars behind
        {"%hhu ",   &m56_m4C1.host_rx_accel_sw_491, BASE_CHAR, REPLAY_USE},	//###57

        //Set speed received from n cars behind
        {"%.1f ",   &m56_m4C1.host_rx_setvsp_491, BASE_FLOAT, REPLAY_USE},	//###58
//m56_m4C2_t
        {"%u ",   &m56_m4C2.message_timeout_counter, BASE_INT, REPLAY_USE},	//###59
        {"%hhu ",   &m56_m4C2.msgcnt_id202, BASE_CHAR, REPLAY_USE},		//###60
//m56_m4C3_t
        {"%u ",   &m56_m4C3.message_timeout_counter, BASE_INT, REPLAY_USE},	//###61
        {"%hhu ",   &m56_m4C3.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###62

        //GPS range to n cars behind
        {"%.1f ",   &m56_m4C3.host_rx_gps_range, BASE_FLOAT, REPLAY_USE},	//###63

        //Tracking status received from n cars behind
        //0=idle, 1=detecting, 2=tracking, 3=coupling, 0xF=invalid
        {"%hhu ",   &m56_m4C3.host_rx_gps_tracking_status, BASE_CHAR, REPLAY_USE},//###64

        //Vehicle position type received from n cars behind
        //0=standalone vehicle, 1=head of train, 2=tail of train,
        //3=middle vehicle of train, 0xF=invalid
        {"%hhu ",   &m56_m4C3.host_rx_gps_veh_pos_type, BASE_CHAR, REPLAY_USE},	//###65

        //Forward sequence number received from n cars behind
        //0=head vehicle of train, 1-0xFE=following vehicles,
        //0xFF=invalid
        {"%hhu ",   &m56_m4C3.host_rx_fwd_seq_num, BASE_CHAR, REPLAY_USE},	//###66

        //Backward sequence number received from n cars behind
        //0=tail vehicle of train, 1-0xFE=preceding vehicles,
        //0xFF=invalid
        {"%hhu ",   &m56_m4C3.host_rx_backwd_seq_num, BASE_CHAR, REPLAY_USE},	//###67
//m56_m4C4_t
        {"%u ",   &m56_m4C4.message_timeout_counter, BASE_INT, REPLAY_USE},	//###68
        {"%hhu ",   &m56_m4C4.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###69

        //WSU ID from n cars behind
        {"%hu ",   &m56_m4C4.host_rx_wsu_id, BASE_CHAR, REPLAY_USE},		//###70

        //Percent channel busy received from n cars behind
        {"%hhu ",   &m56_m4C4.host_rx_chanbusy, BASE_CHAR, REPLAY_USE},		//###71

        //RSS in dBm received from n cars behind
        {"%.1f ",   &m56_m4C4.host_rx_rss, BASE_FLOAT, REPLAY_USE},		//###72

        //RxPktPER% received from n cars behind
        //I assume this is a percentage of packets received
        {"%.1f ",   &m56_m4C4.host_rx_pkt_pct, BASE_FLOAT, REPLAY_USE},	//###73
//m56_m4C5_t
        {"%u ",   &m56_m4C5.message_timeout_counter, BASE_INT, REPLAY_USE},	//###74
        {"%hhu ",   &m56_m4C5.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###75

        //GPS latitude received from n cars behind
        {"%.6f ",   &m56_m4C5.host_rx_gps_latitude, BASE_FLOAT, REPLAY_USE},	//###76

        //GPS heading received from n cars behind
        {"%.1f ",   &m56_m4C5.host_rx_gps_heading, BASE_FLOAT, REPLAY_USE},	//###77
//m56_m4C6_t
        {"%u ",   &m56_m4C6.message_timeout_counter, BASE_INT, REPLAY_USE},	//###78
        {"%hhu ",   &m56_m4C6.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###79

        //GPS longitude received from n cars behind
        {"%.6f ",   &m56_m4C6.host_rx_gps_longitude, BASE_FLOAT, REPLAY_USE},	//###80

        //GPS speed received from n cars behind
        {"%.1f ",   &m56_m4C6.host_rx_gps_speed, BASE_FLOAT, REPLAY_USE},	//###81
//m56_m4C7_t
        {"%u ",   &m56_m4C7.message_timeout_counter, BASE_INT, REPLAY_USE},	//###82
        {"%hhu ",   &m56_m4C7.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###83

        //GPS altitude received from n cars behind
        {"%.2f ",   &m56_m4C7.host_rx_gps_altitude, BASE_FLOAT, REPLAY_USE},	//###84
//m56_m4C8_t
        {"%u ",   &m56_m4C8.message_timeout_counter, BASE_INT, REPLAY_USE},	//###85
        {"%hhu ",   &m56_m4C8.msgcnt_n, BASE_CHAR, REPLAY_USE},			//###86

        //GPS timestamp milliseconds, received from n cars behind
        {"%hu ",   &m56_m4C8.host_rx_gps_ts_millisecond, BASE_SHORT, REPLAY_USE},//###87

        //GPS range to host vehicle received from n cars behind
        {"%.1f ",   &m56_m4C8.host_rx_gps_range2host, BASE_FLOAT, REPLAY_USE},	//###88

        //Relative position to host vehicle received from n cars behind
        //0x00=unclassified, 0x01=ahead, 0x02=behind, 0x03=oncoming,
        //0x04=ahead left, 0x05=ahead right, 0x06=behind left,
        //0x07=behind right, 0x08=oncoming left, 0x09=oncoming right,
        //0x0A=ahead far left, 0X0B=ahead far right, 0x0C=behind far left,
        //0x0D=behind far right, 0x0E=oncoming far left,
        //0x0F=oncoming far right, 0x10=intersecting left
        //0x11=intersecting right
        {"%hhu ",   &m56_m4C8.host_rx_gps_relpos2host, BASE_CHAR, REPLAY_USE},	//###89
};

#define NUM_GFILE_COLUMNS sizeof(comm_specg)/sizeof(data_log_column_spec_t)
int num_gfile_columns = sizeof(comm_specg)/sizeof(data_log_column_spec_t);
