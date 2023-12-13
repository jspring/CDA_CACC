// *****************************************************************************
// Module..: Leddar
//
/// \file    LdPropertyIds.h
///
/// \brief   ID list of properties.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************
#pragma once

namespace LeddarCore
{
    namespace LdPropertyIds
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        /// \enum   eLdPropertyIds
        ///
        /// \brief  Internal IDs of all the properties
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        typedef enum eLdPropertyIds
        {
            // Reserved value
            RESERVED_0  = 0x000001,
            RESERVED_1  = 0x000002,
            RESERVED_2  = 0x001100,
            RESERVED_3  = 0x001101,
            RESERVED_4  = 0x0015a0,
            RESERVED_5  = 0x001800,
            RESERVED_6  = 0x001780,
            RESERVED_7  = 0x001781,
            RESERVED_8  = 0x00178F,
            RESERVED_9  = 0x001790,
            RESERVED_10 = 0x001A00,
            RESERVED_11 = 0x001801,
            RESERVED_12 = 0x001802,
            RESERVED_13 = 0x001803,
            RESERVED_14 = 0x001805,
            RESERVED_15 = 0x001809,
            RESERVED_16 = 0x001806,
            RESERVED_17 = 0x001700,
            RESERVED_18 = 0x001701,
            RESERVED_19 = 0x001705,
            RESERVED_20 = 0x001706,
            RESERVED_21 = 0x001703,
            RESERVED_22 = 0x001704,
            RESERVED_23 = 0x100000,
            RESERVED_24 = 0x100001,
            RESERVED_25 = 0x100002,
            RESERVED_26 = 0x100003,
            RESERVED_27 = 0x100004,
            RESERVED_28 = 0x100005,
            RESERVED_29 = 0x100006,
            RESERVED_30 = 0x100007,
            RESERVED_31 = 0x100008,
            RESERVED_32 = 0x100009,
            RESERVED_33 = 0x10000A,
            RESERVED_34 = 0x10000B,
            RESERVED_35 = 0x10000C,
            RESERVED_36 = 0x10000D,
            RESERVED_37 = 0x10000E,

            // Sensor's properties
            ID_DEVICE_TYPE                      = 0x610018,
            ID_CONNECTION_TYPE                  = 0x610019,
            ID_DEVICE_NAME                      = 0x000022,
            ID_PART_NUMBER                      = 0x000F03,
            ID_SOFTWARE_PART_NUMBER             = 0x0010FC,
            ID_MANUFACTURER_NAME                = 0x610021,
            ID_SERIAL_NUMBER                    = 0x000F00,
            ID_BUILD_DATE                       = 0x610022,
            ID_FIRMWARE_VERSION_STR             = 0x610013, // a sensor can have str, int or both
            ID_FIRMWARE_VERSION_INT             = 0x0010FD, // should be the same as ID_FIRMWARE_VERSION_STR - but kept for retro-compatibility
            ID_FIRMWARE_VERSION_STRUCT          = 0x0010EF, // see struct sFirmwareVersion in LtComLeddarTechPublic.h
            ID_BOOTLOADER_VERSION               = 0x610014,
            ID_BOOTLOADER_PART_NUMBER           = 0x61003C,
            ID_RECEIVER_BOARD_VERSION           = 0x00100D,
            ID_ASIC_VERSION                     = 0x610015,
            ID_FPGA_VERSION                     = 0x610016,
            ID_GROUP_ID_NUMBER                  = 0x610020,
            ID_CARRIER_FIRMWARE_VERSION         = 0x610033,
            ID_CARRIER_FIRMWARE_PART_NUMBER     = 0x610037,
            ID_CARRIER_PART_NUMBER              = 0x610034,
            ID_CARRIER_SERIAL_NUMBER            = 0x610035,
            ID_CARRIER_OPTIONS                  = 0x610036,
            ID_MAC_ADDRESS                      = 0x610038, // Mac address in text format
            ID_IP_ADDRESS                       = 0x000F01,
            ID_IP_MODE                          = 0x000F04, // DHCP Mode
            ID_INTERFACE_GATEWAY_ADDRESS        = 0x00F004,
            ID_INTERFACE_SUBNET_MASK            = 0x00F005,
            ID_PHYSICAL_NEGOTIATION_MODE        = 0x000F05,
            ID_DATA_SERVER_PORT                 = 0x000098,
            ID_DATA_SERVER_PROTOCOL             = 0x001F01,
            ID_OPTIONS                          = 0x000F02,
            ID_CONDITION_COUNT                  = 0x0000B4, ///< M16 detections zones
            ID_CONDITION_OPTIONS                = 0x0000B5, ///< M16 detections zones
            ID_CONDITION_VALUE                  = 0x0000B6, ///< M16 detections zones
            ID_CONDITION_OPERATION              = 0x0000B7, ///< M16 detections zones
            ID_CONDITION_INDEX1                 = 0x0000B8, ///< M16 detections zones
            ID_CONDITION_INDEX2                 = 0x0000B9, ///< M16 detections zones
            ID_CONDITION_RISING_DB              = 0x0000BA, ///< M16 detections zones
            ID_CONDITION_FALLING_DB             = 0x0000BB, ///< M16 detections zones
            ID_ACQ_OPTIONS                      = 0x0000BC,
            ID_TEMPERATURE_SCALE                = 0x001017,
            ID_CPU_LOAD_SCALE                   = 0x500102,
            ID_APD_TEMPERATURE_SCALE            = 0x00101A,
            ID_CRC32                            = 0x0010FE,
            ID_ANGLE_OVR                        = 0x001025,
            ID_TEST_MODE                        = 0x0000DC,
            ID_PULSE_DIVISOR                    = 0x001140,
            ID_APD_VBIAS_VOLTAGE_T0             = 0x001141,
            ID_APD_VBIAS_MULTI_FACTOR           = 0x001142,
            ID_APD_VBIAS_T0                     = 0x001143,
            ID_APD_OPTIONS                      = 0x001144,
            ID_MEMS_PHASE                       = 0x001145,
            ID_BUFFER_SIZE_TCP                  = 0x001F02,
            ID_BUFFER_SIZE_UDP                  = 0x001F03,
            ID_SATURATION_PROC_EN               = 0x001162,
            ID_XTALK_OPTIC_SEG_ENABLE           = 0x001146,
            ID_XTALK_OPTIC_LINE_ENABLE          = 0x001147,
            ID_XTALK_OPTIC_ECH_SEG_ENABLE       = 0x00115A,
            ID_XTALK_OPTIC_ECH_LINE_ENABLE      = 0x00115B,
            ID_APD_TRACK_TEMP_COMP_ENABLE       = 0x001149,
            ID_ACC_DIST_ENABLE                  = 0x001150,
            ID_ACC_DIST_POSITION                = 0x001151,
            ID_ACC_DIST_EXP                     = 0x001152,
            ID_LIMIT_ACC_DIST_POS               = 0x000118,
            ID_LIMIT_ACC_DIST_EXP               = 0x000119,
            ID_THRESHOLD_OPTIONS                = 0x00011A,
            ID_AUTOMATIC_THRESHOLD_SENSI        = 0x00011B,
            ID_AUTOMATIC_THRESHOLD_SENSI_LIMITS = 0x00011E,
            ID_THREHSOLD_POS_OFFSET             = 0x610039, // Position of the threshold, different from usual sensitivity / threshold
            ID_LIMIT_THREHSOLD_POS_OFFSET       = 0x00011D,
            ID_TEMP_COMP                        = 0x001148,
            ID_TEMP_COMP_SLOPE                  = 0x00114A,
            ID_AUTO_HYSTERISIS_WIDTH            = 0x00115C,
            ID_AUTO_HOR_DISTRIBUTION_WIDTH      = 0x00115D,
            ID_AUTO_VER_DISTRIBUTION_WIDTH      = 0x00115E,
            ID_AUTO_CONTROLLER_DELAY            = 0x00115F,
            ID_AUTO_CONTROLLER_STEP             = 0x001160,
            ID_DIGITAL_FILTER_ENABLE            = 0x001161,
            ID_RELEASE_TYPE                     = 0x0010FA,
            ID_XTALK_INTER_TILE_ENABLE          = 0x001169,
            ID_SPACIAL_FILTER_ENABLE            = 0x00116A,
            ID_SYSTEM_TIME                      = 0x00116B,
            ID_SYNCHRONIZATION                  = 0x00116C,
            ID_CORRECT_DISTANCE_INTER_TILE      = 0x00116D,
            ID_HOMOGENIZE_DIST_IN_SAME_TARGET   = 0x00116E,
            ID_ECHO_FLAGS_FILTER                = 0x00180A,
            ID_ECHO_ACTIVE_FLAG_ENABLE          = 0x00180B,
            ID_ECHO_SATURATION_WIDTH            = 0x001808,
            ID_TRACE_SENT                       = 0x001791,

            ID_HSEGMENT       = 0x001020, // Number of horizontal channel
            ID_VSEGMENT       = 0x001021, // Number of vertical channel
            ID_VCHANNEL_INDEX = 0x001022, // Internal: Current vertical channel
            ID_RSEGMENT       = 0x001023,
            ID_REF_SEG_MASK   = 0x610026,
            ID_SUB_HSEGMENT   = 0x001028,
            ID_SUB_MODULE     = 0x001029,

            ID_BASE_SAMPLE_DISTANCE      = 0x0000E0,
            ID_DETECTION_LENGTH          = 0x00100F,
            ID_DISTANCE_SCALE            = 0x001003,
            ID_RAW_AMP_SCALE_BITS        = 0xAA0025,
            ID_RAW_AMP_SCALE             = 0x001002,
            ID_FILTERED_AMP_SCALE_BITS   = 0xAA0027,
            ID_FILTERED_AMP_SCALE        = 0x001004,
            ID_NB_SAMPLE_MAX             = 0x00100E, // Max trace length
            ID_CROSSTALK_SCALE           = 0x00101C,
            ID_CROSSTALK_INTERHEAD_SCALE = 0x00101D,

            ID_ACCUMULATION_EXP          = 0x0000A0,
            ID_ACCUMULATION_LIMITS       = 0x001011,
            ID_OVERSAMPLING_EXP          = 0x0000A1,
            ID_OVERSAMPLING_LIMITS       = 0x001012,
            ID_BASE_POINT_COUNT          = 0x0000A2,
            ID_BASE_POINT_COUNT_LIMITS   = 0x000086,
            ID_SEGMENT_ENABLE            = 0x610008,
            ID_SEGMENT_ENABLE_COM        = 0x0000D9,
            ID_REFERENCE_SEGMENT_ENABLE  = 0x610027,
            ID_PRECISION                 = 0x610005, // precision = smoothing
            ID_PRECISION_ENABLE          = 0x610006,
            ID_PRECISION_LIMITS          = 0x000107,
            ID_XTALK_ECHO_REMOVAL_ENABLE = 0x610011,
            ID_XTALK_REMOVAL_ENABLE      = 0x610010,
            ID_SATURATION_COMP_ENABLE    = 0x610009,
            ID_OVERSHOOT_MNG_ENABLE      = 0x61000B,
            ID_SENSIVITY                 = 0x610007, // Sensitivity = threshold offset
            ID_SENSIVITY_OLD             = 0x0000A3, // Sensitivity = threshold offset - used in M16 sensors
            ID_SENSIVITY_LIMITS          = 0x001018,
            ID_ACQUISITION_OPTION_MASK   = 0x001019,
            ID_STATIC_SENSITIVITY_ENABLE = 0x001168,
            ID_REF_PULSE_RATE            = 0x610023,
            ID_PULSE_RATE                = 0x610024,
            ID_CHANGE_DELAY              = 0x0000BD,
            ID_CHANGE_DELAY_LIMITS       = 0x001014,
            ID_GAIN_ENABLE               = 0x0000A4,
            ID_REFRESH_RATE              = 0x001008,
            ID_REFRESH_RATE_LIST         = 0x0010FB, // IS16
            ID_TRACE_LENGTH              = 0x001001,
            ID_TRACE_POINT_STEP          = 0x001009, // Distance between two trace points
            ID_START_TRACE               = 0x0000E1,
            ID_START_TRACE_LIMITS        = 0x0000E2,
            ID_NUMBER_TRACE_SENT         = 0x00009B,
            ID_TRACE_TYPE                = 0x00FFFE, // Trace type (M16 only)
            ID_DISTANCE_RESOLUTION       = 0x0000D8,
            ID_ECHO_AMPLITUDE_MAX        = 0x001026,
            ID_TRIGGER_MODE              = 0x0000E4,
            ID_ACQUISITION_MODE          = 0x0000E5,
            ID_RAISING_FALLING_EDGE      = 0x0000E6,

            ID_LED_INTENSITY             = 0x00002A,
            ID_LED_INTENSITY_LIST        = 0xAA0013,
            ID_LED_PWR_ENABLE            = 0x61000C,
            ID_LED_AUTO_PWR_ENABLE       = 0x61000D,
            ID_LED_AUTO_FRAME_AVG        = 0x61000F,
            ID_LED_AUTO_ECHO_AVG         = 0x61000E,
            ID_LED_AUTO_FRAME_AVG_LIMITS = 0x00004A,
            ID_LED_AUTO_ECHO_AVG_LIMITS  = 0x00004B,
            ID_LED_USR_PWR_COUNT         = 0xAA0061,
            ID_PWM_LASER                 = 0x0000DF,

            ID_DEMERGING_ENABLE            = 0x610012,
            ID_STATIC_NOISE_REMOVAL_ENABLE = 0x0000CF,
            ID_STATIC_NOISE_TEMP_ENABLE    = 0x00FFCF, // Static noise temperature removal enable
            ID_STATIC_NOISE_CALIB_TEMP     = 0x00FFCD, // The temperature at which the static noise got calibrated
            ID_STATIC_NOISE_UPDATE_ENABLE  = 0x0000D2, // LeddarOne only - Pulse noise = static noise
            ID_STATIC_NOISE_UPDATE_RATE    = 0x0000D0, // LeddarOne only - Pulse noise = static noise
            ID_STATIC_NOISE_UPDATE_AVERAGE = 0x0000D1, // LeddarOne only - Pulse noise = static noise
            ID_LEARNED_TRACE_OPTIONS       = 0x000134,
            ID_ALGO_REQUESTS               = 0x001709,
            ID_ALGO_RIPPLE_CLEANER         = 0x61003A,
            ID_NB_CYCLE_PER_SCAN           = 0x0000DD,
            ID_PWM_PERIOD                  = 0x0000DE,
            ID_PWM_TABLE                   = 0x0000DF,

            ID_THRESH_AGG_AMP                          = 0x1153,
            ID_THRESH_VICTIM_DIST                      = 0x1154,
            ID_THRESH_ELEC_AGG_AMP                     = 0x1155,
            ID_THRESH_ECH_VICTIM_LEFT                  = 0x1156,
            ID_THRESH_ECH_VICTIM_RIGHT                 = 0x1157,
            ID_THRESH_GAUSS_SENSIB                     = 0x1158,
            ID_THRESH_M_PEAK                           = 0x1159,
            ID_PDLT_TRIANGLE_THRESHOLD                 = 0x1163,
            ID_PDLT_TRIANGLE_THRESHOLD_LIMITS          = 0x1164,
            ID_PDLT_TRIANGLE_THRESHOLD_DISTANCE        = 0x1165,
            ID_PDLT_TRIANGLE_THRESHOLD_DISTANCE_LIMITS = 0x1166,
            ID_GRADUAL_THRESHOLD_ENABLE                = 0x1167,
            ID_STATIC_THRESHOLD_DISTANCES              = 0x1168,
            ID_STATIC_THRESHOLD_AMPLITUDES             = 0x1169,

            ID_INTERF_ENABLE              = 0x610040,
            ID_SAWTOOTH_CORRECTION_ENABLE = 0x61003A,
            ID_LINE_RANDOM_ENABLE         = 0x61003B,

            ID_DIST_VS_AMP_COMP_ENABLE   = 0x610047,
            ID_TEMP_DISTANCE_COMP_ENABLE = 0x610048,
            ID_TIMEBASEDELAY_ENABLE      = 0x610049,
            ID_FRAME_TO_FRAME_ENABLE     = 0x61004A,
            ID_TIMEBASEDELAY_CALIB_TEMP  = 0x61004B,

            ID_STATUS_ALERT = 0x610046,

            ID_TIMEBASE_DELAY          = 0x00012A,
            ID_STATIC_NOISE            = 0x00012C,
            ID_CHANNEL_ANGLE_AZIMUT    = 0x00012D,
            ID_CHANNEL_ANGLE_ELEVATION = 0x00012E,
            ID_INTENSITY_COMPENSATIONS = 0x000132,
            ID_REAL_DISTANCE_OFFSET    = 0x001006,
            ID_MAX_ECHOES_PER_CHANNEL  = 0x001024,

            ID_ORIGIN_X            = 0x0000A5,
            ID_ORIGIN_Y            = 0x0000A6,
            ID_ORIGIN_Z            = 0x0000A7,
            ID_YAW                 = 0x0000A8,
            ID_PITCH               = 0x0000A9,
            ID_ROLL                = 0x0000AA,
            ID_C_DETECTION_RANGE   = 0x200000,
            ID_C_REFRESH_RATE      = 0x200001,
            ID_C_PRODUCT_TYPE      = 0x200002,
            ID_HFOV                = 0x200003,
            ID_VFOV                = 0x200004,
            ID_C_START_TRACE_RANGE = 0x200005,
            ID_SUB_HFOV            = 0x200006,
            ID_SUB_HPOSITION       = 0x200008,

            ID_LICENSE               = 0x000104,
            ID_LICENSE_INFO          = 0x000105,
            ID_VOLATILE_LICENSE      = 0x000120,
            ID_VOLATILE_LICENSE_INFO = 0x000121,

            ID_COM_SERIAL_PORT_BAUDRATE         = 0x0000AB,
            ID_COM_SERIAL_PORT_DATA_BITS        = 0x0000AC,
            ID_COM_SERIAL_PORT_PARITY           = 0x0000AD,
            ID_COM_SERIAL_PORT_STOP_BITS        = 0x0000AE,
            ID_COM_SERIAL_PORT_ADDRESS          = 0x0000AF,
            ID_COM_SERIAL_PORT_FLOW_CONTROL     = 0x00FFFF,
            ID_COM_SERIAL_PORT_BAUDRATE_OPTIONS = 0x0000DB, // Baudrate options mask
            ID_COM_SERIAL_PORT_LOGICAL_PORT     = 0xAA1007,
            ID_COM_SERIAL_PORT_MAX_ECHOES       = 0x0000C9,
            ID_COM_SERIAL_PORT_ECHOES_RES       = 0xAA1009,
            ID_COM_SERIAL_PORT_CURRENT_PORT     = 0xAA100A,
            ID_COM_CAN_PORT_BAUDRATE            = 0x0000B0,
            ID_COM_CAN_PORT_TX_MSG_BASE_ID      = 0x0000B1,
            ID_COM_CAN_PORT_RX_MSG_BASE_ID      = 0x0000B2,
            ID_COM_CAN_PORT_FRAME_FORMAT        = 0x0000B3,
            ID_COM_CAN_PORT_PORT_OPTIONS        = 0x0000D3, // can options mask
            ID_COM_CAN_PORT_MAILBOX_DELAY       = 0x0000D4,
            ID_COM_CAN_PORT_PORT_ACQCYCLE_DELAY = 0x0000D5,
            ID_COM_CAN_PORT_MAX_ECHOES          = 0x0000D6,
            ID_COM_CAN_PORT_MAX_ECHOES_LIMIT    = 0x0000D7,
            ID_COM_CAN_PORT_OPTIONS_MASK        = 0x0000DA,
            ID_COM_CAN_PORT_LOGICAL_PORT        = 0xAA1015,
            ID_COM_CAN_PORT_ECHOES_RES          = 0xAA1017,

            // IS16
            ID_IS16_ZONE_RISING_DB       = 0x0000BE,
            ID_IS16_ZONE_FALLING_DB      = 0x0000BF,
            ID_IS16_OUTPUT_NPN_PNP       = 0x0000C0,
            ID_IS16_OUTPUT_INVERSION     = 0x0000C1,
            ID_IS16_ZONE_FARS            = 0x0000C2,
            ID_IS16_ZONE_NEARS           = 0x0000C3,
            ID_IS16_ZONE_SEGMENT_ENABLES = 0x0000C4,
            ID_IS16_ZONE_ENABLES         = 0x0000C5,
            ID_IS16_TEACH_MARGIN         = 0x0000C6,
            ID_IS16_ZONE_EDIT_MODE       = 0x0000C7,
            ID_IS16_LOCK_PANEL           = 0x0000C8,
            ID_IS16_ALGO_TYPE            = 0x0000CC,
            ID_IS16_LCD_CONTRAST         = 0x0000CD,
            ID_IS16_LCD_BRIGHTNESS       = 0x0000CE,
            // Derived properties for IS16
            ID_IS16_MEASUREMENT_RATE = 0x300000,
            ID_IS16_USEFUL_RANGE     = 0x300001,
            ID_IS16_OUTPUT_CONFIG    = 0x300002,
            ID_IS16_DISTANCE_MODE    = 0x300003,
            ID_IS16_ZONE_DISPLAYED   = 0x300004,
            ID_IS16_ZONE_ENABLED     = 0x300005,

            // DTec
            ID_CHANNEL_AREA             = 0x011000,
            ID_CAL_APD                  = 0x011001,
            ID_CAL_AMP                  = 0x011002,
            ID_CAL_IMG                  = 0x011003,
            ID_PAN_TILT                 = 0x011004,
            ID_ACTIVE_ZONES             = 0x011005,
            ID_PULSE_WIDTH_COMPENSATION = 0x011006,

            // Result State's properties
            ID_RS_CPU_LOAD           = 0x500100,
            ID_MEASURED_FREQ         = 0x500101,
            ID_CURRENT_LED_INTENSITY = 0x500103,
            ID_RS_BACKUP             = 0x500104, // Calibration backup: 0=invalid backup, 1=factory backup, 2=user backup
            ID_RS_TIMESTAMP_STD      = 0x001A11,
            ID_RS_TIMESTAMP          = 0x001A01,
            ID_RS_CURRENT_TIMES_MS   = 0x011A01,
            ID_RS_SYSTEM_TEMP        = 0x001A03,
            ID_RS_POSITION           = 0x001A04, // DTec
            ID_RS_SOURCE_CURRENT     = 0x001A05,
            ID_RS_SOURCE_VOLTAGE     = 0x001A06,
            ID_RS_ECHO_COUNT         = 0x001A07, // DTec
            ID_RS_DISTANCES          = 0x001A08, // DTec
            ID_RS_AMPLITUDES         = 0x001A09, // DTec
            ID_RS_PRESENCE_COUNTS    = 0x001A0B, // DTec
            ID_RS_STATES_V2          = 0x001A0C, // DTec
            ID_RS_DISCRETE_OUTPUTS   = 0x001A0F, // Flag to know if there is something in the detection zone (IS16 & evalkit)
            ID_RS_ACQ_CURRENT_PARAMS = 0x001A10, // Sent with every trace, contains led power and several flags (M16)
            ID_RS_PREDICT_TEMP       = 0x001A13,
            ID_RS_APD_TEMP           = 0x001A14,
            ID_RS_APD_GAIN           = 0x001A15,
            ID_RS_NOISE_LEVEL        = 0x001A16,
            ID_RS_ADC_RSSI           = 0x001A17,
            ID_RS_SNR                = 0x001A18,
            ID_RS_V3M_TEMP           = 0x001A19,
            ID_RS_TIMESTAMP64        = 0x001A1A,
            ID_STATE_CPU_TEMP        = 0x001A1B,
            ID_RS_FRAME_ID           = 0x001A1C,
            ID_RS_PMIC_TEMP          = 0x001A20,
            ID_RS_NOISE_LEVEL_AVG    = 0x001A21,

            // LeddarEngine frame config
            ID_LE_FRM_QTY               = 0x700001,
            ID_LE_FRM_OPTICAL_TILES_QTY = 0x700002,
            ID_LE_FRM_VSEG              = 0x700003,
            ID_LE_FRM_HSEG              = 0x700004,
            ID_LE_FRM_FRAME_SIZE        = 0x700005,
            ID_LE_FRM_FRAME_RATE        = 0x700006,

            // LeddarEngine optical tile config
            ID_LE_OPT_TOTAL_QTY            = 0x700010,
            ID_LE_OPT_FRAME_CFG_INDEX       = 0x700011,
            ID_LE_OPT_OPTICAL_TILE_INDEX    = 0x700012,
            ID_LE_OPT_ACQUISITION_TILES_QTY = 0x700013,
            ID_LE_OPT_VSEG                  = 0x700014,
            ID_LE_OPT_HSEG                  = 0x700015,
            ID_LE_OPT_TILE_SEQ_NUMBER       = 0x700016,
            ID_LE_OPT_DBSD_TRANSITION_TIME  = 0x700017,
            ID_LE_OPT_DBSD_POSITION         = 0x700018,

            // LeddarEngine acquisition tile config
            ID_LE_ACQ_TOTAL_QTY             = 0x700040,
            ID_LE_ACQ_FRAME_CFG_INDEX        = 0x700041,
            ID_LE_ACQ_OPTICAL_TILE_INDEX     = 0x700042,
            ID_LE_ACQ_ACQUISITION_TILE_INDEX = 0x700043,
            ID_LE_ACQ_SEGMENTS_QTY           = 0x700044,
            ID_LE_ACQ_VSEG                   = 0x700045,
            ID_LE_ACQ_HSEG                   = 0x700046,
            ID_LE_ACQ_TILE_SEQ_NUMBER        = 0x700047,
            ID_LE_ACQ_ACCUMULATIONS          = 0x700048,
            ID_LE_ACQ_OVERSAMPLING           = 0x700049,
            ID_LE_ACQ_BASE_POINTS            = 0x70004A,
            ID_LE_ACQ_LASERS_ENABLED         = 0x70004B,

            // LeddarEngine config

            ID_LE_FRAME_CFG_INDEX          = 0x700500,
            ID_LE_WAVEFORM_ROI   = 0x700501,
            ID_LE_UDP_RX_IP                = 0x700502,
            ID_LE_UDP_RX_DETECTIONS_PORT   = 0x700503,
            ID_LE_UDP_RX_RAW_WF_PORT       = 0x700504,
            ID_LE_UDP_RX_PROCESSED_WF_PORT = 0x700505,

        } eLdPropertyIds;
    } // namespace LdPropertyIds

} // namespace LeddarCore
