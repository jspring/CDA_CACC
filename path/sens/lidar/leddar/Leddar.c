// *****************************************************************************
// Module..: SDK -- Software development kit for Leddar products. RS-485
//           demonstration program.
//
/// \file    Leddar.c
///
/// \brief   Function definitions for the Leddar layer of the demo.
///
// Copyright (c) 2013-2014 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Leddar.h"
#include "Modbus.h"

static int   gConfigurationLevel = LEDDAR_NO_CONFIGURATION;
static LtU16 gBuildNumber = 0;
static int   gDeviceType;

// *****************************************************************************
// Function: LeddarConnect
//
/// \brief   Try to connect to a sensor on the given serial port.
///
/// \param   aPortName  Name of serial port to open (e.g.: COM4 on Windows or
///                     ttyUSB0 on Linux).
/// \param   aAddress   The Modbus address of the sensor to talk to.
///
/// \return  LT_SUCCESS or any one of the LT error codes.
// *****************************************************************************

LtResult
LeddarConnect( char *aPortName, LtByte aAddress )
{
    LtResult lResult;
//printf("LeddarConnect: Got to 1\n");
    lResult = ModbusConnect( aPortName, aAddress );
//printf("LeddarConnect: Got to 2 lResult %d\n", lResult);
    
    if ( lResult == LT_SUCCESS )
    {
//printf("LeddarConnect: Got to 3\n");
        lResult = ModbusSend( MODBUS_SERVER_ID, NULL, 0 );
//printf("LeddarConnect: Got to 4 lResult %d\n", lResult);
        
        if ( lResult == LT_SUCCESS )
        {
//printf("LeddarConnect: Got to 5\n");
            LtByte lId[MODBUS_MAX_PAYLOAD];

            lResult = ModbusReceive( lId );
            
            if ( lResult >= 0 )
            {
                gBuildNumber = lId[136] + (lId[137]<<8);
                gDeviceType = lId[150];

                // Identify which of the Leddar sensor model we are talking
                // to, to know features are available.
                if ( gDeviceType == LEDDAR_MODULE )
                {
                    gConfigurationLevel = LEDDAR_FULL_CONFIGURATION;
                    return LT_SUCCESS;
                }
                else if ( gDeviceType == LEDDAR_IS16 )
                {
                    gConfigurationLevel = LEDDAR_SIMPLE_CONFIGURATION;
                    return LT_SUCCESS;
                }
                else if ( gDeviceType == LEDDAR_EVAL_KIT )
                {
                    gConfigurationLevel = LEDDAR_NO_CONFIGURATION;
//                    gConfigurationLevel = LEDDAR_SIMPLE_CONFIGURATION;
                    return LT_SUCCESS;
                }

                // Unrecognized device!
                lResult = LT_ERROR;
            }
        }

        ModbusDisconnect();
    }

    return lResult;
}

// *****************************************************************************
// Function: LeddarDisconnect
//
/// \brief   Disconnect. Has no effect if was not connected.
// *****************************************************************************

void
LeddarDisconnect( void )
{
    ModbusDisconnect();
    gConfigurationLevel = LEDDAR_NO_CONFIGURATION;
}

// *****************************************************************************
// Function: LeddarDeviceType
//
/// \brief   Property reader for the device type of the
///          currently connected sensor.
///
/// \return  LEDDAR_EVAL_KIT or LEDDAR_IS16 or LEDDAR_MODULE
// *****************************************************************************

int
LeddarDeviceType( void )
{
    return gDeviceType;
}

// *****************************************************************************
// Function: LeddarBuildNumber
//
/// \brief   Property reader for the build number of the firmware for the
///          currently connected sensor.
///
/// \return  A version number
// *****************************************************************************

LtU16
LeddarBuildNumber( void )
{
    return gBuildNumber;
}

// *****************************************************************************
// Function: LeddarConfigurationLevel
//
/// \brief   Property reader for the level of configurability of the
///          currently connected sensor.
///
/// \return  One of the LEDDAR_*_CONFIGURATION constants.
// *****************************************************************************

int
LeddarConfigurationLevel( void )
{
    return gConfigurationLevel;
}

// *****************************************************************************
// Function: LeddarGetResults
//
/// \brief   Use Leddar custom function 0x41 to retrieve the detections and
///          other acquisition results.
///          Note that due to the payload length limitation in Modbus, at
///          most 48 detections can be returned (this maximum can be configured
///          to a lower value).
///
/// \param   aDetections  Pointer to array where detections will be written.
///
/// \return  LT_SUCCESS or any one of the LT error codes.
// *****************************************************************************

LtResult
LeddarGetResults( LtAcquisition *aAcquisition )
{
    LtResult lResult;
    LtBool   lGetTemperature = LeddarConfigurationLevel() != LEDDAR_NO_CONFIGURATION;

    if ( lGetTemperature )
    {
        lResult = LeddarGetTemperature( &aAcquisition->mTemperature );

        if ( lResult != LT_SUCCESS )
        {
            return lResult;
        }
    }
    else
    {
        aAcquisition->mTemperature = -1000;
    }

    lResult = ModbusSend( 0x41, NULL, 0 );

    if ( lResult == LT_SUCCESS )
    {
        LtByte lBuffer[MODBUS_MAX_PAYLOAD];

        lResult = ModbusReceive( lBuffer );

        if ( lResult > 0 )
        {
            int i;
            LtDetection *lDetections = aAcquisition->mDetections;
            
            aAcquisition->mDetectionCount = lBuffer[0] < LEDDAR_MAX_DETECTIONS ? lBuffer[0] : LEDDAR_MAX_DETECTIONS;

            for( i=0; i<aAcquisition->mDetectionCount; ++i )
            {
                lDetections[i].mDistance = ( lBuffer[1+i*5] + lBuffer[2+i*5]*256 )/100.f;
                lDetections[i].mAmplitude = ( lBuffer[3+i*5] + lBuffer[4+i*5]*256 )/64.f;
                lDetections[i].mSegment = lBuffer[5+i*5]>>4;
                lDetections[i].mFlags = lBuffer[5+i*5] & 0xf;
            }

            // Get the extra info after the detections
            i = lBuffer[0]*5 + 1;

            if ( lResult > i )
            {
                aAcquisition->mTimestamp =  lBuffer[i]
                                          + (lBuffer[i+1]<<8)
                                          + (lBuffer[i+2]<<16)
                                          + (lBuffer[i+3]<<24);
                aAcquisition->mStates = lBuffer[i+4] + (lBuffer[i+5]<<8);
            }
            else
            {
                aAcquisition->mTimestamp = 0;
                aAcquisition->mStates = 0;
            }

            return LT_SUCCESS;
        }
    }

    return lResult;
}

// *****************************************************************************
// Function: LeddarGetResultsAlt
//
/// \brief   Use input registers to retrieve the detections and other
///          acquisition results.
///
/// Retrieves only the first detections for each channel. To get more detections
/// simply increase the number of registers read.
///
/// \param   aDetections  Pointer to array where detections will be written.
///
/// \return  LT_SUCCESS or any one of the LT error codes.
// *****************************************************************************

LtResult
LeddarGetResultsAlt( LtAcquisition *aAcquisition )
{
    LtResult lResult;
    LtBool   lGetTemperature = LeddarConfigurationLevel() != LEDDAR_NO_CONFIGURATION;
    LtU16    lStart = LEDDAR_INPUT_ACQ_PARAMS;
    LtU16    lCount = 35;
    LtU16    lValues[LEDDAR_INPUT_DISTANCES+2*LEDDAR_CHANNEL_COUNT];
    LtU16    lOffset = LEDDAR_INPUT_ACQ_PARAMS;

    if ( lGetTemperature )
    {
        lStart = LEDDAR_INPUT_TEMPERATURE;
        lCount += LEDDAR_INPUT_ACQ_PARAMS-LEDDAR_INPUT_TEMPERATURE;
        lOffset = 0;
    }

    lResult = ModbusReadInputRegisters( lStart, lCount, lValues+lOffset );

    if ( lResult == LT_SUCCESS )
    {
        int i;

        if ( lGetTemperature )
        {
            aAcquisition->mTemperature = lValues[LEDDAR_INPUT_TEMPERATURE]/LEDDAR_TEMPERATURE_SCALE;
        }
        else
        {
            aAcquisition->mTemperature = -1000;
        }

        aAcquisition->mStates = lValues[LEDDAR_INPUT_ACQ_PARAMS];
        aAcquisition->mTimestamp =  lValues[LEDDAR_INPUT_TIMESTAMP]
                                  + (lValues[LEDDAR_INPUT_TIMESTAMP+1]<<16);

        aAcquisition->mDetectionCount = 0;
        
        for( i=0; i<LEDDAR_CHANNEL_COUNT; ++i )
        {
            int k = i + LEDDAR_INPUT_DISTANCES;

            if ( lValues[k+LEDDAR_CHANNEL_COUNT] != 0 )
            {
                int j=aAcquisition->mDetectionCount;

                aAcquisition->mDetections[j].mDistance = lValues[k]/100.f;
                aAcquisition->mDetections[j].mAmplitude = lValues[k+LEDDAR_CHANNEL_COUNT]/64.f;
                aAcquisition->mDetections[j].mSegment = i;
                aAcquisition->mDetections[j].mFlags = 1;
                
                ++aAcquisition->mDetectionCount;
            }
        }
    }

    return lResult;
}

// *****************************************************************************
// Function: LeddarGetTemperature
//
/// \brief   Return the current sensor internal temperature.
///
/// \param   aValue  Pointer to a variable that on output will contain the
///                  temperature in degree Celsius if no error.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult
LeddarGetTemperature( float *aValue )
{
    LtU16 lValue;

    LtResult lResult = ModbusReadInputRegisters( LEDDAR_INPUT_TEMPERATURE, 1, &lValue );

    if ( lResult == LT_SUCCESS )
    {
        *aValue = lValue/LEDDAR_TEMPERATURE_SCALE;
    }

    return lResult;
}

// *****************************************************************************
// Function: LeddarGetParameter
//
/// \brief   Generic configuration parameter read access function (for integer
///          parameters).
///
/// \param   aNo     Number of parameter to read (one of the LEDDAR_CONFIG_*
///                  constants).
/// \param   aValue  Pointer to a variable that on output will contain the
///                  value if LT_SUCCESS is returned.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult
LeddarGetParameter( LtU16 aNo, LtU16 *aValue )
{
    if ( aNo > LEDDAR_CONFIG_MAX_DETECTIONS )
    {
        return LT_INVALID_ARGUMENT;
    }

    return ModbusReadHoldingRegister( aNo, aValue );
}

// *****************************************************************************
// Function: LeddarSetParameter
//
/// \brief   Generic configuration parameter write access function (for integer
///          parameters).
///
/// \param   aNo     Number of parameter to write (one of the LEDDAR_CONFIG_*
///                  constants).
/// \param   aValue  The new value to set.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult
LeddarSetParameter( LtU16 aNo, LtU16 aValue )
{
    if ( aNo > LEDDAR_CONFIG_MAX_DETECTIONS )
    {
        return LT_INVALID_ARGUMENT;
    }

    return ModbusWriteRegister( aNo, aValue );
}

// *****************************************************************************
// Function: LeddarGetThreshold
//
/// \brief   Get the current detection threshold offset.
///
/// \param   aValue  Pointer to a variable that will contain the value on
///                  output if LT_SUCCESS is returned.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult
LeddarGetThreshold( float *aValue )
{
    LtU16 lValue;

    LtResult lResult = ModbusReadHoldingRegister( LEDDAR_CONFIG_THRESHOLD, &lValue );

    if ( lResult == LT_SUCCESS )
    {
        *aValue = ((Lt16)lValue)/256.f;
    }

    return lResult;
}

// *****************************************************************************
// Function: LeddarSetThreshold
//
/// \brief   Set the current detection threshold offset.
///
/// \param   aValue  New value to set.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult
LeddarSetThreshold( float aValue )
{
    return ModbusWriteRegister( LEDDAR_CONFIG_THRESHOLD, (Lt16)(aValue*256) );
}

// End of file Leddar.c
