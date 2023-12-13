// *****************************************************************************
// Module..: SDK -- Software development kit for Leddar products. RS-485
//           demonstration program.
//
/// \file    Main.c
///
/// \brief   This is the main file containing the menu driver.
///
// Copyright (c) 2013-2014 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "LeddarSDK3.2.0_x86/Includes/LeddarC.h"
#include "LeddarSDK3.2.0_x86/Includes/LeddarProperties.h"
#include "Leddar.h"

static jmp_buf exit_env;
static int sig_list[] = {
        SIGINT,
        SIGQUIT,
        SIGTERM,
        SIGALRM,
        ERROR,
};

static void sig_hand(int code)
{
        if (code == SIGALRM)
                return;
        else
                longjmp(exit_env, code);
}


static db_clt_typ *pclt;
static leddar_t leddar;
static int db_num = 123;
static char use_db = 0;
static char verbose = 0;

// *****************************************************************************
// Function: GetMenuChoice
//
/// \brief   Loops until a key is entered that is valid in a menu.
///
/// \return  The key entered.
// *****************************************************************************

static int
GetMenuChoice( void )
{
    int lResult = 0;
    while( ( lResult < '1' ) || ( lResult > '9' ) )
    {
        lResult = toupper( GetKey() );

        if ( lResult == 'Q' )
        {
            break;
        }
    }

    return lResult;
}

// *****************************************************************************
// Function: DisplayDetections
//
/// \brief   Display detection as well as other information in a continuous
///          loop until the user presses a key.
///
/// \param   aAlt  If true use alternate method with registers. Otherwise
///                use Modbus function 0x41.
// *****************************************************************************

static void
DisplayDetections( LtBool aAlt )
{
    LtAcquisition lAcquisition;

    while(1)
    {
        LtResult lResult;

        if ( aAlt )
        {
            lResult = LeddarGetResultsAlt( &lAcquisition );
        }
        else
        {
            lResult = LeddarGetResults( &lAcquisition );
        }

        if ( lResult == LT_SUCCESS )
        {
            int i;
            LtDetection *lDetections = lAcquisition.mDetections;

		if(verbose) {
            printf( "\nTimestamp    : %d\n", lAcquisition.mTimestamp );
            if ( lAcquisition.mTemperature > -100 )
            {
                printf( "Temperature  : %.1f deg C\n", lAcquisition.mTemperature );
            }
            printf( "LED Intensity: %d%%\n",
                    lAcquisition.mStates & LEDDAR_STATES_INTENSITY_MASK );
            printf( "Demerge status: %d\n\n",
                    (lAcquisition.mStates>>LEDDAR_STATES_DEMERGE_SHIFT) & 1 );

            for( i=0; i<lAcquisition.mDetectionCount; ++i )
            {
                printf( "%2d %6.2f %6.2f %2d\n", lDetections[i].mSegment,
                        lDetections[i].mDistance, lDetections[i].mAmplitude,
                        lDetections[i].mFlags );
            }

		}
	    printf("Distance: ");
            for( i=0; i<lAcquisition.mDetectionCount; ++i )
            {
                printf( "%4.2f ", lDetections[i].mDistance);
		leddar.distance[i] = lDetections[i].mDistance;
            }
	    printf("\n");
		if(use_db) {
			printf("Using database! db_num %d\n", db_num);
			if( db_clt_write(pclt, db_num, sizeof(leddar_t), &leddar) == 0)
				exit(EXIT_FAILURE);
		}

        }
        else
        {
            puts( "Communication error, aborting." );
            break;
        }
    }

    // Absorb the key used to stop the loop.
//    GetKey();
}

// *****************************************************************************
// Function: DisplayConfiguration
//
/// \brief   Display the current configuration parameters. What is displayed
///          is adapted for the type or sensor (assumed not called when
///          sensor does not support configuration).
// *****************************************************************************

static void
DisplayConfiguration( void )
{
#define DC_COMM_ERROR "Communication Error!"

    LtU16 lValue;
    int   lLevel = LeddarConfigurationLevel();
    float lValuef;

    puts( "" );

    if ( lLevel == LEDDAR_FULL_CONFIGURATION )
    {
        printf( "Accumulation           : " );
        if ( LeddarGetParameter( LEDDAR_CONFIG_ACCUMULATION, &lValue ) == LT_SUCCESS )
        {
            printf( "%d (%d)\n", lValue, 1<<lValue );
        }
        else
        {
            puts( DC_COMM_ERROR );
        }

        printf( "Oversampling           : " );
        if ( LeddarGetParameter( LEDDAR_CONFIG_OVERSAMPLING, &lValue ) == LT_SUCCESS )
        {
            printf( "%d (%d)\n", lValue, 1<<lValue );
        }
        else
        {
            puts( DC_COMM_ERROR );
        }

        printf( "Base sample count      : " );
        if ( LeddarGetParameter( LEDDAR_CONFIG_SAMPLE_COUNT, &lValue ) == LT_SUCCESS )
        {
            printf( "%d\n", lValue );
        }
        else
        {
            puts( DC_COMM_ERROR );
        }
    }
    else if ( lLevel == LEDDAR_SIMPLE_CONFIGURATION )
    {
        printf( "Sampling rate          : " );
        if ( LeddarGetParameter( LEDDAR_CONFIG_RATE, &lValue ) == LT_SUCCESS )
        {
            printf( "%d (%g Hz)\n", lValue, 12800.f/(1<<lValue) );
        }
        else
        {
            puts( DC_COMM_ERROR );
        }
    }

    printf( "Detection threshold    : " );
    if ( LeddarGetThreshold( &lValuef ) == LT_SUCCESS )
    {
        printf( "%.2f\n", lValuef );
    }
    else
    {
        puts( DC_COMM_ERROR );
    }

    printf( "LED power              : " );
    if ( LeddarGetParameter( LEDDAR_CONFIG_LED_POWER, &lValue ) == LT_SUCCESS )
    {
        printf( "%d%%\n", lValue );
    }
    else
    {
        puts( DC_COMM_ERROR );
    }

    printf( "Change delay           : " );
    if ( LeddarGetParameter( LEDDAR_CONFIG_CHANGE_DELAY, &lValue ) == LT_SUCCESS )
    {
        printf( "%d\n", lValue );
    }
    else
    {
        puts( DC_COMM_ERROR );
    }

    if ( LeddarGetParameter( LEDDAR_CONFIG_OPTIONS, &lValue ) == LT_SUCCESS )
    {
        printf( "Automatic LED intensity: %s\n",
                (lValue&LEDDAR_OPTION_AUTO_LED) ? "Enabled" : "Disabled" );
        printf( "Object demerging       : %s\n",
                (lValue&LEDDAR_OPTION_DEMERGE) ? "Enabled" : "Disabled" );
    }
    else
    {
        printf( "Automatic LED intensity: %s\n", DC_COMM_ERROR );
        printf( "Object demerging       : %s\n", DC_COMM_ERROR );
    }

    printf( "Maximum detections     : " );
    if ( LeddarGetParameter( LEDDAR_CONFIG_MAX_DETECTIONS, &lValue ) == LT_SUCCESS )
    {
        printf( "%d\n", lValue );
    }
    else
    {
        puts( DC_COMM_ERROR );
    }

}

// *****************************************************************************
// Function: ConfigurationMenu
//
/// \brief   Display a menu to allow changing configuration parameters.
// *****************************************************************************

static void
ConfigurationMenu( void )
{
    for(;;)
    {
        int      lChoice;
        float    lValue;
        LtResult lResult;
        int      lLevel = LeddarConfigurationLevel();

        puts( "\n" );
        if ( lLevel == LEDDAR_FULL_CONFIGURATION )
        {
            puts( "1. Change accumulation" );
            puts( "2. Change oversampling" );
            puts( "3. Change base sample count" );
        }
        else if ( lLevel == LEDDAR_SIMPLE_CONFIGURATION )
        {
            puts( "4. Change measurement rate" );
        }
        puts( "5. Change detection threshold offset" );
        puts( "6. Change LED power" );
        puts( "7. Change options" );
        puts( "8. Change Change delay" );
        puts( "9. Change maximum detection count" );
        puts( "Q. Exit" );

        lChoice = GetMenuChoice();

        if ( lChoice == 'Q' )
        {
            break;
        }

        printf( "\nEnter new value: " );
        scanf( "%f", &lValue );

        switch( lChoice )
        {
            case '1':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_ACCUMULATION, (LtU16) lValue );
                break;
            case '2':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_OVERSAMPLING, (LtU16) lValue );
                break;
            case '3':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_SAMPLE_COUNT, (LtU16) lValue );
                break;
            case '4':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_RATE, (LtU16) lValue );
                break;
            case '5':
                lResult = LeddarSetThreshold( lValue );
                break;
            case '6':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_LED_POWER, (LtU16) lValue );
                break;
            case '7':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_OPTIONS, (LtU16) lValue );
                break;
            case '8':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_CHANGE_DELAY, (LtU16) lValue );
                break;
            case '9':
                lResult = LeddarSetParameter( LEDDAR_CONFIG_MAX_DETECTIONS, (LtU16) lValue );
                break;
            default:
                lResult = LT_SUCCESS;
                break;
        }

        if ( lResult != LT_SUCCESS )
        {
            puts( "Operation failed!" );
        }
    }
}

// *****************************************************************************
// Function: ConnectMenu
//
/// \brief   Display a menu of actions that can be performed when connected.
// *****************************************************************************

static void
ConnectMenu(char *lBuffer, int lAddress)
{
    LtBool lHasDetectionRegisters;

    if ( LeddarConnect( lBuffer, lAddress ) == LT_SUCCESS )
    {
        // The input registers for detections were not available in earlier
        // versions of the sensors.
        lHasDetectionRegisters =   (    ( LeddarDeviceType() == LEDDAR_EVAL_KIT )
                                     && ( LeddarBuildNumber() > 6 ) )
                                || (    ( LeddarDeviceType() == LEDDAR_IS16 )
                                     && ( LeddarBuildNumber() > 2 ) )
                                || (    ( LeddarDeviceType() == LEDDAR_MODULE )
                                     && ( LeddarBuildNumber() > 4 ) );

    unsigned short myaddress = 0;

          if(ModbusReadHoldingRegister( 0x00AF, &myaddress) == LT_SUCCESS)
		printf( "Sensor address %u\n", myaddress);
	else
		printf( "Couldn't get address\n");

//        LeddarGetProperty(lBuffer, PID_SERIAL_ADDRESS, 0, &myaddress);
//        LeddarGetProperty(lBuffer, 0xAF, 0, &myaddress);
//    printf( "Sensor address %f\n", myaddress);
//exit(EXIT_SUCCESS);
        for(;;)
        {
            int lChoice;

//            puts( "\n\n1. Display detections (Function 0x41)" );
            if ( lHasDetectionRegisters )
            {
            	lChoice = '1';
//                puts( "2. Display detections (Input Registers)" );
            }
		else
            		lChoice = '2';
            if ( LeddarConfigurationLevel() > LEDDAR_NO_CONFIGURATION )
            {
                puts( "3. Display configuration" );
                puts( "4. Change configuration" );
            }
//            puts( "5. Disconnect" );

//            lChoice = GetMenuChoice();

            switch( lChoice )
            {
                case '1':
                    DisplayDetections( 0 );
                    break;
                case '2':
                    DisplayDetections( 1 );
                    break;
                case '3':
                    DisplayConfiguration();
                    break;
                case '4':
                    ConfigurationMenu();
                    break;
                case '5':
                    LeddarDisconnect();
                    return;
            }
        }


    }
    else
    {
        puts( "\nConnection failed!" );
    }
}


// *****************************************************************************
// Function: main
//
/// \brief   Standard C entry point!
// *****************************************************************************

int
main( int argc, char *argv[] )
{
        int option;
	char create_db_vars = 0;
        char   *lBuffer = "/dev/ser4";
        int    lAddress = 1;
        LtBool lHasDetectionRegisters;
        LtAcquisition *lAcquisition;
        LtDetection *lDetections;

        char hostname[MAXHOSTNAMELEN+1];
        char *domain = DEFAULT_SERVICE; // usually no need to change this
        int xport = COMM_OS_XPORT;      // set correct for OS in sys_os.h
        int exitsig;

        lAcquisition = calloc(sizeof(LtAcquisition), 1);
        lDetections = lAcquisition->mDetections;

        while ((option = getopt(argc, argv, "p:a:dcv")) != EOF) {
                switch (option) {
                case 'p':
                        lBuffer = strdup(optarg);
			if(strcmp(lBuffer, "dev4") != 0) {
				db_num = DB_LEDDAR_1_VAR;
				printf("ser4\n");
			}
			else {
				db_num = DB_LEDDAR_2_VAR;
				printf("ser6\n");
			}
                        break;
                case 'a':
                        lAddress = atoi(optarg);
                        break;
                case 'd':
                        use_db = 1;
                        break;
                case 'c':
                        create_db_vars = 1;
                        use_db = 1;
                        break;
                case 'v':
                        verbose = 1;
                        break;
		default:
			printf("Usage: %s -p <port, def. /dev/ser4> -a <sensor address, def. 1> -d (use database) -c (create database variables) -v (verbose)\n", argv[0]);
			exit(EXIT_FAILURE);
                }
        }

	if(use_db) {
	        get_local_name(hostname, MAXHOSTNAMELEN);
		if((pclt = clt_login(argv[0], hostname, domain, xport)) == NULL ) {
			fprintf(stderr, "%s: clt_login errors\n", argv[0]);
			exit(EXIT_FAILURE);
		}
		if(create_db_vars) {
			if(clt_create( pclt, db_num, db_num, sizeof(leddar_t)) == FALSE ) {
				printf("create error, var %d size %d\n", db_num, sizeof(leddar_t));
				exit(EXIT_FAILURE);
			}
		}
	}
        if(( exitsig = setjmp(exit_env)) != 0) {
                db_list_done(pclt, NULL, 0, NULL, 0);
                exit(EXIT_SUCCESS);
        } else
                sig_ign(sig_list, sig_hand);

	ConnectMenu(lBuffer, lAddress);

    return 0;
}
