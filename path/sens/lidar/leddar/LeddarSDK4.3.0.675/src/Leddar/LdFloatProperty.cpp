// *****************************************************************************
// Module..: Leddar
//
/// \file    LdFloatProperty.cpp
///
/// \brief   Definition of functions for class LdFloatProperty.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdFloatProperty.h"
#include "LtScope.h"
#include "LtStringUtils.h"

#include <cassert>
#include <iomanip>
#include <ios>
#include <limits>
#include <math.h>
#include <sstream>
#include <string>

LeddarCore::LdFloatProperty::LdFloatProperty( const LdFloatProperty &aProperty )
    : LdProperty( aProperty )
{
    std::lock_guard<std::recursive_mutex> lock( aProperty.mPropertyMutex );
    mMinValue = aProperty.mMinValue;
    mMaxValue = aProperty.mMaxValue;
    mScale    = aProperty.mScale;
    mDecimals = aProperty.mDecimals;
}

// *****************************************************************************
// Function: LdFloatProperty::LdFloatProperty
//
/// \brief   Constructor.
///
/// The limits are set to the maximum range for a float. If fixed point
/// format they are then updated to the maximum raw range.
///
/// \param   aFeatures  See LdProperty.
/// \param   aId        See LdProperty.
/// \param   aDeviceId  See LdProperty.
/// \param   aUnitSize  See LdProperty.
/// \param   aScale     The scale factor between the floating point and
///                     fixed point representations, i.e. fixed point =
///                     floating point * scale. If the scale is 0, the value
///                     is natively stored as a float.
/// \param   aDecimals  The number of decimals to use when transforming a
///                     value to its text form.
/// \param aCategory    See LdProperty.
/// \param aDescription See LdProperty.
///
/// \author  Louis Perreault
///
/// \since   August 2014
// *****************************************************************************
LeddarCore::LdFloatProperty::LdFloatProperty( LdProperty::eCategories aCategory, uint32_t aFeatures, uint32_t aId, uint16_t aDeviceId, uint32_t aUnitSize, uint32_t aScale,
                                              uint32_t aDecimals, const std::string &aDescription )
    : LdProperty( LdProperty::TYPE_FLOAT, aCategory, aFeatures, aId, aDeviceId, aUnitSize, sizeof( int32_t ), aDescription )
    , mMinValue( -std::numeric_limits<float>::max() )
    , mMaxValue( std::numeric_limits<float>::max() )
    , mScale( aScale )
    , mDecimals( aDecimals )
{
    SetMaxLimits();
}

// *****************************************************************************
// Function: LdFloatProperty::PerformValue
//
/// \brief   Return the current value of the property at the given index.
///
/// \param   aIndex  Index of value to get.
///
/// \return  The value.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
float LeddarCore::LdFloatProperty::PerformValue( size_t aIndex ) const
{
    VerifyInitialization();

    if( aIndex >= PerformCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
    }

    // If scale is 0 value is a float directly. Otherwise it is a fixed-point
    // that we must transform into a float.
    if( mScale != 0 )
    {
        return static_cast<float>( PerformRawValue( aIndex ) ) / mScale;
    }

    // cppcheck-suppress invalidPointerCast
    return reinterpret_cast<const float *>( CStorage() )[aIndex];
}

// *****************************************************************************
// Function: LdFloatProperty::PerformDeviceValue
//
/// \brief   Return the backup value (that is the value in the device).
///
/// \param   aIndex  Index of value to get.
///
/// \return  The value.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
float LeddarCore::LdFloatProperty::PerformDeviceValue( size_t aIndex ) const
{
    VerifyInitialization();

    if( aIndex >= PerformCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
    }

    // If scale is 0 value is a float directly. Otherwise it is a fixed-point
    // that we must transform into a float.
    if( mScale != 0 )
    {
        return static_cast<float>( PerformRawDeviceValue( aIndex ) ) / mScale;
    }

    // cppcheck-suppress invalidPointerCast
    return reinterpret_cast<const float *>( BackupStorage() )[aIndex];
}

// *****************************************************************************
// Function: LdFloatProperty::PerformGetStringValue
//
/// \brief   Utility function to transform a float value to text in a standard
///          way.
///
/// \param   aIndex  Index value to tranform to text
///
/// \return  The value in its textual form.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
std::string LeddarCore::LdFloatProperty::PerformGetStringValue( size_t aIndex ) const
{
    std::stringstream lResult;
    lResult << std::fixed << std::setprecision( mDecimals ) << PerformValue( aIndex );
    return lResult.str();
}

// *****************************************************************************
// Function: LdFloatProperty::PerformSetMaxLimits
//
/// \brief   Set the limits to the maximum range from the current scale and
///          unit size.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
void LeddarCore::LdFloatProperty::PerformSetMaxLimits( void )
{
    if( mScale != 0 )
    {
        switch( PerformUnitSize() )
        {
        case 1:
            mMinValue = std::numeric_limits<char>::min() / static_cast<float>( mScale );
            mMaxValue = std::numeric_limits<char>::max() / static_cast<float>( mScale );
            break;

        case 2:
            mMinValue = std::numeric_limits<int16_t>::min() / static_cast<float>( mScale );
            mMaxValue = std::numeric_limits<int16_t>::max() / static_cast<float>( mScale );
            break;

        case 4:
            mMinValue = std::numeric_limits<int32_t>::min() / static_cast<float>( mScale );
            mMaxValue = std::numeric_limits<int32_t>::max() / static_cast<float>( mScale );
            break;
        }

        // Truncate to the nearest value according to the decimals
        const float lRounder = static_cast<float>( static_cast<int>( pow( 10, mDecimals ) ) );

        mMinValue = ( mMinValue * lRounder ) / lRounder;
        mMaxValue = ( mMaxValue * lRounder ) / lRounder;
    }
}

// *****************************************************************************
// Function: LdFloatProperty::PerformSetLimits
//
/// \brief   Change the minimum and maximum allowed values.
///
/// The current value(s) will be clipped to the new limits.
///
/// \param   aMin  The minimum allowed value.
/// \param   aMax  The maximum allowed value.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
void LeddarCore::LdFloatProperty::PerformSetLimits( float aMin, float aMax )
{
    if( aMin > aMax )
    {
        throw std::invalid_argument( "Invalid min value is higher than the max value." );
    }

    if( ( aMin != mMinValue ) || ( aMax != mMaxValue ) )
    {
        mMinValue = aMin;
        mMaxValue = aMax;

        // Clip current values only if the property was initialized
        if( IsInitialized() )
        {
            const size_t lCount = PerformCount();

            for( size_t i = 0; i < lCount; ++i )
            {
                const float lValue = PerformValue( i );

                if( lValue < mMinValue )
                {
                    PerformSetValue( i, mMinValue );
                }
                else if( lValue > mMaxValue )
                {
                    PerformSetValue( i, mMaxValue );
                }
            }
        }

        EmitSignal( LdObject::LIMITS_CHANGED );
    }
}

// *****************************************************************************
// Function: LdFloatProperty::PerformSetRawLimits
//
/// \brief   Change the minimum and maximum allowed values using raw values.
///
/// The current value(s) will be clipped to the new limits.
///
/// \param   aMin  The minimum allowed value.
/// \param   aMax  The maximum allowed value.
///
/// \author  Patrick Boulay
///
/// \since   March 2016
// *****************************************************************************
void LeddarCore::LdFloatProperty::PerformSetRawLimits( int32_t aMin, int32_t aMax )
{
    float lMinValue = mScale == 0 ? aMin : aMin / static_cast<float>( mScale );
    float lMaxValue = mScale == 0 ? aMax : aMax / static_cast<float>( mScale );

    PerformSetLimits( lMinValue, lMaxValue );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarCore::LdFloatProperty::PerformSetAnyValue( size_t aIndex, const boost::any &aNewValue )
///
/// \brief  Set the property value
///
/// \author David L�vy
/// \date   February 2021
///
/// \exception  std::invalid_argument   Thrown when an invalid argument error condition occurs.
///
/// \param  aIndex      Zero-based index of the.
/// \param  aNewValue   The new value.
////////////////////////////////////////////////////////////////////////////////////////////////////
void LeddarCore::LdFloatProperty::PerformSetAnyValue( size_t aIndex, const boost::any &aNewValue )
{
    if( aNewValue.type() == typeid( float ) )
    {
        PerformSetValue( aIndex, boost::any_cast<float>( aNewValue ) );
    }
    else if( aNewValue.type() == typeid( double ) )
    {
        PerformSetValue( aIndex, boost::any_cast<double>( aNewValue ) );
    }
    else if( aNewValue.type() == typeid( int ) )
    {
        PerformSetValue( aIndex, boost::any_cast<int>( aNewValue ) );
    }
    else
        throw std::invalid_argument( "Invalid value type" );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn	LeddarCore::LdProperty *LeddarCore::LdFloatProperty::PerformClone()
///
/// \brief	Performs the clone action
///
/// \returns	Null if it fails, else a pointer to a LeddarCore::LdProperty.
///
/// \author	Alain Ferron
/// \date	March 2021
////////////////////////////////////////////////////////////////////////////////////////////////////
LeddarCore::LdProperty *LeddarCore::LdFloatProperty::PerformClone() { return new LdFloatProperty( *this ); }

// *****************************************************************************
// Function: LdFloatProperty::PerformSetRawValue
//
/// \brief   Change the current raw value at the given index.
///
/// Limits are not checked in this mode. The native representation must be
/// fixed point or the resulting stored value will be garbage.
///
/// \param   aIndex  Index of value to change.
/// \param   aValue  The new value.
///
/// \exception std::out_of_range Invalid property count
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
void LeddarCore::LdFloatProperty::PerformSetRawValue( size_t aIndex, int32_t aValue )
{
    CanEdit();

    // Initialize the count to 1 on the fist SetValue if not done before.
    if( PerformCount() == 0 && aIndex == 0 )
    {
        PerformSetCount( 1 );
    }

    assert( mScale != 0 );

    if( aIndex >= PerformCount() )
    {
        throw std::out_of_range( "Invalid property count." );
    }

    if( ( aValue < mMinValue * mScale ) || ( aValue > mMaxValue * mScale ) )
    {
        throw std::out_of_range( "Value outside the limits. Property id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
    }

    if( !IsInitialized() || aValue != PerformRawValue( aIndex ) )
    {
        reinterpret_cast<int32_t *>( Storage() )[aIndex] = aValue;
        SetInitialized( true );
        EmitSignal( LdObject::VALUE_CHANGED );
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarCore::LdFloatProperty::PerformForceRawValue( size_t aIndex, int32_t aValue )
///
/// \brief  Force raw value
///
/// Limits are not checked in this mode. The native representation must be
/// fixed point or the resulting stored value will be garbage.
///
/// \param   aIndex  Index of value to change.
/// \param   aValue  The new value.
///
/// \exception std::out_of_range Invalid property count
///
/// \author David Levy
/// \date   March 2019
////////////////////////////////////////////////////////////////////////////////////////////////////
void LeddarCore::LdFloatProperty::PerformForceRawValue( size_t aIndex, int32_t aValue )
{
    LeddarUtils::LtScope<bool> lForceEdit( &mCheckEditable, true );
    mCheckEditable = false;
    PerformSetRawValue( aIndex, aValue );
}

// *****************************************************************************
// Function: LdFloatProperty::PerformSetValue
//
/// \brief   Change the current value at the given index.
///
/// An exception will be throwned if the new value is not within the limits.
///
/// \param   aIndex  Index of value to change.
/// \param   aValue  The new value.
///
/// \exception std::out_of_range Value outside the limits or invalid property count
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
void LeddarCore::LdFloatProperty::PerformSetValue( size_t aIndex, float aValue )
{
    CanEdit();

    // Initialize the count to 1 on the fist SetValue if not done before.
    if( PerformCount() == 0 && aIndex == 0 )
    {
        PerformSetCount( 1 );
    }

    if( aIndex >= PerformCount() )
    {
        throw std::out_of_range( "Index not valid, verify property count. Property id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
    }

    if( ( aValue < mMinValue ) || ( aValue > mMaxValue ) )
    {
        throw std::out_of_range( "Value outside the limits. Property id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
    }

    if( mScale == 0 )
    {
        if( !IsInitialized() || aValue != PerformValue( aIndex ) )
        {
            // cppcheck-suppress invalidPointerCast
            float &lFloatValue = reinterpret_cast<float *>( Storage() )[aIndex];
            lFloatValue        = aValue;
            SetInitialized( true );
            EmitSignal( LdObject::VALUE_CHANGED );
        }
    }
    else
    {
        std::ostringstream lNewValueStream;
        lNewValueStream << aValue;

        std::ostringstream lOldValueStream;

        if( IsInitialized() )
        {
            lOldValueStream << PerformValue( aIndex );
        }

        if( !IsInitialized() || lNewValueStream.str() != lOldValueStream.str() )
        {
            // Must put a 0.5 offset to have rounding instead of truncating.
            const int32_t lRaw = static_cast<int32_t>( aValue * mScale + ( aValue >= 0 ? 0.5f : -0.5f ) );

            PerformSetRawValue( aIndex, lRaw );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarCore::LdFloatProperty::PerformForceValue( size_t aIndex, float aValue )
///
/// \brief  Force value
///
/// \param   aIndex  Index of value to change.
/// \param   aValue  The new value.
///
/// \exception std::out_of_range Value outside the limits or invalid property count
///
/// \author David Levy
/// \date   March 2019
////////////////////////////////////////////////////////////////////////////////////////////////////
void LeddarCore::LdFloatProperty::PerformForceValue( size_t aIndex, float aValue )
{
    LeddarUtils::LtScope<bool> lForceEdit( &mCheckEditable, true );
    mCheckEditable = false;
    PerformSetValue( aIndex, aValue );
}

// *****************************************************************************
// Function: LdFloatProperty::PerformSetStringValue
//
/// \brief   Property writer for the value as text.
///
/// The text must be valid as a floating point value. The value will only
/// be changed if the new text, once standardized is different than the
/// current text value (so changes must be seen within the number of
/// decimals configured).
///
/// \param   aIndex  Index of value to write.
/// \param   aValue  The new value.
///
/// \exception std::invalid_argument No conversion could be be performed ( from std::stof )
/// \exception std::out_of_range Value out of range ( from std::stof )
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************
void LeddarCore::LdFloatProperty::PerformSetStringValue( size_t aIndex, const std::string &aValue )
{
    CanEdit();
    std::string lCurrent = "";

    if( IsInitialized() )
        lCurrent = PerformGetStringValue( aIndex );

    if( !IsInitialized() || lCurrent != aValue )
    {
        float lValue;
        std::stringstream lStreamValue( aValue );
        lStreamValue.exceptions( std::ios_base::failbit | std::ios_base::badbit );
        lStreamValue >> lValue;

        // Re-normalized and verify if really different (for example
        // if the current value is 1.00 and aValue contains "1", once
        // normalized it will still be 1.00 so no differences).
        std::ostringstream lNormalizedStream;
        lNormalizedStream << lValue;

        if( lNormalizedStream.str() != lCurrent )
        {
            PerformSetValue( aIndex, lValue );
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarCore::LdFloatProperty::PerformForceStringValue( size_t aIndex, const std::string &aValue )
///
/// \brief  Force string value
///
/// \param   aIndex  Index of value to write.
/// \param   aValue  The new value.
///
/// \exception std::invalid_argument No conversion could be be performed ( from std::stof )
/// \exception std::out_of_range Value out of range ( from std::stof )
///
/// \author David Levy
/// \date   March 2019
////////////////////////////////////////////////////////////////////////////////////////////////////
void LeddarCore::LdFloatProperty::PerformForceStringValue( size_t aIndex, const std::string &aValue )
{
    LeddarUtils::LtScope<bool> lForceEdit( &mCheckEditable, true );
    mCheckEditable = false;
    PerformSetStringValue( aIndex, aValue );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn void LeddarCore::LdFloatProperty::PerformSetRawStorage( uint8_t *aBuffer, size_t aCount, uint32_t aSize )
///
/// \brief  Set storage directly in memory
///
/// \param [in,out] aBuffer Buffer to copy.
/// \param          aCount  Number of element in the buffer.
/// \param          aSize   Size of each element in the buffer.
///
/// \author David L�vy
/// \date   November 2020
////////////////////////////////////////////////////////////////////////////////////////////////////
void LeddarCore::LdFloatProperty::PerformSetRawStorage( uint8_t *aBuffer, size_t aCount, uint32_t aSize )
{
    if( aSize == 4 || aSize == PerformStride() )
        LdProperty::PerformSetRawStorage( aBuffer, aCount, aSize );
    else
    {
        CanEdit();

        if( PerformCount() != aCount )
        {
            PerformSetCount( aCount );
        }

        if( aSize > sizeof( uint64_t ) )
        {
            throw std::invalid_argument( "Unable to SetRawStorage, invalid size: " + LeddarUtils::LtStringUtils::IntToString( aSize ) +
                                         " id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
        }

        for( uint32_t i = 0; i < aCount; i++ )
        {
            int64_t lValue( 0 );

            if( aSize == sizeof( int8_t ) )
            {
                lValue = reinterpret_cast<int8_t *>( aBuffer )[i];
            }
            else if( aSize == sizeof( int16_t ) )
            {
                lValue = reinterpret_cast<int16_t *>( aBuffer )[i];
            }
            else if( aSize == sizeof( int64_t ) )
            {
                lValue = reinterpret_cast<int64_t *>( aBuffer )[i];
            }
            else
            {
                throw std::logic_error( "Couldnt set storage value - Invalid size: " + LeddarUtils::LtStringUtils::IntToString( aSize ) +
                                        " id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
            }

            if( mStride == sizeof( int8_t ) )
            {
                reinterpret_cast<int8_t *>( &Storage()[0] )[i] = static_cast<int8_t>( lValue );
            }
            else if( mStride == sizeof( int16_t ) )
            {
                reinterpret_cast<int16_t *>( &Storage()[0] )[i] = static_cast<int16_t>( lValue );
            }
            else if( mStride == sizeof( int32_t ) )
            {
                reinterpret_cast<int32_t *>( &Storage()[0] )[i] = static_cast<int32_t>( lValue );
            }
            else if( mStride == sizeof( int64_t ) )
            {
                reinterpret_cast<int64_t *>( &Storage()[0] )[i] = static_cast<int64_t>( lValue );
            }
            else
            {
                throw std::logic_error( "Couldnt set storage value - Invalid stride: " + LeddarUtils::LtStringUtils::IntToString( mStride ) +
                                        " id: " + LeddarUtils::LtStringUtils::IntToString( PerformGetId(), 16 ) );
            }
        }
    }

    SetInitialized( true );
    EmitSignal( LdObject::VALUE_CHANGED );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// \fn int32_t LeddarCore::LdFloatProperty::PerformRawValue( size_t aIndex ) const
///
/// \brief  Gets the raw value (not scaled)
///
/// \exception  std::logic_error    Raised when a logic error condition occurs.
///
/// \param  aIndex  Index of the value to get.
///
/// \returns    An int32_t.
///
/// \author David L�vy
/// \date   February 2021
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t LeddarCore::LdFloatProperty::PerformRawValue( size_t aIndex ) const
{
    switch( PerformStride() )
    {
    case sizeof( int8_t ):
        return reinterpret_cast<const int8_t *>( CStorage() )[aIndex];
        break;
    case sizeof( int16_t ):
        return reinterpret_cast<const int16_t *>( CStorage() )[aIndex];
        break;
    case sizeof( int32_t ):
        return reinterpret_cast<const int32_t *>( CStorage() )[aIndex];
        break;
    default:
        throw std::logic_error( "Raw value only valid with size 1, 2 or 4" );
    }
}