//*****************************************************************************
//! @file       als_srf06eb.c
//! @brief      SmartRF06EB device driver implementation for SFH5711 ambient
//!             light sensor.
//!
//! Revised     $Date: 2013-04-11 19:57:23 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9711 $
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef ALS_EXCLUDE


/**************************************************************************//**
* @addtogroup als_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"
#include "als_sfh5711.h"
#include "gpio.h"           // Access to driverlib GPIO functions
#include "adc.h"            // Access to SOC_ADC functions

/******************************************************************************
* DEFINES
*/


/******************************************************************************
* LOCAL VARIABLES AND FUNCTION PROTOTYPES
*/


/******************************************************************************
* FUNCTIONS
*/
/**************************************************************************//**
* @brief    This function initializes the ALS. The sensor is powered up and the
*           onboard ADC is configured.
*
*
* @return   None
******************************************************************************/
void
alsInit(void)
{
    //
    // Set ALS power pin (output high)
    //
    GPIOPinTypeGPIOOutput(BSP_ALS_PWR_BASE, BSP_ALS_PWR);
    GPIOPinWrite(BSP_ALS_PWR_BASE, BSP_ALS_PWR, BSP_ALS_PWR);

    //
    // Configure ALS output pin as hw controlled, no drive (same as for timer)
    //
    GPIOPinTypeTimer(BSP_ALS_OUT_BASE, BSP_ALS_OUT);

    //
    // Configure ADC, Internal reference, 512 decimation rate (12bit)
    //
    SOCADCSingleConfigure(SOCADC_12_BIT, SOCADC_REF_INTERNAL);
}


/**************************************************************************//**
* @brief    This function uninitializes the ALS. This function assumes that
*           the ALS power pin has already been configured as output using,
*           for example, alsInit().
*
* @return   None
******************************************************************************/
void
alsUninit(void)
{
    //
    // Power down the light sensor
    //
    GPIOPinWrite(BSP_ALS_PWR_BASE, BSP_ALS_PWR, 0);   // Low
}


/**************************************************************************//**
* @brief    This function triggers and returns ADC conversion from the ALS
*           output. A 12-bit ADC conversion results in a value of [0, 4095].
*
* @return   Returns the value read from the light sensor
******************************************************************************/
uint16_t
alsRead(void)
{
    uint16_t ui1Dummy;

    //
    // Trigger single conversion on AIN6 (connected to LV_ALS_OUT).
    //
    SOCADCSingleStart(SOCADC_AIN6);

    //
    // Wait until conversion is completed
    //
    while(!SOCADCEndOfCOnversionGet())
    {
    }

    //
    // Get data and shift down based on decimation rate
    //
    ui1Dummy = SOCADCDataGet();
    return(ui1Dummy >> SOCADC_12_BIT_RSHIFT);
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef ALS_EXCLUDE
