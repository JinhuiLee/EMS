//*****************************************************************************
//! @file       usb.c
//! @brief      USB low level function implementation for CC2538.
//!
//! Revised     $Date: 2013-04-08 13:14:23 +0200 (Mon, 08 Apr 2013) $
//! Revision    $Revision: 9658 $
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

#include "usb.h"
#include "debug.h"
#include "interrupt.h"
#include "gpio.h"


void UsbEnable(void)
{
    //
    // Reset USB
    //
    HWREG(USB_CTRL) = 0x00;
    
    //
    // Enable USB
    //
    HWREG(USB_CTRL) |= 0x00000001;
    
    //
    // Enable USB PLL
    //
    UsbPllEnable();
}


void UsbDplusPullUpEnable(void)
{
    GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 1);
    GPIODirModeSet(GPIO_C_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT);
}


void UsbPllEnable(void)
{
    HWREG(USB_CTRL) |= 0x00000002;
    
    //
    // Poll PLL_LOCKED in USB_CTRL register until locked
    //
    while(!(HWREG(USB_CTRL) & 0x80));
}


void UsbPllDisable(void)
{
    HWREG(USB_CTRL) &= ~0x00000002;
}


void UsbWaitForXoscStable(void)
{
    while((HWREG(SYS_CTRL_CLOCK_STA) & (SYS_CTRL_CLOCK_STA_OSC | SYS_CTRL_CLOCK_STA_XOSC_STB)) != SYS_CTRL_CLOCK_STA_XOSC_STB);
}
