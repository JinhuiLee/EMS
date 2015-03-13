//*****************************************************************************
//! @file       usb_cdc_hooks.c
//! @brief      Contains the necessary hook functions for various USB request
//!             processing that is featured from the USB firmware library. Some
//!             functions are empty.
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

#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"


/** \brief This function processes USB class requests with OUT data phase.
 *
 * This function stalls USB endpoint 0 if the request is unsupported.
 */
void usbcrHookProcessOut(void)
{

    if(usbSetupHeader.request == USBCDC_REQ_SET_CONTROL_LINE_STATE)
    {
        if(usbfwData.ep0Status == EP_IDLE)
        {
            usbCdcData.rts = usbSetupHeader.value & 0x01;
        }

    }
    else if(usbSetupHeader.request == USBCDC_REQ_SET_LINE_CODING)
    {
        if(usbfwData.ep0Status == EP_IDLE)
        {
            if(usbSetupHeader.length == sizeof(USBCDC_LINE_CODING))
            {
                usbSetupData.pBuffer = &usbCdcData.lineCoding;
                usbSetupData.bytesLeft = sizeof(USBCDC_LINE_CODING);
                usbfwData.ep0Status = EP_RX;
            }
            else
            {
                usbfwData.ep0Status = EP_STALL;
            }
        }

    }
    else
    {
        usbfwData.ep0Status = EP_STALL;
    }

} // usbcrHookProcessOut


/** \brief This function processes USB class requests with IN data phase.
 *
 * This function stalls USB endpoint 0 if the request is unsupported.
 */
void usbcrHookProcessIn(void)
{

    if(usbSetupHeader.request == USBCDC_REQ_GET_LINE_CODING)
    {
        if(usbfwData.ep0Status == EP_IDLE)
        {
            usbSetupData.pBuffer = &usbCdcData.lineCoding;
            usbSetupData.bytesLeft = usbSetupHeader.length;
            usbfwData.ep0Status = EP_TX;
        }

    }
    else
    {
        usbfwData.ep0Status = EP_STALL;
    }

} // usbcrHookProcessIn


// ****************************************************************************
// Unused/unsupported hooks
// ****************************************************************************

void usbvrHookProcessOut(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbvrHookProcessIn(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbsrHookSetDescriptor(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbsrHookSynchFrame(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbsrHookClearFeature(void)
{
    usbfwData.ep0Status = EP_STALL;
}
void usbsrHookSetFeature(void)
{
    usbfwData.ep0Status = EP_STALL;
}
uint8_t usbsrHookModifyGetStatus(USB_SETUP_HEADER* pSetupHeader, uint8_t ep0Status, uint16_t* pStatus)
{
    return ep0Status;
}
void usbsrHookProcessEvent(uint8_t event, uint8_t index) {}
void usbirqHookProcessEvents(void) {}
