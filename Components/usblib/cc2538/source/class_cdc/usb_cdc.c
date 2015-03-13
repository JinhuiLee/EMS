//*****************************************************************************
//! @file       usb_cdc.c
//! @brief      Application support for the HID class.
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


/******************************************************************************
 * INCLUDES
 */
#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"
#include "usb_in_buffer.h"
#include "usb_out_buffer.h"


/******************************************************************************
 * PROTOTYPES
 */
extern USB_EPIN_RINGBUFFER_DATA usbCdcInBufferData;
extern USB_EPOUT_RINGBUFFER_DATA usbCdcOutBufferData;


USBCDC_DATA usbCdcData;




/**************************************************************************//**
* @brief        USB UART init function.
*               - Set initial line decoding to 8/NONE/1.
*               - Initialise the USB Firmware Library and the USB controller.
*
* @param        none
*
* @return       none
*/
void usbCdcInit(uint32_t baudrate)
{

    // Initialize line coding
    usbCdcData.lineCoding.dteRate    = baudrate;
    usbCdcData.lineCoding.charFormat = USBCDC_CHAR_FORMAT_1_STOP_BIT;
    usbCdcData.lineCoding.parityType = USBCDC_PARITY_TYPE_NONE;
    usbCdcData.lineCoding.dataBits   = 8;

    // Initialise hardware flow control
    usbCdcData.rts = 0; // TRUE when DCE connected
    usbCdcData.cts = 1; // Indicate CTS to DCE (here handled internally as CDC does not directly support CTC).

    // Initialize the USB library
    usbfwInit();

    // Initialize the USB interrupt handler with bit mask containing all processed USBIRQ events
    usbirqInit(USBIRQ_EVENT_RESET | USBIRQ_EVENT_SETUP | USBIRQ_EVENT_SUSPEND | USBIRQ_EVENT_RESUME);

    // Activate the USB D+ pull-up resistor
    UsbDplusPullUpEnable();

} // usbCdcInit




/**************************************************************************//**
* @brief        The USB UART main task function. Must be called from the
*               applications main loop.
*
* @param        none
*
* @return       none
*/
void usbCdcProcessEvents(void)
{

    // If there are any unprocessed events
    if(USBIRQ_GET_EVENT_MASK() & (USBIRQ_EVENT_RESUME | USBIRQ_EVENT_RESET | USBIRQ_EVENT_SUSPEND | USBIRQ_EVENT_SETUP))
    {

        // Handle USB resume
        if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESUME)
        {
            USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESUME);
        }

        // Handle USB reset
        if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_RESET)
        {
            USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_RESET);
            usbfwResetHandler();
        }

        // Handle USB suspend
        if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SUSPEND)
        {
            USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SUSPEND);
            usbsuspEnter();
        }

        // Handle USB packets on EP0
        if(USBIRQ_GET_EVENT_MASK() & USBIRQ_EVENT_SETUP)
        {
            USBIRQ_CLEAR_EVENTS(USBIRQ_EVENT_SETUP);
            usbfwSetupHandler();
        }
    }

    // Handle COM port data transfer
    if(usbCdcData.cts && usbCdcData.rts)
    {

        // Process OUT data
        usbobufPollEndpoint(&usbCdcOutBufferData);

        // Process IN data
        usbibufPollEndpoint(&usbCdcInBufferData);
    }

} // usbCdcProcessEvents
