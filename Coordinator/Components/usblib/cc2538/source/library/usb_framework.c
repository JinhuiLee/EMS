//*****************************************************************************
//! @file       usb_framework.c
//! @brief      USB library common functionality.
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

/// \addtogroup module_usb_framework
/// @{

//
/// Modifies the definition of EXTERN so that global variables are declared by the header files
//
#define USBFRAMEWORK_C

#include "usb_firmware_library_headers.h"


//
/// Function pointer type for "void func(void)"
//
typedef void (*USBFW_VFPTR)(void);

//
/// Function pointer used by usbfwSetupHandler()
//
static USBFW_VFPTR pProcessFunc;


/** \brief Initializes the USB framework
 *
 * This function should be called when the microcontroller is ready to accept USB traffic. It enables the
 * USB peripheral unit and enables the pull-up resistor on the D+ line.
 */
void usbfwInit(void)
{
    //
    // Set default values
    //
    usbfwData.selfPowered = (usbdpGetConfigurationDesc(1, 0)->bmAttributes & 0x40) ? true : false;
    usbfwData.remoteWakeup = false;
    usbfwResetHandler();

    //
    // Enable the USB peripheral unit
    //
    UsbEnable();

    //
    // Allow the USB pad IP to to enter standby
    //
    HWREG(CCTEST_USBCTRL) = CCTEST_USBCTRL_USB_STB_M;

}


/** \brief Handles USB reset signalling
 *
 * This function should be called, either from the USB interrupt or the main loop, when the \c USBCIF.RST
 * flag has been set. Keep in mind that all bits in \c USBCIF register are cleared when the register is
 * read. The function puts the device into the default state (not yet addressed), and puts all endpoints
 * (except EP0) into the \ref EP_HALT state
 */
void usbfwResetHandler(void)
{
    //
    // Reset the USB state
    //    
    usbfwData.usbState = DEV_DEFAULT;
    usbfwData.configurationValue = 0;

    //
    // Reset all endpoints
    //
    usbfwData.ep0Status = EP_IDLE;
    usbfwSetAllEpStatus(EP_HALT);

    //
    // Reset last function pointer
    //
    pProcessFunc = NULL;

}


/** \brief USB Setup Handler
 *
 * This function should be called either from the USB interrupt or the main loop when the
 * \c USB_IIF.EP0IF flag has been set. A detailed description of the framework is found in the
 * \ref section_setup_handler_usage section.
 */
void usbfwSetupHandler(void)
{
    uint32_t controlReg;
    uint32_t oldEndpoint;
    uint32_t bytesNow;

    //
    // Save the old index setting, then select endpoint 0 and fetch the control register
    //
    oldEndpoint = USBFW_GET_SELECTED_ENDPOINT();
    USBFW_SELECT_ENDPOINT(0);
    controlReg = HWREG(USB_CS0);

    //
    // Update the USB device address after the status stage
    //
    if(usbfwData.ep0Status == EP_ADDRESS)
    {
        if(!(controlReg & USB_CS0_OUTPKTRDY_M))
        {
            uint8_t address = usbSetupHeader.valueLsb;
            HWREG(USB_ADDR) = address;
            if(usbfwData.usbState < DEV_CONFIGURED)
            {
                if(address)
                {
                    usbfwData.usbState = DEV_ADDRESS;
                }
                else
                {
                    usbfwData.usbState = DEV_DEFAULT;
                }
            }
        }
        usbfwData.ep0Status = EP_IDLE;
    }

    //
    // A STALL handshake was transmitted to the host
    //
    if(controlReg & USB_CS0_SENTSTALL_M)
    {
        HWREG(USB_CS0) = 0x00;
        usbfwData.ep0Status = EP_IDLE;
    }

    //
    // The last transfer was ended prematurely by a new SETUP packet
    //
    if(controlReg & USB_CS0_SETUPEND_M)
    {
        HWREG(USB_CS0) = USB_CS0_CLRSETUPEND_M;
        usbfwData.ep0Status = EP_CANCEL;
        if(pProcessFunc)
        {
            pProcessFunc();
        }
        usbfwData.ep0Status = EP_IDLE;
    }

    //
    // Receive OUT packets
    //
    if(usbfwData.ep0Status == EP_RX)
    {
        if(controlReg & USB_CS0_OUTPKTRDY_M)
        {
            //
            // Read FIFO
            //
            uint32_t bytesNow = HWREG(USB_CNT0);
            usbfwReadFifo(USB_F0, bytesNow, usbSetupData.pBuffer);
            usbSetupData.bytesLeft -= bytesNow;
            usbSetupData.pBuffer = ((uint8_t*) usbSetupData.pBuffer) + bytesNow;

            //
            // Arm the endpoint
            //
            if(usbSetupData.bytesLeft)
            {
                HWREG(USB_CS0) = USB_CS0_CLROUTPKTRDY_M;
            }
            else
            {
                HWREG(USB_CS0) = USB_CS0_CLROUTPKTRDY_M | USB_CS0_DATAEND_M;
            }

            //
            // Make a call to the appropriate request handler when done
            //
            if(usbSetupData.bytesLeft == 0)
            {
                if(pProcessFunc)
                {
                    pProcessFunc();
                }
                usbfwData.ep0Status = EP_IDLE;
            }
        }

        //
        // Return here since nothing more will happen until the next interrupt
        //
        USBFW_SELECT_ENDPOINT(oldEndpoint);
        return;

        //
        // Let the application handle the reception
        //
    }
    else if(usbfwData.ep0Status == EP_MANUAL_RX)
    {
        if(pProcessFunc)
        {
            pProcessFunc();
        }
    }

    //
    // Receive SETUP header
    //
    if(usbfwData.ep0Status == EP_IDLE)
    {
        if(controlReg & USB_CS0_OUTPKTRDY_M)
        {
            usbfwReadFifo(USB_F0, 8, &usbSetupHeader);

            //
            // Handle control transfers individually
            //
            pProcessFunc = NULL;
            switch(usbSetupHeader.requestType & (RT_MASK_TYPE | RT_MASK_DIR))
            {
                //
                // Standard requests without data or with data from the host (OUT)
                //
            case RT_STD_OUT:
                switch(usbSetupHeader.request)
                {
                case USBSR_REQ_SET_ADDRESS:
                    usbsrSetAddress();
                    break;
                case USBSR_REQ_SET_FEATURE:
                    usbsrSetFeature();
                    break;
                case USBSR_REQ_CLEAR_FEATURE:
                    usbsrClearFeature();
                    break;
                case USBSR_REQ_SET_CONFIGURATION:
                    usbsrSetConfiguration();
                    break;
                case USBSR_REQ_SET_INTERFACE:
                    usbsrSetInterface();
                    break;
                case USBSR_REQ_SET_DESCRIPTOR:
                    pProcessFunc = usbsrHookSetDescriptor;
                    usbsrHookSetDescriptor();
                    break;
                default:
                    usbfwData.ep0Status = EP_STALL;
                    break;
                }
                break;

                //
                // Standard requests with data to the host (IN)
                //
            case RT_STD_IN:
                switch(usbSetupHeader.request)
                {
                case USBSR_REQ_GET_STATUS:
                    usbsrGetStatus();
                    break;
                case USBSR_REQ_GET_DESCRIPTOR:
                    usbsrGetDescriptor();
                    break;
                case USBSR_REQ_GET_CONFIGURATION:
                    usbsrGetConfiguration();
                    break;
                case USBSR_REQ_GET_INTERFACE:
                    usbsrGetInterface();
                    break;
                case USBSR_REQ_SYNCH_FRAME:
                    pProcessFunc = usbsrHookSynchFrame;
                    usbsrHookSynchFrame();
                    break;
                default:
                    usbfwData.ep0Status = EP_STALL;
                    break;
                }
                break;

                //
                // Vendor requests
                //
            case RT_VEND_OUT:
                pProcessFunc = usbvrHookProcessOut;
                usbvrHookProcessOut();
                break;
            case RT_VEND_IN:
                pProcessFunc = usbvrHookProcessIn;
                usbvrHookProcessIn();
                break;

                //
                // Class requests
                //
            case RT_CLASS_OUT:
                pProcessFunc = usbcrHookProcessOut;
                usbcrHookProcessOut();
                break;
            case RT_CLASS_IN:
                pProcessFunc = usbcrHookProcessIn;
                usbcrHookProcessIn();
                break;

                //
                // Unrecognized request: Stall the endpoint
                //
            default:
                usbfwData.ep0Status = EP_STALL;
                break;
            }

            //
            // Arm/stall the endpoint
            //
            if(usbfwData.ep0Status == EP_STALL)
            {
                HWREG(USB_CS0) = USB_CS0_CLROUTPKTRDY_M | USB_CS0_SENDSTALL_M;
            }
            else if((usbfwData.ep0Status == EP_TX) || (usbfwData.ep0Status == EP_RX))
            {
                HWREG(USB_CS0) = USB_CS0_CLROUTPKTRDY_M;
            }
            else
            {
                HWREG(USB_CS0) = USB_CS0_CLROUTPKTRDY_M | USB_CS0_DATAEND_M;
            }
        }
    }

    //
    // Transmit IN packets
    //
    if(usbfwData.ep0Status == EP_TX)
    {
        controlReg = USB_CS0_INPKTRDY_M;

        //
        // The last frame should contain 0 to (EP0_PACKET_SIZE - 1) bytes
        //
        if(usbSetupData.bytesLeft < USB_EP0_PACKET_SIZE)
        {
            bytesNow = usbSetupData.bytesLeft;
            controlReg |= USB_CS0_DATAEND_M;
        }
        else
        {
            //
            // All other packets should have the maximum length
            //
            bytesNow = USB_EP0_PACKET_SIZE;
        }

        //
        // Load the FIFO and move the pointer
        //
        usbfwWriteFifo(USB_F0, bytesNow, usbSetupData.pBuffer);
        usbSetupData.pBuffer = ((uint8_t*) usbSetupData.pBuffer) + bytesNow;
        usbSetupData.bytesLeft -= bytesNow;

        //
        // Arm the FIFO (even for a zero-length packet)
        //
        HWREG(USB_CS0) = controlReg;

        //
        // Make a call to the appropriate request handler when done
        //
        if(bytesNow < USB_EP0_PACKET_SIZE)
        {
            if(pProcessFunc)
            {
                pProcessFunc();
            }
            usbfwData.ep0Status = EP_IDLE;
        }

        //
        // Let the application handle the transmission
        //
    }
    else if(usbfwData.ep0Status == EP_MANUAL_TX)
    {
        if(pProcessFunc)
        {
            pProcessFunc();
        }
    }

    //
    // Restore the old index setting
    //
    USBFW_SELECT_ENDPOINT(oldEndpoint);
}


/** \brief Changes the state of endpoint 1-5 IN/OUT
 *
 * This is an internal function used by the library.
 *
 * \param[in]       status
 *     The new status for each endpoint
 */
void usbfwSetAllEpStatus(EP_STATUS status)
{
    for(int n = 0; n < sizeof(usbfwData.pEpInStatus); n++)
    {
        usbfwData.pEpInStatus[n] = status;
    }
    for(int n = 0; n < sizeof(usbfwData.pEpOutStatus); n++)
    {
        usbfwData.pEpOutStatus[n] = status;
    }
}


/** \brief Reads from the selected OUT endpoint FIFO, without using DMA
 *
 * The FIFO must be re-armed after reading it empty (using the \ref USBFW_ARM_OUT_ENDPOINT() macro). This
 * is not necessary when flushing the FIFO.
 *
 * \param[in]       fifoReg
 *     FIFO register address (USB_Fx, where 'x' is between 0 and 5, e.g. USB_F4 for Endpoint 4)
 * \param[in]       count
 *     The number of bytes to read
 * \param[in]       *pData
 *     Pointer to the storage location for the read data (in any memory space)
 */
void usbfwReadFifo(uint32_t fifoReg, uint32_t count, void* pData)
{
    uint8_t* pTemp = pData;
    while(count--)
    {
        *(pTemp++) = HWREG(fifoReg);
    }
}


/** \brief Writes to the selected IN endpoint FIFO, without using DMA
 *
 * Note that the FIFO must be armed in order to be transmitted (using the \ref USBFW_ARM_IN_ENDPOINT()
 * macro).
 *
 * \param[in]       fifoReg
 *     FIFO register address (USB_Fx, where 'x' is between 0 and 5, e.g. USB_F4 for Endpoint 4)
 * \param[in]       count
 *     The number of bytes to write
 * \param[in]       *pData
 *     Pointer to the data to be written
 */
void usbfwWriteFifo(uint32_t fifoReg, uint32_t count, void *pData)
{
    uint8_t* pTemp = pData;
    while(count--)
    {
        HWREG(fifoReg) = *(pTemp++);
    }
}


//@}
