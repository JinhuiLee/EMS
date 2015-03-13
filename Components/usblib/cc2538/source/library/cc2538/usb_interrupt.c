//*****************************************************************************
//! @file       usb_interrupt.c
//! @brief      USB library interrupt initialisation and ISR.
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

/// \addtogroup module_usb_interrupt
/// @{
#include "usb_firmware_library_headers.h"


// Internal functions
static void usbirqFunctionEventHandler(void);
static void usbirqWakeupEventHandler(void);


/** \brief Initializes the \ref module_usb_interrupt module
 *
 * This function shall be called after the \ref module_usb_framework module has been initialized.
 * Use interrupt priority/preemption control to adjust the priority of the USB interrupt relative to
 * other interrupts.
 *
 * \param[in]       irqMask
 *     A bit mask containing \c USBIRQ_EVENT bits for all events that shall be reported
 */
void usbirqInit(uint16_t irqMask)
{
    //
    // Initialize variables
    //
    usbirqData.eventMask = 0x0000;
    usbirqData.irqMask   = irqMask;
    usbirqData.inSuspend = false;

    //
    // Initialize interrupt mask registers
    //
    HWREG(USB_CIE) = irqMask;
    HWREG(USB_IIE) = irqMask >> 4;
    HWREG(USB_OIE) = (irqMask >> 9) & 0x3E;

    //
    // Register the wake-up interrupt
    //
    IntRegister(INT_GPIOD, usbirqWakeupEventHandler);

    //
    // Register and enable the function interrupt
    //
    IntRegister(INT_USB2538, usbirqFunctionEventHandler);
    IntEnable(INT_USB2538);

}




/** \brief USB function interrupt handler
 *
 * Clears the USB function interrupt flags and converts all pending USB function interrupts into events.
 */
static void usbirqFunctionEventHandler(void)
{
    uint16_t eventMask;
    uint32_t usbcif;

    // Ensure that the XOSC is stable
    UsbWaitForXoscStable();

    // Ensure that the USB PLL is enabled and stable before accessing registers
    UsbPllEnable();

    usbcif = HWREG(USB_CIF);
    if(usbcif & USB_CIF_RSTIF_M)
    {

        // All interrupts (except suspend) are by default enabled by hardware, so re-initialize the
        // enable bits to avoid unwanted interrupts
        HWREG(USB_CIE) = usbirqData.irqMask;
        HWREG(USB_IIE) = usbirqData.irqMask >> 4;
        HWREG(USB_OIE) = (usbirqData.irqMask >> 9) & 0x3E;

        // Enable suspend mode when suspend signaling is detected on the bus
        HWREG(USB_POW) |= USB_POW_SUSPENDEN_M;
    }

    // If we get a suspend event, we should always enter suspend mode. We must however be sure that we
    // exit the suspend loop upon resume or reset signaling.
    if(usbcif & USB_CIF_SUSPENDIF)
    {
        usbirqData.inSuspend = true;
    }
    if(usbcif & (USB_CIF_RSTIF | USB_CIF_RESUMEIF))
    {
        usbirqData.inSuspend = false;
    }

    // Add new events to existing, not yet processed (this can be done without critical section because
    // processing is done at a lower preemption level
    eventMask  = usbcif;
    eventMask |= HWREG(USB_IIF) << 4;
    eventMask |= HWREG(USB_OIF) << 9;
    usbirqData.eventMask |= eventMask;

    // Allow for high-priority event processing in interrupt context
    usbirqHookProcessEvents();

    // Clear the CPU interrupt
    IntPendClear(INT_USB2538);

} // usbirqFunctionEventHandler




/** \brief USB wakeup interrupt handler
 *
 * If we've woken on falling edge on the USB D+ line, notify the currently waiting \ref usbsuspEnter(),
 * which has been called from main context. This resumes execution in main context.
 */
static void usbirqWakeupEventHandler(void)
{

    // If resume signaling has been detected, notify usbsuspEnter()
    if(HWREG(GPIO_D_BASE + GPIO_O_USB_IRQ_ACK))
    {
        HWREG(GPIO_D_BASE + GPIO_O_USB_IRQ_ACK)     = GPIO_USB_IRQ_ACK_USBACK;
        HWREG(GPIO_D_BASE + GPIO_O_IRQ_DETECT_ACK)  = GPIO_IRQ_DETECT_ACK_PDIACK7;
        CPUsev();
        usbirqData.waitForPm1Exit = false;
        IntDisable(INT_GPIOD);
    }

    // Clear the CPU interrupt
    IntPendClear(INT_GPIOD);

} // usbirqWakeupEventHandler


//@}
