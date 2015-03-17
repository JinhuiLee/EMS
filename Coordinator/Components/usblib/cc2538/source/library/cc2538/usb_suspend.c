//*****************************************************************************
//! @file       usb_suspend.c
//! @brief      USB library suspend and wakeup functionality.
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

/// \addtogroup module_usb_suspend
/// @{
#include "usb_firmware_library_headers.h"
#include "hw_nvic.h"




/** \brief Puts the chip into low-power mode with XOSC off and full retention during USB suspend.
 *
 * This function must be called from main context upon the reception of a \ref USBIRQ_EVENT_SUSPEND
 * event. To comply with the USB specification, this must happen within 7 ms after the event occurs. The
 * chip will stay in this function in low-power mode until activity resumes on the bus or remote wakeup
 * is generated using \ref usbsuspDoRemoteWakeup().
 */
void usbsuspEnter(void)
{
    //
    // Setup registers
    //
    HWREG(GPIO_D_BASE + GPIO_O_USB_IRQ_ACK)     = GPIO_USB_IRQ_ACK_USBACK;
    HWREG(GPIO_D_BASE + GPIO_O_IRQ_DETECT_ACK)  = GPIO_IRQ_DETECT_ACK_PDIACK7;
    HWREG(GPIO_D_BASE + GPIO_O_PI_IEN)         |= GPIO_PI_IEN_PDIEN7;
    HWREG(SYS_CTRL_IWE)                        |= SYS_CTRL_IWE_USB_IWE;

    //
    // Enter PM1 only when the USB IP still is in suspend
    //
    if(usbirqData.inSuspend)
    {
        //
        // Let the application know that we're entering suspend mode
        //
        usbsuspHookEnteringSuspend(usbfwData.remoteWakeup);
        
        //
        // Disable the USB PLL and enter PM1
        //
        usbirqData.waitForPm1Exit = true;
        UsbPllDisable();
        IntPendClear(INT_GPIOD);
        IntEnable(INT_GPIOD);
        SysCtrlPowerModeSet(SYS_CTRL_PM_1);
        while(usbirqData.waitForPm1Exit)
        {
            HWREG(NVIC_SYS_CTRL) |= NVIC_SYS_CTRL_SLEEPDEEP;
            CPUwfe();
            HWREG(NVIC_SYS_CTRL) &= ~NVIC_SYS_CTRL_SLEEPDEEP;
        }
        UsbPllEnable();

        //
        // Let the application know that we've exited suspend mode
        //
        usbsuspHookExitingSuspend();
    }

    HWREG(SYS_CTRL_IWE) &= ~SYS_CTRL_IWE_USB_IWE;

}




/** \brief Attempts USB remote wakeup.
 *
 * This function can be called from interrupt context while the USB device is suspend mode. If the device
 * is privileged to do so (see \c usbfwData.remoteWakeup and the \ref USBSR_EVENT_REMOTE_WAKEUP_ENABLED
 * and \ref USBSR_EVENT_REMOTE_WAKEUP_DISABLED events), remote wakeup will be performed. Note that this
 * function will block for 10 ms while the resume signal is set on the bus.
 *
 * \return
 *     \c TRUE if the remote wakeup was performed (the privilege had been granted), otherwise \c FALSE
 *     (the device is still in suspend mode, or was not suspended in the first place).
 */
uint8_t usbsuspDoRemoteWakeup(void)
{
    uint8_t remoteWakeupPerformed = false;

    //
    // Disable interrupts while performing remote wakeup
    //
    bool intDisabled = IntMasterDisable();
    if(usbirqData.inSuspend)
    {
        //
        // Ensure that the XOSC is stable
        //
        UsbWaitForXoscStable();

        //
        // Take the USB pad IP out of standby early to avoid D+/D- crossing error upon resume signaling
        //
        HWREG(CCTEST_USBCTRL) = 0x00000000; // Clear CCTEST_USBCTRL_USB_STB_M

        //
        // Enable the USB PLL
        //
        UsbPllEnable();

        //
        // Perform remote wakeup by holding the USB resume signal for 10 ms
        //
        HWREG(USB_POW) |= USB_POW_RESUME_M;
        SysCtrlDelay((SysCtrlClockGet() / 3000) * 10);
        HWREG(USB_POW) &= ~USB_POW_RESUME_M;

        //
        // Allow the USB pad IP to to enter standby
        //
        HWREG(CCTEST_USBCTRL) = CCTEST_USBCTRL_USB_STB_M;

        //
        // If waiting to exit from the usbsuspEnter() loop waiting on usbirqData.waitForPm1Exit to be
        // cleared, we disable and fake such an the wakeup interrupt from here
        //
        if(usbirqData.waitForPm1Exit)
        {
            HWREG(GPIO_D_BASE + GPIO_O_USB_IRQ_ACK)     = GPIO_USB_IRQ_ACK_USBACK;
            HWREG(GPIO_D_BASE + GPIO_O_IRQ_DETECT_ACK)  = GPIO_IRQ_DETECT_ACK_PDIACK7;
            CPUsev();
            usbirqData.waitForPm1Exit = false;
            IntDisable(INT_GPIOD);
            IntPendClear(INT_GPIOD);
        }

        remoteWakeupPerformed = true;
    }

    //
    // Re-enable interrupts
    //
    if(!intDisabled)
    {
        IntMasterEnable();
    }

    return remoteWakeupPerformed;

}


//@}
