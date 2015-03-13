//*****************************************************************************
//! @file       usb_standard_request.c
//! @brief      Handle USB standard requests.
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

/// \addtogroup module_usb_standard_requests
/// @{
#include "usb_firmware_library_headers.h"




/** \brief Processes the \c GET_STATUS request (returns status for the specified recipient)
 *
 * The recipient field in \ref USB_SETUP_HEADER.requestType specifies the recipient. This is either
 * the (one and only) device, a specific interface or a specific endpoint. Some of the status bits can
 * be changed by the host with the \c SET_FEATURE and \c CLEAR_FEATURE requests.
 *
 * The usbsrHookModifyGetStatus() hook is always called after the standard processing is done, but before
 * the status is returned to the host. This allows the application to handle class- or vendor-specific
 * behavior.
 *
 * <b>Parameters</b>:
 * - VALUE: Always 0
 * - INDEX: Depends upon the recipient:
 *     - DEVICE: Always 0
 *     - INTERFACE: Interface number
 *     - ENDPOINT: Endpoint address
 * - LENGTH: Always 2
 *
 * <b>Data (IN)</b>:
 * Depends upon the recipient (the bit field illustrations are MSB first, LSB last):
 * - DEVICE: <tt>00000000.000000RS</tt>, where R(1) = DEVICE_REMOTE_WAKEUP and S(0) = SELF_POWERED
 * - INTERFACE: <tt>00000000.00000000</tt> (all bits are reserved)
 * - ENDPOINT: <tt>00000000.0000000H</tt>, where H(0) = ENDPOINT_HALT
 */
void usbsrGetStatus(void)
{
    uint8_t endpoint;
    static uint16_t status;

    //
    // Initialize the status word
    //
    status = 0x0000;

    //
    // Common sanity check
    //
    if(usbSetupHeader.value || usbSetupHeader.indexMsb || (usbSetupHeader.length != 2))
    {
        usbfwData.ep0Status = EP_STALL;

        //
        // Return status for device, interface, or endpoint
        //
    }
    else
    {
        switch(usbSetupHeader.requestType)
        {
            //
            // Device status:
            //     Bit 0: Self powered
            //     Bit 1: Remote wake-up allowed
            //
        case RT_IN_DEVICE:
            //
            // Sanity check
            //
            if(usbSetupHeader.indexLsb)
            {
                usbfwData.ep0Status = EP_STALL;
                
                //
                // Get the bit values from the USBFW_DATA struct
                //
            }
            else
            {
                //
                // Self powered?
                //
                status = usbfwData.selfPowered ? 0x0001 : 0x0000;
                
                //
                // Remote wakeup?
                //
                if(usbfwData.remoteWakeup)
                {
                    status |= 0x0002;
                }
            }
            break;

            //
            // Interface status:
            //     All bits are reserved
            //
        case RT_IN_INTERFACE:
            //
            // Sanity check
            //
            if(usbfwData.usbState != DEV_CONFIGURED)
            {
                usbfwData.ep0Status = EP_STALL;
            }
            else
            {
                //
                // Handled by inititalization
                //
            }
            break;

            //
            // Endpoint status:
            //     Bit 0: Endpoint halted
            //
        case RT_IN_ENDPOINT:
            endpoint = usbSetupHeader.indexLsb & 0x7F;

            //
            // Sanity check
            //
            if((usbfwData.usbState != DEV_CONFIGURED) || (endpoint > 5))
            {
                usbfwData.ep0Status = EP_STALL;

                //
                // Translate endpoint address to status index and return the status
                //
            }
            else
            {

                //
                // IN
                //
                if(usbSetupHeader.index & 0x80)
                {
                    status = (usbfwData.pEpInStatus[endpoint - 1] == EP_HALT) ? 0x0001 : 0x0000;
                    
                    //
                    // OUT
                    //
                }
                else
                {
                    status = (usbfwData.pEpOutStatus[endpoint - 1] == EP_HALT) ? 0x0001 : 0x0000;
                }
            }
            break;

        default:
            usbfwData.ep0Status = EP_STALL;
            break;
        }
        
        //
        // Pass it by the application so it can add more bits and/or change the status from EP_STALL to
        // EP_TX (for class-specific status values)
        //
        usbfwData.ep0Status = usbsrHookModifyGetStatus(&usbSetupHeader, usbfwData.ep0Status, &status);

        if(usbfwData.ep0Status != EP_STALL)
        {
            //
            // Send it
            //
            usbSetupData.pBuffer = &status;
            usbSetupData.bytesLeft = 2;
            usbfwData.ep0Status = EP_TX;
        }
    }

}




/** \brief Internal function used for the very similar \c SET_FEATURE and \c CLEAR_FEATURE requests
 *
 * This function either sets or clears the specified feature on the specified recipient.
 *
 * \param[in]       set
 *     When TRUE, the feature is set. When FALSE, the feature is cleared.
 *
 * \return
 *     TRUE if the selected feature is supported by the USB library. FALSE to indicate that
 *     \ref usbsrHookClearFeature() or \ref usbsrHookSetFeature() must be called.
 */
static uint8_t usbsrChangeFeature(uint8_t set)
{
    uint8_t endpoint;
    
    //
    // Sanity check
    //
    if(usbSetupHeader.length || ((usbfwData.usbState != DEV_CONFIGURED) && (usbSetupHeader.index != 0)))
    {
        usbfwData.ep0Status = EP_STALL;
        //
        // Handle based on recipient
        //
    }
    else
    {
        switch(usbSetupHeader.requestType & RT_MASK_RECIP)
        {
            //
            // Device
            //
        case RT_RECIP_DEV:
            
            //
            // Sanity check
            //
            if(usbSetupHeader.valueLsb != USBSR_FEATSEL_DEVICE_REMOTE_WAKEUP)
            {
                return false;
            }
            else
            {
                usbfwData.remoteWakeup = set;
                usbsrHookProcessEvent(set ? USBSR_EVENT_REMOTE_WAKEUP_ENABLED : USBSR_EVENT_REMOTE_WAKEUP_DISABLED, 0);
            }
            break;
            //
            // Interface
            //
        case RT_RECIP_IF:
            return false;

            //
            // Endpoint
            //
        case RT_RECIP_EP:
            endpoint = usbSetupHeader.indexLsb & 0x7F;
            
            //
            // Sanity check
            //
            if(usbSetupHeader.valueLsb != USBSR_FEATSEL_ENDPOINT_HALT)
            {
                return false;
            }
            else if(endpoint > 5)
            {
                usbfwData.ep0Status = EP_STALL;
            }
            else
            {
                USBFW_SELECT_ENDPOINT(endpoint);
                
                //
                // IN
                //
                if(usbSetupHeader.indexLsb & 0x80)
                {
                    HWREG(USB_CS0_CSIL) = set ? USB_CSIL_SENDSTALL_M : USB_CSIL_CLRDATATOG_M;
                    usbfwData.pEpInStatus[endpoint - 1] = set ? EP_HALT : EP_IDLE;
                    usbsrHookProcessEvent(set ? USBSR_EVENT_EPIN_STALL_SET : USBSR_EVENT_EPIN_STALL_CLEARED, endpoint);
                    
                    //
                    // OUT
                    //
                }
                else
                {
                    HWREG(USB_CSOL) = set ? USB_CSOL_SENDSTALL_M : USB_CSOL_CLRDATATOG_M;
                    usbfwData.pEpOutStatus[endpoint - 1] = set ? EP_HALT : EP_IDLE;
                    usbsrHookProcessEvent(set ? USBSR_EVENT_EPOUT_STALL_SET : USBSR_EVENT_EPOUT_STALL_CLEARED, endpoint);
                }
                USBFW_SELECT_ENDPOINT(0);
            }
            break;

        default:
            usbfwData.ep0Status = EP_STALL;
            break;
        }
    }
    return true;
}




/** \brief Processes the \c CLEAR_FEATURE request (clears or disables a specific feature)
 *
 * The feature selector value must be appropriate to the recipient.
 *
 * <b>Parameters</b>:
 * - VALUE: Feature selector:
 *     - \c USBSR_FEATSEL_DEVICE_REMOTE_WAKEUP(1): Disable remote wakeup
 *     - \c USBSR_FEATSEL_ENDPOINT_HALT(0): Clear the halt feature for the specified endpoint (not
 *       endpoint 0!)
 * - INDEX: Depends upon the recipient:
 *     - DEVICE: Always 0
 *     - INTERFACE: Interface number
 *     - ENDPOINT: Endpoint address
 * - LENGTH: Always 0
 */
void usbsrClearFeature(void)
{
    if(!usbsrChangeFeature(false))
    {
        usbsrHookClearFeature();
    }
}




/** \brief Processes the \c SET_FEATURE request (sets or enables a specific feature)
 *
 * The feature selector value must be appropriate to the recipient.
 *
 * <b>Parameters</b>:
 * - VALUE: Feature selector:
 *     - \c USBSR_FEATSEL_DEVICE_REMOTE_WAKEUP(1): Enable remote wakeup
 *     - \c USBSR_FEATSEL_ENDPOINT_HALT(0): Set the halt feature for the specified endpoint (not endpoint
 *       0!)
 * - INDEX: Depends upon the recipient:
 *     - DEVICE: Always 0
 *     - INTERFACE: Interface number
 *     - ENDPOINT: Endpoint address
 * - LENGTH: Always 0
 */
void usbsrSetFeature(void)
{
    if(!usbsrChangeFeature(true))
    {
        usbsrHookSetFeature();
    }
}




/** \brief Processes the \c SET_ADDRESS request (sets the device address for all future device
 * accesses)
 *
 * If the value is between 1 and 127 and the device is in the default state, it will enter the address
 * state upon successful completion of the \c SET_ADDRESS request's status stage. If it already is in the
 * address state, it starts to use the newly-specified address.
 *
 * If the value is 0 and the device is in the address state, it will enter the default state upon
 * successful completion of the \c SET_ADDRESS request's status stage. If it already is in the default
 * state, nothing happens.
 *
 * <b>Parameters</b>:
 * - VALUE: The device address (0-127)
 * - INDEX: Always 0
 * - LENGTH: Always 0
 */
void usbsrSetAddress(void)
{
    //
    // Sanity check
    //
    if(usbSetupHeader.index || usbSetupHeader.length || usbSetupHeader.valueMsb || (usbSetupHeader.valueLsb & 0x80))
    {
        usbfwData.ep0Status = EP_STALL;
    }
    else
    {
        //
        // Update the device address
        //
        usbfwData.ep0Status = EP_ADDRESS;
    }

}




/** \brief Processes the \c GET_DESCRIPTOR request (returns the specified USB descriptor)
 *
 * The descriptors are retrieved using either the \ref module_usb_descriptor_parser module's automatic
 * parsing functions (for device and configuration descriptors), or the manually generated
 * \ref pUsbDescriptorLut[] lookup table (for string and non-standard formatted descriptors). When the
 * USB host requests the configuration descriptor, this includes also the interface descriptors and their
 * sub-descriptors, with a total length specified by \ref USB_CONFIGURATION_DESCRIPTOR.wTotalLength.
 *
 * The \ref pUsbDescriptorLut[] lookup table takes the expected VALUE and INDEX fields for each
 * descriptor, and returns pointer and length.
 *
 * <b>Parameters</b>:
 * - VALUE.MSB: Descriptor type
 * - VALUE.LSB: Descriptor index
 * - INDEX: 0, or language ID for string descriptors (currently not supported)
 * - LENGTH: Descriptor length (either the requested number of bytes, or the length of the descriptor,
 *           whichever is the smallest)
 *
 * <b>Data (IN)</b>:
 * The descriptor(s)
 */
void usbsrGetDescriptor(void)
{
    const USB_DESCRIPTOR_LUT* pUsbDescriptorLutEntry;

    //
    // Which descriptor?
    //
    switch(usbSetupHeader.valueMsb)
    {

    case USB_DESC_TYPE_DEVICE:
        //
        // Device descriptor
        //
        usbSetupData.pBuffer   = (void*) usbdpGetDeviceDesc();
        usbSetupData.bytesLeft = ((USB_DEVICE_DESCRIPTOR*) usbSetupData.pBuffer)->bLength;
        break;

    case USB_DESC_TYPE_CONFIG:
        //
        // Configuration descriptor
        //
        usbSetupData.pBuffer   = (void*) usbdpGetConfigurationDesc(0, usbSetupHeader.valueLsb);
        usbSetupData.bytesLeft = ((USB_CONFIGURATION_DESCRIPTOR*) usbSetupData.pBuffer)->wTotalLength;
        break;

    default:
        //
        // String or other type descriptor
        //
        pUsbDescriptorLutEntry = usbdpGetDescByLut(usbSetupHeader.value, usbSetupHeader.index);
        if(pUsbDescriptorLutEntry)
        {
            usbSetupData.pBuffer   = (void*) pUsbDescriptorLutEntry->pDesc;
            usbSetupData.bytesLeft = pUsbDescriptorLutEntry->length;
        }
        else
        {
            //
            // Not found, so stall below
            //
            usbSetupData.pBuffer   = NULL;
        }
    }

    //
    // Stall EP0 if no descriptor was found
    //
    if(!usbSetupData.pBuffer)
    {
        usbfwData.ep0Status = EP_STALL;
    }

    if(usbfwData.ep0Status != EP_STALL)
    {
        //
        // Limit the returned descriptor size (the PC wants to know about sizes before
        // polling the complete descriptors)
        //
        if(usbSetupData.bytesLeft > usbSetupHeader.length)
        {
            usbSetupData.bytesLeft = usbSetupHeader.length;
        }

        usbfwData.ep0Status = EP_TX;
    }

}



/** \brief Internally used function that configures all endpoints for the specified interface
 *
 * The new endpoint setup overwrites the old. Unused endpoints keep their current setup. The user is
 * responsible for ensuring that no endpoint buffers overwrite each other, and that interfaces do not
 * cause conflicts. The \ref pUsbInterfaceEpDblbufLut[] lookup table must contain an entry for each
 * interface descriptor to define endpoint double-buffering.
 *
 * \param[in]       *pInterface
 *     A pointer to the interface descriptor
 */
static void usbsrConfigureEndpoints(const USB_INTERFACE_DESCRIPTOR* pInterface)
{
    const USB_ENDPOINT_DESCRIPTOR* pEndpoint;
    const USB_INTERFACE_EP_DBLBUF_LUT* pUsbInterfaceEpDblbufInfo;

    //
    // Locate the double-buffer settings
    //
    if(pInterface->bNumEndpoints)
    {
        pUsbInterfaceEpDblbufInfo = (const USB_INTERFACE_EP_DBLBUF_LUT*) pUsbInterfaceEpDblbufLut;
        while(pUsbInterfaceEpDblbufInfo->pInterface != pInterface)
        {
            pUsbInterfaceEpDblbufInfo++;
        }
    }

    //
    // For each endpoint in this interface
    //
    for(uint8_t n = 0; n < pInterface->bNumEndpoints; n++)
    {
        if(pEndpoint = usbdpFindNext(USB_DESC_TYPE_ENDPOINT, 0))
        {
            //
            // Get the endpoint index
            //
            uint32_t endpoint = pEndpoint->bEndpointAddress & 0x0F;
            USBFW_SELECT_ENDPOINT(endpoint);

            uint32_t csRegValue = 0x00;
            uint32_t maxpRegValue = (pEndpoint->wMaxPacketSize + 7) >> 3;

            //
            // For IN endpoints...
            //
            if(pEndpoint->bEndpointAddress & 0x80)
            {
                //
                // Clear data toggle, and flush twice (due to double buffering)
                //
                HWREG(USB_CS0_CSIL) = USB_CSIL_CLRDATATOG_M | USB_CSIL_FLUSHPACKET_M;
                HWREG(USB_CS0_CSIL) = USB_CSIL_FLUSHPACKET_M;
                
                //
                // USBCSIH
                //
                if((pEndpoint->bmAttributes & USB_EP_ATTR_TYPE_BM) == USB_EP_ATTR_ISO)
                {
                    //
                    // ISO flag
                    //
                    csRegValue |= USB_CSIH_ISO_M;
                }
                if(pUsbInterfaceEpDblbufInfo->inMask & (1 << endpoint))
                {
                    //
                    // Double buffering
                    //
                    csRegValue |= USB_CSIH_INDBLBUF_M;
                }
                HWREG(USB_CSIH) = csRegValue;

                //
                // Max transfer size
                //
                HWREG(USB_MAXI) = maxpRegValue;

                //
                // Endpoint status
                //
                usbfwData.pEpInStatus[endpoint - 1] = EP_IDLE;

                //
                // For OUT endpoints...
                //
            }
            else
            {
                //
                // Clear data toggle, and flush twice (due to double buffering)
                //
                HWREG(USB_CSOL) = USB_CSOL_CLRDATATOG_M | USB_CSOL_FLUSHPACKET_M;
                HWREG(USB_CSOL) = USB_CSOL_FLUSHPACKET_M;

                //
                // USBCSOH
                //
                if((pEndpoint->bmAttributes & USB_EP_ATTR_TYPE_BM) == USB_EP_ATTR_ISO)
                {
                    //
                    // ISO flag
                    //
                    csRegValue |= USB_CSOH_ISO_M;    
                }
                if(pUsbInterfaceEpDblbufInfo->outMask & (1 << endpoint))
                {
                    //
                    // Double buffering
                    //
                    csRegValue |= USB_CSOH_OUTDBLBUF_M;    
                }
                HWREG(USB_CSOH) = csRegValue;

                //
                // Max transfer size
                //
                HWREG(USB_MAXO) = maxpRegValue;

                //
                // Endpoint status
                //
                usbfwData.pEpOutStatus[endpoint - 1] = EP_IDLE;
            }
            USBFW_SELECT_ENDPOINT(0);
        }
    }
}




/** \brief Processes the \c GET_CONFIGURATION request (returns the current device configuration value)
 *
 * If the returned value is 0, the device is not configured (not in the configured state)
 *
 * <b>Parameters</b>:
 * - VALUE: Always 0
 * - INDEX: Always 0
 * - LENGTH: Always 1
 *
 * <b>Data (IN)</b>:
 * The non-zero \ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue of the currently selected
 * configuration.
 */
void usbsrGetConfiguration(void)
{
    //
    // Sanity check
    //
    if(usbSetupHeader.value || usbSetupHeader.index || (usbSetupHeader.length != 1))
    {
        usbfwData.ep0Status = EP_STALL;
    }
    else
    {
        //
        // Return the current configuration
        //
        usbSetupData.pBuffer = &usbfwData.configurationValue;
        usbSetupData.bytesLeft = 1;
        usbfwData.ep0Status = EP_TX;
    }

}




/** \brief Processes the \c SET_CONFIGURATION request (sets the device configuration)
 *
 * The configuration value must either be 0, in which case the device enters the address state, or it
 * must match a configuration value from one of the USB configuration descriptors. If there is a match,
 * the device enters the configured state.
 *
 * This request resets all interfaces to alternate setting 0, and uses the \c usbsrConfigureEndpoints()
 * function to automatically setup all endpoint registers.
 *
 * <b>Parameters</b>:
 * - VALUE: The configuration value (0-255)
 * - INDEX: Always 0
 * - LENGTH: Always 0
 */
void usbsrSetConfiguration(void)
{
    const USB_CONFIGURATION_DESCRIPTOR* pConfiguration;
    const USB_INTERFACE_DESCRIPTOR* pInterface;

    //
    // Sanity check
    //
    if((usbfwData.usbState == DEV_DEFAULT) || usbSetupHeader.index || usbSetupHeader.length || usbSetupHeader.valueMsb)
    {
        usbfwData.ep0Status = EP_STALL;
    }
    else
    {
        //
        // Default endpoint setup
        //
        
        usbsrHookProcessEvent(USBSR_EVENT_CONFIGURATION_CHANGING, 0);

        //
        // Configure relevant endpoints
        //
        if(usbSetupHeader.valueLsb)
        {
            //
            // Find the correct configuration descriptor...
            //
            pConfiguration = usbdpGetConfigurationDesc(usbSetupHeader.valueLsb, 0);

            //
            // If it exists...
            //
            if(pConfiguration)
            {
                usbfwData.usbState = DEV_CONFIGURED;
                usbfwData.configurationValue = usbSetupHeader.valueLsb;
                
                //
                // For each interface...
                //
                for(int n = 0; n < pConfiguration->bNumInterfaces; n++)
                {
                    usbfwData.pAlternateSetting[n] = 0x00;
                    
                    //
                    // Look only for alternate setting 0
                    //
                    do
                    {
                        pInterface = usbdpFindNext(USB_DESC_TYPE_INTERFACE, 0);
                    }
                    while(pInterface->bAlternateSetting != usbfwData.pAlternateSetting[n]);
                    
                    //
                    // Configure all endpoints in this interface
                    //
                    usbsrConfigureEndpoints(pInterface);
                }
            }
            else
            {
                //
                // If not, then stall the endpoint
                //
                usbfwData.ep0Status = EP_STALL;
            }
        }
        else
        {
            //
            // Unconfigure endpoints
            //
            usbfwData.configurationValue = usbSetupHeader.valueLsb;
            usbfwData.usbState = DEV_ADDRESS;
            usbfwSetAllEpStatus(EP_HALT);
        }
        usbsrHookProcessEvent(USBSR_EVENT_CONFIGURATION_CHANGED, 0);
    }
}




/** \brief Processes the \c GET_INTERFACE request (returns the selected alternate setting for the
 * specified interface)
 *
 * <b>Parameters</b>:
 * - VALUE: Always 0
 * - INDEX: Interface number
 * - LENGTH: Always 1
 *
 * <b>Data (IN)</b>:
 * The alternate setting for the selected interface
 */
void usbsrGetInterface(void)
{

    //
    // Sanity check
    //
    if((usbfwData.usbState != DEV_CONFIGURED) || (usbSetupHeader.requestType != RT_IN_INTERFACE) || usbSetupHeader.value || (usbSetupHeader.length != 1))
    {
        usbfwData.ep0Status = EP_STALL;
    }
    else
    {
        //
        // Return the current alternate setting
        //
        usbSetupData.pBuffer = &usbfwData.pAlternateSetting[usbSetupHeader.index];
        usbSetupData.bytesLeft = 1;
        usbfwData.ep0Status = EP_TX;
    }

}




/** \brief Processes the \c SET_INTERFACE request (selects an alternate setting for the specified
 * interface)
 *
 * This function uses the \c usbsrConfigureEndpoints() to automatically setup the relevant endpoint
 * registers.
 *
 * <b>Parameters</b>:
 * - VALUE: Alternate setting
 * - INDEX: Interface number
 * - LENGTH: Always 0
 */
void usbsrSetInterface(void)
{
    const USB_INTERFACE_DESCRIPTOR* pInterface;

    //
    // Sanity check
    //
    if((usbfwData.usbState != DEV_CONFIGURED) || (usbSetupHeader.requestType != RT_OUT_INTERFACE) || usbSetupHeader.length)
    {
        usbfwData.ep0Status = EP_STALL;
    }
    else
    {
        //
        // Verify that the desired alternate setting is available, and then make the switch
        //
        if(pInterface = usbdpGetInterfaceDesc(usbfwData.configurationValue, usbSetupHeader.index, usbSetupHeader.value))
        {
            usbsrHookProcessEvent(USBSR_EVENT_INTERFACE_CHANGING, usbSetupHeader.index);
            usbfwData.pAlternateSetting[usbSetupHeader.index] = usbSetupHeader.value;

            //
            // Configure all endpoints in this interface
            //
            usbsrConfigureEndpoints(pInterface);
            usbsrHookProcessEvent(USBSR_EVENT_INTERFACE_CHANGED, usbSetupHeader.index);

            //
            // This interface does not exist
            //
        }
        else
        {
            usbfwData.ep0Status = EP_STALL;
        }
    }
}


//@}
