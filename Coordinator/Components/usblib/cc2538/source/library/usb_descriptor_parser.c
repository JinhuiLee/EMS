//*****************************************************************************
//! @file       usb_descriptor_parser.c
//! @brief      USB descriptor parser.
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

/// \addtogroup module_usb_descriptor_parser
//@{
#include "usb_firmware_library_headers.h"


//-------------------------------------------------------------------------------------------------------
/// \name Standardized Indexes Into USB Descriptors
//@{
#define DESC_LENGTH_IDX             0 ///< Index of the bLength field (all descriptors)
#define DESC_TYPE_IDX               1 ///< Index of the bDescriptorType field (all descriptors)
//@}
//-------------------------------------------------------------------------------------------------------

//
/// External data
//
extern const uint8_t *usbDescriptor;

//
/// Internal data
//
typedef struct
{
    const uint8_t* pDesc; ///< Pointer to the current descriptor
} USBDP_DATA;

//
/// Internal data
//
static USBDP_DATA usbdpData;


/** \brief	Initializes a search
 *
 * This function must be called before each new search to reset \ref USBDP_DATA.pDesc.
 */
void usbdpInit(void)
{
    usbdpData.pDesc = (const uint8_t*) &usbDescriptor;
}


/** \brief	Locates the next descriptor of the wanted type
 *
 * This function parses through \ref usbDescriptor until:
 * - It hits one with <tt>bDescriptorType = wantedType</tt>, in which case it returns a pointer to that
 *   descriptor. \ref USBDP_DATA.pDesc will then point to the next descriptor.
 * - It hits one with <tt>bDescriptorType = haltAtType</tt>, in which case it returns a NULL-pointer.
 *   \ref USBDP_DATA.pDesc will then point to the descriptor halted at.
 * - It reaches the end of \ref usbDescriptor, in which case it returns a NULL-pointer.
 *   \ref USBDP_DATA.pDesc will (continue to) point to the end of \ref usbDescriptor.
 *
 * \note To begin a search with this function, \ref usbdpInit should be called first. It should not be
 *       called when continuing a search, for instance after a call to \ref usbdpGetConfigurationDesc().
 *
 * \param[in]       wantedType
 *     The wanted descriptor type (e.g. \ref USB_DESC_TYPE_DEVICE)
 * \param[in]       haltAtType
 *     The parser halts when it reaches this descriptor type, unless \c haltAtType is \c 0 (which in any
 *     case is an invalid \c bDescriptorType value).
 *
 * \return
 *     A pointer to the wanted descriptor type, or \c NULL if it was not found.
 */
const void* usbdpFindNext(uint8_t wantedType, uint8_t haltAtType)
{
    const void* pResult = NULL;

    //
    // As long as we haven't reached the end...
    //
    while(usbdpData.pDesc != (void*)(&usbDescriptor + 1))
    {
        //
        // If we have a match on wantedType...
        //
        if(usbdpData.pDesc[DESC_TYPE_IDX] == wantedType)
        {
            pResult = usbdpData.pDesc;
            usbdpData.pDesc += usbdpData.pDesc[DESC_LENGTH_IDX];
            break;
            
            //
            // If we have a match on haltAtType...
            //
        }
        else if(usbdpData.pDesc[DESC_TYPE_IDX] == haltAtType)
        {
            if(haltAtType)
            {
                break;
            }
        }

        //
        // Move on to the next descriptor
        //
        usbdpData.pDesc += usbdpData.pDesc[DESC_LENGTH_IDX];
    }

    return pResult;
}


/** \brief	Locates the (one and only) device descriptor
 *
 * \note It is not necessary to call \ref usbdpInit() before this function.
 *
 * \return
 *     A pointer to the \ref USB_DEVICE_DESCRIPTOR, or \c NULL if it was not found.
 */
const USB_DEVICE_DESCRIPTOR* usbdpGetDeviceDesc(void)
{
    usbdpInit();
    return usbdpFindNext(USB_DESC_TYPE_DEVICE, 0);
}


/** \brief	Locates a configuration descriptor
 *
 * The search will either look for a descriptor with a specific
 * \ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue, or the n'th configuration descriptor (by
 * "index")
 *
 * \note It is not necessary to call \ref usbdpInit() before this function.
 *
 * \param[in]       cfgValue
 *     The configuration value to search for (\ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue), or
 *     0 to find descriptor by index
 * \param[in]       cfgIndex
 *     A zero-based index for the configuration descriptor to find. This value is ignored unless
 *     \c cfgValue is 0.
 *
 * \return
 *     A pointer to the \ref USB_DEVICE_DESCRIPTOR, or \c NULL if it was not found.
 */
const USB_CONFIGURATION_DESCRIPTOR* usbdpGetConfigurationDesc(uint8_t cfgValue, uint8_t cfgIndex)
{
    const USB_CONFIGURATION_DESCRIPTOR* pConfigurationDesc;
    usbdpInit();

    //
    // As long as there are more configuration descriptors...
    //
    while(pConfigurationDesc = usbdpFindNext(USB_DESC_TYPE_CONFIG, 0))
    {
        //
        // Search by value?
        //
        if(cfgValue)
        {
            if(cfgValue == pConfigurationDesc->bConfigurationValue)
            {
                break;
            }
            //
            // Search by index? (search cfgIndex+1 times)
            //
        }
        else if(!cfgIndex--)
        {
            break;
        }
    }

    return pConfigurationDesc;
}


/** \brief	Locates an interface descriptor
 *
 * The function will first go to the configuration descriptor that matches the supplied configuration
 * value, and then locate the interface descriptor that matches the given interface number and alternate
 * setting.
 *
 * \note It is not necessary to call \ref usbdpInit() before this function.
 *
 * \param[in]       cfgValue
 *     The configuration value (\ref USB_CONFIGURATION_DESCRIPTOR.bConfigurationValue)
 * \param[in]       intNumber
 *     The interface number (\ref USB_INTERFACE_DESCRIPTOR.bInterfaceNumber)
 * \param[in]       altSetting
 *     The alternate setting (\ref USB_INTERFACE_DESCRIPTOR.bAlternateSetting)
 *
 * \return
 *     A pointer to the \ref USB_INTERFACE_DESCRIPTOR, or \c NULL if it was not found.
 */
const USB_INTERFACE_DESCRIPTOR* usbdpGetInterfaceDesc(uint8_t cfgValue, uint8_t intNumber, uint8_t altSetting)
{
    const USB_INTERFACE_DESCRIPTOR* pInterfaceDesc;

    //
    // First get to the correct configuration descriptor
    //
    usbdpGetConfigurationDesc(cfgValue, 0);

    //
    // Then find a match on the interface descriptor
    //
    while(pInterfaceDesc = usbdpFindNext(USB_DESC_TYPE_INTERFACE, USB_DESC_TYPE_CONFIG))
    {
        if((pInterfaceDesc->bInterfaceNumber == intNumber) && (pInterfaceDesc->bAlternateSetting == altSetting))
        {
            break;
        }
    }

    return pInterfaceDesc;
}


/** \brief	Locates descriptors by using the lookup table on the GET_DESCRIPTOR value/index parameters
 *
 * This function is used to retrieve:
 * - String descriptors with language ID
 * - Non-standard formatted descriptors, such as the HID class' report descriptors
 * - Standard-formatted class-defined descriptors, such as the \ref USBHID_DESCRIPTOR
 *
 * \param[in]       value
 *     The value field of the GET_DESCRIPTOR request
 * \param[in]       index
 *     The index field of the GET_DESCRIPTOR request
 *
 * \return
 *     A pointer to the descriptor, or \c NULL if it was not found.
 */
const USB_DESCRIPTOR_LUT* usbdpGetDescByLut(uint16_t value, uint16_t index)
{
    //
    // Search in the lookup table
    //
    const USB_DESCRIPTOR_LUT* pUsbDescriptorLutEntry = pUsbDescriptorLut;
    while(pUsbDescriptorLutEntry->pDesc)
    {
        if((pUsbDescriptorLutEntry->value == value) && (pUsbDescriptorLutEntry->index == index))
        {
            return pUsbDescriptorLutEntry;
        }
        pUsbDescriptorLutEntry++;
    }

    return NULL;
}


//@}
