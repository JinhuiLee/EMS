//*****************************************************************************
//! @file       app_usb_descriptor.c
//! @brief      USB descriptor for CDC class.
//!
//! Revised     $Date: 2013-02-25 14:00:56 +0100 (Mon, 25 Feb 2013) $
//! Revision    $Revision: 9379 $
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

#include "app_usb_descriptor.h"
#include <stdio.h>


const USB_DESCRIPTOR usbDescriptor = {
    { // device
        sizeof(USB_DEVICE_DESCRIPTOR),
        USB_DESC_TYPE_DEVICE,           // bDescriptorType
        0x0200,                         // bcdUSB (USB 2.0)
        0x02,                           // bDeviceClass (CDC)
        0x00,                           // bDeviceSubClass
        0x00,                           // bDeviceProtocol
        USB_EP0_PACKET_SIZE,            // bMaxPacketSize0
        0x0451,                         // idVendor (Texas Instruments)
        0x16C8,                         // idProduct (CC2538 CDC)
        0x0100,                         // bcdDevice (v1.0)
        0x01,                           // iManufacturer
        0x02,                           // iProduct
        0x00,                           // iSerialNumber
        0x01,                           // bNumConfigurations
    },
        { // configuration0
            sizeof(USB_CONFIGURATION_DESCRIPTOR),
            USB_DESC_TYPE_CONFIG,           // bDescriptorType
            SIZEOF_CONFIGURATION0_DESC,     // wTotalLength
            0x02,                           // bNumInterfaces
            0x01,                           // bConfigurationValue
            0x00,                           // iConfiguration
            0xA0,                           // bmAttributes (7,4-0: res, 6: self-powered, 5: remote wakeup)
            25                              // bMaxPower (max 2 * 25 = 50 mA)
        },
            { // interface0
                sizeof(USB_INTERFACE_DESCRIPTOR),
                USB_DESC_TYPE_INTERFACE,        // bDescriptorType
                0x00,                           // bInterfaceNumber
                0x00,                           // bAlternateSetting (none)
                0x01,                           // bNumEndpoints
                0x02,                           // bInterfaceClass (CDC communication interface)
                0x02,                           // bInterfaceSubClass (Abstract control model)
                0x01,                           // bInterfaceProcotol (V25TER)
                0x00                            // iInterface
            },
                { // hdrFunc0
                    sizeof(USBCDC_HEADER_FUNC_DESCRIPTOR),
                    USB_DESC_TYPE_CS_INTERFACE,     // bDescriptorType
                    USBCDC_FUNCDESC_HEADER,         // bDescriptorSubType
                    0x0110                          // bcdCDC
                },
                { // absCallMgmtFunc0
                    sizeof(USBCDC_ABSTRACT_CTRL_MGMT_FUNC_DESCRIPTOR),
                    USB_DESC_TYPE_CS_INTERFACE,     // bDescriptorType
                    USBCDC_FUNCDESC_ABS_CTRL_MGMT,  // bDescriptorSubType
                    0x02                            // bmCapabilities (supported class requests)
                },
                { // unionIfFunc0
                    sizeof(USBCDC_UNION_INTERFACE_FUNC_DESCRIPTOR),
                    USB_DESC_TYPE_CS_INTERFACE,     // bDescriptorType
                    USBCDC_FUNCDESC_UNION_IF,       // bDescriptorSubType
                    0x00,                           // bMasterInterface
                    0x01                            // bSlaveInterface0
                },
                { // callMgmtFunc0
                    sizeof(USBCDC_CALL_MGMT_FUNC_DESCRIPTOR),
                    USB_DESC_TYPE_CS_INTERFACE,     // bDescriptorType
                    USBCDC_FUNCDESC_CALL_MGMT,      // bDescriptorSubType
                    0x00,                           // bmCapabilities
                    0x01                            // bDataInterface
                },
                { // endpoint0
                    sizeof(USB_ENDPOINT_DESCRIPTOR),
                    USB_DESC_TYPE_ENDPOINT,         // bDescriptorType
                    0x82,                           // bEndpointAddress
                    USB_EP_ATTR_INT,                // bmAttributes (INT)
                    0x0040,                         // wMaxPacketSize
                    0x40                            // bInterval (64 full-speed frames = 64 ms)
                },
            { // interface1
                sizeof(USB_INTERFACE_DESCRIPTOR),
                USB_DESC_TYPE_INTERFACE,        // bDescriptorType
                0x01,                           // bInterfaceNumber
                0x00,                           // bAlternateSetting (none)
                0x02,                           // bNumEndpoints
                0x0A,                           // bInterfaceClass (CDC data interface)
                0x00,                           // bInterfaceSubClass (none)
                0x00,                           // bInterfaceProcotol (no protocol)
                0x00                            // iInterface
            },
                { // endpoint1
                    sizeof(USB_ENDPOINT_DESCRIPTOR),
                    USB_DESC_TYPE_ENDPOINT,         // bDescriptorType
                    0x84,                           // bEndpointAddress
                    USB_EP_ATTR_BULK,               // bmAttributes (BULK)
                    0x0040,                         // wMaxPacketSize
                    0x00                            // bInterval
                },
                { // endpoint2
                    sizeof(USB_ENDPOINT_DESCRIPTOR),
                    USB_DESC_TYPE_ENDPOINT,         // bDescriptorType
                    0x04,                           // bEndpointAddress
                    USB_EP_ATTR_BULK,               // bmAttributes (BULK)
                    0x0040,                         // wMaxPacketSize
                    0x00                            // bInterval
                },
    { // strings
        { // langIds
            sizeof(USB_STRING_0_DESCRIPTOR),
            USB_DESC_TYPE_STRING,
            0x0409 // English US
        },
        { // manufacturer
            sizeof(USB_STRING_1_DESCRIPTOR),
            USB_DESC_TYPE_STRING,
            'T', 'e', 'x', 'a', 's', ' ', 'I', 'n', 's', 't', 'r', 'u', 'm', 'e', 'n', 't', 's'
        },
        { // product
            sizeof(USB_STRING_2_DESCRIPTOR),
            USB_DESC_TYPE_STRING,
            'C', 'C', '2', '5', '3', '8', ' ', 'U', 'S', 'B', ' ', 'C', 'D', 'C'
        }
    }
};

// Serial number (application-initialized, depends on chip serial number, IEEE address etc.)
USB_STRING_3_DESCRIPTOR usbSerialNumberStringDesc;


// Look-up table for descriptors other than device and configuration (table is NULL-terminated)
const USB_DESCRIPTOR_LUT pUsbDescriptorLut[] = {
//    value   index   length                           pDesc
    { 0x0300, 0x0000, sizeof(USB_STRING_0_DESCRIPTOR), &usbDescriptor.strings.langIds },
    { 0x0301, 0x0409, sizeof(USB_STRING_1_DESCRIPTOR), &usbDescriptor.strings.manufacturer },
    { 0x0302, 0x0409, sizeof(USB_STRING_2_DESCRIPTOR), &usbDescriptor.strings.product },
    { 0x0303, 0x0409, sizeof(USB_STRING_3_DESCRIPTOR), &usbSerialNumberStringDesc },
    { 0x0000, 0x0000, 0,                               NULL }
};


// Look-up table entry specifying double-buffering for each interface descriptor
const USB_INTERFACE_EP_DBLBUF_LUT pUsbInterfaceEpDblbufLut[] = {
//    pInterface                 inMask  outMask
    { &usbDescriptor.interface0, 0x0000, 0x0000 },
    { &usbDescriptor.interface1, 0x0000, 0x0000 }
};
