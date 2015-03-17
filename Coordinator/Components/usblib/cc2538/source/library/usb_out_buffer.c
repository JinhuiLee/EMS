//*****************************************************************************
//! @file       usb_out_buffer.c
//! @brief      USB OUT FIFO Ring Buffer.
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

/// \addtogroup module_usb_out_buffer
//@{
#include "usb_firmware_library_headers.h"
#include "usb_out_buffer.h"




/** \brief Polled to transfer data from the OUT endpoint to the ring-buffer
 *
 * The function will transfer data from the endpoint until its FIFO is empty or the ring-buffer is full.
 * The provided data structure contains properties and status of the ring-buffer and properties of the
 * associated endpoint.
 *
 * When the ring-buffer runs full, reading from the endpoint FIFO stops and can be continued in a later
 * call after popping data using \ref usbobufPop().
 *
 * \param[in,out]   *pObufData
 *     Properties and status of the ring-buffer and its associated USB endpoint
 */
void usbobufPollEndpoint(USB_EPOUT_RINGBUFFER_DATA* pObufData)
{
    uint32_t oldEndpoint;

    //
    // Save the old endpoint index setting, then select endpoint
    //
    oldEndpoint = USBFW_GET_SELECTED_ENDPOINT();
    USBFW_SELECT_ENDPOINT(pObufData->endpointIndex);

    //
    // While there is received data in the endpoint FIFO ...
    //
    while(USBFW_OUT_ENDPOINT_DISARMED())
    {

        //
        // Find the number of bytes to read
        //
        uint16_t count = USBFW_GET_OUT_ENDPOINT_COUNT_LOW();

        //
        // Count limited by number of unused bytes?
        //
        if(count > (pObufData->size - pObufData->count))
        {
            count = pObufData->size - pObufData->count;
        }

        //
        // Transfer from the FIFO register to the ring-buffer
        //
        pObufData->count += count;
        while(count--)
        {
            pObufData->pBuffer[pObufData->head++] = HWREG(pObufData->endpointReg);
            if(pObufData->head == pObufData->size)
            {
                pObufData->head = 0;
            }
        }

        //
        // If we've emptied the FIFO, arm it
        //
        if(USBFW_GET_OUT_ENDPOINT_COUNT_LOW() == 0)
        {
            USBFW_ARM_OUT_ENDPOINT();
        }
    }

    //
    // Restore endpoint selection
    //
    USBFW_SELECT_ENDPOINT(oldEndpoint);

}


/** \brief Returns the number of bytes currently stored in the ring-buffer
 *
 * This is the maximum number of bytes that can be read at the moment using \ref usbobufPop().
 *
 * \param[in,out]   *pObufData
 *     Properties and status of the ring-buffer and its associated USB endpoint
 *
 * \return
 *     Number of bytes currently stored
 */
uint16_t usbobufGetMaxPopCount(USB_EPOUT_RINGBUFFER_DATA* pObufData)
{
    return pObufData->count;
}


/** \brief Pops data stored in the ring-buffer
 *
 * Use \ref usbobufGetMaxPopCount() to prevent ring-buffer underflow.
 *
 * \param[in,out]   *pObufData
 *     Properties and status of the ring-buffer and its associated USB endpoint
 * \param[out]   *pDest
 *     Destination buffer for the read data
 * \param[in]   count
 *     Number of bytes to transfer from the ring-buffer to the destination buffer
 */
void usbobufPop(USB_EPOUT_RINGBUFFER_DATA* pObufData, uint8_t* pDest, uint16_t count)
{
    //
    // Transfer from the ring-buffer to the destination buffer
    //
    pObufData->count -= count;
    
    while(count--)
    {
        *(pDest++) = pObufData->pBuffer[pObufData->tail++];
        if(pObufData->tail == pObufData->size)
        {
            pObufData->tail = 0;
        }
    }
    
}


//@}
