//*****************************************************************************
//! @file       usb_in_buffer.c
//! @brief      USB IN FIFO Ring Buffer.
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

/// \addtogroup module_usb_in_buffer
//@{
#include "usb_firmware_library_headers.h"
#include "usb_in_buffer.h"




/** \brief Polled to transfer data from the ring-buffer to the IN endpoint
 *
 * The function will transfer data from the ring-buffer is empty or the endpoint FIFO (single- or double-
 * buffered) is full. The provided data structure contains properties and status of the ring-buffer and
 * properties of the associated endpoint.
 *
 * Transfers are done using the maximum endpoint packet size until the ring-buffer runs empty.
 *
 * \param[in,out]   *pIbufData
 *     Properties and status of the ring-buffer and its associated USB endpoint
 */
void usbibufPollEndpoint(USB_EPIN_RINGBUFFER_DATA* pIbufData)
{
    uint32_t oldEndpoint;

    //
    // Save the old endpoint index setting, then select endpoint
    //
    oldEndpoint = USBFW_GET_SELECTED_ENDPOINT();
    USBFW_SELECT_ENDPOINT(pIbufData->endpointIndex);

    //
    // While there is an empty endpoint FIFO ...
    //
    while(USBFW_IN_ENDPOINT_DISARMED())
    {
        //
        // Find the number of bytes to write
        //
        uint16_t count = pIbufData->count;

        //
        // Count limited by the endpoint size?
        //
        if(count > pIbufData->endpointSize)
        {
            count = pIbufData->endpointSize;
        }

        //
        // Bail out when writing nothing
        //
        if(count == 0)
        {
            break;
        }
        
        //
        // Transfer from the FIFO register to the ring-buffer
        //
        pIbufData->count -= count;
        while(count--)
        {
            HWREG(pIbufData->endpointReg) = pIbufData->pBuffer[pIbufData->tail++];
            if(pIbufData->tail == pIbufData->size)
            {
                pIbufData->tail = 0;
            }

        }

        //
        // ARM the FIFO
        //
        USBFW_ARM_IN_ENDPOINT();
    }

    //
    // Restore endpoint selection
    //
    USBFW_SELECT_ENDPOINT(oldEndpoint);

}




/** \brief Returns the number of unused bytes in the ring-buffer
 *
 * This is the maximum number of bytes that can be written at the moment using \ref usbibufPush().
 *
 * \param[in,out]   *pIbufData
 *     Properties and status of the ring-buffer and its associated USB endpoint
 *
 * \return
 *     Number of unused bytes
 */
uint16_t usbibufGetMaxPushCount(USB_EPIN_RINGBUFFER_DATA* pIbufData)
{
    return pIbufData->size - pIbufData->count;
}




/** \brief Pushes data to the ring-buffer
 *
 * Use \ref usbibufGetMaxPushCount() to prevent ring-buffer overflow.
 *
 * \param[in,out]   *pIbufData
 *     Properties and status of the ring-buffer and its associated USB endpoint
 * \param[in]   *pSrc
 *     Source buffer for the data to be written
 * \param[in]   count
 *     Number of bytes to transfer from the source buffer to the ring-buffer
 */
void usbibufPush(USB_EPIN_RINGBUFFER_DATA* pIbufData, uint8_t* pSrc, uint16_t count)
{
    //
    // Transfer from the source buffer to the ring-buffer
    //
    pIbufData->count += count;
    while(count--)
    {
        pIbufData->pBuffer[pIbufData->head++] = *(pSrc++);
        if(pIbufData->head == pIbufData->size)
        {
            pIbufData->head = 0;
        }
    }

}


//@}
