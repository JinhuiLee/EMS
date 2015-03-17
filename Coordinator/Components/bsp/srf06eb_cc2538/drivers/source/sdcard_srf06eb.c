//*****************************************************************************
//! @file       sdcard_srf06eb.c
//! @brief      SmartRF06EB device driver implementation for microSD card
//!             reader. Card detection pin is not implemented/connected on
//!             SmartRF06EB.
//!
//!             This implementation implements basic read write functions and
//!             supports SD card versions 1 and 2, plus MMC cards version 3.
//!
//! Revised     $Date: 2013-04-11 19:41:57 +0200 (Thu, 11 Apr 2013) $
//! Revision    $Revision: 9707 $
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
// ***************************************************************************/
#ifndef SDCARD_EXCLUDE


/**************************************************************************//**
* @addtogroup sdcard_api
* @{
******************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "sdcard.h"
#include "sys_ctrl.h"           // Access to driverlib sys_ctrl fns
#include "ioc.h"                // Access to driverlib io control fns
#include "gpio.h"               // Access to driverlib gpio fns
#include "ssi.h"                // Access to driverlib ssi fns

#include "bsp_led.h"


/******************************************************************************
* DEFINES
*/
// Base address of SSI module used by implementation
#define SDCARD_SSI_BASE         BSP_SPI_SSI_BASE

// Bitmask to enable SSI module
#define SDCARD_SSI_ENABLE_BM    BSP_SPI_SSI_ENABLE_BM

// Macro for asserting SD card CSn (set low)
#define SDCARD_SPI_BEGIN()      GPIOPinWrite(BSP_SDCARD_CS_BASE,              \
                                             BSP_SDCARD_CS, 0);

// Macro for deasserting SD card CSn (set high)
#define SDCARD_SPI_END()        GPIOPinWrite(BSP_SDCARD_CS_BASE,              \
                                             BSP_SDCARD_CS, BSP_SDCARD_CS);

/******************************************************************************
* VARIABLES AND LOCAL FUNCTIONS
*/
//
//! Variable holdeing card status and type
//
static uint8_t ui8SdOptions = 0;
static uint8_t sdEnterSpiMode(void);
static uint8_t sdGetByte(void);

//
// Utility functions
//
static void sdCardSendCommand(const uint8_t ui8Cmd, uint32_t ui32Data,
                              const uint8_t ui8Crc);
static uint8_t sdCardGetResponse(uint8_t * pui8Buffer, uint8_t ui8Type);
static uint8_t sdCardGetData(uint8_t * pui8Buffer, uint16_t ui32ByteCount);


/******************************************************************************
* FUNCTIONS
*/

/**************************************************************************//**
* @brief    This function initialies an SD/MMC card. This function must be run
*           before you can use the SD card. This function assumes that the SPI
*           interface has been initialized using, for example, bspSpiInit().
*
* @return   Returns SDCARD_SUCCESS on success
******************************************************************************/
uint8_t
sdCardInit(void)
{
    uint_fast32_t ui32OldClock, ui32Count;
    uint8_t ui8Response[5];
    uint_fast8_t uc3V3Off = !(GPIOPinRead(BSP_3V3_EN_BASE, BSP_3V3_EN) & 0xFF);

    //
    // Reset options variable
    //
    ui8SdOptions = SDCARD_STATUS_NOCARD;

    //
    // If 3.3-V domain is initially off, make sure it's off >1 ms for a complete
    // sd card power cycle
    //
    if(uc3V3Off)
    {
        //
        // Approx 1.5 ms delay
        //
        SysCtrlDelay((SysCtrlClockGet() / 667) / 3);
    }

    //
    // Enable 3.3-V domain (it takes <= 600 us to stabilize)
    //
    bsp3V3DomainEnable();

    //
    // Configure CSn as output high
    //
    GPIOPinTypeGPIOOutput(BSP_SDCARD_CS_BASE, BSP_SDCARD_CS);
    SDCARD_SPI_END();

    //
    // If 3.3-V domain was off, wait for it to stabilize (~1.1 ms)
    //
    if(uc3V3Off)
    {
        SysCtrlDelay((SysCtrlClockGet() / 1000 / 3));
    }

    //
    // Wait for any ongoing SPI transfers to complete
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }

    //
    // Set SPI clock speed to 400kHz (max speed for SD/MMC init)
    //
    ui32OldClock = bspSpiClockSpeedGet();
    bspSpiClockSpeedSet(SDCARD_SPI_INIT_FREQ);

    //
    // Enter SPI mode
    //
    if(sdEnterSpiMode() != 1)
    {
        //
        // No valid card. Undo all changes and return status
        //
        SDCARD_SPI_END();
        bsp3V3DomainDisable();
        bspSpiClockSpeedSet(ui32OldClock);
        return SDCARD_IDLE_STATE_TIMEOUT;
    }

    //
    // A card is present
    //
    ui8SdOptions |= SDCARD_STATUS_NOINIT;

    //
    // Determine card type
    //
    sdCardSendCommand(SDCARD_CMD8, 0x1AA, 0x87);
    if(sdCardGetResponse(ui8Response, SDCARD_R3) == 1)
    {
        if(ui8Response[3] == 0x01 && ui8Response[4] == 0xAA)
        {
            //
            // SDv2
            //
            ui8SdOptions |= SDCARD_TYPE_SDV2;

            //
            // Wait for initialization to complete
            //
            ui32Count = SDCARD_MAX_CMD_POLL_COUNT;
            while(ui32Count > 0)
            {
                ui32Count--;
                sdCardSendCommand(SDCARD_CMD55, 0, SDCARD_DUMMY);
                sdCardGetResponse(&ui8Response[0], SDCARD_R1);
                sdCardSendCommand(SDCARD_ACMD41, 0x40000000, SDCARD_DUMMY);
                sdCardGetResponse(&ui8Response[1], SDCARD_R1);
                if(ui8Response[0] > 1 || ui8Response[1] > 1 || ui32Count == 0)
                {
                    //
                    // Something went wrong. Undo all changes and return
                    // status.
                    //
                    SDCARD_SPI_END();
                    bsp3V3DomainDisable();
                    bspSpiClockSpeedSet(ui32OldClock);
                    if(!ui32Count)
                    {
                        return SDCARD_SEND_OP_COND_TIMEOUT;
                    }
                    return SDCARD_ERROR;
                }
                if(ui8Response[1] == 0)
                {
                    break;
                }
            }

            //
            // Are addresses given in bytes (OCR.CCS=0) or blocks(OCR.CCS=1)?
            //
            sdCardSendCommand(SDCARD_CMD58, 0, SDCARD_DUMMY);
            if(sdCardGetResponse(ui8Response, SDCARD_R3) == 0)
            {
                ui8SdOptions |= (ui8Response[1] & 0x40) ? SDCARD_BLOCK_ADDR : 0;
            }
        }
        else
        {
            //
            // Unknown card. Undo all changes and return status.
            //
            SDCARD_SPI_END();
            bsp3V3DomainDisable();
            bspSpiClockSpeedSet(ui32OldClock);
            return SDCARD_ERROR;
        }
    }
    else
    {
        //
        // SDv1 or MMCv3?
        //

        //
        // Send (CMD55+)ACM41. MMCv3 will return invalid command
        //
        ui32Count--;
        sdCardSendCommand(SDCARD_CMD55, 0, SDCARD_DUMMY);
        sdCardGetResponse(&ui8Response[0], SDCARD_R1);
        sdCardSendCommand(SDCARD_ACMD41, 0, SDCARD_DUMMY);
        sdCardGetResponse(&ui8Response[1], SDCARD_R1);

        if((ui8Response[0] <= 1) || (ui8Response[1] <= 1))
        {
            //
            // SDv1
            //
            ui8SdOptions |= SDCARD_TYPE_SDV1;

            //
            // Wait for initialization to complete
            //
            ui32Count = SDCARD_MAX_CMD_POLL_COUNT;
            while(ui32Count > 0)
            {
                ui32Count--;
                sdCardSendCommand(SDCARD_CMD55, 0, SDCARD_DUMMY);
                sdCardGetResponse(&ui8Response[0], SDCARD_R1);
                sdCardSendCommand(SDCARD_ACMD41, 0, SDCARD_DUMMY);
                sdCardGetResponse(&ui8Response[1], SDCARD_R1);
                if(((ui8Response[0] > 1) || (ui8Response[1] > 1) ||
                        (ui32Count == 0)))
                {
                    //
                    // Something went wrong. Undo all changes and return
                    // status.
                    //
                    SDCARD_SPI_END();
                    bsp3V3DomainDisable();
                    bspSpiClockSpeedSet(ui32OldClock);
                    if(!ui32Count)
                    {
                        return SDCARD_SEND_OP_COND_TIMEOUT;
                    }
                    return SDCARD_ERROR;
                }
                if(ui8Response[1] == 0)
                {
                    //
                    // Response OK
                    //
                    break;
                }
            }
        }
        else
        {
            //
            // MMCv3
            //
            ui8SdOptions |= SDCARD_TYPE_MMCV3;

            //
            // Wait for initialization to complete
            //
            ui32Count = SDCARD_MAX_CMD_POLL_COUNT;
            while(ui32Count > 0)
            {
                ui32Count--;
                sdCardSendCommand(SDCARD_CMD1, 0, SDCARD_DUMMY);
                sdCardGetResponse(&ui8Response[0], SDCARD_R1);
                if((ui8Response[0] > 1) || (ui32Count == 0))
                {
                    //
                    // Something went wrong. Undo all changes and return
                    // status.
                    //
                    SDCARD_SPI_END();
                    bsp3V3DomainDisable();
                    bspSpiClockSpeedSet(ui32OldClock);
                    if(!ui32Count)
                    {
                        return SDCARD_SEND_OP_COND_TIMEOUT;
                    }
                    return SDCARD_ERROR;
                }
                if(ui8Response[0] == 0)
                {
                    break;
                }
            }
        }
    }

    //
    // Card is initialized, return to old SPI speed
    //
    bspSpiClockSpeedSet(ui32OldClock);

    //
    // Set blocklength for SDv1 and MMCv3
    //
    if(!(ui8SdOptions & SDCARD_TYPE_SDV2))
    {
        //
        // Set block length and get response
        //
        sdCardSendCommand(SDCARD_SET_BLOCKLEN, SDCARD_BLOCKLENGTH, SDCARD_DUMMY);
        sdCardGetResponse(ui8Response, SDCARD_R1);

        //
        // Check response
        //
        if(ui8Response[0] != 0)
        {
            SDCARD_SPI_END();
            return SDCARD_SET_BLOCKLEN_TIMEOUT;
        }
    }

    SDCARD_SPI_END();

    //
    // Init successful, update status variable and return status
    //
    ui8SdOptions = (ui8SdOptions & ~SDCARD_STATUS_M) | SDCARD_STATUS_READY;
    return SDCARD_SUCCESS;
}


/**************************************************************************//**
* @brief    This function checks the card connection status.
*
* @return   Returns \b SDCARD_STATUS_READY if card is present and initialized.
* @return   Returns \b SDCARD_STATUS_NOINIT if card is present, but not
*           initialized.
* @return   Returns \b SDCARD_STATUS_NOCARD if no card is detected.
******************************************************************************/
uint8_t
sdCardStatus(void)
{
    uint8_t ui8Status;
    uint32_t ui32Dummy, ui32OldClock;

    //
    // If 3V3 domain is powered off, return "no card"
    //
    if(!(GPIOPinRead(BSP_3V3_EN_BASE, BSP_3V3_EN) & 0xFF))
    {
        //
        // Clear options variable and return status
        //
        ui8SdOptions = 0;
        return SDCARD_STATUS_NOCARD;
    }

    //
    // Not supporting hot-swapping. If card is initialized, do not check again
    //
    if((ui8SdOptions & SDCARD_STATUS_M) == SDCARD_STATUS_READY)
    {
        return SDCARD_STATUS_READY;
    }

    //
    // Wait for any ongoing transfers to finish and empty fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Set low SPI speed
    //
    ui32OldClock = bspSpiClockSpeedGet();
    bspSpiClockSpeedSet(SDCARD_SPI_INIT_FREQ);

    if(sdEnterSpiMode() == 1)
    {
        //
        // Card present
        //
        ui8Status = SDCARD_STATUS_NOINIT;
    }
    else
    {
        //
        // No card
        //
        ui8Status = SDCARD_STATUS_NOCARD;
    }

    SDCARD_SPI_END();
    bspSpiClockSpeedSet(ui32OldClock);

    //
    // Update options variable and return status
    //
    ui8SdOptions = (ui8SdOptions & ~SDCARD_STATUS_M) | ui8Status;
    return ui8Status;
}


/**************************************************************************//**
* @brief    This function reads block \e ui32Block from the SD card. The
*           function converts argument \e ui32Block to byte address if needed.
*
* @param    ui32Block   is the logical block to read (LBA, Logical Block
*                       Addressing).
* @param    pui8Buffer  is a pointer to the destination array for read data.
*
* @return   Returns SDCARD_SUCCESS on success
******************************************************************************/
uint8_t
sdCardBlockRead(uint32_t ui32Block, uint8_t *pui8Buffer)
{
    uint8_t ui8Response;
    uint32_t ui32Dummy;

    if(!(ui8SdOptions & SDCARD_BLOCK_ADDR))
    {
        //
        // Convert address to byte address
        //
        ui32Block = ui32Block * SDCARD_BLOCKLENGTH;
    }

    //
    // Wait for any ongoing transfers to finish and empty fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Assert CSn
    //
    SDCARD_SPI_BEGIN();

    //
    // Send Command 17 (read single block) and retrieve response
    //
    sdCardSendCommand(SDCARD_READ_SINGLE_BLOCK, ui32Block, SDCARD_DUMMY);
    sdCardGetResponse(&ui8Response, SDCARD_R1);
    if(ui8Response != 0)
    {
        //
        // Something went wrong. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        if((ui8Response & 0x0F) == 0x05)
        {
            return SDCARD_ILLEGAL_CMD;
        }
        else
        {
            return SDCARD_ERROR;
        }
    }

    //
    // Wait for data and retrieve them
    //
    if(SDCARD_SUCCESS != sdCardGetData(pui8Buffer, SDCARD_BLOCKLENGTH))
    {
        //
        // Timeout. Deassert CSn and return status
        //
        SDCARD_SPI_END();
        return SDCARD_READ_BLOCK_TIMEOUT;
    }

    //
    // Deassert CSn and return status
    //
    SDCARD_SPI_END();
    return SDCARD_SUCCESS;
}


/**************************************************************************//**
* @brief    This function writes \b SDCARD_BLOCKLENGTH bytes to block
*           \e ui32Block on the SD card. The function converts argument
*           \e ui32Block to byte address if needed.
*
* @param    ui32Block       is the logical block to write (LBA).
* @param    pui8Buffer      is a pointer to the source array with data.
*
* @return   Returns SDCARD_SUCCESS on success
******************************************************************************/
uint8_t
sdCardBlockWrite(uint32_t ui32Block, const uint8_t * pui8Buffer)
{
    uint32_t ui32Dummy, ui32Count;
    uint8_t ui8Response;

    if(!(ui8SdOptions & SDCARD_BLOCK_ADDR))
    {
        //
        // Convert address to byte address
        //
        ui32Block = ui32Block * SDCARD_BLOCKLENGTH;
    }

    //
    // Wait for any ongoing SPI transfer and empty fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Assert CSn
    //
    SDCARD_SPI_BEGIN();

    //
    // Send Command 24 (write single block) and retrieve response
    //
    sdCardSendCommand(SDCARD_WRITE_BLOCK, ui32Block, SDCARD_DUMMY);
    if(sdCardGetResponse(&ui8Response, SDCARD_R1) != 0)
    {
        //
        // Something went wrong. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        return SDCARD_ERROR;
    }

    //
    // Send token to indicate start of data block
    //
    SSIDataPut(SDCARD_SSI_BASE, SDCARD_SINGLE_DATA_READY);

    //
    // Send actual data
    //
    for(ui32Count = 0; ui32Count < SDCARD_BLOCKLENGTH; ui32Count++)
    {
        SSIDataPut(SDCARD_SSI_BASE, pui8Buffer[ui32Count]);
    }

    //
    // Send two dummy CRC bytes
    //
    SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);
    SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);

    //
    // Wait for data to be transferred and empty fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Wait for card finished
    //
    sdCardGetResponse(&ui8Response, SDCARD_R1);

    //
    // Check response
    //
    if((ui8Response & 0x0F) != 0x05)
    {
        //
        // Something went wrong. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        return SDCARD_ILLEGAL_CMD;
    }

    //
    // Wait for writing to finish
    //
    for(ui32Count = 0; ui32Count < SDCARD_MAX_RESPONSE_POLL_COUNT;
            ui32Count++)
    {
        //
        // Read response byte and interpret result.
        //
        ui8Response = sdGetByte();
        if(ui8Response == 0xFF)
        {
            SDCARD_SPI_END();
            return SDCARD_SUCCESS;
        }
    }

    //
    // Timeout. Deassert CSn and return status.
    //
    SDCARD_SPI_END();
    return SDCARD_WRITE_BLOCK_TIMEOUT;
}


/**************************************************************************//**
* @brief    This function returns the SD card block size. This function does
*           not access the SD card.
*
* @return   Returns block size in bytes
******************************************************************************/
uint32_t
sdCardGetBlockSize(void)
{
    return SDCARD_BLOCKLENGTH;
}


/**************************************************************************//**
* @brief    This function reads out the 16-byte long card identification data
*           (CID) register.
*
* @param    psCid       is a pointer to the \e tSdCardCid structure.
*
* @return   Returns SDCARD_SUCCESS on success
******************************************************************************/
uint8_t
sdCardGetCid(tSdCardCid * psCid)
{
    uint8_t pui8Cid[16];
    uint8_t ui8Response;
    uint32_t ui32Dummy;

    //
    // Wait for any ongoing transfers to complete and empty fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Assert CSn
    //
    SDCARD_SPI_BEGIN();

    //
    // Send command and retrieve response
    //
    sdCardSendCommand(SDCARD_SEND_CID, 0, SDCARD_DUMMY);
    sdCardGetResponse(&ui8Response, SDCARD_R1);
    if(ui8Response != 0)
    {
        //
        // Something went wrong. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        if((ui8Response & 0x0F) == 0x05)
        {
            return SDCARD_ILLEGAL_CMD;
        }
        else
        {
            return SDCARD_ERROR;
        }
    }

    //
    // Wait for data and retrieve it
    //
    if(SDCARD_SUCCESS != sdCardGetData(pui8Cid, 16))
    {
        //
        // Timeout. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        return SDCARD_SEND_CID_TIMEOUT;

    }

    //
    // Deassert CSn
    //
    SDCARD_SPI_END();

    //
    // Store retrieved data into CID structure.
    //
    psCid->ui8Mid     = pui8Cid[0];
    psCid->pui8Pid[0] = pui8Cid[1];
    psCid->pui8Pid[1] = pui8Cid[2];
    psCid->pcNm[0]    = pui8Cid[3];
    psCid->pcNm[1]    = pui8Cid[4];
    psCid->pcNm[2]    = pui8Cid[5];
    psCid->pcNm[3]    = pui8Cid[6];
    psCid->pcNm[4]    = pui8Cid[7];
    psCid->ui8Rev     = pui8Cid[8];
    psCid->ui32Psn    = (((uint32_t)pui8Cid[9]  << 24) |                      \
                         ((uint32_t)pui8Cid[10] << 16) |                      \
                         ((uint32_t)pui8Cid[11] <<  8) |                      \
                         ((uint32_t)pui8Cid[12]));
    psCid->ui8MdtY    = pui8Cid[14] & 0x0F;
    psCid->ui8MdtM    = (((pui8Cid[13] << 4) & 0xF0) |                        \
                         ((pui8Cid[14] & 0xF0) >> 4));

    //
    // Return status
    //
    return SDCARD_SUCCESS;
}


/**************************************************************************//**
* @brief    This function reads out the card specific data (CSD) register . The
*           size of the CSD register is 16 bytes. The data are stored in the
*           buffer specified by \e pui8Csd with MSB first; for example,
*           CSD[127:126] is at \e pui8Csd[7:6].
*
* @param    pui8Csd      is a pointer to the destination array.
*
* @return   Returns SDCARD_SUCCESS on success
******************************************************************************/
uint8_t
sdCardGetCsd(uint8_t * pui8Csd)
{
    uint8_t ui8Response;
    uint32_t ui32Dummy;

    //
    // Wait for any ongoing transfers to complete and empty fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Assert CSn
    //
    SDCARD_SPI_BEGIN();

    //
    // Send command and retrieve response
    //
    sdCardSendCommand(SDCARD_SEND_CSD, 0, SDCARD_DUMMY);
    sdCardGetResponse(&ui8Response, SDCARD_R1);
    if(ui8Response != 0)
    {
        //
        // Something went wrong. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        if((ui8Response & 0x0F) == 0x05)
        {
            return SDCARD_ILLEGAL_CMD;
        }
        else
        {
            return SDCARD_ERROR;
        }
    }

    //
    // Wait for data and retrieve them
    //
    if(SDCARD_SUCCESS != sdCardGetData(pui8Csd, 16))
    {
        //
        // Timeout. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        return SDCARD_SEND_CSD_TIMEOUT;
    }

    //
    // Deassert CSn and return status.
    //
    SDCARD_SPI_END();
    return SDCARD_SUCCESS;
}


/**************************************************************************//**
* @brief    This function reads out the 2-byte SD card status register.
*
* @param    pui8Buffer      is a pointer to the destination array.
*
* @return   Returns SDCARD_SUCCESS on success
******************************************************************************/
uint8_t
sdCardGetStatusReg(uint8_t * pui8Buffer)
{
    uint8_t pui8Response[2];
    uint32_t ui32Dummy;

    //
    // Wait for any ongoing transfers to complete and empty fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Assert CSn
    //
    SDCARD_SPI_BEGIN(); // CS low

    //
    // Send SEND_STATUS command and retrieve response
    //
    sdCardSendCommand(SDCARD_SEND_STATUS, 0, SDCARD_DUMMY);
    sdCardGetResponse(pui8Response, SDCARD_R2);
    if(pui8Response[0] != 0)
    {
        //
        // Something went wrong. Deassert CSn and return status.
        //
        SDCARD_SPI_END();
        if((pui8Response[0] & 0x0F) == 0x05)
        {
            return SDCARD_ILLEGAL_CMD;
        }
        else
        {
            return SDCARD_ERROR;
        }
    }

    //
    // Deassert CSn
    //
    SDCARD_SPI_END();

    //
    // Copy values to buffer and return status
    //
    *pui8Buffer++ = pui8Response[0];
    *pui8Buffer   = pui8Response[1];
    return SDCARD_SUCCESS;
}


/**************************************************************************//**
* @brief    This function returns the size of card in KiB (1 KiB = 2^10 bytes).
*
* @return   Returns size of card in bytes, 0 if failed
******************************************************************************/
uint32_t
sdCardGetSize(void)
{
    uint8_t pui8Csd[16];
    uint8_t ui8ReadBlLen, ui8CSizeMult;
    uint32_t ui32CSize;
    uint32_t ui32CardSize = 0;

    //
    // Get CSD Register with basic Information about card
    //
    if(SDCARD_SUCCESS != sdCardGetCsd(pui8Csd))
    {
        return 0;
    }

    //
    // Determine CSD structure version
    //
    if(pui8Csd[0] == 0)
    {
        //
        // CSD structure version 1.0
        // Capacity = (C_SIZE+1) * 2^(C_SIZE_MULT + 2 + READ_BL_LEN) bytes
        //
        ui8ReadBlLen = (pui8Csd[5] & 0x0F);
        ui32CSize    = (pui8Csd[8]         >>  6) |
                       (pui8Csd[7]         <<  2) |
                       ((pui8Csd[6] & 0x03) << 10);
        ui8CSizeMult = ((pui8Csd[9] & 0x03) <<  1) |
                       (pui8Csd[10]        >> 6);

        ui32CardSize = ((ui32CSize + 1) <<                                    \
                        (uint32_t)(ui8ReadBlLen + ui8CSizeMult + 2));

        //
        // Convert to kibibytes
        //
        ui32CardSize >>= 10;

    }
    else if(pui8Csd[0] == 0x40)
    {
        //
        // CSD structure version 2.0
        // Capacity = (C_SIZE + 1) * 512 * 1024 bytes
        //
        ui32CSize = (pui8Csd[9]               |
                     pui8Csd[8]         << 8  |
                     (pui8Csd[7] & 0x3F) << 16);

        //
        // Calculate value (in kibibytes, that is, skipping 1024)
        //
        ui32CardSize = (ui32CSize + 1) * 512;
    }

    //
    // Return card capacity in kibibytes
    //
    return ui32CardSize;
}


/******************************************************************************
* LOCAL FUNCTIONS
*/
/**************************************************************************//**
* @brief    Local function. Tries to brings the SD Card into SPI mode.
*           The SPI speed must be configured to 100 kHz - 400 kHz for the
*           SPI initialization to work. This function will leave CSn asserted
*           (low) when returning.
*
* @return   Returns card response
******************************************************************************/
static uint8_t
sdEnterSpiMode(void)
{
    uint8_t ui8Count, ui8Response;
    uint32_t ui32Dummy;

    //
    // Deassert CSn
    //
    SDCARD_SPI_END();

    //
    // Clock out 74+ clock cycles with CSn high
    //
    for(ui8Count = 0; ui8Count < 10;  ui8Count++)
    {
        SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);
    }

    //
    // Wait for transfer to complete and empty SPI buffer
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Dummy))
    {
    }

    //
    // Assert CSn
    //
    SDCARD_SPI_BEGIN();

    //
    // Send CMD0 and wait for response 0x01 (try up to 4 times)
    //
    ui8Count = 4;
    do
    {
        //
        // Card is still in native mode, thus CRC (0x95) is required.
        //
        sdCardSendCommand(SDCARD_CMD0, 0, 0x95);
        sdCardGetResponse(&ui8Response, SDCARD_R1);
        ui8Count--;
    }
    while((ui8Count > 0) && (ui8Response != 1));

    //
    // Return response
    //
    return ui8Response;
}


/**************************************************************************//**
* @brief    Local function. It clocks out one dummy byte and returns the
*           answer.
*
* @return   Returns the byte received over SPI
******************************************************************************/
static uint8_t
sdGetByte(void)
{
    uint32_t ui32Data;
    SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);
    SSIDataGet(SDCARD_SSI_BASE, &ui32Data);
    return (uint8_t)(ui32Data & 0x000000FF);
}


/**************************************************************************//**
* @brief    Function waits for data ready token and reads \e ui32ByteCount bytes
*           of data from the SD card. \e ui32ByteCount must be a multiple of 4.
*           This function assumes the SD card CSn signal to be asserted (low).
*
* @param    pui8Buffer      Pointer to destination buffer.
* @param    ui32ByteCount   Number of bytes expected from SD card. Must be a
*                           a multiple of 4, that is, CRC bytes should not be
*                           included in this number.
*
* @return   Returns SDCARD_SUCCESS on success
******************************************************************************/
static uint8_t
sdCardGetData(uint8_t * pui8Buffer, uint16_t ui32ByteCount)
{
    uint32_t ui32Data;
    uint32_t ui32Count = 0;
    uint8_t ui8Response = SDCARD_DUMMY;

    //
    // Wait for card to send data ready token.
    //
    while(ui8Response != SDCARD_SINGLE_DATA_READY)
    {
        ui8Response = sdGetByte();
        ui32Count++;
        if(ui32Count > SDCARD_MAX_RESPONSE_POLL_COUNT)
        {
            //
            // Timeout
            //
            return SDCARD_DATA_READY_TIMEOUT;
        }
    }

    //
    // Retrieve actual data
    //
    for(ui32Count = 0; ui32Count < ui32ByteCount; ui32Count += 4)
    {
        //
        // Send 4 dummy bytes
        //
        SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);
        SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);
        SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);
        SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);

        //
        // Read 4 data bytes and store to buffer
        //
        SSIDataGet(SDCARD_SSI_BASE, &ui32Data);
        pui8Buffer[ui32Count + 0] = (uint8_t)(ui32Data & 0x000000FF);
        SSIDataGet(SDCARD_SSI_BASE, &ui32Data);
        pui8Buffer[ui32Count + 1] = (uint8_t)(ui32Data & 0x000000FF);
        SSIDataGet(SDCARD_SSI_BASE, &ui32Data);
        pui8Buffer[ui32Count + 2] = (uint8_t)(ui32Data & 0x000000FF);
        SSIDataGet(SDCARD_SSI_BASE, &ui32Data);
        pui8Buffer[ui32Count + 3] = (uint8_t)(ui32Data & 0x000000FF);
    }

    //
    // Get two CRC bytes (ignored)
    //
    SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);
    SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);

    //
    // Host must supply 8 clocks after the (last) response byte is retrieved
    //
    SSIDataPut(SDCARD_SSI_BASE, SDCARD_DUMMY);

    //
    // Wait for data transfer to finish and empty FIFO
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(SDCARD_SSI_BASE, &ui32Data))
    {
    }

    //
    // Return status
    //
    return SDCARD_SUCCESS;
}


/**************************************************************************//**
* @brief    Local function. Gets the response from the SD Card. In SPI mode,
*           SD cards sends a response after every command.
*
* @param    pui8Buffer      Pointer to where to store response.
* @param    ui8ByteCount    The expected response type's size in bytes. It is
*                           recommended to use one of the following values
*                           \li \b SDCARD_R1 (1 byte response)
*                           \li \b SDCARD_R2 (2 byte response)
*                           \li \b SDCARD_R3 (5 byte response)
*
* @return   Returns first byte of CD card response
******************************************************************************/
static uint8_t
sdCardGetResponse(uint8_t * pui8Buffer, uint8_t ui8ByteCount)
{
    uint8_t ui8Response;
    uint8_t ui8WaitCount = 0;

    //
    // Up to 9 clocks until first response byte is received
    //
    while(ui8WaitCount++ < 9)
    {
        ui8Response = sdGetByte();
        if(ui8Response != 0xFF)
        {
            *pui8Buffer++ = ui8Response;
            ui8ByteCount--;
            break;
        }
    }

    //
    // Retrieve remaining bytes in response
    //
    while(ui8ByteCount--)
    {
        *pui8Buffer++ = sdGetByte();
    }

    //
    // Host must supply 8 clocks after the (last) response byte is retrieved
    //
    sdGetByte();

    //
    // Return first byte in response
    //
    return ui8Response;
}


/**************************************************************************//**
* @brief    Local function. Sends a command frame to the SD Card.
*           @note Application specific commands (ACMD<n>) must be preceded by
*           CMD55 (and response).
*
* @param    ui8Cmd      The command to send.
* @param    ui32Data    The options or parameters for the command.
* @param    ui8Crc      CRC Checksum, by default CRC is off in SPI mode but the
*                       correct checksum is needed for, for example, CMD0 to
*                       enter SPI mode.
*
* @return   None
******************************************************************************/
static void
sdCardSendCommand(const uint8_t ui8Cmd, uint32_t ui32Data,
                  const uint8_t ui8Crc)
{
    uint32_t ui32Dummy;
    uint8_t ui8Frame[6];
    uint8_t ui8Count;

    //
    // Build frame ([0]Command, [4:1]arguments - LSB first], [5]CRC checksum)
    //
    ui8Frame[0] = (ui8Cmd | 0x40);
    ui8Frame[1] = (uint8_t)(ui32Data >> 24);
    ui8Frame[2] = (uint8_t)(ui32Data >> 16);
    ui8Frame[3] = (uint8_t)(ui32Data >>  8);
    ui8Frame[4] = (uint8_t)(ui32Data);
    ui8Frame[5] = ui8Crc;

    //
    // Send frame
    //
    for(ui8Count = 0; ui8Count < 6; ui8Count++)
    {
        SSIDataPut(SDCARD_SSI_BASE, ui8Frame[ui8Count]);
    }

    //
    // Wait for frame to be sent and empty in fifo
    //
    while(SSIBusy(SDCARD_SSI_BASE))
    {
    }
    while(SSIDataGetNonBlocking(BSP_SPI_SSI_BASE, &ui32Dummy))
    {
    }
}


/**************************************************************************//**
* Close the Doxygen group.
* @}
******************************************************************************/
#endif // #ifndef SDCARD_EXCLUDE
