/**************************************************************************************************
  Filename:       _hal_uart.c
  Revised:        $Date: 2013-05-17 11:25:11 -0700 (Fri, 17 May 2013) $
  Revision:       $Revision: 34355 $

  Description:    This file contains the interface to the UART.


  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#if HAL_UART_USB
#include "_hal_uart_usb.c"
#else
#include "_hal_uart_isr.c"
#endif
/////////////////////////////////////////////////////////////////////////   
/*
#include "string.h"
#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"
#include "usb_in_buffer.h"
#include "usb_out_buffer.h"

#include "OSAL_Memory.h"   
#include "OSAL.h"
#include "MT_UART.h"
#include "MT.h"
extern mtOSALSerialData_t  *pMsg;   
extern USB_EPIN_RINGBUFFER_DATA usbCdcInBufferData;
extern USB_EPOUT_RINGBUFFER_DATA usbCdcOutBufferData;
extern uint8_t pInBuffer[64];
extern uint8_t pOutBuffer[64];
extern uint8_t pAppBuffer[64];
uint8 usb_start_flag = 0;
uint8 usb_end_flag = 0;
extern uint8 USB_Msg_in[128];
extern uint8 dataLen;
*/

/////////////////////////////////////////////////////////////////////////
/* ------------------------------------------------------------------------------------------------
 *                                           Global Functions
 * ------------------------------------------------------------------------------------------------
 */

/*************************************************************************************************
 * @fn      HalUARTInit()
 *
 * @brief   Initialize the UART
 *
 * @param   none
 *
 * @return  none
 *************************************************************************************************/
void HalUARTInit(void)
{
#if HAL_UART_USB
  HalUARTInitUSB();
#else
  HalUARTInitIsr();
#endif
}

/*************************************************************************************************
 * @fn      HalUARTOpen()
 *
 * @brief   Open a port based on the configuration
 *
 * @param   port   - UART port
 *          config - contains configuration information
 *          cBack  - Call back function where events will be reported back
 *
 * @return  Status of the function call
 *************************************************************************************************/
uint8 HalUART0Open(uint8 port, halUARTCfg_t *config)
{
#if HAL_UART_USB
  (void)port;
  HalUARTOpenUSB(config);
  return HAL_UART_SUCCESS;
#else
  return(HalUART0OpenIsr(port, config));
#endif
}

uint8 HalUART1Open(uint8 port, halUARTCfg_t *config)
{
#if HAL_UART_USB
  (void)port;
  HalUARTOpenUSB(config);
  return HAL_UART_SUCCESS;
#else
  return(HalUART1OpenIsr(port, config));
#endif
}

/*************************************************************************************************
 * @fn      Hal_UARTPoll
 *
 * @brief   This routine simulate polling and has to be called by the main loop
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
void HalUARTPoll(void)
{
#ifdef HAL_UART_USB
  HalUARTPollUSB();
#else
  HalUARTPollIsr();
  HalUART0PollIsr();
  /*
  usbCdcProcessEvents();
    //
    // Implement COM-port loopback
    //
    uint16_t count = 0;
    count = usbibufGetMaxPushCount(&usbCdcInBufferData);

    uint16_t maxPopCount = 0;
    maxPopCount = usbobufGetMaxPopCount(&usbCdcOutBufferData);

    if (count > maxPopCount)
    {
        count = maxPopCount;
    }

    if (count)
    {
        usbobufPop(&usbCdcOutBufferData, pAppBuffer, count); // receive function
        //usbibufPush(&usbCdcInBufferData, pAppBuffer, count); // send function
        uint16 index = 0;
        for(index = 0; index < count; index++)
            USB_Msg_in[index + 64 * usb_start_flag] = *(pAppBuffer + index);
        usb_start_flag++;                        
        usb_end_flag = 0;
    }
    else if (!count && usb_start_flag)
    {
        usb_start_flag = 0;
        usb_end_flag = 1;         
        dataLen = USB_Msg_in[10];
        
        pMsg = (mtOSALSerialData_t *)osal_msg_allocate( 50 );
        if (pMsg)
        {
            pMsg->hdr.event = CMD_USB_MSG;
            pMsg->msg = (uint8 *)(pMsg + 1);
            pMsg->msg[0] = 1 ;                                  
        }
        osal_msg_send(App_TaskID, (uint8 *)pMsg );
        osal_msg_deallocate((uint8*)pMsg );
    }
*/
#endif
}

/*************************************************************************************************
 * @fn      HalUARTClose()
 *
 * @brief   Close the UART
 *
 * @param   port - UART port (not used.)
 *
 * @return  none
 *************************************************************************************************/
void HalUARTClose(uint8 port)
{
#ifdef HAL_UART_USB
  
#else   
 HalUARTCloseIsr(port);
#endif
}

/*************************************************************************************************
 * @fn      HalUARTRead()
 *
 * @brief   Read a buffer from the UART
 *
 * @param   port - UART port (not used.)
 *          ppBuffer - pointer to a pointer that points to the data that will be read
 *          length - length of the requested buffer
 *
 * @return  length of buffer that was read
 *************************************************************************************************/
uint16 HalUARTRead ( uint8 port, uint8 *pBuffer, uint16 length )
{
#if HAL_UART_USB
  return HalUARTRx(pBuffer, length);
#else
  return (HalUARTReadIsr( port, pBuffer, length ));
#endif
}

uint16 HalUART0Read ( uint8 port, uint8 *pBuffer, uint16 length )
{
#if HAL_UART_USB
  return HalUARTRx(pBuffer, length);
#else
  return (HalUART0ReadIsr( port, pBuffer, length ));
#endif
}

/*************************************************************************************************
 * @fn      HalUARTWrite()
 *
 * @brief   Write a buffer to the UART
 *
 * @param   port    - UART port (not used.)
 *          pBuffer - pointer to the buffer that will be written
 *          length  - length of
 *
 * @return  length of the buffer that was sent
 *************************************************************************************************/
uint16 HalUART1Write(uint8 port, uint8 *pBuffer, uint16 length)
{
#if HAL_UART_USB
  return HalUARTTx(pBuffer, length);
#else
  return (HalUART1WriteIsr( port, pBuffer, length ));
#endif
}

uint16 HalUART0Write(uint8 port, uint8 *pBuffer, uint16 length)
{
#if HAL_UART_USB
  return HalUARTTx(pBuffer, length);
#else
  return (HalUART0WriteIsr( port, pBuffer, length ));
#endif
}

/*************************************************************************************************
 * @fn      Hal_UART_RxBufLen()
 *
 * @brief   Calculate Rx Buffer length of a port
 *
 * @param   port - UART port (not used.)
 *
 * @return  length of current Rx Buffer
 *************************************************************************************************/
uint16 Hal_UART_RxBufLen (uint8 port)
{
#if HAL_UART_USB
  return HalUARTRxAvailUSB();
#else
  return (Hal_UART_RxBufLenIsr(port));
#endif
}

uint16 Hal_UART0_RxBufLen (uint8 port)
{
#if HAL_UART_USB
  return HalUARTRxAvailUSB();
#else
  return (Hal_UART0_RxBufLenIsr(port));
#endif
}

/**************************************************************************************************
*/



