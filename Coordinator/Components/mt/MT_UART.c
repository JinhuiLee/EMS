/***************************************************************************************************
  Filename:       MT_UART.c
  Revised:        $Date: 2009-03-12 16:25:22 -0700 (Thu, 12 Mar 2009) $
  Revision:       $Revision: 19404 $

  Description:  This module handles anything dealing with the serial port.

  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED ¡°AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

***************************************************************************************************/

/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include "ZComDef.h"
#include "OSAL.h"
#include "hal_uart.h"
#include "MT.h"
#include "MT_UART.h"
#include "OSAL_Memory.h"
#include "hal_lcd.h"
#include "hal_led.h"


/***************************************************************************************************
 * MACROS
 ***************************************************************************************************/

/***************************************************************************************************
 * CONSTANTS
 ***************************************************************************************************/
/* State values for ZTool protocal */
#define START1_STATE            0x00
#define A0A7_START2_STATE       0x01
#define LEN_STATE               0x02
#define DATA_STATE              0x03
#define DATA2_STATE             0x04
#define CHS_STATE               0x05
#define END_STATE               0x06


/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/
/* Used to indentify the application ID for osal task */
uint8 App_TaskID;

/* ZTool protocal parameters */
uint8 state;
uint8  CMD_Token[2];
uint8  LEN_Token;
uint8  CHS_Token;
uint8  END_Token;
mtOSALSerialData_t  *pMsg;
uint8  tempDataLen;
uint8  A0A7_Num;
uint8  A0A7_Arr[9];
uint8 CHS_cnt;
uint8 START_rec;
uint8 CHS_1;
#if defined (ZAPP_P1) || defined (ZAPP_P2)
uint16  MT_UartMaxZAppBufLen;
bool    MT_UartZAppRxStatus;
#endif

uint8 uart0_flag = 2;
uint8 uart1_flag = 2;

/***************************************************************************************************
 *                                          LOCAL FUNCTIONS
 ***************************************************************************************************/

/***************************************************************************************************
 * @fn      MT_UartInit
 *
 * @brief   Initialize MT with UART support
 *
 * @param   None
 *
 * @return  None
***************************************************************************************************/
void MT_UartInit ()
{
    halUARTCfg_t uartConfig;

    /* Initialize APP ID */
    App_TaskID = 0;

    /* UART Configuration */
    uartConfig.configured           = TRUE;
    uartConfig.baudRate             = HAL_UART_BR_115200;
    uartConfig.flowControl          = FALSE;
    uartConfig.flowControlThreshold = MT_UART_DEFAULT_THRESHOLD;
    uartConfig.rx.maxBufSize        = MT_UART_DEFAULT_MAX_RX_BUFF;
    uartConfig.tx.maxBufSize        = MT_UART_DEFAULT_MAX_TX_BUFF;
    uartConfig.idleTimeout          = MT_UART_DEFAULT_IDLE_TIMEOUT;
    uartConfig.intEnable            = TRUE;
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
    uartConfig.callBackFunc         = MT_UartProcessZToolData;
#elif defined (ZAPP_P1) || defined (ZAPP_P2)
    uartConfig.callBackFunc         = MT_UartProcessZAppData;
#else
    uartConfig.callBackFunc         = NULL;
#endif

   /* Start UART */
#if defined (MT_UART_DEFAULT_PORT)
    
    HalUART0Open (HAL_UART_PORT_0, &uartConfig);
    HalUART1Open (HAL_UART_PORT_1, &uartConfig);
#else
    /* Silence IAR compiler warning */
    (void)uartConfig;
#endif


    /* Initialize for ZApp */
#if defined (ZAPP_P1) || defined (ZAPP_P2)
    /* Default max bytes that ZAPP can take */
    MT_UartMaxZAppBufLen  = 1;
    MT_UartZAppRxStatus   = MT_UART_ZAPP_RX_READY;
#endif
    
}

/***************************************************************************************************
 * @fn      MT_SerialRegisterTaskID
 *
 * @brief   This function registers the taskID of the application so it knows
 *          where to send the messages whent they come in.
 *
 * @param   void
 *
 * @return  void
 ***************************************************************************************************/
void MT_UartRegisterTaskID( byte taskID )
{
    App_TaskID = taskID;
}

/***************************************************************************************************
 * @fn      SPIMgr_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ***************************************************************************************************/
byte MT_UartCalcFCS( uint8 *msg_ptr, uint8 len )
{
    byte x;
    byte xorResult;

    xorResult = 0;

    for ( x = 0; x < len; x++, msg_ptr++ )
        xorResult = xorResult ^ *msg_ptr;

    return ( xorResult );
}


/***************************************************************************************************
 * @fn      MT_UartProcessZToolData
 *
 * @brief   | SOP | Data Length  |   CMD   |   Data   |  FCS  |
 *          |  1  |     1        |    2    |  0-Len   |   1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port     - UART port
 *          event    - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void MT_UartProcessZToolData ( uint8 port, uint8 event )
{
    uint8  ch;
    uint8  bytesInRxBuffer;
    int firstloo = 0;
    uint8 coor_index = 0;
    (void)event;  // Intentionally unreferenced parameter
    while (Hal_UART0_RxBufLen(port))
    {
        HalUART0Read (port, &ch, 1);
        
        switch (state)
        {

        case START1_STATE:
            if (ch == MT_UART_START_FLAG)
            {
                START_rec = ch;
                state = A0A7_START2_STATE;
                A0A7_Num = 0 ;
            }

            break;


        case A0A7_START2_STATE:
            A0A7_Arr[A0A7_Num++] = ch;
            if (A0A7_Num == 9)
            {
                state = LEN_STATE;
                //HalLcdWriteString( "send successfully1", HAL_LCD_LINE_7 );
            }
            else
                state = A0A7_START2_STATE;

            break;

        case LEN_STATE:

            LEN_Token = ch;
            tempDataLen = 0;          
            state = DATA_STATE;
            //HalLcdWriteString( "send successfully2", HAL_LCD_LINE_7 );
                        
            break;

        case DATA_STATE:
             
            /*Only excute this at the first loop*/
            if(firstloo == 0)
            {
                /*
                if(ch == 0x0A)
                {

                    LEN_Token = (uint8)(LEN_Token + 49);
                    HalLcdWriteString( "eeee", HAL_LCD_LINE_6 );
                }
               */
                /* Allocate memory for the data */
                pMsg = (mtOSALSerialData_t *)osal_msg_allocate( sizeof ( mtOSALSerialData_t ) +
                        MT_RPC_FRAME_HDR_SZ + LEN_Token + 20 );
                if (pMsg)
                {
                    /* Fill up what we can */
                    pMsg->hdr.event = CMD_SERIAL_MSG;
                    pMsg->msg = (uint8 *)(pMsg + 1);
                    pMsg->msg[MT_RPC_POS_LEN] = LEN_Token ;
                   
                    
                }
                else
                {
                    state = START1_STATE;
                    break;
                }
            }
            firstloo++;
            
            /* Fill in the buffer the first byte of the data */
            pMsg->msg[MT_RPC_FRAME_HDR_SZI + tempDataLen++] = ch;

            /* Check number of bytes left in the Rx buffer */
            bytesInRxBuffer = Hal_UART_RxBufLen(port);

            /* If the remain of the data is there, read them all, otherwise, just read enough */
            if (bytesInRxBuffer <= LEN_Token - tempDataLen)
            {
                HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZI + tempDataLen], bytesInRxBuffer);
                tempDataLen += bytesInRxBuffer;
                HalLcdWriteString( "FAILi", HAL_LCD_LINE_7 );//5
            }
            else
            {
                HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZI + tempDataLen], LEN_Token - tempDataLen );
                tempDataLen += (LEN_Token - tempDataLen);
                HalLcdWriteString( "FAILii", HAL_LCD_LINE_7 );//5
            }

            if ( tempDataLen == LEN_Token )
            {
                state = CHS_STATE;
                firstloo = 0;
                HalLcdWriteString( "FAILiii", HAL_LCD_LINE_7 );
            }
            else
            {
                state = DATA_STATE;
                HalLcdWriteString( "FAILiiii", HAL_LCD_LINE_7 );
            }

            //HalLcdWriteString( "send successfully3", HAL_LCD_LINE_7 );
            break;

        case CHS_STATE:

            CHS_Token = ch;
            CHS_1 = 0;
            state = END_STATE;
            pMsg->msg[MT_RPC_FRAME_HDR_SZI + LEN_Token] = CHS_Token;
            break;

        case END_STATE:           
            END_Token = ch;
            //if(ch == MT_UART_END_FLAG)
            //{
            //    state = START1_STATE;
            //}
            pMsg->msg[MT_RPC_FRAME_HDR_SZI + LEN_Token + 1] = END_Token;
            for(coor_index = 0; coor_index < 8; coor_index++)                      ////12.16
           {
              pMsg->msg[MT_RPC_FRAME_HDR_SZI + LEN_Token + 2 + coor_index] = A0A7_Arr[coor_index];
           }
            //HalLcdWriteString( "send successfully4", HAL_LCD_LINE_7 );
            osal_msg_send(App_TaskID, (byte *)pMsg );

            /* Reset the state, send or discard the buffers at this point */
            state = START1_STATE;            
            osal_msg_deallocate((uint8*)pMsg );
            uart0_flag = 1;
            uart1_flag = 0;
            break;

        default:
            break;
        }
               
    }
    
    while (Hal_UART_RxBufLen(port))
    {
        HalUARTRead (port, &ch, 1);
        
        switch (state)
        {

        case START1_STATE:
            if (ch == MT_UART_START_FLAG)
            {
                START_rec = ch;
                state = A0A7_START2_STATE;
                A0A7_Num = 0 ;
            }

            break;


        case A0A7_START2_STATE:
            A0A7_Arr[A0A7_Num++] = ch;
            if (A0A7_Num == 9)
            {
                state = LEN_STATE;
                //HalLcdWriteString( "send successfully1", HAL_LCD_LINE_7 );
            }
            else
                state = A0A7_START2_STATE;

            break;

        case LEN_STATE:

            LEN_Token = ch;
            tempDataLen = 0;          
            state = DATA_STATE;
            //HalLcdWriteString( "send successfully2", HAL_LCD_LINE_7 );
                        
            break;

        case DATA_STATE:
             
            /*Only excute this at the first loop*/
            if(firstloo == 0)
            {
              /*
                if(ch == 0x0A)
                {

                    LEN_Token = (uint8)(LEN_Token + 49);
                    HalLcdWriteString( "eeee", HAL_LCD_LINE_6 );
                }
              */
                /* Allocate memory for the data */
                pMsg = (mtOSALSerialData_t *)osal_msg_allocate( sizeof ( mtOSALSerialData_t ) +
                        MT_RPC_FRAME_HDR_SZ + LEN_Token + 20 );
                if (pMsg)
                {
                    /* Fill up what we can */
                    pMsg->hdr.event = CMD_SERIAL_MSG;
                    pMsg->msg = (uint8 *)(pMsg + 1);
                    pMsg->msg[MT_RPC_POS_LEN] = LEN_Token ;
                                       
                }
                else
                {
                    state = START1_STATE;
                    break;
                }
            }
            firstloo++;
            
            /* Fill in the buffer the first byte of the data */
            pMsg->msg[MT_RPC_FRAME_HDR_SZI + tempDataLen++] = ch;

            /* Check number of bytes left in the Rx buffer */
            bytesInRxBuffer = Hal_UART_RxBufLen(port);

            /* If the remain of the data is there, read them all, otherwise, just read enough */
            if (bytesInRxBuffer <= LEN_Token - tempDataLen)
            {
                HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZI + tempDataLen], bytesInRxBuffer);
                tempDataLen += bytesInRxBuffer;
                HalLcdWriteString( "FAILi", HAL_LCD_LINE_7 );//5
            }
            else
            {
                HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZI + tempDataLen], LEN_Token - tempDataLen );
                tempDataLen += (LEN_Token - tempDataLen);
                HalLcdWriteString( "FAILii", HAL_LCD_LINE_7 );//5
            }

            if ( tempDataLen == LEN_Token )
            {
                state = CHS_STATE;
                firstloo = 0;
                HalLcdWriteString( "FAILiii", HAL_LCD_LINE_4 );
            }
            else
            {
                state = DATA_STATE;
                HalLcdWriteString( "FAILiiii", HAL_LCD_LINE_4 );
            }

            //HalLcdWriteString( "send successfully3", HAL_LCD_LINE_7 );
            break;

        case CHS_STATE:

            CHS_Token = ch;
            CHS_1 = 0;
            state = END_STATE;
            pMsg->msg[MT_RPC_FRAME_HDR_SZI + LEN_Token] = CHS_Token;
            break;

        case END_STATE:

            END_Token = ch;
            //if(ch == MT_UART_END_FLAG)
            //{
            //    state = START1_STATE;
            //}
            pMsg->msg[MT_RPC_FRAME_HDR_SZI + LEN_Token + 1] = END_Token;
            for(coor_index = 0; coor_index < 8; coor_index++)                      ////12.16
           {
              pMsg->msg[MT_RPC_FRAME_HDR_SZI + LEN_Token + 2 + coor_index] = A0A7_Arr[coor_index];
           }
            HalLcdWriteString( "send successfully4", HAL_LCD_LINE_4 );
            osal_msg_send(App_TaskID, (byte *)pMsg );

            /* Reset the state, send or discard the buffers at this point */
            state = START1_STATE;            
            osal_msg_deallocate((uint8*)pMsg );
            uart0_flag = 0;
            uart1_flag = 1;
            break;

        default:
            break;
        }
        
        
    }
}

#if defined (ZAPP_P1) || defined (ZAPP_P2)
/***************************************************************************************************
 * @fn      MT_UartProcessZAppData
 *
 * @brief   | SOP | CMD  |   Data Length   | FSC  |
 *          |  1  |  2   |       1         |  1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port    - UART port
 *          event   - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void MT_UartProcessZAppData ( uint8 port, uint8 event )
{

    osal_event_hdr_t  *msg_ptr;
    uint16 length = 0;
    uint16 rxBufLen  = Hal_UART_RxBufLen(MT_UART_DEFAULT_PORT);

    /*
       If maxZAppBufferLength is 0 or larger than current length
       the entire length of the current buffer is returned.
    */
    if ((MT_UartMaxZAppBufLen != 0) && (MT_UartMaxZAppBufLen <= rxBufLen))
    {
        length = MT_UartMaxZAppBufLen;
    }
    else
    {
        length = rxBufLen;
    }

    /* Verify events */
    if (event == HAL_UART_TX_FULL)
    {
        // Do something when TX if full
        return;
    }

    if (event & ( HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
    {
        if ( App_TaskID )
        {
            /*
               If Application is ready to receive and there is something
               in the Rx buffer then send it up
            */
            if ((MT_UartZAppRxStatus == MT_UART_ZAPP_RX_READY ) && (length != 0))
            {
                /* Disable App flow control until it processes the current data */
                MT_UartAppFlowControl (MT_UART_ZAPP_RX_NOT_READY);

                /* 2 more bytes are added, 1 for CMD type, other for length */
                msg_ptr = (osal_event_hdr_t *)osal_msg_allocate( length + sizeof(osal_event_hdr_t) );
                if ( msg_ptr )
                {
                    msg_ptr->event = SPI_INCOMING_ZAPP_DATA;
                    msg_ptr->status = length;

                    /* Read the data of Rx buffer */
                    HalUARTRead( MT_UART_DEFAULT_PORT, (uint8 *)(msg_ptr + 1), length );

                    /* Send the raw data to application...or where ever */
                    osal_msg_send( App_TaskID, (uint8 *)msg_ptr );
                }
            }
        }
    }
}

/***************************************************************************************************
 * @fn      SPIMgr_ZAppBufferLengthRegister
 *
 * @brief
 *
 * @param   maxLen - Max Length that the application wants at a time
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartZAppBufferLengthRegister ( uint16 maxLen )
{
    /* If the maxLen is larger than the RX buff, something is not right */
    if (maxLen <= MT_UART_DEFAULT_MAX_RX_BUFF)
        MT_UartMaxZAppBufLen = maxLen;
    else
        MT_UartMaxZAppBufLen = 1; /* default is 1 byte */
}

/***************************************************************************************************
 * @fn      SPIMgr_AppFlowControl
 *
 * @brief
 *
 * @param   status - ready to send or not
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartAppFlowControl ( bool status )
{

    /* Make sure only update if needed */
    if (status != MT_UartZAppRxStatus )
    {
        MT_UartZAppRxStatus = status;
    }

    /* App is ready to read again, ProcessZAppData have to be triggered too */
    if (status == MT_UART_ZAPP_RX_READY)
    {
        MT_UartProcessZAppData (MT_UART_DEFAULT_PORT, HAL_UART_RX_TIMEOUT );
    }

}

#endif //ZAPP

/***************************************************************************************************
***************************************************************************************************/