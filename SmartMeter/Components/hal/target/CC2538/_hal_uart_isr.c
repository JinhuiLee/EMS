/*************************************************************************************************
Filename:       _hal_uart_isr.c
  Revised:        $Date: 2013-10-21 17:25:23 -0700 (Mon, 21 Oct 2013) $
  Revision:       $Revision: 35745 $

  Description:    This file contains the interface to the UART.


  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#include "hal_board.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "OSAL.h"
#include "OSAL_Timers.h"
#include "hw_ioc.h"
#include "hw_uart.h"
#include "gpio.h" 
/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_UART_PORT                      UART1_BASE
#define HAL_UART_SYS_CTRL                  SYS_CTRL_PERIPH_UART1
#define HAL_UART_INT_CTRL                  INT_UART1
#define HalUartISR                         interrupt_uart1
#define HalUart0ISR                        interrupt_uart0
  
//*****************************************************************************

#define EXAMPLE_PIN_UART0_RXD            GPIO_PIN_0
#define EXAMPLE_PIN_UART0_TXD            GPIO_PIN_1
#define EXAMPLE_GPIO_BASE0               GPIO_A_BASE

#define EXAMPLE_PIN_UART1_RXD            GPIO_PIN_2  //RF1.2
#define EXAMPLE_PIN_UART1_TXD            GPIO_PIN_3  //RF1.4
//#define EXAMPLE_PIN_TXD            GPIO_PIN_0  //RF2.11
#define EXAMPLE_GPIO_BASE1               GPIO_C_BASE

//*****************************************************************************

   
/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

const uint32 UBRRTable[] = {
  9600,
  19200,
  38400,
  57600,
  115200
};

static halUARTCfg_t uartRecord;
static halUARTCfg_t uartRecord0;
static bool txMT;
static bool txMT0;
/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void recRst(void);
static void procRx(void);
static void procTx(void);
static void procRx0(void);
static void procTx0(void);

/* ------------------------------------------------------------------------------------------------
 *                                           Global Functions
 * ------------------------------------------------------------------------------------------------
 */

void interrupt_uart(void);

/*************************************************************************************************
 * @fn      HalUARTInitIsr()
 *
 * @brief   Initialize the UART
 *
 * @param   none
 *
 * @return  none
 *************************************************************************************************/
void HalUARTInitIsr(void)
{
   SysCtrlPeripheralEnable(HAL_UART_SYS_CTRL);
   SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);
  /* Setup PB0 as UART_CTS, PD3 as UART_RTS  
   * PA1 as UART_TX and PA0 as UART_RX
   */ 
  //IOCPinConfigPeriphOutput(GPIO_A_BASE, GPIO_PIN_1, IOC_MUX_OUT_SEL_UART1_TXD);
  //IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_0, IOC_UARTRXD_UART1);
  //GPIOPinTypeUARTInput(GPIO_A_BASE, GPIO_PIN_0);
  //GPIOPinTypeUARTOutput(GPIO_A_BASE, GPIO_PIN_1);  
  
    //IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_TXD, IOC_MUX_OUT_SEL_UART1_TXD);
    //GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_TXD);
    //IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_RXD, IOC_UARTRXD_UART1);
    //GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_RXD);

    
    //IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
    //GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD);
    //IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD, IOC_UARTRXD_UART0);
    //GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD);
  recRst();

}

/*************************************************************************************************
 * @fn      HalUARTOpenIsr()
 *
 * @brief   Open a port based on the configuration
 *
 * @param   port   - UART port
 *          config - contains configuration information
 *          cBack  - Call back function where events will be reported back
 *
 * @return  Status of the function call
 *************************************************************************************************/
uint8 HalUART1OpenIsr(uint8 port, halUARTCfg_t *config)
{
    //IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
    //GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD);
    //IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD, IOC_UARTRXD_UART0);
    //GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD);
    
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_TXD, IOC_MUX_OUT_SEL_UART1_TXD);
    GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_TXD);
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_RXD, IOC_UARTRXD_UART1);
    GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_RXD);

  
  if (uartRecord.configured)
  {
    HalUARTClose(port);
  }

  if (config->baudRate > HAL_UART_BR_115200)
  {
    return HAL_UART_BAUDRATE_ERROR;
  }

  if (((uartRecord.rx.pBuffer = osal_mem_alloc(config->rx.maxBufSize)) == NULL) ||
      ((uartRecord.tx.pBuffer = osal_mem_alloc(config->tx.maxBufSize)) == NULL))
  {
    if (uartRecord.rx.pBuffer != NULL)
    {
      osal_mem_free(uartRecord.rx.pBuffer);
      uartRecord.rx.pBuffer = NULL;
    }

    return HAL_UART_MEM_FAIL;
  }
  
  if(config->flowControl)
  {
    IOCPinConfigPeriphOutput(GPIO_D_BASE, GPIO_PIN_3, IOC_MUX_OUT_SEL_UART1_RTS);
    GPIOPinTypeUARTOutput(GPIO_D_BASE, GPIO_PIN_3);
    IOCPinConfigPeriphInput(GPIO_B_BASE, GPIO_PIN_0, IOC_UARTCTS_UART1);
    GPIOPinTypeUARTInput(GPIO_B_BASE, GPIO_PIN_0);
  }
  
  IntEnable(HAL_UART_INT_CTRL);

  uartRecord.configured = TRUE;
  uartRecord.baudRate = config->baudRate;
  uartRecord.flowControl = config->flowControl;
  uartRecord.flowControlThreshold = (config->flowControlThreshold > config->rx.maxBufSize) ? 0 :
                                     config->flowControlThreshold;
  uartRecord.idleTimeout = config->idleTimeout;
  uartRecord.rx.maxBufSize = config->rx.maxBufSize;
  uartRecord.tx.maxBufSize = config->tx.maxBufSize;
  uartRecord.intEnable = config->intEnable;
  uartRecord.callBackFunc = config->callBackFunc;

  UARTConfigSetExpClk(HAL_UART_PORT, SysCtrlClockGet(), UBRRTable[uartRecord.baudRate],             
                         (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE));

  /* FIFO level set to 1/8th for both RX and TX which is 2 bytes */
  UARTFIFOLevelSet(HAL_UART_PORT, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
  UARTFIFOEnable(HAL_UART_PORT);

  /* Clear and enable UART TX, RX, CTS and Recieve Timeout interrupt */
  UARTIntClear(HAL_UART_PORT, (UART_INT_RX | UART_INT_TX | UART_INT_CTS | UART_INT_RT ));
  UARTIntEnable(HAL_UART_PORT, (UART_INT_RX | UART_INT_TX | UART_INT_CTS | UART_INT_RT ));
  

  
  if(config->flowControl)
  {
    /* Enable hardware flow control by enabling CTS and RTS */
    HWREG(HAL_UART_PORT + UART_O_CTL) |= (UART_CTL_CTSEN | UART_CTL_RTSEN );
  }
  UARTEnable(HAL_UART_PORT);
  
  //UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), UBRRTable[uartRecord.baudRate],  
  //                     (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  return HAL_UART_SUCCESS;
}

uint8 HalUART0OpenIsr(uint8 port, halUARTCfg_t *config)
{
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD);
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD, IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD);
    
    if (uartRecord0.configured)
    {
      HalUARTClose(port);
    }

    if (config->baudRate > HAL_UART_BR_115200)
    {
      return HAL_UART_BAUDRATE_ERROR;
    }

    if (((uartRecord0.rx.pBuffer = osal_mem_alloc(config->rx.maxBufSize)) == NULL) ||
        ((uartRecord0.tx.pBuffer = osal_mem_alloc(config->tx.maxBufSize)) == NULL))
    {
      if (uartRecord0.rx.pBuffer != NULL)
      {
        osal_mem_free(uartRecord0.rx.pBuffer);
        uartRecord0.rx.pBuffer = NULL;
      }

      return HAL_UART_MEM_FAIL;
    }
    
    if(config->flowControl)
    {
      IOCPinConfigPeriphOutput(GPIO_D_BASE, GPIO_PIN_3, IOC_MUX_OUT_SEL_UART1_RTS);
      GPIOPinTypeUARTOutput(GPIO_D_BASE, GPIO_PIN_3);
      IOCPinConfigPeriphInput(GPIO_B_BASE, GPIO_PIN_0, IOC_UARTCTS_UART1);
      GPIOPinTypeUARTInput(GPIO_B_BASE, GPIO_PIN_0);
    }
    
    IntEnable(INT_UART0);

    uartRecord0.configured = TRUE;
    uartRecord0.baudRate = config->baudRate;
    uartRecord0.flowControl = config->flowControl;
    uartRecord0.flowControlThreshold = (config->flowControlThreshold > config->rx.maxBufSize) ? 0 :
                                       config->flowControlThreshold;
    uartRecord0.idleTimeout = config->idleTimeout;
    uartRecord0.rx.maxBufSize = config->rx.maxBufSize;
    uartRecord0.tx.maxBufSize = config->tx.maxBufSize;
    uartRecord0.intEnable = config->intEnable;
    uartRecord0.callBackFunc = config->callBackFunc;

     //////////////////////////////////////
  UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), UBRRTable[uartRecord0.baudRate],  
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
  UARTFIFOEnable(UART0_BASE);
  UARTIntClear(UART0_BASE, (UART_INT_RX | UART_INT_TX | UART_INT_CTS | UART_INT_RT ));
  UARTIntEnable(UART0_BASE, (UART_INT_RX | UART_INT_TX | UART_INT_CTS | UART_INT_RT ));
  ///////////////////////////////////////
  UARTEnable(UART0_BASE);

  return HAL_UART_SUCCESS;
    
}
/*************************************************************************************************
 * @fn      Hal_UARTPollIsr
 *
 * @brief   This routine simulate polling and has to be called by the main loop
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
void HalUARTPollIsr(void)
{
  uint16 head = uartRecord.tx.bufferHead;
  uint16 tail = uartRecord.tx.bufferTail;
  /* If port is not configured, no point to poll it. */
  if (!uartRecord.configured)  
  {
    return;
  }

  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION(intState);
  procRx();
  procTx();
  HAL_EXIT_CRITICAL_SECTION(intState);

  uint8 evts = 0;
  /* Report if Rx Buffer is full. */
  if ((Hal_UART_RxBufLen(0) + 1) >= uartRecord.rx.maxBufSize)  
  {
    evts = HAL_UART_RX_FULL;
  }

  /* Report if Rx Buffer is idled. */
  if ((uartRecord.rxChRvdTime != 0) &&  
     ((osal_GetSystemClock() - uartRecord.rxChRvdTime) > uartRecord.idleTimeout))
  {
    uartRecord.rxChRvdTime = 0;
    evts |= HAL_UART_RX_TIMEOUT;
  }

  if (Hal_UART_RxBufLen(0) >= uartRecord.rx.maxBufSize - uartRecord.flowControlThreshold)
  {
    evts |= HAL_UART_RX_ABOUT_FULL;
  }

  if (!txMT && (head == tail))
  {
    txMT = true;
    evts |= HAL_UART_TX_EMPTY;
  }

  if (evts && uartRecord.callBackFunc)
  {
    (uartRecord.callBackFunc)(0, evts);
  }

}

void HalUART0PollIsr(void)
{
  uint16 head = uartRecord0.tx.bufferHead;
  uint16 tail = uartRecord0.tx.bufferTail;
  /* If port is not configured, no point to poll it. */
  if (!uartRecord0.configured)  
  {
    return;
  }

  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION(intState);
  procRx0();
  procTx0();
  HAL_EXIT_CRITICAL_SECTION(intState);

  uint8 evts = 0;
  /* Report if Rx Buffer is full. */
  if ((Hal_UART0_RxBufLen(0) + 1) >= uartRecord0.rx.maxBufSize)  
  {
    evts = HAL_UART_RX_FULL;
  }

  /* Report if Rx Buffer is idled. */
  if ((uartRecord0.rxChRvdTime != 0) &&  
     ((osal_GetSystemClock() - uartRecord0.rxChRvdTime) > uartRecord0.idleTimeout))
  {
    uartRecord0.rxChRvdTime = 0;
    evts |= HAL_UART_RX_TIMEOUT;
  }

  if (Hal_UART0_RxBufLen(0) >= uartRecord0.rx.maxBufSize - uartRecord0.flowControlThreshold)
  {
    evts |= HAL_UART_RX_ABOUT_FULL;
  }

  if (!txMT0 && (head == tail))
  {
    txMT0 = true;
    evts |= HAL_UART_TX_EMPTY;
  }

  if (evts && uartRecord0.callBackFunc)
  {
    (uartRecord0.callBackFunc)(0, evts);
  }

}

/*************************************************************************************************
 * @fn      HalUARTCloseIsr()
 *
 * @brief   Close the UART
 *
 * @param   port - UART port (not used.)
 *
 * @return  none
 *************************************************************************************************/
void HalUARTCloseIsr(uint8 port)
{
  (void)port;

  UARTDisable(HAL_UART_PORT);
  
  if (uartRecord.configured)
  {
    (void)osal_mem_free(uartRecord.rx.pBuffer);
    (void)osal_mem_free(uartRecord.tx.pBuffer);
    recRst();
  }
  
  UARTDisable(UART0_BASE);
  if (uartRecord0.configured)
  {
    (void)osal_mem_free(uartRecord0.rx.pBuffer);
    (void)osal_mem_free(uartRecord0.tx.pBuffer);
    recRst();
  }
}

/*************************************************************************************************
 * @fn      HalUARTReadIsr()
 *
 * @brief   Read a buffer from the UART
 *
 * @param   port - UART port (not used.)
 *          ppBuffer - pointer to a pointer that points to the data that will be read
 *          length - length of the requested buffer
 *
 * @return  length of buffer that was read
 *************************************************************************************************/
uint16 HalUARTReadIsr ( uint8 port, uint8 *pBuffer, uint16 length )
{
  uint16 cnt, idx;
  (void)port;

  /* If port is not configured, no point to read it. */
  if (!uartRecord.configured)
  {
    return 0;
  }

  /* If requested length is bigger than what in 
   * buffer, re-adjust it to the buffer length.
   */
  cnt = Hal_UART_RxBufLen(0);
  if (cnt < length)
  {
    length = cnt;
  }

  idx = uartRecord.rx.bufferHead;
  for (cnt = 0; cnt < length; cnt++)
  {
    pBuffer[cnt] = uartRecord.rx.pBuffer[idx++];

    if (idx >= uartRecord.rx.maxBufSize)
    {
      idx = 0;
    }
  }
  uartRecord.rx.bufferHead = idx;

  /* Return number of bytes read. */
  return length;  
}

uint16 HalUART0ReadIsr ( uint8 port, uint8 *pBuffer, uint16 length )
{
  uint16 cnt, idx;
  (void)port;

  /* If port is not configured, no point to read it. */
  if (!uartRecord0.configured)
  {
    return 0;
  }

  /* If requested length is bigger than what in 
   * buffer, re-adjust it to the buffer length.
   */
  cnt = Hal_UART0_RxBufLen(0);
  if (cnt < length)
  {
    length = cnt;
  }

  idx = uartRecord0.rx.bufferHead;
  for (cnt = 0; cnt < length; cnt++)
  {
    pBuffer[cnt] = uartRecord0.rx.pBuffer[idx++];

    if (idx >= uartRecord0.rx.maxBufSize)
    {
      idx = 0;
    }
  }
  uartRecord0.rx.bufferHead = idx;

  /* Return number of bytes read. */
  return length;  
}

/*************************************************************************************************
 * @fn      HalUARTWriteIsr()
 *
 * @brief   Write a buffer to the UART
 *
 * @param   port    - UART port (not used.)
 *          pBuffer - pointer to the buffer that will be written
 *          length  - length of
 *
 * @return  length of the buffer that was sent
 *************************************************************************************************/
uint16 HalUART1WriteIsr(uint8 port, uint8 *pBuffer, uint16 length)
{
  GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_1, 0x02);
  uint32 switch_timenew = osal_GetSystemClock();
  while (osal_GetSystemClock() - switch_timenew < 300)
  {
  };
  
  (void)port;

  if (!uartRecord.configured)
  {
    return 0;
  }

  uint16 idx = uartRecord.tx.bufferHead;
  uint16 cnt = uartRecord.tx.bufferTail;

  if (cnt == idx)
  {
    cnt = uartRecord.tx.maxBufSize;
  }
  else if (cnt > idx)
  {
    cnt = uartRecord.tx.maxBufSize - cnt + idx;
  }
  else /* (cnt < idx) */
  {
    cnt = idx - cnt;
  }

  /* Accept "all-or-none" on write request. */
  if (cnt < length)
  {
    return 0;
  }

  txMT = false;
  idx = uartRecord.tx.bufferTail;

  for (cnt = 0; cnt < length; cnt++)
  {
    uartRecord.tx.pBuffer[idx++] = pBuffer[cnt];

    if (idx >= uartRecord.tx.maxBufSize)
    {
      idx = 0;
    }
  }

  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION(intState);
  uartRecord.tx.bufferTail = idx;
  procTx();
  HAL_EXIT_CRITICAL_SECTION(intState);
  
   switch_timenew = osal_GetSystemClock();
    while (osal_GetSystemClock() - switch_timenew < 300)
    {
    };
    GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_1, 0x00);  
  /* Return the number of bytes actually put into the buffer. */
  return length;  
}


uint16 HalUART0WriteIsr(uint8 port, uint8 *pBuffer, uint16 length)
{
  (void)port;

  if (!uartRecord0.configured)
  {
    return 0;
  }

  uint16 idx = uartRecord0.tx.bufferHead;
  uint16 cnt = uartRecord0.tx.bufferTail;

  if (cnt == idx)
  {
    cnt = uartRecord0.tx.maxBufSize;
  }
  else if (cnt > idx)
  {
    cnt = uartRecord0.tx.maxBufSize - cnt + idx;
  }
  else /* (cnt < idx) */
  {
    cnt = idx - cnt;
  }

  /* Accept "all-or-none" on write request. */
  if (cnt < length)
  {
    return 0;
  }

  txMT0 = false;
  idx = uartRecord0.tx.bufferTail;

  for (cnt = 0; cnt < length; cnt++)
  {
    uartRecord0.tx.pBuffer[idx++] = pBuffer[cnt];

    if (idx >= uartRecord0.tx.maxBufSize)
    {
      idx = 0;
    }
  }

  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION(intState);
  uartRecord0.tx.bufferTail = idx;
  procTx0();
  HAL_EXIT_CRITICAL_SECTION(intState);

  /* Return the number of bytes actually put into the buffer. */
  return length;  
}

/*************************************************************************************************
 * @fn      Hal_UART_RxBufLenIsr()
 *
 * @brief   Calculate Rx Buffer length of a port
 *
 * @param   port - UART port (not used.)
 *
 * @return  length of current Rx Buffer
 *************************************************************************************************/
uint16 Hal_UART_RxBufLenIsr(uint8 port)
{
  int16 length = uartRecord.rx.bufferTail;
  (void)port;

  length -= uartRecord.rx.bufferHead;
  if  (length < 0)
    length += uartRecord.rx.maxBufSize;

  return (uint16)length;
}

uint16 Hal_UART0_RxBufLenIsr(uint8 port)
{
  int16 length = uartRecord0.rx.bufferTail;
  (void)port;

  length -= uartRecord0.rx.bufferHead;
  if  (length < 0)
    length += uartRecord0.rx.maxBufSize;

  return (uint16)length;
}

/*************************************************************************************************
 * @fn      Hal_UART_TxBufLen()
 *
 * @brief   Calculate Tx Buffer length of a port
 *
 * @param   port - UART port (not used.)
 *
 * @return  length of current Tx buffer
 *************************************************************************************************/
uint16 Hal_UART_TxBufLen( uint8 port )
{
  int16 length = uartRecord.tx.bufferTail;
  (void)port;

  length -= uartRecord.tx.bufferHead;
  if  (length < 0)
    length += uartRecord.tx.maxBufSize;

  return (uint16)length;
}

uint16 Hal_UART0_TxBufLen( uint8 port )
{
  int16 length = uartRecord0.tx.bufferTail;
  (void)port;

  length -= uartRecord0.tx.bufferHead;
  if  (length < 0)
    length += uartRecord0.tx.maxBufSize;

  return (uint16)length;
}

/*************************************************************************************************
 * @fn      recRst()
 *
 * @brief   Reset the UART record.
 *
 * @param   none
 *
 * @return  none
 *************************************************************************************************/
static void recRst(void)
{
  uartRecord.configured        = FALSE;
  uartRecord.rx.bufferHead     = 0;
  uartRecord.rx.bufferTail     = 0;
  uartRecord.rx.pBuffer        = (uint8 *)NULL;
  uartRecord.tx.bufferHead     = 0;
  uartRecord.tx.bufferTail     = 0;
  uartRecord.tx.pBuffer        = (uint8 *)NULL;
  uartRecord.rxChRvdTime       = 0;
  uartRecord.intEnable         = FALSE;
  
  uartRecord0.configured        = FALSE;
  uartRecord0.rx.bufferHead     = 0;
  uartRecord0.rx.bufferTail     = 0;
  uartRecord0.rx.pBuffer        = (uint8 *)NULL;
  uartRecord0.tx.bufferHead     = 0;
  uartRecord0.tx.bufferTail     = 0;
  uartRecord0.tx.pBuffer        = (uint8 *)NULL;
  uartRecord0.rxChRvdTime       = 0;
  uartRecord0.intEnable         = FALSE;
}

/*************************************************************************************************
 * @fn      procRx
 *
 * @brief   Process Rx bytes.
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
static void procRx(void)
{
  uint16 tail = uartRecord.rx.bufferTail;

  while (UARTCharsAvail(HAL_UART_PORT))
  //while (UARTCharsAvail(UART0_BASE))
  {
    uartRecord.rx.pBuffer[tail++] = UARTCharGetNonBlocking(HAL_UART_PORT);
    //uartRecord.rx.pBuffer[tail++] = UARTCharGetNonBlocking(UART0_BASE);
    if (tail >= uartRecord.rx.maxBufSize)
    {
      tail = 0;
    }
  }

  if (uartRecord.rx.bufferTail != tail)
  {
    uartRecord.rx.bufferTail = tail;
    uartRecord.rxChRvdTime = osal_GetSystemClock();
  }
}


static void procRx0(void)
{
  uint16 tail = uartRecord0.rx.bufferTail;

  //while (UARTCharsAvail(HAL_UART_PORT))
  while (UARTCharsAvail(UART0_BASE))
  {
    //uartRecord.rx.pBuffer[tail++] = UARTCharGetNonBlocking(HAL_UART_PORT);
    uartRecord0.rx.pBuffer[tail++] = UARTCharGetNonBlocking(UART0_BASE);
    if (tail >= uartRecord0.rx.maxBufSize)
    {
      tail = 0;
    }
  }

  if (uartRecord0.rx.bufferTail != tail)
  {
    uartRecord0.rx.bufferTail = tail;
    uartRecord0.rxChRvdTime = osal_GetSystemClock();
  }
}
/*************************************************************************************************
 * @fn      procTx
 *
 * @brief   Process Tx bytes.
 *
 * @param   void
 *
 * @return  void
 *************************************************************************************************/
static void procTx(void)
{
  uint16 head = uartRecord.tx.bufferHead;
  uint16 tail = uartRecord.tx.bufferTail;

  while ((head != tail) && (UARTCharPutNonBlocking(HAL_UART_PORT, uartRecord.tx.pBuffer[head])))
  {
    if (++head >= uartRecord.tx.maxBufSize)
    {
      head = 0;
    }
  }

  uartRecord.tx.bufferHead = head;
}

static void procTx0(void)
{
  uint16 head = uartRecord0.tx.bufferHead;
  uint16 tail = uartRecord0.tx.bufferTail;

  while ((head != tail) && (UARTCharPutNonBlocking(UART0_BASE, uartRecord0.tx.pBuffer[head])))
  {
    if (++head >= uartRecord0.tx.maxBufSize)
    {
      head = 0;
    }
  }

  uartRecord0.tx.bufferHead = head;
}
/*************************************************************************************************
 * @fn      UART Rx/Tx ISR
 *
 * @brief   Called when a serial byte is ready to read and/or write.
 * NOTE:   Assumes that uartRecord.configured is TRUE if this interrupt is enabled.
 *
 * @param   void
 *
 * @return  void
**************************************************************************************************/
void HalUartISR(void)
{
  UARTIntClear(HAL_UART_PORT, (UART_INT_RX |  UART_INT_RT));
  procRx();

  UARTIntClear(HAL_UART_PORT, (UART_INT_TX | UART_INT_CTS));
  procTx();
}

void HalUart0ISR(void)
{
  UARTIntClear(UART0_BASE, (UART_INT_RX |  UART_INT_RT));
  procRx0();

  UARTIntClear(UART0_BASE, (UART_INT_TX | UART_INT_CTS));
  procTx0();
}
/**************************************************************************************************
*/
