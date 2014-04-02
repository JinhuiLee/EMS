/**************************************************************************************************
  Filename:       zcl_SmartMeter.c
  Revised:        $Date: 2013-10-18 11:49:27 -0700 (Fri, 18 Oct 2013) $
  Revision:       $Revision: 35718 $

  Description:    Zigbee Cluster Library - sample device application.


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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
  This device will act as a temperature sensor. It updates the current
  temperature on the thermostat when the user sends the desired
  temperature using SW1.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Send current temperature
    - SW2: Invoke EZMode
    - SW3: Adjust temperature
    - SW5: Go to Help screen

  Temperature:
    - SW1: Increase temperature
    - SW3: Decrease temperature
    - SW5: Enter temperature
  ----------------------------------------
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_ms.h"

#include "zcl_smartmeter.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
      
/* adopted from Guo Xu's code */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "hw_memmap.h"
#include "gpio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "interrupt.h"
#include "adc.h"
#include "sys_ctrl.h"
#include "hw_sys_ctrl.h"
#include "systick.h"
#include "uartstdio.h"
#include "hw_cctest.h"
#include "hw_rfcore_xreg.h"

#include "bsp.h"
#include "bsp_key.h"
#include "lcd_dogm128_6.h"
/*********************************************************************
 * MACROS
 */

// how often to report temperature in ms
//#define SmartMeter_REPORT_INTERVAL   10000

// how often to sample ADC in millisecond
#define SmartMeter_PowerCal_INTERVAL   1 



/*********************************************************************
 * CONSTANTS
 */
#define USR_RX_GET 0xC0
#define USR_TX_GET 0xC1
#define USR_RT_SET 0xC2
#define USR_TX_SET 0xC3
#define COM_PARAM  0xC4
#define COM_DATA   0xC5
#define POWER_RESET  0xC6
#define POWER_RELAY  0xC7
#define POWER_START  0xC8
#define FLASH_PARAM  0x0401

//#define SUCCESS  0xB0
#define FAILURE  0xB1

#define ATTRID_MS_PARAMETER_MEASURED_VALUE  0xA0
#define ATTRID_MS_DATA_MEASURED_VALUE  0xA1
#define ATTRID_MS_ADDRESS_MEASURED_VALUE 0xA2
#define ATTRID_MS_COMMAND_RESET_MEASURED_VALUE 0xA3
#define ATTRID_MS_COMMAND_RELAY_MEASURED_VALUE 0xA4
#define ATTRID_MS_COMMAND_RESTART_MEASURED_VALUE 0xA5   
/*********************************************************************
* Pin Definition
*/
const uint16 senPinVoltage = SOCADC_AIN6;
const uint16 senPinCurrent = SOCADC_AIN7;

//! This example uses the following peripherals and I/O signals.
//!
//! The following UART signals are configured only for displaying console
//! messages.  These are not required for operation of
//! ADC
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
// UART pin for test purpose
#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1
#define EXAMPLE_GPIO_UART_BASE          GPIO_A_BASE
/
 
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSmartMeter_TaskID;

uint8 zclSmartMeterSeqNum;

static byte gPermitDuration = 0x00;

uint16 flagreset;
uint16 relay;
uint16 flaginc;
unit8 SerControl
unit32 ENERGY_RESET_VALUE;

uint16 RMS_V;
uint16 RMS_I;
unit32 POWER;
unit16 POWER1;
unit16 POWER0;
unit32 ENERGY;
unit16 ENERGY1;
unit16 ENERGY0;
unit16 SM_V;
unit16 SM_I;
unit16 STATUS

uint16 MIN_ADC;
uint16 MAX_ADC;
uint16 SAMPLE_INT;
uint16 SAMPLE_WIN;
uint16 MAG;
uint16 MAX_V;
uint16 MIN_V;
uint16 MAX_I;
uint16 MIN_I;
uint16 ADC_DELAY;

uint16 senValueV;
uint16 senValueI;
uint16 realVol;
uint16 realCu0;
uint32 powerVa0;
uint32 energyVa0;
uint32 energyVal_Lcd_display;
uint16 RMS_V;
uint12 RMS_I;
uint32 VrmsTemp;
uint64 IrmsTemp;
uint64 powerTemp;
uint32 l_nSamples;

uint64 sm_ADD; //smart meter external IEEE address
uint16 sm_nwkADD;  //smart meter network address


uint8 *psm_ADD; //pointer to smart meter external IEEE address
uint16 *psm_nwkADD; //pointer to smart meter network address

unit64 coordinator_extAddr // Coordinator extended IEEE address
unit8 *pcoordinator_extAddr //pointer to coordinator IEEE address

psm_ADD = &sm_ADD; //set pointer to smart meter address sm_ADD
pcoordinator_extAddr = &coordinator_extAddr  //set pointer to coordinator_Addr
psm_nwkADD = &sm_nwkADD;

unint16 paramReg[10];
unint8 *pparamReg //pointer to paramReg
//memory map paramReg[10];
&paramReg[0] = 0x2000_0020;
pparamReg = &paramReg[0];  //set pointer to paramReg[0]

unint16 dataReg[13];
unint8 *pdataReg //pointer to dataReg
//memory map dataReg
&dataReg[0] = ox2000_0000;
pdataReg = &dataReg[0]; //set pointer to dataReg[0]


/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSmartMeter_DstAddr;
static uint8 numsmAttr = 7;                                     
// number of Basic Cluster attributes in report
afAddrType_t c_DstAddr;


#ifdef ZCL_EZMODE
static void zclSmartMeter_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSmartMeter_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSmartMeter_RegisterEZModeData =
{
  &zclSmartMeter_TaskID,
  SmartMeter_EZMODE_NEXTSTATE_EVT,
  SmartMeter_EZMODE_TIMEOUT_EVT,
  &zclSmartMeterSeqNum,
  zclSmartMeter_EZModeCB
};

// NOT ZCL_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
  //data attribute for SmartMeter defined in zcl_SmartMeter_data_c
  ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,  /*added for SmartMeter*/
  ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,  /*added for SmartMeter*/
  ZCL_CLUSTER_ID_MS_RESET_MEASURMENT,  /*added for SmartMeter*/
  ZCL_CLUSTER_ID_MS_RELAY_MEASUREMENT,  /*added for SmartMeter*/
  ZCL_CLUSTER_ID_MS_START_MEASUREMENT   /*added for SmartMeter*/
  ZCL_CLUSTER_ID_MS_ACK_MEASUREMENT   /*added for SmartMeter*/ 
};
#define ZCLSmartMeter_BINDINGLIST        1
#endif

devStates_t zclSmartMeter_NwkState = DEV_INIT;

uint8 giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;   // display main screen mode first

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t SmartMeter_TestEp =
{
  20,                                 // Test endpoint
  &zclSmartMeter_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSmartMeter_HandleKeys( byte shift, byte keys );
static void zclSmartMeter_BasicResetCB( void );
static void zclSmartMeter_IdentifyCB( zclIdentify_t *pCmd );
static void zclSmartMeter_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSmartMeter_ProcessIdentifyTimeChange( void );

// app display functions
void zclSmartMeter_LcdDisplayUpdate(void);
void zclSmartMeter_LCDDisplayUpdate(void);
void zclSmartMeter_LcdPowerDisplayUpdate(void);
void zclSmartMeter_LcdDisplayMainMode(void);
void zclSmartMeter_LcdDisplayTempMode(void);
void zclSmartMeter_LcdDisplayHelpMode(void);

// app SmartMeter functions
void zclSmartMeter_SendTemp(void);
void zclSmartMeter_SendParam(void);
void zclSmartMeter_SendData(void);
void zclSmartMeter_SendReset(void);
void zclSmartMeter_SendRelay(void);
void zclSmartMeter_SendStart(void);
void zclSmartMeter_SendAdd(void);
void zclSmartMeter_nvReadParam(void);
void zclSmartMeter_nvWriteParam(void);
void zclSmartMeter_dataRegInit(void);
void zclSmartMeter_paramRegInit(void);
void zclSmartMeter_parameterInit(void);
void zclSmartMeter_calPowerInc(void);
uint16 zclSmartMeter_map(uint16 senValue, uint16 MIN_ADC, uint16 MAX_ADC, uint16 MIN_PEAK, uint16 MAX_PEAK);

// app SmartMeter functions
void zclSmartMeter_ADC_init(void);
uint16 analogRead(unit16 senValue);
void InitConsole(void);
uint16 sm_rand(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSmartMeter_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSmartMeter_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSmartMeter_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSmartMeter_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSmartMeter_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSmartMeter_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSmartMeter_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );

#endif // ZCL_DISCOVER


/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]    = " ";
const char sDeviceName[]   = "  Temp Sensor";
const char sSwTempUp[]     = "SW1: Raise Temp";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwTempDown[]   = "SW3: Lower Temp";
const char sSwHelp[]       = "SW5: Help";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSmartMeter_CmdCallbacks =
{
  zclSmartMeter_BasicResetCB,        // Basic Cluster Reset command
  zclSmartMeter_IdentifyCB,          // Identify command
#ifdef ZCL_EZMODE
  NULL,                                           // Identify EZ-Mode Invoke command
  NULL,                                           // Identify Update Commission State command
#endif
  NULL,                                           // Identify Trigger Effect command
  zclSmartMeter_IdentifyQueryRspCB,  // Identify Query Response command
  NULL,             				                      // On/Off cluster command
  NULL,                                           // On/Off cluster enhanced command Off with Effect
  NULL,                                           // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                           // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                           // Level Control Move to Level command
  NULL,                                           // Level Control Move command
  NULL,                                           // Level Control Step command
  NULL,                                           // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                           // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                           // Scene Store Request command
  NULL,                                           // Scene Recall Request command
  NULL,                                           // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                           // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                           // Get Event Log command
  NULL,                                           // Publish Event Log command
#endif
  NULL,                                           // RSSI Location command
  NULL                                            // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclSmartMeter_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSmartMeter_Init( byte task_id )
{
  zclSmartMeter_TaskID = task_id;

  // Set destination address to indirect
  zclSmartMeter_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSmartMeter_DstAddr.endPoint = 0;
  zclSmartMeter_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSmartMeter_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SmartMeter_ENDPOINT, &zclSmartMeter_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( SmartMeter_ENDPOINT, SmartMeter_MAX_ATTRIBUTES, zclSmartMeter_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSmartMeter_TaskID );
  
#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSmartMeter_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSmartMeter_TaskID );

  // Register for a test endpoint
  afRegister( &SmartMeter_TestEp );

#ifdef LCD_SUPPORTED
  // display the device name
  //HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_3 );
   HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_4 );
#endif
  
  // Initialize parameters by reading from flash memory
  zclSmartMeter_nvReadParam( void )
  // Copy parameters read from flash into paramReg  
  zclSmartMeter_parameterInit(void);
 
  
  //Initialize SmartMeter data register
  zclSmartMeter_dataRegInit();
  
  //Initialize SmartMeter parameter register
  zclSmartMeter_paramRegInit();
  
  //Copy SmartMeter parameters from flash
  zclSmartMeter_parameterInit();
  
  //Initialize ADC 
  zclSmartMeter_ADC_init();
  // Get smart meter 64-bit IEEE external address and network address
 APSME_LookupNwkAddr( psm_ADD, psm_nwkADD );
 sm_ADD = *psm_ADD;
 sm_nwkADD = *sm_nwkADD;
 // Set destination address to 64-bit  >>? check 
  zclCoordinator_DstAddr.addrMode = (afAddrMode_t)Addr64Bit;
  zclCoordinator_DstAddr.endPoint = Coordinator_ENDPOINT;
  zclCoordinator_DstAddr.addr.shortAddr = 0;   
 //set designation address
  &zclSmartMeter_DstAddr = pcoordinator_extAddr;
// Set up the serial console to use for displaying messages.  This is
// just for debugging purpose and is not needed for Systick operation.
//
InitConsole();
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSmartMeter_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSmartMeter_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSmartMeter_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSmartMeter_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSmartMeter_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSmartMeter_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( (zclSmartMeter_NwkState == DEV_ZB_COORD) ||
               (zclSmartMeter_NwkState == DEV_ROUTER)   ||
               (zclSmartMeter_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
            zclSmartMeter_LcdDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif // ZCL_EZMODE
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SmartMeter_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSmartMeter_IdentifyTime > 0 )
      zclSmartMeter_IdentifyTime--;
    zclSmartMeter_ProcessIdentifyTimeChange();

    return ( events ^ SmartMeter_IDENTIFY_TIMEOUT_EVT );
  }

#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & SmartMeter_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SmartMeter_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SmartMeter_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SmartMeter_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

  if ( events & SmartMeter_MAIN_SCREEN_EVT )
  {
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
    zclSmartMeter_LcdDisplayUpdate();

    return ( events ^ SmartMeter_MAIN_SCREEN_EVT );
  }

  if ( events & SmartMeter_TEMP_SEND_EVT )
  {
    zclSmartMeter_SendTemp();

    // report current temperature reading every 10 seconds
   osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_TEMP_SEND_EVT, SmartMeter_REPORT_INTERVAL );
    return ( events ^ SmartMeter_TEMP_SEND_EVT );
    
    if ( events & SmartMeter_ADC_SEND_EVT )
  {
   
    zclSmartMeter_PowerCal();
    // get ADC reading every 1 millisecond
    osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, SmartMeter_PowerCal_INTERVAL );
    return ( events ^ SmartMeter_ADC_SEND_EVT );
    
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSmartMeter_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclSmartMeter_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    // increase temperature
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

    if ( zclSmartMeter_MeasuredValue < zclSmartMeter_MaxMeasuredValue )
    {
      zclSmartMeter_MeasuredValue = zclSmartMeter_MeasuredValue + 100;  // considering using whole number value
    }
    else if ( zclSmartMeter_MeasuredValue >= zclSmartMeter_MaxMeasuredValue )
    {
      zclSmartMeter_MeasuredValue = zclSmartMeter_MaxMeasuredValue;
    }

    // Send temperature information
    zclSmartMeter_SendTemp();
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    if ( ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE ) ||
        ( giTemperatureSensorScreenMode == TEMPSENSE_HELPMODE ) )
    {
      giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

#ifdef ZCL_EZMODE
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT };   // only bind on the Temperature Measurement cluster

      // Invoke EZ-Mode
      ezModeData.endpoint = SmartMeter_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( ( zclSmartMeter_NwkState == DEV_ZB_COORD ) ||
           ( zclSmartMeter_NwkState == DEV_ROUTER )   ||
           ( zclSmartMeter_NwkState == DEV_END_DEVICE ) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        // Temperature Sensor is an initiator
      ezModeData.numActiveInClusters = 1;
      ezModeData.pActiveInClusterIDs = clusterIDs;
      ezModeData.numActiveOutClusters = 0;   // active output cluster
      ezModeData.pActiveOutClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );

#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

      // NOT ZCL_EZMODE, Use EndDeviceBind
#else
      {
        zAddrType_t dstAddr;
        dstAddr.addrMode = Addr16Bit;
        dstAddr.addr.shortAddr = 0;   // Coordinator makes the EDB match

        // Initiate an End Device Bind Request, this bind request will
        // only use a cluster list that is important to binding.
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
        ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                              SmartMeter_ENDPOINT,
                              ZCL_HA_PROFILE_ID,
                              0, NULL,
                              ZCLSmartMeter_BINDINGLIST, bindingOutClusters,
                              FALSE );
      }
#endif // ZCL_EZMODE
    }
  }

  if ( keys & HAL_KEY_SW_3 )
  {
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

    // decrease the temperature
    if ( zclSmartMeter_MeasuredValue > zclSmartMeter_MinMeasuredValue )
    {
      zclSmartMeter_MeasuredValue = zclSmartMeter_MeasuredValue - 100;  // considering using whole number value
    }
    else if ( zclSmartMeter_MeasuredValue >= zclSmartMeter_MinMeasuredValue )
    {
      zclSmartMeter_MeasuredValue = zclSmartMeter_MinMeasuredValue;
    }

    // Send temperature information
    zclSmartMeter_SendTemp();
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;

    if ( ( zclSmartMeter_NwkState == DEV_ZB_COORD ) ||
         ( zclSmartMeter_NwkState == DEV_ROUTER ) )
    {
      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;
      NLME_PermitJoiningRequest( gPermitDuration );
    }
  }

  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    zclSmartMeter_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    if ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE )
    {
      giTemperatureSensorScreenMode = TEMPSENSE_HELPMODE;
    }
    else if ( giTemperatureSensorScreenMode == TEMPSENSE_HELPMODE )
    {
#ifdef LCD_SUPPORTED
      HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
      giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
    }
  }

  // update display
  zclSmartMeter_LcdDisplayUpdate();
}

/*********************************************************************
 * @fn      zclSmartMeter_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSmartMeter_LcdDisplayUpdate( void )
{
  // turn on red LED for temperatures >= 24.00C
  if ( zclSmartMeter_MeasuredValue >= 2400 )
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  }
  // turn on green LED for temperatures <= 20.00C
  else if ( zclSmartMeter_MeasuredValue <= 2000 )
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
  }
  // turn on both red and green LEDs for temperatures between 20.00C and 24.00C
  else
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  }

  if ( giTemperatureSensorScreenMode == TEMPSENSE_HELPMODE )
  {
    zclSmartMeter_LcdDisplayHelpMode();
  }
  else
  {
    zclSmartMeter_LCDDisplayMainMode();
  }
}
/*********************************************************************
 * @fn      zclSmartMeter_LCDDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSmartMeter_LCDDisplayUpdate( void )
{

#ifdef LCD_SUPPORTED
  char sDisplayVoltage[16];
  char sDisplayCurrent[16];
  char sDisplayEnergy[16];
  
  int16 VOLTAGE = dataRegister[4];
  int16 CURRENT = dataRegister[5];
  int32 ENERGY = BUILD_UNIT32(dataRegister[8], dataRegister[9]);
  
   osal_memcpy( sDisplayVoltage, "V: ", 11 );
  _ltoa(  VOLTAGE , (void *)(&sDisplayVoltage[11]), 10 ); // only use whole number
  
  osal_memcpy( sDisplayCurrent, "A: ", 11 );
  _ltoa(  CURRENT , (void *)(&sDisplayCurrent[11]), 10 ); // only use whole number  
  
   osal_memcpy( sDisplayEnergy, "kWh: ", 11 );
  _ltoa(  ENERGY , (void *)(&sDisplayEnergy[11]), 10 ); // only use whole number 
  
  HalLcdWriteString( (char *)sDisplayVoltage, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sDisplayCurrent, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sDisplayEnergy, HAL_LCD_LINE_3 );
#endif
  
}

/*********************************************************************
 * @fn      zclSmartMeter_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSmartMeter_LcdDisplayMainMode( void )
{
  char sDisplayTemp[16];

  if ( zclSmartMeter_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclSmartMeter_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclSmartMeter_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( 2 );
  }

  // display current temperature
  osal_memcpy(sDisplayTemp, "TEMP: ", 6);
  _ltoa( ( zclSmartMeter_MeasuredValue / 100 ), (void *)(&sDisplayTemp[6]), 10 );   // convert temperature to whole number
  osal_memcpy( &sDisplayTemp[8], "C", 2 );
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_2 );
#endif

#ifdef LCD_SUPPORTED
  if ( ( zclSmartMeter_NwkState == DEV_ZB_COORD ) ||
       ( zclSmartMeter_NwkState == DEV_ROUTER ) )
  {
    // display help key with permit join status
    if ( gPermitDuration )
    {
      HalLcdWriteString( "SW5: Help      *", HAL_LCD_LINE_3 );
    }
    else
    {
      HalLcdWriteString( "SW5: Help       ", HAL_LCD_LINE_3 );
    }
  }
  else
  {
    // display help key
    HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3 );
  }
#endif
}

/*********************************************************************
 * @fn      zclSmartMeter_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSmartMeter_LcdDisplayHelpMode( void )
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwTempUp, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwTempDown, HAL_LCD_LINE_3 );
#endif
}

/*********************************************************************
 * @fn      zclSmartMeter_SendTemp
 *
 * @brief   Called to send current temperature information to the thermostat
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendTemp( void )
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_TEMPERATURE_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&zclSmartMeter_MeasuredValue);

    zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                       ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
  }

  osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}
/*********************************************************************
 * @fn      zclSmartMeter_SendParam     *
 *
 * @brief   Called to send SmartMeter parameter information to the coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendParm( void )
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  int16 packet[] = {USER_TX_GET, SUCCESS, MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN,
  MAG, MIN_V, MAX_V, MIN_I, MAX_I, ADC_DELAY};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                       ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
  }

  osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}
/*********************************************************************
 * @fn      zclSmartMeter_SendData     *
 *
 * @brief   Called to send SmartMeter data information to the coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendData( void )
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  uint16 POWER1 =(powerVal>>16) & 0xFFFF;
  uint16 POWER0 =powerVal & 0xFFFF;
  uint16 ENERGY1 =(powerVal>>16) & 0xFFFF;
  uint16 ENERGY0 =powerVal & 0xFFFF;
  int16_t packet[] = {USER_TX_GET, SUCCESS, RMS_V, RMS_I, POWER1, POWER0, ENERGY1, ENERGY0, SM_V, SM_I, STATUS};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_DATA_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                       ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
  }

  osal_mem_free( pReportCmd );
}  
#endif  // ZCL_REPORT


#ifdef ZCL_REPORT
/*********************************************************************
 * @fn      zclSmartMeter_SendReset     *
 *
 * @brief   Called to send SmartMeter reset information to the coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendReset( void )
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  uint16 ENERGY_RESET_VALUE_1;
  uint16 ENERGY_RESET_VALUE_0;
  ENERGY_RESET_VALUE_1 = ((ENERGY_RESET_VALUE >> 16) && 0xFFFF);
  ENERGY_RESET_VALUE_0 = ENERGY_RESET_VALUE && 0xFFFF;
  int16_t packet[] = {USER_TX_SET, SUCCESS, flagreset, ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_RESET_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                       ZCL_CLUSTER_ID_MS_RESET_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
  }

  osal_mem_free( pReportCmd );
}  
#endif  // ZCL_REPORT

/*********************************************************************
 * @fn      zclSmartMeter_SendRelay    *
 *
 * @brief   Called to send SmartMeter relay information to the coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendRelay( void )
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;

  int16_t packet[] = {USER_TX_SET, SUCCESS, relay};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_RELAY_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                       ZCL_CLUSTER_ID_MS_RELAY_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
  }

  osal_mem_free( pReportCmd );
}  
#endif  // ZCL_REPORT

/*********************************************************************
 * @fn      zclSmartMeter_SendRestart     *
 *
 * @brief   Called to send SmartMeter relay information to the coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendRestart( void )
{
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;

  int16_t packet[] = {USER_TX_SET, SUCCESS, flaginc};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_RESTART_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                       ZCL_CLUSTER_ID_MS_RESTART_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
  }

  osal_mem_free( pReportCmd );
}  
#endif  // ZCL_REPORT
/*********************************************************************
 * @fn      zclSmartMeter_SendAdd    *
 *
 * @brief   Called to send SmartMeter external IEEE address information 
 * in response to a network discovery broadcast from coordinator
 *
 * @return  none
 */
static void zclSmartMeter_SendAdd( void )
{
#ifdef ZCL_REPORT
  
  zclReportCmd_t *pReportCmd;

 
  uint16 ADD_3 = ((sm_ADD >> 48) && 0xFFFF);
  uint16 ADD_2 = ((sm_ADD >> 32) && 0xFFFF);
  uint16 ADD_1 = ((sm_ADD >> 16) && 0xFFFF);
  uint16 ADD_0 = sm_ADD  && 0xFFFF;
  
 
  int16 packet[] = {USER_TX_GET, COM_ADD, ADD_3, ADD_2, ADD_1, ADD_0};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_ADD_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                       ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
  }

  osal_mem_free( pReportCmd );
}  
#endif  // ZCL_REPORT
/*********************************************************************
 * @fn      zclSmartMeter_nvReadParam
 *
 * @brief   Called to read SmartMeter parameter information from Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_nvReadParam( void )
{
  osalSnvId_t FLASH_PARAM;
  osalSnvLeng_t len;
  len = 20;  //bytes
  pparmRegtemp = &paramRegtemp[0];
  //locate item in flash memory
  osal_nv_item_init (FLASH_PARAM, len, NULL);
  //read from flash memory and load it into paramReg
  osal_nv_read (FLASH_PARAM, 0, len, *pparamReg);
}
 
/*********************************************************************
 * @fn      zclSmartMeter_nvWriteParam
 *
 * @brief   Called to write SmartMeter parameter information into Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_nvWriteParam( void )
{
 osalSnvId_t FLASH_PARAM;
  osalSnvLeng_t len;
  len = 20;

  //locate item in flash
  osal_nv_item_init (FLASH_PARAM, len, NULL);
  //write paramReg to FLASH
  osal_nv_write (FLASH_PARAM, 0, len, *pparamReg);
   
} 


#ifdef ZCL_REPORT
/*********************************************************************
 * @fn      zclSmartMeter_ProcessInReportCmd  *
 *
 * @brief   Process the Incoming Command Report Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static void zclSmartMeter_ProcessInReportCmd( zclIncomingMsg_t *pInMsg )
{
  zclReportCmd_t *pInParamterReport;
 
  pInParameterReport = (zclReportCmd_t *)pInMsg->attrCmd;
  uin16 OOMMAND = (pInParameterReport->attrList(0).attData[0]);
  uint16 OPERATION = (pInParameterReport->attrList(0).attData[1]);
  uint16 ENERGY_RESET_VALUE_1;
  uint16 ENERGY_RESET_VALUE_0;
  if ((COMMAND == USR_RX_GET) && (OPERATION == COM_PARAM) && 
      (pInParameterrReport->attrList[0].attrID) == ATTRID_MS_PARAMETER_MEASURED_VALUE) 
        // send the current parameter value to send over the air to Coordinator
  zclSmartMeter_SendParm( void );
  
 if ((COMMAND == USR_RX_GET) && (OPERATION == COM_DATA) && 
      (pInParameterrReport->attrList[0].attrID) == ATTRID_MS_DATA_MEASURED_VALUE) 
    // send the current data value sent over the air to Coordinator
  zclSmartMeter_SendData( void ); 

 if ((COMMAND == USR_RX_SET) && (OPERATION == COM_PARAM) && 
     (pInParameterrReport->attrList[0].attrID) == ATTRID_MS_PARAMETER_MEASURED_VALUE) {
   // set the current parameter sent over the air from the Coordinator
  MIN_ADC = pInParameterReport->attrList[0].attrData[2];
  MAX_ADC = pInParameterReport->attrList[0].attrData[3];
  SAMPLE_INT = pInParameterReport->attrList[0].attrData[4];
  SAMPLE_WIN = pInParameterReport->attrList[0].attrData[5]
  MAG = pInParameterReport->attrList[0].attrData[6];
  MIN_V = pInParameterReport->attrList[0].attrData[7];
  MAX_V = pInParameterReport->attrList[0].attrData[8];
  MIN_I = pInParameterReport->attrList[0].attrData[9];
  MAX_I = pInParameterReport->attrList[0].attrData[10];
  ADC_DELAY = pInParameterReport->attrList[0].attrData[11];
  parmReg[]={MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN, MAG, MIN_V, MAX_V, MIN_I,
  MAX_I, ADC_DELAY};
  
  //Update flash memory
  zclSmartMeter_nvWriteParam( void );


   // send the current parameter value to send over the air to Coordinator
  zclSmartMeter_SendParm( void );
     }
  if ((COMMAND == USR_RX_SET) && (OPERATION == POWER_RESET) && 
     (pInParameterrReport->attrList[0].attrID) == ATTRID_MS_RESET_MEASURED_VALUE) {
       flagreset = pInParameterReport->attrList[0].attrData[2];
       ENERGY_RESET_VALUE_1 = pInParameterReport->attrList[0].attrData[3];
       ENERGY_RESET_VALUE_0 = pInParameterReport->attrList[0].attrData[4];
       ENERGY_RESET_VALUE = BUILD UNIT32(ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0);
   // send the current flagreset and ENERGY_RESET_VALUE to send over the air to Coordinator
     zclSmartMeter_SendReset(void);
     }
   if ((COMMAND == USR_RX_SET) && (OPERATION == POWER_RELAY) && 
     (pInParameterrReport->attrList[0].attrID) == ATTRID_MS_RELAY_MEASURED_VALUE) {
       relay = pInParameterReport->attrList[0].attrData[2];
       // send the current relay value to send over the air to Coordinator
     zclSmartMeter_SendRelay(void);  
     }
    if ((COMMAND == USR_RX_SET) && (OPERATION == POWER_RESTART) && 
     (pInParameterrReport->attrList[0].attrID) == ATTRID_MS_RESTART_MEASURED_VALUE) {
       flaginc = pInParameterReport->attrList[0].attrData[2];
       // send the current flaginc value to send over the air to Coordinator
     zclSmartMeter_SendRestart(void);                                                   
     }
if ((COMMAND == USR_RX_GET) && (OPERATION == COM_ADD) && 
     (pInParameterrReport->attrList[0].attrID) == ATTRID_MS_ADD_MEASURED_VALUE) { 
    // get the coordinator IEEE address
      uint16 coordinator_Addr_3 = pInParameterReport->attrList[0].attrData[2];  
      uint16 coordinator_Addr_2 = pInParameterReport->attrList[0].attrData[3];
      uint16 coordinator_Addr_1 = pInParameterReport->attrList[0].attrData[4];
      uint16 coordinator_Addr_0 = pInParameterReport->attrList[0].attrData[5];
      coordinator_extAddr= BUILD UNIT64(coordinator_Addr_3, coordinator_Addr_2,
                                     coordinator_Addr_1, coordinator_Addr_0);
   // send smart meter IEEE address to coordinator after a random delay from
   // time of reception of network discovery command
       uint16 SmartMeter_NwkDiscov_INTERVAL = sm_rand()*600000;
     osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_NwkDiscov_SEND_EVT, SmartMeter_NwkDiscov_INTERVAL );
  if (events & Coordinator_NwkDiscov_SEND_EVT)  
     zclSmartMeter_SendAdd( void );  //send smart meter address to coordinator
     }
/*********************************************************************
 * @fn      zclSmartMeter_parameterInit    *
 *
 * @brief   Called to initialize parameters
 *
 * @param   none
 *
 * @return  none
 */
 
 static void zclSmartMeter_parameterInit(void)
{
  
  zclSmartMeter_nvReadParam( ); //read parameter from FLASH
  //load into paramReg
  for (index=0, index < 10; index++) {
    paramReg[index] = BUID UNIT16 (*(pparamReg+1), *pparamReg);
    paramReg = paramReg +2;
  }
  /*paramReg[0] = 0;
  paramReg[1] = 1023;
  paramReg[2]=1000;
  paramReg[3]=60000;
  MAG = paramReg[4]=100;
  paramReg[5]=110;
  paramReg[6]=-110;
  paramReg[7]=2;
  paramReg[8]=-2;
  paramReg[9]=200;*/
  
  MIN_ADC = paramReg[0];
  MAX_ADC = paramReg[1];
  SAMPLE_INT = paramReg[2];
  SAMPLE_WIN = paramReg[3];
  MAG = paramReg[4];
  MAX_V = paramReg[5];
  MIN_V = paramReg[6];
  MAX_I = paramReg[7];
  MIN_I = paramReg[8];
  ADC_DELAY = paramReg[9];
  
  flaginc=0;
  senValueV=0;
  senValueI=0;
  realVol=0;
  realCur=0;
  powerVal=0;
  energyVal=0;
  energyVal_Lcd_display=0;
  RMS_V=0;
  RMS_I=0;
  VrmsTemp=0;
  IrmsTemp=0;
  powerTemp=0;
  l_nSamples=1;
}
 
}
 /*********************************************************************
 * @fn      zclSmartMeter_PowerCal
 *
 * @brief   Called to calculate power
 *
 * @param   none
 *
 * @return  none
 */
 
 static void zclSmartMeter_calPowerInc(void) 
 if(flaginc)
 {//do the power calculation incrementally
   
   senValueV=analogRead(senPinVoltage);   //get ADC voltage value
   senValueI=analogRead(senPinCurrent);   //get ADC current value
   realVol=zclSmartMeter_map(senValueV, MIN_ADC, MAX_ADC, MIN_V*1.414, MAX_V*1.414); //map to real voltage
   realCur=zclSmartMeter_map(senValueI, MIN_ADC, MAX_ADC, MIN_I*1.414, MAX_I*1.414)*MAG;  //map to real current, mag by MAG
   VrmsTemp+=(int32)realVol*(int32)realVol;  //accumulate V^2
   IrmsTemp+=(int32)realCur*(int32)realCur;  //accumulate I^2
   l_nSamples+=1;
 }
 else
 {//calculate the power and energy
   RMS_V=(unit16)sqrt(VrmsTemp/l_nSamples); get RMS voltage
   RMS_I=(uint16)sqrt(IrmsTemp/l_nSamples); get RMS current
   powerVal=(uint32)RMS_V*RMS_I;
   energyVal=(uint32)(powerVal*(SAMPLE_INT+ADC_DELAY)/1000000/3600/1000); //energy magnified by MAG in kWh
   energyVal_Lcd_display =(uint32)(energyVal*3000*1000); //energy magnified by MAG in W.s
   VrmsTemp=0;
   IrmsTemp=0;
   l_nSamples=0;
   
   //update LCD display
   zclSmartMeter_LcdPowerDisplayUpdate(void);
   
 } 
  /*********************************************************************
 * @fn      zclSmartMeter_map
 *
 * @brief   Called to calculate power
 *
 * @param   none
 *
 * @return  none
 */
 
 uint16 zclSmartMeter_map(uint16 senValue, uint16 MIN_ADC, uint16 MAX_ADC, uint16 MIN_PEAK, uint16 MAX_PEAK)
 {
 return (senValue/(MAX_ADC-MIN_ADC)*(MAX_PEAK-MIN_PEAK));
 }
 
  /*********************************************************************
 * @fn      zclSmartMeter_ADC_init
 *
 * @brief   Called to calculate power
 *
 * @param   none
 *
 * @return  none
 */
 
 void zclSmartMeter_ADC_init(void)
 {
//
// Set the clocking to run directly from the external crystal/oscillator.
// (no ext 32k osc, no internal osc)
//
SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);
//?? May be done in board initillization
//
// Set IO clock to the same as system clock
//?? May be done in board initillization
// 
SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
    
//
// Enable RF Core,  requested by ADC
//
SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
//?? May be done in board initillization    
//
// Configure ADC, Internal reference, 256 decimation rate (10bit)
//
SOCADCSingleConfigure(SOCADC_10_BIT, SOCADC_REF_INTERNAL);
 }
 /*****************************************************************************
* 
* @fn      analogRead
*
* @brief   Call ADC
*
* @param   none
*
* @return  uint16
*
* Adopted from GuoXu's code
*/
uint16 analogRead(uint16 senPinValue)
 {

//   
// Trigger single conversion on AIN6 to get VoltageValue
//
SOCADCSingleStart(senPinValue);
//
// Wait until conversion is completed
//
while(!SOCADCEndOfCOnversionGet())
{
}
//
// Get data and shift down based on decimation rate
//
return (SOCADCDataGet() >> SOCADC_10_BIT_RSHIFT);

}        
 /*********************************************************************
 * @fn      zclSmartMeter_dataRegInit
 *
 * @brief   Called to initialize data registers
 *
 * @param   none
 *
 * @return  none
 */
 
 static void zclSmartMeter_dataRegInit(void) 
 {
   uint8 i;
   for (i=0; i<13; i++)
     dataReg(i) = 0;
 }
 /*********************************************************************
 * @fn      zclSmartMeter_paramRegInit
 *
 * @brief   Called to initialize data registers
 *
 * @param   none
 *
 * @return  none
 */
 
 static void zclSmartMeter_paramRegInit(void) 
 {
   uint8 i;
   for (i=0; i<10; i++)
     paramReg(i) = 0;
 }
 /*********************************************************************
 * @fn      sm_rand
 *
 * @brief   generate a random number in the range of 0 to 100
 *
 * @param   none
 *
 * @return  none
 */
 
 uint16 sm_rand(void)
 {
   return rand()%100
 }
/******************************************************************************* 
@fn      zclSmartMeter_Lcd_DisplayUpdate
*
* @brief   Handle LCD display
*
* @param   none
*
* @return  none
*
* Adopted from GuoXu's code
*/

void zclSmartMeter_LcdPowerDisplayUpdate(void)
{
char secondLcdBuf[LCD_BYTES];

        
//
// Clear local LCD buffers
//
lcdBufferClear(0);
lcdBufferClear(secondLcdBuf);

//
// Write default buffer
//
lcdBufferPrintString(0, "SmartMeter", 1, eLcdPage0);
lcdBufferInvertPage(0, 0, 127, eLcdPage0);
//Write RMS_V to default buffer 
lcdBufferPrintStringAligned(0, "Voltage:", eLcdAlignLeft, eLcdPage2);
lcdBufferPrintIntAligned(0, RMS_V, eLcdAlignCenter, eLcdPage2);
lcdBufferPrintStringAligned(0, "V", eLcdAlignRight, eLcdPage2);
    
//Write RMS_I to default buffer
lcdBufferPrintStringAligned(0, "Current:", eLcdAlignLeft,eLcdPage3);
lcdBufferPrintIntAligned(0, RMS_I, eLcdAlignCenter, eLcdPage3);
lcdBufferPrintStringAligned(0, "A", eLcdAlignRight, eLcdPage3);
//Write powerVal to default buffer
lcdBufferPrintStringAligned(0, "Power:", eLcdAlignLeft, eLcdPage4);
lcdBufferPrintIntAligned(0, powerVal, eLcdAlignCenter, eLcdPage4);
lcdBufferPrintStringAligned(0, "W", eLcdAlignRight, eLcdPage4);
//Write EnergyVal to default buffer
lcdBufferPrintStringAligned(0, "Energy:", eLcdAlignLeft, eLcdPage5);
lcdBufferPrintIntAligned(0, energyVal_Lcd_display, eLcdAlignCenter, eLcdPage5);
lcdBufferPrintStringAligned(0, "W.sec", eLcdAlignRight, eLcdPage5);
//Write school name
lcdBufferPrintStringAligned(0, "ITU", eLcdAlignCenter, eLcdPage7);


//
// Send the default buffer to LCD
//
lcdSendBuffer(0);

 //

 //
 // Print the result on UART for debugging purpose
 //
 //UARTprintf(" voltage ADC readout: %d\n", senValueV);
 //UARTprintf(" current ADC readout: %d\n", senValueI);
        
 //UARTprintf("RMS_V: %d", RMS_V);
 //UARTprintf(" V\n");
 //UARTprintf("RMS_I: %d", RMS_I);
 //UARTprintf(" A\n");
 //UARTprintf("powerVal: %d", powerVal);
 //UARTprintf(" W\n");
 //UARTprintf("energyVal_Lcd_display: %d", energyVal_Lcd_display);
 //UARTprintf(" W.sec\n");
}  

/***************************************************************************
* This function sets up UART0 to be used for a console to display information
* for debugging purpose.
*
* adopted from GuoXu's code
*****************************************************************************/
void InitConsole(void)
{
 //
 // Map UART signals to the correct GPIO pins and configure them as
 // hardware controlled.
 //
 IOCPinConfigPeriphOutput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_TXD, 
                             IOC_MUX_OUT_SEL_UART0_TXD);
 GPIOPinTypeUARTOutput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_TXD);
    
 IOCPinConfigPeriphInput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_RXD, 
                            IOC_UARTRXD_UART0);
 GPIOPinTypeUARTInput(EXAMPLE_GPIO_UART_BASE, EXAMPLE_PIN_UART_RXD);
     
 //
 // Initialize the UART (UART0) for console I/O.
 //
 UARTStdioInit(0);
}
/*********************************************************************
 * @fn      zclSmartMeter_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_ProcessIdentifyTimeChange( void )
{
  if ( zclSmartMeter_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSmartMeter_OnOff )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    }
    else
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    }

    osal_stop_timerEx( zclSmartMeter_TaskID, SmartMeter_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSmartMeter_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
 * @fn      zclSmartMeter_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSmartMeter_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSmartMeter_IdentifyTime = pCmd->identifyTime;
  zclSmartMeter_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSmartMeter_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSmartMeter_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp )
{
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSmartMeter_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSmartMeter_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSmartMeter_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSmartMeter_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSmartMeter_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSmartMeter_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSmartMeter_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSmartMeter_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      zclSmartMeter_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSmartMeter_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSmartMeter_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSmartMeter_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSmartMeter_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSmartMeter_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
  {
    osal_mem_free( pInMsg->attrCmd );
  }
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSmartMeter_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSmartMeter_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < readRspCmd->numAttr; i++ )
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSmartMeter_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSmartMeter_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSmartMeter_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSmartMeter_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSmartMeter_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSmartMeter_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSmartMeter_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSmartMeter_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSmartMeter_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSmartMeter_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

#ifdef ZCL_EZMODE

/*********************************************************************
 * @fn      zclSmartMeter_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSmartMeter_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  zclEZMode_ActionData_t data;
  ZDO_MatchDescRsp_t *pMatchDescRsp;

  // Let EZ-Mode know of the Match Descriptor Response
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    pMatchDescRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pMatchDescRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free( pMatchDescRsp );
  }
}

/*********************************************************************
 * @fn      zclSmartMeter_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSmartMeter_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclSmartMeter_IdentifyTime = ( EZMODE_TIME / 1000 );  // convert to seconds
    zclSmartMeter_ProcessIdentifyTimeChange();
  }

  // autoclosing, show what happened (success, cancelled, etc...)
  if( state == EZMODE_STATE_AUTOCLOSE )
  {
#ifdef LCD_SUPPORTED
    pStr = NULL;
    err = pData->sAutoClose.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      pStr = "EZMode: Success";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if ( pStr )
    {
      if ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSmartMeter_IdentifyTime = 0;
    zclSmartMeter_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // "EZDst:1234 EP:34"
      osal_memcpy( szLine, "EZDst:", 6 );
      zclHA_uint16toa( pData->sFinish.nwkaddr, &szLine[6] );
      osal_memcpy( &szLine[10], " EP:", 4 );
      _ltoa( pData->sFinish.ep, (void *)(&szLine[14]), 16 );  // _ltoa NULL terminates
      pStr = szLine;
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giTemperatureSensorScreenMode == TEMPSENSE_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif  // LCD_SUPPORTED

    // show main UI screen 3 seconds after joining network
    osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_MAIN_SCREEN_EVT, 3000 );

    // report current temperature reading 15 seconds after joinging the network
    osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_TEMP_SEND_EVT, SmartMeter_REPORT_INTERVAL );
  }
}
#endif // ZCL_EZMODE

/****************************************************************************
****************************************************************************/


