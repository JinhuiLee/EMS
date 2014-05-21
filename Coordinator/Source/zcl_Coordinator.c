/**************************************************************************************************
  Filename:       zcl_Coordinator.c
  Revised:        $Date: 2013-10-18 17:02:21 -0700 (Fri, 18 Oct 2013) $
  Revision:       $Revision: 35724 $

  Description:    Zigbee Cluster Library - sample device application.

**************************************************************************************************/

/*********************************************************************
  This device will act as a Coordinator.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: 
    - SW2: Invoke EZMode
    - SW3: 
    - SW4: Enable/Disable Permit Join
    - SW5: 
  ----------------------------------------
*********************************************************************/
/*********************************************************************
* Pin Definition
*/
//const uint16 senPinVoltage = SOCADC_AIN6;
//const uint16 senPinCurrent = SOCADC_AIN7;

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

#define COORDINATOR_PIN_UART_RXD            GPIO_PIN_0
#define COORDINATOR_PIN_UART_TXD            GPIO_PIN_1
#define COORDINATOR_GPIO_UART_BASE          GPIO_A_BASE


/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_APP.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_hvac.h"
#include "zcl_ms.h"

#include "zcl_Coordinator.h"

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
 * CONSTANTS
 */
#define USR_RX_GET 0xC0
#define USR_TX_GET 0xC1
#define USR_RT_SET 0xC2
#define USR_TX_SET 0xC3
#define COM_PARAM  0xC4
#define COM_DATA   0xC5
#define COM_ADD    0xC6
#define RESET      0xC7
#define RELAY      0xC8
#define START      0xC9
#define FLASH_PARAM  0x0401   
 
// #define SYS_CTRL_RCGCUART       0x400D2028  /**< UART[1:0] clocks - active mode */
//#define ATTRID_MS_PARAMETER_MEASURED_VALUE  0xA0
//#define ATTRID_MS_DATA_MEASURED_VALUE  0xA1
//#define ATTRID_MS_ADD_MEASURED_VALUE 0xA2
//
/*
#define ATTRID_MS_PARAMETER_MEASURED_VALUE                               0x0012
#define ATTRID_MS_DATA_MEASURED_VALUE                                    0x0013
#define ATTRID_MS_ADD_MEASURED_VALUE                                     0x0014
#define ATTRID_MS_COM_MEASURED_VALUE                                     0x0015
*/
// wait time for all smart meter to respond to network discovery command
#define Coordinator_NwkDiscov_INTERVAL 5000 //in millisecond
// wait time to request the next smart meter to send data
//#define Coordinator_SendData_INTERVAL  1000 //in millisecond

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclCoordinator_TaskID;
uint8 zclCoordinatorSeqNum;
static byte gPermitDuration = 0x00;

uint16 paramReg[10]; 
uint16 SmartMeterparamReg[10]; //parameters send by SmartMeter
uint16 dataReg_Ping[13], dataReg_Pong[13];
uint8 dataRegSel;
uint64 sm_ADD[350]; //smart meter address registers --500
uint16 sm_index;  //index for sm_Add[index]
uint16 index;  //general purpose index
uint16 sm_max; //total number of smart meter
uint8 controlReg[14];
//uint8 SerControl;
uint32 ENERGY_RESET_VALUE;
uint32 SmartMeter_ENERGY_RESET_VALUE; //smart meter response to RESET
uint64 RM_ADD;
uint16 flagreset;
uint16 flagrelay;
uint16 flaginc;
uint16 datain_complete; //read of smart meter data completed
uint16 SmartMeter_flagreset; //smart meter response to RESET
uint16 SmartMeter_relay;  //smart meter response to RELAY
uint16 SmartMeter_flaginc; //smart meter response to RESTART

//
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

/* Define memory mapping*/
uint16 *pparamReg;  //pointer to paramReg
//&paramReg[0] = 0x2000_0040;
//pparamReg = &paramReg[0];

uint16 *pdataReg_Ping;  //pointer to dataReg_Ping
//&dataReg_Ping[0] = 0x2000_0000;
//pdataReg_Ping = &dataReg_Ping[0];

uint16 *pdataReg_Pong;  //pointer to dataReg_Pong
//&dataReg_Pong[0] = 0x2000_0020;
//pdataReg_Pong = &dataReg_Pong[0];

uint8 *pcontrolReg; //pointer to controlReg
//pcontrolReg = 0x2000_060;

//&SerControl = 0x2000_0060;
//uint8 pSerControl;
//pSerControl = &SerControl

//&ENERGY_RESET_VALUE = 0x2000_0062;
//uint8 pENERGY_RESET_VALUE;
//pENERGY_RESET_VALUE = &ENERGY_RESET_VALUE;

//&RM_ADD = 0x2000_0066;
//uint8 pRM_ADD;
//pRM_ADD = &RM_ADD;

uint64 coordinator_extAddr;   //coordinator external IEEE address
uint16 coordinator_nwkAddr;  //coordinator network address

uint8 *pcoordinator_extAddr; //pointer to coordinator external IEEE address
//uint16 coordinator_nwkAddr; //pointer to coordinator network address

//pcoordinator_extAddr = &coordinator_exAddr;  //set pointer to coordinator external IEEE address
//pcoordinator_nwkAddr = &coordinator_nwkAddr;  //set pointer to coordinator network address

uint64 *psm_ADD; //pointer to smartmeter external IEEE addresses in routing table
//&sm_ADD[0] = 0x2000_0100;
//psm_ADD=&sm_ADD[0]; //set pointer to sm_ADD[0]


/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclCoordinator_DstAddr;

#ifdef ZCL_EZMODE
static void zclCoordinator_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclCoordinator_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclCoordinator_RegisterEZModeData =
{
  &zclCoordinator_TaskID,
  Coordinator_EZMODE_NEXTSTATE_EVT,
  Coordinator_EZMODE_TIMEOUT_EVT,
  &zclCoordinatorSeqNum,
  zclCoordinator_EZModeCB
};

// NOT ZCL_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    
};
#define ZCLCoordinator_BINDINGLIST_OUT     1

static cId_t bindingInClusters[] =
{
  ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
   //data attribute for SmartMeter defined in zcl_SmartMeter_data_c 
  ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,  // added for SmartMeter
  ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,  // added for SmartMeter
  ZCL_CLUSTER_ID_MS_ADD_MEASURMENT,  // added for SmartMeter
  ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,  // added for SmartMeter
};
#define ZCLCoordinator_BINDINGLIST_IN      2
#endif

uint8 giThermostatScreenMode = THERMOSTAT_MAINMODE;   // display the main screen mode first

devStates_t zclCoordinator_NwkState = DEV_INIT;

static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t Coordinator_TestEp =
{
  20,                                 // Test endpoint
  &zclCoordinator_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclCoordinator_HandleKeys( byte shift, byte keys );
static void zclCoordinator_BasicResetCB( void );
static void zclCoordinator_IdentifyCB( zclIdentify_t *pCmd );
static void zclCoordinator_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclCoordinator_ProcessIdentifyTimeChange( void );
static void zclCoordinator_ProcessAppMsg( uint8 srcEP, uint8 len, uint8 *msg );
static void zclCoordinator_ProcessFoundationMsg( afAddrType_t *dstAddr, uint16 clusterID,
                                                      zclFrameHdr_t *hdr, zclParseCmd_t *pParseCmd );
static void zclCoordinator_NetDiscov( void );

// app display functions
// void zclCoordinator_LcdDisplayUpdate(void);
void zclCoordinator_LCDDisplayUpdate(void);
// void zclCoordinator_LcdDisplayMainMode(void);
// void zclCoordinator_LcdDisplayHeatMode(void);
// void zclCoordinator_LcdDisplayCoolMode(void);
//void zclCoordinator_LcdDisplayHelpMode(void);

//Coordinator functions
void zclCoordinator_SendData( void );
void zclCoordinator_SendParam( void );
void zclCoordinator_SetParam( void );
void zclCoordinator_nvWriteParam( void );
void zclCoordinator_nvReadParam( void );
void zclCoordinator_SetParm( void );
void zclCoordinator_NetDiscov( void );
void zclCoordinator_SendAck( void );
void zclCoordinator_SendRestart( void );
void zclCoordinator_SendReset(void);
void zclCoordinator_SendRelay(void);
void zclCoordinator_parameterInit(void);
void zclCoordinator_controlRegInit(void);
uint8 zclCoordinator_SmartMeterParamCompare(void);
uint8 zclCoordinator_ProcessInGetParamReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInSetParamReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInGetDataReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInAddReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInResetReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInRestartReportCmd( zclIncomingMsg_t *pInMsg );
void zclCoordinator_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInRelayReportCmd( zclIncomingMsg_t *pInMsg );
void zclCoordinator_dataRegInit (void);
void InitConsole(void);
uint32 BUILD_UINT32_16 (uint16 num1, uint16 numb2);
uint64 BUILD_UINT64_8 (uint8 numb1, uint8 numb2, uint8 numb3, uint8 numb4, uint8 numb5, uint8 numb6, uint8 numb7, uint8 numb8);
uint64 BUILD_UINT64_16 (uint16 numb1, uint16 numb2, uint16 numb3, uint16 numb4);
//uint16 BUILD_UINT16 (uint8 numb1, uint8 numb2);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclCoordinator_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
//static uint8 zclCoordinator_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg ); //removed
#endif
#ifdef ZCL_WRITE
//static uint8 zclCoordinator_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg ); //removed
#endif
#ifdef ZCL_REPORT
static void zclCoordinator_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
#endif  // ZCL_REPORT
//static uint8 zclCoordinator_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg ); //removed

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]     = " ";
const char sDeviceName[]    = "     Coordinator";
// const char sSwHeatSet[]     = "SW1: Set Heating";
const char sSwEZMode[]      = "SW2: EZ-Mode";
// const char sSwCoolSet[]     = "SW3: Set Cooling";
// const char sTempLine2[]     = "SW1:+";
// const char sTempLine3[]     = "SW3:-  SW5:Enter";
// const char sSwHelp[]        = "SW5: Help";
// const char sStoreHeatTemp[] = "HEAT TEMP SAVED";
// const char sStoreCoolTemp[] = "COOL TEMP SAVED";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclCoordinator_CmdCallbacks =
{
  zclCoordinator_BasicResetCB,            // Basic Cluster Reset command
  zclCoordinator_IdentifyCB,              // Identify command
#ifdef ZCL_EZMODE
  NULL,                                        // Identify EZ-Mode Invoke command
  NULL,                                        // Identify Update Commission State command
#endif
  NULL,                                        // Identify Trigger Effect command
  zclCoordinator_IdentifyQueryRspCB,      // Identify Query Response command
  NULL,             				                   // On/Off cluster command
  NULL,                                        // On/Off cluster enhanced command Off with Effect
  NULL,                                        // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                        // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                        // Level Control Move to Level command
  NULL,                                        // Level Control Move command
  NULL,                                        // Level Control Step command
  NULL,                                        // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                        // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                        // Scene Store Request command
  NULL,                                        // Scene Recall Request command
  NULL,                                        // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                        // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                        // Get Event Log command
  NULL,                                        // Publish Event Log command
#endif
  NULL,                                        // RSSI Location command
  NULL                                         // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclCoordinator_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclCoordinator_Init( byte task_id )
{
  
  zclCoordinator_TaskID = task_id;

  // Set destination address to indirect
//  zclCoordinator_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
//  zclCoordinator_DstAddr.endPoint = 0;
//  zclCoordinator_DstAddr.addr.shortAddr = 0;
  
  // This app is part of the Home Automation Profile
  zclHA_Init( &zclCoordinator_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( Coordinator_ENDPOINT, &zclCoordinator_CmdCallbacks );

  // Register the application's attribute list
  zcl_registerAttrList( Coordinator_ENDPOINT, Coordinator_MAX_ATTRIBUTES, zclCoordinator_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclCoordinator_TaskID );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclCoordinator_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclCoordinator_TaskID );

  // Register for a test endpoint
  afRegister( &Coordinator_TestEp );

 ZDO_RegisterForZDOMsg( zclCoordinator_TaskID, End_Device_Bind_rsp );
 ZDO_RegisterForZDOMsg( zclCoordinator_TaskID, Match_Desc_rsp );

#ifdef LCD_SUPPORTED
  // display the device name
  // HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_3 );
     HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_5 );
#endif

     
 //Initialize controlReg in the coordinator
 zclCoordinator_controlRegInit();
//Initialize parameters in the coordinator
  zclCoordinator_parameterInit();
//Initialize SmartMeter data register
  zclCoordinator_dataRegInit();
 //Initialize UART
 // UARTInit();
 
// Set up the serial console to use for displaying messages.  This is
// just for debugging purpose and is not needed for Systick operation.
//

// Get coordinator 64-bit IEEE external address and network address
   pcoordinator_extAddr = NLME_GetExtAddr();
   coordinator_extAddr = *pcoordinator_extAddr;
   coordinator_nwkAddr = NLME_GetShortAddr();
   
//Initialize routing table sm_ADD[index]
  sm_max = 0;
   for (sm_index = 0; sm_index < 100; sm_index++)
    {  
      sm_ADD[sm_index] = 0;
    }
 
// SerControl = SerControl & 0xFB;                
// Read parameters in the SmartMeter
// Set destination address to 64-bit
  zclCoordinator_DstAddr.addrMode = (afAddrMode_t)Addr64Bit;
  zclCoordinator_DstAddr.endPoint = Coordinator_ENDPOINT;
  zclCoordinator_DstAddr.addr.shortAddr = 0;  
 
//Send request for smart meter parameters using round robin method
   for (index=0; index < sm_max; index++)
   {
//&zclCoordinator_DstAddr = &sm_ADD[index];
    
  zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[index])>>56)&0x00000000000000FF); //AF.h; highest
  zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[index])>>48)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[index])>>40)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[index])>>32)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[index])>>24)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[index])>>16)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[index])>>8)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[index])&0x00000000000000FF);  

 InitConsole();
  
    zclCoordinator_SendParam();
// Compare SmartMeter parameters received to that of coordinator
// If they are difference, then reprogram
    if ((zclCoordinator_SmartMeterParamCompare()) == 1)
      {
        zclCoordinator_SetParam();
      }
   }

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
uint16 zclCoordinator_event_loop( uint8 task_id, uint16 events )
{
 
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclCoordinator_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclCoordinator_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif

        case MT_SYS_APP_MSG:
          // Message received from MT
          zclCoordinator_ProcessAppMsg( ((mtSysAppMsg_t *)MSGpkt)->endpoint,
                                          ((mtSysAppMsg_t *)MSGpkt)->appDataLen,
                                          ((mtSysAppMsg_t *)MSGpkt)->appData );
          break;

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclCoordinator_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclCoordinator_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclCoordinator_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( ( zclCoordinator_NwkState == DEV_ZB_COORD ) ||
               ( zclCoordinator_NwkState == DEV_ROUTER )   ||
               ( zclCoordinator_NwkState == DEV_END_DEVICE ) )
          {
#ifndef HOLD_AUTO_START
            // display main mode
            giThermostatScreenMode = THERMOSTAT_MAINMODE;
            // zclCoordinator_LcdDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif  // ZCL_EZMODE
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

  if ( events & Coordinator_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclCoordinator_IdentifyTime > 0 )
    {
      zclCoordinator_IdentifyTime--;
    }
    zclCoordinator_ProcessIdentifyTimeChange();

    return ( events ^ Coordinator_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & Coordinator_MAIN_SCREEN_EVT )
  {
    giThermostatScreenMode = THERMOSTAT_MAINMODE;
    // zclCoordinator_LcdDisplayUpdate();

    return ( events ^ Coordinator_MAIN_SCREEN_EVT );
  }

#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & Coordinator_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ Coordinator_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & Coordinator_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ Coordinator_EZMODE_TIMEOUT_EVT );
  }
#endif // ZCL_EZMODE
 

 //****************************************************************************
 // Design Option 2:                                   *
 // Server period request smart meter to send data
 // Send data if bit 7 of controlReg is set to 1
 // Reset this bit when operation is completed
 //
 //if ( events & Coordinator_DATA_SEND_EVT )
  
  
  if (((controlReg[0] >> 7) & 0x01) == 1)
  {
  events = Coordinator_DATA_SEND_EVT;
 
  //Send request for smart meter data using round robin method
  //Specify which smart meter to request data
  //&zclCoordinator_DstAddr = &sm_ADD[sm_index];
  zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_index])>>56)&0x00000000000000FF); //AF.h; highest
  zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_index])>>48)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_index])>>40)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_index])>>32)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_index])>>24)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_index])>>16)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_index])>>8)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_index])&0x00000000000000FF);  
  //Stop power calculation
    flaginc = 0;
    zclCoordinator_SendRestart();
  //Send request to smart meter to send data 
    datain_complete =0; //reset datain_complete
    zclCoordinator_SendData();   //send request to get data
//While waiting for data send from smart meter, send data from other dataReg
//
      UARTEnable(COORDINATOR_GPIO_UART_BASE ); 
       
      if (dataRegSel == 1) 
      {
        pdataReg_Pong=&dataReg_Pong[0];    //initialize pointer
      for (index=0; index <26; index++)
       {
       UARTCharPut (COORDINATOR_GPIO_UART_BASE, *pdataReg_Pong); //send data
        pdataReg_Pong++;
        }  
      }
     if (dataRegSel == 0) 
      {
        pdataReg_Ping=&dataReg_Ping[0];    //initialize pointer
      for (index=0; index <26; index++)
       {
        UARTCharPut (COORDINATOR_GPIO_UART_BASE, *pdataReg_Ping); //send data
        pdataReg_Ping++;
        }  
      }  
   
      UARTDisable(COORDINATOR_GPIO_UART_BASE ); 
      controlReg[0] = controlReg[0] & 0x7F; //reset bit 7 to 0
   //   waiting for data send from smart meter to complete
        while (!datain_complete)
        {
        }
       dataRegSel =  dataRegSel ^ 0x01 ; //toggle dataRegSel
  //Restart power calculation
     flaginc = 1;
    zclCoordinator_SendRestart();
    sm_index++;
    if (sm_index == sm_max)
        sm_index = 0;
// get data periodically by delay Coordinator_SendData_INTERVAL
// osal_start_timerEx( zclCoordinator_TaskID, Coordinator_Data_SEND_EVT, Coordinator_SendData_INTERVAL );
    return ( events ^ Coordinator_DATA_SEND_EVT );
  }
// 
// Send current parameter to server if parameter read is enabled (bit 0=1)  *
// Reset this parameter to its default state when operation is finished.
//
 
  if ((controlReg[0]  & 0x01) == 1)
  {
      events = Coordinator_PARAM_SEND_EVT;
      pparamReg=&paramReg[0];    //initialize pointer
      UARTEnable(COORDINATOR_GPIO_UART_BASE ); 
 

  for (index=0; index <20; index++)
       {
        UARTCharPut (COORDINATOR_GPIO_UART_BASE, *pparamReg); //send parameters
        pparamReg++;
        }
  UARTDisable(COORDINATOR_GPIO_UART_BASE );
  controlReg[0] = controlReg[0] & 0xFE; //reset bit 0 to 0
  return (events ^ Coordinator_PARAM_SEND_EVT);
 
  } 
  
 //
 // Server write to paramReg if parameter write is enable (bit 1=0)  *
 // This operation will also write into FLASH or coordinator and smart meters
 // Reset this bit to default when write operation is completed
 //
 if (((controlReg[0] >> 1)& 0x01) == 0) 
{
 // UART receive parameter data from server and store to parmReg[]
      events = Coordinator_PARAM_SET_EVT;
      pparamReg=&paramReg[0];    //initialize pointer
      UARTEnable(COORDINATOR_GPIO_UART_BASE );
//
// Check for characters. Wait until a character is placed
// into the receive FIFO.
//
 while(!UARTCharsAvail(COORDINATOR_GPIO_UART_BASE ))
 {}
  for (index=0; index < 20; index++)
  {  
// Get the character(s) in the receive FIFO.
   *pparamReg= UARTCharGetNonBlocking(COORDINATOR_GPIO_UART_BASE);
    pparamReg++;
  }
   UARTDisable (COORDINATOR_GPIO_UART_BASE );
//write parameters into FLASH memory of coordaintor
   zclCoordinator_nvWriteParam(); 
//write parameters to all the smart meters
   for (sm_index=0; sm_index < sm_max; sm_index++) {
   // &zclCoordinator_DstAdfdr = psm_ADD;
  zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_index])>>56)&0x00000000000000FF); //AF.h; highest
  zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_index])>>48)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_index])>>40)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_index])>>32)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_index])>>24)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_index])>>16)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_index])>>8)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_index])&0x00000000000000FF);   
    
    zclCoordinator_SetParam();
//check if parameter set is successful
    if (zclCoordinator_SmartMeterParamCompare()) {
     UARTprintf( "parameter set failed");
      // exit;
    }
    sm_index++;
   }
   controlReg[0]=controlReg[0] | 0x02; //reset bit 1 to 1
   return (events ^ Coordinator_PARAM_SET_EVT);
   
}
// 
// Reset energy calculation if bit 2 is 1   *
// Reset this bit to its default value when operation is finished
//
 if (((controlReg[0] >> 2)& 0x01) == 1) 
{
      events = Coordinator_RESET_SEND_EVT;
      //&zclCoordinator_DstAddr = pRM_ADD;
  RM_ADD = BUILD_UINT64_8(controlReg[13], controlReg[12], controlReg[11], controlReg[10], controlReg[9], controlReg[8], controlReg[7], controlReg[6]);
  zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((RM_ADD)>>56)&0x00000000000000FF); //AF.h; highest
  zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((RM_ADD)>>48)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((RM_ADD)>>40)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((RM_ADD)>>32)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((RM_ADD)>>24)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((RM_ADD)>>16)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((RM_ADD)>>8)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((RM_ADD)&0x00000000000000FF);   
    //  ENERGY_RESET_VALUE = *pENERGY_RESET_VALUE;
  flagreset =(controlReg[0]>>2) & 0xFF;
 ENERGY_RESET_VALUE = BUILD_UINT32(controlReg[5], controlReg[4], controlReg[3], controlReg[2]); 
      zclCoordinator_SendReset();
      if (!(SmartMeter_ENERGY_RESET_VALUE == ENERGY_RESET_VALUE)){
        UARTprintf("RESET failed");
       //exit;
      }
      controlReg[0]=controlReg[0] & 0xFB; // reset bit 2 to 0
      return (events ^ Coordinator_RESET_SEND_EVT);
      
  }
 // 
 // Switch relay off if bit 3 is set to 1           *
 // Reset this bit to its default value when operation is finished
 //
  
 if (((controlReg[0] >> 3)& 0x01) == 1) 
{
      events = Coordinator_RELAY_SEND_EVT;
     // &zclCoordinator_DstAddr = pRM_ADD; 
  
  zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((RM_ADD)>>56)&0x00000000000000FF); //AF.h; highest
  zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((RM_ADD)>>48)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((RM_ADD)>>40)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((RM_ADD)>>32)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((RM_ADD)>>24)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((RM_ADD)>>16)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((RM_ADD)>>8)&0x00000000000000FF);
  zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((RM_ADD)&0x00000000000000FF);  
  flagrelay = (controlReg[0]>>3) & 0xFF;
      zclCoordinator_SendRelay();
      if (!(SmartMeter_relay == flagrelay)){
        UARTprintf("RELAY failed");
       //exit;
      }
      controlReg[0]=controlReg[0] & 0xF7;// reset bit 3 to 0
      return (events ^ Coordinator_RELAY_SEND_EVT);
      
  } 
 // 
 // Perform network discovery if bit 4 is 1           *  
//  Reset this bit to its default value when operation is finished
//
 if (((controlReg[0] >> 4)& 0x01) == 1) 
{
       events = Coordinator_NWKDISCOV_SEND_EVT; 
       zclCoordinator_NetDiscov();
//Wait 5 seconds for all the smart meters to respond to network discovery command
//Each smart meter will send its IEEE address to coordinator during this time
//A routing table sm_Addr[index] will be built. 
//The maximum number of smart meter is sm_max.
  osal_start_timerEx( zclCoordinator_TaskID, Coordinator_NWKDISCOV_SEND_EVT, Coordinator_NwkDiscov_INTERVAL );
  while (~(events & Coordinator_NWKDISCOV_SEND_EVT )){
  }  //wait for Coordinator_NwkDiscov_INTERVAL trigger event Coordinator_NwkDiscov_SEND_EVT
  controlReg[0]=controlReg[0] & 0xEF; //reset bit 4 to 0     
  return (events ^ Coordinator_NWKDISCOV_SEND_EVT);
 
  } 
  //
  // Send routing table when bit 5 is set to 1                 *     
  // Reset this bit to its default value when operation is finished
  //
 if (((controlReg[0] >> 5)& 0x01) == 1) 
{ 
      events = Coordinator_RTABLE_SEND_EVT;
      psm_ADD=&sm_ADD[0];    //initialize pointer
      UARTEnable(COORDINATOR_GPIO_UART_BASE ); 
   

  for (index=0; index <64*sm_max; index++)
       {
        UARTCharPut (COORDINATOR_GPIO_UART_BASE, *psm_ADD); //send parameters
        psm_ADD++;
        }
  UARTDisable(COORDINATOR_GPIO_UART_BASE ); 
  controlReg[0]=controlReg[0] & 0xDF; //reset bit 5 to 0
  return (events ^ Coordinator_RTABLE_SEND_EVT);
  
}

// 
// Send current control register to server if controlReg read is enabled (bit 6=1) *
// Reset this parameter to its default state when operation is finished.
//
      
  if (((controlReg[0] >>6)  & 0x01) == 1)
  {
      events = Coordinator_CONTROL_SEND_EVT;
      pcontrolReg=&controlReg[0];    //initialize pointer
      UARTEnable(COORDINATOR_GPIO_UART_BASE );  


  for (index=0; index <14; index++)
       {
        UARTCharPut (COORDINATOR_GPIO_UART_BASE, *pcontrolReg); //send parameters 
        pcontrolReg++;
        }
  UARTDisable(COORDINATOR_GPIO_UART_BASE );
  controlReg[0] = controlReg[0] & 0xBF; //reset bit 6 to 0
  return (events ^ Coordinator_CONTROL_SEND_EVT);
 
  } 
  //
  // Write to controlReg when parameter write is disabled  *
  // This is the normal write operation from server

 if (((controlReg[0] >> 1)& 0x01) == 1) 
{
 // UART recieve parameter data from server and store to controlReg[]
      events = Coordinator_CONTROL_SET_EVT;
      pcontrolReg=&controlReg[0];    //initialize pointer
      UARTEnable(COORDINATOR_GPIO_UART_BASE ); 
//
// Check for characters. Wait until a character is placed
// into the receive FIFO.
//

while(!UARTCharsAvail(COORDINATOR_GPIO_UART_BASE ))
{}

  for (index=0; index < 14; index++)
  {// Get the character(s) in the receive FIFO.
    *pcontrolReg= UARTCharGetNonBlocking(COORDINATOR_GPIO_UART_BASE); //lhy
    pcontrolReg++;
  }
   UARTDisable (COORDINATOR_GPIO_UART_BASE ); 
   return (events ^ Coordinator_CONTROL_SET_EVT);
}

  // Discard unknown events
  return 0;
 
}

/*********************************************************************
 * @fn      zclCoordinator_HandleKeys
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
static void zclCoordinator_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    /*
    // in heating mode
    if ( giThermostatScreenMode == THERMOSTAT_HEATMODE )
    {
      // increase heating setpoint, considering whole numbers where necessary
      if ( zclCoordinator_OccupiedHeatingSetpoint < zclCoordinator_MaxHeatSetpointLimit )
      {
        zclCoordinator_OccupiedHeatingSetpoint = zclCoordinator_OccupiedHeatingSetpoint + 100;
      }
      else if ( zclCoordinator_OccupiedHeatingSetpoint >= zclCoordinator_MaxHeatSetpointLimit )
      {
        zclCoordinator_OccupiedHeatingSetpoint = zclCoordinator_MaxHeatSetpointLimit;
      }
    }
    // in cooling mode
    else if ( giThermostatScreenMode == THERMOSTAT_COOLMODE )
    {
      // increase cooling setpoint, considering whole numbers where necessary
      if ( zclCoordinator_OccupiedCoolingSetpoint < zclCoordinator_MaxCoolSetpointLimit )
      {
        zclCoordinator_OccupiedCoolingSetpoint = zclCoordinator_OccupiedCoolingSetpoint + 100;
      }
      else if ( zclCoordinator_OccupiedCoolingSetpoint >= zclCoordinator_MaxCoolSetpointLimit )
      {
        zclCoordinator_OccupiedCoolingSetpoint = zclCoordinator_MaxCoolSetpointLimit;
      }
    }
    // set screen mode to heat mode
    else
    {
      giThermostatScreenMode = THERMOSTAT_HEATMODE;
    }
    */
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    if ( ( giThermostatScreenMode == THERMOSTAT_MAINMODE ) ||
         ( giThermostatScreenMode == THERMOSTAT_HELPMODE ) )
    {
      giThermostatScreenMode = THERMOSTAT_MAINMODE;

#ifdef ZCL_EZMODE
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_HVAC_THERMOSTAT };   // only bind on the Thermostat cluster

      // Invoke EZ-Mode
      ezModeData.endpoint = Coordinator_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( ( zclCoordinator_NwkState == DEV_ZB_COORD ) ||
           ( zclCoordinator_NwkState == DEV_ROUTER )   ||
           ( zclCoordinator_NwkState == DEV_END_DEVICE ) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        // Thermostat is an initiator
      ezModeData.numActiveInClusters = 0;
      ezModeData.pActiveInClusterIDs = NULL;
      ezModeData.numActiveOutClusters = 1;   // active output cluster
      ezModeData.pActiveOutClusterIDs = clusterIDs;
      zcl_InvokeEZMode( &ezModeData );

#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

      // NOT ZCL_EZMODE, use EndDeviceBind
#else
      zAddrType_t dstAddr;
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request, this bind request will
      // only use a cluster list that is important to binding.
      dstAddr.addrMode = afAddr16Bit;
      dstAddr.addr.shortAddr = 0;   // Coordinator makes the match
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            Coordinator_ENDPOINT,
                            ZCL_HA_PROFILE_ID,
                            ZCLCoordinator_BINDINGLIST_IN, bindingInClusters,
                            ZCLCoordinator_BINDINGLIST_OUT, bindingOutClusters,
                            TRUE );
#endif // ZCL_EZMODE
    }
  }

  if ( keys & HAL_KEY_SW_3 )
  {
    /*
    if ( giThermostatScreenMode == THERMOSTAT_COOLMODE )
    {
      // decrease cooling setpoint, considering whole numbers where necessary
      if ( zclCoordinator_OccupiedCoolingSetpoint > zclCoordinator_MinCoolSetpointLimit )
      {
        zclCoordinator_OccupiedCoolingSetpoint = zclCoordinator_OccupiedCoolingSetpoint - 100;
      }
      else if ( zclCoordinator_OccupiedCoolingSetpoint <= zclCoordinator_MinCoolSetpointLimit )
      {
        zclCoordinator_OccupiedCoolingSetpoint = zclCoordinator_MinCoolSetpointLimit;
      }
    }
    // in heating mode
    else if ( giThermostatScreenMode == THERMOSTAT_HEATMODE )
    {
      // decrease heating setpoint, considering whole numbers where necessary
      if ( zclCoordinator_OccupiedHeatingSetpoint > zclCoordinator_MinHeatSetpointLimit )
      {
        zclCoordinator_OccupiedHeatingSetpoint = zclCoordinator_OccupiedHeatingSetpoint - 100;
      }
      else if ( zclCoordinator_OccupiedHeatingSetpoint <= zclCoordinator_MinHeatSetpointLimit )
      {
        zclCoordinator_OccupiedHeatingSetpoint = zclCoordinator_MinHeatSetpointLimit;
      }
    }
    // set screen mode to cool mode
    else
    {
      giThermostatScreenMode = THERMOSTAT_COOLMODE;
    }
    */
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    giThermostatScreenMode = THERMOSTAT_MAINMODE;

    if ( ( zclCoordinator_NwkState == DEV_ZB_COORD ) ||
         ( zclCoordinator_NwkState == DEV_ROUTER ) )
    {
      // toggle permit join
      gPermitDuration = gPermitDuration ? 0 : 0xff;
      NLME_PermitJoiningRequest( gPermitDuration  );
    }
  }
/*
  if ( shift && ( keys & HAL_KEY_SW_5 ) )
  {
    zclCoordinator_BasicResetCB();
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    if ( keys & HAL_KEY_SW_5 )
    {
      // in heating or cooling setpoint mode
      if ( giThermostatScreenMode == THERMOSTAT_HEATMODE )
      {
#ifdef LCD_SUPPORTED
        // save current heat setpoint temperature
        HalLcdWriteString( (char *)sStoreHeatTemp, HAL_LCD_LINE_2 );
#endif
        giThermostatScreenMode = THERMOSTAT_MAINMODE;
      }
      else if ( giThermostatScreenMode == THERMOSTAT_COOLMODE )
      {
#ifdef LCD_SUPPORTED
        // save current cool setpoint temperature
        HalLcdWriteString( (char *)sStoreCoolTemp, HAL_LCD_LINE_2 );
#endif
        giThermostatScreenMode = THERMOSTAT_MAINMODE;
      }
      else if ( giThermostatScreenMode == THERMOSTAT_MAINMODE )
      {
        giThermostatScreenMode = THERMOSTAT_HELPMODE;
      }
      else if ( giThermostatScreenMode == THERMOSTAT_HELPMODE )
      {
#ifdef LCD_SUPPORTED
        HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
#endif
        giThermostatScreenMode = THERMOSTAT_MAINMODE;
      }
    }
  }
*/
  // update display
 // zclCoordinator_LcdDisplayUpdate();
  zclCoordinator_LCDDisplayUpdate();
}

/*********************************************************************
 * @fn      zclCoordinator_LCDDisplayUpdate
 *
 * @brief   Called to update the LCD display for test purpose
 *
 * @param   none
 *
 * @return  none
 */
void zclCoordinator_LCDDisplayUpdate( void )
{

#ifdef LCD_SUPPORTED
  char sDisplayVoltage[16];
  char sDisplayCurrent[16];
  char sDisplayEnergy[16];
  
  uint16 VOLTAGE = dataReg_Ping[4];
  uint16 CURRENT = dataReg_Ping[5];
  uint32 ENERGY = BUILD_UINT32_16(dataReg_Ping[8], dataReg_Ping[9]);
  
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
 * @fn      zclCoordinator_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
/*
void zclCoordinator_LcdDisplayUpdate( void )
{
  // use LEDs to show heating or cooling cycles based off local temperature
  if ( zclCoordinator_LocalTemperature != NULL )
  {
    if ( zclCoordinator_LocalTemperature <= zclCoordinator_OccupiedHeatingSetpoint )
    {
      // turn on heating
      zclCoordinator_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_HEAT;
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
    }
    else if ( zclCoordinator_LocalTemperature >= zclCoordinator_OccupiedCoolingSetpoint )
    {
      // turn on cooling
      zclCoordinator_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_COOL;
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
    }
    else
    {
      // turn off heating/cooling
      zclCoordinator_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_OFF;
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
    }
  }

  if ( giThermostatScreenMode == THERMOSTAT_HEATMODE )
  {
    //zclCoordinator_LcdDisplayHeatMode();
  }
  else if ( giThermostatScreenMode == THERMOSTAT_COOLMODE )
  {
   // zclCoordinator_LcdDisplayCoolMode();
  }
  else if ( giThermostatScreenMode == THERMOSTAT_HELPMODE )
  {
    //zclCoordinator_LcdDisplayHelpMode();
  }
  else
  {
   // zclCoordinator_LcdDisplayMainMode();
  }
}
*/
/*********************************************************************
 * @fn      zclCoordinator_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
/*
void zclCoordinator_LcdDisplayMainMode( void )
{
  char sDisplayTemp[16];

  if ( zclCoordinator_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclCoordinator_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclCoordinator_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( 2 );
  }

  osal_memcpy( sDisplayTemp, "TEMP: ", 6 );

  // if local temperature has not been set, make note on display
  if ( zclCoordinator_LocalTemperature == NULL )
  {
    osal_memcpy( &sDisplayTemp[6], "N/A", 4 );
  }
  else
  {
    _ltoa( ( zclCoordinator_LocalTemperature / 100 ), (void *)(&sDisplayTemp[6]), 10 ); // only use whole number
    osal_memcpy( &sDisplayTemp[8], "C", 2 );
  }
#ifdef LCD_SUPPORTED
  // display current temperature
  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_2 );
#endif

#ifdef LCD_SUPPORTED
  if ( ( zclCoordinator_NwkState == DEV_ZB_COORD ) ||
       ( zclCoordinator_NwkState == DEV_ROUTER ) )
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
    HalLcdWriteString( (char *)sSwHelp, HAL_LCD_LINE_3);
  }
#endif
}
*/
/*********************************************************************
 * @fn      zclCoordinator_LcdDisplayHelpMode
 *
 * @brief   Called to display the SW options on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
/*
void zclCoordinator_LcdDisplayHelpMode( void )
{
#ifdef LCD_SUPPORTED
  HalLcdWriteString( (char *)sSwHeatSet, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sSwEZMode, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sSwCoolSet, HAL_LCD_LINE_3 );
#endif
}
*/
/*********************************************************************
 * @fn      zclCoordinator_LcdDisplayHeatMode
 *
 * @brief   Called to display the heating setpoint temperature on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
/*
void zclCoordinator_LcdDisplayHeatMode( void )
{
#ifdef LCD_SUPPORTED
  char sDisplayTemp[16];

  osal_memcpy( sDisplayTemp, "HEAT TEMP: ", 11 );
  _ltoa( ( zclCoordinator_OccupiedHeatingSetpoint / 100 ), (void *)(&sDisplayTemp[11]), 10 ); // only use whole number
  osal_memcpy( &sDisplayTemp[13], "C", 2 );

  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sTempLine2, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sTempLine3, HAL_LCD_LINE_3 );
#endif
}
*/
/*********************************************************************
 * @fn      zclCoordinator_LcdDisplayCoolMode
 *
 * @brief   Called to display the cooling setpoint temperature on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
/*
void zclCoordinator_LcdDisplayCoolMode( void )
{
#ifdef LCD_SUPPORTED
  char sDisplayTemp[16];

  osal_memcpy(sDisplayTemp, "COOL TEMP: ", 11);
  _ltoa( ( zclCoordinator_OccupiedCoolingSetpoint / 100 ), (void *)(&sDisplayTemp[11]), 10 ); // only use whole number
  osal_memcpy( &sDisplayTemp[13], "C", 2 );

  HalLcdWriteString( (char *)sDisplayTemp, HAL_LCD_LINE_1 );
  HalLcdWriteString( (char *)sTempLine2, HAL_LCD_LINE_2 );
  HalLcdWriteString( (char *)sTempLine3, HAL_LCD_LINE_3 );
#endif
}
*/
/*********************************************************************
 * @fn      zclCoordinator_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_ProcessIdentifyTimeChange( void )
{
  if ( zclCoordinator_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclCoordinator_TaskID, Coordinator_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclCoordinator_OnOff )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    }
    else
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    }

    osal_stop_timerEx( zclCoordinator_TaskID, Coordinator_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclCoordinator_ProcessAppMsg
 *
 * @brief   Process DoorLock messages
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   len - number of bytes
 * @param   msg - pointer to message
 *          0 - lo byte destination address
 *          1 - hi byte destination address
 *          2 - destination endpoint
 *          3 - lo byte cluster ID
 *          4 - hi byte cluster ID
 *          5 - message length
 *          6 - destination address mode (first byte of data)
 *          7 - zcl command frame
 *
 * @return  none
 */
static void zclCoordinator_ProcessAppMsg( uint8 srcEP, uint8 len, uint8 *msg )
{
  afAddrType_t dstAddr;
  uint16 clusterID;
  zclFrameHdr_t hdr;
  uint8 *pData;
  uint8 dataLen;

  dstAddr.addr.shortAddr = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  dstAddr.endPoint = *msg++;
  clusterID = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  dataLen = *msg++; // Length of message (Z-Tool can support up to 255 octets)
  dstAddr.addrMode = (afAddrMode_t)(*msg++);
  dataLen--; // Length of ZCL frame

  // Begining of ZCL frame
  pData = zclParseHdr( &hdr, msg );
  dataLen -= (uint8)( pData - msg );

  // Is this a foundation type message?
  if ( zcl_ProfileCmd( hdr.fc.type ) )
  {
    if ( hdr.fc.manuSpecific )
    {
      // We don't support any manufacturer specific command -- just forward it.
      zcl_SendCommand( srcEP, &dstAddr, clusterID, hdr.commandID, FALSE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr.fc.disableDefaultRsp, hdr.manuCode, hdr.transSeqNum, dataLen, pData );
    }
    else
    {
      zclParseCmd_t cmd;

      cmd.endpoint = srcEP;
      cmd.dataLen = dataLen;
      cmd.pData = pData;

      zclCoordinator_ProcessFoundationMsg( &dstAddr, clusterID, &hdr, &cmd );
    }
  }
  else
  {
    // Nope, must be specific to the cluster ID
    if ( hdr.fc.manuSpecific )
    {
      // We don't support any manufacturer specific command -- just forward it.
      zcl_SendCommand( srcEP, &dstAddr, clusterID, hdr.commandID, TRUE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr.fc.disableDefaultRsp, hdr.manuCode, hdr.transSeqNum, dataLen, pData );
    }
  }
}

/*********************************************************************
 * @fn      zclCoordinator_ProcessFoundationMsg
 *
 * @brief   Process Foundation message
 *
 * @param   srcEP - Sending Apps endpoint
 * @param   dstAddr - where to send the request
 * @param   clusterID - real cluster ID
 * @param   hdr - pointer to the message header
 * @param   len - length of the received message
 * @param   data - received message
 *
 * @return  none
 */
static void zclCoordinator_ProcessFoundationMsg( afAddrType_t *dstAddr, uint16 clusterID,
                                                zclFrameHdr_t *hdr, zclParseCmd_t *pParseCmd )
{
  
#if defined(ZCL_READ) || defined(ZCL_WRITE) || defined(ZCL_REPORT) || defined(ZCL_DISCOVER)
  void *cmd;
#endif

  switch ( hdr->commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ:
      cmd = zclParseInReadCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendRead( Coordinator_ENDPOINT, dstAddr, clusterID, (zclReadCmd_t *)cmd,
                      ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_READ

#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWrite( Coordinator_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                       ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_WRITE_UNDIVIDED:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWriteUndivided( Coordinator_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                                ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_WRITE_NO_RSP:
      cmd = zclParseInWriteCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendWriteNoRsp( Coordinator_ENDPOINT, dstAddr, clusterID, (zclWriteCmd_t *)cmd,
                            ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_WRITE

#ifdef ZCL_REPORT
    case ZCL_CMD_CONFIG_REPORT:
      cmd = zclParseInConfigReportCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendConfigReportCmd( Coordinator_ENDPOINT, dstAddr,  clusterID, (zclCfgReportCmd_t *)cmd,
                                 ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      cmd = zclParseInReadReportCfgCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendReadReportCfgCmd( Coordinator_ENDPOINT, dstAddr, clusterID, (zclReadReportCfgCmd_t *)cmd,
                                  ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;

    case ZCL_CMD_REPORT:
      cmd = zclParseInReportCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendReportCmd( Coordinator_ENDPOINT, dstAddr, clusterID, (zclReportCmd_t *)cmd,
                           ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_REPORT
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_ATTRS:
      cmd = zclParseInDiscAttrsCmd( pParseCmd );
      if ( cmd )
      {
        zcl_SendDiscoverAttrsCmd( Coordinator_ENDPOINT, dstAddr, clusterID, (zclDiscoverAttrsCmd_t *)cmd,
                                  ZCL_FRAME_CLIENT_SERVER_DIR, hdr->fc.disableDefaultRsp, hdr->transSeqNum );
        osal_mem_free( cmd );
      }
      break;
#endif // ZCL_DISCOVER

    default:
      // Unsupported command -- just forward it.
      zcl_SendCommand( pParseCmd->endpoint, dstAddr, clusterID, hdr->commandID, FALSE, ZCL_FRAME_CLIENT_SERVER_DIR,
                       hdr->fc.disableDefaultRsp, 0, hdr->transSeqNum, pParseCmd->dataLen, pParseCmd->pData );
      break;
  }
  
}

/*********************************************************************
 * @fn      zclCoordinator_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_BasicResetCB( void )
{
  // Put device back to factory default settings
  zgWriteStartupOptions( ZG_STARTUP_SET, 3 );   // bit set both default configuration and default network

  // restart device
  MT_SysCommandProcessing( aProcessCmd );
}

/*********************************************************************
 * @fn      zclCoordinator_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclCoordinator_IdentifyCB( zclIdentify_t *pCmd )
{
  zclCoordinator_IdentifyTime = pCmd->identifyTime;
  zclCoordinator_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclCoordinator_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclCoordinator_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
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
 * @fn      zclCoordinator_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclCoordinator_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
    //  zclCoordinator_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
    //  zclCoordinator_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    case ZCL_CMD_CONFIG_REPORT:
      //zclCoordinator_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclCoordinator_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclCoordinator_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclCoordinator_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      zclCoordinator_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      //zclCoordinator_ProcessInDefaultRspCmd( pInMsg );
      break;

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
 * @fn      zclCoordinator_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
/*
static uint8 zclCoordinator_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
*/
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclCoordinator_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
/*
static uint8 zclCoordinator_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}*/
#endif // ZCL_WRITE

#ifdef ZCL_REPORT
/*********************************************************************
 * @fn      zclCoordinator_ProcessInReportCmd  <<<<<<<<<<<<<<<<<<<<<<<
 *
 * @brief   Process the "Profile" Report Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static void zclCoordinator_ProcessInReportCmd( zclIncomingMsg_t *pInMsg )
{
  /*
  zclReportCmd_t *pInTempSensorReport;
  zclReportCmd_t *pOutDemandReport;
  uint8 outDemandBuffer[sizeof( zclReportCmd_t ) + ( 2 * sizeof( zclReport_t ) )];
  bool send = TRUE;

  pInTempSensorReport = (zclReportCmd_t *)pInMsg->attrCmd;

  if ( pInTempSensorReport->attrList[0].attrID != ATTRID_MS_TEMPERATURE_MEASURED_VALUE )
  {
    return;
  }

  pOutDemandReport = (zclReportCmd_t *)outDemandBuffer;

  // store the current temperature value sent over the air from temperature sensor
  zclCoordinator_LocalTemperature = BUILD_UINT16(pInTempSensorReport->attrList[0].attrData[0], pInTempSensorReport->attrList[0].attrData[1]);

  // update display with current temperature information, set current mode
  // zclCoordinator_LcdDisplayUpdate();
  zclCoordinator_LCDDisplayUpdate(); 
  
  pOutDemandReport->numAttr = 2;
  pOutDemandReport->attrList[0].attrID = ATTRID_HVAC_THERMOSTAT_PI_HEATING_DEMAND;
  pOutDemandReport->attrList[0].dataType = ZCL_DATATYPE_UINT8;
  pOutDemandReport->attrList[1].attrID = ATTRID_HVAC_THERMOSTAT_PI_COOLING_DEMAND;
  pOutDemandReport->attrList[1].dataType = ZCL_DATATYPE_UINT8;
  */
/*
  // send heating demand to heating/cooling unit
  if ( zclCoordinator_SystemMode == HVAC_THERMOSTAT_SYSTEM_MODE_HEAT )
  {
    zclCoordinator_HeatingDemand = 100; // 100%
    zclCoordinator_CoolingDemand = 0;  // off

    pOutDemandReport->attrList[0].attrData = &zclCoordinator_HeatingDemand;
    pOutDemandReport->attrList[1].attrData = &zclCoordinator_CoolingDemand;
  }
  // send cooling demand to heating/cooling unit
  else if ( zclCoordinator_SystemMode == HVAC_THERMOSTAT_SYSTEM_MODE_COOL )
  {
    zclCoordinator_HeatingDemand = 0;  // off
    zclCoordinator_CoolingDemand = 100;  // 100%

    pOutDemandReport->attrList[0].attrData = &zclCoordinator_HeatingDemand;
    pOutDemandReport->attrList[1].attrData = &zclCoordinator_CoolingDemand;
  }
  // turn heating/cooling unit off
  else if ( zclCoordinator_SystemMode == HVAC_THERMOSTAT_SYSTEM_MODE_OFF )
  {
    zclCoordinator_HeatingDemand = 0;  // off
    zclCoordinator_CoolingDemand = 0;  // off

    pOutDemandReport->attrList[0].attrData = &zclCoordinator_HeatingDemand;
    pOutDemandReport->attrList[1].attrData = &zclCoordinator_CoolingDemand;
  }
  else
  {
    send = FALSE;
  }
*/
 /* 
  if ( send )
  {
    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                      ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
                      pOutDemandReport, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }
  */
 //  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 // Process incomming parameter report from smart meter in response to GET parameter command *
 //
  zclReportCmd_t *pInParameterReport;
  
  pInParameterReport = (zclReportCmd_t *)pInMsg->attrCmd;
  uint16 OPERATION = pInParameterReport->attrList[0].attrData[0];
  uint16 RESULT = pInParameterReport->attrList[0].attrData[1];
 
  if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) && 
      (pInParameterReport->attrList[0].attrID == ATTRID_MS_PARAMETER_MEASURED_VALUE)) {
 
    // store the current parameter value sent over the air from smart meter *
  SmartMeterparamReg[0] = pInParameterReport->attrList[0].attrData[2];
  SmartMeterparamReg[1] = pInParameterReport->attrList[0].attrData[3];
  SmartMeterparamReg[2] = pInParameterReport->attrList[0].attrData[4];
  SmartMeterparamReg[3] = pInParameterReport->attrList[0].attrData[5];
  SmartMeterparamReg[4] = pInParameterReport->attrList[0].attrData[6];
  SmartMeterparamReg[5] = pInParameterReport->attrList[0].attrData[7];
  SmartMeterparamReg[6] = pInParameterReport->attrList[0].attrData[8];
  SmartMeterparamReg[7] = pInParameterReport->attrList[0].attrData[9];
  SmartMeterparamReg[8] = pInParameterReport->attrList[0].attrData[10];
  SmartMeterparamReg[9] = pInParameterReport->attrList[0].attrData[11];
     }  
  
  
  // Process incomming parameter report from smart meter in response to SET parameter command *
   if ((OPERATION == USR_TX_SET) && (RESULT == SUCCESS) && 
      (pInParameterReport->attrList[0].attrID == ATTRID_MS_PARAMETER_MEASURED_VALUE)) {
 
    // store the current parameter value sent over the air from smart meter
       
  SmartMeterparamReg[0] = pInParameterReport->attrList[0].attrData[2];
  SmartMeterparamReg[1] = pInParameterReport->attrList[0].attrData[3];
  SmartMeterparamReg[2] = pInParameterReport->attrList[0].attrData[4];
  SmartMeterparamReg[3] = pInParameterReport->attrList[0].attrData[5];
  SmartMeterparamReg[4] = pInParameterReport->attrList[0].attrData[6];
  SmartMeterparamReg[5] = pInParameterReport->attrList[0].attrData[7];
  SmartMeterparamReg[6] = pInParameterReport->attrList[0].attrData[8];
  SmartMeterparamReg[7] = pInParameterReport->attrList[0].attrData[9];
  SmartMeterparamReg[8] = pInParameterReport->attrList[0].attrData[10];
  SmartMeterparamReg[9] = pInParameterReport->attrList[0].attrData[11];
      }
  //Process incomming data report from smart meter in response to GET date command *
   if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) && 
      (pInParameterReport->attrList[0].attrID == ATTRID_MS_DATA_MEASURED_VALUE)) {

    // store the current data value sent over the air from smart meter
    if (dataRegSel == 0) {
  dataReg_Pong[0] = ((sm_ADD[sm_index]>>48) & 0xFFFF);
  dataReg_Pong[1] = ((sm_ADD[sm_index]>>32) & 0xFFFF);
  dataReg_Pong[2] = ((sm_ADD[sm_index]>>16) & 0xFFFF);
  dataReg_Pong[3] = sm_ADD[sm_index] & 0xFFFF;
  dataReg_Pong[4] = pInParameterReport->attrList[0].attrData[2];
  dataReg_Pong[5] = pInParameterReport->attrList[0].attrData[3];
  dataReg_Pong[6] = pInParameterReport->attrList[0].attrData[4];
  dataReg_Pong[7] = pInParameterReport->attrList[0].attrData[5];
  dataReg_Pong[8] = pInParameterReport->attrList[0].attrData[6];
  dataReg_Pong[9] = pInParameterReport->attrList[0].attrData[7];
  dataReg_Pong[10] = pInParameterReport->attrList[0].attrData[8];
  dataReg_Pong[11] = pInParameterReport->attrList[0].attrData[9];
  dataReg_Pong[12] = pInParameterReport->attrList[0].attrData[10]; 
    } 
 else  if (dataRegSel ==1) {
  dataReg_Ping[0] = ((sm_ADD[sm_index]>>48) & 0xFFFF);
  dataReg_Ping[1] = ((sm_ADD[sm_index]>>32) & 0xFFFF);
  dataReg_Ping[2] = ((sm_ADD[sm_index]>>16) & 0xFFFF);
  dataReg_Ping[3] = sm_ADD[sm_index] & 0xFFFF;
  dataReg_Ping[4] = pInParameterReport->attrList[0].attrData[2];
  dataReg_Ping[5] = pInParameterReport->attrList[0].attrData[3];
  dataReg_Ping[6] = pInParameterReport->attrList[0].attrData[4];
  dataReg_Ping[7] = pInParameterReport->attrList[0].attrData[5];
  dataReg_Ping[8] = pInParameterReport->attrList[0].attrData[6];
  dataReg_Ping[9] = pInParameterReport->attrList[0].attrData[7];
  dataReg_Ping[10] = pInParameterReport->attrList[0].attrData[8];
  dataReg_Ping[11] = pInParameterReport->attrList[0].attrData[9];
  dataReg_Ping[12] = pInParameterReport->attrList[0].attrData[10]; 
    }
    datain_complete=1;  //set flag to indicate all data have been received
 }
  // Process incomming address report from smart meter in response to route discovery command *
   if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) &&
         ((pInParameterReport->attrList[0].attrID)== ATTRID_MS_ADD_MEASURED_VALUE )) {
  sm_ADD[sm_index] = BUILD_UINT64_16(pInParameterReport->attrList[0].attrData[2], 
                             pInParameterReport->attrList[0].attrData[3], 
                             pInParameterReport->attrList[0].attrData[4], 
                             pInParameterReport->attrList[0].attrData[5]); 
  //Ack smart meter with sm_Add[index
  zclCoordinator_SendAck();
  sm_index++; //increment index
  sm_max++; //total number of smart meter detected
  
         }
  // Process incomming energy reset report from smart meter in response to RESET command *
   if ((OPERATION == RESET) && (RESULT == SUCCESS) && 
      (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE)) {
  SmartMeter_flagreset = pInParameterReport->attrList[0].attrData[2]; 
  uint16 energy_reset_value1 =  pInParameterReport->attrList[0].attrData[3];
  uint16 energy_reset_value0 =  pInParameterReport->attrList[0].attrData[4];
  SmartMeter_ENERGY_RESET_VALUE = BUILD_UINT32_16(energy_reset_value1, energy_reset_value0);
      }
   // Process incomming relay report from smart meter in response to RELAY command *
   if ((OPERATION == RELAY) && (RESULT == SUCCESS)  &&
      (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE)) {      
    SmartMeter_relay = pInParameterReport->attrList[0].attrData[3]; 
      }
     // Process incomming restart report from smart meter in response to RESTART command *
   if ((OPERATION == START) && (RESULT == SUCCESS)  &&
      (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE)) {      
    SmartMeter_flaginc = pInParameterReport->attrList[0].attrData[2]; 
      } 
}
#endif  // ZCL_REPORT
/*********************************************************************
 * @fn      zclCoordinator_SendParam *
 *
 * @brief   Called to request Smart Meter to send parameters to the Coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendParam( void )
{
  
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  int16 packet[] = {USR_RX_GET, COM_PARAM};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                       ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
 #endif  // ZCL_REPORT 
  
}


/*********************************************************************
 * @fn      zclCoordinator_SetParam *
 *
 * @brief   Called to set SmartMeter parameter information from the Coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SetParam( void )
{
  
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  int16 packet[] = {USR_RT_SET, COM_PARAM, MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN, 
  MAG, MIN_V, MAX_V, MIN_I, MAX_I, ADC_DELAY};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                       ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
  #endif  // ZCL_REPORT
  
}


/*********************************************************************
 * @fn      zclCoordinator_SendData *
 *
 * @brief   Called to request SmartMeter to send power data information to the Coodinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendData( void )
{
  
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  int16 packet[] = {USR_RX_GET, COM_DATA};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_DATA_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                       ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
  #endif  // ZCL_REPORT
  
}

/*********************************************************************
 * @fn      zclCoordinator_SendReset *
 *
 * @brief   Called to send command to smart meter to reset energy calculation to specified value
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendReset( void )
{
  
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;
  int16 energy_reset_value_1 = ((ENERGY_RESET_VALUE>>16) & 0xFF);
  int16 energy_reset_value_0 = ENERGY_RESET_VALUE & 0xFF;
  int16 packet[] = {RESET, flagreset, energy_reset_value_1, energy_reset_value_0 };  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                       ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
  #endif  // ZCL_REPORT
  
}


/*********************************************************************
 * @fn      zclCoordinator_SendRelay *
 *
 * @brief   Called to send command to set relay in smart meter
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendRelay( void )
{
  
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;

  int16 packet[] = {RELAY, flagrelay };  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                       ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
  #endif  // ZCL_REPORT
  
}


/*********************************************************************
 * @fn      zclCoordinator_SendRestart *
 *
 * @brief   Called to command to smart meter to restart power calculation
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendRestart( void )
{
  
#ifdef ZCL_REPORT
  zclReportCmd_t *pReportCmd;

  int16 packet[] = {START, flaginc };  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                       ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
  #endif  // ZCL_REPORT
  
}
/*********************************************************************
 * @fn      zclCoordinator_NetDiscov *
 *
 * @brief   Called to discover smart meter IEEE address
 *         data pack contains coordinator IEEE address
 *         using Broadcast
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_NetDiscov( void )
{
  
  #ifdef ZCL_REPORT
  afAddrType_t Bc_DstAddr;
  zclReportCmd_t *pReportCmd;
  uint16 ADD_3 = ((coordinator_extAddr) >>48 & 0xFFFF);
  uint16 ADD_2 = ((coordinator_extAddr) >>32 & 0xFFFF);
  uint16 ADD_1 = ((coordinator_extAddr) >>16 & 0xFFFF);
  uint16 ADD_0 = *pcoordinator_extAddr & 0xFFFF;
  uint16 packet[] = {USR_RX_GET, COM_ADD, ADD_3, ADD_2, ADD_1, ADD_0};  
  
  Bc_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast; 
  Bc_DstAddr.endPoint = Coordinator_ENDPOINT; 
  Bc_DstAddr.addr.shortAddr = 0xFFFF; 
  
  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
    
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_ADD_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);
    zcl_SendReportCmd( Coordinator_ENDPOINT, &Bc_DstAddr,
                       ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
  #endif  // ZCL_REPORT
  
} 


/*********************************************************************
 * @fn      zclCoordinator_SendAck  *
 *
 * @brief   Called to send Ack to smart meter to confirm association
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendAck( void )
{
  
  #ifdef ZCL_REPORT
  uint16 sm_ADD3 = (sm_ADD[sm_index]>>48) & 0xFF;
  uint16 sm_ADD2 = (sm_ADD[sm_index]>>32) & 0xFF;
  uint16 sm_ADD1 = (sm_ADD[sm_index]>>16) & 0xFF;
  uint16 sm_ADD0 = sm_ADD[sm_index] & 0xFF;
  zclReportCmd_t *pReportCmd;
  int16 packet[] = {USR_RX_GET, SUCCESS, sm_ADD3, sm_ADD2, sm_ADD1, sm_ADD0};  

  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_MS_ADD_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&packet[0]);

    zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                       ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                       pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
  }

  osal_mem_free( pReportCmd );
  #endif  // ZCL_REPORT
  
}


 /*********************************************************************
 * @fn      zclCoordinator_parameterInit     *
 *
 * @brief   Called to initialize parameters
 *
 * @param   none
 *
 * @return  none
 */
 static void zclCoordinator_parameterInit(void)
{
  zclCoordinator_nvReadParam( ); //read parameter from FLASH
  //load into paramReg
  //for (index=0; index < 10; index++) {
  //  paramReg[index] = BUILD_UINT16 (*(pparamReg+1), *pparamReg);
  //  paramReg = paramReg +2;
  //}
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
  
  MIN_ADC=paramReg[0];
  MAX_ADC=paramReg[1];
  SAMPLE_INT=paramReg[2];
  SAMPLE_WIN=paramReg[3];
  MAG = paramReg[4];
  MAX_V = paramReg[5];
  MIN_V = paramReg[6];
  MAX_I = paramReg[7];
  MIN_I = paramReg[8];
  ADC_DELAY = paramReg[9];
  index =0;
  sm_index=0;
  dataRegSel=0;
  sm_max=0;
}

/*********************************************************************
 * @fn      zclCoordinator_controlRegInit  
 *
 * @brief   Called to initialize parameters
 *
 * @param   none
 *
 * @return  none
 */
 
 static void zclCoordinator_controlRegInit(void)
{
  
  //ServControl = 0b00001010;
  ENERGY_RESET_VALUE = 0x00000000;
  RM_ADD=0x0000000000000000;
  controlReg[0]=0x0A;
  controlReg[1]=0;
  controlReg[2]=0;
  controlReg[3]=0;
  controlReg[4]=0;
  controlReg[5]=0;
  controlReg[6]=0;
  controlReg[7]=0;
  controlReg[8]=0;
  controlReg[9]=0;
  controlReg[10]=0;
  controlReg[11]=0;
  controlReg[12]=0;
  controlReg[13]=0;
  
}
/*********************************************************************
 * @fn      zclSmartMeter_nvReadParam
 *
 * @brief   Called to read SmartMeter parameter information from Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_nvReadParam( void )
{
  //osalSnvId_t FLASH_PARAM;
  //osalSnvLeng_t len;
  //len = 20;  //bytes
  uint16 *flashpt;
  //flashpt = 0x20000020;  //point to memory mapped address
  flashpt = &paramReg[0];
  //locate item in flash memory
  osal_nv_item_init (FLASH_PARAM, 20, NULL);
  //read from flash memory and load it into paramReg
  osal_nv_read (FLASH_PARAM, 0, 20, flashpt);
}
 
/*********************************************************************
 * @fn      zclCoordinator_nvWriteParam
 *
 * @brief   Called to write SmartMeter parameter information into Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_nvWriteParam( void )
{
 //osalSnvId_t FLASH_PARAM;
  //osalSnvLeng_t len;
  //len = 20;
  uint16 *flashpt;
  //flashpt = 0x20000020;  //point to memory mapped address
  flashpt = &paramReg[0];
  //locate item in flash
  osal_nv_item_init (FLASH_PARAM, 20, NULL);
  //write paramReg to FLASH
  osal_nv_write (FLASH_PARAM, 0, 20, flashpt);
   
} 

/*********************************************************************
 * @fn      zclCoordinator_SmartMeterParamCompare
 *
 * @brief   Called to compare parameters
 *
 * @param   none
 *
 * @return  uint8
 */

uint8 zclCoordinator_SmartMeterParamCompare (void)
{
  
  uint8 returnvalue;
  for (index=0; index<10; index++)
  if (SmartMeterparamReg[index] == paramReg[index])
      returnvalue = 0;
      else
        returnvalue = 1;
      return  returnvalue;
 
}

/*********************************************************************
 * @fn      zclCoordinator_dataRegInit
 *
 * @brief   Called to initialize data registers
 *
 * @param   none
 *
 * @return  none
 */
 
 static void zclCoordinator_dataRegInit(void) 
 {
   uint8 i;
   for (i=0; i<13; i++)
   {
     dataReg_Pong[i] = 0;
     dataReg_Ping[i] = 0;
   }
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
 IOCPinConfigPeriphOutput(COORDINATOR_GPIO_UART_BASE, COORDINATOR_PIN_UART_TXD, 
                             IOC_MUX_OUT_SEL_UART0_TXD);
 GPIOPinTypeUARTOutput(COORDINATOR_GPIO_UART_BASE, COORDINATOR_PIN_UART_TXD);
    
 IOCPinConfigPeriphInput(COORDINATOR_GPIO_UART_BASE, COORDINATOR_PIN_UART_RXD, 
                            IOC_UARTRXD_UART0);
 GPIOPinTypeUARTInput(COORDINATOR_GPIO_UART_BASE, COORDINATOR_PIN_UART_RXD);
     
 //
 // Initialize the UART (UART0) for console I/O.
 //
 UARTStdioInit(0);

}

/*********************************************************************
 * @fn      BUILD_UINT32_16
 *
 * @brief   concatenate 16 bits into 32 bits
 *
 * @param   numb1, numb2
 *
 * @return  32 bit result
 */
 
 uint32 BUILD_UINT32_16 (uint16 num1, uint16 num2)
 {
   uint32 result = num1;
   result = (result << 16) | num2;
   return result;
 }
/*********************************************************************
 * @fn      BUILD_UINT64_16
 *
 * @brief   concatenate 16 bits into 64 bits
 *
 * @param   num1, num2, num3, num4
 *
 * @return  64 bit result
 */
 
 uint64 BUILD_UINT64_16 (uint16 num1, uint16 num2, uint16 num3, uint16 num4)
 {
   uint64 result = num1;
   result = (result << 16) | num2;
   result = (result << 16) | num3;
   result = (result << 16) | num4;
   return result;
 }
/*********************************************************************
 * @fn      BUILD_UINT64_8
 *
 * @brief   concatenate 8 bits into 64 bits
 *
 * @param   num1, num2, num3, num4, numb5, numb6, numb7, numb8
 *
 * @return  64 bit result
 */
 
 uint64 BUILD_UINT64_8 (uint8 num1, uint8 num2, uint8 num3, uint8 num4, uint8 num5, uint8 num6, uint8 num7, uint8 num8)
 {
   uint64 result = num1;
   result = (result << 8) | num2;
   result = (result << 8) | num3;
   result = (result << 8) | num4;
   result = (result << 8) | num5;
   result = (result << 8) | num6;
   result = (result << 8) | num7;
   result = (result << 8) | num8;
   return result;
 }

/*********************************************************************
 * @fn      zclCoordinator_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
/*static uint8 zclCoordinator_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}*/

#ifdef ZCL_EZMODE
/*********************************************************************
 * @fn      zclCoordinator_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclCoordinator_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
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
 * @fn      zclCoordinator_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - EZ-Mode state
 *          pData - data appropriate to state
 *
 * @return  none
 */
static void zclCoordinator_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
  
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclCoordinator_IdentifyTime = (EZMODE_TIME / 1000);  // convert to seconds
    zclCoordinator_ProcessIdentifyTimeChange();
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
    if ( pStr )
    {
      if ( giThermostatScreenMode == THERMOSTAT_MAINMODE )
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if ( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclCoordinator_IdentifyTime = 0;
    zclCoordinator_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      // "EZDst:1234 EP:34"
      osal_memcpy(szLine, "EZDst:", 6);
      zclHA_uint16toa( pData->sFinish.nwkaddr, &szLine[6]);
      osal_memcpy(&szLine[10], " EP:", 4);
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
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giThermostatScreenMode == THERMOSTAT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif  // LCD_SUPPORTED

    // show main UI screen 3 seconds after completing EZ-Mode
    osal_start_timerEx( zclCoordinator_TaskID, Coordinator_MAIN_SCREEN_EVT, 3000 );
  }
}
#endif // ZCL_EZMODE

/****************************************************************************
****************************************************************************/


