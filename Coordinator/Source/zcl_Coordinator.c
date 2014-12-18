/**************************************************************************************************
  Filename:       zcl_SmartMeter.c
  Revised:        $Date: 2014-12-15 17:02:21 -0700 
  Revision:       $Revision: 35718 $
  Description:   This device will act as a smart meter.
**************************************************************************************************/
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
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
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
#include "OSAL_Clock.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define USR_RX_GET 0xC0
#define USR_TX_GET 0xC1
#define USR_RX_SET 0xC2
#define USR_TX_SET 0xC3
#define COM_PARAM  0xC4
#define COM_DATA   0xC5
#define COM_ADD    0xC6
#define RESET      0xC7
#define RELAY      0xC8
#define START      0xC9
#define SET_PARAM  0xCA
#define ACK_SUCCESS  0xCB
#define CALIBRATE   0xCC
#define CAL_VOL     0xCD
#define CAL_CUR     0xCE
#define CAL_ENE     0xCF
#define TIME_SET     0xD0
#define GET_CALPARAM 0xD1

#define FLASH_PARAM  0x1000
#define FLASH_PARAMCAL  0x1040


// how often to sample ADC in millisecond
//#define SmartMeter_PowerCal_INTERVAL   56
#define SmartMeter_PowerCal_INTERVAL   1
#define SmartMeter_Calibration_INTERVAL 1
/*********************************************************************
* Pin Definition
*/
const uint16 senPin1Voltage = SOCADC_AIN5;
const uint16 senPin1Current = SOCADC_AIN6;
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
#define SMARTMETER_PIN_UART_RXD            GPIO_PIN_0
#define SMARTMETER_PIN_UART_TXD            GPIO_PIN_1
#define SMARTMETER_GPIO_UART_BASE          GPIO_A_BASE
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSmartMeter_TaskID;
uint8 zclSmartMeterSeqNum;
uint16 flagreset = 0;
uint16 flagrelay = 2;
uint8 flaginc;
uint8 SerControl;
uint32 ENERGY_RESET_VALUE;
uint16 ENERGY_RESET_VALUE_1;
uint16 ENERGY_RESET_VALUE_0;
uint32 POWER;
uint16 POWER1;
uint16 POWER0;
uint32 ENERGY;
uint16 ENERGY1;
uint16 ENERGY0;
uint16 SM_V;
uint16 SM_I;
uint16 STATUS;

//paramters------------------------------
uint16 MIN_ADC; //paramReg[0]
uint16 MAX_ADC; //paramReg[1]
uint16 SAMPLE_INT;//paramReg[2]
uint16 SAMPLE_WIN;//paramReg[3]
uint16 MAG_V1;//paramReg[4]
uint16 MAG_I1;//paramReg[5]
uint16 MAG_V2;//paramReg[6]
uint16 MAG_I2;//paramReg[7]
uint16 MAG_V3;//paramReg[8]
uint16 MAG_I3;//paramReg[9]
uint16 MAG_E1 = 100;
uint16 MAG_E2;
uint16 MAG_E3;

uint16 MIN_V;//paramReg[10]
uint16 MAX_V;//paramReg[11]
uint16 MIN_I;//paramReg[12]
uint16 MAX_I;//paramReg[13]
uint16 T_EFF;//paramReg[14]
uint16 SHARP1;//paramReg[15]
uint16 SHARP2;//paramReg[16]
uint16 PEAK1;//paramReg[17]
uint16 PEAK2;//paramReg[18]
uint16 PEAK3;//paramReg[19]
uint16 SHOULDER1;//paramReg[20]
uint16 SHOULDER2;//paramReg[21]
uint16 SHOULDER3;//paramReg[22]
uint16 OFF;//paramReg[23]
uint16 N_SM;//paramReg[24]

uint16 sm_index = 0;
uint16 YEAR;
uint16 MONTH;
uint16 DAY;
uint16 HOUR;
uint16 MINUTE;
uint16 SECOND;
//uint16 timeReg[10] = {0};
uint32 sys_timeold = 0;
uint32 sys_timenew = 0;
uint32 sys_secold = 0;
uint32 sys_secnew = 0;

UTCTimeStruct TimeStruct;

uint32 enecal_timeold = 0;
uint32 enecal_timenew = 0;
uint16 enecal_timeperiod = 0;
uint32 enecal_cycle = 0;

uint8 V_CAL = 0;
uint8 I_CAL = 0;
uint8 T_CAL = 0;
uint8 N_CAL = 0;
uint16 CAL_OPT = 0;

uint32 E_CAL = 0;

uint16 senValueV[3] = {0};
uint16 senValueI[3] = {0};
int16 realVol[3] = {0};        //can be negative
int16 realCur[3] = {0};        //can be negative
uint32 powerVal[3] = {0};
uint32 energyVal[3] = {0};
uint32 enecal_energy[3] = {0};
double energy_display[3] = {0};

uint32 energyVal_Lcd_display;
uint16 RMS_V1;
uint16 RMS_V2;
uint16 RMS_V3;
uint16 RMS_I1;
uint16 RMS_I2;
uint16 RMS_I3;
uint16 THETA_1;
uint16 THETA_2;
uint16 THETA_3;


float CurDisplay[3] = {0};
float PowerDisplay[3] = {0};


int32 VrmsTemp[3] = {0};
int32 IrmsTemp[3] = {0};
int32 powerTemp[3] = {0};
uint32 l_nSamples;
uint16 CRMS_V[3] = {0};
uint16 CRMS_I[3] = {0};
double Energy[3];
uint8 cal_index = 0;

uint16 status;
char Enstring[25];    //parameter for display floating number


uint64 sm_ADD; //smart meter external IEEE address
uint16 sm_nwkADD;  //smart meter network address
uint8 *psm_ADD; //pointer to smart meter external IEEE address
uint64 coordinator_extAddr; // Coordinator extended IEEE address
uint16 paramReg[25] = {0};
uint16 calReg[20] = {0};

uint16 dataReg[30];
uint16 coordinator_Addr_3;
uint16 coordinator_Addr_2;
uint16 coordinator_Addr_1;
uint16 coordinator_Addr_0;
uint16 ADD_3;
uint16 ADD_2;
uint16 ADD_1;
uint16 ADD_0;
uint16 SM_ADD16 = 0;

uint32 time_old = 0;
uint32 time_new = 0;
/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSmartMeter_DstAddr;
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
static void zclsmartmeter_startEZmode( void );
// app display functions
void zclSmartMeter_LCDDisplayUpdate(void);
void zclSmartMeter_LcdPowerDisplayUpdate(void);
void zclSmartMeter_LcdDisplayTempMode(void);
void zclSmartMeter_LcdDisplayTestMode(void);
// app SmartMeter functions
static void zclSmartMeter_SendParam(void);
static void zclSmartMeter_SendTime(void);
static void zclSmartMeter_SendData(void);
static void zclSmartMeter_SendReset(void);
static void zclSmartMeter_SendRelay(void);
static void zclSmartMeter_SendAdd(void);
static void zclSmartMeter_nvReadParam(void);
static void zclSmartMeter_nvWriteParam(void);
static void zclSmartMeter_SendCalibrate(void);
static void zclSmartMeter_SendCalPara(void);

void zclSmartMeter_dataRegInit(void);
void zclSmartMeter_parameterInit(void); //include paramRegInit
void zclSmartMeter_calibrateInc(void);
void zclSmartMeter_UpdateDataReg(void);
int32 zclSmartMeter_map(int16 senValue, int16 MIN_ADC, int16 MAX_ADC, int16 MIN_PEAK, int16 MAX_PEAK);
// app SmartMeter functions
void zclSmartMeter_ADC_init(void);
uint16 analogRead(uint32 senValue);
void InitConsole(void);
uint16 sm_rand(void);
uint32 BUILD_UINT32_16(uint16 num1, uint16 num2);
uint64 BUILD_UINT64_16(uint16 num1, uint16 num2, uint16 num3, uint16 num4);
// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSmartMeter_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSmartMeter_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSmartMeter_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_REPORT
static void zclSmartMeter_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
#endif  // ZCL_REPORT
/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]    = " ";
const char sDeviceName[]   = "     Smart Meter";
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
    // Initialize pins for relay control
    GPIOPinTypeGPIOOutput(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
    //PC0 is in cut-off state, PC1 is in on state
    GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x02);  //LED2=1, LED1=0
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
    HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_5 );
#endif
    zclsmartmeter_startEZmode();
    // Initialize SmartMeter parameters
    zclSmartMeter_parameterInit();
    //Initialize SmartMeter data register
    zclSmartMeter_dataRegInit();
    //Initialize ADC
    zclSmartMeter_ADC_init();
    // Get smart meter 64-bit IEEE external address and network address
    psm_ADD = saveExtAddr;
    sm_nwkADD = NLME_GetShortAddr();
    ADD_3 = (uint16)saveExtAddr[6] + ((((uint16)saveExtAddr[7]) << 8) & 0xFF00);
    ADD_2 = (uint16)saveExtAddr[4] + ((((uint16)saveExtAddr[5]) << 8) & 0xFF00);
    ADD_1 = (uint16)saveExtAddr[2] + ((((uint16)saveExtAddr[3]) << 8) & 0xFF00);
    ADD_0 = (uint16)saveExtAddr[0] + ((((uint16)saveExtAddr[1]) << 8) & 0xFF00);
    sm_ADD = ((uint64)ADD_0 & 0xFFFF) + ((((uint64)ADD_1) << 16) & 0xFFFF0000) + ((((uint64)ADD_2) << 32) & 0xFFFF00000000) + ((((uint64)ADD_3) << 48) & 0xFFFF000000000000);
    // Set up the serial console to use for displaying messages.  This is
    // just for debugging purpose and is not needed for Systick operation.
    InitConsole();
    
    enecal_timeold = osal_GetSystemClock();
    enecal_timenew = enecal_timeold;
    energy_display[0] = 0;
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
    afIncomingMSGPacket_t *MSGpkt; //AF.h
    (void)task_id;  // Intentionally unreferenced parameter
    char  lcdString[10];
        

    
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
                    zclSmartMeter_LCDDisplayUpdate();
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
        zclSmartMeter_LCDDisplayUpdate();
        return ( events ^ SmartMeter_MAIN_SCREEN_EVT );
    }
    if ( events & SmartMeter_ADC_SEND_EVT )
    {
        zclSmartMeter_calibrateInc();

        if(CAL_OPT != 0)
            // get ADC reading every 1 millisecond
            osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, SmartMeter_Calibration_INTERVAL );
        else
            // get ADC reading every 56 millisecond
            osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, SmartMeter_PowerCal_INTERVAL );
        
            sys_timenew = osal_GetSystemClock();
            sys_secnew = sys_secold + (uint32)((float)((sys_timenew - sys_timeold)*3.125)/1000);    
            //sys_timeold = sys_timenew;
            //sys_secold = sys_secnew;
            osal_ConvertUTCTime(&TimeStruct , sys_secnew); 
            if(TimeStruct.month == 0)
            {    
                TimeStruct.month = 12;
                TimeStruct.year--;
            }
    
#ifdef LCD_SUPPORTED
    sprintf((char *)lcdString, "%d %d %d %d %d %d",TimeStruct.year, TimeStruct.month, 
            TimeStruct.day, TimeStruct.hour, TimeStruct.minutes, TimeStruct.seconds);
    HalLcdWriteString( lcdString, HAL_LCD_LINE_7 );
#endif    
    
        return ( events ^ SmartMeter_ADC_SEND_EVT );
    }
    
      
    
    
    
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
      zclSmartMeter_LcdPowerDisplayUpdate();
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
    if ( shift && ( keys & HAL_KEY_SW_5 ) )
    {
    }
    else if ( keys & HAL_KEY_SW_5 )
    {
    }
}
void zclsmartmeter_startEZmode( void )
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
        ezModeData.numActiveInClusters = 1; //1
        ezModeData.pActiveInClusterIDs = clusterIDs;
        ezModeData.numActiveOutClusters = 0;   // active output cluster
        ezModeData.pActiveOutClusterIDs = NULL;
        zcl_InvokeEZMode( &ezModeData );
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif
#endif // ZCL_EZMODE
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
    uint16 VOLTAGE = dataReg[4];
    uint16 CURRENT = dataReg[5];
    uint32 ENERGY = BUILD_UINT32_16(dataReg[8], dataReg[9]);
    osal_memcpy( sDisplayVoltage, "V: ", 11 );
    _ltoa(  VOLTAGE , (void *)(&sDisplayVoltage[11]), 10 ); // only use whole number
    osal_memcpy( sDisplayCurrent, "A: ", 11 );
    _ltoa(  CURRENT , (void *)(&sDisplayCurrent[11]), 10 ); // only use whole number
    osal_memcpy( sDisplayEnergy, "kWh: ", 11 );
    _ltoa(  ENERGY , (void *)(&sDisplayEnergy[11]), 10 ); // only use whole number
    HalLcdWriteString( (char *)sDisplayVoltage, HAL_LCD_LINE_1 );
    HalLcdWriteString( (char *)sDisplayCurrent, HAL_LCD_LINE_2 );
    HalLcdWriteString( (char *)sDisplayEnergy, HAL_LCD_LINE_3 );
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_4 );
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_5 );
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_6 );
    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_7 );
    HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_8 );
#endif
}
void zclSmartMeter_LcdDisplayTestMode( void )
{
    char sDisplayCoIEEEaddr[32];
    uint8 IEEEaddr[8];
    // display coordinator IEEE addr
    osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
    if ( (coordinator_Addr_0 == NULL) || (coordinator_Addr_1 == NULL) || (coordinator_Addr_2 == NULL) || (coordinator_Addr_3 == NULL))
    {
        osal_memcpy( &sDisplayCoIEEEaddr[5], "N/A", 4 );
    }
    else
    {
        osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
        IEEEaddr[0] = (coordinator_Addr_3 >> 8) & 0x00FF;
        IEEEaddr[1] = coordinator_Addr_3 & 0x00FF;
        IEEEaddr[2] = (coordinator_Addr_2 >> 8) & 0x00FF;
        IEEEaddr[3] = coordinator_Addr_2 & 0x00FF;
        IEEEaddr[4] = (coordinator_Addr_1 >> 8) & 0x00FF;
        IEEEaddr[5] = coordinator_Addr_1 & 0x00FF;
        IEEEaddr[6] = (coordinator_Addr_0 >> 8) & 0x00FF;
        IEEEaddr[7] = coordinator_Addr_0 & 0x00FF;
        for(int i = 0; i < 8; i++)
        {
            _ltoa( IEEEaddr[i] >> 4, (void *)(&sDisplayCoIEEEaddr[5 + i * 2]), 16 );
            _ltoa( IEEEaddr[i] & 0x0F, (void *)(&sDisplayCoIEEEaddr[5 + i * 2 + 1]), 16 );
        }
    }
#ifdef LCD_SUPPORTED
    HalLcdWriteString( (char *)sDisplayCoIEEEaddr, HAL_LCD_LINE_1 );
#endif
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
static void zclSmartMeter_SendParam( void )
{
#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;

    uint16 packet[] = {  USR_TX_GET, SUCCESS, MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN,                         
                         MIN_V, MAX_V, MIN_I, MAX_I,
                         SHARP1, SHARP2,
                         PEAK1, PEAK2, PEAK3,
                         SHOULDER1, SHOULDER2, SHOULDER3,
                         OFF, N_SM, sm_index
                      };

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT512;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                           ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}
/*********************************************************************
 * @fn      zclSmartMeter_SendTime     *

 *
 * @brief   Called to send SmartMeter parameter information to the coordinator

 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendTime( void )                                         //added 11.12

{
#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;
    uint16 packet[] = {TIME_SET, SUCCESS, YEAR, MONTH, DAY, HOUR, MINUTE,
                       SECOND, sm_index
                      };

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;

        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT256;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                           ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                          // ???
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }

    HalLcdWriteString( "sendTime", HAL_LCD_LINE_7 );
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


    POWER1 = (powerVal[0] >> 16) & 0xFFFF;
    POWER0 = powerVal[0] & 0xFFFF;
    ENERGY1 = (energyVal[0] >> 16) & 0xFFFF;
    ENERGY0 = energyVal[0] & 0xFFFF;
    uint16 packet[] = {USR_TX_GET, SUCCESS, ADD_3, ADD_2, ADD_1, ADD_0, SM_ADD16,
                       RMS_V1, RMS_I1, THETA_1, RMS_V2, RMS_I2, THETA_2, RMS_V3, RMS_I3, THETA_3, SM_V, SM_I, YEAR, MONTH, DAY, HOUR, MINUTE, SECOND, STATUS  
                      };
    UARTprintf(" RMS_V1: %d\n", RMS_V1);
    UARTprintf(" RMS_I1: %d\n", RMS_I1);

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;

        pReportCmd->attrList[0].attrID = ATTRID_MS_DATA_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT512;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,

                           ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }


    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}

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


    uint16 energy_reset_value_1 = 0;
    uint16 energy_reset_value_0 = 0;
    energy_reset_value_1 = (uint16)(((uint32)(energy_display[0]) >> 16) & 0xFFFF);
    energy_reset_value_0 = (uint16)((uint32)(energy_display[0]) & 0xFFFF);


    uint16 packet[] = {(uint16)RESET, (uint16)SUCCESS, flagreset, energy_reset_value_1, energy_reset_value_0};

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;

        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,

                           ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}
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





    uint16 packet[] = {RELAY, SUCCESS, flagrelay};
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;

        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT64;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                           ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}

/*********************************************************************

* @fn      zclSmartMeter_SendCalibrate
*

* @brief   Called to send SmartMeter calibration information to the coordinator
*
* @param   none
*
* @return  none
*/

static void zclSmartMeter_SendCalibrate(void)
{
#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;

    uint16 packet[] = {CALIBRATE, SUCCESS, V_CAL, I_CAL, T_CAL, N_CAL};
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;

        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                           ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT     
}


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
    uint16 packet[] = {START, SUCCESS, flaginc};
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT64;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                           ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
}
/*********************************************************************
 * @fn      zclSmartMeter_SendAdd    *
 *
 * @brief   Called to send SmartMeter external IEEE address information
 * in response to a network discovery broadcast from coordinator
 *
 * @return  none
 */
static void zclSmartMeter_SendAdd(void)
{
#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;
    uint16 packet[] = {USR_TX_GET, SUCCESS, ADD_3, ADD_2, ADD_1, ADD_0};
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_ADD_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        // send smart meter IEEE address to coordinator after a random delay from
        // time of reception of network discovery command
        uint16 SmartMeter_NwkDiscov_INTERVAL = sm_rand() * 1000; //generate a random delay from 0 to 16000000 SysTIck
        // add random delay function here
        unsigned long ulValue_start;
        // NVIC_ST_CURRENT register
        // must be written to force the reload. Any write to this register clears the SysTick counter to 0
        // and causes a reload with the supplied period on the next clock.
        // add code to write to  NVIC_ST_CURRENT register
        // Configure and enable the SysTick counter.
        SysTickPeriodSet(100000); //16000000
        SysTickEnable();
        // Read the current SysTick value.
        ulValue_start = SysTickValueGet();
        while (SysTickValueGet() - ulValue_start < SmartMeter_NwkDiscov_INTERVAL)
        {
        };
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                           ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT 
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
static void zclSmartMeter_nvReadParam( void )
{
    uint16 *flashpt;
    flashpt = &paramReg[0];
    //locate item in flash memory
    osal_nv_item_init (FLASH_PARAM, 36, NULL);
    //read from flash memory and load it into paramReg
    osal_nv_read (FLASH_PARAM, 0, 36, flashpt);
    
      
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
    uint16 *flashpt;
    
    paramReg[0] = MIN_ADC;
    paramReg[1] = MAX_ADC;
    paramReg[2] = SAMPLE_INT;
    paramReg[3] = SAMPLE_WIN;
    //paramReg[4] = MAG_V1;
    //paramReg[5] = MAG_I1;
    //paramReg[6] = MAG_V2;
    //paramReg[7] = MAG_I2;
    //paramReg[8] = MAG_V3;
    //paramReg[9] = MAG_I3;
    paramReg[4] = MIN_V;
    paramReg[5] = MAX_V;
    paramReg[6] = MIN_I;
    paramReg[7] = MAX_I;
    //paramReg[14] = T_EFF;
    paramReg[8] = SHARP1;
    paramReg[9] = SHARP2;
    paramReg[10] = PEAK1;
    paramReg[11] = PEAK2;
    paramReg[12] = PEAK3;
    paramReg[13] = SHOULDER1;
    paramReg[14] = SHOULDER2;
    paramReg[15] = SHOULDER3;
    paramReg[16] = OFF;
    paramReg[17] = N_SM;    
    
    int i;
    for(i = 0; i < 18; i++)
        UARTprintf(" writeparamReg[%d]: %d\n", i, paramReg[i]);
    UARTprintf(" sm_index: %d\n", sm_index);

    flashpt = &paramReg[0];
    //locate item in flash
    osal_nv_item_init (FLASH_PARAM, 36, NULL);
    //write paramReg to FLASH
    osal_nv_write (FLASH_PARAM, 0, 36, flashpt);
    
}

/*********************************************************************
 * @fn      zclSmartMeter_calReadParam
 *
 * @brief   Called to read SmartMeter calibration related parameter information from Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_calReadParam( void )
{
    uint16 *flashpt;
    flashpt = &calReg[0];
    //locate item in flash memory
    osal_nv_item_init (FLASH_PARAMCAL, 30, NULL);
    //read from flash memory and load it into paramReg
    osal_nv_read (FLASH_PARAMCAL, 0, 30, flashpt);
           
}
/*********************************************************************
 * @fn      zclSmartMeter_calWriteParam
 *
 * @brief   Called to write SmartMeter calibration related parameter information into Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_calWriteParam( void )
{
    uint16 *flashpt;
    
    calReg[0] = ADD_3;  //high
    calReg[1] = ADD_2;
    calReg[2] = ADD_1;
    calReg[3] = ADD_0;  
    
    if(ADD_3 == 0x0012 && ADD_2 == 0x4B00 && ADD_1 == 0x040F && ADD_0 == 0x1A3C)
        SM_ADD16 = 0x0000;
    else if(ADD_3 == 0x0012 && ADD_2 == 0x4B00 && ADD_1 == 0x040F && ADD_0 == 0x1C77)
        SM_ADD16 = 0x0001;
    else if(ADD_3 == 0x0012 && ADD_2 == 0x4B00 && ADD_1 == 0x040E && ADD_0 == 0xF19E)
        SM_ADD16 = 0x0002;
    
    calReg[4] = SM_ADD16;
    calReg[5] = MAG_V1;
    calReg[6] = MAG_I1;
    calReg[7] = MAG_E1;
    calReg[8] = MAG_V2;
    calReg[9] = MAG_I2;
    calReg[10] = MAG_E2;
    calReg[11] = MAG_V3;
    calReg[12] = MAG_I3;
    calReg[13] = MAG_E3;
    calReg[14] = T_EFF;
    
    
    int i;
    for(i = 0; i < 15; i++)
        UARTprintf(" writecalReg[%d]: %d\n", i, calReg[i]);

    flashpt = &calReg[0];
    //locate item in flash
    osal_nv_item_init (FLASH_PARAMCAL, 30, NULL);
    //write paramReg to FLASH
    osal_nv_write (FLASH_PARAMCAL, 0, 30, flashpt);    
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
    zclReportCmd_t *pInParameterReport;
    pInParameterReport = (zclReportCmd_t *)pInMsg->attrCmd;
    uint16 COMMAND = BUILD_UINT16(pInParameterReport->attrList[0].attrData[0], pInParameterReport->attrList[0].attrData[1]);
    uint16 OPERATION = BUILD_UINT16(pInParameterReport->attrList[0].attrData[2], pInParameterReport->attrList[0].attrData[3]);
    if ((COMMAND == USR_RX_GET) && (OPERATION == COM_PARAM) &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_PARAMETER_MEASURED_VALUE)
        // send the current parameter value to send over the air to Coordinator
    {
        uint16 SmartMeter_NwkDiscov_INTERVAL = sm_rand() * 1000; //generate a random delay from 0 to 16000000 SysTIck
        // add random delay function here
        unsigned long ulValue_start;
        // NVIC_ST_CURRENT register
        // must be written to force the reload. Any write to this register clears the SysTick counter to 0
        // and causes a reload with the supplied period on the next clock.
        // add code to write to  NVIC_ST_CURRENT register
        // Configure and enable the SysTick counter.
        SysTickPeriodSet(100000); //16000000
        SysTickEnable();
        // Read the current SysTick value.
        ulValue_start = SysTickValueGet();
        while (SysTickValueGet() - ulValue_start < SmartMeter_NwkDiscov_INTERVAL)
        {
        };
        zclSmartMeter_SendParam();
    }
    if ((COMMAND == USR_RX_GET) && (OPERATION == COM_DATA) &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_DATA_MEASURED_VALUE)
        // send the current data value sent over the air to Coordinator
    {
        flaginc = 1;
        
        sys_timenew = osal_GetSystemClock();
        sys_secnew = sys_secold + (uint32)((float)((sys_timenew - sys_timeold)*3.125)/1000);
        osal_ConvertUTCTime(&TimeStruct , sys_secnew);
        if(TimeStruct.month == 0)
        {    
            TimeStruct.month = 12;
            TimeStruct.year--;
        }
        
        SECOND = (uint16)TimeStruct.seconds;
        MINUTE = (uint16)TimeStruct.minutes;
        HOUR = (uint16)TimeStruct.hour;
        DAY = (uint16)TimeStruct.day;
        MONTH = (uint16)TimeStruct.month;
        YEAR = (uint16)TimeStruct.year;
        
        UARTprintf(" sys_timenew: %d\n", sys_timenew);
        UARTprintf(" sys_timeold: %d\n", sys_timeold);
        UARTprintf(" sys_secnew: %d\n", sys_secnew);
        UARTprintf(" YEAR: %d\n", YEAR);
        UARTprintf(" MONTH: %d\n", MONTH);
        UARTprintf(" DAY: %d\n", DAY);
        UARTprintf(" HOUR: %d\n", HOUR);
        UARTprintf(" MINUTE: %d\n", MINUTE);
        UARTprintf(" SECOND: %d\n", SECOND);
        
        zclSmartMeter_SendData();
    }
    if ((COMMAND == USR_RX_SET) && (OPERATION == SET_PARAM) &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_PARAMETER_MEASURED_VALUE)
    {
        // set the current parameter sent over the air from the Coordinator
        MIN_ADC = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        MAX_ADC = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        SAMPLE_INT = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        SAMPLE_WIN = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);       
        MIN_V = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        MAX_V = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        MIN_I = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);
        MAX_I = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]);
        SHARP1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20], pInParameterReport->attrList[0].attrData[21]);
        SHARP2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22], pInParameterReport->attrList[0].attrData[23]);
        PEAK1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24], pInParameterReport->attrList[0].attrData[25]);
        PEAK2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26], pInParameterReport->attrList[0].attrData[27]);
        PEAK3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[28], pInParameterReport->attrList[0].attrData[29]);
        SHOULDER1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[30], pInParameterReport->attrList[0].attrData[31]);
        SHOULDER2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[32], pInParameterReport->attrList[0].attrData[33]);
        SHOULDER3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[34], pInParameterReport->attrList[0].attrData[35]);
        OFF = BUILD_UINT16(pInParameterReport->attrList[0].attrData[36], pInParameterReport->attrList[0].attrData[37]);
        N_SM = BUILD_UINT16(pInParameterReport->attrList[0].attrData[38], pInParameterReport->attrList[0].attrData[39]);
        sm_index = BUILD_UINT16(pInParameterReport->attrList[0].attrData[40], pInParameterReport->attrList[0].attrData[41]);

        

        //Update flash memory
        zclSmartMeter_nvWriteParam();
        // send the current parameter value to send over the air to Coordinator
        zclSmartMeter_SendParam();
    }

    if ((COMMAND == TIME_SET) &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)                                    //added 11.12
    {
        // set the current parameter sent over the air from the Coordinator
        YEAR = BUILD_UINT16(pInParameterReport->attrList[0].attrData[2], pInParameterReport->attrList[0].attrData[3]);
        MONTH = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        DAY = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        HOUR = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        MINUTE = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        SECOND = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        sm_index = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        
        //timeReg[0] = YEAR;
        //timeReg[1] = MONTH;
        //timeReg[2] = DAY;
        //timeReg[3] = HOUR;
        //timeReg[4] = MINUTE;
        //timeReg[5] = SECOND;
        HalLcdWriteString( "timesetvalue", HAL_LCD_LINE_7 );
        UARTprintf(" YEAR: %d\n", YEAR);
        UARTprintf(" MONTH: %d\n", MONTH);
        UARTprintf(" DAY: %d\n", DAY);
        UARTprintf(" HOUR: %d\n", HOUR);
        UARTprintf(" MINUTE: %d\n", MINUTE);
        UARTprintf(" SECOND: %d\n", SECOND);
        UARTprintf(" sm_index: %d\n", sm_index);
        
        int i;
        for(i = 0; i < 18; i++)
        UARTprintf(" readparamReg[%d]: %d\n", i, paramReg[i]);
        
        for(i = 0; i < 15; i++)
        UARTprintf(" readcalReg[%d]: %d\n", i, calReg[i]);
        //Update flash memory
        //  zclSmartMeter_nvWriteParam();                                    
        // send the current parameter value to send over the air to Coordinator

        
        sys_timeold = osal_GetSystemClock();
        sys_timenew = sys_timeold;
        TimeStruct.seconds = (uint8)SECOND;
        TimeStruct.minutes = (uint8)MINUTE;
        TimeStruct.hour = (uint8)HOUR;
        TimeStruct.day = (uint8)DAY;
        TimeStruct.month = (uint8)MONTH;
        TimeStruct.year = (uint16)YEAR;
        
        UARTprintf(" TimeStruct.seconds: %d\n", TimeStruct.seconds);
        UARTprintf(" TimeStruct.minutes: %d\n", TimeStruct.minutes);
        UARTprintf(" TimeStruct.hour: %d\n", TimeStruct.hour);
        UARTprintf(" TimeStruct.day: %d\n", TimeStruct.day);
        UARTprintf(" TimeStruct.month: %d\n", TimeStruct.month);
        UARTprintf(" TimeStruct.year: %d\n", TimeStruct.year);
        sys_secold = osal_ConvertUTCSecs( &TimeStruct );
        UARTprintf(" sys_secold: %d\n", sys_secold);
        zclSmartMeter_SendTime();
        //osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_CLOCK_EVT, 1000 );  //continues 1s
    }

    if ((COMMAND == RESET)  &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
    {
        flagreset = OPERATION;
        ENERGY_RESET_VALUE_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        ENERGY_RESET_VALUE_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        ENERGY_RESET_VALUE = BUILD_UINT32_16(ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0);
        energy_display[0] = (double)ENERGY_RESET_VALUE;
        
        UARTprintf(" ENERGY_RESET_VALUE: %d\n", ENERGY_RESET_VALUE);
        // send the current flagreset and ENERGY_RESET_VALUE to send over the air to Coordinator
        zclSmartMeter_SendReset();
    }


    if ((COMMAND == RELAY) &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
    {
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "SendRelay SUCCESS", HAL_LCD_LINE_2 );
#endif
        flagrelay = OPERATION;
        if(!flagrelay)
            GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01); // PC1=0, PC0=1
        else if (flagrelay == 1)
            GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x02); // PC1=1, PC0=0

        // send the current relay value to send over the air to Coordinator
        zclSmartMeter_SendRelay();
        flagrelay = 2;
    }

    if ((COMMAND == START) &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
    {
        flaginc = OPERATION;
        // send the current flaginc value to send over the air to Coordinator
        zclSmartMeter_SendRestart();
    }

    if ((COMMAND == USR_RX_GET) && (OPERATION == COM_ADD) &&
            (pInParameterReport->attrList[0].attrID) == ATTRID_MS_ADD_MEASURED_VALUE)
    {
        // get the coordinator IEEE address
        coordinator_Addr_3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        coordinator_Addr_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        coordinator_Addr_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        coordinator_Addr_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        coordinator_extAddr = BUILD_UINT64_16(coordinator_Addr_3, coordinator_Addr_2,
                                              coordinator_Addr_1, coordinator_Addr_0);
        // Set destination address to 64-bit  check &zclSmartMeter_DstAddr
        zclSmartMeter_DstAddr.addrMode = (afAddrMode_t)Addr64Bit;
        zclSmartMeter_DstAddr.endPoint = SmartMeter_ENDPOINT;
        zclSmartMeter_DstAddr.addr.extAddr[7] = (uint8)(((coordinator_extAddr) >> 56) & 0x00000000000000FF);
        zclSmartMeter_DstAddr.addr.extAddr[6] = (uint8)(((coordinator_extAddr) >> 48) & 0x00000000000000FF);
        zclSmartMeter_DstAddr.addr.extAddr[5] = (uint8)(((coordinator_extAddr) >> 40) & 0x00000000000000FF);
        zclSmartMeter_DstAddr.addr.extAddr[4] = (uint8)(((coordinator_extAddr) >> 32) & 0x00000000000000FF);
        zclSmartMeter_DstAddr.addr.extAddr[3] = (uint8)(((coordinator_extAddr) >> 24) & 0x00000000000000FF);
        zclSmartMeter_DstAddr.addr.extAddr[2] = (uint8)(((coordinator_extAddr) >> 16) & 0x00000000000000FF);
        zclSmartMeter_DstAddr.addr.extAddr[1] = (uint8)(((coordinator_extAddr) >> 8) & 0x00000000000000FF);
        zclSmartMeter_DstAddr.addr.extAddr[0] = (uint8)((coordinator_extAddr) & 0x00000000000000FF);
        // send smart meter IEEE address to coordinator after a random delay from
        // time of reception of network discovery command  -- add code
        zclSmartMeter_SendAdd();
        zclSmartMeter_LcdDisplayTestMode(); // display C's IEEE address
    }

    if ((COMMAND == USR_RX_GET) && (OPERATION == ACK_SUCCESS) &&
            ((pInParameterReport->attrList[0].attrID) == ATTRID_MS_ACK_MEASURED_VALUE ))
    {
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "SendAck SUCCESS", HAL_LCD_LINE_2 );
#endif
    }
    
      if ((COMMAND == USR_RX_GET) && (OPERATION == GET_CALPARAM) &&
            ((pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE ))
    {
        zclSmartMeter_SendCalPara();
    }
    
    if ((COMMAND == CALIBRATE) && (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
    {
        V_CAL = BUILD_UINT16(pInParameterReport->attrList[0].attrData[2], pInParameterReport->attrList[0].attrData[3]);
        I_CAL = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        T_CAL = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        N_CAL = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);

        time_old = osal_GetSystemClock();
        time_new = time_old;
        
        
        l_nSamples = 0;
        VrmsTemp[0] = 0;
        IrmsTemp[0] = 0;

        if(V_CAL != 0 && I_CAL == 0)
        {
            CAL_OPT = CAL_VOL;
            MAG_V1 = 0;
            UARTprintf(" CAL_VOL ");
            UARTprintf(" V_CAL: %d\n", V_CAL);
            UARTprintf(" I_CAL: %d\n", I_CAL);
            UARTprintf(" T_CAL: %d\n", T_CAL);
            UARTprintf(" N_CAL: %d\n", N_CAL);
        }
        else if(V_CAL == 0 && I_CAL != 0)
        {
            CAL_OPT = CAL_CUR;
            MAG_I1 = 0;
            UARTprintf(" CAL_CUR ");
            UARTprintf(" V_CAL: %d\n", V_CAL);
            UARTprintf(" I_CAL: %d\n", I_CAL);
            UARTprintf(" T_CAL: %d\n", T_CAL);
            UARTprintf(" N_CAL: %d\n", N_CAL);
        }
        else if(V_CAL != 0 && I_CAL != 0)
        {
            CAL_OPT = CAL_ENE;
            MAG_E1 = 100;
            MAG_E2 = 100;
            MAG_E3 = 100;
            enecal_cycle = 0;
            UARTprintf(" CAL_ENE ");
            UARTprintf(" V_CAL: %d\n", V_CAL);
            UARTprintf(" I_CAL: %d\n", I_CAL);
            UARTprintf(" T_CAL: %d\n", T_CAL);
            UARTprintf(" N_CAL: %d\n", N_CAL);
        }

    }

}
#endif  // ZCL_REPORT

/*********************************************************************

* @fn      zclSmartMeter_SendCalPara
*
* @brief   Called to send SmartMeter calibration parameter to the coordinator
*
* @param   none
*
* @return  none
*/

static void zclSmartMeter_SendCalPara(void)
{
#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;

    uint16 packet[] = {GET_CALPARAM, SUCCESS, SM_ADD16, ADD_3, ADD_2, ADD_1, ADD_0,
                        MAG_V1, MAG_I1, MAG_V2, MAG_I2, MAG_V3, MAG_I3,
                       T_EFF};   
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT256;
        pReportCmd->attrList[0].attrData = (void *)(packet);
        zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                           ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
    }
    osal_mem_free( pReportCmd );
    
            UARTprintf(" SM_ADD16: 0x%x\n", SM_ADD16);
            UARTprintf(" ADD_3: 0x%x\n", ADD_3);
            UARTprintf(" ADD_2: 0x%x\n", ADD_2);
            UARTprintf(" ADD_1: 0x%x\n", ADD_1);
            UARTprintf(" ADD_0: 0x%x\n", ADD_0);
            UARTprintf(" MAG_V1: 0x%x\n", MAG_V1);
            UARTprintf(" MAG_I1: 0x%x\n", MAG_I1);
            UARTprintf(" T_EFF: 0x%x\n", T_EFF);
            
#endif  // ZCL_REPORT     
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
    uint8 i;
    for (i = 0; i < 18; i++)
        paramReg[i] = 0;
    zclSmartMeter_nvReadParam(); //read parameter from FLASH
    MIN_ADC = paramReg[0];
    MAX_ADC = paramReg[1];
    SAMPLE_INT = paramReg[2];
    SAMPLE_WIN = paramReg[3];
    MAX_V = paramReg[4];
    MIN_V = paramReg[5];
    MAX_I = paramReg[6];
    MIN_I = paramReg[7];
    SHARP1 = paramReg[8];
    SHARP2 = paramReg[9];
    PEAK1 = paramReg[10];
    PEAK2 = paramReg[11];
    PEAK3 = paramReg[12];
    SHOULDER1 = paramReg[13];
    SHOULDER2 = paramReg[14];
    SHOULDER3 = paramReg[15];
    OFF = paramReg[16];
    N_SM = paramReg[17];
    
    for (i = 0; i < 15; i++)
        calReg[i] = 0;
    zclSmartMeter_calReadParam();

    SM_ADD16 = calReg[4];
    MAG_V1 = calReg[5];
    MAG_I1 = calReg[6];
    MAG_E1 = calReg[7];
    MAG_V2 = calReg[8];
    MAG_I2 = calReg[9];
    MAG_E2 = calReg[10];
    MAG_V3 = calReg[11];
    MAG_I3 = calReg[12];
    MAG_E3 = calReg[13];
    T_EFF = calReg[14];
    
    
    
    flaginc = 1;
}

/*********************************************************************
* @fn      zclSmartMeter_calibrateInc
*
* @brief   Called to calculate power
*
* @param   none
*
* @return  none
*/
static void zclSmartMeter_calibrateInc(void)
{
    if(flaginc == 0)
    {
        //do the power calculation incrementally
        senValueV[0] = analogRead(senPin1Voltage); //get ADC voltage value
        senValueI[0] = analogRead(senPin1Current); //get ADC current value
        realVol[0] = (int16)(zclSmartMeter_map((int16)senValueV[0], (int16)MIN_ADC, (int16)MAX_ADC, (int16)((float)(MIN_V) * sqrt(2)*(-1)), (int16)((float)MAX_V * sqrt(2)))); //map to real voltage
        realCur[0] = (int16)(zclSmartMeter_map((int16)senValueI[0], (int16)MIN_ADC, (int16)MAX_ADC, (int16)((float)(MIN_I * 100) * sqrt(2)*(-1) ), (int16)(MAX_I * 100 * sqrt(2)))); //map to real current, mag by MAG
        VrmsTemp[0] += realVol[0] * realVol[0]; //accumulate V^2
        IrmsTemp[0] += realCur[0] * realCur[0]; //accumulate I^2
       /*
        UARTprintf(" senValueV[0]: %d\n", senValueV[0]);
        UARTprintf(" senValueI[0]: %d\n", senValueI[0]);
        UARTprintf(" (int16)senValueV[0]: %d\n", (int16)senValueV[0]);
        UARTprintf(" (int16)MIN_ADC: %d\n", (int16)MIN_ADC);
        UARTprintf(" (int16)MAX_ADC: %d\n", (int16)MAX_ADC);
        UARTprintf(" MIN_V: %d\n", (int16)((float)(MIN_V) * sqrt(2)*(-1)));
        UARTprintf(" MAX_V: %d\n", (int16)((float)MAX_V * sqrt(2)));               
        UARTprintf(" realCur[0]: %d\n", realCur[0]);
        UARTprintf(" realVol[0]: %d\n", realVol[0]);
         */   
        //UARTprintf(" senValueI[0]: %d\n", senValueI[0]);
        //UARTprintf(" realCur[0]: %d\n", realCur[0]);
        //UARTprintf(" (int16)MIN_I: %d\n", (int16)MIN_I);
        //UARTprintf(" (int16)MAX_I: %d\n", (int16)MAX_I);
        
        if (CAL_OPT != CAL_VOL && CAL_OPT != CAL_CUR)
        {          
            if((l_nSamples%25) == 0 && (l_nSamples != 0))
            {
                flaginc = 1;        
                enecal_timenew = osal_GetSystemClock();
                enecal_timeperiod = enecal_timenew - enecal_timeold;                  
                enecal_timeold = enecal_timenew;
                
                if(CAL_OPT == CAL_ENE)
                    enecal_energy[0] = enecal_energy[0] + energyVal[0];
                
                energy_display[0] = energy_display[0] + energyVal[0];  //energy for display use
                //UARTprintf(" energy_display[0]: %d\n", (int)( energy_display[0]/100000 ));
                //UARTprintf(" energy_display[0]1: %d\n", (int)( energy_display[0]));
                
                energyVal[0] = 0;

                if(enecal_cycle == 0)
                {
                    enecal_timeperiod = 0;
                    enecal_energy[0] = 0;
                }
                enecal_cycle ++;
                
                if(CAL_OPT == CAL_ENE)
                {                    
                    UARTprintf(" enecal_timeperiod: %d\n", enecal_timeperiod);
                    UARTprintf(" enecal_energy[0]: %d\n", enecal_energy[0]);
                    UARTprintf(" enecal_cycle: %d\n", enecal_cycle);
                }
            }
        }
          
        
            
        //if(l_nSamples == ((uint32)(T_CAL * (200000 / SAMPLE_INT)) / N_CAL) && CAL_OPT == CAL_VOL) //200000 TO 1000000
        if(((uint16)((float)(time_new - time_old)*3.125)*N_CAL >= T_CAL *1000) && CAL_OPT == CAL_VOL)
        {
            UARTprintf(" VrmsTemp[0]: %d\n", VrmsTemp[0]);
            CRMS_V[0] = (uint16)sqrt(VrmsTemp[0] / l_nSamples); //get RMS voltage
            VrmsTemp[0] = 0;
            IrmsTemp[0] = 0;

            MAG_V1 += (uint16)(V_CAL * 100 / CRMS_V[0]);
            //UARTprintf(" senValueV[0]: %d\n", senValueV[0]);
            //UARTprintf(" senValueI[0]: %d\n", senValueI[0]);
            //UARTprintf(" realCur[0]: %d\n", realCur[0]);
            UARTprintf(" realVol[0]: %d\n", realVol[0]);
            UARTprintf(" MAG_V1: %d\n", MAG_V1);


            //time_new = osal_GetSystemClock();
           // UARTprintf(" (time_new - time_old)>>24: %x\n", (uint8)((time_new - time_old) >> 24));
           // UARTprintf(" (time_new - time_old)>>16: %x\n", (uint8)((time_new - time_old) >> 16));
           // UARTprintf(" (time_new - time_old)>>8: %x\n", (uint8)((time_new - time_old) >> 8));
           // UARTprintf(" (time_new - time_old): %x\n", (uint8)(time_new - time_old));
            UARTprintf(" l_nSamples: %d\n", l_nSamples);
            
            UARTprintf(" CRMS_V[0]: %d\n", CRMS_V[0]);            
            time_old = time_new;

            l_nSamples = 0;
            if (cal_index == N_CAL - 1)
            {
                MAG_V1 = MAG_V1 / N_CAL;
                UARTprintf(" MAG_V1FINAL: %d\n", MAG_V1);
                //zclSmartMeter_nvWriteParam();
                zclSmartMeter_calWriteParam();
                // send the calibration value over the air to Coordinator
                zclSmartMeter_SendCalibrate();
                cal_index = 0;
                CAL_OPT = 0;
            }
            else
                cal_index++;
        }

        //else if(l_nSamples == ((uint32)(T_CAL * (200000 / SAMPLE_INT)) / N_CAL) && CAL_OPT == CAL_CUR) ///////////////////////////200000 TO 1000000
        else if(((uint16)((float)(time_new - time_old)*3.125)*N_CAL >= T_CAL *1000) && CAL_OPT == CAL_CUR)
        {
            UARTprintf(" VrmsTemp[0]: %d\n", VrmsTemp[0]);
            CRMS_I[0] = (uint16)sqrt(IrmsTemp[0] / l_nSamples); //get RMS voltage
            VrmsTemp[0] = 0;
            IrmsTemp[0] = 0;

            //UARTprintf(" senValueI[0]: %d\n", senValueI[0]);
            UARTprintf(" realCur[0]: %d\n", realCur[0]);
            //UARTprintf(" senValueV[0]: %d\n", senValueV[0]);
            //UARTprintf(" realVol[0]: %d\n", realVol[0]);
            MAG_I1 += (uint16)(I_CAL * 10000 / CRMS_I[0]);
            UARTprintf(" MAG_I1: %d\n", MAG_I1);
            
           // time_new = osal_GetSystemClock();
            //UARTprintf(" (time_new - time_old)>>24: %x\n", (uint8)((time_new - time_old) >> 24));
           // UARTprintf(" (time_new - time_old)>>16: %x\n", (uint8)((time_new - time_old) >> 16));
           // UARTprintf(" (time_new - time_old)>>8: %x\n", (uint8)((time_new - time_old) >> 8));
           // UARTprintf(" (time_new - time_old): %x\n", (uint8)(time_new - time_old));
            UARTprintf(" l_nSamples: %d\n", l_nSamples);
            
            UARTprintf(" CRMS_I[0]: %d\n", CRMS_I[0]);
            time_old = time_new;

            l_nSamples = 0;
            if (cal_index == N_CAL - 1)
            {
                MAG_I1 = MAG_I1 / N_CAL;
                UARTprintf(" MAG_I1FINAL: %d\n", MAG_I1);
                zclSmartMeter_calWriteParam();
                // send the calibration value over the air to Coordinator
                zclSmartMeter_SendCalibrate();
                cal_index = 0;
                CAL_OPT = 0;
            }
            else
                cal_index++;
        }

        //else if(l_nSamples == ((uint32)(T_CAL * (200000 / SAMPLE_INT)) / N_CAL) && CAL_OPT == CAL_ENE) /////////////////////////200000 TO 1000000
        else if(((uint16)((float)(time_new - time_old)*3.125) >= T_CAL *1000) && CAL_OPT == CAL_ENE)
        {
            //CRMS_V[0] = (uint16)sqrt(VrmsTemp[0] / l_nSamples); //get RMS voltage
            //CRMS_I[0] = (uint16)sqrt(IrmsTemp[0] / l_nSamples); //get RMS current
            //VrmsTemp[0] = 0;
            //IrmsTemp[0] = 0;
            

            MAG_E1 = (uint16)((V_CAL * I_CAL * T_CAL * 10000) / (enecal_energy[0]/1000));
            enecal_energy[0] = 0;
            UARTprintf(" MAG_E1: %d\n", MAG_E1);            
            UARTprintf(" 0000: %d\n", (V_CAL * I_CAL * T_CAL * 10000)); 
            
            UARTprintf(" l_nSamples: %d\n", l_nSamples);
            UARTprintf(" enecal_cycle: %d\n", enecal_cycle);
            UARTprintf(" enecal_timeperiod: %d\n", enecal_timeperiod);
            time_old = time_new;
            l_nSamples = 0;

            E_CAL = MAG_V1 * MAG_I1 * T_CAL;           
            T_EFF = enecal_timeperiod * 1000;
            zclSmartMeter_calWriteParam();
            // send the calibration value over the air to Coordinator
            zclSmartMeter_SendCalibrate();

            CAL_OPT = 0;
        }
        else
            time_new = osal_GetSystemClock();
        
        l_nSamples += 1;
    }
    else
    {
        flaginc = 0;
        //calculate the power and energy
        RMS_V1 = (uint16)((sqrt(VrmsTemp[0] / l_nSamples) * MAG_V1) / 100); //get RMS voltage
        RMS_I1 = (uint16)((sqrt(IrmsTemp[0] / l_nSamples) * MAG_I1) / 100); //get RMS current
        powerVal[0] = RMS_V1 * RMS_I1;                     //get power
        energyVal[0] += (int32)(( powerVal[0] * enecal_timeperiod * 3.125 ) * MAG_E1 / 100);  //energy in W.s  * 100000
        //Energy[0] = (double)((double)energyVal[0] / 1000 / 3600 / 100000); //energy magnified in kWh
        Energy[0] = (double)((double)energy_display[0] / 1000 / 3600 / 100000); //energy magnified in kWh
        
        //zclSmartMeter_UpdateDataReg();
        
        CurDisplay[0] = (float)((float)(RMS_I1 )/ 100);
        PowerDisplay[0] = (float)((float)(powerVal[0]) / 100);
        
        //UARTprintf("powerVal[0] %d\n", powerVal[0]);
        //UARTprintf("energyVal[0] %d\n", energyVal[0]);
        //UARTprintf("MAG_V1 %d\n", MAG_V1);
        //UARTprintf("MAG_I1 %d\n", MAG_I1);
        //UARTprintf("MAG_E1 %d\n", MAG_E1);
        //UARTprintf(" l_nSamples: %d\n", l_nSamples);
        VrmsTemp[0] = 0;
        IrmsTemp[0] = 0;
        l_nSamples = 0;
    }


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
int32 zclSmartMeter_map(int16 senValue, int16 MIN_ADC, int16 MAX_ADC, int16 MIN_PEAK, int16 MAX_PEAK)
{
      return((int32)((float)(senValue *(MAX_PEAK - MIN_PEAK))/ MAX_ADC) + MIN_PEAK);
}
/*********************************************************************
* @fn      zclSmartMeter_updata dataReg
*
* @brief   Called to updata data register
*
* @param   none
*
* @return  none
*/
static void zclSmartMeter_UpdateDataReg( void )
{
    uint16 sm_ADD3 = (sm_ADD >> 48) & 0xFFFF;
    uint16 sm_ADD2 = (sm_ADD >> 32) & 0xFFFF;
    uint16 sm_ADD1 = (sm_ADD >> 16) & 0xFFFF;
    uint16 sm_ADD0 = sm_ADD & 0xFFFF;
    uint16 sm_ADD16 = sm_nwkADD;

    dataReg[0] = sm_ADD3;
    dataReg[1] = sm_ADD2;
    dataReg[2] = sm_ADD1;
    dataReg[3] = sm_ADD0;
    dataReg[4] = sm_ADD16;
    dataReg[5] = RMS_V1;
    dataReg[6] = RMS_I1;
    dataReg[7] = THETA_1;
    dataReg[8] = RMS_V2;
    dataReg[9] = RMS_I2;
    dataReg[10] = THETA_2;
    dataReg[11] = RMS_V3;
    dataReg[12] = RMS_I3;
    dataReg[13] = THETA_3;
    dataReg[20] = SM_V;
    dataReg[21] = SM_I;    
    dataReg[22] = STATUS;

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
    // Set the clocking to run directly from the external crystal/oscillator.
    // (no ext 32k osc, no internal osc)
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);


    // Set IO clock to the same as system clock


    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
    // Enable RF Core,  requested by ADC
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);


    // Configure ADC, Internal reference, 512 decimation rate (12bit)
    SOCADCSingleConfigure(SOCADC_12_BIT, SOCADC_REF_INTERNAL);
    // Initialize the GPIO pin configuration.
    // Set pins AIN6 and AIN7 as GPIO input for ADC input using, SW controlled.


    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_7);
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
uint16 analogRead(uint32 senPinValue)
{
    // Trigger single conversion on AINPIN to get VoltageValue
    SOCADCSingleStart(senPinValue);
    //
    // Wait until conversion is completed
    while(!SOCADCEndOfCOnversionGet())
    {
    }
    // Get data and shift down based on decimation rate
    return (SOCADCDataGet() >> 5);
    //return (SOCADCDataGet());
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
    for (i = 0; i < 13; i++)
        dataReg[i] = 0;
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
    return rand() % 100;
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
    // Clear local LCD buffers
    lcdBufferClear(0);
    lcdBufferClear(secondLcdBuf);
    // Write default buffer
    lcdBufferPrintString(0, "Smart Meter", 1, eLcdPage0);
    lcdBufferInvertPage(0, 0, 127, eLcdPage0);
    //Write RMS_V to default buffer
    lcdBufferPrintStringAligned(0, "Voltage:", eLcdAlignLeft, eLcdPage2);
    lcdBufferPrintIntAligned(0, RMS_V1, eLcdAlignCenter, eLcdPage2);
    lcdBufferPrintStringAligned(0, "V", eLcdAlignRight, eLcdPage2);
    //Write RMS_I to default buffer
    lcdBufferPrintStringAligned(0, "Current:", eLcdAlignLeft, eLcdPage3);
    lcdBufferPrintFloatAligned(0, CurDisplay[0], 2, eLcdAlignCenter, eLcdPage3);
    lcdBufferPrintStringAligned(0, "A", eLcdAlignRight, eLcdPage3);
    //Write powerVal to default buffer
    lcdBufferPrintStringAligned(0, "Power:", eLcdAlignLeft, eLcdPage4);
    lcdBufferPrintFloatAligned(0, PowerDisplay[0], 2, eLcdAlignCenter, eLcdPage4);
    lcdBufferPrintStringAligned(0, "W", eLcdAlignRight, eLcdPage4);
    //Write EnergyVal to default buffer
    lcdBufferPrintStringAligned(0, "Energy:", eLcdAlignLeft, eLcdPage5);
    lcdBufferPrintFloatAligned(0, Energy[0], 6, eLcdAlignCenter, eLcdPage5);
    lcdBufferPrintStringAligned(0, "kWh", eLcdAlignRight, eLcdPage5);
    //Write school name
    lcdBufferPrintStringAligned(0, "ITU", eLcdAlignCenter, eLcdPage7);
    // Send the default buffer to LCD
    lcdSendBuffer(0);
}
/***************************************************************************
* This function sets up UART0 to be used for a console to display information
* for debugging purpose.
*
* adopted from GuoXu's code
*****************************************************************************/
void InitConsole(void)
{
    // Map UART signals to the correct GPIO pins and configure them as
    // hardware controlled.
    IOCPinConfigPeriphOutput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_TXD,
                             IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_TXD);
    IOCPinConfigPeriphInput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_RXD,
                            IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_RXD);
    // Initialize the UART (UART0) for console I/O.
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
#ifdef ZCL_REPORT
        // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_REPORT:
        zclSmartMeter_ProcessInReportCmd( pInMsg );
        break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
        zclSmartMeter_ProcessInDefaultRspCmd( pInMsg );
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
    // Device is notified of the Default Response command.
    (void)pInMsg;
    return ( TRUE );
}


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
        osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, 2000 );
    }
}
#endif // ZCL_EZMODE
/****************************************************************************
****************************************************************************/
