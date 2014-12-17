/**************************************************************************************************
  Filename:       zcl_Coordinator.c
  Revised:        $Date: 2014-12-15 17:02:21 -0700 
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

//#define COORDINATOR_PIN_UART_RXD            GPIO_PIN_0
//#define COORDINATOR_PIN_UART_TXD            GPIO_PIN_1
//#define COORDINATOR_GPIO_UART_BASE          GPIO_A_BASE


//Michelle: for UART settings
//*****************************************************************************

#define EXAMPLE_PIN_UART_RXD            GPIO_PIN_0
#define EXAMPLE_PIN_UART_TXD            GPIO_PIN_1
#define EXAMPLE_GPIO_BASE               GPIO_A_BASE


//*****************************************************************************

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
#include "hal_uart.h"

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

// Michelle
#include "hw_ints.h"
#include "hw_types.h"
#include "uart.h"

#include "MT.h"
#include "MT_UART.h"
#include "OSAL_Clock.h"
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
#define TIME_SET     0xD0
#define GET_CALPARAM 0xD1

#define FLASH_PARAM  0x1000


// wait time for all smart meter to respond to network discovery command
#define Coordinator_NwkDiscov_INTERVAL 5000 //in millisecond
// wait time to request the next smart meter to send data
#define Coordinator_SendData_INTERVAL  1000 //in millisecond

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclCoordinator_TaskID;
uint8 zclCoordinatorSeqNum;

uint16 paramReg[25] = {0};

uint16 SmartMeterparamReg[25]; //parameters send by SmartMeter
uint16 SmartMeterTimeReg[10]; //Time send by SmartMeter
uint16 dataReg_Ping[23];
uint16 dataReg_Pong[23];
uint8 dataRegSel = 1;
uint8 PowSpCal = 0;
uint64 sm_ADD[100] = {0}; //smart meter address registers --350 max
uint16 sm_ADD_0[100] = {0};
uint16 sm_ADD_1[100] = {0};
uint16 sm_ADD_2[100] = {0};
uint16 sm_ADD_3[100] = {0};
uint8 sm_ADD_status[100];
uint16 SM_ADD16 = 0;

uint64 sm_ADD_reg = 0;
uint8 V_CAL = 0;
uint8 I_CAL = 0;
uint8 T_CAL = 0;
uint8 N_CAL = 0;

UTCTimeStruct TimeStruct;
uint32 sys_secold = 0;
uint32 sys_secnew = 0;
uint32 sys_timeold = 0;
uint32 sys_timenew = 0;

uint16 sm_id;
uint16 sm_index = 0;  //index for sm_Add[index]
uint16 index;  //general purpose index
uint16 sm_max = 0; //total number of smart meter
uint8 controlReg[35] = {0};
uint8 parameter_in[60] = {0};

uint16 timeReg[6] = {0};
uint8 sm_receive_flag[100] = {0};
uint8 sm_retry_times[100] = {3};
uint8 relay_receive_flag = 0;
uint8 relay_retry_times = 3;
uint16 ack_index = 0;
uint8 NetDiscov_flag = 0;
uint8 Routingtable_flag = 0;
uint16 Drr_retry_cnt = 0;
uint8  Drr_flag = 1;
uint8  ACK_flag = 0;
uint8  Timeout_Ping = 0;
uint8  Timeout_Pong = 0;
uint16 Drr_Retry_Zero = 0;
uint16 first_write_flag = 0;  //write both ping and pong with SM1 and SM2
uint16 first_complete = 0;
uint8  Timeout_bit = 0;

uint8 cal_receive_flag = 0;

uint32 ENERGY_RESET_VALUE = 0xffffffff;
uint32 SmartMeter_ENERGY_RESET_VALUE = 0xfffffffe; //smart meter response to RESET
uint64 RM_ADD;
uint16 flagreset = 0;
uint16 flagrelay = 2;
uint16 flaginc;
uint16 datain_complete = 0; //read of smart meter data completed
uint16 SmartMeter_flagreset; //smart meter response to RESET
uint16 SmartMeter_relay = 0;  //smart meter response to RELAY
uint16 SmartMeter_flaginc; //smart meter response to RESTART
uint8 Msg_in[120] = {0}; //for save UART DATA
uint8 Round_end_flag;

uint16 MIN_ADC; 
uint16 MAX_ADC; 
uint16 SAMPLE_INT;
uint16 SAMPLE_WIN;
uint16 MAG_V1;
uint16 MAG_I1;
uint16 MAG_V2;
uint16 MAG_I2;
uint16 MAG_V3;
uint16 MAG_I3;
uint16 MIN_V;
uint16 MAX_V;
uint16 MIN_I;
uint16 MAX_I;
uint16 T_EFF;
uint16 SHARP1;
uint16 SHARP2;
uint16 PEAK1;
uint16 PEAK2;
uint16 PEAK3;
uint16 SHOULDER1;
uint16 SHOULDER2;
uint16 SHOULDER3;
uint16 OFF;
uint16 N_SM;
uint16 YEAR;
uint16 MONTH;
uint16 DAY;
uint16 HOUR;
uint16 MINUTE;
uint16 SECOND;

uint16 calsm_id;
uint16 calSMADD_3;
uint16 calSMADD_2;
uint16 calSMADD_1;
uint16 calSMADD_0;
uint16 calMAG_V1;
uint16 calMAG_I1;
uint16 calMAG_V2;
uint16 calMAG_I2;
uint16 calMAG_V3;
uint16 calMAG_I3;
uint16 calT_EFF;

uint8 dataLen;  //length of the packdge
uint8 flag_calget = 0;


uint16 ADD_3;
uint16 ADD_2;
uint16 ADD_1;
uint16 ADD_0;

extern mtOSALSerialData_t  *pMsg;
/* Define memory mapping*/
uint16 *pparamReg;  //pointer to paramReg

uint16 *pdataReg_Ping;  //pointer to dataReg_Ping

uint16 *pdataReg_Pong;  //pointer to dataReg_Pong

uint8 *pcontrolReg; //pointer to controlReg

uint64 coordinator_extAddr;   //coordinator external IEEE address
uint16 coordinator_nwkAddr;  //coordinator network address

uint8 *pcoordinator_extAddr; //pointer to coordinator external IEEE address

uint64 *psm_ADD; //pointer to smartmeter external IEEE addresses in routing table

uint16 test_i = 1;

int sm_flag_and = 1;

uint8 pack_out[100] = {0};

uint32 time_old = 0;

uint32 time_new = 0;

uint8 cmd_right_flag = 1;
/*********************************************************************
 * GLOBAL FUNCTIONS
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
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    //data attribute for SmartMeter defined in zcl_SmartMeter_data_c
    ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,  // added for SmartMeter
    ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,  // added for SmartMeter
    ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,  // added for SmartMeter
    ZCL_CLUSTER_ID_MS_COM_MEASUREMENT  // added for SmartMeter
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

static void zclscoordinator_startEZmode( void );


static void zclCoordinator_NetDiscov( void );

// app display functions
void zclCoordinator_LCDDisplayUpdate(void);
void zclCoordinator_LcdDisplayTestMode(void);
void zclCoordinator_LcdDisplayTestMode_smaddr(void);

//Coordinator functions
static void zclCoordinator_SendData( void );
//static void zclCoordinator_SendParam( void );
static void zclCoordinator_SetParam( void );
static void zclCoordinator_SetTime();
static void zclCoordinator_nvWriteParam( void );
static void zclCoordinator_nvReadParam( void );
static void zclCoordinator_SendAck( void );
static void zclCoordinator_SendRestart( void );
static void zclCoordinator_SendReset(void);
static void zclCoordinator_SendRelay(void);
static void zclCoordinator_SendCalibrate(void);
static void zclCoordinator_sendcalreg(void);
void zclCoordinator_parameterInit(void);
void zclCoordinator_controlRegInit(void);
uint8 zclCoordinator_SmartMeterParamCompare(void);
uint8 zclCoordinator_SmartMeterTimeCompare(void);
uint8 zclCoordinator_ProcessInGetParamReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInSetParamReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInGetDataReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInAddReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInResetReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInRestartReportCmd( zclIncomingMsg_t *pInMsg );
void zclCoordinator_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
uint8 zclCoordinator_ProcessInRelayReportCmd( zclIncomingMsg_t *pInMsg );
void zclCoordinator_dataRegInit (void);
void zclCoordinator_sendACK(uint8 ControlReg0, uint8 ControlReg1, uint8 ControlReg2); //send ACK to local server when command is done successfully
//void zclCoordinator_sendRetry(void);//send Retry to local server when transmit error
void zclCoordinator_ReadRoutingTable(uint8 sm_i);
static void zclCoordinator_getcalParam(void);
static void zclCoordinator_calregtimeout(void);




uint32 BUILD_UINT32_16 (uint16 num1, uint16 numb2);
uint64 BUILD_UINT64_8 (uint8 numb1, uint8 numb2, uint8 numb3, uint8 numb4, uint8 numb5, uint8 numb6, uint8 numb7, uint8 numb8);
uint64 BUILD_UINT64_16 (uint16 numb1, uint16 numb2, uint16 numb3, uint16 numb4);

static void initUART(void); //Use UART.c functions directly
int16_t uint16ToInt16(uint16_t u_i);
uint16_t int16ToUint16(int16_t i);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclCoordinator_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclCoordinator_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg ); //removed
#endif
#ifdef ZCL_WRITE
static uint8 zclCoordinator_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg ); //removed
#endif
#ifdef ZCL_REPORT
static void zclCoordinator_ProcessInReportCmd( zclIncomingMsg_t *pInMsg );
#endif  // ZCL_REPORT
static uint8 zclCoordinator_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg ); //removed

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]     = " ";
const char sDeviceName[]    = "     Coordinator";
const char sSwEZMode[]      = "SW2: EZ-Mode";
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

    HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_5 );
#endif

    //MT Uart Initial
    MT_UartInit();
    MT_UartRegisterTaskID(zclCoordinator_TaskID);



    initUART();  //Michelle: Use UART funcitons directly

    //Initialize controlReg in the coordinator
    zclCoordinator_controlRegInit();
    //Initialize parameters in the coordinator
    zclCoordinator_parameterInit();
    //Initialize SmartMeter data register
    zclCoordinator_dataRegInit();

    zclscoordinator_startEZmode(); //lhy

    // Get coordinator 64-bit IEEE external address and network address
    pcoordinator_extAddr = saveExtAddr; //ZDApp.h uint8 saveExtAddr[];
    coordinator_nwkAddr = NLME_GetShortAddr();

    ADD_3 = (uint16)saveExtAddr[6] + ((((uint16)saveExtAddr[7]) << 8) & 0xFF00); //lhy
    ADD_2 = (uint16)saveExtAddr[4] + ((((uint16)saveExtAddr[5]) << 8) & 0xFF00);
    ADD_1 = (uint16)saveExtAddr[2] + ((((uint16)saveExtAddr[3]) << 8) & 0xFF00);
    ADD_0 = (uint16)saveExtAddr[0] + ((((uint16)saveExtAddr[1]) << 8) & 0xFF00);
    coordinator_extAddr = ((uint64)ADD_0 & 0xFFFF) + ((((uint64)ADD_1) << 16) & 0xFFFF0000) + ((((uint64)ADD_2) << 32) & 0xFFFF00000000) + ((((uint64)ADD_3) << 48) & 0xFFFF000000000000);


    //Initialize routing table sm_ADD[index]
    for (sm_index = 0; sm_index < 100; sm_index++)
    {
        sm_ADD[sm_index] = 0;
    }
    sm_index = 0;

    controlReg[0] = 0x08;
    controlReg[1] = 0x03;
    controlReg[2] = 0x00;
}
/********************************************************
//send ACK to local server when command is done successfully
void zclCoordinator_sendACK(void)
**********************************************************/

void zclCoordinator_sendACK(uint8 ControlReg0, uint8 ControlReg1, uint8 ControlReg2)
{
    int i;
    pack_out[0] = 0x68;

    for(i = 1; i <= 8; i++)
        pack_out[i] = 0;

    pack_out[9] = 0x68;
    pack_out[10] = 0x03;

    controlReg[0] = ControlReg0;
    controlReg[1] = ControlReg1;
    controlReg[2] = ControlReg2;
    pack_out[11] = ControlReg0;
    pack_out[12] = ControlReg1;
    pack_out[13] = ControlReg2;

    for(i = 0; i < 14; i++)
        pack_out[14] += pack_out[i];

    pack_out[15] = 0x16;

    UARTEnable(UART0_BASE );
    for (i = 0; i < 16; i++)
    {
        UARTCharPut(UART0_BASE, pack_out[i]);
    }
    UARTDisable(UART0_BASE);
}

/**************************************************
ProcessUartData()

********************************************************/
void ProcessUartData( mtOSALSerialData_t *Uart_Msg)
{

    int index;

    dataLen = Uart_Msg->msg[0];

    Msg_in[0] = 0x68;
    for (index = 1; index <= 8; index++)
        Msg_in[index] = 0x00;
    Msg_in[9] = 0x68;

    if(dataLen == 0x50)
        Msg_in[10] = 0x1F;
    else
        Msg_in[10] = dataLen;


    Msg_in[11] = Uart_Msg->msg[1] ;

    for (index = 1; index < (int)(dataLen + 2); index++)
    {
        Msg_in[index + 11] = Uart_Msg->msg[index + 1] ;
    }

    HalLcdWriteString( "processUARTDATA", HAL_LCD_LINE_7 );
    if(dataLen == 0x50)
        dataLen = 0x1F;
}

/*********************************************************************
 * @fn          zclCoordinator_event_loop
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

    int16_t value;

    uint16_t u_value;
    uint8_t u_partA;
    uint8_t u_partB;
    char  lcdString[10];
    /*
    #ifdef LCD_SUPPORTED
        sprintf((char *)lcdString, "test_i: %d", (uint8)(test_i & 0xFF) );
        HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
        test_i++;
    #endif
    */


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
                    zclCoordinator_LCDDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
                    zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif  // ZCL_EZMODE


                }
                break;

            case CMD_SERIAL_MSG:                                           //ZcomDef.h 10.8
                ProcessUartData((mtOSALSerialData_t *)pMsg);
                osal_set_event(task_id, Coordinator_UART_EVT);
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





    //start to process data from UART
    int i;
    for(i = 0; i < 100; i++)
        pack_out[i] = 0;

    uint16 checksum = 0;

    if(Msg_in[0])   //if there is message in
    {

        int index;
        checksum = 0;

        if(dataLen == 3)
        {


            for (index = 0; index < 14; index++)
            {
                checksum += Msg_in[index];
            }

            if((uint8)(checksum & 0xFF) != Msg_in[14] || Msg_in[15] != 0x16)    //if sum is wrong or end flag is not 0x16
            {
                zclCoordinator_sendACK(0x08, 0x03, 0x05);
                cmd_right_flag = 0;
            }
        }
        else
        {
            for (index = 0; index < 42; index++)
            {
                checksum += Msg_in[index];
            }

            if((uint8)(checksum & 0xFF) != Msg_in[42] || Msg_in[43] != 0x16)    //if sum is wrong or end flag is not 0x16
            {
                zclCoordinator_sendACK(0x08, 0x03, 0x05);
                cmd_right_flag = 0;
            }

            else if (((Msg_in[11] >> 1) & 0x01) == 1)             //if success in reading controlReg and the command is set parameter,then continue to read the parameter
            {

                int i;
                checksum = 0;
                for (index = 0; index < 47; index++)
                {
                    checksum += Msg_in[index + 44];
                }


                if((uint8)(checksum & 0xFF) != Msg_in[91] || Msg_in[92] != 0x16)  //if sum is wrong or end flag is not 0x16
                {
                    zclCoordinator_sendACK(0x08, 0x03, 0x05);
                    cmd_right_flag = 0;
                }

                else   //if no problem with the package, get parameter out
                {
                    for(i = 0; i < 47; i++)
                        parameter_in[i] = Msg_in[i + 44];
                }
            }
        }
    }



    if  ((events & Coordinator_UART_EVT ) && cmd_right_flag)

    {

        // command: control register write
        // Write to controlReg when controlReg write is disabled  *
        // This is the normal write operation from server

        if ((controlReg[1] & 0x01) == 1)
        {
            int i;

            for(i = 0 ; i < dataLen; i++)
            {
                controlReg[i] = Msg_in[i + 11];
            }

            for(i = 0; i < 100; i++)
                Msg_in[i] = 0;

        }

        // command: parameter read
        // Send current parameter to server if parameter read is enabled (bit 0=1)  *
        // Reset this parameter to its default state when operation is finished.


        if ((controlReg[0] & 0x01) == 1)

        {
            controlReg[1] = controlReg[1] & 0xFD;// disable write

            pack_out[0] = 0x68;
            int i;
            for(i = 1; i <= 8; i++)
                pack_out[i] = 0;

            pack_out[9] = 0x68;
            pack_out[10] = 0x27;

            for (i = 0; i < 18; i++)
            {

                //int16_t to uint16_t first
                value = paramReg[i];
                u_value = int16ToUint16(value);

                // uint16_t into two uint8_t first
                u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                u_partB = (uint8_t) (u_value & 0x00FF);

                pack_out[11 + i * 2] = u_partA;
                pack_out[12 + i * 2] = u_partB;
            }

            controlReg[0] = controlReg[0] & 0xFE; //reset bit 0 to 0
            controlReg[1] = controlReg[1] | 0x03;

            pack_out[47] = controlReg[0];
            pack_out[48] = controlReg[1];
            pack_out[49] = controlReg[2];

            pack_out[50] = 0;
            for(i = 0; i < 50; i++)
                pack_out[50] += pack_out[i];

            pack_out[51] = 0x16;

            UARTEnable(UART0_BASE );
            for (i = 0; i < 52; i++)
            {
                UARTCharPut(UART0_BASE, pack_out[i]);
            }
            UARTDisable(UART0_BASE);

        }

        // command: parameter write
        // Server write to paramReg if parameter write is enable (bit 1=0)  *
        // This operation will also write into FLASH or coordinator and smart meters
        // Reset this bit to default when write operation is completed


        else if (((controlReg[0] >> 1) & 0x01) == 1)

        {

            int i;

            for (i = 0; i < 18; i++)
            {
                paramReg[i] = (uint16)(((uint16)(parameter_in[i * 2 + 11])) << 8) + (uint16)(parameter_in[i * 2 + 12]);
            }

            //write parameters into FLASH memory of coordaintor
            zclCoordinator_nvWriteParam();

            //write parameters to all the smart meters
            for (sm_id = 0; sm_id < sm_max; sm_id++)
            {
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_id]) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_id]) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_id]) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_id]) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_id]) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_id]) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_id]) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_id]) & 0x00000000000000FF);

                zclCoordinator_SetParam();
            }

            osal_set_event(task_id, Coordinator_ProcessParaSet_EVT);

            time_old = osal_GetSystemClock();
            time_new = time_old;

        }


        // command: energy calculation reset
        // Reset energy calculation if bit 2 is 1   *
        // Reset this bit to its default value when operation is finished


        else if (((controlReg[0] >> 2) & 0x01) == 1)
        {
            int i;
            for(i = 0; i < 8; i++)
                zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[14 - i];

            flagreset = 1;

            ENERGY_RESET_VALUE = BUILD_UINT32(controlReg[6], controlReg[5], controlReg[4], controlReg[3]);

            zclCoordinator_SendReset();

            osal_set_event(task_id, Coordinator_EnergyResetWait_EVT);
            time_old = osal_GetSystemClock();
            time_new = time_old;


        }


        // command: relay control
        // Switch smart meter power condition if bit 3 is set to 0  *
        // Reset this bit to its default value when operation is finished
        //else if (((controlReg[0] >> 3) & 0x01) == 0)  // power relay control


        else if ((controlReg[0] & 0xF7) == 0 && controlReg[2] == 0x00 && controlReg[1] == 0x02)  // except bit3, other bits are all 0, and controlReg[2] is 0x00
        {
            int i;

            for(i = 0; i < 8; i++)
                zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[14 - i];

            if (((controlReg[0] >> 3) & 0x01) == 0)  // power relay off (disconnect line power)
            {
                flagrelay = 0;
                zclCoordinator_SendRelay();

                osal_set_event(task_id, Coordinator_RelaySet_EVT);
            }

            else // power relay on (connect to line power)
            {
                flagrelay = 1;
                zclCoordinator_SendRelay();


                osal_set_event(task_id, Coordinator_RelaySet_EVT);
            }

            time_old = osal_GetSystemClock();
            time_new = time_old;

        }


        // command: network discovery
        // Perform network discovery if bit 4 is 1           *
        //  Reset this bit to its default value when operation is finished


        else if (((controlReg[0] >> 4) & 0x01) == 1)
        {
            //Routingtable_flag = 0;
            zclCoordinator_NetDiscov();	       //sent a broadcast to all smart meter

            osal_set_event(task_id, Network_WAIT_EVT);
            time_old = osal_GetSystemClock();
            time_new = time_old;
            int i;
            for(i = 0; i < 100; i++)
                sm_ADD_status[i] = 1;

        }


        // command: routing table read
        // Send routing table when bit 5 is set to 1                 *
        // Reset this bit to its default value when operation is finished

        // else if ((((controlReg[0] >> 5) & 0x01) == 1) && (Routingtable_flag == 0))
        else if (((controlReg[0] >> 5) & 0x01) == 1)
        {

            if(sm_max == 1)
            {

                if(controlReg[2] == 0)

                {
                    sm_index = 0;
                    controlReg[0] = 0x08;
                    controlReg[1] = 0x03;
                    controlReg[2] = 0x00;


                    zclCoordinator_ReadRoutingTable(sm_index);
                }

            }
            else
            {

                if((controlReg[2] >> 3) & 0x01 == 1)
                {

                    if(sm_index < sm_max - 1)
                    {
                        controlReg[0] = 0x28;
                        controlReg[1] = 0x03;
                        controlReg[2] = 0x00;
                        zclCoordinator_ReadRoutingTable(sm_index);   //if not finish, continue to send the next one
                    }

                    else if(Routingtable_flag)
                    {
                        Routingtable_flag = 0;                      //set Routingtable_flag to 0 after complete read the routing table.
                        controlReg[0] = 0x08;       //reset bit 5 to 0
                        controlReg[1] = 0x03;
                        controlReg[2] = 0x00;
                        zclCoordinator_ReadRoutingTable(sm_index);
                    }
                }

                else
                {
                    Routingtable_flag = 1;
                    sm_index = 0;                                    //the first one

                    controlReg[0] = 0x28;
                    controlReg[1] = 0x03;
                    controlReg[2] = 0x00;
                    zclCoordinator_ReadRoutingTable(sm_index);


                }
            }
        }


        // command: control register read
        // Send current control register to server if controlReg read is enabled (bit 6=1) *
        // Reset this parameter to its default state when operation is finished.


        else if (((controlReg[0] >> 6)  & 0x01) == 1)
        {

            //controlReg[1] = controlReg[1] & 0xFB; //reset byte 1 bit 0 to 0, controlReg not writable

            pack_out[0] = 0x68;
            int i;
            for(i = 1; i <= 8; i++)
                pack_out[i] = 0;

            pack_out[9] = 0x68;
            pack_out[10] = 0x22;

            for(i = 11; i < 42; i++)
                pack_out[i] = controlReg[i - 11];

            controlReg[0] = controlReg[0] & 0xBF; //reset bit 6 to 0
            controlReg[1] = controlReg[1] | 0x01; //reset byte1 bit 0 to 1

            pack_out[42] = controlReg[0];
            pack_out[43] = controlReg[1];
            pack_out[44] = controlReg[2];
            pack_out[45] = 0;
            for(i = 0; i < 45; i++)
                pack_out[45] += pack_out[i];

            pack_out[46] = 0x16;


            UARTEnable(UART0_BASE );
            for (i = 0; i < 47; i++)
            {
                UARTCharPut(UART0_BASE, pack_out[i]);
            }
            UARTDisable(UART0_BASE);
        }


        // command: Timeset
        else if (((controlReg[1] >> 5) & 0x01) == 1)
        {
            int i;

            for(i = 0; i < 6; i++)
            {
                timeReg[i] = (controlReg[i * 2 + 15] << 8) + controlReg[i * 2 + 16];
            }

            //write parameters to all the smart meters
            for (sm_id = 0; sm_id < sm_max; sm_id++)
            {
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_id]) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_id]) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_id]) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_id]) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_id]) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_id]) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_id]) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_id]) & 0x00000000000000FF);

                zclCoordinator_SetTime();
            }

            sys_timeold = osal_GetSystemClock();
            sys_timenew = sys_timeold;
            TimeStruct.seconds = (uint8)timeReg[5];
            TimeStruct.minutes = (uint8)timeReg[4];
            TimeStruct.hour = (uint8)timeReg[3];
            TimeStruct.day = (uint8)timeReg[2];
            TimeStruct.month = (uint8)timeReg[1];
            TimeStruct.year = (uint16)timeReg[0];
            sys_secold = osal_ConvertUTCSecs( &TimeStruct );

            osal_set_event(task_id, Coordinator_ProcessParaSet_EVT);

            time_old = osal_GetSystemClock();
            time_new = time_old;

        }

        //command: calregister read
        else if (((controlReg[1] >> 6) & 0x01) == 1)
        {
            int i;

            for(i = 0; i < 8; i++)
                zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[14 - i];
            flag_calget = 0;

            zclCoordinator_getcalParam();

            osal_set_event(task_id, GETCAL_WAIT_EVT);

            time_old = osal_GetSystemClock();
            time_new = time_old;
        }

        //command: data register read
        //Send request for smart meter data using round robin method
        //Specify which smart meter to request data
        //&zclCoordinator_DstAddr = &sm_ADD[sm_index];


        else if (((controlReg[0] >> 7) & 0x01) == 1 && Drr_flag == 1 && (((controlReg[2] >> 3) & 0x01) == 0))

        {
            HalLcdWriteString( "DRREVT", HAL_LCD_LINE_7 );
            ack_index = 0;
            datain_complete = 0;
            first_write_flag = 1;
            // &zclCoordinator_DstAdfdr = psm_ADD;
            while((sm_ADD_status[ack_index] == 0) && (ack_index < (sm_max - 1)))
            {
                ack_index++;
            }

            zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[ack_index]) >> 56) & 0x00000000000000FF); //AF.h; highest
            zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[ack_index]) >> 48) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[ack_index]) >> 40) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[ack_index]) >> 32) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[ack_index]) >> 24) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[ack_index]) >> 16) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[ack_index]) >> 8) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[ack_index]) & 0x00000000000000FF);

            //Stop power calculation
            flaginc = 0;
            zclCoordinator_SendRestart();
            //Send request to smart meter to send data

            //zclCoordinator_SendData();   //send request to get data

            osal_set_event(task_id, DATA_WAIT_EVT);
            ACK_flag = 1;

            time_old = osal_GetSystemClock();
            time_new = time_old;
            Drr_flag = 0;

        }


        else if((((controlReg[2] >> 3) & 0x01) == 1) && (((controlReg[0] >> 7) & 0x01) == 1) && ACK_flag == 1)  //ACKcommand
        {
            if((controlReg[2] & 0x01) == 1)//check is retry or not!
            {
                dataRegSel = dataRegSel ^ 0x01;
                osal_set_event(task_id, DATA_CL_EVT);
            }

            else
            {
                if(((controlReg[1] >> 1) & 0x01) == 0)
                {
                    PowSpCal = 1;

                }

                osal_set_event(task_id, ACK_WAIT_EVT);

            }
        }


        //voltage calibration command
        else if  (((controlReg[1] >> 2) & 0x01) == 1 )
        {

            int i;
            for(i = 0; i < 8; i++)
                zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[14 - i];
            zclCoordinator_SendCalibrate();

            osal_set_event(task_id, Calibration_WAIT_EVT);
            time_old = osal_GetSystemClock();
            time_new = time_old;
        }


        //current calibration command
        else if  (((controlReg[1] >> 3) & 0x01) == 1 )
        {

            int i;
            for(i = 0; i < 8; i++)
                zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[14 - i];
            zclCoordinator_SendCalibrate();

            osal_set_event(task_id, Calibration_WAIT_EVT);
            time_old = osal_GetSystemClock();
            time_new = time_old;

        }
        //engergy calibration command
        else if  (((controlReg[1] >> 4) & 0x01) == 1 )
        {
            int i;
            for(i = 0; i < 8; i++)
                zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[14 - i];
            zclCoordinator_SendCalibrate();

            osal_set_event(task_id, Calibration_WAIT_EVT);
            time_old = osal_GetSystemClock();
            time_new = time_old;

        }


        return ( events ^ Coordinator_UART_EVT );

    }

    cmd_right_flag = 1;




    if ( events & Coordinator_ProcessParaSet_EVT )
    {
        int index;
        time_new = osal_GetSystemClock();

        if((time_new - time_old) > 0x00000FA0)
        {
            for(index = 0; index < sm_max; index++)
            {
                if(sm_receive_flag[index] == 0)

                {
                    sm_receive_flag[index] = 3; //Change status to ignore status
                    sm_ADD_status[index] = 0;   //The address is invalid

                }
                if(sm_receive_flag[index] == 1 || sm_receive_flag[index] == 2)
                    sm_receive_flag[index] = 0;
            }
            zclCoordinator_sendACK(0x08, 0x03, 0x02);//time out
            time_new = 0;
            time_old = 0;
        }

        else
        {

            for(index = 0; index < sm_max; index++)
            {

                if(sm_ADD_status[index] == 1)

                    sm_flag_and = sm_flag_and && sm_receive_flag[index];
            }

            if(sm_flag_and && sm_max)
            {
                zclCoordinator_sendACK(0x08, 0x03, 0x00);

                for(index = 0; index < sm_max; index++)
                {
                    if(sm_receive_flag[index] != 3)
                        sm_receive_flag[index] = 0;
                }

                time_new = 0;
                time_old = 0;
            }

            else
            {
                sm_flag_and = 1;
                osal_set_event(task_id, Coordinator_ProcessParaSet_EVT);
            }
        }

        return ( events ^ Coordinator_ProcessParaSet_EVT );
    }


    if ( events & Coordinator_EnergyResetWait_EVT )

    {
        int index;
        time_new = osal_GetSystemClock();

        if((time_new - time_old) > 0x00000FA0)
        {
            sm_ADD_reg = BUILD_UINT64_8(controlReg[7], controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14]);
            for(index = 0; index < sm_max; index++)
            {
                if(sm_ADD_reg == sm_ADD[index])




                    sm_ADD_status[index] = 0;  //The address is invalid
            }
            sm_ADD_reg = 0;
            flagreset = 0;

            zclCoordinator_sendACK(0x08, 0x03, 0x02);//time out

            ENERGY_RESET_VALUE = 0xffffffff;
            SmartMeter_ENERGY_RESET_VALUE = 0xfffffffe;
            time_new = 0;
            time_old = 0;
        }

        else
        {
            if (SmartMeter_ENERGY_RESET_VALUE == ENERGY_RESET_VALUE)
            {
                zclCoordinator_sendACK(0x08, 0x03, 0x00);

                ENERGY_RESET_VALUE = 0xffffffff;
                SmartMeter_ENERGY_RESET_VALUE = 0xfffffffe;
                time_new = 0;
                time_old = 0;
                flagreset = 0;
            }

            else
            {
                osal_set_event(task_id, Coordinator_EnergyResetWait_EVT);

            }
        }
        return ( events ^ Coordinator_EnergyResetWait_EVT );
    }




    if ( events & Coordinator_RelaySet_EVT )

    {
        int index;
        time_new = osal_GetSystemClock();
        if((time_new - time_old) > 0x00000FA0)
        {
            sm_ADD_reg = BUILD_UINT64_8(controlReg[7], controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14]);
            for(index = 0; index < sm_max; index++)
            {
                if(sm_ADD_reg == sm_ADD[index])
                    sm_ADD_status[index] = 0;  //The address is invalid
            }
            sm_ADD_reg = 0;

            zclCoordinator_sendACK(0x08, 0x03, 0x02);//time out

            relay_receive_flag = 0;

            time_new = 0;
            time_old = 0;
        }
        else
        {
            if (relay_receive_flag)


            {
                if(SmartMeter_relay == 1)
                    zclCoordinator_sendACK(0x08, 0x03, 0x00);
                else if (SmartMeter_relay == 0)
                    zclCoordinator_sendACK(0x00, 0x03, 0x00);

                relay_receive_flag = 0;



                time_new = 0;
                time_old = 0;
            }

            else
            {
                osal_set_event(task_id, Coordinator_RelaySet_EVT);

            }
        }
        return ( events ^ Coordinator_RelaySet_EVT );

    }

    if ( events & Calibration_WAIT_EVT )

    {
        int index;
        time_new = osal_GetSystemClock();
        if((time_new - time_old) > 0x000F8530)   ///??????
        {
            sm_ADD_reg = BUILD_UINT64_8(controlReg[7], controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14]);
            for(index = 0; index < sm_max; index++)
            {
                if(sm_ADD_reg == sm_ADD[index])
                    sm_ADD_status[index] = 0;  //The address is invalid
            }
            sm_ADD_reg = 0;

            zclCoordinator_sendACK(0x08, 0x03, 0x02);//time out

            cal_receive_flag = 0;
            time_new = 0;
            time_old = 0;
        }
        else
        {
            if ( cal_receive_flag )

            {
                zclCoordinator_sendACK(0x08, 0x03, 0x00);

                cal_receive_flag = 0;

                time_new = 0;
                time_old = 0;
            }
            else
            {
                osal_set_event(task_id, Calibration_WAIT_EVT);

            }
        }
        return ( events ^ Calibration_WAIT_EVT );

    }

    if ( events & Network_WAIT_EVT )
    {
        time_new = osal_GetSystemClock();
        if((time_new - time_old) > 0x00001F40) //8000ms
        {
            //zclCoordinator_sendACK(0x08, 0x03, 0x02);//time out
            int i;
            controlReg[0] = 0x08; // reset controlReg[0] to default
            controlReg[1] = 0x03;// reset controlReg[1] to default
            controlReg[2] = 0x00;// reset controlReg[2] to default

            pack_out[0] = 0x68;
            for(i = 1; i <= 8; i++)
                pack_out[i] = 0;
            pack_out[9] = 0x68;
            pack_out[10] = 0x05;
            pack_out[11] = (uint8)((sm_max & 0xff00) >> 8);
            pack_out[12] = (uint8)(sm_max & 0x00ff);          //sent the max of smart meter to local server
            pack_out[13] = controlReg[0];
            pack_out[14] = controlReg[1];
            pack_out[15] = controlReg[2];
            for(i = 0; i < 16; i++)
                pack_out[16] += pack_out[i];
            pack_out[17] = 0x16;

            UARTEnable(UART0_BASE );
            for (i = 0; i < 18; i++)
            {
                UARTCharPut(UART0_BASE, pack_out[i]);
            }
            UARTDisable(UART0_BASE);

            time_new = 0;
            time_old = 0;
        }

        else
        {
            osal_set_event(task_id, Network_WAIT_EVT);
        }

        return ( events ^ Network_WAIT_EVT );
    }


    if(events & ACK_WAIT_EVT)
    {

        uint8 smvalid_flag = 0;
        uint8 all_smvalid_flag = 0;

        if(PowSpCal == 1)
        {
            if(ack_index < (sm_max - 1))
            {
                uint16 i;


                for(i = ack_index + 1; i < sm_max; i++)
                    smvalid_flag = smvalid_flag || sm_ADD_status[i];

                if(smvalid_flag == 0)
                {
                    //ack_index++;                                //Need ack_index++ to judge sm_max-1, in DATA_CL_EVT, then it will stop round robin and wait for new command
                    Round_end_flag = 1;
                    //                 dataRegSel = dataRegSel ^ 0x01;
                    osal_set_event(task_id, DATA_CL_EVT);
                    //ack_index = sm_max - 1;
                    Drr_flag = 1;
                    ACK_flag = 0;
                }
                else
                {
                    ack_index++;
                    osal_set_event(task_id, ACK_CS_EVT);
                }
            }
            else
            {

                //ack_index++;                                //Need ack_index++ to judge sm_max-1, in DATA_CL_EVT, then it will stop round robin and wait for new command
                Round_end_flag = 1;

                //                 dataRegSel = dataRegSel ^ 0x01;
                osal_set_event(task_id, DATA_CL_EVT);
                Drr_flag = 1;
                ACK_flag = 0;
            }

        }
        else
        {
            int m = 0;
            for(m = 0; m < sm_max; m++)
                all_smvalid_flag = all_smvalid_flag || sm_ADD_status[m];
            if(all_smvalid_flag == 0)
            {
                //ack_index++;                                //Need ack_index++ to judge sm_max-1, in DATA_CL_EVT, then it will stop round robin and wait for new command
                Round_end_flag = 1;
                //                 dataRegSel = dataRegSel ^ 0x01;
                osal_set_event(task_id, DATA_CL_EVT);
                //ack_index = sm_max - 1;
                Drr_flag = 1;
                ACK_flag = 0;


            }
            else
            {


                if(ack_index < (sm_max - 1))
                {
                    ack_index++;                                  ////prepare the next Smart Meter address.
                    osal_set_event(task_id, ACK_CS_EVT);

                }

                else
                {
                    ack_index = 0;                                //check there is no stop it, so the round robin will continue from the beginning.
                    osal_set_event(task_id, ACK_CS_EVT);

                }
            }
        }

        return ( events ^ ACK_WAIT_EVT );

    }

    // Please update to 20bytes of dataReg
    if(events & DATA_CL_EVT)                                       ///DATA_CL_EVT
    {

        pack_out[0] = 0x68;
        int i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = 0;

        pack_out[9] = 0x68;
        pack_out[10] = 0x31;

        if (dataRegSel == 0)
        {
            HalLcdWriteString( "datareg1", HAL_LCD_LINE_7 );
            if(Timeout_Pong == 1)

            {
                controlReg[2] = 0x02;
                Timeout_Pong = 0;
            }
            else

            {
                controlReg[2] = 0x00;

            }
            for (index = 0; index < 23; index++)

            {


                value = dataReg_Pong[index];
                u_value = int16ToUint16(value);

                // uint16_t into two uint8_t first
                u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                u_partB = (uint8_t) (u_value & 0x00FF);

                pack_out[11 + index * 2] = u_partA;
                pack_out[12 + index * 2] = u_partB;
                //UARTCharPut (UART0_BASE, *pdataReg_Pong); //send data
                //pdataReg_Pong++;
            }

        }

        if (dataRegSel == 1)
        {


            HalLcdWriteString( "datareg0", HAL_LCD_LINE_7 );
            //pdataReg_Ping=&dataReg_Ping[0];    //initialize pointer
            if(Timeout_Ping == 1)

            {
                controlReg[2] = 0x02;
                Timeout_Ping = 0;

            }
            else
            {
                controlReg[2] = 0x00;
            }

            for (index = 0; index < 23; index++)

            {
                //UARTCharPut (UART0_BASE, *pdataReg_Ping); //send data
                //pdataReg_Ping++;
                value = dataReg_Ping[index];
                u_value = int16ToUint16(value);


                // uint16_t into two uint8_t first
                u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                u_partB = (uint8_t) (u_value & 0x00FF);


                pack_out[11 + index * 2] = u_partA;
                pack_out[12 + index * 2] = u_partB;

            }

        }

        controlReg[1] = controlReg[1] | 0x01;
        if(Round_end_flag == 1)
        {
            controlReg[0] = controlReg[0] & 0x0F;
            controlReg[1] = controlReg[1] & 0x01;
            PowSpCal = 0;
            ack_index = 0;
            Round_end_flag = 0;
            flaginc = 1;
            zclCoordinator_SendRestart();
        }
        if(PowSpCal == 1)
        {
            controlReg[1] = controlReg[1] & 0x01;

            if(first_complete == 0 )
                osal_set_event(task_id, DATA_WAIT_EVT);
        }

        else if(first_complete == 0 )
            osal_set_event(task_id, DATA_WAIT_EVT);

        Timeout_bit = 0;

        pack_out[57] = controlReg[0];
        pack_out[58] = controlReg[1];
        pack_out[59] = controlReg[2];
        pack_out[60] = 0;
        for(i = 0; i < 60; i++)
            pack_out[60] += pack_out[i];
        pack_out[61] = 0x16;
        UARTEnable(UART0_BASE );
        for (i = 0; i < 62; i++)
        {
            UARTCharPut(UART0_BASE, pack_out[i]);
        }

        UARTDisable(UART0_BASE );
        controlReg[1] = 0x03;
        dataRegSel = dataRegSel ^ 0x01;

        first_complete = 0;

        return ( events ^ DATA_CL_EVT );

    }


    if(events & ACK_CS_EVT)
    {
        HalLcdWriteString( "ACKCSEVT", HAL_LCD_LINE_7 );
        if((controlReg[2] & 0x01) == 1)   //retry
        {
            pack_out[0] = 0x68;

            int i;
            for(i = 1; i <= 8; i++)
                pack_out[i] = 0;
            pack_out[9] = 0x68;
            pack_out[10] = 0x39;

            if (dataRegSel == 0)
            {
                if(!Drr_Retry_Zero)
                {
                    dataRegSel =  1 ; //toggle dataRegSel
                }
                else
                {
                    //pdataReg_Pong = &dataReg_Pong[0];    //initialize pointer
                    for (index = 0; index < 17; index++)
                    {
                        value = dataReg_Pong[index];
                        u_value = int16ToUint16(value);
                        // uint16_t into two uint8_t first
                        u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                        u_partB = (uint8_t) (u_value & 0x00FF);

                        pack_out[11 + index * 2] = u_partA;
                        pack_out[12 + index * 2] = u_partB;

                        //UARTCharPut (UART0_BASE, *pdataReg_Pong); //send data
                        //pdataReg_Pong++;
                    }
                }
            }
            if (dataRegSel == 1)
            {

                for (index = 0; index < 23; index++)
                {
                    //UARTCharPut (UART0_BASE, *pdataReg_Ping); //send data
                    //pdataReg_Ping++;
                    value = dataReg_Ping[index];
                    u_value = int16ToUint16(value);

                    // uint16_t into two uint8_t first
                    u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                    u_partB = (uint8_t) (u_value & 0x00FF);

                    pack_out[11 + index * 2] = u_partA;
                    pack_out[12 + index * 2] = u_partB;
                }
            }
            //            controlReg[0] = controlReg[0] & 0x7F; //reset bit 7 to 0
            controlReg[1] = controlReg[1] | 0x01;
            if(ack_index == sm_max )                                    //IF SM_MAX NEEDS RETRY
            {
                controlReg[1] = controlReg[1] & 0x01;
            }

            pack_out[57] = controlReg[0];
            pack_out[58] = controlReg[1];
            pack_out[59] = controlReg[2];
            pack_out[60] = 0;
            for(i = 0; i < 60; i++)
                pack_out[60] += pack_out[i];

            pack_out[61] = 0x16;


            UARTEnable(UART0_BASE );
            for (i = 0; i < 62; i++)
            {
                UARTCharPut(UART0_BASE, pack_out[i]);
            }

            UARTDisable(UART0_BASE );
            controlReg[1] = 0x03;
        }
        else
        {



            datain_complete = 0;

            zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[ack_index]) >> 56) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[ack_index]) >> 48) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[ack_index]) >> 40) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[ack_index]) >> 32) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[ack_index]) >> 24) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[ack_index]) >> 16) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[ack_index]) >> 8) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[ack_index]) & 0x00000000000000FF);


            if(sm_ADD_status[ack_index] == 1)
            {

                //Stop power calculation
                flaginc = 0;
                zclCoordinator_SendRestart();
                time_old = osal_GetSystemClock();
                time_new = time_old;

                osal_set_event(task_id, DATA_CL_EVT);

            }

            else
            {
                int i;
                int all_invaild_flag = 0;
                for(i = 0; i < sm_max; i++)
                    all_invaild_flag = all_invaild_flag || sm_ADD_status[i];
                if(all_invaild_flag != 0)
                    osal_set_event(task_id, ACK_WAIT_EVT);
                else
                    osal_set_event(task_id, DATA_CL_EVT);
            }

        }

        return ( events ^ ACK_CS_EVT );

    }

    if(events & DATA_WAIT_EVT)
    {
        //       HalLcdWriteString( "DATAWAITEVT", HAL_LCD_LINE_7 );
        time_new = osal_GetSystemClock();

        if((time_new - time_old) > 0x00001770)
        {

            sys_timenew = osal_GetSystemClock();
            sys_secnew = sys_secold + (uint32)((float)(sys_timenew - sys_timeold) / 1000);
            osal_ConvertUTCTime(&TimeStruct , sys_secnew);
            if(TimeStruct.month == 0)
            {
                TimeStruct.month = 12;
                TimeStruct.year--;
            }

            first_complete = 0;
            datain_complete = 0;

            time_new = 0;
            time_old = 0;
            //zclCoordinator_sendACK(0x08, 0x03, 0x02);//time out
            sm_ADD_status[ack_index] = 0;
            Timeout_bit = 1;
            if (dataRegSel == 1)
            {
                Timeout_Ping = 1;
                dataReg_Ping[0] = (uint16)(((sm_ADD[ack_index]) >> 48) & 0x000000000000FFFF); //ADD_3
                dataReg_Ping[1] = (uint16)(((sm_ADD[ack_index]) >> 32) & 0x000000000000FFFF); //ADD_2
                dataReg_Ping[2] = (uint16)(((sm_ADD[ack_index]) >> 16) & 0x000000000000FFFF); //ADD_1
                dataReg_Ping[3] = (uint16)((sm_ADD[ack_index]) & 0x000000000000FFFF); //ADD_0
                dataReg_Ping[4] = coordinator_nwkAddr;
                dataReg_Ping[5] = 0;
                dataReg_Ping[6] = 0;
                dataReg_Ping[7] = 0;
                dataReg_Ping[8] = 0;
                dataReg_Ping[9] = 0;
                dataReg_Ping[10] = 0;
                dataReg_Ping[11] = 0;
                dataReg_Ping[12] = 0;
                dataReg_Ping[13] = 0;
                dataReg_Ping[14] = 0;
                dataReg_Ping[15] = 0;
                dataReg_Ping[16] = 0;
                dataReg_Ping[17] = (uint16)TimeStruct.year;
                dataReg_Ping[18] = (uint16)TimeStruct.month;
                dataReg_Ping[19] = (uint16)TimeStruct.day;
                dataReg_Ping[20] = (uint16)TimeStruct.hour;
                dataReg_Ping[21] = (uint16)TimeStruct.minutes;
                dataReg_Ping[22] = (uint16)TimeStruct.seconds;

                //set timeout bit
            }
            else
            {
                Timeout_Pong = 1;
                dataReg_Pong[0] = (uint16)(((sm_ADD[ack_index]) >> 48) & 0x000000000000FFFF); //ADD_3
                dataReg_Pong[1] = (uint16)(((sm_ADD[ack_index]) >> 32) & 0x000000000000FFFF); //ADD_2
                dataReg_Pong[2] = (uint16)(((sm_ADD[ack_index]) >> 16) & 0x000000000000FFFF); //ADD_1
                dataReg_Pong[3] = (uint16)((sm_ADD[ack_index]) & 0x000000000000FFFF); //ADD_0
                dataReg_Pong[4] = coordinator_nwkAddr;
                dataReg_Pong[5] = 0;
                dataReg_Pong[6] = 0;
                dataReg_Pong[7] = 0;
                dataReg_Pong[8] = 0;
                dataReg_Pong[9] = 0;
                dataReg_Pong[10] = 0;
                dataReg_Pong[11] = 0;
                dataReg_Pong[12] = 0;
                dataReg_Pong[13] = 0;
                dataReg_Pong[14] = 0;
                dataReg_Pong[15] = 0;
                dataReg_Pong[16] = 0;
                dataReg_Pong[17] = (uint16)TimeStruct.year;
                dataReg_Pong[18] = (uint16)TimeStruct.month;
                dataReg_Pong[19] = (uint16)TimeStruct.day;
                dataReg_Pong[20] = (uint16)TimeStruct.hour;
                dataReg_Pong[21] = (uint16)TimeStruct.minutes;
                dataReg_Pong[22] = (uint16)TimeStruct.seconds;

            }
            //  dataRegSel = dataRegSel ^ 0x01;
            // osal_set_event(task_id, DATA_CL_EVT);

        }
        else
        {
            if(datain_complete)
            {
                if(first_complete)
                {

                    if(ack_index < (sm_max - 1))
                    {
                        ack_index++;                                 //at first, the DRR command actually will send request to two Smart Meter, so the Smart Meter address need update.
                        osal_set_event(task_id, ACK_CS_EVT);
                        //first_complete = 0;
                        datain_complete = 0;
                    }
                    else
                    {
                        ack_index = 0;
                        osal_set_event(task_id, ACK_CS_EVT);
                        //first_complete = 0;
                        datain_complete = 0;
                    }
                }

                else
                {
                    //osal_set_event(task_id, DATA_CL_EVT);
                    datain_complete = 0;
                }

                time_new = 0;
                time_old = 0;
            }

            else
            {
                if(Round_end_flag == 1)
                {

                    datain_complete = 0;
                }
                else
                {
                    osal_set_event(task_id, DATA_WAIT_EVT);
                }

            }
        }

        return ( events ^ DATA_WAIT_EVT );


    }


    if(events & GETCAL_WAIT_EVT)
    {
        int index;
        time_new = osal_GetSystemClock();
        if((time_new - time_old) > 0x00000FA0)
        {
            sm_ADD_reg = BUILD_UINT64_8(controlReg[7], controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14]);
            for(index = 0; index < sm_max; index++)
            {
                if(sm_ADD_reg == sm_ADD[index])
                    sm_ADD_status[index] = 0;  //The address is invalid
            }
            sm_ADD_reg = 0;

            zclCoordinator_calregtimeout();//time out

            flag_calget = 0;

            time_new = 0;
            time_old = 0;
        }
        else
        {
            if (flag_calget)

            {
                zclCoordinator_sendcalreg();


                flag_calget = 0;

                time_new = 0;
                time_old = 0;
            }

            else
            {
                osal_set_event(task_id, GETCAL_WAIT_EVT);
            }
        }
        return ( events ^ GETCAL_WAIT_EVT );

    }

    if(events & TIME_RUNNING_EVT)
    {

        sys_timenew = osal_GetSystemClock();
        sys_secnew = sys_secold + (uint32)((float)(sys_timenew - sys_timeold) / 1000);
        //sys_timeold = sys_timenew;
        //sys_secold = sys_secnew;
        osal_ConvertUTCTime(&TimeStruct , sys_secnew);
        if(TimeStruct.month == 0)
        {
            TimeStruct.month = 12;
            TimeStruct.year--;
        }

#ifdef LCD_SUPPORTED
        sprintf((char *)lcdString, "%d %d %d %d %d %d", TimeStruct.year, TimeStruct.month,
                TimeStruct.day, TimeStruct.hour, TimeStruct.minutes, TimeStruct.seconds);
        HalLcdWriteString( lcdString, HAL_LCD_LINE_7 );
#endif

        osal_set_event(task_id, TIME_RUNNING_EVT);
        return ( events ^ TIME_RUNNING_EVT );
    }

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

}

void zclscoordinator_startEZmode( void )
{

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
    ezModeData.numActiveOutClusters = 1;   // active output cluster //1
    ezModeData.pActiveOutClusterIDs = clusterIDs;
    zcl_InvokeEZMode( &ezModeData );

#ifdef LCD_SUPPORTED
    HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

#endif

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

void zclCoordinator_LcdDisplayTestMode( void )
{
    char sDisplayCoIEEEaddr[32];
    uint8 IEEEaddr[8];

    // display coordinator IEEE addr
    osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
    if ( (ADD_0 == NULL) || (ADD_1 == NULL) || (ADD_2 == NULL) || (ADD_3 == NULL))
    {
        osal_memcpy( &sDisplayCoIEEEaddr[5], "N/A", 4 );
    }
    else
    {
        osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
        IEEEaddr[0] = (ADD_3 >> 8) & 0x00FF; // highest bit: ADD_3
        IEEEaddr[1] = ADD_3 & 0x00FF;
        IEEEaddr[2] = (ADD_2 >> 8) & 0x00FF;
        IEEEaddr[3] = ADD_2 & 0x00FF;
        IEEEaddr[4] = (ADD_1 >> 8) & 0x00FF;
        IEEEaddr[5] = ADD_1 & 0x00FF;
        IEEEaddr[6] = (ADD_0 >> 8) & 0x00FF;
        IEEEaddr[7] = ADD_0 & 0x00FF;

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

void zclCoordinator_LcdDisplayTestMode_smaddr( void )
{
    char sDisplayCoIEEEaddr[32];
    uint8 IEEEaddr[8];

    // display coordinator IEEE addr
    osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
    if ( (sm_ADD_0[sm_index] == NULL) || (sm_ADD_1[sm_index] == NULL) || (sm_ADD_2[sm_index] == NULL) || (sm_ADD_3[sm_index] == NULL))
    {
        osal_memcpy( &sDisplayCoIEEEaddr[5], "N/A", 4 );
    }
    else
    {
        osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
        IEEEaddr[0] = (sm_ADD_3[sm_index] >> 8) & 0x00FF; // highest bit: ADD_3
        IEEEaddr[1] = sm_ADD_3[sm_index] & 0x00FF;
        IEEEaddr[2] = (sm_ADD_2[sm_index] >> 8) & 0x00FF;
        IEEEaddr[3] = sm_ADD_2[sm_index] & 0x00FF;
        IEEEaddr[4] = (sm_ADD_1[sm_index] >> 8) & 0x00FF;
        IEEEaddr[5] = sm_ADD_1[sm_index] & 0x00FF;
        IEEEaddr[6] = (sm_ADD_0[sm_index] >> 8) & 0x00FF;
        IEEEaddr[7] = sm_ADD_0[sm_index] & 0x00FF;

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
        zclCoordinator_ProcessInReadRspCmd( pInMsg );
        break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
        zclCoordinator_ProcessInWriteRspCmd( pInMsg );
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
        zclCoordinator_ProcessInDefaultRspCmd( pInMsg );
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
}
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

    // Process incomming parameter report from smart meter in response to GET parameter command *
    zclReportCmd_t *pInParameterReport;

    pInParameterReport = (zclReportCmd_t *)pInMsg->attrCmd;
    uint16 OPERATION = BUILD_UINT16(pInParameterReport->attrList[0].attrData[0], pInParameterReport->attrList[0].attrData[1]);
    uint16 RESULT = BUILD_UINT16(pInParameterReport->attrList[0].attrData[2], pInParameterReport->attrList[0].attrData[3]);

    uint16 ENERGY_RESET_VALUE_1;
    uint16 ENERGY_RESET_VALUE_0;


    if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) &&
            // if ((OPERATION == USR_TX_GET) && (RESULT == COM_PARAM) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_PARAMETER_MEASURED_VALUE))
    {

        // store the current parameter value sent over the air from smart meter *
        SmartMeterparamReg[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]); //MIN_ADC
        SmartMeterparamReg[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]); //MAX_ADC
        SmartMeterparamReg[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]); //SAMPLE_INT
        SmartMeterparamReg[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]); //SAMPLE_WIN
        SmartMeterparamReg[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]); //MIN_V
        SmartMeterparamReg[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]); //MAX_V
        SmartMeterparamReg[6] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]); //MIN_I
        SmartMeterparamReg[7] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]); //MAX_I
        SmartMeterparamReg[8] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20], pInParameterReport->attrList[0].attrData[21]); //SHARP1
        SmartMeterparamReg[9] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22], pInParameterReport->attrList[0].attrData[23]); //SHARP2
        SmartMeterparamReg[10] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24], pInParameterReport->attrList[0].attrData[25]); //PEAK1
        SmartMeterparamReg[11] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26], pInParameterReport->attrList[0].attrData[27]); //PEAK2
        SmartMeterparamReg[12] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[28], pInParameterReport->attrList[0].attrData[29]); //PEAK3
        SmartMeterparamReg[13] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[30], pInParameterReport->attrList[0].attrData[31]); //SHOULDER1
        SmartMeterparamReg[14] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[32], pInParameterReport->attrList[0].attrData[33]); //SHOULDER2
        SmartMeterparamReg[15] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[34], pInParameterReport->attrList[0].attrData[35]); //SHOULDER3
        SmartMeterparamReg[16] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[36], pInParameterReport->attrList[0].attrData[37]); //OFF
        SmartMeterparamReg[17] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[38], pInParameterReport->attrList[0].attrData[39]); //N_SM
        sm_id = BUILD_UINT16(pInParameterReport->attrList[0].attrData[40], pInParameterReport->attrList[0].attrData[41]);


        if(!zclCoordinator_SmartMeterParamCompare())
        {
            sm_receive_flag[sm_id] = 1;
            sm_retry_times[sm_id] = 3;  //reset retry times

        }
        else
        {
            if(!sm_retry_times[sm_id])
            {
                sm_receive_flag[sm_id] = 2; //means time out
                sm_retry_times[sm_id] = 3;  //reset retry times
            }

            else  //if retry times is not 0
            {
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_id]) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_id]) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_id]) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_id]) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_id]) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_id]) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_id]) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_id]) & 0x00000000000000FF);
                zclCoordinator_SetParam();

                sm_retry_times[sm_id]--;

            }
        }
    }

    if ((OPERATION == TIME_SET) && (RESULT == SUCCESS) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        // store the current time value sent over the air from smart meter *

        SmartMeterTimeReg[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        SmartMeterTimeReg[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        SmartMeterTimeReg[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        SmartMeterTimeReg[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        SmartMeterTimeReg[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        SmartMeterTimeReg[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        sm_id = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);
        //xxxxx



        if(!zclCoordinator_SmartMeterTimeCompare())
        {
            sm_receive_flag[sm_id] = 1;
            sm_retry_times[sm_id] = 3;  //reset retry times

        }
        else
        {
            if(!sm_retry_times[sm_id])
            {
                sm_receive_flag[sm_id] = 2; //means time out
                sm_retry_times[sm_id] = 3;  //reset retry times
            }

            else  //if retry times is not 0
            {
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_id]) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_id]) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_id]) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_id]) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_id]) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_id]) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_id]) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_id]) & 0x00000000000000FF);
                zclCoordinator_SetTime();

                sm_retry_times[sm_id]--;

            }
        }
    }

    //Process incomming data report from smart meter in response to GET date command *
    if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_DATA_MEASURED_VALUE))
    {

        uint16 smADD_3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        uint16 smADD_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        uint16 smADD_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        uint16 smADD_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        uint16 smADD16 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        uint16 RMS_V1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        uint16 RMS_I1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);
        uint16 THETA_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]);
        uint16 RMS_V2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20], pInParameterReport->attrList[0].attrData[21]);
        uint16 RMS_I2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22], pInParameterReport->attrList[0].attrData[23]);
        uint16 THETA_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24], pInParameterReport->attrList[0].attrData[25]);
        uint16 RMS_V3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26], pInParameterReport->attrList[0].attrData[27]);
        uint16 RMS_I3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[28], pInParameterReport->attrList[0].attrData[29]);
        uint16 THETA_3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[30], pInParameterReport->attrList[0].attrData[31]);
        uint16 SM_V = BUILD_UINT16(pInParameterReport->attrList[0].attrData[32], pInParameterReport->attrList[0].attrData[33]);
        uint16 SM_I = BUILD_UINT16(pInParameterReport->attrList[0].attrData[34], pInParameterReport->attrList[0].attrData[35]);
        uint16 YEAR = BUILD_UINT16(pInParameterReport->attrList[0].attrData[36], pInParameterReport->attrList[0].attrData[37]);
        uint16 MONTH = BUILD_UINT16(pInParameterReport->attrList[0].attrData[38], pInParameterReport->attrList[0].attrData[39]);
        uint16 DAY = BUILD_UINT16(pInParameterReport->attrList[0].attrData[40], pInParameterReport->attrList[0].attrData[41]);
        uint16 HOUR = BUILD_UINT16(pInParameterReport->attrList[0].attrData[42], pInParameterReport->attrList[0].attrData[43]);
        uint16 MINUTE = BUILD_UINT16(pInParameterReport->attrList[0].attrData[44], pInParameterReport->attrList[0].attrData[45]);
        uint16 SECOND = BUILD_UINT16(pInParameterReport->attrList[0].attrData[46], pInParameterReport->attrList[0].attrData[47]);
        uint16 STATUS = BUILD_UINT16(pInParameterReport->attrList[0].attrData[48], pInParameterReport->attrList[0].attrData[49]);

        // store the current data value sent over the air from smart meter
        if (dataRegSel == 0)
        {


            dataReg_Pong[0] = smADD_3; //ADD_3
            dataReg_Pong[1] = smADD_2; //ADD_2
            dataReg_Pong[2] = smADD_1; //ADD_1
            dataReg_Pong[3] = smADD_0; //ADD_0
            dataReg_Pong[4] = smADD16;
            dataReg_Pong[5] = RMS_V1;
            dataReg_Pong[6] = RMS_I1;
            dataReg_Pong[7] = THETA_1;
            dataReg_Pong[8] = RMS_V2;
            dataReg_Pong[9] = RMS_I2;
            dataReg_Pong[10] = THETA_2;
            dataReg_Pong[11] = RMS_V3;
            dataReg_Pong[12] = RMS_I3;
            dataReg_Pong[13] = THETA_3;
            dataReg_Pong[14] = SM_V;
            dataReg_Pong[15] = SM_I;
            dataReg_Pong[16] = STATUS;      //STATUS FORM SMART METER
            dataReg_Pong[17] = YEAR;
            dataReg_Pong[18] = MONTH;
            dataReg_Pong[19] = DAY;
            dataReg_Pong[20] = HOUR;
            dataReg_Pong[21] = MINUTE;
            dataReg_Pong[22] = SECOND;
        }
        else if(dataRegSel == 1)
        {
            dataReg_Ping[0] = smADD_3; //ADD_3
            dataReg_Ping[1] = smADD_2; //ADD_2
            dataReg_Ping[2] = smADD_1; //ADD_1
            dataReg_Ping[3] = smADD_0; //ADD_0
            dataReg_Ping[4] = smADD16;
            dataReg_Ping[5] = RMS_V1;
            dataReg_Ping[6] = RMS_I1;
            dataReg_Ping[7] = THETA_1;
            dataReg_Ping[8] = RMS_V2;
            dataReg_Ping[9] = RMS_I2;
            dataReg_Ping[10] = THETA_2;
            dataReg_Ping[11] = RMS_V3;
            dataReg_Ping[12] = RMS_I3;
            dataReg_Ping[13] = THETA_3;
            dataReg_Ping[14] = SM_V;
            dataReg_Ping[15] = SM_I;
            dataReg_Ping[16] = STATUS;      //STATUS FORM SMART METER
            dataReg_Ping[17] = YEAR;
            dataReg_Ping[18] = MONTH;
            dataReg_Ping[19] = DAY;
            dataReg_Ping[20] = HOUR;
            dataReg_Ping[21] = MINUTE;
            dataReg_Ping[22] = SECOND;

        }

        //set flag to indicate all data have been received
        if(first_write_flag == 1)
        {
            // dataRegSel = dataRegSel ^ 0x01;

            first_write_flag = 0;
            first_complete = 1;

        }
        datain_complete = 1;
    }

    // Process incomming address report from smart meter in response to route discovery command *
    if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) &&
            ((pInParameterReport->attrList[0].attrID) == ATTRID_MS_ADD_MEASURED_VALUE ))
    {
        sm_ADD_3[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        sm_ADD_2[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        sm_ADD_1[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        sm_ADD_0[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);

        sm_ADD[sm_index] = BUILD_UINT64_16(sm_ADD_3[sm_index], sm_ADD_2[sm_index],
                                           sm_ADD_1[sm_index], sm_ADD_0[sm_index]);

        zclCoordinator_LcdDisplayTestMode_smaddr();

        zclCoordinator_DstAddr.addrMode = (afAddrMode_t)Addr64Bit;
        zclCoordinator_DstAddr.endPoint = Coordinator_ENDPOINT;
        zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_index]) >> 56) & 0x00000000000000FF); //AF.h; highest
        zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_index]) >> 48) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_index]) >> 40) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_index]) >> 32) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_index]) >> 24) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_index]) >> 16) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_index]) >> 8) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_index]) & 0x00000000000000FF);


        //Ack smart meter with sm_Add[index]
        zclCoordinator_SendAck();

        // for test
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "Nwkdiscov Back", HAL_LCD_LINE_4 );
#endif

        sm_index++; //increment index
        sm_max++; //total number of smart meter detected
    }

    if ((OPERATION == GET_CALPARAM) && (RESULT == SUCCESS) &&
            ((pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE ))
    {

        calsm_id = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        calSMADD_3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        calSMADD_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        calSMADD_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        calSMADD_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        calMAG_V1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        calMAG_I1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);
        calMAG_V2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]);
        calMAG_I2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20], pInParameterReport->attrList[0].attrData[21]);
        calMAG_V3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22], pInParameterReport->attrList[0].attrData[23]);
        calMAG_I3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24], pInParameterReport->attrList[0].attrData[25]);
        calT_EFF = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26], pInParameterReport->attrList[0].attrData[27]);

        flag_calget = 1;
        // for test
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "Get Calpara", HAL_LCD_LINE_4 );
#endif

    }


    // Process incomming energy reset report from smart meter in response to RESET command *
    if ((OPERATION == RESET) && (RESULT == SUCCESS) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        SmartMeter_flagreset = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        ENERGY_RESET_VALUE_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        ENERGY_RESET_VALUE_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        SmartMeter_ENERGY_RESET_VALUE = BUILD_UINT32_16(ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0);


        // for test
#ifdef LCD_SUPPORTED
        char sflagreset[16];
        char sENERGY_RESET_VALUE[32];
        // char sENERGY_RESET_VALUE_1[16];
        _ltoa(  SmartMeter_flagreset , (void *)(&sflagreset[0]), 16 );
        _ltoa(  SmartMeter_ENERGY_RESET_VALUE , (void *)(&sENERGY_RESET_VALUE[0]), 16 );
        // _ltoa(  ENERGY_RESET_VALUE_1 , (void *)(&sENERGY_RESET_VALUE_1[0]), 16 );
        HalLcdWriteString( "SendResetBack SUCCESS", HAL_LCD_LINE_3 );
        HalLcdWriteString( (char *)sflagreset, HAL_LCD_LINE_4 );
        HalLcdWriteString( (char *)sENERGY_RESET_VALUE, HAL_LCD_LINE_5 );
        // HalLcdWriteString( (char *)sENERGY_RESET_VALUE_1, HAL_LCD_LINE_6 );
#endif


    }

    // Process incomming relay report from smart meter in response to RELAY command *
    if ((OPERATION == RELAY) && (RESULT == SUCCESS)  &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        SmartMeter_relay = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4],
                                        pInParameterReport->attrList[0].attrData[5]);
        if (SmartMeter_relay == flagrelay)
        {
            relay_receive_flag = 1;
            flagrelay = 2;
        }
        else
        {
            if (!relay_retry_times)
            {
                relay_receive_flag = 2; //means times out
                relay_retry_times = 3;  //reset retry times
            }

            else  //if retry times is not 0
            {
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((RM_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((RM_ADD) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((RM_ADD) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((RM_ADD) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((RM_ADD) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((RM_ADD) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((RM_ADD) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((RM_ADD) & 0x00000000000000FF);
                zclCoordinator_SendRelay();

                relay_retry_times--;

            }

        }

    }

    if ((OPERATION == CALIBRATE) && (RESULT == SUCCESS)  &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        cal_receive_flag = 1;
    }

    // Process incomming restart report from smart meter in response to RESTART command *
    if ((OPERATION == START) && (RESULT == SUCCESS)  &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        SmartMeter_flaginc = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        zclCoordinator_SendData();   //send request to get data
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
/*
static void zclCoordinator_SendParam( void )
{

#ifdef ZCL_REPORT
   zclReportCmd_t *pReportCmd;
   uint16 packet[] = {USR_RX_GET, COM_PARAM};

   pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

   if ( pReportCmd != NULL )
   {
       pReportCmd->numAttr = 1;
       pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
       pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT32; //zcl.c
       pReportCmd->attrList[0].attrData = (void *)(packet);

       zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                          ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                          pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
   }

   osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT

}
*/


/*********************************************************************
 * @fn      zclCoordinator_SetParam *
 *
 * @brief   Called to set SmartMeter parameter information from the Coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SetParam( void ) //verified
{

#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;


    MIN_ADC = paramReg[0];
    MAX_ADC = paramReg[1];
    SAMPLE_INT = paramReg[2];
    SAMPLE_WIN = paramReg[3];
    MIN_V = paramReg[4];
    MAX_V = paramReg[5];
    MIN_I = paramReg[6];
    MAX_I = paramReg[7];
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

    uint16 packet[] = {USR_RX_SET, SET_PARAM, MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN,                      
                       MIN_V, MAX_V, MIN_I, MAX_I, 
                       SHARP1, SHARP2, PEAK1, PEAK2, PEAK3,
                       SHOULDER1, SHOULDER2, SHOULDER3,
                       OFF, N_SM, sm_id
                      };


    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT512;
        pReportCmd->attrList[0].attrData = (void *)(packet);

        zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                           ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
    }

    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT

}


/*********************************************************************
 * @fn      zclCoordinator_getcalParam *
 *
 * @brief   Called to getcalculation related parameterfrom the  SmartMeter
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_getcalParam( void )
{

#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;

    uint16 packet[] = {USR_RX_GET, GET_CALPARAM};

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT32;
        pReportCmd->attrList[0].attrData = (void *)(packet);

        zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                           ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
    }

    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT

}

/*******************************************************************
 * @fn      zclCoordinator_SetTime *                                                    ///added at 11.12 by Gary
 *

 * @brief   Called to set SmartMeter Time information from the Coordinator
 *
 * @param   none
 *
 * @return  none
 */

static void zclCoordinator_SetTime( void ) //verified
{

#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;


    YEAR = timeReg[0];
    MONTH = timeReg[1];
    DAY = timeReg[2];
    HOUR = timeReg[3];
    MINUTE = timeReg[4];
    SECOND = timeReg[5];

    uint16 packet[] = {TIME_SET, YEAR, MONTH, DAY, HOUR,
                       MINUTE, SECOND, sm_id
                      };

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;


        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT256;
        pReportCmd->attrList[0].attrData = (void *)(packet);

        zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,


                           ATTRID_MS_COM_MEASURED_VALUE,
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
    uint16 packet[] = {USR_RX_GET, COM_DATA};

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_DATA_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT32; //zcl.c


        pReportCmd->attrList[0].attrData = (void *)(packet);

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

    uint16 energy_reset_value_1 = 0;
    uint16 energy_reset_value_0 = 0;

    energy_reset_value_1 = (uint16)((ENERGY_RESET_VALUE >> 16) & 0xFFFF);
    energy_reset_value_0 = (uint16)(ENERGY_RESET_VALUE & 0xFFFF);
    uint16 packet[] = {RESET, flagreset, energy_reset_value_1, energy_reset_value_0 };


    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128;


        pReportCmd->attrList[0].attrData = (void *)(packet);

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
    uint16 packet[] = {RELAY, flagrelay };

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT32; //zcl.c
        pReportCmd->attrList[0].attrData = (void *)(packet);

        zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                           ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
    }

    osal_mem_free( pReportCmd );

#endif  // ZCL_REPORT

}


/*********************************************************************
 * @fn      zclCoordinator_SendVolCalibrate *

 *
 * @brief   Called to send command to do Voltage Calibration in smart meter

 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendCalibrate( void )

{

#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;
    V_CAL = controlReg[27];
    I_CAL = controlReg[28];
    T_CAL = controlReg[29];
    N_CAL = controlReg[30];


    uint16 packet[] = {CALIBRATE, V_CAL, I_CAL, T_CAL, N_CAL};


    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128;
        pReportCmd->attrList[0].attrData = (void *)(packet);

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

    uint16 packet[] = {START, flaginc};

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT32;
        pReportCmd->attrList[0].attrData = (void *)(packet);

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
static void zclCoordinator_NetDiscov( void ) //verified
{
    sm_index = 0;
    sm_max = 0;
#ifdef ZCL_REPORT

    afAddrType_t Bc_DstAddr;
    zclReportCmd_t *pReportCmd;

    uint16 packet[] = {USR_RX_GET, COM_ADD, ADD_3, ADD_2, ADD_1, ADD_0};

    Bc_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    Bc_DstAddr.endPoint = Coordinator_ENDPOINT;
    Bc_DstAddr.addr.shortAddr = 0xFFFF;

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_ADD_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128; //zcl.c
        pReportCmd->attrList[0].attrData = (void *)(packet);

        zcl_SendReportCmd( Coordinator_ENDPOINT, &Bc_DstAddr,
                           ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ ); //zcl.c
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
static void zclCoordinator_SendAck( void )  //verified
{
#ifdef ZCL_REPORT
    zclReportCmd_t *pReportCmd;
    uint16 packet[] = {USR_RX_GET, ACK_SUCCESS, sm_ADD_3[sm_index], sm_ADD_2[sm_index], sm_ADD_1[sm_index], sm_ADD_0[sm_index]};  //lhy

    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_ACK_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128; //zcl.c
        pReportCmd->attrList[0].attrData = (void *)(packet);

        zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                           ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                           pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
        // for test
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "SendAck", HAL_LCD_LINE_3 );
#endif
        //
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
    zclCoordinator_nvReadParam(); //read parameter from FLASH
    paramReg[0] = 0;
    paramReg[1] = 1023;
    paramReg[2] = 1000;
    paramReg[3] = 60;
    paramReg[4] = 110;
    paramReg[5] = 110;
    paramReg[6] = 2;
    paramReg[7] = 2;
    paramReg[8] = 0;
    paramReg[9] = 0;
    paramReg[10] = 0;
    paramReg[11] = 0;
    paramReg[12] = 0;
    paramReg[13] = 0;
    paramReg[14] = 0;
    paramReg[15] = 0;
    paramReg[16] = 0;
    paramReg[17] = 0;


    MIN_ADC = paramReg[0];
    MAX_ADC = paramReg[1];
    SAMPLE_INT = paramReg[2];
    SAMPLE_WIN = paramReg[3];   
    MIN_V = paramReg[4];
    MAX_V = paramReg[5];
    MIN_I = paramReg[6];
    MAX_I = paramReg[7];
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


    index = 0;
    sm_index = 0;
    dataRegSel = 1;
    sm_max = 0;
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
    ENERGY_RESET_VALUE = 0x00000000;
    RM_ADD = 0x0000000000000000;

    controlReg[0] = 0;
    controlReg[1] = 0;
    controlReg[2] = 0;
    controlReg[3] = 0;
    controlReg[4] = 0;
    controlReg[5] = 0;
    controlReg[6] = 0;
    controlReg[7] = 0;
    controlReg[8] = 0;
    controlReg[9] = 0;
    controlReg[10] = 0;
    controlReg[11] = 0;
    controlReg[12] = 0;
    controlReg[13] = 0;

}

/***********************************************/

// Michelle: conversion between uint16_t to int16_t
int16_t uint16ToInt16(uint16_t u_i)
{

    int16_t i;

    if (u_i <= pow(2, 15) - 1)
    {
        i = u_i;
    }
    else
    {
        i = (int16_t) (u_i - pow(2, 16));
    }
    return i;
}

uint16_t int16ToUint16(int16_t i)
{

    uint16_t u_i;

    if (i >= 0)
    {
        u_i = i;
    }
    else
    {
        u_i = (uint16_t)(pow(2, 16) + i);
    }
    return u_i;
}

// End

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
    uint16 *flashpt;

    flashpt = &paramReg[0];
    //locate item in flash memory
    osal_nv_item_init (FLASH_PARAM, 60, NULL);
    //read from flash memory and load it into paramReg
    osal_nv_read (FLASH_PARAM, 0, 60, flashpt);
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
    uint16 *flashpt;

    flashpt = &paramReg[0];
    //locate item in flash
    osal_nv_item_init (FLASH_PARAM, 60, NULL);
    //write paramReg to FLASH
    osal_nv_write (FLASH_PARAM, 0, 60, flashpt);

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
    uint8 num = 0;
    int index;

    for (index = 0; index < 18; index++)
    {
        if (SmartMeterparamReg[index] == paramReg[index])
            num++;
    }
    if (num == 18)
        returnvalue = 0;
    else
        returnvalue = 1;

    for (index = 0; index < 18; index++)
    {
        SmartMeterparamReg[index] = 0xff;
    }

    return  returnvalue;
}


/*********************************************************************
 * @fn      zclCoordinator_SmartMeterTimeCompare                                      //added 11.12
 *
 * @brief   Called to compare time
 *
 * @param   none
 *
 * @return  uint8
 */
uint8 zclCoordinator_SmartMeterTimeCompare (void)
{

    uint8 returnvalue;
    uint8 num = 0;
    int index;

    for (index = 0; index < 6; index++)
    {
        if (SmartMeterTimeReg[index] == timeReg[index])
            num++;
    }
    if (num == 6)
        returnvalue = 0;
    else
        returnvalue = 1;

    for (index = 0; index < 6; index++)
    {
        SmartMeterTimeReg[index] = 0xff;
    }

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
    for (i = 0; i < 20; i++)
    {
        dataReg_Pong[i] = 0x0000;
        dataReg_Ping[i] = 0x0000;
    }
}


static void initUART(void)
{
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);

    //
    // Enable UART peripheral module
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);

    //
    // Disable UART function
    //
    UARTDisable(UART0_BASE);

    //
    // Disable all UART module interrupts
    //
    UARTIntDisable(UART0_BASE, 0x1FFF);

    //
    // Set IO clock as UART clock source
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Map UART signals to the correct GPIO pins and configure them as
    // hardware controlled.
    //
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD);
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD, IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    // This function uses SysCtrlClockGet() to get the system clock
    // frequency.  This could be also be a variable or hard coded value
    // instead of a function call.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));



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
static uint8 zclCoordinator_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{

    // Device is notified of the Default Response command.
    (void)pInMsg;

    return ( TRUE );
}

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

        zclCoordinator_NetDiscov();
        uint8 i;
        for(i = 0; i < 100; i++)
            sm_ADD_status[i] = 1;

        osal_start_timerEx( zclCoordinator_TaskID, TIME_RUNNING_EVT, 1000 );
    }
}
#endif // ZCL_EZMODE

//sent a smart meter IEEE address to local server
// smart meter IEEE address is store in  sm_ADD_i[]
//according to the index coordinator can find the corresponding smart meter IEEE address
static void zclCoordinator_ReadRoutingTable(uint8 sm_i)
{
    int i = 0;
    for(i = 0; i < 100; i++)
        pack_out[i] = 0;
    pack_out[0] = 0x68;
    pack_out[1] = 0;
    pack_out[2] = 0;
    pack_out[3] = 0;
    pack_out[4] = 0;
    pack_out[5] = 0;
    pack_out[6] = 0;
    pack_out[7] = 0;
    pack_out[8] = 0;
    pack_out[9] = 0x68;
    pack_out[10] = 0x0D;
    pack_out[11] = (uint8)((sm_ADD_3[sm_i] >> 8) & 0x00FF);
    pack_out[12] = (uint8)(sm_ADD_3[sm_i] & 0x00FF);
    pack_out[13] = (uint8)((sm_ADD_2[sm_i] >> 8) & 0x00FF);
    pack_out[14] = (uint8)(sm_ADD_2[sm_i] & 0x00FF);
    pack_out[15] = (uint8)((sm_ADD_1[sm_i] >> 8) & 0x00FF);
    pack_out[16] = (uint8)(sm_ADD_1[sm_i] & 0x00FF);
    pack_out[17] = (uint8)((sm_ADD_0[sm_i] >> 8) & 0x00FF);
    pack_out[18] = (uint8)(sm_ADD_0[sm_i] & 0x00FF);
    pack_out[19] = sm_ADD_status[sm_i];

    if(sm_ADD_3[sm_i] == 0x0012 && sm_ADD_2[sm_i] == 0x4B00 && sm_ADD_1[sm_i] == 0x040F && sm_ADD_0[sm_i] == 0x1A3C)
        SM_ADD16 = 0x0000;
    else if(sm_ADD_3[sm_i] == 0x0012 && sm_ADD_2[sm_i] == 0x4B00 && sm_ADD_1[sm_i] == 0x040F && sm_ADD_0[sm_i] == 0x1C77)
        SM_ADD16 = 0x0001;
    else if(sm_ADD_3[sm_i] == 0x0012 && sm_ADD_2[sm_i] == 0x4B00 && sm_ADD_1[sm_i] == 0x040E && sm_ADD_0[sm_i] == 0xF19E)
        SM_ADD16 = 0x0002;

    pack_out[20] = (uint8)((SM_ADD16 & 0xff00) >> 8);
    pack_out[21] = (uint8)(SM_ADD16 & 0x00ff);
    pack_out[22] = controlReg[0];
    pack_out[23] = controlReg[1];
    pack_out[24] = controlReg[2];
    for(i = 0; i < 25; i++)
        pack_out[25] += pack_out[i];
    pack_out[26] = 0x16;

    UARTEnable(UART0_BASE );
    for(i = 0; i < 27; i++)
        UARTCharPut (UART0_BASE, pack_out[i]);
    UARTDisable(UART0_BASE );
    sm_index ++;
}

//sent calibration register package to local server
static void zclCoordinator_sendcalreg()
{
    int i = 0;
    for(i = 0; i < 100; i++)
        pack_out[i] = 0;
    pack_out[0] = 0x68;
    pack_out[1] = 0;
    pack_out[2] = 0;
    pack_out[3] = 0;
    pack_out[4] = 0;
    pack_out[5] = 0;
    pack_out[6] = 0;
    pack_out[7] = 0;
    pack_out[8] = 0;
    pack_out[9] = 0x68;
    pack_out[10] = 0x1B;
    pack_out[11] = (uint8)((calSMADD_3 & 0xff00) >> 8);
    pack_out[12] = (uint8)(calSMADD_3 & 0x00ff);
    pack_out[13] = (uint8)((calSMADD_2 & 0xff00) >> 8);
    pack_out[14] = (uint8)(calSMADD_2 & 0x00ff);
    pack_out[15] = (uint8)((calSMADD_1 & 0xff00) >> 8);
    pack_out[16] = (uint8)(calSMADD_1 & 0x00ff);
    pack_out[17] = (uint8)((calSMADD_0 & 0xff00) >> 8);
    pack_out[18] = (uint8)(calSMADD_0 & 0x00ff);
    pack_out[19] = (uint8)((calsm_id & 0xff00) >> 8);
    pack_out[20] = (uint8)(calsm_id & 0x00ff);
    pack_out[21] = (uint8)((calMAG_V1 & 0xff00) >> 8);
    pack_out[22] = (uint8)(calMAG_V1 & 0x00ff);
    pack_out[23] = (uint8)((calMAG_I1 & 0xff00) >> 8);
    pack_out[24] = (uint8)(calMAG_I1 & 0x00ff);
    pack_out[25] = (uint8)((calMAG_V2 & 0xff00) >> 8);
    pack_out[26] = (uint8)(calMAG_V2 & 0x00ff);
    pack_out[27] = (uint8)((calMAG_I2 & 0xff00) >> 8);
    pack_out[28] = (uint8)(calMAG_I2 & 0x00ff);
    pack_out[29] = (uint8)((calMAG_V3 & 0xff00) >> 8);
    pack_out[30] = (uint8)(calMAG_V3 & 0x00ff);
    pack_out[31] = (uint8)((calMAG_I3 & 0xff00) >> 8);
    pack_out[32] = (uint8)(calMAG_I3 & 0x00ff);
    pack_out[33] = (uint8)((calT_EFF & 0xff00) >> 8);
    pack_out[34] = (uint8)(calT_EFF & 0x00ff);
    controlReg[0] = 0x08;
    controlReg[1] = 0x03;
    controlReg[2] = 0x00;

    pack_out[35] = controlReg[0];
    pack_out[36] = controlReg[1];
    pack_out[37] = controlReg[2];

    for(i = 0; i < 38; i++)
        pack_out[38] += pack_out[i];
    pack_out[39] = 0x16;

    UARTEnable(UART0_BASE );
    for(i = 0; i < 40; i++)
        UARTCharPut (UART0_BASE, pack_out[i]);
    UARTDisable(UART0_BASE );

}

//sent calibration timeout response to local server
static void zclCoordinator_calregtimeout()
{
    int i = 0;
    for(i = 0; i < 100; i++)
        pack_out[i] = 0;
    pack_out[0] = 0x68;
    pack_out[1] = 0;
    pack_out[2] = 0;
    pack_out[3] = 0;
    pack_out[4] = 0;
    pack_out[5] = 0;
    pack_out[6] = 0;
    pack_out[7] = 0;
    pack_out[8] = 0;
    pack_out[9] = 0x68;
    pack_out[10] = 0x1B;

    for(i = 11; i < 35; i++)
        pack_out[i] = 0;

    controlReg[0] = 0x08;
    controlReg[1] = 0x03;
    controlReg[2] = 0x02;

    pack_out[35] = controlReg[0];
    pack_out[36] = controlReg[1];
    pack_out[37] = controlReg[2];

    for(i = 0; i < 38; i++)
        pack_out[38] += pack_out[i];
    pack_out[39] = 0x16;

    UARTEnable(UART0_BASE );
    for(i = 0; i < 40; i++)
        UARTCharPut (UART0_BASE, pack_out[i]);
    UARTDisable(UART0_BASE );

}
