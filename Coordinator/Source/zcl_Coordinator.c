/**************************************************************************************************
  Filename:       zcl_Coordinator.c
  Revised:        $Date: 2014-10-24 11:49:27 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 1 $
  Description:    This device will act as a Coordinator.
**************************************************************************************************/
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
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
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
#include "hw_ints.h"
#include "hw_types.h"
#include "uart.h"
#include "MT.h"
#include "MT_UART.h"
/*********************************************************************
 * CONSTANTS
 */
#define USR_RX_GET   0xC0
#define USR_TX_GET   0xC1
#define USR_RX_SET   0xC2
#define USR_TX_SET   0xC3
#define COM_PARAM    0xC4
#define COM_DATA     0xC5
#define COM_ADD      0xC6
#define RESET        0xC7
#define RELAY        0xC8
#define START        0xC9
#define SET_PARAM    0xCA
#define FLASH_PARAM  0x0401
#define ACK_SUCCESS  0xCA
/*********************************************************************
 * MACROS
 */
/*********************************************************************
* Pin Definition
*/
//! This example uses the following peripherals and I/O signals.
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
#define EXAMPLE_GPIO_BASE               GPIO_A_BASE
/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclCoordinator_TaskID;
uint8 zclCoordinatorSeqNum;
uint16 paramReg[11];
uint16 SmartMeterparamReg[11]; //parameters sent by SmartMeter
uint16 dataReg_Ping[13], dataReg_Pong[13];
uint8 dataRegSel;
uint64 sm_ADD[100]; //smart meter address registers --100 max
uint16 sm_ADD_0[100];
uint16 sm_ADD_1[100];
uint16 sm_ADD_2[100];
uint16 sm_ADD_3[100];
uint16 sm_id;
uint16 sm_index;  //index for sm_Add[index]
uint16 index;  //general purpose index
uint16 sm_max = 0; //total number of smart meter
uint8 controlReg[14]; // Michelle: will change from 14 to 17
uint8 parameter_in[40] = {0};
uint8 sm_receive_flag[100] = {0};
uint8 sm_retry_times[100] = {5};
uint8 relay_receive_flag = 0; 
uint8 relay_retry_times = 0;  
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
uint8  Msg_in[100] = {0}; //for save UART DATA
uint16 MIN_ADC;
uint16 MAX_ADC;
uint16 SAMPLE_INT;
uint16 SAMPLE_WIN;
uint16 MAG_V;
uint16 MAG_I;
uint16 MAX_V;
uint16 MIN_V;
uint16 MAX_I;
uint16 MIN_I;
uint16 T_EFF;
uint16 ADD_3;
uint16 ADD_2;
uint16 ADD_1;
uint16 ADD_0;
extern mtOSALSerialData_t  *pMsg;
uint64 coordinator_extAddr;   //coordinator external IEEE address
uint16 coordinator_nwkAddr;  //coordinator network address
uint16 test_i = 1;
int    sm_flag_and = 1;
uint8  pack_out[100] = {0};
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
#endif
uint8 giThermostatScreenMode = COORDINATOR_MAINMODE;   // display the main screen mode first
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
static void zclCoordinator_BasicResetCB( void );
static void zclCoordinator_IdentifyCB( zclIdentify_t *pCmd );
static void zclCoordinator_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclCoordinator_ProcessIdentifyTimeChange( void );
static void zclscoordinator_startEZmode( void );
static void zclCoordinator_NetDiscov( void );
//Coordinator functions
static void zclCoordinator_SendData( void );
static void zclCoordinator_SetParam( void );
static void zclCoordinator_nvWriteParam( void );
static void zclCoordinator_nvReadParam( void );
static void zclCoordinator_SendAck( void );
static void zclCoordinator_SendRestart( void );
static void zclCoordinator_SendReset(void);
static void zclCoordinator_SendRelay(void);
void zclCoordinator_parameterInit(void);
void zclCoordinator_controlRegInit(void);
uint8 zclCoordinator_SmartMeterParamCompare(void);
void zclCoordinator_dataRegInit (void);
void zclCoordinator_sendACK(void); //send ACK to local server when command is done successfully
void zclCoordinator_sendRetry(void);//send Retry to local server when transmit error
uint32 BUILD_UINT32_16 (uint16 num1, uint16 numb2);
uint64 BUILD_UINT64_8 (uint8 numb1, uint8 numb2, uint8 numb3, uint8 numb4, uint8 numb5, uint8 numb6, uint8 numb7, uint8 numb8);
uint64 BUILD_UINT64_16 (uint16 numb1, uint16 numb2, uint16 numb3, uint16 numb4);
static void initUART(void); //Use UART.c functions directly
int16_t uint16ToInt16(uint16_t u_i);
uint16_t int16ToUint16(int16_t i);
// Functions to process ZCL Foundation incoming Command/Response messages
static void zclCoordinator_ProcessIncomingMsg( zclIncomingMsg_t *msg );
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
    initUART();  //Use UART funcitons directly
    //Initialize controlReg in the coordinator
    zclCoordinator_controlRegInit();
    //Initialize parameters in the coordinator
    zclCoordinator_parameterInit();
    //Initialize SmartMeter data register
    zclCoordinator_dataRegInit();
    zclscoordinator_startEZmode();
    // Get coordinator 64-bit IEEE external address and network address
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
    // event loop
    controlReg[0] = 0x08;
    controlReg[1] = 0x02;
}
/*********************************************************************
 * @fn          zclCoordinator_sendACK
 *
 * @brief       send ACK to local server when command is done successfully.
 *
 * @param       none
 *
 * @return      none
 */
void zclCoordinator_sendACK(void)
{
        int i;
        pack_out[0] = 0x68;
        for(i = 1; i <= 8; i++)
            pack_out[i] = 0;
        pack_out[9] = 0x68;
        pack_out[10] = 0x02;
        controlReg[0] = 0x08; // reset controlReg[0] to defult
        controlReg[1] = 0x02;// reset controlReg[1] to defult
        pack_out[11] = controlReg[0];
        pack_out[12] = controlReg[1];
        for(i = 0; i < 13; i++)
            pack_out[13] += pack_out[i];
        pack_out[14] = 0x16;
        UARTEnable(UART0_BASE );
        for (i = 0; i < 15; i++)
        {
            UARTCharPut(UART0_BASE, pack_out[i]);
        }
        UARTDisable(UART0_BASE);
}
/*********************************************************************
 * @fn          zclCoordinator_sendRetry
 *
 * @brief       send Retry to local server when transmit error.
 *
 * @param       none
 *
 * @return      none
 */
void zclCoordinator_sendRetry(void)
{
        int i;
        pack_out[0] = 0x68;
        for(i = 1; i <= 8; i++)
            pack_out[i] = 0;
        pack_out[9] = 0x68;
        pack_out[10] = 0x02;
        controlReg[0] = 0x08; // reset controlReg[0] to defult
        controlReg[1] = 0x06;// set retry bit to 1
        pack_out[11] = controlReg[0];
        pack_out[12] = controlReg[1];
        for(i = 0; i < 13; i++)
            pack_out[13] += pack_out[i];
        pack_out[14] = 0x16;
        UARTEnable(UART0_BASE );
        for (i = 0; i < 15; i++)
        {
            UARTCharPut(UART0_BASE, pack_out[i]);
        }
        UARTDisable(UART0_BASE);
}
/**************************************************
 * @fn          ProcessUartData
 *
 * @brief       
 *
 * @param       mtOSALSerialData_t
 *
 * @return      none
********************************************************/
void ProcessUartData( mtOSALSerialData_t *Uart_Msg)                   
{
    int index;
    uint8 dataLen;  //length of the packdge
    dataLen = Uart_Msg->msg[0];
    Msg_in[0] = 0x68;
    for (index = 1; index <= 8; index++)
        Msg_in[index] = 0x00;
    Msg_in[9] = 0x68;
    if(dataLen == 0x31)
        Msg_in[10] = 0x0E;
    else
        Msg_in[10] = dataLen;
    Msg_in[11] = Uart_Msg->msg[1] ;
    for (index = 1; index < (int)(dataLen + 2); index++)
    {
        Msg_in[index + 11] = Uart_Msg->msg[index + 1] ;
    }
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
#ifdef LCD_SUPPORTED
    sprintf((char *)lcdString, "test_i: %d", (uint8)(test_i&0xFF) );
    HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
    test_i++;
#endif
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
            case ZCL_INCOMING_MSG:
                // Incoming ZCL Foundation command/response messages
                zclCoordinator_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
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
                    giThermostatScreenMode = COORDINATOR_MAINMODE;
#endif
#ifdef ZCL_EZMODE
                    zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif  // ZCL_EZMODE
               }
                break;
            case CMD_SERIAL_MSG:                                           //ZcomDef.h 10.8
                ProcessUartData((mtOSALSerialData_t *)pMsg);
                osal_set_event(task_id,Coordinator_UART_EVT);
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
    int i;
    for(i=0; i<100; i++)
        pack_out[i] = 0;  
    uint16 checksum = 0;
    controlReg[0] = 0x08;
    controlReg[1] = 0x02;   
    if(Msg_in[0])   //if there is message in
    {      
        int index;
        checksum = 0;
        for (index = 0; index < 25; index++)
        {
            checksum += Msg_in[index];
        } 
        if((uint8)(checksum & 0xFF) != Msg_in[25] || Msg_in[26] != 0x16)    //if sum is wrong or end flag is not 0x16
        {
            zclCoordinator_sendRetry();   
        }
        else if (((Msg_in[11] >> 1) & 0x01) == 1)             //if success in reading controlReg and the command is set parameter,then continue to read the parameter
        {        
            int i;
            checksum = 0;
            for (index = 0; index < 33; index++)
            {
                checksum += Msg_in[index+27];
            }
            if((uint8)(checksum & 0xFF) != Msg_in[60] || Msg_in[61] != 0x16)  //if sum is wrong or end flag is not 0x16
            {                                         
                zclCoordinator_sendRetry();      
            }  
            else   //if no problem with the package, get parameter out
            {
                for(i = 0;i < 35; i++)
                    parameter_in[i] = Msg_in[i+27];
            }
        }     
    }
    if ( (events & Coordinator_UART_EVT ) && ((controlReg[1] & 0x04 ) != 0x04))  //make sure retry bit is not 0
    {
        // command: control register write
        // Write to controlReg when controlReg write is disabled  *
        // This is the normal write operation from server
        if (((controlReg[1] >> 1) & 0x01) == 1)
        {
            int i;           
            for(i = 0 ; i < 14; i++)
            {
                controlReg[i] = Msg_in[i + 11];
            }                        
            controlReg[1] = controlReg[1] | 0x02;//RESET TO DEFULT             
            for(i=0; i< 100; i++)
                Msg_in[i] = 0;
        }
        // command: parameter read
        // Send current parameter to server if parameter read is enabled (bit 0=1)  *
        // Reset this parameter to its default state when operation is finished.
        if ((controlReg[0] & 0x01) == 1)
        {
            controlReg[1] = controlReg[1] & 0xFD;
            pack_out[0] = 0x68;
            int i;
            for(i = 1; i <= 8; i++)
                pack_out[i] = 0;
            pack_out[9] = 0x68;
            pack_out[10] = 0x18;
            for (i = 0; i < 11; i++)
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
            controlReg[1] = controlReg[1] | 0x02;
            pack_out[33] = controlReg[0];
            pack_out[34] = controlReg[1];
            pack_out[35] = 0;
            for(i = 0; i < 35; i++)
                pack_out[35] += pack_out[i];
            pack_out[36] = 0x16;
            UARTEnable(UART0_BASE );
            for (i = 0; i < 37; i++)
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
            for (i = 0; i < 11; i++)
            {
                paramReg[i] = (uint16)(((uint16)(parameter_in[i * 2 + 11])) << 8) + (uint16)(parameter_in[i * 2 + 12]);
            }
            //write parameters into FLASH memory of coordaintor
            zclCoordinator_nvWriteParam();
            UARTEnable(UART0_BASE );
            UARTCharPut(UART0_BASE, 0x11 );
            UARTCharPut(UART0_BASE, (uint8)sm_max );
            UARTCharPut(UART0_BASE, 0x11 );
            UARTDisable(UART0_BASE);        
            //write parameters to all the smart meters
            for (sm_index = 0; sm_index < sm_max; sm_index++)
            {
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_index]) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_index]) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_index]) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_index]) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_index]) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_index]) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_index]) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_index]) & 0x00000000000000FF);
                zclCoordinator_SetParam();
            }       
            osal_set_event(task_id,Coordinator_ProcessParaSet_EVT);
        }         
        // command: energy calculation reset
        // Reset energy calculation if bit 2 is 1   *
        // Reset this bit to its default value when operation is finished
        else if (((controlReg[0] >> 2) & 0x01) == 1)
        {
            RM_ADD = BUILD_UINT64_8(controlReg[6], controlReg[7], controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13]);
            zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((RM_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
            zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((RM_ADD) >> 48) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((RM_ADD) >> 40) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((RM_ADD) >> 32) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((RM_ADD) >> 24) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((RM_ADD) >> 16) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((RM_ADD) >> 8) & 0x00000000000000FF);
            zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((RM_ADD) & 0x00000000000000FF);
            flagreset = (controlReg[0] >> 2) & 0xFF;
            ENERGY_RESET_VALUE = BUILD_UINT32(controlReg[2], controlReg[3], controlReg[4], controlReg[5]);
            zclCoordinator_SendReset();
            int delay_x;
            int delay_y;
            for(delay_x = 0; delay_x < 1000; delay_x++)
                for(delay_y = 0; delay_y < 1000; delay_y++);
            if (!(SmartMeter_ENERGY_RESET_VALUE == ENERGY_RESET_VALUE))
            {
            }
            //send ACK to local server
            zclCoordinator_sendACK(); 
            controlReg[0] = controlReg[0] & 0xFB; // reset bit 2 to 0
        }
         // command: relay control
         // Switch relay off if bit 3 is set to 0           *
         // Reset this bit to its default value when operation is finished
         else if ((controlReg[0] & 0xF7) == 0)  // except bit3, other bits are all 0
          {          
              if (((controlReg[0] >> 3) & 0x01) == 0)  // power relay off (disconnect line power)
              {
                  RM_ADD = BUILD_UINT64_8(controlReg[6], controlReg[7], controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13]);
                  zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((RM_ADD)>>56)&0x00000000000000FF); 
                  zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((RM_ADD)>>48)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((RM_ADD)>>40)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((RM_ADD)>>32)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((RM_ADD)>>24)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((RM_ADD)>>16)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((RM_ADD)>>8)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((RM_ADD)&0x00000000000000FF);                  
                  flagrelay = 0;
                  zclCoordinator_SendRelay();                  
                  osal_set_event(task_id, Coordinator_RelaySet_EVT);
              }              
              else // power relay on (connect to line power)
              {
                  RM_ADD = BUILD_UINT64_8(controlReg[6], controlReg[7], controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13]);
                  zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((RM_ADD)>>56)&0x00000000000000FF); 
                  zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((RM_ADD)>>48)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((RM_ADD)>>40)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((RM_ADD)>>32)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((RM_ADD)>>24)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((RM_ADD)>>16)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((RM_ADD)>>8)&0x00000000000000FF);
                  zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((RM_ADD)&0x00000000000000FF);                 
                  flagrelay = 1;
                  zclCoordinator_SendRelay();                 
                  osal_set_event(task_id, Coordinator_RelaySet_EVT);
              }             
          }
        // command: network discovery
        // Perform network discovery if bit 4 is 1           *
        //  Reset this bit to its default value when operation is finished
        else if (((controlReg[0] >> 4) & 0x01) == 1)
        {
            zclCoordinator_NetDiscov();
            //Wait 5 seconds for all the smart meters to respond to network discovery command
            //Each smart meter will send its IEEE address to coordinator during this time
            //A routing table sm_Addr[index] will be built.
            //The maximum number of smart meter is sm_max.
            zclCoordinator_sendACK();
        }
        // command: routing table read
        // Send routing table when bit 5 is set to 1                 *
        // Reset this bit to its default value when operation is finished
        else if (((controlReg[0] >> 5) & 0x01) == 1)
        {
            for (sm_index = 0; sm_index < sm_max; sm_index++)
            {
                int i;
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
                pack_out[10] = 0x0A;
                pack_out[11] = (uint8)((sm_ADD_3[sm_index] >> 8) & 0x00FF);
                pack_out[12] = (uint8)(sm_ADD_3[sm_index] & 0x00FF);
                pack_out[13] = (uint8)((sm_ADD_2[sm_index] >> 8) & 0x00FF);
                pack_out[14] = (uint8)(sm_ADD_2[sm_index] & 0x00FF);
                pack_out[15] = (uint8)((sm_ADD_1[sm_index] >> 8) & 0x00FF);
                pack_out[16] = (uint8)(sm_ADD_1[sm_index] & 0x00FF);
                pack_out[17] = (uint8)((sm_ADD_0[sm_index] >> 8) & 0x00FF);
                pack_out[18] = (uint8)(sm_ADD_0[sm_index] & 0x00FF);
                controlReg[0] = controlReg[0] & 0xDF; //reset bit 5 to 0
                pack_out[19] = controlReg[0];
                pack_out[20] = controlReg[1];
                for(i = 0; i < 21; i++)
                    pack_out[21] += pack_out[i];
                pack_out[22] = 0x16;
                UARTEnable(UART0_BASE ); 
                for(i = 0; i < 23; i++)
                    UARTCharPut (UART0_BASE, pack_out[i]);
                UARTDisable(UART0_BASE );
            }  
            controlReg[0] = controlReg[0] & 0xDF; //reset bit 5 to 0
        }
        // command: control register read
        // Send current control register to server if controlReg read is enabled (bit 6=1) *
        // Reset this parameter to its default state when operation is finished.
        else if (((controlReg[0] >> 6)  & 0x01) == 1)
        {
            controlReg[1] = controlReg[1] & 0xFB; //reset byte 1 bit 0 to 0, controlReg not writable
            pack_out[0] = 0x68;
            int i;
            for(i = 1; i <= 8; i++)
                pack_out[i] = 0;
            pack_out[9] = 0x68;
            pack_out[10] = 0x10;
            for(i = 11; i <= 24; i++)
                pack_out[i] = controlReg[i - 11];
            controlReg[0] = controlReg[0] & 0xBF; //reset bit 6 to 0
            controlReg[1] = controlReg[1] | 0x02; //reset byte1 bit 1 to 1
            pack_out[25] = controlReg[0];
            pack_out[26] = controlReg[1];
            pack_out[27] = 0;
            for(i = 0; i < 27; i++)
                pack_out[27] += pack_out[i];
            pack_out[28] = 0x16;
            UARTEnable(UART0_BASE );
            for (i = 0; i < 29; i++)
            {
                UARTCharPut(UART0_BASE, pack_out[i]);
            }
            UARTDisable(UART0_BASE);
        }
        //command: data register read
        //Send request for smart meter data using round robin method
        //Specify which smart meter to request data
        else if (((controlReg[0] >> 7) & 0x01) == 1)
        {
            for (sm_index = 0; sm_index < sm_max; sm_index++)
            {
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD[sm_index]) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD[sm_index]) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD[sm_index]) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD[sm_index]) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD[sm_index]) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD[sm_index]) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD[sm_index]) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD[sm_index]) & 0x00000000000000FF);
                UARTEnable(UART0_BASE );
                UARTCharPut(UART0_BASE, 0x87);
                UARTDisable(UART0_BASE );
                //Stop power calculation
                flaginc = 0;
                zclCoordinator_SendRestart();
                //Send request to smart meter to send data
                datain_complete = 0; //reset datain_complete
                zclCoordinator_SendData();   //send request to get data
            }
            // UART begin
            //While waiting for data send from smart meter, send data from other dataReg
            //
            pack_out[0] = 0x68;
            int i;
            for(i = 1; i <= 8; i++)
                pack_out[i] = 0;
            pack_out[9] = 0x68;
            pack_out[10] = 0x45;
            UARTEnable(UART0_BASE );
            if (dataRegSel == 1)
            {
                for (index = 0; index < 26; index++)
                {
                    value = dataReg_Pong[i];
                    u_value = int16ToUint16(value);
                    // uint16_t into two uint8_t first
                    u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                    u_partB = (uint8_t) (u_value & 0x00FF);
                    pack_out[11 + i * 2] = u_partA;
                    pack_out[12 + i * 2] = u_partB;
                }
            }
            if (dataRegSel == 0)
            {
                for (index = 0; index < 26; index++)
                {
                    value = dataReg_Ping[i];
                    u_value = int16ToUint16(value);
                    // uint16_t into two uint8_t first
                    u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                    u_partB = (uint8_t) (u_value & 0x00FF);
                    pack_out[11 + i * 2] = u_partA;
                    pack_out[12 + i * 2] = u_partB;
                }
            }
            controlReg[0] = controlReg[0] & 0x7F; //reset bit 7 to 0
            controlReg[1] = controlReg[1] | 0x02;
            pack_out[66] = controlReg[0];
            pack_out[67] = controlReg[1];
            pack_out[68] = 0;
            for(i = 0; i < 68; i++)
                pack_out[68] += pack_out[i];
            pack_out[69] = 0x16;
            UARTEnable(UART0_BASE );
            for (i = 0; i < 70; i++)
            {
                UARTCharPut(UART0_BASE, pack_out[i]);
            }
            UARTDisable(UART0_BASE );
            dataRegSel =  dataRegSel ^ 0x01 ; //toggle dataRegSel
            //Restart power calculation
            flaginc = 1;
            zclCoordinator_SendRestart();
            sm_index++;
            if (sm_index == sm_max)
            {
                sm_index = 0;
            }
        }                                   
        return ( events ^ Coordinator_UART_EVT );
    }    
    if ( (events & Coordinator_ProcessParaSet_EVT )&&(test_i != 0xFFFF))
    {      
        int index;       
        for(index=0; index<sm_max; index++)
            sm_flag_and = sm_flag_and && sm_receive_flag[index];                         
        if(sm_flag_and&&sm_max)
        {
            zclCoordinator_sendACK();     
            for(i = 0; i < 100; i++)
            {
                 sm_receive_flag[i] = 0;
            }
        }     
        else
        {
            sm_flag_and = 1;
            osal_set_event(task_id, Coordinator_ProcessParaSet_EVT);
        }          
        return ( events ^ Coordinator_ProcessParaSet_EVT );
    }
    if ( (events & Coordinator_RelaySet_EVT )&&(test_i != 0xFFFF))
    {    
        if (relay_receive_flag)
        {
            zclCoordinator_sendACK();
            relay_receive_flag = 0;                      
        }
        else
        {            
            osal_set_event(task_id, Coordinator_RelaySet_EVT);
        }        
        return ( events ^ Coordinator_RelaySet_EVT );
    }     
    return 0;
}
void zclscoordinator_startEZmode( void )
{
#ifdef ZCL_EZMODE
    zclEZMode_InvokeData_t ezModeData;
    static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_HVAC_EMS_COORDINATOR };   // only bind on the COORDINATOR cluster //zcl.h
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
    ezModeData.initiator = TRUE;        // Coordinator is an initiator
    ezModeData.numActiveInClusters = 0;
    ezModeData.pActiveInClusterIDs = NULL;
    ezModeData.numActiveOutClusters = 1;   // active output cluster //1
    ezModeData.pActiveOutClusterIDs = clusterIDs;
    zcl_InvokeEZMode( &ezModeData );
#endif // ZCL_EZMODE
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
    }
    else
    {
        osal_stop_timerEx( zclCoordinator_TaskID, Coordinator_IDENTIFY_TIMEOUT_EVT );
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
#ifdef ZCL_REPORT
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
#ifdef ZCL_REPORT
/*********************************************************************
 * @fn      zclCoordinator_ProcessInReportCmd
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
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_PARAMETER_MEASURED_VALUE))
    {
        // store the current parameter value sent over the air from smart meter *
        SmartMeterparamReg[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]); //MIN_ADC
        SmartMeterparamReg[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]); //MAX_ADC
        SmartMeterparamReg[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]); //SAMPLE_INT
        SmartMeterparamReg[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]); //SAMPLE_WIN
        SmartMeterparamReg[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]); //MAG_V
        SmartMeterparamReg[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]); //MAG_I
        SmartMeterparamReg[6] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]); //MIN_V
        SmartMeterparamReg[7] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]); //MAX_V
        SmartMeterparamReg[8] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20], pInParameterReport->attrList[0].attrData[21]); //MIN_I
        SmartMeterparamReg[9] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22], pInParameterReport->attrList[0].attrData[23]); //MAX_I
        SmartMeterparamReg[10] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24], pInParameterReport->attrList[0].attrData[25]); //T_EFF
        sm_id = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26], pInParameterReport->attrList[0].attrData[27]);              
        if(!zclCoordinator_SmartMeterParamCompare())
        {    
            sm_receive_flag[sm_id] = 1;
            UARTEnable(UART0_BASE ); 
            UARTCharPut (UART0_BASE, 0x77);
            UARTCharPut (UART0_BASE, 0x77);
            UARTCharPut (UART0_BASE, (uint8)sm_id);
            UARTDisable(UART0_BASE );
        }   
        else 
        {
            if(!sm_retry_times[sm_id])
            {
                sm_receive_flag[sm_id] = 2; //means time out
                sm_retry_times[sm_id] = 5;  //reset retry times
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
    //Process incomming data report from smart meter in response to GET date command *
    if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_DATA_MEASURED_VALUE))
    {
        uint16 smADD_3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        uint16 smADD_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        uint16 smADD_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        uint16 smADD_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        uint16 RMS_V = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        uint16 RMS_I = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        uint16 POWER1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);
        uint16 POWER0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]);
        uint16 ENERGY1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20], pInParameterReport->attrList[0].attrData[21]);
        uint16 ENERGY0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22], pInParameterReport->attrList[0].attrData[23]);
        uint16 SM_V = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24], pInParameterReport->attrList[0].attrData[25]);
        uint16 SM_I = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26], pInParameterReport->attrList[0].attrData[27]);
        uint16 STATUS = BUILD_UINT16(pInParameterReport->attrList[0].attrData[28], pInParameterReport->attrList[0].attrData[29]);
        // store the current data value sent over the air from smart meter
        if (dataRegSel == 0)
        {
            dataReg_Pong[0] = smADD_3; //ADD_3
            dataReg_Pong[1] = smADD_2; //ADD_2
            dataReg_Pong[2] = smADD_1; //ADD_1
            dataReg_Pong[3] = smADD_0; //ADD_0
            dataReg_Pong[4] = RMS_V;
            dataReg_Pong[5] = RMS_I;
            dataReg_Pong[6] = POWER1;
            dataReg_Pong[7] = POWER0;
            dataReg_Pong[8] = ENERGY1;
            dataReg_Pong[9] = ENERGY0;
            dataReg_Pong[10] = SM_V;
            dataReg_Pong[11] = SM_I;
            dataReg_Pong[12] = STATUS;
        }
        else  if (dataRegSel == 1)
        {
            dataReg_Ping[0] = smADD_3; //ADD_3
            dataReg_Ping[1] = smADD_2; //ADD_2
            dataReg_Ping[2] = smADD_1; //ADD_1
            dataReg_Ping[3] = smADD_0; //ADD_0
            dataReg_Ping[4] = RMS_V;
            dataReg_Ping[5] = RMS_I;
            dataReg_Ping[6] = POWER1;
            dataReg_Ping[7] = POWER0;
            dataReg_Ping[8] = ENERGY1;
            dataReg_Ping[9] = ENERGY0;
            dataReg_Ping[10] = SM_V;
            dataReg_Ping[11] = SM_I;
            dataReg_Ping[12] = STATUS;
        }
        datain_complete = 1;  //set flag to indicate all data have been received
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
        sm_index++; //increment index
        sm_max++; //total number of smart meter detected
    }
    // Process incomming energy reset report from smart meter in response to RESET command *
    if ((OPERATION == RESET) && (RESULT == SUCCESS) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        SmartMeter_flagreset = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        ENERGY_RESET_VALUE_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        ENERGY_RESET_VALUE_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        SmartMeter_ENERGY_RESET_VALUE = BUILD_UINT32_16(ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0);
    }
    // Process incomming relay report from smart meter in response to RELAY command *
    if ((OPERATION == RELAY) && (RESULT == SUCCESS)  &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        SmartMeter_relay = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        UARTEnable(UART0_BASE );        
        UARTCharPut(UART0_BASE, 0x22);    
        UARTCharPut(UART0_BASE, (uint8)(SmartMeter_relay&0xFF));       
        UARTCharPut(UART0_BASE, (uint8)(flagrelay&0xFF)); 
        UARTDisable(UART0_BASE);
        if (SmartMeter_relay == flagrelay)
            relay_receive_flag = 1;
        else
        {
            if (!relay_retry_times)
            {
                relay_receive_flag = 2; //means times out
                relay_retry_times = 5;  //reset retry times
            }           
            else  //if retry times is not 0
            {    
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((RM_ADD)>>56)&0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((RM_ADD)>>48)&0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((RM_ADD)>>40)&0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((RM_ADD)>>32)&0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((RM_ADD)>>24)&0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((RM_ADD)>>16)&0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((RM_ADD)>>8)&0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((RM_ADD)&0x00000000000000FF);
                zclCoordinator_SendRelay();              
                relay_retry_times--;                         
            } 
        }      
    }
    // Process incomming restart report from smart meter in response to RESTART command *
    if ((OPERATION == START) && (RESULT == SUCCESS)  &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        SmartMeter_flaginc = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
    }
}
#endif  // ZCL_REPORT
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
    MIN_ADC = paramReg[0];
    MAX_ADC = paramReg[1];
    SAMPLE_INT = paramReg[2];
    SAMPLE_WIN = paramReg[3];
    MAG_V = paramReg[4];
    MAG_I = paramReg[5];
    MIN_V = paramReg[6];
    MAX_V = paramReg[7];
    MIN_I = paramReg[8];
    MAX_I = paramReg[9];
    T_EFF = paramReg[10];
    uint16 packet[] = {USR_RX_SET, SET_PARAM, MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN,
                       MAG_V, MAG_I, MIN_V, MAX_V, MIN_I, MAX_I, T_EFF, sm_index
                      };
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT256;
        pReportCmd->attrList[0].attrData = (void *)(packet);
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
    uint16 energy_reset_value_1;
    uint16 energy_reset_value_0;
    energy_reset_value_1 = (uint16)((ENERGY_RESET_VALUE >> 16) & 0xFFFF);
    energy_reset_value_0 = (uint16)(ENERGY_RESET_VALUE & 0xFFFF);
    uint16 packet[] = {RESET, flagreset, energy_reset_value_1, energy_reset_value_0 };
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT64;
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
static void zclCoordinator_NetDiscov( void )
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
static void zclCoordinator_SendAck( void )
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
    paramReg[0] = 0;
    paramReg[1] = 1023;
    paramReg[2] = 1000; //Michelle: changed from 1000us t 1ms
    paramReg[3] = 60000; //Michelle: changed from 60000us to 60ms
    paramReg[4] = 1;
    paramReg[5] = 100;
    paramReg[6] = 110;
    paramReg[7] = 110; //Michelle: test: changed from -110 to 50 because of type
    paramReg[8] = 2;
    paramReg[9] = 2; //Michelle: test: changed from -2 to 0
    paramReg[10] = 200;
    MIN_ADC = paramReg[0];
    MAX_ADC = paramReg[1];
    SAMPLE_INT = paramReg[2];
    SAMPLE_WIN = paramReg[3];
    MAG_V = paramReg[4];
    MAG_I = paramReg[5];
    MIN_V = paramReg[6];
    MAX_V = paramReg[7];
    MIN_I = paramReg[8];
    MAX_I = paramReg[9];
    T_EFF = paramReg[10];
    index = 0;
    sm_index = 0;
    dataRegSel = 0;
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
// conversion between uint16_t to int16_t
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
    uint16 *flashpt;
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
    uint8 num = 0;
    int index;
    for (index = 0; index < 11; index++)
    {
        if (SmartMeterparamReg[index] == paramReg[index])
            num++;
    }
    if (num == 11)
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
    for (i = 0; i < 13; i++)
    {
        dataReg_Pong[i] = 0;
        dataReg_Ping[i] = 0;
    }
}
static void initUART(void)
{
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);
    // Set IO clock to the same as system clock
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
    // Enable UART peripheral module
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);
    // Disable UART function
    UARTDisable(UART0_BASE);
    // Disable all UART module interrupts
    UARTIntDisable(UART0_BASE, 0x1FFF);
    // Set IO clock as UART clock source
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Map UART signals to the correct GPIO pins and configure them as
    // hardware controlled.
    IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_TXD);
    IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD, IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE, EXAMPLE_PIN_UART_RXD);
    // Configure the UART for 115,200, 8-N-1 operation.
    // This function uses SysCtrlClockGet() to get the system clock
    // frequency.  This could be also be a variable or hard coded value
    // instead of a function call.
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
            if ( giThermostatScreenMode == COORDINATOR_MAINMODE )
            {    HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
            }
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
            if ( giThermostatScreenMode == COORDINATOR_MAINMODE )
            {
                HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
            }
        }
#endif  // LCD_SUPPORTED
        zclCoordinator_NetDiscov();
    }
}
#endif // ZCL_EZMODE
