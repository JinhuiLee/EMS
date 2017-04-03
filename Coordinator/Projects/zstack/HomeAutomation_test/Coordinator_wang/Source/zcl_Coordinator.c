/**************************************************************************************************
  Filename:       zcl_Coordinator.c
  Revised:        $Date: 2015-2-10 17:02:21 -0700 $
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
/*
#define EXAMPLE_PIN_UART0_RXD            GPIO_PIN_0
#define EXAMPLE_PIN_UART0_TXD            GPIO_PIN_1
#define EXAMPLE_GPIO_BASE0               GPIO_A_BASE

#define EXAMPLE_PIN_UART1_RXD            GPIO_PIN_2  //RF1.2
#define EXAMPLE_PIN_UART1_TXD            GPIO_PIN_3  //RF1.4
#define EXAMPLE_GPIO_BASE1               GPIO_C_BASE
*/
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
#include "aes.h"
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
#include "OSAL_Clock.h"

#include "string.h"
#include "usb_firmware_library_headers.h"
#include "usb_cdc.h"
#include "usb_in_buffer.h"
#include "usb_out_buffer.h"
#include "rom.h"

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
#define CALIBRATE    0xCC
#define TIME_SET     0xD0
#define COM_CAL      0xD1
#define COM_CONFIG   0xD2
#define TEMP_STOP    0xD5
#define AUTHEN       0xD6
#define SET_KEY      0xD7

#define FLASH_PARAM  0x1000
#define FLASH_RO     0x1100
#define UINT8_TO_16(hiByte, loByte) \
        ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))


// wait time for all smart meter to respond to network discovery command
#define Coordinator_NwkDiscov_INTERVAL 5000 //in millisecond
// wait time to request the next smart meter to send data
#define Coordinator_SendData_INTERVAL  1000 //in millisecond

//wireless or wired connection switch
//#define Connect_Mode WIRED_CONNECTION
#define Connect_Mode WIRELESS_CONNECTION

#define NUM_SMART_METER  5
/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclCoordinator_TaskID;
uint8 zclCoordinatorSeqNum;
uint16 WAIT_EVT_INDEX = 0x0000;
uint8 USB_Msg_in[110] = {0};
uint16 paramReg[25] = {0};
uint8 sm_address_buffer[8] = {0};
uint8 send_buffer[70] = {0};
extern uint8 usb_end_flag;
extern uint8 *usb_alloc_buf;
uint16 SmartMeterparamReg[25]; //parameters send by SmartMeter
uint16 SmartMeterTimeReg[10]; //Time send by SmartMeter
uint16 dataReg_Ping[29] = {0};
uint16 dataReg_Pong[29] = {0};
uint8 dataRegSel = 1;
uint8 PowSpCal = 0;
uint8 smart_auth_index = 0;

typedef struct
{
    uint64 sm_ADD;
    uint8 sm_ADD_status;
    uint8 freq;
    uint64 sm_key_table_hi;
    uint64 sm_key_table_lo;
    uint64 sm_rom_table_hi;
    uint64 sm_rom_table_lo;
    uint64 final_key_table_hi;
    uint64 final_key_table_lo;
    bool flag_end_encry;
} r_table;
r_table routing_table[NUM_SMART_METER];
/*
uint64 sm_ADD[NUM_SMART_METER] = {0}; //smart meter address registers --350 max
uint8 sm_ADD_status[NUM_SMART_METER];
uint8 freq[NUM_SMART_METER] = {0};
uint64 final_key_table_hi[NUM_SMART_METER] = {0};
uint64 final_key_table_lo[NUM_SMART_METER] = {0};
uint64 sm_rom_table_hi[NUM_SMART_METER] = {0};
uint64 sm_rom_table_lo[NUM_SMART_METER] = {0};
uint64 sm_key_table_hi[NUM_SMART_METER] = {0};
uint64 sm_key_table_lo[NUM_SMART_METER] = {0};
bool flag_end_encry[NUM_SMART_METER] = {false};
*/
uint8 coor_addrRegister[8];
uint64 sm_ADD_reg = 0;
uint8 V_CAL = 0;
uint8 I_CAL = 0;
uint8 T_CAL = 0;
uint8 N_CAL = 0;
uint8 INPUT_1_CAL = 0;
uint8 INPUT_2_CAL = 0;

UTCTimeStruct TimeStruct;
uint32 sys_secold = 0;
uint32 sys_secnew = 0;
uint32 sys_timeold = 0;
uint32 sys_timenew = 0;

uint8 sm_ROM_index = 0;
uint16 sm_id;
uint16 sm_index = 0;  //index for routing_table[index].sm_ADD
uint16 index;  //general purpose index
uint8 sm_max = 0; //total number of smart meter
uint16 num_prio_sm_max = 0;
uint8 controlReg[45] = {0};
uint8 num_high_prio = 0;
uint8 try_count = 0;
uint16 timeReg[6] = {0};
//uint8 sm_receive_flag[20] = {0};
//uint8 sm_retry_times[20] = {3};
uint8 relay_receive_flag = 0;
uint8 relay_retry_times = 3;
uint8 ack_index = 0;
uint8 NetDiscov_flag = 0;
uint8 Routingtable_flag = 0;
uint16 Drr_retry_cnt = 0;
uint8  Drr_flag = 1;
uint8  ACK_flag = 0;
uint8  Timeout_Ping = 0;
uint8  Timeout_Pong = 0;
uint16 Drr_Retry_Zero = 0;
uint16 first_write_flag = 1;  //write both ping and pong with SM1 and SM2
uint16 first_complete = 1;
uint8  Timeout_bit = 0;
uint8 all_invaild_flag = 1;
uint8 com_add_rcv_flag = 0;
bool cal_receive_flag = 0;
uint8 para_set_rcv_flag = 0;
uint32 ENERGY_RESET_VALUE = 0xffffffff;
uint32 SmartMeter_ENERGY_RESET_VALUE = 0xfffffffe; //smart meter response to RESET
uint64 RM_ADD;
uint16 flagreset = 0;
uint16 flagrelay = 2;
uint16 flaginc;
uint16 start = 2; //for the power calculation control. added by xu.
uint16 datain_complete = 0; //read of smart meter data completed
uint16 SmartMeter_flagreset; //smart meter response to RESET
uint16 SmartMeter_relay = 0;  //smart meter response to RELAY
uint16 SmartMeter_flaginc; //smart meter response to RESTART
uint16 len_DataReg = 0;
uint8 Round_end_flag = 0;
uint8 flag_config_reg = 0;
uint8 routing_index = 0;
uint8 setpower_index = 0;
uint8 routing_all_flag = 0;
uint8 len_uart1 = 0;
uint8 SmartMeter_start = 2;
uint8 start_receive_flag = 0;
uint8 write_energy_index = 0;
bool flag_encry = false;
uint8 coor_key[16] = {0x00};
uint8 romread[16] = {0x00};
uint8 coor_final_key[16] = {0x00};
uint8 end_final_key[16] = {0x00};

bool flag_auth = false;

uint16 MIN_ADC;
uint16 MAX_ADC;
uint16 SAMPLE_INT;
uint16 SAMPLE_WIN;
/*
uint16 MAG_V1;
uint16 MAG_I1;
uint16 MAG_V2;
uint16 MAG_I2;
uint16 MAG_V3;
uint16 MAG_I3;
*/
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

uint16 SM_CONFIG_2 = 0;
uint16 SM_CONFIG_1 = 0;
uint16 SM_CONFIG_0 = 0;

uint16 calsm_id;
uint16 calSMADD_3;
uint16 calSMADD_2;
uint16 calSMADD_1;
uint16 calSMADD_0;
uint16 calMAG_V[8] = {0};
uint16 calMAG_I[8] = {0};
uint16 calMAG[8] = {0};
uint16 calT_EFF;

uint8 dataLen;  //length of the packdge
uint8 flag_calget = 0;
bool flag_key_set = false;

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
uint32 time_old = 0;
uint32 time_new = 0;
uint8 cmd_right_flag = 1;
uint8 process_uart_index = 0;

uint32 switch_timenew = 0;
uint8 first_time_in = 1;
uint8 second_time_in = 0;

uint8 sm_routing_prio_table[3 * NUM_SMART_METER] = {0};
//uint64 sm_ADD_prio = 0;
////////////////////////////////////////////////////usb
USB_EPIN_RINGBUFFER_DATA usbCdcInBufferData;
USB_EPOUT_RINGBUFFER_DATA usbCdcOutBufferData;
uint8_t pInBuffer[128];
uint8_t pOutBuffer[128];
uint8_t pAppBuffer[128];

//uint8_t ui8AESKey[16] = {0};
uint8 end_index = 0;
uint8 end_i = 0;
bool flag_reset_all_smart_meter = false;
bool flag_reset_smart_meter = false;
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
#define ZCLCoordinator_BINDINGLIST_IN      6

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
static uint8_t AesEncryptDecrypt(uint8_t *pui8Key, uint8_t *pui8Buf, uint8_t ui8KeyLocation, uint8_t ui8Encrypt);

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
static void zclCoordinator_SetPowerCalculation( void );
static void zclCoordinator_SendAuth (void);
static void zclCoordinator_SetKey (uint8 *);
// app display functions
void zclCoordinator_LCDDisplayUpdate(void);
void zclCoordinator_LcdDisplayTestMode(void);
void zclCoordinator_LcdDisplayTestMode_smaddr(void);
void initUSB(void);

//Coordinator functions
static void zclCoordinator_SendData( void );
static void zclCoordinator_SetParam( void );
static void zclCoordinator_SetTime();
static void zclCoordinator_nvWriteParam( void );
static void zclCoordinator_nvReadParam( void );
static void zclCoordinator_SendAck( void );
static void zclCoordinator_SendRestart( void );
static void Process_Wired_Cmd(void);
static void route_table_add(r_table *routing_table, r_table routing_single);
static void route_table_delete(r_table *routing_table, r_table routing_single);
static void zclCoordinator_SendReset(void);
static void zclCoordinator_SendRelay(void);
static void zclCoordinator_SendCalibrate(void);
static void zclCoordinator_sendcalreg(void);
static void zclCoordinator_setconfigReg(void);

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
void zclCoordinator_sendACK(uint8 ControlReg0, uint8 ControlReg1, uint8 ControlReg2, uint8 ControlReg3); //send ACK to local server when command is done successfully
//void zclCoordinator_sendRetry(void);//send Retry to local server when transmit error
void zclCoordinator_ReadRoutingTable(uint8 sm_i);
static void zclCoordinator_getcalParam(void);
static void zclCoordinator_calregtimeout(void);
static void find_end_in_routing_table(uint8 buffer[]);
//static uint8 zclCoordinator_smIEEE_to_id(uint64 sm_ADD_64);
//static void zclCoordinator_id_to_smIEEE(uint8 sm_id_8);
static void send_rom_back(uint8 *reg_ro);

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

//*****************************************************************************
//
// Implementations of function that are required by usb framework.
//
//*****************************************************************************
void usbsuspHookEnteringSuspend(bool remoteWakeupAllowed)
{
    if (remoteWakeupAllowed)
    {
    }
}

void usbsuspHookExitingSuspend(void)
{
}

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

    //GPIOPinTypeGPIOOutput(GPIO_A_BASE, GPIO_PIN_7);
    //GPIOPinWrite(GPIO_A_BASE, GPIO_PIN_7, 0x00);

    //GPIOPinTypeGPIOOutput(GPIO_D_BASE, GPIO_PIN_1);//RF2.14
    //GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_1, 0x00);

    GPIOPinTypeGPIOOutput(GPIO_C_BASE, GPIO_PIN_1);//RF2.14
    GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_1, 0x00);
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

    //HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_5 );
#endif

    //MT Uart Initial
    MT_UartInit();
    MT_UartRegisterTaskID(zclCoordinator_TaskID);

    initUART();

    //
    // Enable AES peripheral
    //
    SysCtrlPeripheralReset(SYS_CTRL_PERIPH_AES);
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_AES);

    initUSB();

    //Initialize controlReg in the coordinator
    zclCoordinator_controlRegInit();
    //Initialize parameters in the coordinator
    zclCoordinator_parameterInit();
    //Initialize SmartMeter data register
    zclCoordinator_dataRegInit();

    //if(Connect_Mode == WIRELESS_CONNECTION)
        //zclscoordinator_startEZmode();

    // Get coordinator 64-bit IEEE external address and network address
    pcoordinator_extAddr = saveExtAddr; //ZDApp.h uint8 saveExtAddr[];
    coordinator_nwkAddr = NLME_GetShortAddr();

    ADD_3 = (uint16)saveExtAddr[6] + ((((uint16)saveExtAddr[7]) << 8) & 0xFF00); //lhy
    ADD_2 = (uint16)saveExtAddr[4] + ((((uint16)saveExtAddr[5]) << 8) & 0xFF00);
    ADD_1 = (uint16)saveExtAddr[2] + ((((uint16)saveExtAddr[3]) << 8) & 0xFF00);
    ADD_0 = (uint16)saveExtAddr[0] + ((((uint16)saveExtAddr[1]) << 8) & 0xFF00);
    coordinator_extAddr = ((uint64)ADD_0 & 0xFFFF) + ((((uint64)ADD_1) << 16) & 0xFFFF0000) + ((((uint64)ADD_2) << 32) & 0xFFFF00000000) + ((((uint64)ADD_3) << 48) & 0xFFFF000000000000);

    for(uint8 i = 0; i < NUM_SMART_METER; i++)
    {
        routing_table[i].sm_ADD_status = 0;
        routing_table[i].sm_ADD = 0;
        routing_table[i].freq = 0;
        routing_table[i].sm_key_table_hi = 0;
        routing_table[i].sm_key_table_lo = 0;
	routing_table[i].sm_rom_table_hi = 0;
        routing_table[i].sm_rom_table_lo = 0;
        routing_table[i].final_key_table_hi = 0;
        routing_table[i].final_key_table_lo = 0;
        routing_table[i].flag_end_encry = 0;
    }
    controlReg[0] = 0x08;
    controlReg[1] = 0x03;
    controlReg[2] = 0x00;
    controlReg[3] = 0x00;
    osal_start_timerEx( zclCoordinator_TaskID, TIME_RUNNING_EVT, 1 );
}

/********************************************************
//USB initiation configuration
void initUSB(void)
**********************************************************/
void initUSB(void)
{
    //
    // Initialize buffers
    //
    memset(&usbCdcInBufferData, 0x00, sizeof(USB_EPIN_RINGBUFFER_DATA));
    usbCdcInBufferData.pBuffer = pInBuffer;
    usbCdcInBufferData.size = sizeof(pInBuffer);
    usbCdcInBufferData.endpointReg = USB_F4;
    usbCdcInBufferData.endpointIndex = 4;
    usbCdcInBufferData.endpointSize = 60;
    memset(&usbCdcOutBufferData, 0x00, sizeof(USB_EPOUT_RINGBUFFER_DATA));
    usbCdcOutBufferData.pBuffer = pOutBuffer;
    usbCdcOutBufferData.size = sizeof(pOutBuffer);
    usbCdcOutBufferData.endpointReg = USB_F4;
    usbCdcOutBufferData.endpointIndex = 4;
    //
    // Enable the USB interface
    //
    //usbCdcInit(115200);
    usbCdcInit(256000);
}


/**************************************************
ProcessUartData()

********************************************************/
void ProcessUartData( mtOSALSerialData_t *Uart_Msg)
{

    uint8 index;
    /*
    dataLen = Uart_Msg->msg[0];

    USB_Msg_in[0] = 0x68;
    for (index = 1; index <= 8; index++)
        USB_Msg_in[index] = Uart_Msg->msg[index + dataLen + 2];
    USB_Msg_in[9] = 0x68;

    USB_Msg_in[10] = dataLen;
    USB_Msg_in[11] = Uart_Msg->msg[1] ;

    for (index = 1; index < (int)(dataLen + 2); index++)
    {
        USB_Msg_in[index + 11] = Uart_Msg->msg[index + 1] ;
    }
    */
    if (Uart_Msg->msg[0] == 0x68)
    {
        dataLen = Uart_Msg->msg[10];
        for (index = 0; index < (int)(dataLen + 13); index++)
        {
            USB_Msg_in[index] = Uart_Msg->msg[index] ;
        }
        //HalUART0Write ( HAL_UART_PORT_0, USB_Msg_in, dataLen + 13);
    }
    else
    {
        for (index = 0; index < 10; index++)
        {
            USB_Msg_in[index] = Uart_Msg->msg[index] ;
        }
        //HalUART0Write ( HAL_UART_PORT_0, USB_Msg_in, 10);
    }
    //HalUART0Write ( HAL_UART_PORT_0, USB_Msg_in, dataLen + 13);

}

uint8 HexToChar(uint8 temp)
{ uint8 dst; if (temp < 10){ dst = temp + '0'; }else{ dst = temp -10 +'A'; }
    return dst;
}
 void showPANID()
{ uint8 tempStr[4]; uint8 dstPan[5] = {0};
  uint8 i;
  int tempPan =  _NIB.nwkPanId;
  tempStr[3] = tempPan&0xf;
  tempStr[2] = (tempPan>>4)&0xf;
  tempStr[1] = (tempPan>>8)&0xf;
  tempStr[0] = (tempPan>>12)&0xf;
  for(i = 0; i<4;i++)
    { dstPan[i] = HexToChar(tempStr[i]); }
    dstPan[4] = '\0';
    HalLcdWriteString( dstPan, HAL_LCD_LINE_3 );
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
    //char  lcdString[10];
    uint8 pack_out[74] = {0};
    /*
    #ifdef LCD_SUPPORTED
        sprintf((char *)lcdString, "test_i: %d", (uint8)(test_i & 0xFF) );
        //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
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
                    showPANID();
#endif
#ifdef ZCL_EZMODE
                    zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif  // ZCL_EZMODE


                }
                break;

            case CMD_SERIAL_MSG:                                           //ZcomDef.h 10.8
                ProcessUartData((mtOSALSerialData_t *)pMsg);
                Process_Wired_Cmd();
                HalLcdWriteString( "datarcv", HAL_LCD_LINE_6 );
                osal_set_event(task_id, Coordinator_USB_EVT);
                break;

            case CMD_USB_MSG:                                           //ZcomDef.h 10.8
                osal_set_event(task_id, Coordinator_USB_EVT);
                HalLcdWriteString( "datarcv", HAL_LCD_LINE_6 );
                //HalUART0Write ( HAL_UART_PORT_0, USB_Msg_in, dataLen + 13);
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

    if(events & TIME_RUNNING_EVT) // EVT that is always running
    {
        /*
                sys_timenew = osal_GetSystemClock();
                sys_secnew = sys_secold + (uint32)((float)(sys_timenew - sys_timeold) / 1000);
                osal_ConvertUTCTime(&TimeStruct , sys_secnew);
                if(TimeStruct.month == 0)
                {
                    TimeStruct.month = 12;
                    TimeStruct.year--;
                }

        #ifdef LCD_SUPPORTED
                sprintf((char *)lcdString, "%d %d %d %d %d %d", TimeStruct.year, TimeStruct.month,
                        TimeStruct.day, TimeStruct.hour, TimeStruct.minutes, TimeStruct.seconds);
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_7 );
        #endif
                //osal_set_event(task_id, TIME_RUNNING_EVT);
                //osal_start_timerEx( zclCoordinator_TaskID, TIME_RUNNING_EVT, 300 );
        */

        char  lcdString[10];
        if(len_uart1) //package is assembled
        {
            if(first_time_in) //ori 1
            {
                first_time_in = 0;
                switch_timenew = osal_GetSystemClock();
                GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_1, 0x02);
                HalLcdWriteString( "uartstep1", HAL_LCD_LINE_3 );
            }
            else if (!second_time_in)
            {

                if (osal_GetSystemClock() - switch_timenew >= 35 )
                {

                    /*
                                        for(uint8 i = 0; i < 8; i++)
                                        {
                                            ui8AESKey[i] = send_buffer[i + 1];
                                        }


                                        if(send_buffer[14] == COM_ADD)
                                        {
                                            for(uint8 i = 8; i < 16; i++)
                                            {
                                                ui8AESKey[i] = 0;
                                            }
                                        }
                    */
                    uint8 len_data = 0;
                    len_data = len_uart1 - 13;


                    /////////////////////////////////////////////////////////////////////
                    find_end_in_routing_table(send_buffer);
                    if (routing_table[end_index].flag_end_encry)
                    {
                        //find_end_in_routing_table(send_buffer);

                        uint8 encry_data[70] = {0};

                        for (uint8 i = 0; i < len_data; i++)
                            encry_data[i] = send_buffer[i + 11];

                        // change data length to make encryption package stay the same
                        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

                        //HalUART0Write ( HAL_UART_PORT_0, encry_data, len_data);
                        for(uint8 i = 0; i < (len_data / 16) ; i++)
                            AesEncryptDecrypt(end_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

                        //HalUART0Write ( HAL_UART_PORT_0, end_final_key, 16);
                        //HalUART0Write ( HAL_UART_PORT_0, encry_data, len_data);
                        send_buffer[10] = len_data;
                        for (uint8 i = 0; i < len_data; i++)
                            send_buffer[i + 11] = encry_data[i];

                        send_buffer[len_data + 11] = 0x00;
                        for (uint8 i = 0; i < len_data + 11; i++ )
                            send_buffer[len_data + 11] +=  send_buffer[i];
                        send_buffer[len_data + 12] = 0x16;
                    }
                    ///////////////////////////////////////////////////////////////////////


                    sprintf((char *)lcdString, "%d %d", routing_table[0].flag_end_encry, routing_table[1].flag_end_encry);
                    HalLcdWriteString( lcdString, HAL_LCD_LINE_4 );


                    HalUART1Write ( HAL_UART_PORT_1, send_buffer, len_data + 13);
                    //HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
                    for( uint8 i = 0; i < 70; i++)
                        send_buffer[i] = 0;
                    second_time_in = 1;

                    if (flag_reset_all_smart_meter)
                    {
                        for(uint8 i = 0; i < NUM_SMART_METER; i++)
                            routing_table[i].flag_end_encry = false;
                        flag_reset_all_smart_meter = false;
                    }
                    if (flag_reset_smart_meter)
                    {
                        routing_table[end_i].flag_end_encry = false;
                        flag_reset_smart_meter = false;
                    }
                    //sprintf((char *)lcdString, "%d", osal_GetSystemClock() - switch_timenew);
                    switch_timenew = osal_GetSystemClock();
                    //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );
                }
            }
            else if (second_time_in)
            {
                //if (osal_GetSystemClock() - switch_timenew >= 35 )
                if (osal_GetSystemClock() - switch_timenew >= 50 )
                {
                    first_time_in = 1;
                    GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_1, 0x00);
                    len_uart1 = 0;
                    second_time_in = 0;
                    //sprintf((char *)lcdString, "%d", osal_GetSystemClock() - switch_timenew);
                    //HalLcdWriteString( lcdString, HAL_LCD_LINE_4 );
                }
            }
        }

        osal_start_timerEx( zclCoordinator_TaskID, TIME_RUNNING_EVT, 5 );

        return ( events ^ TIME_RUNNING_EVT );
    }




    if (events & Coordinator_USB_EVT )
    {
        uint16 i;
        uint8 parameter_in[60] = {0};
        for(i = 0; i < 70; i++)
            pack_out[i] = 0;

        uint16 checksum = 0;

        if(USB_Msg_in[1] == 0xff && USB_Msg_in[8] == 0xff)
        {

            HalLcdWriteString( "coordinator reset", HAL_LCD_LINE_3 );

            uint8 i;
            uint8 pack_out[20] = {0};
            pack_out[0] = 0x68;

            pack_out[1] = (uint8)((ADD_3 & 0xff00) >> 8);
            pack_out[2] = (uint8)(ADD_3 & 0x00ff);
            pack_out[3] = (uint8)((ADD_2 & 0xff00) >> 8);
            pack_out[4] = (uint8)(ADD_2 & 0x00ff);
            pack_out[5] = (uint8)((ADD_1 & 0xff00) >> 8);
            pack_out[6] = (uint8)(ADD_1 & 0x00ff);
            pack_out[7] = (uint8)((ADD_0 & 0xff00) >> 8);
            pack_out[8] = (uint8)(ADD_0 & 0x00ff);


            pack_out[9] = 0x68;
            pack_out[10] = 0x04;

            pack_out[11] = 0x08;
            pack_out[12] = 0x03;
            pack_out[13] = 0x00;
            pack_out[14] = 0x00;

            for(i = 0; i < 15; i++)
                pack_out[15] += pack_out[i];

            pack_out[16] = 0x16;

            uint8 len_data = pack_out[10];
            /*
            if(flag_encry)
            {
                uint8 encry_data[70] = {0};

                for (uint8 i = 0; i < 70; i++)
                    encry_data[i] = 0xFF;

                for (uint8 i = 0; i < len_data; i++)
                    encry_data[i] = pack_out[i + 11];

                len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

                for(uint8 i = 0; i < (len_data / 16) ; i++)
                    AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);
                pack_out[10] = len_data;
                for (uint8 i = 0; i < len_data; i++)
                    pack_out[i + 11] = encry_data[i];

                pack_out[len_data + 11] = 0x00;
                for (uint8 i = 0; i < len_data + 11; i++ )
                    pack_out[len_data + 11] +=  pack_out[i];
                pack_out[len_data + 12] = 0x16;


                //HalUART1Write ( HAL_UART_PORT_1, send_buffer, len_data + 13);
            }
            */

            usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);


            controlReg[0] = 0x08;
            controlReg[1] = 0x03;
            controlReg[2] = 0x00;
            controlReg[3] = 0x00;
            for(i = 4; i < 45; i++)
                controlReg[i] = 0x00;


            flag_encry = false;
            for(uint8 i = 0; i < 3 * NUM_SMART_METER; i++)
                sm_routing_prio_table[i] = 0;

            sm_max = 0;
            num_high_prio = 0;

            for(uint8 i = 0; i < NUM_SMART_METER; i++)
            {
                routing_table[i].sm_ADD_status = 0;
                routing_table[i].sm_ADD = 0;
                routing_table[i].freq = 0;
                routing_table[i].final_key_table_hi = 0;
                routing_table[i].final_key_table_lo = 0;
                routing_table[i].flag_end_encry = 0;
            }

            start = 0;
            ack_index = 0;
            end_index = 0;
            sm_ROM_index = 0;
            end_i = 0;


            write_energy_index = 0;
            sm_index = 0;
            ack_index = 0;
            routing_index = 0;
            smart_auth_index = 0;
            zclCoordinator_SetPowerCalculation();
        }

        else if((ADD_3 == (uint16)(((uint16)USB_Msg_in[1] << 8) + (uint16)USB_Msg_in[2])) && (ADD_2 == (uint16)(((uint16)USB_Msg_in[3] << 8) + (uint16)USB_Msg_in[4]))
                && (ADD_1 == (uint16)(((uint16)USB_Msg_in[5] << 8) + (uint16)USB_Msg_in[6])) && (ADD_0 == (uint16)(((uint16)USB_Msg_in[7] << 8) + (uint16)USB_Msg_in[8])))
        {
            if (flag_encry)
            {
                uint8 len_data = USB_Msg_in[10];
                uint8 encry_data[70] = {0};

                for (uint8 i = 0; i < len_data; i++)
                    encry_data[i] = USB_Msg_in[i + 11];

                for(uint8 i = 0; i < ((len_data % 16) ? (len_data / 16 + 1) : (len_data / 16)) ; i++)
                    AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, DECRYPT_AES);

                for (uint8 i = 0; i < len_data; i++)
                    USB_Msg_in[i + 11] = encry_data[i];
            }

            if(USB_Msg_in[0] && usb_end_flag)   //if there is message in
            {
                //sprintf((char *)lcdString, "%d", 0x22);
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_5 );
                int index;
                checksum = 0;

                uint8 len_data = USB_Msg_in[10];

                for (index = 0; index < 11 + len_data; index++)
                {
                    checksum += USB_Msg_in[index];
                }

                //if((uint8)(checksum & 0xFF) != USB_Msg_in[11 + len_data] || USB_Msg_in[12 + len_data] != 0x16)
                if(0)
                {
                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x05);
                    uint8 i;
                    for(i = 0; i < 110; i++)
                        USB_Msg_in[i] = 0;
                    cmd_right_flag = 0;
                }

                else
                {
                    //if (((USB_Msg_in[11] >> 1) & 0x01) == 1 || USB_Msg_in[13] == 0x10) // Parameter write / key write
                    if (((USB_Msg_in[11] >> 1) & 0x01) == 1) //Parameter write
                    {

                        uint8 len_data0 = USB_Msg_in[10];
                        //uint8 len_data = USB_Msg_in[23 + len_data0];
                        uint8 len_data = *(usb_alloc_buf + 23 + len_data0);
                        checksum = 0;
                        for (index = 0; index < 11 + len_data; index++)
                        {
                            //checksum += USB_Msg_in[index + 13 + len_data0];
                            checksum += *(usb_alloc_buf + index + 13 + len_data0);
                        }

                        //if((uint8)(checksum & 0xFF) != USB_Msg_in[13 + len_data0 + 11 + len_data] || USB_Msg_in[13 + len_data0 + 12 + len_data] != 0x16)
                        //if((uint8)(checksum & 0xFF) != *(usb_alloc_buf + 13 + len_data0 + 11 + len_data) || *(usb_alloc_buf + 13 + len_data0 + 12 + len_data) != 0x16)
                        if(0)
                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x05);
                            uint8 i;
                            for(i = 0; i < 110; i++)
                                USB_Msg_in[i] = 0;
                            cmd_right_flag = 0;
                        }

                        else   //if no problem with the package, get parameter out
                        {
                            if (flag_encry)
                            {
                                uint8 len_data = *(usb_alloc_buf + 23 + len_data0);
                                uint8 encry_data[70] = {0};

                                for (uint8 i = 0; i < len_data; i++)
                                    encry_data[i] = *(usb_alloc_buf + i + 24 + len_data0);


                                for(uint8 i = 0; i < ((len_data % 16) ? (len_data / 16 + 1) : (len_data / 16)) ; i++)
                                    AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, DECRYPT_AES);

                                for (uint8 i = 0; i < len_data; i++)
                                    *(usb_alloc_buf + i + 24 + len_data0) = encry_data[i];

                                //HalUART0Write ( HAL_UART_PORT_0, usb_alloc_buf, 100);
                                //HalUART0Write ( HAL_UART_PORT_0, encry_data, 70);
                            }
                            
                            for(uint8 i = 0; i < len_data; i++)
                                parameter_in[i] = *(usb_alloc_buf + i + 24 + len_data0);
                        }
                    }
                    else if ((USB_Msg_in[13] & 0x01) == 1)             //ROUTING table write
                    {

                        checksum = 0;

                        uint8 len_data0 = USB_Msg_in[10];
                        //uint8 len_addr = USB_Msg_in[63];
                        uint8 len_addr = *(usb_alloc_buf + 23 + len_data0);
                        for (index = 0; index < (len_addr + 11); index++)
                        {
                            //checksum += USB_Msg_in[index + 53];
                            checksum += *(usb_alloc_buf + index + 13 + len_data0);
                        }


                        if (flag_encry)
                        {
                            uint8 len_data = len_addr;
                            uint8 encry_data[70] = {0};

                            for (uint8 i = 0; i < len_data; i++)
                                encry_data[i] = *(usb_alloc_buf + i + 24 + len_data0);


                            for(uint8 i = 0; i < ((len_data % 16) ? (len_data / 16 + 1) : (len_data / 16)) ; i++)
                                AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, DECRYPT_AES);

                            for (uint8 i = 0; i < len_data; i++)
                                *(usb_alloc_buf + i + 24 + len_data0) = encry_data[i];

                            //HalUART0Write ( HAL_UART_PORT_0, usb_alloc_buf, 100);
                            //HalUART0Write ( HAL_UART_PORT_0, encry_data, 70);
                        }
                        //if((uint8)(checksum & 0xFF) != *(usb_alloc_buf + 13 + len_data0 + 11 + len_data) || *(usb_alloc_buf + 13 + len_data0 + 12 + len_data) != 0x16)
                        //if((uint8)(checksum & 0xFF) != USB_Msg_in[len_addr + 64] || USB_Msg_in[len_addr + 65] != 0x16)  //if sum is wrong or end flag is not 0x16
                        if(0)

                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x05);
                            uint8 i;
                            for(i = 0; i < 110; i++)
                                USB_Msg_in[i] = 0;
                            cmd_right_flag = 0;
                        }

                        else   //if no problem with the package, only can have 1 high priority value!!!
                        {
                            r_table routing_single;
                            routing_single.sm_ADD_status = *(usb_alloc_buf + 32 + len_data0);
                            if (routing_single.sm_ADD_status)
                            {
                                //sm_ADD_status[sm_max - 1] = flag_valid;
                                //freq[sm_max - 1] = *(usb_alloc_buf + 33 + len_data0);
                                routing_single.freq = *(usb_alloc_buf + 33 + len_data0);

                                uint8 sm_IEEE_data[16] = {0};
                                uint8 sm_ROM_data[16] = {0};
                                uint8 sm_key_data[16] = {0};
                                for (int index = 0; index < 8; index++)
                                    sm_IEEE_data[index] = *(usb_alloc_buf + 24 + len_data0 + index);

                                routing_single.sm_ADD = BUILD_UINT64_8(sm_IEEE_data[0], sm_IEEE_data[1], sm_IEEE_data[2], sm_IEEE_data[3],
                                                                       sm_IEEE_data[4], sm_IEEE_data[5], sm_IEEE_data[6], sm_IEEE_data[7]);
                                for (int index = 0; index < 16; index++)
                                {
                                    sm_ROM_data[index] = *(usb_alloc_buf + 34 + len_data0 + index);
                                    sm_key_data[index] = *(usb_alloc_buf + 50 + len_data0 + index);
                                }

                                routing_single.sm_rom_table_hi = BUILD_UINT64_8(sm_ROM_data[0], sm_ROM_data[1], sm_ROM_data[2], sm_ROM_data[3], sm_ROM_data[4], sm_ROM_data[5], sm_ROM_data[6], sm_ROM_data[7]);
                                routing_single.sm_rom_table_lo = BUILD_UINT64_8(sm_ROM_data[8], sm_ROM_data[9], sm_ROM_data[10], sm_ROM_data[11], sm_ROM_data[12], sm_ROM_data[13], sm_ROM_data[14], sm_ROM_data[15]);

                                routing_single.sm_key_table_hi = BUILD_UINT64_8(sm_key_data[0], sm_key_data[1], sm_key_data[2], sm_key_data[3], sm_key_data[4], sm_key_data[5], sm_key_data[6], sm_key_data[7]);
                                routing_single.sm_key_table_lo = BUILD_UINT64_8(sm_key_data[8], sm_key_data[9], sm_key_data[10], sm_key_data[11], sm_key_data[12], sm_key_data[13], sm_key_data[14], sm_key_data[15]);

                                AesEncryptDecrypt(sm_key_data, sm_IEEE_data, 0, ENCRYPT_AES);
                                AesEncryptDecrypt(sm_IEEE_data, sm_ROM_data, 0, ENCRYPT_AES);

                                routing_single.final_key_table_hi = BUILD_UINT64_8(sm_ROM_data[0], sm_ROM_data[1], sm_ROM_data[2], sm_ROM_data[3], sm_ROM_data[4], sm_ROM_data[5], sm_ROM_data[6], sm_ROM_data[7]);
                                routing_single.final_key_table_lo = BUILD_UINT64_8(sm_ROM_data[8], sm_ROM_data[9], sm_ROM_data[10], sm_ROM_data[11], sm_ROM_data[12], sm_ROM_data[13], sm_ROM_data[14], sm_ROM_data[15]);

                                routing_single.flag_end_encry = false;
                                if(routing_single.freq != 1)
                                {
                                    num_high_prio ++;
                                }
                                route_table_add(routing_table, routing_single);
                            }
                            else
                            {
                                routing_single.sm_ADD = BUILD_UINT64_8(*(usb_alloc_buf + 24 + len_data0), *(usb_alloc_buf + 25 + len_data0), *(usb_alloc_buf + 26 + len_data0), *(usb_alloc_buf + 27 + len_data0),
                                                                       *(usb_alloc_buf + 28 + len_data0), *(usb_alloc_buf + 29 + len_data0), *(usb_alloc_buf + 30 + len_data0), *(usb_alloc_buf + 31 + len_data0));
                                routing_single.freq = *(usb_alloc_buf + 33 + len_data0);
                                if(routing_single.freq != 1)
                                {
                                    num_high_prio --;
                                }
                                route_table_delete(routing_table, routing_single);

                            }

                            //osal_mem_free(routing_single);

                        }
                    }
                }
            }

            if(cmd_right_flag)
            {
                // command: control register write
                // Write to controlReg when controlReg write is disabled  *
                // This is the normal write operation from server

                //if ((controlReg[1] & 0x01) == 1)
                //modify temperately by Jinhui 1/24/2017 for debugging purpose
                if ((controlReg[1] & 0x01) == 1)
                {
                    uint8 i;

                    for(i = 0; i < 8; i++)
                    {
                        coor_addrRegister[i] = USB_Msg_in[i + 1];
                    }


                    for(i = 0; i < dataLen; i++)
                    {
                        controlReg[i] = USB_Msg_in[i + 11];
                    }

                    for(i = 0; i < 110; i++)
                        USB_Msg_in[i] = 0;

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
                        pack_out[i] = coor_addrRegister[i - 1];

                    pack_out[9] = 0x68;
                    pack_out[10] = 0x28;

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
                    pack_out[50] = controlReg[3];

                    pack_out[51] = 0;
                    for(i = 0; i < 51; i++)
                        pack_out[51] += pack_out[i];

                    pack_out[52] = 0x16;
                    /*
                                UARTEnable(UART0_BASE );
                                for (i = 0; i < 52; i++)
                                {
                                    UARTCharPut(UART0_BASE, pack_out[i]);
                                }
                                UARTDisable(UART0_BASE);
                    */
                    uint8 len_data = pack_out[10];
                    if(flag_encry)
                    {
                        uint8 encry_data[70] = {0};

                        for (uint8 i = 0; i < 70; i++)
                            encry_data[i] = 0xFF;
                        for (uint8 i = 0; i < len_data; i++)
                            encry_data[i] = pack_out[i + 11];

                        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

                        for(uint8 i = 0; i < (len_data / 16) ; i++)
                            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

                        pack_out[10] = len_data;
                        for (uint8 i = 0; i < len_data; i++)
                            pack_out[i + 11] = encry_data[i];

                        pack_out[len_data + 11] = 0x00;
                        for (uint8 i = 0; i < len_data + 11; i++ )
                            pack_out[len_data + 11] +=  pack_out[i];
                        pack_out[len_data + 12] = 0x16;


                        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, len_data + 13);
                    }

                    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);
                    //usbibufPush(&usbCdcInBufferData, pack_out, 53);
                }

                // command: coordinator Authentication
                else if (controlReg[2] == 0x08)
                {
                    //////// TO BE REPLACED
                    uint8 romreg[16] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
                    osal_nv_item_init (FLASH_RO, 40, NULL);
                    osal_nv_write (FLASH_RO, 0, 40, &romreg[0]);
                    //////// TO BE REPLACED


                    osal_nv_item_init (FLASH_RO, 40, NULL);
                    osal_nv_read (FLASH_RO, 0, 40, &romread[0]);
                    send_rom_back(&romread[0]);
                }

                // command: coordinator set key
                else if (controlReg[2] == 0x10)
                {
                    for (uint8 i = 0; i < 16; i++)
                        coor_key[i] = controlReg[40 + i];

                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);

                    uint8 encry_data[16] = {0};

                    encry_data[0] = (uint8)((ADD_3 & 0xff00) >> 8);
                    encry_data[1] = (uint8)(ADD_3 & 0x00ff);
                    encry_data[2] = (uint8)((ADD_2 & 0xff00) >> 8);
                    encry_data[3] = (uint8)(ADD_2 & 0x00ff);
                    encry_data[4] = (uint8)((ADD_1 & 0xff00) >> 8);
                    encry_data[5] = (uint8)(ADD_1 & 0x00ff);
                    encry_data[6] = (uint8)((ADD_0 & 0xff00) >> 8);
                    encry_data[7] = (uint8)(ADD_0 & 0x00ff);

                    //HalUART0Write ( HAL_UART_PORT_0, coor_key, 16);
                    //HalUART0Write ( HAL_UART_PORT_0, encry_data, 16);
                    AesEncryptDecrypt(coor_key, encry_data, 0, ENCRYPT_AES);
                    //HalUART0Write ( HAL_UART_PORT_0, encry_data, 16);

                    for (uint8 i = 0; i < 16; i++)
                        coor_final_key[i] = romread[i];

                    //HalUART0Write ( HAL_UART_PORT_0, encry_data, 16);
                    //HalUART0Write ( HAL_UART_PORT_0, coor_final_key, 16);
                    AesEncryptDecrypt(encry_data, coor_final_key,  0, ENCRYPT_AES);
                    //HalUART0Write ( HAL_UART_PORT_0, coor_final_key, 16);
                    flag_encry = true;
                }

                // command: smart meter authentication
                else if (controlReg[2] == 0x20)
                {
                    //sm_ROM_index = smart_auth_index;
                    if(((controlReg[3] >> 3) & 0x01) == 0)
                    {
                        smart_auth_index = 0;
                        if(controlReg[8] != 0xFF)
                        {
                            uint64 rom_sm_ADD = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11],
                                                               controlReg[12], controlReg[13], controlReg[14], controlReg[15]);

                            for(uint8 i = 0; i < sm_max; i++)
                                if(rom_sm_ADD == routing_table[i].sm_ADD)
                                {
                                    sm_ROM_index = i;
                                    break;
                                }
                        }
                        else
                        {
                            sm_ROM_index = 0;
                        }
                    }
                    else
                    {
                        /*
                          if(controlReg[8] != 0xFF) {
                              uint64 rom_sm_ADD = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11],
                                                             controlReg[12], controlReg[13], controlReg[14], controlReg[15]);

                              for(uint8 i = 0; i < sm_max; i++)
                                  if(rom_sm_ADD == routing_table[i].sm_ADD)
                                  {
                                      sm_ROM_index = i;
                                      break;
                                  }
                          }
                          else {
                              sm_ROM_index = smart_auth_index;
                          }
                        */
                        sm_ROM_index = smart_auth_index;
                    }
                    flag_auth = false;
                    zclCoordinator_SendAuth();
                    WAIT_EVT_INDEX = Smartmeter_Auth;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                }

                // command: relay control
                // Switch smart meter power condition if bit 3 is set to 0  *
                // Reset this bit to its default value when operation is finished
                //else if (((controlReg[0] >> 3) & 0x01) == 0)  // power relay control
                else if ((controlReg[0] & 0xF7) == 0 && controlReg[1] == 0x02 && controlReg[2] == 0x00 && controlReg[3] == 0x00)  // except bit3, other bits are all 0, and controlReg[3] is 0x00
                {
                    //HalLcdWriteString( "relay0", HAL_LCD_LINE_6 );
                    int i;

                    for(i = 0; i < 8; i++)
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];

                    if (controlReg[0] == 0x08)
                        flagrelay = 1;
                    else
                        flagrelay = 0;

                    zclCoordinator_SendRelay();

                    WAIT_EVT_INDEX = Coordinator_RelaySet;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);

                    time_old = osal_GetSystemClock();
                    time_new = time_old;
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
                        paramReg[i] = (uint16)(((uint16)(parameter_in[i * 2])) << 8) + (uint16)(parameter_in[i * 2 + 1]);
                    }

                    //write parameters into FLASH memory of coordaintor
                    zclCoordinator_nvWriteParam();

                    if( Connect_Mode == WIRELESS_CONNECTION)
                    {
                        //write parameters to all the smart meters
                        for (sm_id = 0; sm_id < sm_max; sm_id++)
                        {
                            zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((routing_table[sm_id].sm_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
                            zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((routing_table[sm_id].sm_ADD) >> 48) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((routing_table[sm_id].sm_ADD) >> 40) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((routing_table[sm_id].sm_ADD) >> 32) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((routing_table[sm_id].sm_ADD) >> 24) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((routing_table[sm_id].sm_ADD) >> 16) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((routing_table[sm_id].sm_ADD) >> 8) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((routing_table[sm_id].sm_ADD) & 0x00000000000000FF);

                            zclCoordinator_SetParam();
                        }
                    }
                    else if( Connect_Mode == WIRED_CONNECTION)
                    {
                        zclCoordinator_SetParam();
                    }

                    WAIT_EVT_INDEX = Coordinator_ProcessParaSet;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);

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
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];

                    /*
                    if (controlReg[8] == 0 && controlReg[9] == 0 && controlReg[10] == 0 && controlReg[11] == 0) //Particular C reset
                    {
                        HalLcdWriteString( "ONE coordinator reset", HAL_LCD_LINE_3 );
                        write_energy_index = 0;
                        setpower_index = 0;
                        sm_index = 0;
                        ack_index = 0;
                        routing_index = 0;

                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);

                        controlReg[0] = 0x08;
                        controlReg[1] = 0x03;
                        controlReg[2] = 0x00;
                        controlReg[3] = 0x00;
                        for(i = 4; i < 45; i++)
                            controlReg[i] = 0x00;
                    }
                    else
                    {
                    */
                    if(((controlReg[3] >> 3) & 0x01) == 0)
                        write_energy_index = 0;

                    flagreset = 1;

                    ENERGY_RESET_VALUE = BUILD_UINT32(controlReg[7], controlReg[6], controlReg[5], controlReg[4]);

                    zclCoordinator_SendReset();

                    WAIT_EVT_INDEX = Coordinator_EnergyResetWait;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                    //}
                }





                // command: network discovery
                // Perform network discovery if bit 4 is 1           *
                //  Reset this bit to its default value when operation is finished
                else if (((controlReg[0] >> 4) & 0x01) == 1)
                {
                    if(((controlReg[3] >> 3) & 0x01) == 0)
                        routing_index = 0;

                    zclCoordinator_NetDiscov();

                    WAIT_EVT_INDEX = Network_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;

                }

                // command: system reset
                //else if (((controlReg[0] >> 7) & 0x01) == 0 && ((controlReg[1] >> 1) & 0x01) == 0)
                else if(controlReg[0] == 0x08 && controlReg[1] == 0x00 && controlReg[2] == 0x00 && controlReg[3] == 0x00)
                {
                    /*
                    for(uint8 i = 0; i < 3 * NUM_SMART_METER; i++)
                        sm_routing_prio_table[i] = 0;

                    sm_max = 0;
                    num_high_prio = 0;
                    for(uint8 i = 0; i < NUM_SMART_METER; i++)
                    {
                        routing_table[i].sm_ADD_status = 1;
                        routing_table[i].sm_ADD = 0;
                        routing_table[i].freq = 0;
                        final_key_table_hi[i] = 0;
                        final_key_table_lo[i] = 0;
                    }

                    write_energy_index = 0;
                    sm_index = 0;
                    ack_index = 0;
                    routing_index = 0;
                    smart_auth_index = 0;


                    if(((controlReg[3] >> 3) & 0x01) == 0)
                        setpower_index = 0;

                    start = 0;
                    zclCoordinator_SetPowerCalculation();

                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    */


                    //reset all smart meters
                    if (controlReg[9] == 0xFF)
                    {
                        zclCoordinator_SetPowerCalculation();

                        flag_reset_all_smart_meter = true;
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    }
                    //reset coordinator
                    else if (controlReg[9] == 0x00)
                    {

                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                        flag_encry = false;
                        for(uint8 i = 0; i < 3 * NUM_SMART_METER; i++)
                            sm_routing_prio_table[i] = 0;

                        sm_max = 0;
                        num_high_prio = 0;

                        for(uint8 i = 0; i < NUM_SMART_METER; i++)
                        {
                            routing_table[i].sm_ADD_status = 0;
                            routing_table[i].sm_ADD = 0;
                            routing_table[i].freq = 0;
                            routing_table[i].final_key_table_hi = 0;
                            routing_table[i].final_key_table_lo = 0;
                            routing_table[i].flag_end_encry = 0;
                        }

                        start = 0;
                        ack_index = 0;
                        end_index = 0;
                        sm_ROM_index = 0;
                        end_i = 0;


                        write_energy_index = 0;
                        sm_index = 0;
                        ack_index = 0;
                        routing_index = 0;
                        smart_auth_index = 0;

                        //if(((controlReg[3] >> 3) & 0x01) == 0)
                        //    setpower_index = 0;

                    }
                    else
                    {
                        zclCoordinator_SetPowerCalculation();
                        uint64 IEEE_id = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);


                        for (end_i = 0; end_i < sm_max; end_i++)
                        {
                            if (IEEE_id == routing_table[end_i].sm_ADD)
                                break;
                        }

                        flag_reset_smart_meter = true;
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    }
                    /*
                    WAIT_EVT_INDEX = setPowerCalculation_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                    */
                }


                // command: routing table read
                // Send routing table when bit 5 is set to 1                 *
                // Reset this bit to its default value when operation is finished
                else if (((controlReg[0] >> 5) & 0x01) == 1)
                {
                    if(sm_max == 1)
                    {
                        if(controlReg[3] == 0)

                        {
                            sm_index = 0;
                            controlReg[0] = 0x08;
                            controlReg[1] = 0x03;
                            controlReg[2] = 0x00;
                            controlReg[3] = 0x00;
                            zclCoordinator_ReadRoutingTable(sm_index);
                            sm_index++;
                        }

                    }
                    else if(sm_max == 0)
                    {
                        if(controlReg[3] == 0)
                        {
                            sm_index = 0;
                            controlReg[0] = 0x08;
                            controlReg[1] = 0x03;
                            controlReg[2] = 0x00;
                            controlReg[3] = 0x00;
                            zclCoordinator_ReadRoutingTable(sm_index);
                            sm_index++;
                        }
                    }
                    else
                    {
                        if((controlReg[3] >> 3) & 0x01 == 1)                //ACK
                        {
                            if(sm_index < sm_max - 1)
                            {
                                controlReg[0] = 0x28;
                                controlReg[1] = 0x03;
                                controlReg[2] = 0x00;
                                controlReg[3] = 0x00;
                                zclCoordinator_ReadRoutingTable(sm_index);   //if not finish, continue to send the next one
                                sm_index++;
                            }
                            else if(Routingtable_flag)
                            {
                                Routingtable_flag = 0;                      //set Routingtable_flag to 0 after complete read the routing table.
                                controlReg[0] = 0x08;       //reset bit 5 to 0
                                controlReg[1] = 0x03;
                                controlReg[2] = 0x00;
                                controlReg[3] = 0x00;
                                zclCoordinator_ReadRoutingTable(sm_index);
                                sm_index++;
                            }
                        }
                        else
                        {
                            Routingtable_flag = 1;
                            sm_index = 0;                                    //the first one

                            controlReg[0] = 0x28;
                            controlReg[1] = 0x03;
                            controlReg[2] = 0x00;
                            controlReg[3] = 0x00;
                            zclCoordinator_ReadRoutingTable(sm_index);
                            sm_index++;
                        }
                    }
                }

                // command: routing table write
                else if ((controlReg[2] & 0x01) == 1)
                {

                    /*
                    if(routing_all_flag == 1)
                    {
                        for(uint8 i = 0; i < NUM_SMART_METER; i++)
                            routing_table[i].sm_ADD_status = 1;
                    }
                    routing_all_flag = 0;

                    if(controlReg[8] == 0x00 && controlReg[9] == 0x12 && controlReg[10] == 0x4B)
                    {
                        sm_max++;
                        sm_ADD[sm_max - 1] = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11],
                                                            controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                    }
                    */

                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                }

                // command: control register read
                // Send current control register to server if controlReg read is enabled (bit 6=1) *
                // Reset this parameter to its default state when operation is finished.
                else if (((controlReg[0] >> 6)  & 0x01) == 1)
                {
                    pack_out[0] = 0x68;
                    int i;
                    for(i = 1; i <= 8; i++)
                        pack_out[i] = coor_addrRegister[i - 1];

                    pack_out[9] = 0x68;
                    pack_out[10] = 0x2C;

                    for(i = 11; i < 51; i++)
                        pack_out[i] = controlReg[i - 11];

                    controlReg[0] = controlReg[0] & 0xBF; //reset bit 6 to 0
                    controlReg[1] = controlReg[1] | 0x01; //reset byte1 bit 0 to 1

                    pack_out[51] = controlReg[0];
                    pack_out[52] = controlReg[1];
                    pack_out[53] = controlReg[2];
                    pack_out[54] = controlReg[3];

                    pack_out[55] = 0;
                    for(i = 0; i < 55; i++)
                        pack_out[55] += pack_out[i];

                    pack_out[56] = 0x16;

                    uint8 len_data = pack_out[10];
                    if(flag_encry)
                    {
                        uint8 encry_data[70] = {0};

                        for (uint8 i = 0; i < 70; i++)
                            encry_data[i] = 0xFF;
                        for (uint8 i = 0; i < len_data; i++)
                            encry_data[i] = pack_out[i + 11];

                        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

                        for(uint8 i = 0; i < (len_data / 16) ; i++)
                            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

                        pack_out[10] = len_data;
                        for (uint8 i = 0; i < len_data; i++)
                            pack_out[i + 11] = encry_data[i];

                        pack_out[len_data + 11] = 0x00;
                        for (uint8 i = 0; i < len_data + 11; i++ )
                            pack_out[len_data + 11] +=  pack_out[i];
                        pack_out[len_data + 12] = 0x16;

                    }
                    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);
                }

                // command: Timeset
                else if (((controlReg[1] >> 5) & 0x01) == 1)
                {
                    int i;

                    for(i = 0; i < 6; i++)
                    {
                        timeReg[i] = (controlReg[i * 2 + 16] << 8) + controlReg[i * 2 + 17];
                    }

                    sys_timeold = osal_GetSystemClock();
                    sys_timenew = sys_timeold;
                    TimeStruct.seconds = (uint8)timeReg[5];
                    TimeStruct.minutes = (uint8)timeReg[4];
                    TimeStruct.hour = (uint8)timeReg[3];
                    TimeStruct.day = (uint8)(--timeReg[2]);
                    TimeStruct.month = (uint8)(--timeReg[1]);
                    TimeStruct.year = (uint16)timeReg[0];
                    sys_secold = osal_ConvertUTCSecs( &TimeStruct );

                    if( Connect_Mode == WIRELESS_CONNECTION)
                    {
                        //write parameters to all the smart meters
                        for (sm_id = 0; sm_id < sm_max; sm_id++)
                        {
                            zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((routing_table[sm_id].sm_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
                            zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((routing_table[sm_id].sm_ADD) >> 48) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((routing_table[sm_id].sm_ADD) >> 40) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((routing_table[sm_id].sm_ADD) >> 32) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((routing_table[sm_id].sm_ADD) >> 24) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((routing_table[sm_id].sm_ADD) >> 16) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((routing_table[sm_id].sm_ADD) >> 8) & 0x00000000000000FF);
                            zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((routing_table[sm_id].sm_ADD) & 0x00000000000000FF);
                            zclCoordinator_SetTime();
                        }
                    }
                    else if( Connect_Mode == WIRED_CONNECTION)  //non-broadcast
                    {
                        zclCoordinator_SetTime();
                    }


                    WAIT_EVT_INDEX = Coordinator_ProcessParaSet;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;

                }

                //command: calregister read
                else if (((controlReg[1] >> 6) & 0x01) == 1)
                {
                    int i;
                    for(i = 0; i < 8; i++)
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];
                    flag_calget = 0;

                    zclCoordinator_getcalParam();

                    WAIT_EVT_INDEX = GETCAL_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);

                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                }

                //command: CONFIGURATION register write
                else if (((controlReg[1] >> 7) & 0x01) == 1)
                {
                    int i;
                    for(i = 0; i < 8; i++)
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];
                    flag_config_reg = 0;

                    SM_CONFIG_2 = UINT8_TO_16(controlReg[34], controlReg[35]);
                    SM_CONFIG_1 = UINT8_TO_16(controlReg[36], controlReg[37]);
                    SM_CONFIG_0 = UINT8_TO_16(controlReg[38], controlReg[39]);

                    zclCoordinator_setconfigReg();

                    WAIT_EVT_INDEX = SETCONFIG_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);

                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                }

                //command: data register read
                //Send request for smart meter data using round robin method
                //Specify which smart meter to request data
                //&zclCoordinator_DstAddr = &routing_table[sm_index].sm_ADD;
                //else if (((controlReg[0] >> 7) & 0x01) == 1 && Drr_flag == 1 && (((controlReg[3] >> 3) & 0x01) == 0))
                else if (((controlReg[0] >> 7) & 0x01) == 1 && (((controlReg[3] >> 3) & 0x01) == 0))
                {
                    //update frequency table
                    uint8 j, l, k = 0;
                    uint8 num_low_prio = 0;
                    num_low_prio = sm_max - num_high_prio;

                    for(uint8 i = 0; i < 3 * NUM_SMART_METER; i++)
                        sm_routing_prio_table[i] = 0;

                    for(i = 0; i < num_low_prio; i++) // number of low priority
                    {
                        for(j = 0; j < routing_table[i].freq / num_low_prio; j++)
                        {
                            for(l = 0; l < num_high_prio; l++)
                            {
                                sm_routing_prio_table[k++] = l;
                            }
                        }
                        sm_routing_prio_table[k++] = i + num_high_prio;
                    }
                    num_prio_sm_max = k;

                    //HalLcdWriteString( "DRREVT", HAL_LCD_LINE_7 );
                    ack_index = 0;
                    datain_complete = 0;
                    first_write_flag = 1;
                    // &zclCoordinator_DstAdfdr = psm_ADD;
                    /*
                    while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                    {
                        ack_index++;
                    }
                    */

                    //zclCoordinator_id_to_smIEEE(sm_routing_prio_table[ack_index]);
                    zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
                    zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 48) & 0x00000000000000FF);
                    zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 40) & 0x00000000000000FF);
                    zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 32) & 0x00000000000000FF);
                    zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 24) & 0x00000000000000FF);
                    zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 16) & 0x00000000000000FF);
                    zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 8) & 0x00000000000000FF);
                    zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) & 0x00000000000000FF);

                    //Stop power calculation
                    flaginc = 0;
                    zclCoordinator_SendRestart();
                    //Send request to smart meter to send data

                    //zclCoordinator_SendData();   //send request to get data

                    WAIT_EVT_INDEX = DATA_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    ACK_flag = 1;

                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                    Drr_flag = 0;

                }

                //ACKcommand
                //else if((((controlReg[3] >> 3) & 0x01) == 1) && (((controlReg[0] >> 7) & 0x01) == 1) && ACK_flag == 1)
                //else if((((controlReg[3] >> 3) & 0x01) == 1) && (((controlReg[0] >> 7) & 0x01) == 1))
                else if((controlReg[0] == 0x88 && controlReg[1] == 0x02 && controlReg[2] == 0x00 && controlReg[3] == 0x08) ||
                        (controlReg[0] == 0x08 && controlReg[1] == 0x02 && controlReg[2] == 0x00 && controlReg[3] == 0x08))
                {
                    if((controlReg[3] & 0x01) == 1)//check is retry or not!
                    {
                        dataRegSel = dataRegSel ^ 0x01;
                        osal_set_event(task_id, DATA_CL_EVT);
                    }
                    else
                    {
                        uint8 j, l, k = 0;
                        uint8 num_low_prio = 0;
                        num_low_prio = sm_max - num_high_prio;

                        for(uint8 i = 0; i < 3 * NUM_SMART_METER; i++)
                            sm_routing_prio_table[i] = 0;

                        for(i = 0; i < num_low_prio; i++) // number of low priority
                        {
                            for(j = 0; j < routing_table[i].freq / num_low_prio; j++)
                            {
                                for(l = 0; l < num_high_prio; l++)
                                {
                                    sm_routing_prio_table[k++] = l;
                                }
                            }
                            sm_routing_prio_table[k++] = i + num_high_prio;
                        }
                        num_prio_sm_max = k;

                        if((controlReg[0] == 0x08))
                        {
                            PowSpCal = 1;
                            zclCoordinator_sendACK(0x88, 0x03, 0x00, 0x00);
                        }
                        else
                            osal_set_event(task_id, ACK_WAIT_EVT);

                    }
                }

                //calibration command(voltage, current, gen_input1, gen_input2)
                else if  (((controlReg[1] >> 2) & 0x01) == 1 || ((controlReg[1] >> 3) & 0x01) == 1
                          || ((controlReg[2] >> 1) & 0x01) == 1 || ((controlReg[2] >> 2) & 0x01) == 1)
                {
                    int i;
                    for(i = 0; i < 8; i++)
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];
                    zclCoordinator_SendCalibrate();

                    WAIT_EVT_INDEX = Calibration_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                }

                /*
                //voltage calibration command
                else if  (((controlReg[1] >> 2) & 0x01) == 1 )
                {
                    int i;
                    for(i = 0; i < 8; i++)
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];
                    zclCoordinator_SendCalibrate();

                    WAIT_EVT_INDEX = Calibration_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                }


                //current calibration command
                else if  (((controlReg[1] >> 3) & 0x01) == 1 )
                {
                    int i;
                    for(i = 0; i < 8; i++)
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];
                    zclCoordinator_SendCalibrate();

                    WAIT_EVT_INDEX = Calibration_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;

                }

                //engergy calibration command
                else if  (((controlReg[1] >> 4) & 0x01) == 1 )
                {
                    int i;
                    for(i = 0; i < 8; i++)
                        zclCoordinator_DstAddr.addr.extAddr[i] = controlReg[15 - i];
                    zclCoordinator_SendCalibrate();

                    WAIT_EVT_INDEX = Calibration_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    time_old = osal_GetSystemClock();
                    time_new = time_old;

                }
                */
            }

        }
        cmd_right_flag = 1;

        if (usb_alloc_buf)
            osal_mem_free(usb_alloc_buf);
        return ( events ^ Coordinator_USB_EVT );
    }



    if ( events & ACK_WAIT_EVT )
    {

        //uint8 smvalid_flag = 0;
        uint8 all_smvalid_flag = 0;

        if(PowSpCal == 1)
        {
            if(ack_index < (num_prio_sm_max - 1))
            {
                /*
                uint16 i;
                for(i = ack_index + 1; i < sm_max; i++)
                    smvalid_flag = smvalid_flag || routing_table[i].sm_ADD_status;

                if(smvalid_flag == 0)
                {
                    Round_end_flag = 1;
                    osal_set_event(task_id, DATA_CL_EVT);
                    Drr_flag = 1;
                    ACK_flag = 0;
                }
                else
                {
                    ack_index++;

                    osal_set_event(task_id, ACK_CS_EVT);

                    uint8 test[4] = {0};
                    test[0] = ack_index;
                    test[1] = (uint8)num_prio_sm_max;
                    HalUART0Write ( HAL_UART_PORT_0, test, 4);
                }
                */
                ack_index++;
                osal_set_event(task_id, ACK_CS_EVT);
            }
            else
            {
                Round_end_flag = 1;
                osal_set_event(task_id, DATA_CL_EVT);
                Drr_flag = 1;
                ACK_flag = 0;
            }

        }
        else
        {
            int m = 0;
            for(m = 0; m < sm_max; m++)
                all_smvalid_flag = all_smvalid_flag || routing_table[m].sm_ADD_status;
            if(all_smvalid_flag == 0)
            {
                Round_end_flag = 1;
                osal_set_event(task_id, DATA_CL_EVT);
                Drr_flag = 1;
                ACK_flag = 0;
            }
            else
            {
                if(ack_index < (num_prio_sm_max - 1))
                {
                    ack_index++;                                  ////prepare the next Smart Meter address.
                    /*
                    while((sm_ADD_status[ack_index] == 0) && (ack_index < (sm_max - 1)))
                    {
                        ack_index++;
                    }
                    if((sm_ADD_status[ack_index] == 0) && ack_index == (sm_max - 1))
                    {
                        ack_index = 0;
                        while((sm_ADD_status[ack_index] == 0) && (ack_index < (sm_max - 1)))
                        {
                            ack_index++;
                        }
                    }
                    */
                    osal_set_event(task_id, ACK_CS_EVT);
                }
                else
                {
                    ack_index = 0;                                //check there is no stop it, so the round robin will continue from the beginning.
                    /*
                    while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                    {
                        ack_index++;
                    }*/
                    osal_set_event(task_id, ACK_CS_EVT);
                }
            }
        }
        return ( events ^ ACK_WAIT_EVT );
    }

    // Please update to 20bytes of dataReg
    if ( events & DATA_CL_EVT )                                       ///DATA_CL_EVT
    {
        //HalLcdWriteString( "datawait4", HAL_LCD_LINE_6 );
        pack_out[0] = 0x68;
        int i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = coor_addrRegister[i - 1];
        pack_out[9] = 0x68;
        pack_out[10] = (14 + len_DataReg) * 2 + 4;

        if (dataRegSel == 0)
        {
            //HalLcdWriteString( "datareg1", HAL_LCD_LINE_7 );
            if(Timeout_Pong == 1)
            {
                //controlReg[3] = 0x02;
                Timeout_Pong = 0;
            }
            /*
            else
            {
                controlReg[3] = 0x00;
            }
            */
            for (index = 0; index < 14 + len_DataReg; index++)
            {
                value = dataReg_Pong[index];
                u_value = int16ToUint16(value);
                // uint16_t into two uint8_t first
                u_partA = (uint8_t) ((u_value & 0xFF00) >> 8);
                u_partB = (uint8_t) (u_value & 0x00FF);

                pack_out[11 + index * 2] = u_partA;
                pack_out[12 + index * 2] = u_partB;
            }
        }

        else if (dataRegSel == 1)
        {
            //HalLcdWriteString( "datareg0", HAL_LCD_LINE_7 );
            //pdataReg_Ping=&dataReg_Ping[0];    //initialize pointer
            if(Timeout_Ping == 1)
            {
                //controlReg[3] = 0x02;
                Timeout_Ping = 0;
            }
            /*
            else
            {
                controlReg[3] = 0x00;
            }
            */
            for (index = 0; index < 14 + len_DataReg; index++)
            {
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

            if(first_complete == 0 && all_invaild_flag != 0)
            {
                time_old = osal_GetSystemClock();
                time_new = time_old;
                all_invaild_flag = 1;
                WAIT_EVT_INDEX = DATA_WAIT;
                osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);

            }
        }

        else if(first_complete == 0 && all_invaild_flag != 0)
        {
            time_old = osal_GetSystemClock();
            time_new = time_old;
            all_invaild_flag = 1;
            WAIT_EVT_INDEX = DATA_WAIT;
            osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);

        }

        Timeout_bit = 0;
        if(len_DataReg == 0)
            controlReg[3] = 0x02;
        else
            controlReg[3] = 0x00;
        pack_out[39 + len_DataReg * 2] = controlReg[0];
        pack_out[40 + len_DataReg * 2] = controlReg[1];
        pack_out[41 + len_DataReg * 2] = controlReg[2];
        pack_out[42 + len_DataReg * 2] = controlReg[3];
        pack_out[43 + len_DataReg * 2] = 0;
        for(i = 0; i < 43 + len_DataReg * 2; i++)
            pack_out[43 + len_DataReg * 2] += pack_out[i];
        pack_out[44 + len_DataReg * 2] = 0x16;
        /*
        UARTEnable(UART0_BASE );
        for (i = 0; i < 62; i++)
        {
            UARTCharPut(UART0_BASE, pack_out[i]);
        }
        UARTDisable(UART0_BASE );
        */
        uint8 len_data = pack_out[10];
        if(flag_encry)
        {
            uint8 encry_data[70] = {0};

            for (uint8 i = 0; i < 70; i++)
                encry_data[i] = 0xFF;
            for (uint8 i = 0; i < len_data; i++)
                encry_data[i] = pack_out[i + 11];

            len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

            for(uint8 i = 0; i < (len_data / 16) ; i++)
                AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

            pack_out[10] = len_data;
            for (uint8 i = 0; i < len_data; i++)
                pack_out[i + 11] = encry_data[i];

            pack_out[len_data + 11] = 0x00;
            for (uint8 i = 0; i < len_data + 11; i++ )
                pack_out[len_data + 11] +=  pack_out[i];
            pack_out[len_data + 12] = 0x16;

        }
        usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);
        //usbibufPush(&usbCdcInBufferData, pack_out, 30);
        //usbibufPush(&usbCdcInBufferData, pack_out, 45 + len_DataReg * 2);

        controlReg[1] = 0x03;
        //dataRegSel = dataRegSel ^ 0x01;
        first_complete = 0;

        char lcdString[10];
        //sprintf((char *)lcdString, "DATAREGpackout0: %d", pack_out[0]);
        //HalLcdWriteString( lcdString, HAL_LCD_LINE_7 );

        return ( events ^ DATA_CL_EVT );

    }

    if ( events & ACK_CS_EVT )
    {
        //HalLcdWriteString( "datawait3", HAL_LCD_LINE_6 );
        /*
        zclCoordinator_id_to_smIEEE(sm_routing_prio_table[ack_index]);
        zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((sm_ADD_prio) >> 56) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((sm_ADD_prio) >> 48) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((sm_ADD_prio) >> 40) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((sm_ADD_prio) >> 32) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((sm_ADD_prio) >> 24) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((sm_ADD_prio) >> 16) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((sm_ADD_prio) >> 8) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((sm_ADD_prio) & 0x00000000000000FF);
        */
        zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 56) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 48) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 40) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 32) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 24) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 16) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 8) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) & 0x00000000000000FF);

        //sm_routing_prio_table[18] = (uint8)sm_routing_prio_table[ack_index];
        //sm_routing_prio_table[19] = (uint8)num_prio_sm_max;
        //HalUART0Write ( HAL_UART_PORT_0, sm_routing_prio_table, 20);

        //if(sm_ADD_status[ack_index] == 1)
        if(routing_table[sm_routing_prio_table[ack_index]].sm_ADD_status == 1)
        {
            //Stop power calculation
            flaginc = 0;
            /*
            uint32 wait_timenew = osal_GetSystemClock();
            while (osal_GetSystemClock() - wait_timenew < 20)
            {
            };
            */
            zclCoordinator_SendRestart();
            time_old = osal_GetSystemClock();
            time_new = time_old;
            osal_set_event(task_id, DATA_CL_EVT);
        }

        else
        {
            int i;
            all_invaild_flag = 0;
            for(i = 0; i < sm_max; i++)
                all_invaild_flag = all_invaild_flag || routing_table[i].sm_ADD_status;
            if(all_invaild_flag != 0)
                osal_set_event(task_id, ACK_WAIT_EVT);
            else
                osal_set_event(task_id, DATA_CL_EVT);
        }
        return ( events ^ ACK_CS_EVT );
    }


    if ( events & Coordinator_WAIT_SERIES_EVT )
    {
        if ( WAIT_EVT_INDEX == Coordinator_ProcessParaSet )
        {
            /*
              if( Connect_Mode == WIRELESS_CONNECTION)
              {
                  int index;
                  time_new = osal_GetSystemClock();

                  if((time_new - time_old) > 0x0000013B)
                  {
                      for(index = 0; index < sm_max; index++)
                      {
                          if(sm_receive_flag[index] == 0)

                          {
                              sm_receive_flag[index] = 3; //Change status to ignore status
                              //routing_table[index].sm_ADD_status = 0;   //The address is invalid
                              ;

                          }
                          if(sm_receive_flag[index] == 1 || sm_receive_flag[index] == 2)
                              //sm_receive_flag[index] = 0;
                              ;
                      }
                      zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                      time_new = 0;
                      time_old = 0;
                      WAIT_EVT_INDEX = 0x0000;

                  }

                  else
                  {

                      for(index = 0; index < sm_max; index++)
                      {

                          if(routing_table[index].sm_ADD_status == 1)

                              sm_flag_and = sm_flag_and && sm_receive_flag[index];
                      }

                      if(sm_flag_and && sm_max)
                      {
                          zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);

                          for(index = 0; index < sm_max; index++)
                          {
                              if(sm_receive_flag[index] != 3)
                                  sm_receive_flag[index] = 0;
                          }

                          WAIT_EVT_INDEX = 0x0000;
                          time_new = 0;
                          time_old = 0;
                      }

                      else
                      {
                          sm_flag_and = 1;
                          WAIT_EVT_INDEX = Coordinator_ProcessParaSet;
                          osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                      }
                  }
              }
              else */
            if( Connect_Mode == WIRED_CONNECTION || Connect_Mode == WIRELESS_CONNECTION)
            {
                int index;
                time_new = osal_GetSystemClock();

                if((time_new - time_old) > 0x0000013B)
                {
                    sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                    for(index = 0; index < sm_max; index++)
                    {
                        if(sm_ADD_reg == routing_table[index].sm_ADD)
                            //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                            ;
                    }
                    sm_ADD_reg = 0;
                    flagreset = 0;

                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out

                    ENERGY_RESET_VALUE = 0xffffffff;
                    SmartMeter_ENERGY_RESET_VALUE = 0xfffffffe;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    if (para_set_rcv_flag)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                        para_set_rcv_flag = 0;
                        time_new = 0;
                        time_old = 0;
                        WAIT_EVT_INDEX = 0x0000;
                        flagreset = 0;
                    }
                    else
                    {
                        WAIT_EVT_INDEX = Coordinator_ProcessParaSet;
                        osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    }
                }
            }

        }

        else if ( WAIT_EVT_INDEX == Coordinator_EnergyResetWait )
        {
            int index;
            time_new = osal_GetSystemClock();

            if((time_new - time_old) > 0x0000013B)
            {
                sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;
                flagreset = 0;

                if(controlReg[8] != 0xFF)
                {
                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);
                }
                else
                {
                    if(write_energy_index < sm_max - 1)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                        write_energy_index++;
                    }

                    else if(write_energy_index == sm_max - 1)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                        write_energy_index = 0;
                    }
                }


                ENERGY_RESET_VALUE = 0xffffffff;
                SmartMeter_ENERGY_RESET_VALUE = 0xfffffffe;
                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (SmartMeter_ENERGY_RESET_VALUE == ENERGY_RESET_VALUE)
                {
                    if (controlReg[8] != 0xFF)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    }
                    else
                    {
                        if(write_energy_index < sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x0C, 0x03, 0x00, 0x00);
                            write_energy_index++;
                        }
                        else if(write_energy_index == sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                            write_energy_index = 0;
                        }
                    }


                    ENERGY_RESET_VALUE = 0xffffffff;
                    SmartMeter_ENERGY_RESET_VALUE = 0xfffffffe;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                    flagreset = 0;
                }
                else
                {
                    WAIT_EVT_INDEX = Coordinator_EnergyResetWait;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
        }

        else if ( WAIT_EVT_INDEX == Smartmeter_Auth )
        {
            int index;
            time_new = osal_GetSystemClock();
            if((time_new - time_old) > 0x00000258)
            {
                if(controlReg[8] != 0xFF)
                    sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                else
                    sm_ADD_reg = routing_table[smart_auth_index].sm_ADD;

                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;

                if(controlReg[8] != 0xFF)
                {
                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);
                }
                else
                {
                    if(smart_auth_index < sm_max - 1)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x20, 0x02);//time out
                        //routing_index++;
                        smart_auth_index++;
                    }

                    else if(smart_auth_index == sm_max - 1)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                        //routing_index = 0;
                        smart_auth_index = 0;
                    }
                }


                flag_auth = false;
                flag_key_set = false;

                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (flag_auth && flag_key_set)
                {
                    if(controlReg[8] != 0xFF)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    }
                    else
                    {
                        if(smart_auth_index < sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x20, 0x00);
                            smart_auth_index++;
                        }
                        else if(smart_auth_index == sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                            smart_auth_index = 0;
                        }
                    }

                    flag_auth = false;
                    flag_key_set = false;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    WAIT_EVT_INDEX = Smartmeter_Auth;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
        }
        /*
        else if ( WAIT_EVT_INDEX == Coordinator_Setkey )
        {
            int index;
            time_new = osal_GetSystemClock();
            if((time_new - time_old) > 0x0000013B)
            {
                sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;

                zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out

                flag_key_set = false;

                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (flag_key_set)
                {
                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);

                    flag_key_set = false;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    WAIT_EVT_INDEX = Coordinator_Setkey;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
        }*/

        else if ( WAIT_EVT_INDEX == Coordinator_RelaySet )
        {
            int index;
            time_new = osal_GetSystemClock();
            if((time_new - time_old) > 0x0000013B)
            {
                sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;

                zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out

                relay_receive_flag = 0;

                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (relay_receive_flag)
                {
                    //zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);

                    //if(SmartMeter_relay == 1)
                    if(flagrelay == 1)
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x10);
                    else if(flagrelay == 0)
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);

                    flagrelay = 2;
                    relay_receive_flag = 0;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    WAIT_EVT_INDEX = Coordinator_RelaySet;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
        }

        else if ( WAIT_EVT_INDEX == setPowerCalculation_WAIT )
        {
            int index;
            time_new = osal_GetSystemClock();
            if((time_new - time_old) > 0x0000013B)
            {
                if(controlReg[8] != 0xFF)
                    sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                else
                    sm_ADD_reg = routing_table[setpower_index].sm_ADD;

                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;

                if(controlReg[8] != 0xFF)
                {
                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);
                }
                else
                {
                    if(setpower_index < sm_max - 1)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                        setpower_index++;
                    }

                    else if(setpower_index == sm_max - 1)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                        setpower_index = 0;
                    }
                }

                start_receive_flag = 0;

                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (start_receive_flag)
                {
                    /*
                      if(SmartMeter_start == 0)
                      {
                          if (controlReg[8] != 0xFF)
                          {
                              zclCoordinator_sendACK(0x08, 0x01, 0x00, 0x00);
                          }
                          else
                          {
                              if(setpower_index < sm_max - 1)
                              {
                                      zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                                      setpower_index++;
                              }
                              else if(setpower_index == sm_max - 1)
                              {
                                      zclCoordinator_sendACK(0x08, 0x01, 0x00, 0x00);
                                      setpower_index = 0;
                              }
                          }
                      }
                    */

                    if (controlReg[8] != 0xFF)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    }
                    else
                    {
                        if(setpower_index < sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x08, 0x01, 0x00, 0x00);
                            setpower_index++;
                        }
                        else if(setpower_index == sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                            setpower_index = 0;
                        }
                    }

                    start_receive_flag = 0;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    WAIT_EVT_INDEX = setPowerCalculation_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
        }

        else if ( WAIT_EVT_INDEX == Calibration_WAIT )
        {

            //int index;
            time_new = osal_GetSystemClock();
            if((time_new - time_old) > 0x00000FA0)   ///4000ms
            {
                /*
                sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;
                */


                cal_receive_flag = 0;
                zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (cal_receive_flag == 1)
                {

                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    cal_receive_flag = 0;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    WAIT_EVT_INDEX = Calibration_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }

        }

        else if ( WAIT_EVT_INDEX == Network_WAIT )
        {
            if( Connect_Mode == WIRELESS_CONNECTION)
            {
                time_new = osal_GetSystemClock();
                if((time_new - time_old) > 0x0000013B) //600ms
                {
                    //zclCoordinator_sendACK(0x08, 0x03, 0x00 ,0x02);//time out
                    int i;
                    controlReg[0] = 0x08; // reset controlReg[0] to default
                    controlReg[1] = 0x03;// reset controlReg[1] to default
                    controlReg[2] = 0x00;// reset controlReg[2] to default
                    controlReg[3] = 0x00;// reset controlReg[3] to default


                    pack_out[0] = 0x68;
                    for(i = 1; i <= 8; i++)
                        pack_out[i] = coor_addrRegister[i - 1];
                    pack_out[9] = 0x68;
                    pack_out[10] = 0x06;
                    pack_out[11] = (uint8)((sm_max & 0xff00) >> 8);
                    pack_out[12] = (uint8)(sm_max & 0x00ff);          //sent the max of smart meter to local server
                    pack_out[13] = controlReg[0];
                    pack_out[14] = controlReg[1];
                    pack_out[15] = controlReg[2];
                    pack_out[16] = controlReg[3];
                    for(i = 0; i < 17; i++)
                        pack_out[17] += pack_out[i];
                    pack_out[18] = 0x16;
                    /*
                    UARTEnable(UART0_BASE );
                    for (i = 0; i < 18; i++)
                    {
                        UARTCharPut(UART0_BASE, pack_out[i]);
                    }
                    UARTDisable(UART0_BASE);
                    */
                    uint8 len_data = pack_out[10];
                    if(flag_encry)
                    {
                        uint8 encry_data[70] = {0};

                        for (uint8 i = 0; i < 70; i++)
                            encry_data[i] = 0xFF;
                        for (uint8 i = 0; i < len_data; i++)
                            encry_data[i] = pack_out[i + 11];

                        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

                        for(uint8 i = 0; i < (len_data / 16) ; i++)
                            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

                        pack_out[10] = len_data;
                        for (uint8 i = 0; i < len_data; i++)
                            pack_out[i + 11] = encry_data[i];

                        pack_out[len_data + 11] = 0x00;
                        for (uint8 i = 0; i < len_data + 11; i++ )
                            pack_out[len_data + 11] +=  pack_out[i];
                        pack_out[len_data + 12] = 0x16;

                    }
                    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);
                    //usbibufPush(&usbCdcInBufferData, pack_out, 18);
                    WAIT_EVT_INDEX = 0x0000;
                    time_new = 0;
                    time_old = 0;
                }
                else
                {
                    WAIT_EVT_INDEX = Network_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
            else if( Connect_Mode == WIRED_CONNECTION)
            {
                int index;
                time_new = osal_GetSystemClock();
                if((time_new - time_old) > 0x0000013B)
                {
                    if(controlReg[8] != 0xFF)
                        sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                    else
                        sm_ADD_reg = routing_table[routing_index].sm_ADD;

                    for(index = 0; index < sm_max; index++)
                    {
                        if(sm_ADD_reg == routing_table[index].sm_ADD)
                            routing_table[index].sm_ADD_status = 0;  //The address is invalid
                    }
                    sm_ADD_reg = 0;

                    if(controlReg[8] != 0xFF)
                    {
                        zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);
                    }
                    else
                    {
                        if(routing_index < sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x18, 0x03, 0x00, 0x02);//time out
                            routing_index++;
                        }

                        else if(routing_index == sm_max - 1)
                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out
                            routing_index = 0;
                        }
                    }
                    relay_receive_flag = 0;

                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    if (com_add_rcv_flag == 1)
                    {
                        if(controlReg[8] != 0xFF)
                        {
                            zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                            sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11],
                                                        controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                        }
                        else
                        {
                            sm_ADD_reg = routing_table[routing_index].sm_ADD;
                            if(routing_index < sm_max - 1)
                            {
                                zclCoordinator_sendACK(0x18, 0x03, 0x00, 0x00);
                                routing_index++;
                            }
                            else if(routing_index == sm_max - 1)
                            {
                                zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                                routing_index = 0;
                            }
                        }
                        for(index = 0; index < sm_max; index++)
                        {
                            if(sm_ADD_reg == routing_table[index].sm_ADD)
                                routing_table[index].sm_ADD_status = 1;  //The address is valid
                        }
                        sm_ADD_reg = 0;

                        com_add_rcv_flag = 0;
                        time_new = 0;
                        time_old = 0;
                        WAIT_EVT_INDEX = 0x0000;
                    }
                    else
                    {
                        WAIT_EVT_INDEX = Network_WAIT;
                        osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    }
                }
            }

        }

        else if ( WAIT_EVT_INDEX == DATA_WAIT )
        {
            HalLcdWriteString( "datawait1", HAL_LCD_LINE_6 );
            time_new = osal_GetSystemClock();

            if((time_new - time_old) > 0x00000640)//1500ms
            {
                HalLcdWriteString( "timeout", HAL_LCD_LINE_1 );
                sys_timenew = osal_GetSystemClock();
                sys_secnew = sys_secold + (uint32)((float)(sys_timenew - sys_timeold) / 1000);
                osal_ConvertUTCTime(&TimeStruct , sys_secnew);

                time_new = 0;
                time_old = 0;
                datain_complete = 0;

                if(try_count < 0)
                {
                    flaginc = 0;
                    zclCoordinator_SendRestart();
                    time_old = osal_GetSystemClock();
                    time_new = time_old;
                    WAIT_EVT_INDEX = DATA_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    try_count ++;
                }
                else
                {
                    WAIT_EVT_INDEX = 0x0000;
                    //sm_ADD_status[ack_index] = 0;
                    try_count = 0;

                    Timeout_bit = 1;
                    if (dataRegSel == 1)
                    {
                        Timeout_Ping = 1;
                    }
                    else
                    {
                        Timeout_Pong = 1;
                    }

                    //zclCoordinator_id_to_smIEEE(sm_routing_prio_table[ack_index]);
                    dataReg_Ping[0] = (uint16)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 48) & 0x000000000000FFFF); //ADD_3
                    dataReg_Ping[1] = (uint16)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 32) & 0x000000000000FFFF); //ADD_2
                    dataReg_Ping[2] = (uint16)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 16) & 0x000000000000FFFF); //ADD_1
                    dataReg_Ping[3] = (uint16)((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) & 0x000000000000FFFF); //ADD_0
                    dataReg_Ping[4] = 0;
                    dataReg_Ping[5] = 0;
                    dataReg_Ping[6] = 0;
                    len_DataReg = 0;
                    /*
                    uint8 i = 0;
                    for(i = 0; i < len_DataReg; i++)
                        dataReg_Ping[7 + i] = 0;
                    */
                    dataReg_Ping[7 + len_DataReg] = 0;   //STATUS
                    dataReg_Ping[8 + len_DataReg] = (uint16)((TimeStruct.month == 12) ? (TimeStruct.year + 1) : TimeStruct.year); //YEAR
                    dataReg_Ping[9 + len_DataReg] = (uint16)((TimeStruct.month == 12) ? 1 : (TimeStruct.month + 1)); //MONTH
                    dataReg_Ping[10 + len_DataReg] = (uint16)TimeStruct.day + 1;
                    dataReg_Ping[11 + len_DataReg] = (uint16)TimeStruct.hour;
                    dataReg_Ping[12 + len_DataReg] = (uint16)TimeStruct.minutes;
                    dataReg_Ping[13 + len_DataReg] = (uint16)TimeStruct.seconds;

                    //zclCoordinator_id_to_smIEEE(sm_routing_prio_table[ack_index]);
                    dataReg_Pong[0] = (uint16)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 48) & 0x000000000000FFFF); //ADD_3
                    dataReg_Pong[1] = (uint16)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 32) & 0x000000000000FFFF); //ADD_2
                    dataReg_Pong[2] = (uint16)(((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) >> 16) & 0x000000000000FFFF); //ADD_1
                    dataReg_Pong[3] = (uint16)((routing_table[sm_routing_prio_table[ack_index]].sm_ADD) & 0x000000000000FFFF); //ADD_0
                    dataReg_Pong[4] = 0;
                    dataReg_Pong[5] = 0;
                    dataReg_Pong[6] = 0;
                    len_DataReg = 0;
                    /*
                    uint8 i = 0;
                    for(i = 0; i < len_DataReg; i++)
                        dataReg_Pong[7 + i] = 0;
                    */
                    dataReg_Pong[7 + len_DataReg] = 0;   //STATUS
                    dataReg_Pong[8 + len_DataReg] = (uint16)TimeStruct.year;
                    dataReg_Pong[9 + len_DataReg] = (uint16)TimeStruct.month;
                    dataReg_Pong[10 + len_DataReg] = (uint16)TimeStruct.day;
                    dataReg_Pong[11 + len_DataReg] = (uint16)TimeStruct.hour;
                    dataReg_Pong[12 + len_DataReg] = (uint16)TimeStruct.minutes;
                    dataReg_Pong[13 + len_DataReg] = (uint16)TimeStruct.seconds;

                    dataRegSel = dataRegSel ^ 0x01;
                    if(first_complete == 1)
                    {
                        /*
                          if(ack_index < (sm_max - 1))
                          {
                              ack_index++;                                 //at first, the DRR command actually will send request to two Smart Meter, so the Smart Meter address need update.
                              while((sm_ADD_status[ack_index] == 0) && (ack_index < (sm_max - 1)))
                              {
                                  ack_index++;
                              }

                              if(sm_ADD_status[ack_index])
                              {
                                  osal_set_event(task_id, ACK_CS_EVT);
                                  datain_complete = 0;
                              }
                              else
                              {
                                  ack_index = 0;
                                  all_invaild_flag = 0;
                                  osal_set_event(task_id, DATA_CL_EVT);
                              }
                          }
                          else
                          {
                              ack_index = 0;
                              all_invaild_flag = 0;
                              osal_set_event(task_id, DATA_CL_EVT);
                          }
                          first_complete = 0;
                        */
                        if(ack_index < (num_prio_sm_max - 1))
                        {
                            ack_index++;                                 //at first, the DRR command actually will send request to two Smart Meter, so the Smart Meter address need update.
                            /*     while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                                 {
                                     ack_index++;
                                 }
                                 if((sm_ADD_status[ack_index] == 0) && ack_index == (sm_max - 1))
                                 {
                                     ack_index = 0;
                                     while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                                     {
                                         ack_index++;
                                     }
                                 }*/

                            osal_set_event(task_id, ACK_CS_EVT);

                        }
                        else
                        {
                            ack_index = 0;
                            /*    while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                                {
                                    ack_index++;
                                }*/
                            osal_set_event(task_id, ACK_CS_EVT);

                        }
                        first_complete = 0;

                    }
                }
            }
            else
            {
                if(datain_complete == 1)
                {
                    HalLcdWriteString( "datawait2", HAL_LCD_LINE_6 );
                    if(first_complete == 1)
                    {
                        HalLcdWriteString( "datawait3", HAL_LCD_LINE_6 );
                        if(ack_index < (num_prio_sm_max - 1))
                        {
                            ack_index++;                                 //at first, the DRR command actually will send request to two Smart Meter, so the Smart Meter address need update.
                            /*   while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                               {
                                   ack_index++;
                               }
                               if((sm_ADD_status[ack_index] == 0) && ack_index == (sm_max - 1))
                               {
                                   ack_index = 0;
                                   while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                                   {
                                       ack_index++;
                                   }
                               }*/

                            /*
                            uint32 wait_timenew = osal_GetSystemClock();
                            while (osal_GetSystemClock() - wait_timenew < 100)
                            {
                            };
                            */
                            osal_set_event(task_id, ACK_CS_EVT);
                            datain_complete = 0;
                        }
                        else
                        {
                            ack_index = 0;
                            /* while((sm_ADD_status[ack_index] == 0) && (ack_index < (num_prio_sm_max - 1)))
                             {
                                 ack_index++;
                             }*/
                            osal_set_event(task_id, ACK_CS_EVT);
                            datain_complete = 0;
                        }
                        first_complete = 0;
                    }
                    else
                    {
                        datain_complete = 0;
                    }
                    try_count = 0;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    if(Round_end_flag == 1)
                    {
                        datain_complete = 0;
                        Round_end_flag = 0;
                        time_new = 0;
                        time_old = 0;
                        try_count = 0;
                        WAIT_EVT_INDEX = 0x0000;
                    }
                    else
                    {
                        WAIT_EVT_INDEX = DATA_WAIT;
                        osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                    }
                }
            }
        }

        else if ( WAIT_EVT_INDEX == GETCAL_WAIT )
        {
            int index;
            time_new = osal_GetSystemClock();
            if((time_new - time_old) > 0x0000013B)
            {
                sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;

                zclCoordinator_calregtimeout();//time out

                flag_calget = 0;

                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (flag_calget)
                {
                    zclCoordinator_sendcalreg();
                    flag_calget = 0;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    WAIT_EVT_INDEX = GETCAL_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
        }

        else if ( WAIT_EVT_INDEX ==  SETCONFIG_WAIT )
        {
            int index;
            time_new = osal_GetSystemClock();
            if((time_new - time_old) > 0x0000013B)
            {
                sm_ADD_reg = BUILD_UINT64_8(controlReg[8], controlReg[9], controlReg[10], controlReg[11], controlReg[12], controlReg[13], controlReg[14], controlReg[15]);
                for(index = 0; index < sm_max; index++)
                {
                    if(sm_ADD_reg == routing_table[index].sm_ADD)
                        //routing_table[index].sm_ADD_status = 0;  //The address is invalid
                        ;
                }
                sm_ADD_reg = 0;

                zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x02);//time out

                flag_config_reg = 0;

                time_new = 0;
                time_old = 0;
                WAIT_EVT_INDEX = 0x0000;
            }
            else
            {
                if (flag_config_reg)
                {
                    zclCoordinator_sendACK(0x08, 0x03, 0x00, 0x00);
                    flag_config_reg = 0;
                    time_new = 0;
                    time_old = 0;
                    WAIT_EVT_INDEX = 0x0000;
                }
                else
                {
                    WAIT_EVT_INDEX = SETCONFIG_WAIT;
                    osal_set_event(task_id, Coordinator_WAIT_SERIES_EVT);
                }
            }
        }
        return ( events ^ Coordinator_WAIT_SERIES_EVT );
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
        /*
          uint8 send_buffer[] = {
                                   0x68, 0x00, 0x12, 0x4B, 0x00, 0x04, 0x13, 0x31,
                                   0x76, 0x68, 0x10, 0xDD, 0xB2, 0x1E, 0xAE, 0x59,
                                   0x63, 0x59, 0x68, 0x98, 0xA5, 0xDF, 0x67, 0xFD,
                                   0x53, 0x1F, 0x82, 0x47, 0x16
                                };

            uint8 encry_data[60] = {0};

            uint8 len_data = send_buffer[10];
            for (uint8 i = 0; i < len_data; i++)
                 encry_data[i] = send_buffer[i + 11];

            len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

            for(uint8 i = 0; i < (len_data / 16) ; i++)
                AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, DECRYPT_AES);

            send_buffer[10] = len_data;
            for (uint8 i = 0; i < len_data; i++)
                send_buffer[i + 11] = encry_data[i];

            send_buffer[len_data + 11] = 0x00;
            for (uint8 i = 0; i < len_data + 11; i++ )
                send_buffer[len_data + 11] +=  send_buffer[i];
            send_buffer[len_data + 12] = 0x16;

            HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
          */
        /*
        //routing table write 1
        //68 00 12 4B 00 04 13 31
        //  76 68 38 08 02 01 00 00
        //  00 00 00 00 00 00 00 00
        //  00 00 00 00 00 00 00 00
        //  00 00 00 00 00 00 00 00
        //  00 00 00 00 00 00 00 00
        //  00 00 00 00 00 00 00 00
        //  00 00 00 00 00 00 00 00
        // 00 00 00 9E 16
        uint8 send_buffer[] = {  0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                                 0x76, 0x68, 0x08, 0x08, 0x02, 0x01, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x9e, 0x16
                             };

          uint8 encry_data[60] = {0};

          uint8 len_data = send_buffer[10];
          for (uint8 i = 0; i < len_data; i++)
               encry_data[i] = send_buffer[i + 11];

          len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

          for(uint8 i = 0; i < (len_data / 16) ; i++)
              AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

          send_buffer[10] = len_data;
          for (uint8 i = 0; i < len_data; i++)
              send_buffer[i + 11] = encry_data[i];

          send_buffer[len_data + 11] = 0x00;
          for (uint8 i = 0; i < len_data + 11; i++ )
              send_buffer[len_data + 11] +=  send_buffer[i];
          send_buffer[len_data + 12] = 0x16;

          HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
        */

        //routing table write 2
        //68 00 12 4B 00 04 13 31
        //76 68 2A 00 12 4B 00 04
        //13 31 BF 01 01 11 12 13
        //14 15 16 17 18 AA BB CC
        //DD EE FF 00 11 11 22 33
        //44 55 66 77 88 11 22 33
        //44 55 66 77 88 55 16
        /*
         uint8 send_buffer[] = { 0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                                 0x76, 0x68, 0x2A, 0x00, 0x12, 0x4B, 0x00, 0x04,
                                 0x13, 0x36, 0x98, 0x01, 0x01, 0x11, 0x12, 0x13,
                                 0x14, 0x15, 0x16, 0x17, 0x18, 0xAA, 0xBB, 0xCC,
                                 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x11, 0x22, 0x33,
                                 0x44, 0x55, 0x66, 0x77, 0x88, 0x11, 0x22, 0x33,
                                 0x44, 0x55, 0x66, 0x77, 0x88, 0x9e, 0x16
                             };

          uint8 encry_data[60] = {0};

          uint8 len_data = send_buffer[10];
          for (uint8 i = 0; i < len_data; i++)
               encry_data[i] = send_buffer[i + 11];

          len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

          for(uint8 i = 0; i < (len_data / 16) ; i++)
              AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

          send_buffer[10] = len_data;
          for (uint8 i = 0; i < len_data; i++)
              send_buffer[i + 11] = encry_data[i];

          send_buffer[len_data + 11] = 0x00;
          for (uint8 i = 0; i < len_data + 11; i++ )
              send_buffer[len_data + 11] +=  send_buffer[i];
          send_buffer[len_data + 12] = 0x16;

          HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
          */

        //Authenticate smart
        //68 00 12 4B 00 04 13 31
        //76 68 38 08 02 20 00 00
        //00 00 00 00 12 4B 00 04
        //13 31 BF 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 9E 16
        /*
        uint8 send_buffer[69] = {0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                               0x76, 0x68, 0x38, 0x08, 0x02, 0x20, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x12, 0x4B, 0x00, 0x04,
                               0x13, 0x31, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x9e, 0x16
                               };

        uint8 encry_data[56];

        uint8 len_data = send_buffer[10];
        for (uint8 i = 0; i < len_data; i++)
             encry_data[i] = send_buffer[i + 11];

        len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

        for(uint8 i = 0; i < (len_data / 16) ; i++)
            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

        send_buffer[10] = len_data;
        for (uint8 i = 0; i < len_data; i++)
            send_buffer[i + 11] = encry_data[i];

        send_buffer[len_data + 11] = 0x00;
        for (uint8 i = 0; i < len_data + 11; i++ )
            send_buffer[len_data + 11] +=  send_buffer[i];
        send_buffer[len_data + 12] = 0x16;
        HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
        */

        /*
                //network discovery
                //68 00 12 4B 00 04 13 31
                //76 68 38 18 02 00 00 00
                //00 00 00 00 12 4B 00 04
                //13 31 BF 00 00 00 00 00
                //00 00 00 00 00 00 00 00
                //00 00 00 00 00 00 00 00
                //00 00 00 00 00 00 00 00
                //00 00 00 00 00 00 00 00
                //00 00 00 9E 16
                uint8 send_buffer[] = { 0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                                        0x76, 0x68, 0x38, 0x18, 0x02, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff,
                                        0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x9e, 0x16
                                      };

                uint8 encry_data[70] = {0};

                uint8 len_data = send_buffer[10];
                for (uint8 i = 0; i < len_data; i++)
                    encry_data[i] = send_buffer[i + 11];

                len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

                for(uint8 i = 0; i < (len_data / 16) ; i++)
                    AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

                send_buffer[10] = len_data;
                for (uint8 i = 0; i < len_data; i++)
                    send_buffer[i + 11] = encry_data[i];

                send_buffer[len_data + 11] = 0x00;
                for (uint8 i = 0; i < len_data + 11; i++ )
                    send_buffer[len_data + 11] +=  send_buffer[i];
                send_buffer[len_data + 12] = 0x16;

                HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
        */
        //////////////////////////////////////////////////////////////////////////////////////////////////
        //68 00 12 4B 00 04 13 31
        //76 68 10 2B 75 2F 60 22
        //00 E0 F2 65 6D 2C 68 B7
        //A8 59 E7 23 16
        /*
          uint8 send_buffer[] = { 0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                                 0x76, 0x68, 0x10, 0x2b, 0x75, 0x2f, 0x60, 0x22,
                                 0x00, 0xe0, 0xf2, 0x65, 0x6d, 0x2c, 0x68, 0xb7,
                                 0xa8, 0x59, 0xe7, 0x23, 0x16
                             };

          uint8 encry_data[60] = {0};

          uint8 len_data = send_buffer[10];
          for (uint8 i = 0; i < len_data; i++)
               encry_data[i] = send_buffer[i + 11];

          len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

          for(uint8 i = 0; i < (len_data / 16) ; i++)
              AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, DECRYPT_AES);

          send_buffer[10] = len_data;
          for (uint8 i = 0; i < len_data; i++)
              send_buffer[i + 11] = encry_data[i];

          send_buffer[len_data + 11] = 0x00;
          for (uint8 i = 0; i < len_data + 11; i++ )
              send_buffer[len_data + 11] +=  send_buffer[i];
          send_buffer[len_data + 12] = 0x16;

          //HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
        */

        //Authenticate smart
        //68 00 12 4B 00 04 13 31
        //76 68 38 08 02 10 00 00
        //00 00 00 00 12 4B 00 04
        //13 31 BF 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 9E 16
        /*
            uint8 send_buffer[69] = { 0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                                   0x76, 0x68, 0x38, 0x08, 0x02, 0x20, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff,
                                   0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x9e, 0x16
                               };

            uint8 encry_data[70] = {0};

            uint8 len_data = send_buffer[10];
            for (uint8 i = 0; i < len_data; i++)
                 encry_data[i] = send_buffer[i + 11];

            len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

            for(uint8 i = 0; i < (len_data / 16) ; i++)
                AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

            send_buffer[10] = len_data;
            for (uint8 i = 0; i < len_data; i++)
                send_buffer[i + 11] = encry_data[i];

            send_buffer[len_data + 11] = 0x00;
            for (uint8 i = 0; i < len_data + 11; i++ )
                send_buffer[len_data + 11] +=  send_buffer[i];
            send_buffer[len_data + 12] = 0x16;

            HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
        */


        //data register read
        //68 00 12 4B 00 04 13 31
        //76 68 38 88 02 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 00 00 00 00 00
        //00 00 00 E6 16
        /*
                uint8 send_buffer[69] = { 0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                                       0x76, 0x68, 0x38, 0x08, 0x00, 0x00, 0x08, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x8c, 0x16
                                   };

                uint8 encry_data[60] = {0};

                uint8 len_data = send_buffer[10];
                for (uint8 i = 0; i < len_data; i++)
                     encry_data[i] = send_buffer[i + 11];

                len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

                for(uint8 i = 0; i < (len_data / 16) ; i++)
                    AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

                send_buffer[10] = len_data;
                for (uint8 i = 0; i < len_data; i++)
                    send_buffer[i + 11] = encry_data[i];

                send_buffer[len_data + 11] = 0x00;
                for (uint8 i = 0; i < len_data + 11; i++ )
                    send_buffer[len_data + 11] +=  send_buffer[i];
                send_buffer[len_data + 12] = 0x16;
                HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
                */

        //ack
        //68 00 12 4B 00 04 13 31
        //76 68 04 88 02 00 08 CA
        //16
        /*
        uint8 send_buffer[] = { 0x68, 0x00, 0x12, 0x4b, 0x00, 0x04, 0x13, 0x31,
                                0x76, 0x68, 0x04, 0x08, 0x02, 0x20, 0x08, 0x8c,
                                0x16
                           };

        uint8 encry_data[60] = {0};

        uint8 len_data = send_buffer[10];
        for (uint8 i = 0; i < len_data; i++)
             encry_data[i] = send_buffer[i + 11];

        len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;

        for(uint8 i = 0; i < (len_data / 16) ; i++)
            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

        send_buffer[10] = len_data;
        for (uint8 i = 0; i < len_data; i++)
            send_buffer[i + 11] = encry_data[i];

        send_buffer[len_data + 11] = 0x00;
        for (uint8 i = 0; i < len_data + 11; i++ )
            send_buffer[len_data + 11] +=  send_buffer[i];
        send_buffer[len_data + 12] = 0x16;
        HalUART0Write ( HAL_UART_PORT_0, send_buffer, len_data + 13);
        */
        //zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_CONFIG_STATE|ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
        //SystemResetSoft();
        //ROM_ResetDevice();
        /*
        int status1 = ROM_PageErase(0x0027f800,0x800);
        uint32_t pulData[4] = {0xffffffff, 0xf0ffffff, 0x00000001, 0x00200000};
        int status2 = ROM_ProgramFlash(pulData, 0x0027ffd0, 16);

        //zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_CONFIG_STATE|ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
        //SystemResetSoft();
        if (status1 == 0 && status2 == 0)
            ROM_ResetDevice();
        */
        /*
        uint8 send_buffer[2] = {0x55, 0x55};
        HalUART1Write ( HAL_UART_PORT_1, send_buffer, 2);
        */
      
      
      //zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_CONFIG_STATE|ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
       SystemResetSoft();
    }

    if ( keys & HAL_KEY_SW_2 )
    {
        zclCoordinator_SetTime();
    }

    if ( keys & HAL_KEY_SW_3 )
    {
        zclscoordinator_startEZmode();
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      #ifdef LCD_SUPPORTED
        HalLcdWriteString( "Bind", HAL_LCD_LINE_3 );
      #endif
      giThermostatScreenMode = THERMOSTAT_MAINMODE;
      zAddrType_t dstAddr;
      HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );

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
      initUSB();
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
    //HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
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

    //HalLcdWriteString( (char *)sDisplayVoltage, HAL_LCD_LINE_1 );
    //HalLcdWriteString( (char *)sDisplayCurrent, HAL_LCD_LINE_2 );
    //HalLcdWriteString( (char *)sDisplayEnergy, HAL_LCD_LINE_3 );
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
    //HalLcdWriteString( (char *)sDisplayCoIEEEaddr, HAL_LCD_LINE_1 );
#endif

}

void zclCoordinator_LcdDisplayTestMode_smaddr( void )
{
    char sDisplayCoIEEEaddr[32];
    uint8 IEEEaddr[8];

    // display coordinator IEEE addr
    osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
    //if ( (sm_ADD_0[sm_index] == NULL) || (sm_ADD_1[sm_index] == NULL) || (sm_ADD_2[sm_index] == NULL) || (sm_ADD_3[sm_index] == NULL))
    if(routing_table[sm_index].sm_ADD == NULL)
    {
        osal_memcpy( &sDisplayCoIEEEaddr[5], "N/A", 4 );
    }
    else
    {
        osal_memcpy(sDisplayCoIEEEaddr, "IEEE:", 5);
        /*
        IEEEaddr[0] = (sm_ADD_3[sm_index] >> 8) & 0x00FF; // highest bit: ADD_3
        IEEEaddr[1] = sm_ADD_3[sm_index] & 0x00FF;
        IEEEaddr[2] = (sm_ADD_2[sm_index] >> 8) & 0x00FF;
        IEEEaddr[3] = sm_ADD_2[sm_index] & 0x00FF;
        IEEEaddr[4] = (sm_ADD_1[sm_index] >> 8) & 0x00FF;
        IEEEaddr[5] = sm_ADD_1[sm_index] & 0x00FF;
        IEEEaddr[6] = (sm_ADD_0[sm_index] >> 8) & 0x00FF;
        IEEEaddr[7] = sm_ADD_0[sm_index] & 0x00FF;
        */
        IEEEaddr[0] = (uint8)(((routing_table[sm_index].sm_ADD) >> 56) & 0x00000000000000FF);
        IEEEaddr[1] = (uint8)(((routing_table[sm_index].sm_ADD) >> 48) & 0x00000000000000FF);
        IEEEaddr[2] = (uint8)(((routing_table[sm_index].sm_ADD) >> 40) & 0x00000000000000FF);
        IEEEaddr[3] = (uint8)(((routing_table[sm_index].sm_ADD) >> 32) & 0x00000000000000FF);
        IEEEaddr[4] = (uint8)(((routing_table[sm_index].sm_ADD) >> 24) & 0x00000000000000FF);
        IEEEaddr[5] = (uint8)(((routing_table[sm_index].sm_ADD) >> 16) & 0x00000000000000FF);
        IEEEaddr[6] = (uint8)(((routing_table[sm_index].sm_ADD) >> 8) & 0x00000000000000FF);
        IEEEaddr[7] = (uint8)((routing_table[sm_index].sm_ADD) & 0x00000000000000FF);
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
  #ifdef LCD_SUPPORTED
        HalLcdWriteString( "Received", HAL_LCD_LINE_4 );
#endif
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
  
    #ifdef LCD_SUPPORTED
        HalLcdWriteString( "Received report", HAL_LCD_LINE_4 );
#endif
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
        #ifdef LCD_SUPPORTED
          HalLcdWriteString( "set_para_ack", HAL_LCD_LINE_4 );
        #endif
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

        para_set_rcv_flag = 1;
        /*
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
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((routing_table[sm_id].sm_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((routing_table[sm_id].sm_ADD) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((routing_table[sm_id].sm_ADD) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((routing_table[sm_id].sm_ADD) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((routing_table[sm_id].sm_ADD) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((routing_table[sm_id].sm_ADD) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((routing_table[sm_id].sm_ADD) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((routing_table[sm_id].sm_ADD) & 0x00000000000000FF);
                zclCoordinator_SetParam();

                sm_retry_times[sm_id]--;

            }
        }*/
    }

    if ((OPERATION == TIME_SET) && (RESULT == SUCCESS) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_COM_MEASURED_VALUE))
    {
        // store the current time value sent over the air from smart meter *
        #ifdef LCD_SUPPORTED
          HalLcdWriteString( "TIME_SET", HAL_LCD_LINE_4 );
        #endif
        SmartMeterTimeReg[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        SmartMeterTimeReg[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        SmartMeterTimeReg[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        SmartMeterTimeReg[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        SmartMeterTimeReg[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        SmartMeterTimeReg[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        sm_id = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);

        para_set_rcv_flag = 1;
        /*
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
                zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((routing_table[sm_id].sm_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
                zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((routing_table[sm_id].sm_ADD) >> 48) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((routing_table[sm_id].sm_ADD) >> 40) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((routing_table[sm_id].sm_ADD) >> 32) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((routing_table[sm_id].sm_ADD) >> 24) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((routing_table[sm_id].sm_ADD) >> 16) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((routing_table[sm_id].sm_ADD) >> 8) & 0x00000000000000FF);
                zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((routing_table[sm_id].sm_ADD) & 0x00000000000000FF);
                zclCoordinator_SetTime();

                sm_retry_times[sm_id]--;

            }
        }
        */
    }

    //Process incomming data report from smart meter in response to GET date command *
    if ((OPERATION == USR_TX_GET) && (RESULT == SUCCESS) &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_DATA_MEASURED_VALUE))
    {
        #ifdef LCD_SUPPORTED
          HalLcdWriteString( "PingPong", HAL_LCD_LINE_4 );
        #endif
        /*
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
        */
        // store the current data value sent over the air from smart meter
        if (dataRegSel == 0)
        {
            dataReg_Pong[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
            dataReg_Pong[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
            dataReg_Pong[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
            dataReg_Pong[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
            dataReg_Pong[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
            dataReg_Pong[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
            dataReg_Pong[6] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);

            len_DataReg = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]);
            uint8 i = 0;
            for(i = 0; i < len_DataReg; i++)
                dataReg_Pong[7 + i] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20 + i * 2], pInParameterReport->attrList[0].attrData[21 + i * 2]);
            dataReg_Pong[7 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[21 + len_DataReg * 2]);
            dataReg_Pong[8 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[23 + len_DataReg * 2]);
            dataReg_Pong[9 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[25 + len_DataReg * 2]);
            dataReg_Pong[10 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[27 + len_DataReg * 2]);
            dataReg_Pong[11 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[28 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[29 + len_DataReg * 2]);
            dataReg_Pong[12 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[30 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[31 + len_DataReg * 2]);
            dataReg_Pong[13 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[32 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[33 + len_DataReg * 2]);
            /*
            int i;
            UARTEnable(UART0_BASE);
            for(i = 0; i < 23; i++)
                UARTCharPut(UART0_BASE, dataReg_Pong[i]);
            UARTDisable(UART0_BASE);
            */
            //usbibufPush(&usbCdcInBufferData, dataReg_Pong, 23);
        }
        else if(dataRegSel == 1)
        {

            dataReg_Ping[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
            dataReg_Ping[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
            dataReg_Ping[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
            dataReg_Ping[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
            dataReg_Ping[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
            dataReg_Ping[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
            dataReg_Ping[6] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);

            len_DataReg = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]);
            uint8 i = 0;
            for(i = 0; i < len_DataReg; i++)
                dataReg_Ping[7 + i] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20 + i * 2], pInParameterReport->attrList[0].attrData[21 + i * 2]);
            dataReg_Ping[7 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[21 + len_DataReg * 2]);
            dataReg_Ping[8 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[23 + len_DataReg * 2]);
            dataReg_Ping[9 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[25 + len_DataReg * 2]);
            dataReg_Ping[10 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[27 + len_DataReg * 2]);
            dataReg_Ping[11 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[28 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[29 + len_DataReg * 2]);
            dataReg_Ping[12 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[30 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[31 + len_DataReg * 2]);
            dataReg_Ping[13 + len_DataReg] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[32 + len_DataReg * 2], pInParameterReport->attrList[0].attrData[33 + len_DataReg * 2]);
            /*
            int i;
            UARTEnable(UART0_BASE);
            for(i = 0; i < 23; i++)
                UARTCharPut(UART0_BASE, dataReg_Ping[i]);
            UARTDisable(UART0_BASE);
            */
            //usbibufPush(&usbCdcInBufferData, dataReg_Pong, 23);
        }
        dataRegSel = dataRegSel ^ 0x01;
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
        #ifdef LCD_SUPPORTED
          HalLcdWriteString( "USR_TX_GET", HAL_LCD_LINE_4 );
        #endif
        /*
        sm_ADD_3[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        sm_ADD_2[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        sm_ADD_1[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        sm_ADD_0[sm_index] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        */
        routing_table[sm_index].sm_ADD = BUILD_UINT64_8(pInParameterReport->attrList[0].attrData[5], pInParameterReport->attrList[0].attrData[4],
                                         pInParameterReport->attrList[0].attrData[7], pInParameterReport->attrList[0].attrData[6],
                                         pInParameterReport->attrList[0].attrData[9], pInParameterReport->attrList[0].attrData[8],
                                         pInParameterReport->attrList[0].attrData[11], pInParameterReport->attrList[0].attrData[10]);

        zclCoordinator_LcdDisplayTestMode_smaddr();

        zclCoordinator_DstAddr.addrMode = (afAddrMode_t)Addr64Bit;
        zclCoordinator_DstAddr.endPoint = Coordinator_ENDPOINT;
        zclCoordinator_DstAddr.addr.extAddr[7] = (uint8)(((routing_table[sm_index].sm_ADD) >> 56) & 0x00000000000000FF); //AF.h; highest
        zclCoordinator_DstAddr.addr.extAddr[6] = (uint8)(((routing_table[sm_index].sm_ADD) >> 48) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[5] = (uint8)(((routing_table[sm_index].sm_ADD) >> 40) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[4] = (uint8)(((routing_table[sm_index].sm_ADD) >> 32) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[3] = (uint8)(((routing_table[sm_index].sm_ADD) >> 24) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[2] = (uint8)(((routing_table[sm_index].sm_ADD) >> 16) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[1] = (uint8)(((routing_table[sm_index].sm_ADD) >> 8) & 0x00000000000000FF);
        zclCoordinator_DstAddr.addr.extAddr[0] = (uint8)((routing_table[sm_index].sm_ADD) & 0x00000000000000FF);


        //Ack smart meter with routing_table[index].sm_ADD
        zclCoordinator_SendAck();

        // for test
#ifdef LCD_SUPPORTED
        //HalLcdWriteString( "Nwkdiscov Back", HAL_LCD_LINE_4 );
#endif

        sm_index++; //increment index
        sm_max++; //total number of smart meter detected
    }

    if ((OPERATION == COM_CAL) && (RESULT == SUCCESS) &&
            ((pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE ))
    {

        calsm_id = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        calSMADD_3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        calSMADD_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        calSMADD_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        calSMADD_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[12], pInParameterReport->attrList[0].attrData[13]);
        SM_CONFIG_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[14], pInParameterReport->attrList[0].attrData[15]);
        SM_CONFIG_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[16], pInParameterReport->attrList[0].attrData[17]);
        SM_CONFIG_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[18], pInParameterReport->attrList[0].attrData[19]);
        calMAG_V[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[20], pInParameterReport->attrList[0].attrData[21]);
        calMAG_I[0] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[22], pInParameterReport->attrList[0].attrData[23]);
        calMAG_V[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[24], pInParameterReport->attrList[0].attrData[25]);
        calMAG_I[1] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[26], pInParameterReport->attrList[0].attrData[27]);
        calMAG_V[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[28], pInParameterReport->attrList[0].attrData[29]);
        calMAG_I[2] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[30], pInParameterReport->attrList[0].attrData[31]);
        calMAG_V[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[32], pInParameterReport->attrList[0].attrData[33]);
        calMAG_I[3] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[34], pInParameterReport->attrList[0].attrData[35]);
        calMAG_V[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[36], pInParameterReport->attrList[0].attrData[37]);
        calMAG_I[4] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[38], pInParameterReport->attrList[0].attrData[39]);
        calMAG_V[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[40], pInParameterReport->attrList[0].attrData[41]);
        calMAG_I[5] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[42], pInParameterReport->attrList[0].attrData[43]);
        calMAG_V[6] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[44], pInParameterReport->attrList[0].attrData[45]);
        calMAG_I[6] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[46], pInParameterReport->attrList[0].attrData[47]);
        calMAG_V[7] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[48], pInParameterReport->attrList[0].attrData[49]);
        calMAG_I[7] = BUILD_UINT16(pInParameterReport->attrList[0].attrData[50], pInParameterReport->attrList[0].attrData[51]);
        calT_EFF = BUILD_UINT16(pInParameterReport->attrList[0].attrData[52], pInParameterReport->attrList[0].attrData[53]);

        flag_calget = 1;
        // for test
#ifdef LCD_SUPPORTED
        //HalLcdWriteString( "Get Calpara", HAL_LCD_LINE_4 );
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
        //HalLcdWriteString( "SendResetBack SUCCESS", HAL_LCD_LINE_3 );
        //HalLcdWriteString( (char *)sflagreset, HAL_LCD_LINE_4 );
        //HalLcdWriteString( (char *)sENERGY_RESET_VALUE, HAL_LCD_LINE_5 );
        // //HalLcdWriteString( (char *)sENERGY_RESET_VALUE_1, HAL_LCD_LINE_6 );
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
            //flagrelay = 2;
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
    if ((OPERATION == TEMP_STOP) && (RESULT == SUCCESS)  &&
            (pInParameterReport->attrList[0].attrID == ATTRID_MS_ADD_MEASURED_VALUE))
    {
        HalLcdWriteString( "Send Data request", HAL_LCD_LINE_3 );
        SmartMeter_flaginc = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        zclCoordinator_SendData();   //send request to get data
    }

    if ((OPERATION == COM_CONFIG) && (RESULT == SUCCESS)  &&
            (pInParameterReport->attrList[0].attrID == ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT))
    {

        uint16 get_SM_CONFIG_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        uint16 get_SM_CONFIG_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        uint16 get_SM_CONFIG_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);

        if( get_SM_CONFIG_2 == SM_CONFIG_2 && get_SM_CONFIG_1 == SM_CONFIG_1 && get_SM_CONFIG_0 == SM_CONFIG_0 )
            flag_config_reg = 1;
    }

}
#endif  // ZCL_REPORT


/*********************************************************************
 * @fn      Process_Wired_Cmd
 *
 * @brief   Process the wired Command from SMART METER
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static void Process_Wired_Cmd(void)
{
    /*
      uint8_t ui8AESKey[16] =  { 0x00, 0x12, 0x4b, 0x00, 0x04, 0x0f, 0x98, 0x00,
          0x00, 0x12, 0x4b, 0x00, 0x04, 0x0f, 0x98, 0x00 };
    */
    char  lcdString[10];
    //sprintf((char *)lcdString, "%d %d",(int)outing_table[end_index].flag_end_encry, (int)end_index);
    //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
    if (routing_table[end_index].flag_end_encry)
    {
        uint8 len_data = USB_Msg_in[10];

        uint8 encry_data[70] = {0};

        for (uint8 i = 0; i < len_data; i++)
            encry_data[i] = USB_Msg_in[i + 11];

        for(uint8 i = 0; i < ((len_data % 16) ? (len_data / 16 + 1) : (len_data / 16)) ; i++)
            AesEncryptDecrypt(end_final_key, encry_data + 16 * i, 0, DECRYPT_AES);

        for (uint8 i = 0; i < len_data; i++)
            USB_Msg_in[i + 11] = encry_data[i];
    }

    uint16 OPERATION = UINT8_TO_16(USB_Msg_in[11], USB_Msg_in[12]);
    uint16 RESULT = UINT8_TO_16(USB_Msg_in[13], USB_Msg_in[14]);

    uint16 ENERGY_RESET_VALUE_1;
    uint16 ENERGY_RESET_VALUE_0;

    //char  lcdString[10];
    //sprintf((char *)lcdString, "%x %x", (uint8)OPERATION, (uint8)RESULT );
    //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

    if ((OPERATION == SET_PARAM) && (RESULT == SUCCESS))
    {
        // store the current parameter value sent over the air from smart meter *
        SmartMeterparamReg[0] = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]); //MIN_ADC
        SmartMeterparamReg[1] = UINT8_TO_16(USB_Msg_in[17], USB_Msg_in[18]); //MAX_ADC
        SmartMeterparamReg[2] = UINT8_TO_16(USB_Msg_in[19], USB_Msg_in[20]); //SAMPLE_INT
        SmartMeterparamReg[3] = UINT8_TO_16(USB_Msg_in[21], USB_Msg_in[22]); //SAMPLE_WIN
        SmartMeterparamReg[4] = UINT8_TO_16(USB_Msg_in[23], USB_Msg_in[24]); //MIN_V
        SmartMeterparamReg[5] = UINT8_TO_16(USB_Msg_in[25], USB_Msg_in[26]); //MAX_V
        SmartMeterparamReg[6] = UINT8_TO_16(USB_Msg_in[27], USB_Msg_in[28]); //MIN_I
        SmartMeterparamReg[7] = UINT8_TO_16(USB_Msg_in[29], USB_Msg_in[30]); //MAX_I
        SmartMeterparamReg[8] = UINT8_TO_16(USB_Msg_in[31], USB_Msg_in[32]); //SHARP1
        SmartMeterparamReg[9] = UINT8_TO_16(USB_Msg_in[33], USB_Msg_in[34]); //SHARP2
        SmartMeterparamReg[10] = UINT8_TO_16(USB_Msg_in[35], USB_Msg_in[36]); //PEAK1
        SmartMeterparamReg[11] = UINT8_TO_16(USB_Msg_in[37], USB_Msg_in[38]); //PEAK2
        SmartMeterparamReg[12] = UINT8_TO_16(USB_Msg_in[39], USB_Msg_in[40]); //PEAK3
        SmartMeterparamReg[13] = UINT8_TO_16(USB_Msg_in[41], USB_Msg_in[42]); //SHOULDER1
        SmartMeterparamReg[14] = UINT8_TO_16(USB_Msg_in[43], USB_Msg_in[44]); //SHOULDER2
        SmartMeterparamReg[15] = UINT8_TO_16(USB_Msg_in[45], USB_Msg_in[46]); //SHOULDER3
        SmartMeterparamReg[16] = UINT8_TO_16(USB_Msg_in[47], USB_Msg_in[48]); //OFF
        SmartMeterparamReg[17] = UINT8_TO_16(USB_Msg_in[49], USB_Msg_in[50]); //N_SM
        sm_id = UINT8_TO_16(USB_Msg_in[51], USB_Msg_in[52]);
        /*
        if(!zclCoordinator_SmartMeterParamCompare())
        {
            sm_receive_flag[sm_id] = 1;
        }
        */
        para_set_rcv_flag = 1;

    }

    if ((OPERATION == TIME_SET) && (RESULT == SUCCESS))
    {
        // store the current time value sent over the air from smart meter *

        SmartMeterTimeReg[0] = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]);
        SmartMeterTimeReg[1] = UINT8_TO_16(USB_Msg_in[17], USB_Msg_in[18]);
        SmartMeterTimeReg[2] = UINT8_TO_16(USB_Msg_in[19], USB_Msg_in[20]);
        SmartMeterTimeReg[3] = UINT8_TO_16(USB_Msg_in[21], USB_Msg_in[22]);
        SmartMeterTimeReg[4] = UINT8_TO_16(USB_Msg_in[23], USB_Msg_in[24]);
        SmartMeterTimeReg[5] = UINT8_TO_16(USB_Msg_in[25], USB_Msg_in[26]);
        sm_id = UINT8_TO_16(USB_Msg_in[27], USB_Msg_in[28]);
        /*
                if(!zclCoordinator_SmartMeterTimeCompare())
                {
                    sm_receive_flag[sm_id] = 1;
                }
                */
        para_set_rcv_flag = 1;
    }

    if ((OPERATION == COM_DATA) && (RESULT == SUCCESS))  //data register read ... later
    {

        // store the current data value sent over the air from smart meter
        if (dataRegSel == 0)
        {
            dataReg_Pong[0] = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]); //ADD_3
            dataReg_Pong[1] = UINT8_TO_16(USB_Msg_in[17], USB_Msg_in[18]); //ADD_2
            dataReg_Pong[2] = UINT8_TO_16(USB_Msg_in[19], USB_Msg_in[20]); //ADD_1
            dataReg_Pong[3] = UINT8_TO_16(USB_Msg_in[21], USB_Msg_in[22]); //ADD_0
            dataReg_Pong[4] = UINT8_TO_16(USB_Msg_in[23], USB_Msg_in[24]);
            dataReg_Pong[5] = UINT8_TO_16(USB_Msg_in[25], USB_Msg_in[26]);
            dataReg_Pong[6] = UINT8_TO_16(USB_Msg_in[27], USB_Msg_in[28]);

            len_DataReg = UINT8_TO_16(USB_Msg_in[29], USB_Msg_in[30]);
            uint8 i = 0;
            for(i = 0; i < len_DataReg; i++)
                dataReg_Pong[7 + i] = UINT8_TO_16(USB_Msg_in[31 + i * 2], USB_Msg_in[32 + i * 2]);
            dataReg_Pong[7 + len_DataReg] = UINT8_TO_16(USB_Msg_in[31 + len_DataReg * 2], USB_Msg_in[32 + len_DataReg * 2]);
            dataReg_Pong[8 + len_DataReg] = UINT8_TO_16(USB_Msg_in[33 + len_DataReg * 2], USB_Msg_in[34 + len_DataReg * 2]);
            dataReg_Pong[9 + len_DataReg] = UINT8_TO_16(USB_Msg_in[35 + len_DataReg * 2], USB_Msg_in[36 + len_DataReg * 2]);
            dataReg_Pong[10 + len_DataReg] = UINT8_TO_16(USB_Msg_in[37 + len_DataReg * 2], USB_Msg_in[38 + len_DataReg * 2]);
            dataReg_Pong[11 + len_DataReg] = UINT8_TO_16(USB_Msg_in[39 + len_DataReg * 2], USB_Msg_in[40 + len_DataReg * 2]);
            dataReg_Pong[12 + len_DataReg] = UINT8_TO_16(USB_Msg_in[41 + len_DataReg * 2], USB_Msg_in[42 + len_DataReg * 2]);
            dataReg_Pong[13 + len_DataReg] = UINT8_TO_16(USB_Msg_in[43 + len_DataReg * 2], USB_Msg_in[44 + len_DataReg * 2]);
            /*
            int i;
            UARTEnable(UART0_BASE);
            for(i = 0; i < 23; i++)
                UARTCharPut(UART0_BASE, dataReg_Pong[i]);
            UARTDisable(UART0_BASE);
            */
            //usbibufPush(&usbCdcInBufferData, dataReg_Pong, 23);
        }
        else if(dataRegSel == 1)
        {

            dataReg_Ping[0] = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]); //ADD_3
            dataReg_Ping[1] = UINT8_TO_16(USB_Msg_in[17], USB_Msg_in[18]); //ADD_2
            dataReg_Ping[2] = UINT8_TO_16(USB_Msg_in[19], USB_Msg_in[20]); //ADD_1
            dataReg_Ping[3] = UINT8_TO_16(USB_Msg_in[21], USB_Msg_in[22]); //ADD_0
            dataReg_Ping[4] = UINT8_TO_16(USB_Msg_in[23], USB_Msg_in[24]);
            dataReg_Ping[5] = UINT8_TO_16(USB_Msg_in[25], USB_Msg_in[26]);
            dataReg_Ping[6] = UINT8_TO_16(USB_Msg_in[27], USB_Msg_in[28]);

            len_DataReg = UINT8_TO_16(USB_Msg_in[29], USB_Msg_in[30]);
            uint8 i = 0;
            for(i = 0; i < len_DataReg; i++)
                dataReg_Ping[7 + i] = UINT8_TO_16(USB_Msg_in[31 + i * 2], USB_Msg_in[32 + i * 2]);
            dataReg_Ping[7 + len_DataReg] = UINT8_TO_16(USB_Msg_in[31 + len_DataReg * 2], USB_Msg_in[32 + len_DataReg * 2]);
            dataReg_Ping[8 + len_DataReg] = UINT8_TO_16(USB_Msg_in[33 + len_DataReg * 2], USB_Msg_in[34 + len_DataReg * 2]);
            dataReg_Ping[9 + len_DataReg] = UINT8_TO_16(USB_Msg_in[35 + len_DataReg * 2], USB_Msg_in[36 + len_DataReg * 2]);
            dataReg_Ping[10 + len_DataReg] = UINT8_TO_16(USB_Msg_in[37 + len_DataReg * 2], USB_Msg_in[38 + len_DataReg * 2]);
            dataReg_Ping[11 + len_DataReg] = UINT8_TO_16(USB_Msg_in[39 + len_DataReg * 2], USB_Msg_in[40 + len_DataReg * 2]);
            dataReg_Ping[12 + len_DataReg] = UINT8_TO_16(USB_Msg_in[41 + len_DataReg * 2], USB_Msg_in[42 + len_DataReg * 2]);
            dataReg_Ping[13 + len_DataReg] = UINT8_TO_16(USB_Msg_in[43 + len_DataReg * 2], USB_Msg_in[44 + len_DataReg * 2]);
            /*
            int i;
            UARTEnable(UART0_BASE);
            for(i = 0; i < 23; i++)
                UARTCharPut(UART0_BASE, dataReg_Ping[i]);
            UARTDisable(UART0_BASE);
            */
            //usbibufPush(&usbCdcInBufferData, dataReg_Pong, 23);
        }

        //set flag to indicate all data have been received
        if(first_write_flag == 1)
        {
            // dataRegSel = dataRegSel ^ 0x01;

            first_write_flag = 0;
            first_complete = 1;
        }
        datain_complete = 1;
        HalLcdWriteString( "COMdata", HAL_LCD_LINE_5 );
    }

    if ((OPERATION == COM_ADD) && (RESULT == SUCCESS))
    {
        //HalLcdWriteString( "COMADD", HAL_LCD_LINE_3 );
        /*
        sm_ADD_3[sm_index] = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]);
        sm_ADD_2[sm_index] = UINT8_TO_16(USB_Msg_in[17], USB_Msg_in[18]);
        sm_ADD_1[sm_index] = UINT8_TO_16(USB_Msg_in[19], USB_Msg_in[20]);
        sm_ADD_0[sm_index] = UINT8_TO_16(USB_Msg_in[21], USB_Msg_in[22]);


        routing_table[sm_index].sm_ADD = BUILD_UINT64_16(sm_ADD_3[sm_index], sm_ADD_2[sm_index],
                                           sm_ADD_1[sm_index], sm_ADD_0[sm_index]);

        uint16 index;
        uint8 add_duplicate_flag = 0;
        for(index = 0; index < sm_index; index ++)
        {
            if(routing_table[sm_index].sm_ADD == routing_table[index].sm_ADD)
                add_duplicate_flag = 1;
        }

        zclCoordinator_LcdDisplayTestMode_smaddr();
        if(!add_duplicate_flag)
        {
            sm_index++; //increment index
            sm_max++; //total number of smart meter detected
        }
        */
        zclCoordinator_LcdDisplayTestMode_smaddr();
        com_add_rcv_flag = 1;
    }

    if ((OPERATION == COM_CAL) && (RESULT == SUCCESS)) //cal register read ... later
    {
        calsm_id = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]);
        calSMADD_3 = UINT8_TO_16(USB_Msg_in[17], USB_Msg_in[18]);
        calSMADD_2 = UINT8_TO_16(USB_Msg_in[19], USB_Msg_in[20]);
        calSMADD_1 = UINT8_TO_16(USB_Msg_in[21], USB_Msg_in[22]);
        calSMADD_0 = UINT8_TO_16(USB_Msg_in[23], USB_Msg_in[24]);
        SM_CONFIG_2 = UINT8_TO_16(USB_Msg_in[25], USB_Msg_in[26]);
        SM_CONFIG_1 = UINT8_TO_16(USB_Msg_in[27], USB_Msg_in[28]);
        SM_CONFIG_0 = UINT8_TO_16(USB_Msg_in[29], USB_Msg_in[30]);
        calMAG[0] = UINT8_TO_16(USB_Msg_in[31], USB_Msg_in[32]);
        calMAG[1] = UINT8_TO_16(USB_Msg_in[33], USB_Msg_in[34]);
        calMAG[2] = UINT8_TO_16(USB_Msg_in[35], USB_Msg_in[36]);
        calMAG[3] = UINT8_TO_16(USB_Msg_in[37], USB_Msg_in[38]);
        calMAG[4] = UINT8_TO_16(USB_Msg_in[39], USB_Msg_in[40]);
        calMAG[5] = UINT8_TO_16(USB_Msg_in[41], USB_Msg_in[42]);
        calMAG[6] = UINT8_TO_16(USB_Msg_in[43], USB_Msg_in[44]);
        calMAG[7] = UINT8_TO_16(USB_Msg_in[45], USB_Msg_in[46]);
        calT_EFF = UINT8_TO_16(USB_Msg_in[47], USB_Msg_in[48]);

        flag_calget = 1;
        // for test
#ifdef LCD_SUPPORTED
        //HalLcdWriteString( "Get Calpara", HAL_LCD_LINE_4 );
#endif
    }

    if ((OPERATION == RESET) && (RESULT == SUCCESS))
    {
        SmartMeter_flagreset = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]);
        ENERGY_RESET_VALUE_1 = UINT8_TO_16(USB_Msg_in[17], USB_Msg_in[18]);
        ENERGY_RESET_VALUE_0 = UINT8_TO_16(USB_Msg_in[19], USB_Msg_in[20]);
        SmartMeter_ENERGY_RESET_VALUE = BUILD_UINT32_16(ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0);

        // for test
#ifdef LCD_SUPPORTED
        char sflagreset[16];
        char sENERGY_RESET_VALUE[32];
        // char sENERGY_RESET_VALUE_1[16];
        _ltoa(  SmartMeter_flagreset , (void *)(&sflagreset[0]), 16 );
        _ltoa(  SmartMeter_ENERGY_RESET_VALUE , (void *)(&sENERGY_RESET_VALUE[0]), 16 );
        // _ltoa(  ENERGY_RESET_VALUE_1 , (void *)(&sENERGY_RESET_VALUE_1[0]), 16 );
        //HalLcdWriteString( "SendResetBack SUCCESS", HAL_LCD_LINE_3 );
        //HalLcdWriteString( (char *)sflagreset, HAL_LCD_LINE_4 );
        //HalLcdWriteString( (char *)sENERGY_RESET_VALUE, HAL_LCD_LINE_5 );
        // //HalLcdWriteString( (char *)sENERGY_RESET_VALUE_1, HAL_LCD_LINE_6 );
#endif
    }

    if ((OPERATION == RELAY) && (RESULT == SUCCESS))
    {
        SmartMeter_relay = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]);
        relay_receive_flag = 1;
        //flagrelay = 2;
    }

    if ((OPERATION == AUTHEN) && (RESULT == SUCCESS))
    {
        flag_auth = true;
        //send_rom_back(&end_romread[0]);
        if (routing_table[sm_ROM_index].sm_rom_table_hi == BUILD_UINT64_8(USB_Msg_in[15], USB_Msg_in[16], USB_Msg_in[17], USB_Msg_in[18], USB_Msg_in[19], USB_Msg_in[20], USB_Msg_in[21], USB_Msg_in[22]) &&
                routing_table[sm_ROM_index].sm_rom_table_lo == BUILD_UINT64_8(USB_Msg_in[23], USB_Msg_in[24], USB_Msg_in[25], USB_Msg_in[26], USB_Msg_in[27], USB_Msg_in[28], USB_Msg_in[29], USB_Msg_in[30]))
        {
            uint8 end_key[16] = {0};
            end_key[0] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_hi) >> 56) & 0x00000000000000FF);
            end_key[1] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_hi) >> 48) & 0x00000000000000FF);
            end_key[2] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_hi) >> 40) & 0x00000000000000FF);
            end_key[3] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_hi) >> 32) & 0x00000000000000FF);
            end_key[4] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_hi) >> 24) & 0x00000000000000FF);
            end_key[5] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_hi) >> 16) & 0x00000000000000FF);
            end_key[6] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_hi) >> 8) & 0x00000000000000FF);
            end_key[7] = (uint8)((routing_table[sm_ROM_index].sm_key_table_hi) & 0x00000000000000FF);

            end_key[8] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_lo) >> 56) & 0x00000000000000FF);
            end_key[9] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_lo) >> 48) & 0x00000000000000FF);
            end_key[10] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_lo) >> 40) & 0x00000000000000FF);
            end_key[11] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_lo) >> 32) & 0x00000000000000FF);
            end_key[12] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_lo) >> 24) & 0x00000000000000FF);
            end_key[13] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_lo) >> 16) & 0x00000000000000FF);
            end_key[14] = (uint8)(((routing_table[sm_ROM_index].sm_key_table_lo) >> 8) & 0x00000000000000FF);
            end_key[15] = (uint8)((routing_table[sm_ROM_index].sm_key_table_lo) & 0x00000000000000FF);


            routing_table[sm_ROM_index].flag_end_encry = false;
            flag_key_set = false;
            uint32 wait_timenew = osal_GetSystemClock();
            while (osal_GetSystemClock() - wait_timenew < 60)
            {
            };

            zclCoordinator_SetKey(end_key);
        }
    }

    if ((OPERATION == SET_KEY) && (RESULT == SUCCESS))
    {
        flag_key_set = true;
        routing_table[sm_ROM_index].flag_end_encry = true;
    }

    if ((OPERATION == CALIBRATE) && (RESULT == SUCCESS))
    {
        HalLcdWriteString( "cali", HAL_LCD_LINE_7 );
        cal_receive_flag = 1;
    }

    if ((OPERATION == TEMP_STOP) && (RESULT == SUCCESS))
    {
        //HalLcdWriteString( "start2", HAL_LCD_LINE_5 );

        uint32 wait_timenew = osal_GetSystemClock();
        /*
                while (osal_GetSystemClock() - wait_timenew < 30)
                {
                };
        */
        SmartMeter_flaginc = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]);
        zclCoordinator_SendData();   //send request to get data
    }

    if ((OPERATION == START) && (RESULT == SUCCESS))
    {
        SmartMeter_start = UINT8_TO_16(USB_Msg_in[15], USB_Msg_in[16]);

        //if(SmartMeter_start == start)
        start_receive_flag = 1;
    }

    if ((OPERATION == COM_CONFIG) && (RESULT == SUCCESS))
    {
        flag_config_reg = 1;
    }
}



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
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[70] = {0};
        //uint16 coor_index = 0xffff;
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
        //coor_index = zclCoordinator_recognise_coor_id();

        uint16 packet[] = {USR_RX_SET, SET_PARAM, MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN,
                           MIN_V, MAX_V, MIN_I, MAX_I,
                           SHARP1, SHARP2, PEAK1, PEAK2, PEAK3,
                           SHOULDER1, SHOULDER2, SHOULDER3,
                           OFF, N_SM, sm_id
                          };

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = controlReg[i + 7];
        }

        //uint16 smart_meter_index = 0xffff;
        //smart_meter_index = zclCoordinator_recognise_sm_id();
        //send_buffer[1] = (uint8)(smart_meter_index & 0x00ff);
        //for(i = 2; i < 9; i++)
        //{
        //    send_buffer[i] = 0xee;
        //}
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
    }
}


/*********************************************************************
 * @fn      zclCoordinator_getcalParam *
 *
 * @brief   Called to getcalculation related parameter from the SmartMeter
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_getcalParam( void )
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;

        uint16 packet[] = {USR_RX_GET, COM_CAL};

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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;
        //coor_index = zclCoordinator_recognise_coor_id();
        uint16 packet[] = {USR_RX_GET, COM_CAL};

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = controlReg[i + 7];
        }
        /*
        uint16 smart_meter_index = 0xffff;
        smart_meter_index = zclCoordinator_recognise_sm_id();
        send_buffer[1] = (uint8)(smart_meter_index & 0x00ff);
        for(i = 2; i < 9; i++)
        {
            send_buffer[i] = 0xee;
        }
        */
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
    }
}

/*********************************************************************
 * @fn      zclCoordinator_setconfigReg *
 *
 * @brief   Called to set configuration register to paticular smart meter
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_setconfigReg( void )
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;

        uint16 packet[] = {COM_CONFIG, SM_CONFIG_2, SM_CONFIG_1, SM_CONFIG_0};

        pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

        if ( pReportCmd != NULL )
        {
            pReportCmd->numAttr = 1;
            pReportCmd->attrList[0].attrID = ATTRID_MS_PARAMETER_MEASURED_VALUE;
            pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT64;
            pReportCmd->attrList[0].attrData = (void *)(packet);

            zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                               ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                               pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
        }

        osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
    }
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;
        //coor_index = zclCoordinator_recognise_coor_id();
        uint16 packet[] = {COM_CONFIG, SM_CONFIG_2, SM_CONFIG_1, SM_CONFIG_0};

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = controlReg[i + 7];
        }
        /*
        uint16 smart_meter_index = 0xffff;
        smart_meter_index = zclCoordinator_recognise_sm_id();
        send_buffer[1] = (uint8)(smart_meter_index & 0x00ff);
        for(i = 2; i < 9; i++)
        {
            send_buffer[i] = 0xee;
        }
        */
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
    }
}

/*******************************************************************
 * @fn      zclCoordinator_SetTime *
 *

 * @brief   Called to set SmartMeter Time information from the Coordinator
 *
 * @param   none
 *
 * @return  none
 */

static void zclCoordinator_SetTime( void ) //verified
{
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[50] = {0};
        //uint16 coor_index = 0xffff;
        YEAR = timeReg[0];
        MONTH = timeReg[1];
        DAY = timeReg[2];
        HOUR = timeReg[3];
        MINUTE = timeReg[4];
        SECOND = timeReg[5];

        uint16 packet[] = {TIME_SET, YEAR, MONTH, DAY, HOUR,
                           MINUTE, SECOND, sm_id
                          };

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = controlReg[i + 7];
        }
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
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

    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;
        uint16 packet[] = {USR_RX_GET, COM_DATA};

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = sm_address_buffer[i - 1];
            sm_address_buffer[i - 1] = 0x00;
        }


        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);

        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
        //HalUART0Write ( HAL_UART_PORT_0, send_buffer, 13 + sizeof(packet));

        char  lcdString[10];
        //sprintf((char *)lcdString, "%x %x %x %x", send_buffer[5], send_buffer[6], send_buffer[7], send_buffer[8] );
        //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;
        uint16 energy_reset_value_1 = 0;
        uint16 energy_reset_value_0 = 0;

        energy_reset_value_1 = (uint16)((ENERGY_RESET_VALUE >> 16) & 0xFFFF);
        energy_reset_value_0 = (uint16)(ENERGY_RESET_VALUE & 0xFFFF);
        uint16 packet[] = {RESET, flagreset, energy_reset_value_1, energy_reset_value_0};


        send_buffer[0] = 0x68;
        if(controlReg[3] != 0x08 && controlReg[8] != 0xFF)
        {
            for(i = 1; i < 9; i++)
            {
                send_buffer[i] = controlReg[i + 7];
            }
        }
        else
        {
            if(write_energy_index < sm_max)
            {
                send_buffer[1] = (uint8)(((routing_table[write_energy_index].sm_ADD) >> 56) & 0x00000000000000FF);
                send_buffer[2] = (uint8)(((routing_table[write_energy_index].sm_ADD) >> 48) & 0x00000000000000FF);
                send_buffer[3] = (uint8)(((routing_table[write_energy_index].sm_ADD) >> 40) & 0x00000000000000FF);
                send_buffer[4] = (uint8)(((routing_table[write_energy_index].sm_ADD) >> 32) & 0x00000000000000FF);
                send_buffer[5] = (uint8)(((routing_table[write_energy_index].sm_ADD) >> 24) & 0x00000000000000FF);
                send_buffer[6] = (uint8)(((routing_table[write_energy_index].sm_ADD) >> 16) & 0x00000000000000FF);
                send_buffer[7] = (uint8)(((routing_table[write_energy_index].sm_ADD) >> 8) & 0x00000000000000FF);
                send_buffer[8] = (uint8)((routing_table[write_energy_index].sm_ADD) & 0x00000000000000FF);
            }
        }

        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);

        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
    }
}

/*********************************************************************
 * @fn      zclCoordinator_SendAuth *
 *
 * @brief   Called to send command to send authentication in smart meter
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SendAuth( void )
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {USR_RX_GET, AUTHEN};

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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;

        uint16 packet[] = {USR_RX_GET, AUTHEN}; //sizeof(packet) = 4

        send_buffer[0] = 0x68;
        if(controlReg[3] != 0x08 && controlReg[8] != 0xFF)
        {
            for(i = 1; i < 9; i++)
            {
                send_buffer[i] = controlReg[i + 7];
            }
        }
        else
        {
            if(routing_index < sm_max)
            {
                send_buffer[1] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 56) & 0x00000000000000FF);
                send_buffer[2] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 48) & 0x00000000000000FF);
                send_buffer[3] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 40) & 0x00000000000000FF);
                send_buffer[4] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 32) & 0x00000000000000FF);
                send_buffer[5] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 24) & 0x00000000000000FF);
                send_buffer[6] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 16) & 0x00000000000000FF);
                send_buffer[7] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 8) & 0x00000000000000FF);
                send_buffer[8] = (uint8)((routing_table[smart_auth_index].sm_ADD) & 0x00000000000000FF);
            }
        }

        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);
        //HalUART0Write ( HAL_UART_PORT_0, send_buffer, 13 + sizeof(packet));
    }
}

/*********************************************************************
 * @fn      zclCoordinator_SetKey *
 *
 * @brief   Called to send command to set key in smart meter
 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SetKey( uint8 *parameter )
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {USR_RX_SET, SET_KEY, UINT8_TO_16(parameter[0], parameter[1]), UINT8_TO_16(parameter[2], parameter[3]), UINT8_TO_16(parameter[4], parameter[5]), UINT8_TO_16(parameter[6], parameter[7]),
                           UINT8_TO_16(parameter[8], parameter[9]), UINT8_TO_16(parameter[10], parameter[11]), UINT8_TO_16(parameter[12], parameter[13]), UINT8_TO_16(parameter[14], parameter[15])
                          };

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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;

        uint16 packet[] = {USR_RX_SET, SET_KEY, UINT8_TO_16(parameter[0], parameter[1]), UINT8_TO_16(parameter[2], parameter[3]), UINT8_TO_16(parameter[4], parameter[5]), UINT8_TO_16(parameter[6], parameter[7]),
                           UINT8_TO_16(parameter[8], parameter[9]), UINT8_TO_16(parameter[10], parameter[11]), UINT8_TO_16(parameter[12], parameter[13]), UINT8_TO_16(parameter[14], parameter[15])
                          };

        send_buffer[0] = 0x68;
        if(controlReg[3] != 0x08 && controlReg[8] != 0xFF)
        {
            for(i = 1; i < 9; i++)
            {
                send_buffer[i] = controlReg[i + 7];
            }
        }
        else
        {
            if(routing_index < sm_max)
            {
                send_buffer[1] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 56) & 0x00000000000000FF);
                send_buffer[2] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 48) & 0x00000000000000FF);
                send_buffer[3] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 40) & 0x00000000000000FF);
                send_buffer[4] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 32) & 0x00000000000000FF);
                send_buffer[5] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 24) & 0x00000000000000FF);
                send_buffer[6] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 16) & 0x00000000000000FF);
                send_buffer[7] = (uint8)(((routing_table[smart_auth_index].sm_ADD) >> 8) & 0x00000000000000FF);
                send_buffer[8] = (uint8)((routing_table[smart_auth_index].sm_ADD) & 0x00000000000000FF);
            }
        }
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {RELAY, flagrelay};

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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;
        uint16 packet[] = {RELAY, flagrelay}; //sizeof(packet) = 4

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = controlReg[i + 7];
        }
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);

        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
        //HalLcdWriteString( "datasend", HAL_LCD_LINE_4 );

        /*
              uint16 Switch_INTERVAL =15 * 1000; //generate a random delay from 0 to 16000000 SysTIck
              // add random delay function here
              unsigned long Switch_ulValue_start;
              // NVIC_ST_CURRENT register
              // must be written to force the reload. Any write to this register clears the SysTick counter to 0
              // and causes a reload with the supplied period on the next clock.
              // add code to write to  NVIC_ST_CURRENT register
              // Configure and enable the SysTick counter.
              SysTickPeriodSet(100000); //16000000
              SysTickEnable();
              // Read the current SysTick value.
              Switch_ulValue_start = SysTickValueGet();
              while (SysTickValueGet() - Switch_ulValue_start < Switch_INTERVAL)
              {
              };

              GPIOPinWrite(GPIO_A_BASE, GPIO_PIN_7, 0x00);
              */
    }
}

/*********************************************************************
 * @fn      zclCoordinator_SetPowerCalculation *

 *
 * @brief   Called to send command to set relay in smart meter

 *
 * @param   none
 *
 * @return  none
 */
static void zclCoordinator_SetPowerCalculation( void )
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {START, start};

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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;
        uint16 packet[] = {START, start}; //sizeof(packet) = 4

        send_buffer[0] = 0x68;

        if(controlReg[9] != 0xFF && controlReg[9] != 0x00)
        {
            for(i = 1; i < 9; i++)
            {
                send_buffer[i] = controlReg[i + 7];
            }
        }
        else
        {
            /*
              if(setpower_index < sm_max)
              {
                  send_buffer[1] = (uint8)(((routing_table[setpower_index].sm_ADD) >> 56) & 0x00000000000000FF);
                  send_buffer[2] = (uint8)(((routing_table[setpower_index].sm_ADD) >> 48) & 0x00000000000000FF);
                  send_buffer[3] = (uint8)(((routing_table[setpower_index].sm_ADD) >> 40) & 0x00000000000000FF);
                  send_buffer[4] = (uint8)(((routing_table[setpower_index].sm_ADD) >> 32) & 0x00000000000000FF);
                  send_buffer[5] = (uint8)(((routing_table[setpower_index].sm_ADD) >> 24) & 0x00000000000000FF);
                  send_buffer[6] = (uint8)(((routing_table[setpower_index].sm_ADD) >> 16) & 0x00000000000000FF);
                  send_buffer[7] = (uint8)(((routing_table[setpower_index].sm_ADD) >> 8) & 0x00000000000000FF);
                  send_buffer[8] = (uint8)((routing_table[setpower_index].sm_ADD) & 0x00000000000000FF);
              }*/
            for(i = 1; i < 9; i++)
            {
                send_buffer[i] = 0xFF;
            }
        }

        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);

        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
        //HalLcdWriteString( "datasend", HAL_LCD_LINE_4 );

    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        V_CAL = controlReg[28];
        I_CAL = controlReg[29];
        T_CAL = controlReg[30];
        N_CAL = controlReg[31];

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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[40] = {0};
        //uint16 coor_index = 0xffff;
        V_CAL = controlReg[28];
        I_CAL = controlReg[29];
        T_CAL = controlReg[30];
        N_CAL = controlReg[31];
        INPUT_1_CAL = controlReg[32];
        INPUT_2_CAL = controlReg[33];

        uint16 packet[] = {CALIBRATE, V_CAL, I_CAL, T_CAL, N_CAL, INPUT_1_CAL, INPUT_2_CAL};

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = controlReg[i + 7];
        }
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);

        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
        //HalUART0Write ( HAL_UART_PORT_0, send_buffer, 13 + sizeof(packet));
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;

        //uint16 packet[] = {START, flaginc};
        uint16 packet[] = {TEMP_STOP, flaginc};
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;

        //uint16 packet[] = {START, flaginc};
        uint16 packet[] = {TEMP_STOP, flaginc};
        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = zclCoordinator_DstAddr.addr.extAddr[8 - i];
            sm_address_buffer[i - 1] = send_buffer[i];
        }
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);


        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
        //HalUART0Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
    }
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

    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        afAddrType_t Bc_DstAddr;
        zclReportCmd_t *pReportCmd;

        sm_index = 0;
        sm_max = 0;
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;

        uint16 packet[] = {USR_RX_GET, COM_ADD, ADD_3, ADD_2, ADD_1, ADD_0};

        send_buffer[0] = 0x68;

        if(controlReg[3] != 0x08 && controlReg[8] != 0xFF)
        {
            for(i = 1; i < 9; i++)
            {
                send_buffer[i] = controlReg[i + 7];
            }
        }
        else
        {
            if(routing_index < sm_max)
            {
                send_buffer[1] = (uint8)(((routing_table[routing_index].sm_ADD) >> 56) & 0x00000000000000FF);
                send_buffer[2] = (uint8)(((routing_table[routing_index].sm_ADD) >> 48) & 0x00000000000000FF);
                send_buffer[3] = (uint8)(((routing_table[routing_index].sm_ADD) >> 40) & 0x00000000000000FF);
                send_buffer[4] = (uint8)(((routing_table[routing_index].sm_ADD) >> 32) & 0x00000000000000FF);
                send_buffer[5] = (uint8)(((routing_table[routing_index].sm_ADD) >> 24) & 0x00000000000000FF);
                send_buffer[6] = (uint8)(((routing_table[routing_index].sm_ADD) >> 16) & 0x00000000000000FF);
                send_buffer[7] = (uint8)(((routing_table[routing_index].sm_ADD) >> 8) & 0x00000000000000FF);
                send_buffer[8] = (uint8)((routing_table[routing_index].sm_ADD) & 0x00000000000000FF);
                //HalUART0Write ( HAL_UART_PORT_0, send_buffer, 9);
            }
        }
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);

        send_buffer[13 + sizeof(packet)] = routing_index;
        send_buffer[14 + sizeof(packet)] = sm_max;
        //HalUART0Write ( HAL_UART_PORT_0, send_buffer, 15 + sizeof(packet));
    }

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
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {USR_RX_GET, ACK_SUCCESS, (uint16)(((routing_table[sm_index].sm_ADD) >> 48) & 0x000000000000FFFF), (uint16)(((routing_table[sm_index].sm_ADD) >> 32) & 0x000000000000FFFF),
                           (uint16)(((routing_table[sm_index].sm_ADD) >> 16) & 0x000000000000FFFF), (uint16)((routing_table[sm_index].sm_ADD) & 0x000000000000FFFF)
                          };

        pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );

        if ( pReportCmd != NULL )
        {
            pReportCmd->numAttr = 1;
            pReportCmd->attrList[0].attrID = ATTRID_MS_ACK_MEASURED_VALUE;
            pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT256; //zcl.c
            pReportCmd->attrList[0].attrData = (void *)(packet);

            zcl_SendReportCmd( Coordinator_ENDPOINT, &zclCoordinator_DstAddr,
                               ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                               pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclCoordinatorSeqNum++ );
            // for test
#ifdef LCD_SUPPORTED
            //HalLcdWriteString( "SendAck", HAL_LCD_LINE_3 );
#endif
        }

        osal_mem_free( pReportCmd );
    }
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint8 i = 0;
        //uint8 send_buffer[30] = {0};
        //uint16 coor_index = 0xffff;

        uint16 packet[] = {COM_ADD, ACK_SUCCESS, (uint16)(((routing_table[sm_index].sm_ADD) >> 48) & 0x000000000000FFFF), (uint16)(((routing_table[sm_index].sm_ADD) >> 32) & 0x000000000000FFFF),
                           (uint16)(((routing_table[sm_index].sm_ADD) >> 16) & 0x000000000000FFFF), (uint16)((routing_table[sm_index].sm_ADD) & 0x000000000000FFFF)
                          };

        send_buffer[0] = 0x68;
        for(i = 1; i < 9; i++)
        {
            send_buffer[i] = controlReg[i + 7];
        }
        send_buffer[9] = 0x68;
        send_buffer[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            send_buffer[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            send_buffer[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        send_buffer[11 + sizeof(packet)] = 0x00;
        for(i = 0; i < 11 + sizeof(packet); i++)
            send_buffer[11 + sizeof(packet)] += send_buffer[i];

        send_buffer[12 + sizeof(packet)] = 0x16;
        len_uart1 = 13 + sizeof(packet);

        //HalUART1Write ( HAL_UART_PORT_1, send_buffer, 13 + sizeof(packet));
    }
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
static void zclCoordinator_parameterInit(void)                                     //0xinitial
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
    first_complete = 1;
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

    for(uint8 i = 0 ; i < 45; i++)
        controlReg[i] = 0;

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
    for (i = 0; i < 29; i++)
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
    //osal_start_timerEx( zclCoordinator_TaskID, TIME_RUNNING_EVT, 1 );
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
             HalLcdWriteString ( pStr, HAL_LCD_LINE_3 );
            if ( giThermostatScreenMode == THERMOSTAT_MAINMODE )
            {
                HalLcdWriteString ( pStr, HAL_LCD_LINE_3 );
            }
        }
#endif  // LCD_SUPPORTED
        if( Connect_Mode == WIRELESS_CONNECTION)
        {
            zclCoordinator_NetDiscov();
        }
        uint8 i;
        for(i = 0; i < NUM_SMART_METER; i++)
            routing_table[i].sm_ADD_status = 1;


    }
}
#endif // ZCL_EZMODE

//sent a smart meter IEEE address to local server
// smart meter IEEE address is store in  sm_ADD_i[]
//according to the index coordinator can find the corresponding smart meter IEEE address
static void zclCoordinator_ReadRoutingTable(uint8 sm_i)
{
    int i = 0;
    uint8 pack_out[59] = {0};
    pack_out[0] = 0x68;
    for(i = 1; i <= 8; i++)
        pack_out[i] = coor_addrRegister[i - 1];
    pack_out[9] = 0x68;
    pack_out[10] = 0x2E;
    pack_out[11] = (uint8)(((routing_table[sm_i].sm_ADD) >> 56) & 0x00000000000000FF);
    pack_out[12] = (uint8)(((routing_table[sm_i].sm_ADD) >> 48) & 0x00000000000000FF);
    pack_out[13] = (uint8)(((routing_table[sm_i].sm_ADD) >> 40) & 0x00000000000000FF);
    pack_out[14] = (uint8)(((routing_table[sm_i].sm_ADD) >> 32) & 0x00000000000000FF);
    pack_out[15] = (uint8)(((routing_table[sm_i].sm_ADD) >> 24) & 0x00000000000000FF);
    pack_out[16] = (uint8)(((routing_table[sm_i].sm_ADD) >> 16) & 0x00000000000000FF);
    pack_out[17] = (uint8)(((routing_table[sm_i].sm_ADD) >> 8) & 0x00000000000000FF);
    pack_out[18] = (uint8)((routing_table[sm_i].sm_ADD) & 0x00000000000000FF);
    pack_out[19] = routing_table[sm_i].sm_ADD_status;
    pack_out[20] = routing_table[sm_i].freq;

    pack_out[21] = (uint8)(((routing_table[sm_i].sm_rom_table_hi) >> 56) & 0x00000000000000FF);
    pack_out[22] = (uint8)(((routing_table[sm_i].sm_rom_table_hi) >> 48) & 0x00000000000000FF);
    pack_out[23] = (uint8)(((routing_table[sm_i].sm_rom_table_hi) >> 40) & 0x00000000000000FF);
    pack_out[24] = (uint8)(((routing_table[sm_i].sm_rom_table_hi) >> 32) & 0x00000000000000FF);
    pack_out[25] = (uint8)(((routing_table[sm_i].sm_rom_table_hi) >> 24) & 0x00000000000000FF);
    pack_out[26] = (uint8)(((routing_table[sm_i].sm_rom_table_hi) >> 16) & 0x00000000000000FF);
    pack_out[27] = (uint8)(((routing_table[sm_i].sm_rom_table_hi) >> 8) & 0x00000000000000FF);
    pack_out[28] = (uint8)((routing_table[sm_i].sm_rom_table_hi) & 0x00000000000000FF);
    pack_out[29] = (uint8)(((routing_table[sm_i].sm_rom_table_lo) >> 56) & 0x00000000000000FF);
    pack_out[30] = (uint8)(((routing_table[sm_i].sm_rom_table_lo) >> 48) & 0x00000000000000FF);
    pack_out[31] = (uint8)(((routing_table[sm_i].sm_rom_table_lo) >> 40) & 0x00000000000000FF);
    pack_out[32] = (uint8)(((routing_table[sm_i].sm_rom_table_lo) >> 32) & 0x00000000000000FF);
    pack_out[33] = (uint8)(((routing_table[sm_i].sm_rom_table_lo) >> 24) & 0x00000000000000FF);
    pack_out[34] = (uint8)(((routing_table[sm_i].sm_rom_table_lo) >> 16) & 0x00000000000000FF);
    pack_out[35] = (uint8)(((routing_table[sm_i].sm_rom_table_lo) >> 8) & 0x00000000000000FF);
    pack_out[36] = (uint8)((routing_table[sm_i].sm_rom_table_lo) & 0x00000000000000FF);

    pack_out[37] = (uint8)(((routing_table[sm_i].sm_key_table_hi) >> 56) & 0x00000000000000FF);
    pack_out[38] = (uint8)(((routing_table[sm_i].sm_key_table_hi) >> 48) & 0x00000000000000FF);
    pack_out[39] = (uint8)(((routing_table[sm_i].sm_key_table_hi) >> 40) & 0x00000000000000FF);
    pack_out[40] = (uint8)(((routing_table[sm_i].sm_key_table_hi) >> 32) & 0x00000000000000FF);
    pack_out[41] = (uint8)(((routing_table[sm_i].sm_key_table_hi) >> 24) & 0x00000000000000FF);
    pack_out[42] = (uint8)(((routing_table[sm_i].sm_key_table_hi) >> 16) & 0x00000000000000FF);
    pack_out[43] = (uint8)(((routing_table[sm_i].sm_key_table_hi) >> 8) & 0x00000000000000FF);
    pack_out[44] = (uint8)((routing_table[sm_i].sm_key_table_hi) & 0x00000000000000FF);
    pack_out[45] = (uint8)(((routing_table[sm_i].sm_key_table_lo) >> 56) & 0x00000000000000FF);
    pack_out[46] = (uint8)(((routing_table[sm_i].sm_key_table_lo) >> 48) & 0x00000000000000FF);
    pack_out[47] = (uint8)(((routing_table[sm_i].sm_key_table_lo) >> 40) & 0x00000000000000FF);
    pack_out[48] = (uint8)(((routing_table[sm_i].sm_key_table_lo) >> 32) & 0x00000000000000FF);
    pack_out[49] = (uint8)(((routing_table[sm_i].sm_key_table_lo) >> 24) & 0x00000000000000FF);
    pack_out[50] = (uint8)(((routing_table[sm_i].sm_key_table_lo) >> 16) & 0x00000000000000FF);
    pack_out[51] = (uint8)(((routing_table[sm_i].sm_key_table_lo) >> 8) & 0x00000000000000FF);
    pack_out[52] = (uint8)((routing_table[sm_i].sm_key_table_lo) & 0x00000000000000FF);


    pack_out[53] = controlReg[0];
    pack_out[54] = controlReg[1];
    pack_out[55] = controlReg[2];
    pack_out[56] = controlReg[3];
    for(i = 0; i < 57; i++)
        pack_out[57] += pack_out[i];
    pack_out[58] = 0x16;
    uint8 len_data = pack_out[10];
    if(flag_encry)
    {
        uint8 encry_data[70] = {0};

        for (uint8 i = 0; i < 70; i++)
            encry_data[i] = 0xFF;
        for (uint8 i = 0; i < len_data; i++)
            encry_data[i] = pack_out[i + 11];

        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

        for(uint8 i = 0; i < (len_data / 16) ; i++)
            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

        pack_out[10] = len_data;
        for (uint8 i = 0; i < len_data; i++)
            pack_out[i + 11] = encry_data[i];

        pack_out[len_data + 11] = 0x00;
        for (uint8 i = 0; i < len_data + 11; i++ )
            pack_out[len_data + 11] +=  pack_out[i];
        pack_out[len_data + 12] = 0x16;

    }
    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);


    //test
    /*
    uint8 j, l, k = 0;
    uint8 num_low_prio = 0;
    num_low_prio = sm_max - num_high_prio;

    for(uint8 i = 0; i < 3 * NUM_SMART_METER; i++)
        sm_routing_prio_table[i] = 0;

    for(i = 0; i < num_low_prio; i++) // number of low priority
    {
        for(j = 0; j < routing_table[i].freq / num_low_prio; j++)
        {
            for(l = 0; l < num_high_prio; l++)
            {
                sm_routing_prio_table[k++] = l;
            }
        }
        sm_routing_prio_table[k++] = i + num_high_prio;
    }
    num_prio_sm_max = k;
    sm_routing_prio_table[48] = (uint8)sm_max;
    sm_routing_prio_table[49] = (uint8)num_prio_sm_max;
    HalUART0Write ( HAL_UART_PORT_0, sm_routing_prio_table, 50);
    */
}

/*
static uint8 zclCoordinator_smIEEE_to_id(uint64 sm_ADD_64)
{

    if(sm_ADD_64 == 0x00124B00040F1A3C)
        return 0x00;
    else if(sm_ADD_64 == 0x00124B00040F1C77)
        return 0x01;
    else if(sm_ADD_64 == 0x00124B00040EF19E)
        return 0x02;
    else if(sm_ADD_64 == 0x00124B000413318E)
        return 0x03;
    else if(sm_ADD_64 == 0x00124B000413328A)
        return 0x04;
    else if(sm_ADD_64 == 0x00124B00040EF391)
        return 0x05;
    else if(sm_ADD_64 == 0x00124B00041331BF)
        return 0x06;
    else if(sm_ADD_64 == 0x00124B0004133698)
        return 0x07;

    else
        return 0xff;
}

static void zclCoordinator_id_to_smIEEE(uint8 sm_id_8)
{

    if(sm_id_8 == 0x00)
        sm_ADD_prio =  0x00124B00040F1A3C;
    else if(sm_id_8 == 0x01)
        sm_ADD_prio = 0x00124B00040F1C77;
    else if(sm_id_8 == 0x02)
        sm_ADD_prio = 0x00124B00040EF19E;
    else if(sm_id_8 == 0x03)
        sm_ADD_prio = 0x00124B000413318E;
    else if(sm_id_8 == 0x04)
        sm_ADD_prio = 0x00124B000413328A;
    else if(sm_id_8 == 0x05)
        sm_ADD_prio = 0x00124B00040EF391;
    else if(sm_id_8 == 0x06)
        sm_ADD_prio = 0x00124B00041331BF;
    else if(sm_id_8 == 0x07)
        sm_ADD_prio = 0x00124B0004133698;


    else
        sm_ADD_prio = 0xffffffffffffffff;
}
*/
//sent calibration register package to local server
static void zclCoordinator_sendcalreg(void)
{
    uint8 i = 0;
    uint8 pack_out[50] = {0};
    pack_out[0] = 0x68;
    for(i = 1; i <= 8; i++)
        pack_out[i] = coor_addrRegister[i - 1];
    pack_out[9] = 0x68;

    pack_out[10] = 0x24;
    pack_out[11] = (uint8)((calSMADD_3 & 0xff00) >> 8);
    pack_out[12] = (uint8)(calSMADD_3 & 0x00ff);
    pack_out[13] = (uint8)((calSMADD_2 & 0xff00) >> 8);
    pack_out[14] = (uint8)(calSMADD_2 & 0x00ff);
    pack_out[15] = (uint8)((calSMADD_1 & 0xff00) >> 8);
    pack_out[16] = (uint8)(calSMADD_1 & 0x00ff);
    pack_out[17] = (uint8)((calSMADD_0 & 0xff00) >> 8);
    pack_out[18] = (uint8)(calSMADD_0 & 0x00ff);

    pack_out[19] = (uint8)((SM_CONFIG_2 & 0xff00) >> 8);
    pack_out[20] = (uint8)(SM_CONFIG_2 & 0x00ff);
    pack_out[21] = (uint8)((SM_CONFIG_1 & 0xff00) >> 8);
    pack_out[22] = (uint8)(SM_CONFIG_1 & 0x00ff);
    pack_out[23] = (uint8)((SM_CONFIG_0 & 0xff00) >> 8);
    pack_out[24] = (uint8)(SM_CONFIG_0 & 0x00ff);

    for(i = 0; i < 8; i++)
    {
        pack_out[25 + i * 2] = (uint8)((calMAG[i] & 0xff00) >> 8);
        pack_out[26 + i * 2] = (uint8)(calMAG[i] & 0x00ff);
    }
    pack_out[41] = (uint8)((calT_EFF & 0xff00) >> 8);
    pack_out[42] = (uint8)(calT_EFF & 0x00ff);
    controlReg[0] = 0x08;
    controlReg[1] = 0x03;
    controlReg[2] = 0x00;
    controlReg[3] = 0x00;

    pack_out[43] = controlReg[0];
    pack_out[44] = controlReg[1];
    pack_out[45] = controlReg[2];
    pack_out[46] = controlReg[3];

    for(i = 0; i < 47; i++)
        pack_out[47] += pack_out[i];
    pack_out[48] = 0x16;
    /*
    UARTEnable(UART0_BASE );
    for(i = 0; i < 40; i++)
        UARTCharPut (UART0_BASE, pack_out[i]);
    UARTDisable(UART0_BASE );
    */
    uint8 len_data = pack_out[10];
    if(flag_encry)
    {
        uint8 encry_data[70] = {0};

        for (uint8 i = 0; i < 70; i++)
            encry_data[i] = 0xFF;
        for (uint8 i = 0; i < len_data; i++)
            encry_data[i] = pack_out[i + 11];

        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

        for(uint8 i = 0; i < (len_data / 16) ; i++)
            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

        pack_out[10] = len_data;
        for (uint8 i = 0; i < len_data; i++)
            pack_out[i + 11] = encry_data[i];

        pack_out[len_data + 11] = 0x00;
        for (uint8 i = 0; i < len_data + 11; i++ )
            pack_out[len_data + 11] +=  pack_out[i];
        pack_out[len_data + 12] = 0x16;

    }
    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);
    //usbibufPush(&usbCdcInBufferData, pack_out, 49);
}

//sent calibration timeout response to local server
static void zclCoordinator_calregtimeout()
{
    int i = 0;
    uint8 pack_out[65] = {0};
    pack_out[0] = 0x68;

    for(i = 1; i <= 8; i++)
        pack_out[i] = coor_addrRegister[i - 1];
    pack_out[9] = 0x68;
    pack_out[10] = 0x24;

    for(i = 11; i < 46; i++)
        pack_out[i] = 0;

    controlReg[0] = 0x08;
    controlReg[1] = 0x03;
    controlReg[2] = 0x00;
    controlReg[3] = 0x02;

    pack_out[43] = controlReg[0];
    pack_out[44] = controlReg[1];
    pack_out[45] = controlReg[2];
    pack_out[46] = controlReg[3];

    for(i = 0; i < 47; i++)
        pack_out[47] += pack_out[i];
    pack_out[48] = 0x16;
    /*
    UARTEnable(UART0_BASE );
    for(i = 0; i < 40; i++)
        UARTCharPut (UART0_BASE, pack_out[i]);
    UARTDisable(UART0_BASE );
    */
    uint8 len_data = pack_out[10];
    if(flag_encry)
    {
        uint8 encry_data[70] = {0};

        for (uint8 i = 0; i < 70; i++)
            encry_data[i] = 0xFF;
        for (uint8 i = 0; i < len_data; i++)
            encry_data[i] = pack_out[i + 11];

        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

        for(uint8 i = 0; i < (len_data / 16) ; i++)
            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

        pack_out[10] = len_data;
        for (uint8 i = 0; i < len_data; i++)
            pack_out[i + 11] = encry_data[i];

        pack_out[len_data + 11] = 0x00;
        for (uint8 i = 0; i < len_data + 11; i++ )
            pack_out[len_data + 11] +=  pack_out[i];
        pack_out[len_data + 12] = 0x16;

    }
    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);
    //usbibufPush(&usbCdcInBufferData, pack_out, 49);
}

/********************************************************
//send ACK to local server when command is done successfully
void zclCoordinator_ACK(void)
**********************************************************/
void zclCoordinator_sendACK(uint8 ControlReg0, uint8 ControlReg1, uint8 ControlReg2, uint8 ControlReg3)
{
    uint8 i;
    uint8 pack_out[20] = {0};
    pack_out[0] = 0x68;

    for(i = 1; i <= 8; i++)
        pack_out[i] = coor_addrRegister[i - 1];

    pack_out[9] = 0x68;
    pack_out[10] = 0x04;

    controlReg[0] = ControlReg0;
    controlReg[1] = ControlReg1;
    controlReg[2] = ControlReg2;
    controlReg[3] = ControlReg3;

    pack_out[11] = ControlReg0;
    pack_out[12] = ControlReg1;
    pack_out[13] = ControlReg2;
    pack_out[14] = ControlReg3;

    for(i = 0; i < 15; i++)
        pack_out[15] += pack_out[i];

    pack_out[16] = 0x16;
    /*
    UARTEnable(UART0_BASE );
    for (i = 0; i < 16; i++)
    {
        UARTCharPut(UART0_BASE, pack_out[i]);
    }
    UARTDisable(UART0_BASE);
    */
    uint8 len_data = pack_out[10];
    if(flag_encry)
    {
        uint8 encry_data[70] = {0};

        for (uint8 i = 0; i < 70; i++)
            encry_data[i] = 0xFF;
        for (uint8 i = 0; i < len_data; i++)
            encry_data[i] = pack_out[i + 11];

        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

        for(uint8 i = 0; i < (len_data / 16) ; i++)
            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

        pack_out[10] = len_data;
        for (uint8 i = 0; i < len_data; i++)
            pack_out[i + 11] = encry_data[i];

        pack_out[len_data + 11] = 0x00;
        for (uint8 i = 0; i < len_data + 11; i++ )
            pack_out[len_data + 11] +=  pack_out[i];
        pack_out[len_data + 12] = 0x16;

    }
    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);
    //usbibufPush(&usbCdcInBufferData, pack_out, 17);
}

//*****************************************************************************
//
// AesEncryptDecrypt
//
// param   pui8Key            Pointer to buffer containing the key
// param   ui8KeyLocation     location of Key in the Key RAM. Must be one of
//                            the following:
//                            KEY_AREA_0
//                            KEY_AREA_1
//                            KEY_AREA_2
//                            KEY_AREA_3
//                            KEY_AREA_4
//                            KEY_AREA_5
//                            KEY_AREA_6
//                            KEY_AREA_7
// param   pui8Buf            pointer to input Buffer
// param   ui8Encrypt is set 'true' to ui8Encrypt or set 'false' to decrypt.
// return  AES_SUCCESS if successful
//
//*****************************************************************************
uint8_t AesEncryptDecrypt(uint8_t *pui8Key,
                          uint8_t *pui8Buf,
                          uint8_t ui8KeyLocation,
                          uint8_t ui8Encrypt)
{

    //
    // example using polling
    //

    AESLoadKey((uint8_t *)pui8Key, ui8KeyLocation);
    AESECBStart(pui8Buf, pui8Buf, ui8KeyLocation, ui8Encrypt, false);

    //
    // wait for completion of the operation
    //
    do
    {
        ASM_NOP;
    }
    while(!(AESECBCheckResult()));

    AESECBGetResult();

    return (AES_SUCCESS);
}


static void send_rom_back(uint8 *reg_ro)
{
    uint8 i;
    uint8 pack_out[33] = {0};
    pack_out[0] = 0x68;

    for(i = 1; i <= 8; i++)
        pack_out[i] = coor_addrRegister[i - 1];

    pack_out[9] = 0x68;
    pack_out[10] = 0x14;

    for (i = 0; i < 16; i++)
    {
        pack_out[11 + i] = *(reg_ro + i);
    }

    controlReg[0] = 0x08;
    controlReg[1] = 0x03;
    controlReg[2] = 0x00;
    controlReg[3] = 0x00;
    pack_out[27] = 0x08;
    pack_out[28] = 0x03;
    pack_out[29] = 0x00;
    pack_out[30] = 0x00;

    for(i = 0; i < 31; i++)
        pack_out[31] += pack_out[i];

    pack_out[32] = 0x16;

    uint8 len_data = pack_out[10];
    if(flag_encry)
    {
        uint8 encry_data[70] = {0};

        for (uint8 i = 0; i < 70; i++)
            encry_data[i] = 0xFF;
        for (uint8 i = 0; i < len_data; i++)
            encry_data[i] = pack_out[i + 11];

        len_data = (len_data % 16) ? ((len_data / 16 + 1) * 16) : len_data;

        for(uint8 i = 0; i < (len_data / 16) ; i++)
            AesEncryptDecrypt(coor_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);

        pack_out[10] = len_data;
        for (uint8 i = 0; i < len_data; i++)
            pack_out[i + 11] = encry_data[i];

        pack_out[len_data + 11] = 0x00;
        for (uint8 i = 0; i < len_data + 11; i++ )
            pack_out[len_data + 11] +=  pack_out[i];
        pack_out[len_data + 12] = 0x16;

    }
    usbibufPush(&usbCdcInBufferData, pack_out, len_data + 13);

    //usbibufPush(&usbCdcInBufferData, pack_out, 33);
}

void find_end_in_routing_table(uint8 buffer[])
{
    uint64 IEEE_id = BUILD_UINT64_8(buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8]);

    for (end_index = 0; end_index < sm_max; end_index++)
    {
        if (IEEE_id == routing_table[end_index].sm_ADD)
            break;
    }
    if(end_index < sm_max)
    {
        end_final_key[0] = (uint8)(((routing_table[end_index].final_key_table_hi) >> 56) & 0x00000000000000FF);
        end_final_key[1] = (uint8)(((routing_table[end_index].final_key_table_hi) >> 48) & 0x00000000000000FF);
        end_final_key[2] = (uint8)(((routing_table[end_index].final_key_table_hi) >> 40) & 0x00000000000000FF);
        end_final_key[3] = (uint8)(((routing_table[end_index].final_key_table_hi) >> 32) & 0x00000000000000FF);
        end_final_key[4] = (uint8)(((routing_table[end_index].final_key_table_hi) >> 24) & 0x00000000000000FF);
        end_final_key[5] = (uint8)(((routing_table[end_index].final_key_table_hi) >> 16) & 0x00000000000000FF);
        end_final_key[6] = (uint8)(((routing_table[end_index].final_key_table_hi) >> 8) & 0x00000000000000FF);
        end_final_key[7] = (uint8)((routing_table[end_index].final_key_table_hi) & 0x00000000000000FF);

        end_final_key[8] = (uint8)(((routing_table[end_index].final_key_table_lo) >> 56) & 0x00000000000000FF);
        end_final_key[9] = (uint8)(((routing_table[end_index].final_key_table_lo) >> 48) & 0x00000000000000FF);
        end_final_key[10] = (uint8)(((routing_table[end_index].final_key_table_lo) >> 40) & 0x00000000000000FF);
        end_final_key[11] = (uint8)(((routing_table[end_index].final_key_table_lo) >> 32) & 0x00000000000000FF);
        end_final_key[12] = (uint8)(((routing_table[end_index].final_key_table_lo) >> 24) & 0x00000000000000FF);
        end_final_key[13] = (uint8)(((routing_table[end_index].final_key_table_lo) >> 16) & 0x00000000000000FF);
        end_final_key[14] = (uint8)(((routing_table[end_index].final_key_table_lo) >> 8) & 0x00000000000000FF);
        end_final_key[15] = (uint8)((routing_table[end_index].final_key_table_lo) & 0x00000000000000FF);
    }
}


void route_table_add(r_table *routing_table, r_table routing_single)
{
    int i = 0;
    if (sm_max > 0)
    {

        while(routing_single.freq < routing_table[i].freq && i < sm_max)
        {
            i++;
        }
        int k = 0;
        for(k = sm_max - 1; k > i - 1; k--)
        {
            routing_table[k + 1].sm_ADD = routing_table[k].sm_ADD;
            routing_table[k + 1].freq = routing_table[k].freq;
            routing_table[k + 1].sm_ADD_status = routing_table[k].sm_ADD_status;
            routing_table[k + 1].sm_key_table_hi = routing_table[k].sm_key_table_hi;
            routing_table[k + 1].sm_key_table_lo = routing_table[k].sm_key_table_lo;
            routing_table[k + 1].sm_rom_table_hi = routing_table[k].sm_rom_table_hi;
            routing_table[k + 1].sm_rom_table_lo = routing_table[k].sm_rom_table_lo;
            routing_table[k + 1].final_key_table_hi = routing_table[k].final_key_table_hi;
            routing_table[k + 1].final_key_table_lo = routing_table[k].final_key_table_lo;
            routing_table[k + 1].flag_end_encry = routing_table[k].flag_end_encry;

        }
    }
    routing_table[i].sm_ADD = routing_single.sm_ADD;
    routing_table[i].freq = routing_single.freq;
    routing_table[i].sm_ADD_status = routing_single.sm_ADD_status;
    routing_table[i].sm_key_table_hi = routing_single.sm_key_table_hi;
    routing_table[i].sm_key_table_lo = routing_single.sm_key_table_lo;
    routing_table[i].sm_rom_table_hi = routing_single.sm_rom_table_hi;
    routing_table[i].sm_rom_table_lo = routing_single.sm_rom_table_lo;
    routing_table[i].final_key_table_hi = routing_single.final_key_table_hi;
    routing_table[i].final_key_table_lo = routing_single.final_key_table_lo;
    routing_table[i].flag_end_encry = routing_single.flag_end_encry;
    sm_max++;
}

void route_table_delete(r_table *routing_table, r_table routing_single)
{

    if(sm_max > 0)
    {
        int i = 0;
        while(routing_single.sm_ADD != routing_table[i].sm_ADD && i < sm_max)
        {
            i++;
        }
        int k = 0;
        for(k = i; k < sm_max - 1; k++)
        {
            routing_table[k].sm_ADD = routing_table[k + 1].sm_ADD;
            routing_table[k].freq = routing_table[k + 1].freq;
            routing_table[k].sm_ADD_status = routing_table[k + 1].sm_ADD_status;
            routing_table[k].sm_key_table_hi = routing_table[k + 1].sm_key_table_hi;
            routing_table[k].sm_key_table_lo = routing_table[k + 1].sm_key_table_lo;
            routing_table[k].sm_rom_table_hi = routing_table[k + 1].sm_rom_table_hi;
            routing_table[k].sm_rom_table_lo = routing_table[k + 1].sm_rom_table_lo;
            routing_table[k].final_key_table_hi = routing_table[k + 1].final_key_table_hi;
            routing_table[k].final_key_table_lo = routing_table[k + 1].final_key_table_lo;
            routing_table[k].flag_end_encry = routing_table[k + 1].flag_end_encry;
        }
        sm_max--;
    }
}