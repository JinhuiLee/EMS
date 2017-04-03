/**************************************************************************************************
  Filename:       zcl_SmartMeter.c
  Revised:        $Date: 2014-10-17 11:49:27 -0700 (Fri, 18 Oct 2013) $
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
#include "aes.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_ms.h"
#include "zcl_smartmeter.h"
#include "onboard.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
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
#include "MT.h"
#include "MT_UART.h"
#include "rom.h"
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
#define ACK_SUCCESS 0xCB
#define CALIBRATE   0xCC
#define CAL_VOL     0xCD
#define CAL_CUR     0xCE
#define CAL_ENE     0xCF
#define TIME_SET    0xD0
#define COM_CAL     0xD1
#define COM_CONFIG   0xD2
#define CAL_GEN_1    0xD3
#define CAL_GEN_2    0xD4
#define TEMP_STOP    0xD5
#define AUTHEN       0xD6
#define SET_KEY      0xD7

#define FLASH_PARAM  0x1000
#define FLASH_PARAMCAL  0x1100
#define FLASH_CONFIG 0x1078
#define FLASH_RO     0x1200
#define FFT_N 16
//*****************************************************************
#define TYPE_VOL_DELTA 			0x00
#define TYPE_VOL_WYLE 			0x01
#define TYPE_CUR 			0x02
#define TYPE_GEN_INPUT1 		0x03
#define TYPE_GEN_INPUT2 		0x04
#define UNUSET_CNL                      0x05
#define SUM_SQUR_LIMIT                  3000000000
//******************************************************************


//#define COORDINATORADDRESS 0x01                 // which coordinator to communicate
//*****************************************************************************
/*
#define EXAMPLE_PIN_UART0_RXD            GPIO_PIN_0
#define EXAMPLE_PIN_UART0_TXD            GPIO_PIN_1
#define EXAMPLE_GPIO_BASE0               GPIO_A_BASE

#define EXAMPLE_PIN_UART1_RXD            GPIO_PIN_2  //RF1.2
#define EXAMPLE_PIN_UART1_TXD            GPIO_PIN_3  //RF1.4
#define EXAMPLE_GPIO_BASE1               GPIO_C_BASE
*/
//*****************************************************************************
// how often to sample ADC in millisecond
//#define SmartMeter_PowerCal_INTERVAL   56
#define SmartMeter_PowerCal_INTERVAL   1
#define SmartMeter_Calibration_INTERVAL 1
#define UINT8_TO_16(hiByte, loByte) \
          ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

//#define Connect_Mode WIRED_CONNECTION
#define Connect_Mode WIRELESS_CONNECTION
//#define TIMEINDEX 0.1
#define TIMEINDEX 1            
//#defien TIMEINDEX 3.125

#define PI 3.1416
typedef struct
{
    double real;
    double img;
} complex;
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
uint16 flagrelay = 1;
uint16 start = 0;    //used for start or stop the power calculation, added by xu.
uint8 flaginc = 1;
uint8 phase_dec_flag[8] = {0};
uint32 phase_dec_V1 = 0;
//uint32 phase_dec_V2 = 0;
unsigned long phase_dec_V2 = 0;
unsigned long ulValue_old = 0;
unsigned long ulValue_new = 0;
uint32 phase_dec_V3 = 0;

//SM_CONFIG register
uint8 SM_CONFIG_5 = 0x00;
uint8 SM_CONFIG_4 = 0x00;
uint8 SM_CONFIG_3 = 0x00;
uint8 SM_CONFIG_2 = 0x00;
uint8 SM_CONFIG_1 = 0x00;
uint8 SM_CONFIG_0 = 0x00;
uint16 ConfigReg[3] = {0};
///////////////////////////////////////////////////////////////new variables
uint8 ch_info[8] = {0};
uint8 ch_Tag[8] = {0};
uint16 senValue[8] = {0};
uint8 len_DataReg = 0;
uint8 Num_phase[5] = {0};

int16 REAL_V1 = 0;           //can be negative
int16 reg_REAL_V1 = 0;
int16 phase_REAL_V1 = 0;
uint32 rmsTemp_V1 = 0;
int32 totalTemp_V1 = 0;
uint8 overflow_num_V1 = 0;

int16 REAL_V2 = 0;           //can be negative
int16 reg_REAL_V2 = 0;
int16 phase_REAL_V2 = 0;
uint32 rmsTemp_V2 = 0;
int32 totalTemp_V2 = 0;
uint8 overflow_num_V2 = 0;

int16 REAL_V3 = 0;           //can be negative
int16 reg_REAL_V3 = 0;
int16 phase_REAL_V3 = 0;
uint32 rmsTemp_V3 = 0;
int32 totalTemp_V3 = 0;
uint8 overflow_num_V3 = 0;

int16 REAL_I1[8] = {0};
int16 reg_REAL_I1[8] = {0};
uint32 rmsTemp_I1[8] = {0};
int32 totalTemp_I1[8] = {0};
uint8 overflow_num_I1[8] = {0};

int16 REAL_I2[8] = {0};
int16 reg_REAL_I2[8] = {0};
uint32 rmsTemp_I2[8] = {0};
int32 totalTemp_I2[8] = {0};
uint8 overflow_num_I2[8] = {0};

int16 REAL_I3[8] = {0};
int16 reg_REAL_I3[8] = {0};
uint32 rmsTemp_I3[8] = {0};
int32 totalTemp_I3[8] = {0};
uint8 overflow_num_I3[8] = {0};

int16 REAL_VD = 0;
uint32 rmsTemp_VD = 0;
uint8 overflow_num_VD = 0;

int16 REAL_ID = 0;
uint32 rmsTemp_ID = 0;
uint8 overflow_num_ID = 0;

uint32 powerVal[8] = {0};
uint64 energyVal[8] = {0};
uint32 enecal_energy[8] = {0};
//double energy_display[8] = {0};
double Energy[8];
float CurDisplay[8] = {0};
float PowerDisplay[8] = {0};

int16 Theta1[8] = {0xffff};
int16 Theta2[8] = {0xffff};
int16 Theta3[8] = {0xffff};
uint16 VIT_dataReg[16] = {0};
//-----------------------------------------------------
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
uint16 STATUS = 0;
uint16 run_T_EFF;
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

uint16 V_CAL = 0;
uint16 I_CAL = 0;
uint16 T_CAL = 0;
uint16 N_CAL = 0;
uint16 INPUT_1_CAL = 0;
uint16 INPUT_2_CAL = 0;
uint8 CAL_OPT = 0;
//uint32 E_CAL = 0;

uint16 senValueV[3] = {0};
uint16 senValueI[3] = {0};
int16 realVol[3] = {0};        //can be negative
int16 realCur[3] = {0};        //can be negative

uint16 REAL_GEN_INPUT1[8] = {0};
uint32 accu_GEN_INPUT1[8] = {0};

uint16 REAL_GEN_INPUT2[8] = {0};
uint32 accu_GEN_INPUT2[8] = {0};

int16_t value;
uint16_t u_value;
uint8_t u_partA;
uint8_t u_partB;

uint32 energyVal_Lcd_display;
uint16 RMS_V1;
uint16 RMS_V2;
uint16 RMS_V3;
uint16 RMS_I1[8];
uint16 RMS_I2[8];
uint16 RMS_I3[8];
uint16 THETA_1;
uint16 THETA_2;
uint16 THETA_3;


uint8 power_flag = 1;

int32 VrmsTemp[3] = {0};
int32 IrmsTemp[3] = {0};
int32 powerTemp[3] = {0};
uint32 l_nSamples = 0;
uint16 CRMS_V[8] = {0};
uint16 CRMS_I[8] = {0};
uint32 rmsTemp_calV[8] = {0};
uint32 rmsTemp_calI[8] = {0};
int16 totalTemp_calV[8] = {0};
int16 totalTemp_calI[8] = {0};
uint16 MAG_V[8] = {100};
uint16 MAG_I[8] = {100};
uint16 MAG_GEN_INPUT1[8] = {100};
uint16 MAG_GEN_INPUT2[8] = {100};
int16 Cal_Vreal[8] = {0};
int16 Cal_Ireal[8] = {0};


uint8 cal_index = 0;


uint16 test_i = 1;
uint16 status;
char Enstring[25];    //parameter for display floating number

uint64 sm_ADD; //smart meter external IEEE address
uint16 sm_nwkADD;  //smart meter network address
uint8 *psm_ADD; //pointer to smart meter external IEEE address
uint64 coordinator_extAddr; // Coordinator extended IEEE address
uint16 paramReg[18] = {0};
uint16 calReg[40] = {0};

uint16 dataReg[30];
uint16 coordinator_Addr_3 = 0x0000;
uint16 coordinator_Addr_2 = 0x0000;
uint16 coordinator_Addr_1 = 0x0000;
uint16 coordinator_Addr_0 = 0x0000;
uint16 ADD_3;
uint16 ADD_2;
uint16 ADD_1;
uint16 ADD_0;
uint16 SM_ADD16 = 0;
uint16 smADD[4] = {0};

uint32 time_old = 0;
uint32 time_new = 0;
uint8 dataLen;  //length of the packdge
uint8 Msg_in[120] = {0}; //for save UART DATA
uint8 pack_out[120] = {0}; //for send out UART data to coordinator
extern mtOSALSerialData_t  *pMsg;

uint8 romread[16] = {0x00};
//variables related to FFT
complex x[30], W[16];
uint16 result_PHASE[3] = {0};
int16 VOI_sample[8*FFT_N] = {0};
int16 V_sample[16] = {0};

uint32 switch_timenew = 0;
uint8 first_time_in = 1;
uint8 second_time_in = 0;
uint8 len_uart1 = 0;

uint8_t end_final_key[16] = {0};
bool flag_end_encry = false;
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
// NOT ZCL_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
      ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    //data attribute for SmartMeter defined in zcl_SmartMeter_data_c
    ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,  // added for SmartMeter
    ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,  // added for SmartMeter
    ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,  // added for SmartMeter
    ZCL_CLUSTER_ID_MS_COM_MEASUREMENT  // added for SmartMeter
};
#define ZCLSMARTMETER_BINDINGLIST        6
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


// FUNCTIONS related to FFT
static void fft();
static void initW();
static void change();
static void add(complex , complex , complex *);
static void mul(complex , complex , complex *);
static void sub(complex , complex , complex *);
static void divi(complex , complex , complex *);

static uint8_t AesEncryptDecrypt(uint8_t *pui8Key, uint8_t *pui8Buf, uint8_t ui8KeyLocation, uint8_t ui8Encrypt);
// app SmartMeter functions
static void zclSmartMeter_SendParam(void);
static void zclSmartMeter_SendTime(void);
static void zclSmartMeter_SendData(void);
static void zclSmartMeter_SendReset(void);
static void zclSmartMeter_SendRelay(void);
static void zclSmartMeter_SendAdd(void);
static void zclSmartMeter_SendStart(void);

static void zclSmartMeter_SendCalibrate(void);
static void zclSmartMeter_SendCalPara(void);
static void zclSmartMeter_SendRestart(void);
static void zclSmartMeter_SendKeyAck(void);

static void zclSmartMeter_nvReadParam(void);
static void zclSmartMeter_nvWriteParam(void);

static void zclSmartMeter_WriteConfigReg(void);
static void zclSmartMeter_ReadConfigReg(void);
static void zclSmartMeter_SendConfigAck(void);
static void initUART(void);
static uint8 recognise_sm_id(void);

int16_t uint16ToInt16(uint16_t u_i);
uint16_t int16ToUint16(int16_t i);
void Configuration_Reg_Process();
void zclSmartMeter_sendACK(uint8 ControlReg0, uint8 ControlReg1, uint8 ControlReg2); //send ACK to coordinator
void ProcessUartData( mtOSALSerialData_t *Uart_Msg);
void zclSmartMeter_ProcessUART_Pkt(void);
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
    //GPIOPinTypeGPIOOutput(GPIO_A_BASE, GPIO_PIN_7);
    //GPIOPinWrite(GPIO_A_BASE, GPIO_PIN_7, 0x00);

    //GPIOPinTypeGPIOOutput(GPIO_D_BASE, GPIO_PIN_0);
    //GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_0, 0x00);
    
    /////////////////////////////////////////////////////////////////////
    //GPIOPinTypeGPIOOutput(GPIO_D_BASE, GPIO_PIN_1);//RF2.12    
    //GPIOPinWrite(GPIO_D_BASE, GPIO_PIN_1, 0x00); //old RSE
    
    
    GPIOPinTypeGPIOOutput(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1));  
    
    ////////////////////////////////////////////////////////////////////////////
    //PC0 is in cut-off state, PC1 is in on state
    //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01);  //LED2=0, LED1=1
    GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x01);  
    GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_1, 0x00);  //new RSE pin
    
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
    //HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_5 );
#endif
    //if( Connect_Mode == WIRELESS_CONNECTION)
       // zclsmartmeter_startEZmode();
    // Initialize SmartMeter parameters
    zclSmartMeter_parameterInit();
    //Initialize SmartMeter data register
    zclSmartMeter_dataRegInit();
    //Initialize ADC
    zclSmartMeter_ADC_init();
    // Get smart meter 64-bit IEEE external address and network address
    psm_ADD = saveExtAddr;
    sm_nwkADD = NLME_GetShortAddr();
    smADD[0] = (uint16)saveExtAddr[6] + ((((uint16)saveExtAddr[7]) << 8) & 0xFF00);
    smADD[1] = (uint16)saveExtAddr[4] + ((((uint16)saveExtAddr[5]) << 8) & 0xFF00);
    smADD[2] = (uint16)saveExtAddr[2] + ((((uint16)saveExtAddr[3]) << 8) & 0xFF00);
    smADD[3] = (uint16)saveExtAddr[0] + ((((uint16)saveExtAddr[1]) << 8) & 0xFF00);
    sm_ADD = ((uint64)smADD[3] & 0xFFFF) + ((((uint64)smADD[2]) << 16) & 0xFFFF0000) + ((((uint64)smADD[1]) << 32) & 0xFFFF00000000) + ((((uint64)smADD[0]) << 48) & 0xFFFF000000000000);
    ADD_3 = smADD[0];
    ADD_2 = smADD[1];
    ADD_1 = smADD[2];
    ADD_0 = smADD[3];
    
    //zclSmartMeter_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    //zclSmartMeter_DstAddr.addr.shortAddr = 0x0000;
    
    // Set up the serial console to use for displaying messages.  This is
    // just for debugging purpose and is not needed for Systick operation.
    InitConsole();
    //MT Uart Initial
    MT_UartInit();
    MT_UartRegisterTaskID(zclSmartMeter_TaskID);
    initUART();
    
    //
    // Enable AES peripheral
    //
    SysCtrlPeripheralReset(SYS_CTRL_PERIPH_AES);
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_AES);
    enecal_timeold = osal_GetSystemClock();
    enecal_timenew = enecal_timeold;
    Configuration_Reg_Process();
    osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, 1 );

}


//Initiation of configuration register
void Configuration_Reg_Process()  //once
{
    //uint8 Num_phase[4] = {0};//phase0: DC phase1 phase2 phase3
    ch_info[7] = (uint8)(SM_CONFIG_3 & 0xf0) >> 4;
    ch_info[6] = (uint8)(SM_CONFIG_3 & 0x0f);
    ch_info[5] = (uint8)(SM_CONFIG_2 & 0xf0) >> 4;
    ch_info[4] = (uint8)(SM_CONFIG_2 & 0x0f);
    ch_info[3] = (uint8)(SM_CONFIG_1 & 0xf0) >> 4;
    ch_info[2] = (uint8)(SM_CONFIG_1 & 0x0f);
    ch_info[1] = (uint8)(SM_CONFIG_0 & 0xf0) >> 4;
    ch_info[0] = (uint8)(SM_CONFIG_0 & 0x0f);

    uint8 i = 0;
    len_DataReg = 0;
    Num_phase[0] = 0;
    Num_phase[1] = 0;
    Num_phase[2] = 0;
    Num_phase[3] = 0;
    Num_phase[4] = 0;

    for(i = 0; i < 8; i++)
    {
        switch(ch_info[i])
        {
        case 0x00:
            ch_Tag[i] = UNUSET_CNL;
            break;
        case 0x01:
            len_DataReg++;
            if(SM_CONFIG_4 == 0x01)
                ch_Tag[i] = TYPE_VOL_DELTA;
            else if( SM_CONFIG_4 == 0x00 )
                ch_Tag[i] = TYPE_VOL_WYLE;
            break;
        case 0x02:
            len_DataReg++;
            if(SM_CONFIG_4 == 0x01)
                ch_Tag[i] = TYPE_VOL_DELTA;
            else if( SM_CONFIG_4 == 0x00 )
                ch_Tag[i] = TYPE_VOL_WYLE;
            break;
        case 0x03:
            len_DataReg++;
            if(SM_CONFIG_4 == 0x01)
                ch_Tag[i] = TYPE_VOL_DELTA;
            else if( SM_CONFIG_4 == 0x00 )
                ch_Tag[i] = TYPE_VOL_WYLE;
            break;
        case 0x09:
            ch_Tag[i] = TYPE_CUR;
            Num_phase[1] ++;
            len_DataReg += 2;
            break;
        case 0x0A:
            ch_Tag[i] = TYPE_CUR;
            Num_phase[2] ++;
            len_DataReg += 2;
            break;
        case 0x0B:
            ch_Tag[i] = TYPE_CUR;
            Num_phase[3] ++;
            len_DataReg += 2;
            break;
        case 0x04:
            ch_Tag[i] = TYPE_GEN_INPUT1;
            Num_phase[0] ++;
            len_DataReg++;
            break;
        case 0x0C:
            ch_Tag[i] = TYPE_GEN_INPUT2;
            Num_phase[4] ++;
            len_DataReg++;
            break;
        default:
            ch_Tag[i] = UNUSET_CNL;
            break;
        }
    }
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


void bindToZC(){
  zclSmartMeter_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;

  zclSmartMeter_DstAddr.addr.shortAddr = NLME_GetCoordShortAddr();

  zclSmartMeter_DstAddr.endPoint = SmartMeter_ENDPOINT;
  
//  dstAddr.addrMode = Addr16Bit;
//  dstAddr.addr.shortAddr = 0; 
//  //byte address[] = {0x00,0x12,0x4B,0x00,0x06,0x3A,0x4D,0x19};
//  //for (int i = 0 ; i < 8 ; i++)
//  //    dstAddr.addr.extAddr[i] = address[i]; 
//  // Coordinator makes the EDB match
//
//  // Initiate an End Device Bind Request, this bind request will
//  // only use a cluster list that is important to binding.
//  ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
//                                SmartMeter_ENDPOINT,
//                                ZCL_HA_PROFILE_ID,
//                                0, NULL,
//                                ZCLSMARTMETER_BINDINGLIST, bindingOutClusters,
//                                FALSE );
//  
  HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
  
//  ZDP_MatchDescReq(&dstAddr, NLME_GetShortAddr(),
//                        ZCL_HA_PROFILE_ID,
//                        0, NULL,
//                        ZCLSMARTMETER_BINDINGLIST, bindingOutClusters,
//                        FALSE );
  
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

    /*
        #ifdef LCD_SUPPORTED
            sprintf((char *)lcdString, "test_i: %d", (uint8)(test_i & 0xFF) );
            HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
            test_i++;
        #endif
    */


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
                  showPANID();
                  bindToZC();
#ifndef HOLD_AUTO_START
                    giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
                    //zclSmartMeter_LCDDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
                    zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif // ZCL_EZMODE
                }
                break;
            case CMD_SERIAL_MSG:                                           //ZcomDef.h 10.8
                //HalLcdWriteString( "datain1", HAL_LCD_LINE_4 );
                ProcessUartData((mtOSALSerialData_t *)pMsg);
                zclSmartMeter_ProcessUART_Pkt();

                //osal_set_event(task_id, SmartMeter_UART_EVT);
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
            //osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, SmartMeter_Calibration_INTERVAL );
            osal_set_event(task_id, SmartMeter_ADC_SEND_EVT);
        else
            // get ADC reading every 56 millisecond
            //osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, SmartMeter_PowerCal_INTERVAL );
            osal_set_event(task_id, SmartMeter_ADC_SEND_EVT);
        sys_timenew = osal_GetSystemClock();
        sys_secnew = sys_secold + (uint32)((float)((sys_timenew - sys_timeold) * TIMEINDEX) / 1000);
        //sys_timeold = sys_timenew;
        //sys_secold = sys_secnew;

        uint8 old_sec_print = TimeStruct.seconds;
        osal_ConvertUTCTime(&TimeStruct , sys_secnew);


        if(old_sec_print != TimeStruct.seconds)
        {
#ifdef LCD_SUPPORTED
            sprintf((char *)lcdString, "%d %d %d %d %d %d", 
            ((TimeStruct.month == 12)? (TimeStruct.year + 1) : TimeStruct.year), 
            ((TimeStruct.month == 12)? 1: (TimeStruct.month + 1)),
            TimeStruct.day + 1, TimeStruct.hour, TimeStruct.minutes, TimeStruct.seconds);
            HalLcdWriteString( lcdString, HAL_LCD_LINE_7 );
#endif

            //MT_UartInit();
            //MT_UartRegisterTaskID(zclSmartMeter_TaskID);
            //initUART();
            /*
            uint8 show_tick[4] = {0};
            ulValue_new = SysTickValueGet();
            show_tick[0] = (uint8)(((ulValue_new - ulValue_old) >> 24) & 0x000000FF);
            show_tick[1] = (uint8)(((ulValue_new - ulValue_old) >> 16) & 0x000000FF);
            show_tick[2] = (uint8)(((ulValue_new - ulValue_old) >> 8) & 0x000000FF);
            show_tick[3] = (uint8)(((ulValue_new - ulValue_old)) & 0x000000FF);
            sprintf((char *)lcdString, "%d %d %d %d ", show_tick[0], show_tick[1],
                    show_tick[2], show_tick[3]);
            HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );

            ulValue_old = ulValue_new;
            */

        }
        
        ////////////////////////////////////wait
        char  lcdString[10];
        if(len_uart1) //package is assembled
        {
           if(first_time_in) //ori 1
           {
              first_time_in = 0;
              switch_timenew = osal_GetSystemClock();
              GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_1, 0x02);
              //HalLcdWriteString( "uartstep1", HAL_LCD_LINE_3 );
           }
           else if (!second_time_in)
           {   
              /*
              uint8_t ui8AESKey[16] =  { 0x00, 0x12, 0x4b, 0x00, 0x04, 0x0f, 0x98, 0x00,
        0x00, 0x12, 0x4b, 0x00, 0x04, 0x0f, 0x98, 0x00 };
              
              uint8 len_data = len_uart1 - 13;
        
              for(uint8 i = 0; i < ((len_data%16)? (len_data / 16 + 1) : (len_data / 16)) ; i++)
                  AesEncryptDecrypt(ui8AESKey, pack_out+ 11 + 16 * i, 0, ENCRYPT_AES);
             
              pack_out[len_uart1 - 2] = 0;
              for (uint8 i = 0; i < len_uart1 - 2; i++ )
                  pack_out[len_uart1 - 2] +=  pack_out[i];
              */
              if (osal_GetSystemClock() - switch_timenew >= 37 )
              {
                  

                  uint8 len_data = len_uart1 - 13;
                      
                  
                  
                  if (flag_end_encry)
                  {
                      uint8 encry_data[70] = {0};
                      
                      for (uint8 i = 0; i < len_data; i++)
                          encry_data[i] = pack_out[i + 11];
                      
                      // change data length to make encryption package stay the same
                      len_data = (len_data%16) ? ((len_data / 16 + 1) * 16) : len_data;
                      
                      //HalUART0Write ( HAL_UART_PORT_0, encry_data, len_data);
                      for(uint8 i = 0; i < (len_data / 16) ; i++)
                          AesEncryptDecrypt(end_final_key, encry_data + 16 * i, 0, ENCRYPT_AES);
                     
                      //HalUART0Write ( HAL_UART_PORT_0, encry_data, len_data);
                      pack_out[10] = len_data;
                      for (uint8 i = 0; i < len_data; i++)
                          pack_out[i + 11] = encry_data[i];
                      
                      pack_out[len_data + 11] = 0x00;
                      for (uint8 i = 0; i < len_data + 11; i++ )
                          pack_out[len_data + 11] +=  pack_out[i];
                      
                      pack_out[len_data + 12] = 0x16;
                  }
                  
                  if (pack_out[12] == SET_KEY)
                      flag_end_encry = true;
                    
                  HalUART1Write ( HAL_UART_PORT_1, pack_out, len_data + 13);  
                  //HalUART0Write ( HAL_UART_PORT_0, pack_out, len_data + 13);    
                  for( uint8 i = 0; i < 120; i++)
                       pack_out[i] = 0;
                  second_time_in = 1;
                  
                  
                  //sprintf((char *)lcdString, "%d", osal_GetSystemClock() - switch_timenew);
                  switch_timenew = osal_GetSystemClock();
                  //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );
              }
           }         
           else if (second_time_in)
           {
              if (osal_GetSystemClock() - switch_timenew >= 52 )
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
        

        ///////////////////////////////////
        
        
        return ( events ^ SmartMeter_ADC_SEND_EVT );
    }





    return 0;
}


static void initUART(void)
{
    SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);

    //
    // Set IO clock to the same as system clock
    //
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
    /*
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
        IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD, IOC_MUX_OUT_SEL_UART0_TXD);
        GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_TXD);
        IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD, IOC_UARTRXD_UART0);
        GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE0, EXAMPLE_PIN_UART0_RXD);

        //IOCPinConfigPeriphOutput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_TXD, IOC_MUX_OUT_SEL_UART1_TXD);
        //GPIOPinTypeUARTOutput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_TXD);
        //IOCPinConfigPeriphInput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_RXD, IOC_UARTRXD_UART1);
        //GPIOPinTypeUARTInput(EXAMPLE_GPIO_BASE1, EXAMPLE_PIN_UART1_RXD);
        //
        // Configure the UART for 115,200, 8-N-1 operation.
        // This function uses SysCtrlClockGet() to get the system clock
        // frequency.  This could be also be a variable or hard coded value
        // instead of a function call.
        //
        UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    */
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
        //Msg_in[index] = 0x00;
        Msg_in[index] = Uart_Msg->msg[index + dataLen + 2];
    Msg_in[9] = 0x68;


    Msg_in[10] = dataLen;


    Msg_in[11] = Uart_Msg->msg[1];

    for (index = 1; index < (int)(dataLen + 2); index++)
    {
        Msg_in[index + 11] = Uart_Msg->msg[index + 1] ;
    }

    HalLcdWriteString( "processUARTDATA", HAL_LCD_LINE_6 );
    //HalUART0Write ( HAL_UART_PORT_0, Msg_in, 13 + dataLen);

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
       // zclSmartMeter_LcdPowerDisplayUpdate();
       // start = 1;
       /*
       uint8_t ui8AESKey[16] =  { 0x00, 0x12, 0x4b, 0x00, 0x04, 0x0f, 0x98, 0x00,
        0x00, 0x12, 0x4b, 0x00, 0x04, 0x0f, 0x98, 0x00 };
       uint8 buffer[37] = { 0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f, 
         0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,    
        0x6c, 0x5c, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x56, 0x53, 0x06, 0x0f, 
        0x41, 0x32, 0x5a, 0x6c, 0xaa };

        char  lcdString[10];
        switch_timenew = osal_GetSystemClock();

        
        AesEncryptDecrypt(ui8AESKey, buffer, 1, ENCRYPT_AES);
        //HalUART0Write ( HAL_UART_PORT_0, buffer, 16);
        AesEncryptDecrypt(ui8AESKey, buffer+16, 1, ENCRYPT_AES);
        //HalUART0Write ( HAL_UART_PORT_0, buffer+16, 16);
        AesEncryptDecrypt(ui8AESKey, buffer+32, 1, ENCRYPT_AES);
        HalUART0Write ( HAL_UART_PORT_0, buffer, 37);
        
        AesEncryptDecrypt(ui8AESKey, buffer, 1, DECRYPT_AES);
        //HalUART0Write ( HAL_UART_PORT_0, buffer, 16);
        AesEncryptDecrypt(ui8AESKey, buffer+16, 1, DECRYPT_AES);
        //HalUART0Write ( HAL_UART_PORT_0, buffer+16, 16);
        AesEncryptDecrypt(ui8AESKey, buffer+32, 1, DECRYPT_AES);
        HalUART0Write ( HAL_UART_PORT_0, buffer, 37);       
       

        sprintf((char *)lcdString, "%d", osal_GetSystemClock() - switch_timenew);
        HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );
        */
        //int status1 = ROM_PageErase(0x0027f800,0x800);
        //uint32_t pulData[4] = {0xffffffff, 0xf0ffffff, 0x00000001, 0x00200000};
        //int status2 = ROM_ProgramFlash(pulData, 0x0027ffd0, 16);
        
        //zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_CONFIG_STATE|ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
        SystemResetSoft();
        //if (status1 == 0 && status2 == 0)
           // ROM_ResetDevice();
    }
    if ( keys & HAL_KEY_SW_2 )
    {
      zclSmartMeter_SendTime();
    }
    if ( keys & HAL_KEY_SW_3 )
    {
      zclsmartmeter_startEZmode();
    }
    if ( keys & HAL_KEY_SW_4 )
    {
      #ifdef LCD_SUPPORTED
        HalLcdWriteString( "Bind", HAL_LCD_LINE_3 );
      #endif
        giTemperatureSensorScreenMode = TEMPSENSE_MAINMODE;
        zAddrType_t dstAddr;
        dstAddr.addrMode = Addr16Bit;
        dstAddr.addr.shortAddr = 0; 
        //byte address[] = {0x00,0x12,0x4B,0x00,0x06,0x3A,0x4D,0x19};
        //for (int i = 0 ; i < 8 ; i++)
        //    dstAddr.addr.extAddr[i] = address[i]; 
        // Coordinator makes the EDB match

        // Initiate an End Device Bind Request, this bind request will
        // only use a cluster list that is important to binding.
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
        ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                              SmartMeter_ENDPOINT,
                              ZCL_HA_PROFILE_ID,
                              0, NULL,
                              ZCLSMARTMETER_BINDINGLIST, bindingOutClusters,
                              FALSE );
        
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
        HalLcdWriteString( "EZMode", HAL_LCD_LINE_3 );
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
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {SET_PARAM, SUCCESS, MIN_ADC, MAX_ADC, SAMPLE_INT, SAMPLE_WIN,
                           MIN_V, MAX_V, MIN_I, MAX_I,
                           SHARP1, SHARP2,
                           PEAK1, PEAK2, PEAK3,
                           SHOULDER1, SHOULDER2, SHOULDER3,
                           OFF, N_SM, sm_index
                          };
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {TIME_SET, SUCCESS, YEAR, MONTH, DAY, HOUR, MINUTE,
                           SECOND, sm_index
                          };
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
}


/*********************************************************************
 * @fn      zclSmartMeter_SendConfigAck     *
 *
 * @brief   Called to send SmartMeter configuration acknowledge to the coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendConfigAck( void )                                         //added 11.12
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {COM_CONFIG, SUCCESS, SM_CONFIG_2, SM_CONFIG_1, SM_CONFIG_0};

        pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
        if ( pReportCmd != NULL )
        {
            pReportCmd->numAttr = 1;
            pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;

            pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT256;
            pReportCmd->attrList[0].attrData = (void *)(packet);
            zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                               ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
                               pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
        }

        osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
    }
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {COM_CONFIG, SUCCESS};

        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;

        /*
        POWER1 = (powerVal[0] >> 16) & 0xFFFF;
        POWER0 = powerVal[0] & 0xFFFF;
        ENERGY1 = (energyVal[0] >> 16) & 0xFFFF;
        ENERGY0 = energyVal[0] & 0xFFFF;
        uint16 packet[] = {USR_TX_GET, SUCCESS, ADD_3, ADD_2, ADD_1, ADD_0, SM_ADD16,
                           RMS_V1, RMS_I1, THETA_1, RMS_V2, RMS_I2, THETA_2, RMS_V3, RMS_I3, THETA_3, SM_V, SM_I, YEAR, MONTH, DAY, HOUR, MINUTE, SECOND, STATUS
                          };
        */
        uint16 packet[32] = {0};
        packet[0] = USR_TX_GET;
        packet[1] = SUCCESS;
        packet[2] = ADD_3;
        packet[3] = ADD_2;
        packet[4] = ADD_1;
        packet[5] = ADD_0;
        packet[6] = ((uint16)(SM_CONFIG_5) << 8) + SM_CONFIG_4;
        packet[7] = ((uint16)(SM_CONFIG_3) << 8) + SM_CONFIG_2;
        packet[8] = ((uint16)(SM_CONFIG_1) << 8) + SM_CONFIG_0;
        packet[9] = (uint16)len_DataReg;
        uint8 i = 0;
        for(i = 0; i < len_DataReg; i++)
            packet[10 + i] =  VIT_dataReg[i];
        packet[10 + len_DataReg] = STATUS;
        packet[11 + len_DataReg] = YEAR;
        packet[12 + len_DataReg] = MONTH;
        packet[13 + len_DataReg] = DAY;
        packet[14 + len_DataReg] = HOUR;
        packet[15 + len_DataReg] = MINUTE;
        packet[16 + len_DataReg] = SECOND;

        //UARTprintf(" RMS_V1: %d\n", RMS_V1);
        //UARTprintf(" RMS_I1: %d\n", RMS_I1);


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
    else if( Connect_Mode == WIRED_CONNECTION)
    {

        //whole length is 17+len_DataReg
        uint16 packet[32] = {0};
        packet[0] = COM_DATA;
        packet[1] = SUCCESS;
        packet[2] = ADD_3;
        packet[3] = ADD_2;
        packet[4] = ADD_1;
        packet[5] = ADD_0;
        packet[6] = ((uint16)(SM_CONFIG_5) << 8) + SM_CONFIG_4;
        packet[7] = ((uint16)(SM_CONFIG_3) << 8) + SM_CONFIG_2;
        packet[8] = ((uint16)(SM_CONFIG_1) << 8) + SM_CONFIG_0;
        packet[9] = (uint16)len_DataReg;
        uint8 i = 0;
        for(i = 0; i < len_DataReg; i++)
            packet[10 + i] = (uint16)(VIT_dataReg[i] * start); // if start = 0, return all 0
        packet[10 + len_DataReg] = STATUS;
        packet[11 + len_DataReg] = YEAR;
        packet[12 + len_DataReg] = MONTH;
        packet[13 + len_DataReg] = DAY;
        packet[14 + len_DataReg] = HOUR;
        packet[15 + len_DataReg] = MINUTE;
        packet[16 + len_DataReg] = SECOND;


        pack_out[0] = 0x68;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)((17 + len_DataReg) * 2);

        for(i = 0; i < (17 + len_DataReg); i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + ((17 + len_DataReg) * 2)] = 0x00;

        for(i = 0; i < 11 + ((17 + len_DataReg) * 2); i++)
            pack_out[11 + ((17 + len_DataReg) * 2)] += pack_out[i];

        pack_out[12 + ((17 + len_DataReg) * 2)] = 0x16;

        uint8 uart0show[4] = {0};
        uart0show[0] = 0xee;
        uart0show[1] = (uint8)(STATUS & 0x00ff);
        uart0show[2] = (uint8)((STATUS & 0xff00) >> 8);
        uart0show[3] = 0xff;
        //HalUART0Write ( HAL_UART_PORT_0, uart0show, 4);


        len_uart1 = 13 + ((17 + len_DataReg) * 2);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + ((17 + len_DataReg) * 2));
        //HalUART0Write ( HAL_UART_PORT_0, pack_out, 13 + ((17 + len_DataReg) * 2));
        HalLcdWriteString( "dataREG2", HAL_LCD_LINE_6 );

    }
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
    uint16 energy_reset_value_1 = 0;
    uint16 energy_reset_value_0 = 0;
    energy_reset_value_1 = (uint16)(((uint32)(ENERGY_RESET_VALUE) >> 16) & 0xFFFF);
    energy_reset_value_0 = (uint16)((uint32)(ENERGY_RESET_VALUE) & 0xFFFF);
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;

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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {(uint16)RESET, (uint16)SUCCESS, flagreset,
                           energy_reset_value_1, energy_reset_value_0
                          };
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {RELAY, SUCCESS, flagrelay};
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;
        //HalLcdWriteString( "relayyyyyy", HAL_LCD_LINE_3 );

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));

    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {CALIBRATE, SUCCESS, V_CAL, I_CAL, T_CAL, N_CAL, INPUT_1_CAL, INPUT_2_CAL};
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        HalLcdWriteString( "send cali", HAL_LCD_LINE_7 );
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
}


/*********************************************************************
 * @fn      zclSmartMeter_SendKeyAck     *
 *
 * @brief   Called to send SmartMeter key ack to the coordinator
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_SendKeyAck( void )
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {SET_KEY, SUCCESS};
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {SET_KEY, SUCCESS};
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
        //HalLcdWriteString( "datastart", HAL_LCD_LINE_6 );
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {START, SUCCESS, start};
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
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {START, SUCCESS, start};
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
        //HalLcdWriteString( "datastart", HAL_LCD_LINE_6 );
    }
}



/*********************************************************************
 * @fn      zclSmartMeter_SendStart    *
 *
 * @brief   Called to send SmartMeter power calculation stop information
 * in response to coordinator
 *
 * @return  none
 */
static void zclSmartMeter_SendStart(void)
{
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {TEMP_STOP, SUCCESS, flaginc};
        pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
        if ( pReportCmd != NULL )
        {
            pReportCmd->numAttr = 1;
            pReportCmd->attrList[0].attrID = ATTRID_MS_ADD_MEASURED_VALUE;
            pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128;
            pReportCmd->attrList[0].attrData = (void *)(packet);
            zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                               ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                               pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
        }
        osal_mem_free( pReportCmd );
    }
#endif  // ZCL_REPORT 
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {TEMP_STOP, SUCCESS, flaginc};
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        HalLcdWriteString( "dataREG1", HAL_LCD_LINE_6 );
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
}

/*********************************************************************
 * @fn      zclSmartMeter_SendAuth    *
 *
 * @brief   Called to send SmartMeter Authentication code to Coordinator
 *
 * @return  none
 */
static void zclSmartMeter_SendAuth(void)
{
  /*
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
      
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;
        uint16 packet[] = {TEMP_STOP, SUCCESS, flaginc};
        pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
        if ( pReportCmd != NULL )
        {
            pReportCmd->numAttr = 1;
            pReportCmd->attrList[0].attrID = ATTRID_MS_ADD_MEASURED_VALUE;
            pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT128;
            pReportCmd->attrList[0].attrData = (void *)(packet);
            zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                               ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
                               pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
        }
        osal_mem_free( pReportCmd );
    }
#endif  // ZCL_REPORT 
    else */
      
    if( Connect_Mode == WIRED_CONNECTION)
    {
        //////// TO BE REPLACED
        uint8 romreg[16] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00, 0x11};
        osal_nv_item_init (FLASH_RO, 40, NULL);
        osal_nv_write (FLASH_RO, 0, 40, &romreg[0]);
        //////// TO BE REPLACED


        osal_nv_item_init (FLASH_RO, 40, NULL);
        osal_nv_read (FLASH_RO, 0, 40, &romread[0]);
        //HalUART0Write ( HAL_UART_PORT_0, romread, 16);
        
        uint16 packet[] = {AUTHEN, SUCCESS, UINT8_TO_16(romread[0], romread[1]), UINT8_TO_16(romread[2], romread[3]), UINT8_TO_16(romread[4], romread[5]), UINT8_TO_16(romread[6], romread[7]),
                                            UINT8_TO_16(romread[8], romread[9]), UINT8_TO_16(romread[10], romread[11]), UINT8_TO_16(romread[12], romread[13]), UINT8_TO_16(romread[14], romread[15])};
        
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
        
    }
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
    if( Connect_Mode == WIRELESS_CONNECTION)
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
    }
#endif  // ZCL_REPORT 
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 packet[] = {COM_ADD, SUCCESS, ADD_3, ADD_2, ADD_1, ADD_0};
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        //uint8 SmartMeter_NwkDiscov_INTERVAL = (recognise_sm_id() + 4); //generate a random delay from 0 to 16000000 SysTIck
        // add random delay function here
        /*
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
        while (SysTickValueGet() - ulValue_start < SmartMeter_NwkDiscov_INTERVAL * 4)
        {
        };
        */
        //uint32 wait_timenew = osal_GetSystemClock();
        //while (osal_GetSystemClock() - wait_timenew < SmartMeter_NwkDiscov_INTERVAL * 5)
        //{
        //}

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
}

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
    if( Connect_Mode == WIRELESS_CONNECTION)
    {
#ifdef ZCL_REPORT
        zclReportCmd_t *pReportCmd;

        uint16 packet[] = {COM_CAL, SUCCESS, SM_ADD16, ADD_3, ADD_2, ADD_1, ADD_0,
                           UINT8_TO_16(SM_CONFIG_5, SM_CONFIG_4), UINT8_TO_16(SM_CONFIG_3, SM_CONFIG_2),
                           UINT8_TO_16(SM_CONFIG_1, SM_CONFIG_0),
                           MAG_V[0], MAG_I[0], MAG_V[1], MAG_I[1], MAG_V[2], MAG_I[2], MAG_V[3], MAG_I[3],
                           MAG_V[4], MAG_I[4], MAG_V[5], MAG_I[5], MAG_V[6], MAG_I[6], MAG_V[7], MAG_I[7],
                           T_EFF
                          };
        pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
        if ( pReportCmd != NULL )
        {
            pReportCmd->numAttr = 1;
            pReportCmd->attrList[0].attrID = ATTRID_MS_COM_MEASURED_VALUE;
            pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT512;
            pReportCmd->attrList[0].attrData = (void *)(packet);
            zcl_SendReportCmd( SmartMeter_ENDPOINT, &zclSmartMeter_DstAddr,
                               ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
                               pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartMeterSeqNum++ );
        }
        osal_mem_free( pReportCmd );

        //UARTprintf(" SM_ADD16: 0x%x\n", SM_ADD16);
        //UARTprintf(" ADD_3: 0x%x\n", ADD_3);
        //UARTprintf(" ADD_2: 0x%x\n", ADD_2);
        //UARTprintf(" ADD_1: 0x%x\n", ADD_1);
        //UARTprintf(" ADD_0: 0x%x\n", ADD_0);
        //UARTprintf(" MAG_V1: 0x%x\n", MAG_V1);
        //UARTprintf(" MAG_I1: 0x%x\n", MAG_I1);
        //UARTprintf(" T_EFF: 0x%x\n", T_EFF);
#endif  // ZCL_REPORT    
    }
    else if( Connect_Mode == WIRED_CONNECTION)
    {
        uint16 MAG[8] = {0};
        uint8 MAG_COUNT = 0;
        if(Num_phase[1])
        {
            MAG[MAG_COUNT] = MAG_V[MAG_COUNT];
            MAG_COUNT++;
            for (uint8 i = 0;  i < Num_phase[1]; i++)
            {
                MAG[MAG_COUNT] = MAG_I[MAG_COUNT];
                MAG_COUNT++;
            }
        }

        if(Num_phase[2])
        {
            MAG[MAG_COUNT] = MAG_V[MAG_COUNT];
            MAG_COUNT++;
            for (uint8 i = 0;  i < Num_phase[2]; i++)
            {
                MAG[MAG_COUNT] = MAG_I[MAG_COUNT];
                MAG_COUNT++;
            }
        }

        if(Num_phase[3])
        {
            MAG[MAG_COUNT] = MAG_V[MAG_COUNT];
            MAG_COUNT++;
            for (uint8 i = 0;  i < Num_phase[3]; i++)
            {
                MAG[MAG_COUNT] = MAG_I[MAG_COUNT];
                MAG_COUNT++;
            }
        }

        if(Num_phase[0])
        {
            for (uint8 i = 0;  i < Num_phase[0]; i++)
            {
                MAG[MAG_COUNT] = MAG_GEN_INPUT1[MAG_COUNT];
                MAG_COUNT++;
            }
        }

        if(Num_phase[4])
        {
            for (uint8 i = 0;  i < Num_phase[4]; i++)
            {
                MAG[MAG_COUNT] = MAG_GEN_INPUT2[MAG_COUNT];
                MAG_COUNT++;
            }
        }

        uint16 packet[] = {COM_CAL, SUCCESS, SM_ADD16, ADD_3, ADD_2, ADD_1, ADD_0,
                           UINT8_TO_16(SM_CONFIG_5, SM_CONFIG_4), UINT8_TO_16(SM_CONFIG_3, SM_CONFIG_2),
                           UINT8_TO_16(SM_CONFIG_1, SM_CONFIG_0),
                           MAG[0], MAG[1], MAG[2], MAG[3], MAG[4], MAG[5], MAG[6], MAG[7],
                           T_EFF
                          };
        pack_out[0] = 0x68;
        uint8 i;
        for(i = 1; i <= 8; i++)
            pack_out[i] = Msg_in[i];

        pack_out[9] = 0x68;
        pack_out[10] = (uint8)sizeof(packet);

        for(i = 0; i < sizeof(packet) / 2; i++)
        {
            pack_out[11 + i * 2] = (uint8_t) ((packet[i] & 0xFF00) >> 8);
            pack_out[12 + i * 2] = (uint8_t) (packet[i] & 0x00FF);
        }

        pack_out[11 + sizeof(packet)] = 0x00;

        for(i = 0; i < 11 + sizeof(packet); i++)
            pack_out[11 + sizeof(packet)] += pack_out[i];

        pack_out[12 + sizeof(packet)] = 0x16;

        len_uart1 = 13 + sizeof(packet);
        //HalUART1Write ( HAL_UART_PORT_1, pack_out, 13 + sizeof(packet));
    }
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

    uint8 pack_o[60] = {0};
    for(uint8 i = 0; i < 18; i++)
    {
        pack_o[i * 2] = (uint8)((paramReg[i] & 0xff00) >> 8);
        pack_o[i * 2 + 1] = (uint8)(paramReg[i] & 0x00ff);
    }
    //HalUART0Write ( HAL_UART_PORT_0, pack_o, 60);

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
        //UARTprintf(" writeparamReg[%d]: %d\n", i, paramReg[i]);
        //UARTprintf(" sm_index: %d\n", sm_index);

        flashpt = &paramReg[0];
    //locate item in flash
    osal_nv_item_init (FLASH_PARAM, 36, NULL);
    //write paramReg to FLASH
    osal_nv_write (FLASH_PARAM, 0, 36, flashpt);

}




/*********************************************************************
 * @fn      zclSmartMeter_WriteConfigReg
 *
 * @brief   Called to write SmartMeter parameter information into Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_WriteConfigReg( void )
{
    uint16 *flashpt;

    paramReg[0] = UINT8_TO_16(SM_CONFIG_5, SM_CONFIG_4);
    paramReg[1] = UINT8_TO_16(SM_CONFIG_3, SM_CONFIG_2);
    paramReg[2] = UINT8_TO_16(SM_CONFIG_1, SM_CONFIG_0);


    uint8 i;
    for(i = 0; i < 3; i++)
        flashpt = &paramReg[0];

    //locate item in flash
    osal_nv_item_init (FLASH_CONFIG, 6, NULL);
    //write paramReg to FLASH
    osal_nv_write (FLASH_CONFIG, 0, 6, flashpt);

}

/*********************************************************************
 * @fn      zclSmartMeter_ReadConfigReg
 *
 * @brief   Called to read SmartMeter calibration related parameter information from Flash
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartMeter_ReadConfigReg( void )
{
    uint16 *flashpt;
    flashpt = &ConfigReg[0];
    //locate item in flash memory
    osal_nv_item_init (FLASH_CONFIG, 6, NULL);
    //read from flash memory and load it into paramReg
    osal_nv_read (FLASH_CONFIG, 0, 6, flashpt);

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

    calReg[4] = UINT8_TO_16(SM_CONFIG_5, SM_CONFIG_4);
    calReg[5] = UINT8_TO_16(SM_CONFIG_3, SM_CONFIG_2);
    calReg[6] = UINT8_TO_16(SM_CONFIG_1, SM_CONFIG_0);

    uint8 i = 0;
    for(i = 0; i < 8; i++)
    {
        if(CAL_OPT == CAL_VOL)
            calReg[7 + i * 4] = MAG_V[i];
        else if(CAL_OPT == CAL_CUR)
            calReg[8 + i * 4] = MAG_I[i];
        else if(CAL_OPT == CAL_GEN_1)
            calReg[9 + i * 4] = MAG_GEN_INPUT1[i];
        else if(CAL_OPT == CAL_GEN_2)
            calReg[10 + i * 4] = MAG_GEN_INPUT2[i];
    }
    T_EFF = run_T_EFF;
    calReg[39] = T_EFF;

    for(i = 0; i < 40; i++)
        //UARTprintf(" writecalReg[%d]: %d\n", i, calReg[i]);
        flashpt = &calReg[0];
    //locate item in flash
    osal_nv_item_init (FLASH_PARAMCAL, 80, NULL);
    //write paramReg to FLASH
    osal_nv_write (FLASH_PARAMCAL, 0, 80, flashpt);
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
    osal_nv_item_init (FLASH_PARAMCAL, 80, NULL);
    //read from flash memory and load it into paramReg
    osal_nv_read (FLASH_PARAMCAL, 0, 80, flashpt);

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
  
    HalLcdWriteString( "CMD Received", HAL_LCD_LINE_7 );
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
    
    //Added by Jinhui, 1/9/2017 to restore wireless function
     else if ((COMMAND == TEMP_STOP))
    {
        flaginc = OPERATION;
        start = 1;
        HalLcdWriteString( "send_start", HAL_LCD_LINE_4 );
        // send the current start value to send over the air to Coordinator
        zclSmartMeter_SendStart();
    }
    
    else if ((COMMAND == USR_RX_GET) && (OPERATION == COM_DATA) &&
             (pInParameterReport->attrList[0].attrID) == ATTRID_MS_DATA_MEASURED_VALUE)
        // send the current data value sent over the air to Coordinator
    {
        HalLcdWriteString( "Send Data", HAL_LCD_LINE_5 );
        time_old = osal_GetSystemClock();
        flaginc = 1;

        sys_timenew = osal_GetSystemClock();
        sys_secnew = sys_secold + (uint32)((float)((sys_timenew - sys_timeold) * TIMEINDEX) / 1000);
        osal_ConvertUTCTime(&TimeStruct , sys_secnew);


        YEAR = (uint16)((TimeStruct.month == 12)? (TimeStruct.year + 1) : TimeStruct.year); //YEAR
        MONTH = (uint16)((TimeStruct.month == 12)? 1: (TimeStruct.month + 1)); //MONTH
        DAY = (uint16)TimeStruct.day + 1;
        HOUR = (uint16)TimeStruct.hour;
        MINUTE = (uint16)TimeStruct.minutes;
        SECOND = (uint16)TimeStruct.seconds;

        STATUS = 0;
        STATUS = STATUS | (uint16)(flagreset << 3);
        STATUS = STATUS | (uint16)(flagrelay << 4);
        STATUS = STATUS | (uint16)(start << 5);

        //HalUART0Write ( HAL_UART_PORT_0, uart0show, 3);
        //UARTprintf(" sys_timenew: %d\n", sys_timenew);
        //UARTprintf(" sys_timeold: %d\n", sys_timeold);
        //UARTprintf(" sys_secnew: %d\n", sys_secnew);
        //UARTprintf(" YEAR: %d\n", YEAR);
        //UARTprintf(" MONTH: %d\n", MONTH);
        //UARTprintf(" DAY: %d\n", DAY);
        //UARTprintf(" HOUR: %d\n", HOUR);
        //UARTprintf(" MINUTE: %d\n", MINUTE);
        //UARTprintf(" SECOND: %d\n", SECOND);

        zclSmartMeter_SendData();
    }
    else if ((COMMAND == USR_RX_SET) && (OPERATION == SET_PARAM) &&
             (pInParameterReport->attrList[0].attrID) == ATTRID_MS_PARAMETER_MEASURED_VALUE)
    {
        HalLcdWriteString( "SET_PARAM", HAL_LCD_LINE_4 );
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

    else if ((COMMAND == TIME_SET) &&
             (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)                                    //added 11.12
    {
        HalLcdWriteString( "sendTime", HAL_LCD_LINE_7 );
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
        //UARTprintf(" YEAR: %d\n", YEAR);
        //UARTprintf(" MONTH: %d\n", MONTH);
        //UARTprintf(" DAY: %d\n", DAY);
        //UARTprintf(" HOUR: %d\n", HOUR);
        //UARTprintf(" MINUTE: %d\n", MINUTE);
        //UARTprintf(" SECOND: %d\n", SECOND);
        //UARTprintf(" sm_index: %d\n", sm_index);
        /*
        int i;
        for(i = 0; i < 18; i++)
            //UARTprintf(" readparamReg[%d]: %d\n", i, paramReg[i]);

        for(i = 0; i < 15; i++)
            //UARTprintf(" readcalReg[%d]: %d\n", i, calReg[i]);
        //Update flash memory
        //  zclSmartMeter_nvWriteParam();
        // send the current parameter value to send over the air to Coordinator
        */

        sys_timeold = osal_GetSystemClock();
        sys_timenew = sys_timeold;
        TimeStruct.seconds = (uint8)SECOND;
        TimeStruct.minutes = (uint8)MINUTE;
        TimeStruct.hour = (uint8)HOUR;
        TimeStruct.day = (uint8)DAY;
        TimeStruct.month = (uint8)MONTH;
        TimeStruct.year = (uint16)YEAR;

        //UARTprintf(" TimeStruct.seconds: %d\n", TimeStruct.seconds);
        //UARTprintf(" TimeStruct.minutes: %d\n", TimeStruct.minutes);
        //UARTprintf(" TimeStruct.hour: %d\n", TimeStruct.hour);
        //UARTprintf(" TimeStruct.day: %d\n", TimeStruct.day);
        //UARTprintf(" TimeStruct.month: %d\n", TimeStruct.month);
        //UARTprintf(" TimeStruct.year: %d\n", TimeStruct.year);
        sys_secold = osal_ConvertUTCSecs( &TimeStruct );
        //UARTprintf(" sys_secold: %d\n", sys_secold);
        zclSmartMeter_SendTime();
        //osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_CLOCK_EVT, 1000 );  //continues 1s
    }

    else if ((COMMAND == RESET)  &&
             (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
    {
        flagreset = OPERATION;
        ENERGY_RESET_VALUE_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        ENERGY_RESET_VALUE_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        ENERGY_RESET_VALUE = BUILD_UINT32_16(ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0);
        //energy_display[0] = (double)ENERGY_RESET_VALUE;
        uint8 i = 0;
        for(i = 0; i < 8; i++)
            energyVal[i] = (uint64)ENERGY_RESET_VALUE;

        //UARTprintf(" ENERGY_RESET_VALUE: %d\n", ENERGY_RESET_VALUE);
        // send the current flagreset and ENERGY_RESET_VALUE to send over the air to Coordinator
        zclSmartMeter_SendReset();
    }


    else if ((COMMAND == RELAY) &&
             (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
    {
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "SendRelay SUCCESS", HAL_LCD_LINE_2 );
#endif
        flagrelay = OPERATION;
        if(!flagrelay)
            //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01); // PC1=0, PC0=1
            GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x01); 
        else if (flagrelay == 1)
            //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x02); // PC1=1, PC0=0
            GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x00); 

        // send the current relay value to send over the air to Coordinator
        zclSmartMeter_SendRelay();

        flagrelay = 2;
    }

    else if ((COMMAND == START) &&
             (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
    {
      
        // original code, commented by Jinhui at 1/30/2017 to restore reset function
        //start = 0;
        //// send the current flaginc value to send over the air to Coordinator
        //zclSmartMeter_SendRestart();
      
        //Following code is copied from wired command handler
        
            start = 0; 
            CAL_OPT = 0;
            // send the current start value to send over the air to Coordinator
            
            /////////////////////////////////////////////
            //reset all the energy calculation related varibles
            rmsTemp_V1 = 0;
            overflow_num_V1 = 0;
            rmsTemp_V2 = 0;
            overflow_num_V2 = 0;
            rmsTemp_V3 = 0;
            overflow_num_V3 = 0;

            for(uint8 i = 0; i < 8; i++)
            {
                rmsTemp_I1[i] = 0;
                overflow_num_I1[i] = 0;
                rmsTemp_I2[i] = 0;
                overflow_num_I2[i] = 0;
                rmsTemp_I3[i] = 0;
                overflow_num_I3[i] = 0;

                accu_GEN_INPUT1[i] = 0;
                accu_GEN_INPUT2[i] = 0;                
            }

            l_nSamples = 0;
            enecal_timeperiod = 0;
            
            flagrelay = 1;
            //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01);
            GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x01);
            flaginc = 1;
            ///////////////////////////////////////
            
            zclSmartMeter_SendRestart();
            zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_CONFIG_STATE|ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
            SystemResetSoft();
        
        
    }

    else if ((COMMAND == USR_RX_GET) && (OPERATION == COM_ADD) &&
             (pInParameterReport->attrList[0].attrID) == ATTRID_MS_ADD_MEASURED_VALUE)
    {
        #ifdef LCD_SUPPORTED
          HalLcdWriteString( "Com Add", HAL_LCD_LINE_2 );
        #endif
        // get the coordinator IEEE address
        coordinator_Addr_3 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[4], pInParameterReport->attrList[0].attrData[5]);
        coordinator_Addr_2 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[6], pInParameterReport->attrList[0].attrData[7]);
        coordinator_Addr_1 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[8], pInParameterReport->attrList[0].attrData[9]);
        coordinator_Addr_0 = BUILD_UINT16(pInParameterReport->attrList[0].attrData[10], pInParameterReport->attrList[0].attrData[11]);
        coordinator_extAddr = BUILD_UINT64_16(coordinator_Addr_3, coordinator_Addr_2,
                                              coordinator_Addr_1, coordinator_Addr_0);
        // Set destination address to 64-bit  check &zclSmartMeter_DstAddr
        zclSmartMeter_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
        zclSmartMeter_DstAddr.addr.shortAddr = 0x0000;
        zclSmartMeter_DstAddr.endPoint = SmartMeter_ENDPOINT;
        //zclSmartMeter_DstAddr.addr.extAddr[7] = (uint8)(((coordinator_extAddr) >> 56) & 0x00000000000000FF);
        //zclSmartMeter_DstAddr.addr.extAddr[6] = (uint8)(((coordinator_extAddr) >> 48) & 0x00000000000000FF);
        //zclSmartMeter_DstAddr.addr.extAddr[5] = (uint8)(((coordinator_extAddr) >> 40) & 0x00000000000000FF);
        //zclSmartMeter_DstAddr.addr.extAddr[4] = (uint8)(((coordinator_extAddr) >> 32) & 0x00000000000000FF);
        //zclSmartMeter_DstAddr.addr.extAddr[3] = (uint8)(((coordinator_extAddr) >> 24) & 0x00000000000000FF);
        //zclSmartMeter_DstAddr.addr.extAddr[2] = (uint8)(((coordinator_extAddr) >> 16) & 0x00000000000000FF);
        //zclSmartMeter_DstAddr.addr.extAddr[1] = (uint8)(((coordinator_extAddr) >> 8) & 0x00000000000000FF);
        //zclSmartMeter_DstAddr.addr.extAddr[0] = (uint8)((coordinator_extAddr) & 0x00000000000000FF);
        // send smart meter IEEE address to coordinator after a random delay from
        // time of reception of network discovery command  -- add code
        zclSmartMeter_SendAdd();
        zclSmartMeter_LcdDisplayTestMode(); // display C's IEEE address
    }

    else if ((COMMAND == USR_RX_GET) && (OPERATION == ACK_SUCCESS) &&
             ((pInParameterReport->attrList[0].attrID) == ATTRID_MS_ACK_MEASURED_VALUE ))
    {
#ifdef LCD_SUPPORTED
        HalLcdWriteString( "SendAck SUCCESS", HAL_LCD_LINE_2 );
#endif
    }

    else if ((COMMAND == USR_RX_GET) && (OPERATION == COM_CAL) &&
             ((pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE ))
    {
        zclSmartMeter_SendCalPara();
    }

    else if ((COMMAND == CALIBRATE) && (pInParameterReport->attrList[0].attrID) == ATTRID_MS_COM_MEASURED_VALUE)
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
            //UARTprintf(" CAL_VOL ");
            //UARTprintf(" V_CAL: %d\n", V_CAL);
            //UARTprintf(" I_CAL: %d\n", I_CAL);
            //UARTprintf(" T_CAL: %d\n", T_CAL);
            //UARTprintf(" N_CAL: %d\n", N_CAL);
        }
        else if(V_CAL == 0 && I_CAL != 0)
        {
            CAL_OPT = CAL_CUR;
            MAG_I1 = 0;
            //UARTprintf(" CAL_CUR ");
            //UARTprintf(" V_CAL: %d\n", V_CAL);
            //UARTprintf(" I_CAL: %d\n", I_CAL);
            //UARTprintf(" T_CAL: %d\n", T_CAL);
            //UARTprintf(" N_CAL: %d\n", N_CAL);
        }
        else if(V_CAL != 0 && I_CAL != 0)
        {
            CAL_OPT = CAL_ENE;

            enecal_cycle = 0;
            //UARTprintf(" CAL_ENE ");
            //UARTprintf(" V_CAL: %d\n", V_CAL);
            //UARTprintf(" I_CAL: %d\n", I_CAL);
            //UARTprintf(" T_CAL: %d\n", T_CAL);
            //UARTprintf(" N_CAL: %d\n", N_CAL);
        }

    }

    else if ((COMMAND == COM_CONFIG) && (pInParameterReport->attrList[0].attrID) == ATTRID_MS_PARAMETER_MEASURED_VALUE)
    {
        #ifdef LCD_SUPPORTED
          HalLcdWriteString( "COM_CONFIG", HAL_LCD_LINE_2 );
        #endif
        SM_CONFIG_5 = pInParameterReport->attrList[0].attrData[3];
        SM_CONFIG_4 = pInParameterReport->attrList[0].attrData[2];
        SM_CONFIG_3 = pInParameterReport->attrList[0].attrData[5];
        SM_CONFIG_2 = pInParameterReport->attrList[0].attrData[4];
        SM_CONFIG_1 = pInParameterReport->attrList[0].attrData[7];
        SM_CONFIG_0 = pInParameterReport->attrList[0].attrData[6];

        //Update flash memory
        zclSmartMeter_WriteConfigReg();
        // send the current parameter value to send over the air to Coordinator
        zclSmartMeter_SendConfigAck();

    }

}
#endif  // ZCL_REPORT

/**************************************************
zclSmartMeter_ProcessUART_Pkt()

********************************************************/
void zclSmartMeter_ProcessUART_Pkt(void)
{
    //HalLcdWriteString( "9876", HAL_LCD_LINE_7 );
    char  lcdString[10];
    
    #ifdef LCD_SUPPORTED
        //sprintf((char *)lcdString, "%x %x",(int)Msg_in[11], (int)Msg_in[12]);
        //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
    #endif
    
    if (flag_end_encry)
    {
        uint8 len_data = Msg_in[10];

        uint8 encry_data[70] = {0};
       
        for (uint8 i = 0; i < len_data; i++)
            encry_data[i] = Msg_in[i + 11];
        
        //HalUART0Write ( HAL_UART_PORT_0, encry_data, len_data);
        for(uint8 i = 0; i < ((len_data%16)? (len_data / 16 + 1) : (len_data / 16)) ; i++)
            AesEncryptDecrypt(end_final_key, encry_data + 16 * i, 0, DECRYPT_AES);
        
        //HalUART0Write ( HAL_UART_PORT_0, encry_data, len_data);
        for (uint8 i = 0; i < len_data; i++)
            Msg_in[i + 11] = encry_data[i];
    }
    
    
    uint16 COMMAND = UINT8_TO_16(Msg_in[11], Msg_in[12]);
    uint16 OPERATION = UINT8_TO_16(Msg_in[13], Msg_in[14]);

    if(Msg_in[1] == 0xFF && Msg_in[2] == 0xFF && Msg_in[3] == 0xFF && Msg_in[4] == 0xFF &&
       Msg_in[5] == 0xFF && Msg_in[6] == 0xFF && Msg_in[7] == 0xFF && Msg_in[8] == 0xFF)
    {
        //if ((COMMAND == START))
        //{                  
            start = 0; 
            CAL_OPT = 0;
            // send the current start value to send over the air to Coordinator
            
            /////////////////////////////////////////////
            //reset all the energy calculation related varibles
            rmsTemp_V1 = 0;
            overflow_num_V1 = 0;
            rmsTemp_V2 = 0;
            overflow_num_V2 = 0;
            rmsTemp_V3 = 0;
            overflow_num_V3 = 0;

            for(uint8 i = 0; i < 8; i++)
            {
                rmsTemp_I1[i] = 0;
                overflow_num_I1[i] = 0;
                rmsTemp_I2[i] = 0;
                overflow_num_I2[i] = 0;
                rmsTemp_I3[i] = 0;
                overflow_num_I3[i] = 0;

                accu_GEN_INPUT1[i] = 0;
                accu_GEN_INPUT2[i] = 0;                
            }

            l_nSamples = 0;
            enecal_timeperiod = 0;
            
            flagrelay = 1;
            //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01);
            GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x01);
            flaginc = 1;
            ///////////////////////////////////////
            
            zclSmartMeter_SendRestart();
            zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_CONFIG_STATE|ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
            SystemResetSoft();
        //}
    }
    if(ADD_3 == ((uint16)((((Msg_in[1]) & 0x00FF) << 8) + (Msg_in[2] & 0x00FF))) && ADD_2 == ((uint16)((((Msg_in[3]) & 0x00FF) << 8) + (Msg_in[4] & 0x00FF)))
            && ADD_1 == ((uint16)((((Msg_in[5]) & 0x00FF) << 8) + (Msg_in[6] & 0x00FF))) && ADD_0 == ((uint16)((((Msg_in[7]) & 0x00FF) << 8) + (Msg_in[8] & 0x00FF))))
    {

        //sprintf((char *)lcdString, "%x %x", (uint8)COMMAND, (uint8)OPERATION );
        //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

        //prosecc relay control command
        if (COMMAND == RELAY)
        {
#ifdef LCD_SUPPORTED
            HalLcdWriteString( "SendRelay SUCCESS", HAL_LCD_LINE_2 );
#endif
            flagrelay = OPERATION;
            if(!flagrelay)
            {
                //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x02); // PC1=1, PC0=0
                GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x00);
                power_flag = 0;
                //flagrelay = 1;
            }
            else if (flagrelay == 1)
            {
                //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01); // PC1=0, PC0=1
                GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x01);
                power_flag = 1;
                //flagrelay = 0;
            }
            // send the current relay value to send over the air to Coordinator
            zclSmartMeter_SendRelay();

            flagrelay = 2;
        }
        
        else if ((COMMAND == USR_RX_GET) && (OPERATION == AUTHEN))
        {          
            zclSmartMeter_SendAuth();   
        }
        
        else if ((COMMAND == USR_RX_SET) && (OPERATION == SET_KEY))
        {          
            uint8 end_key[16] = {0};
            for (uint8 i = 0; i < 16; i++)
                end_key[i] = Msg_in[15 + i];
            
            uint8 encry_data[16] = {0};
            encry_data[0] = (uint8)((ADD_3 & 0xff00) >> 8);
            encry_data[1] = (uint8)(ADD_3 & 0x00ff);
            encry_data[2] = (uint8)((ADD_2 & 0xff00) >> 8);
            encry_data[3] = (uint8)(ADD_2 & 0x00ff);
            encry_data[4] = (uint8)((ADD_1 & 0xff00) >> 8);
            encry_data[5] = (uint8)(ADD_1 & 0x00ff);
            encry_data[6] = (uint8)((ADD_0 & 0xff00) >> 8);
            encry_data[7] = (uint8)(ADD_0 & 0x00ff);

            //HalUART0Write ( HAL_UART_PORT_0, end_key, 16);
            //HalUART0Write ( HAL_UART_PORT_0, encry_data, 16);
            AesEncryptDecrypt(end_key, encry_data, 0, ENCRYPT_AES);
            //HalUART0Write ( HAL_UART_PORT_0, encry_data, 16);

            for (uint8 i = 0; i < 16; i++)
                end_final_key[i] = romread[i];

            //HalUART0Write ( HAL_UART_PORT_0, encry_data, 16);
            //HalUART0Write ( HAL_UART_PORT_0, end_final_key, 16);
            AesEncryptDecrypt(encry_data, end_final_key,  0, ENCRYPT_AES);
            //HalUART0Write ( HAL_UART_PORT_0, end_final_key, 16);
            zclSmartMeter_SendKeyAck();

            //flag_end_encry = true;
        }

        else if ((COMMAND == USR_RX_GET) && (OPERATION == COM_ADD))
        {
            //HalLcdWriteString( "COMADD Network", HAL_LCD_LINE_3 );
            // get the coordinator IEEE address
            coordinator_Addr_3 = UINT8_TO_16(Msg_in[15], Msg_in[16]);
            coordinator_Addr_2 = UINT8_TO_16(Msg_in[17], Msg_in[18]);
            coordinator_Addr_1 = UINT8_TO_16(Msg_in[19], Msg_in[20]);
            coordinator_Addr_0 = UINT8_TO_16(Msg_in[21], Msg_in[22]);
            // send smart meter IEEE address to coordinator after a random delay from
            // time of reception of network discovery command  -- add code
            zclSmartMeter_SendAdd();
            zclSmartMeter_LcdDisplayTestMode(); // display C's IEEE address
        }

        //Process parameter read command
        else if ((COMMAND == USR_RX_GET) && (OPERATION == SET_PARAM))
            // send the current parameter value through UART to Coordinator
        {
            // set the current parameter sent over the air from the Coordinator
            MIN_ADC = UINT8_TO_16(Msg_in[15], Msg_in[16]);
            MAX_ADC = UINT8_TO_16(Msg_in[17], Msg_in[18]);
            SAMPLE_INT = UINT8_TO_16(Msg_in[19], Msg_in[20]);
            SAMPLE_WIN = UINT8_TO_16(Msg_in[21], Msg_in[22]);
            MIN_V = UINT8_TO_16(Msg_in[23], Msg_in[24]);
            MAX_V = UINT8_TO_16(Msg_in[25], Msg_in[26]);
            MIN_I = UINT8_TO_16(Msg_in[27], Msg_in[28]);
            MAX_I = UINT8_TO_16(Msg_in[29], Msg_in[30]);
            SHARP1 = UINT8_TO_16(Msg_in[31], Msg_in[32]);
            SHARP2 = UINT8_TO_16(Msg_in[33], Msg_in[34]);
            PEAK1 = UINT8_TO_16(Msg_in[35], Msg_in[36]);
            PEAK2 = UINT8_TO_16(Msg_in[37], Msg_in[38]);
            PEAK3 = UINT8_TO_16(Msg_in[39], Msg_in[40]);
            SHOULDER1 = UINT8_TO_16(Msg_in[41], Msg_in[42]);
            SHOULDER2 = UINT8_TO_16(Msg_in[43], Msg_in[44]);
            SHOULDER3 = UINT8_TO_16(Msg_in[45], Msg_in[46]);
            OFF = UINT8_TO_16(Msg_in[47], Msg_in[48]);
            N_SM = UINT8_TO_16(Msg_in[49], Msg_in[50]);
            sm_index = UINT8_TO_16(Msg_in[51], Msg_in[52]);

            //Update flash memory
            zclSmartMeter_nvWriteParam();
            // send the current parameter value to send over the air to Coordinator
            zclSmartMeter_SendParam();
        }
        //process power calculation start/stop command -> CHANGED INTO Reset command
        else if ((COMMAND == START))
        {        
            start = 0; 
            CAL_OPT = 0;
            // send the current start value to send over the air to Coordinator
            
            /////////////////////////////////////////////
            //reset all the energy calculation related varibles
            rmsTemp_V1 = 0;
            overflow_num_V1 = 0;
            rmsTemp_V2 = 0;
            overflow_num_V2 = 0;
            rmsTemp_V3 = 0;
            overflow_num_V3 = 0;

            for(uint8 i = 0; i < 8; i++)
            {
                rmsTemp_I1[i] = 0;
                overflow_num_I1[i] = 0;
                rmsTemp_I2[i] = 0;
                overflow_num_I2[i] = 0;
                rmsTemp_I3[i] = 0;
                overflow_num_I3[i] = 0;

                accu_GEN_INPUT1[i] = 0;
                accu_GEN_INPUT2[i] = 0;                
            }

            l_nSamples = 0;
            enecal_timeperiod = 0;
            
            flagrelay = 1;
            //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01);
            GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x01);
            flaginc = 1;
            ///////////////////////////////////////
            
            zclSmartMeter_SendRestart();
            zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_CONFIG_STATE|ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
            SystemResetSoft();
        }

        else if ((COMMAND == TEMP_STOP))
        {
            flaginc = OPERATION;
            start = 1;
            // send the current start value to send over the air to Coordinator
            zclSmartMeter_SendStart();
        }

        //Process dataReg read command
        else if ((COMMAND == USR_RX_GET) && (OPERATION == COM_DATA))
            // send the current data value through UART to Coordinator
        {
            /*
            HalLcdWriteString( "dataREG", HAL_LCD_LINE_4 );
            uint8 uart0show[1] = {0};
            uart0show[0] = 0xFF;
            HalUART0Write ( HAL_UART_PORT_0, uart0show, 1);
            */
            time_old = osal_GetSystemClock();
            flaginc = 1;

            sys_timenew = osal_GetSystemClock();
            sys_secnew = sys_secold + (uint32)((float)((sys_timenew - sys_timeold) * TIMEINDEX) / 1000);
            osal_ConvertUTCTime(&TimeStruct , sys_secnew);

            
            YEAR = (uint16)((TimeStruct.month == 12)? (TimeStruct.year + 1) : TimeStruct.year); //YEAR
            MONTH = (uint16)((TimeStruct.month == 12)? 1: (TimeStruct.month + 1)); //MONTH
            DAY = (uint16)TimeStruct.day + 1;
            HOUR = (uint16)TimeStruct.hour;
            MINUTE = (uint16)TimeStruct.minutes;
            SECOND = (uint16)TimeStruct.seconds;

            STATUS = 0;
            STATUS = STATUS | (uint16)(flagreset << 3);
            STATUS = STATUS | (uint16)(flagrelay << 4);
            STATUS = STATUS | (uint16)(start << 5);

            //UARTprintf(" sys_timenew: %d\n", sys_timenew);
            //UARTprintf(" sys_timeold: %d\n", sys_timeold);
            //UARTprintf(" sys_secnew: %d\n", sys_secnew);
            //UARTprintf(" YEAR: %d\n", YEAR);
            //UARTprintf(" MONTH: %d\n", MONTH);
            //UARTprintf(" DAY: %d\n", DAY);
            //UARTprintf(" HOUR: %d\n", HOUR);
            //UARTprintf(" MINUTE: %d\n", MINUTE);
            //UARTprintf(" SECOND: %d\n", SECOND);

            zclSmartMeter_SendData();
        }
        //Add in more command here

        else if ((COMMAND == USR_RX_SET) && (OPERATION == SET_PARAM))
        {
            // set the current parameter sent over the air from the Coordinator
            MIN_ADC = UINT8_TO_16(Msg_in[15], Msg_in[16]);
            MAX_ADC = UINT8_TO_16(Msg_in[17], Msg_in[18]);
            SAMPLE_INT = UINT8_TO_16(Msg_in[19], Msg_in[20]);
            SAMPLE_WIN = UINT8_TO_16(Msg_in[21], Msg_in[22]);
            MIN_V = UINT8_TO_16(Msg_in[23], Msg_in[24]);
            MAX_V = UINT8_TO_16(Msg_in[25], Msg_in[26]);
            MIN_I = UINT8_TO_16(Msg_in[27], Msg_in[28]);
            MAX_I = UINT8_TO_16(Msg_in[29], Msg_in[30]);
            SHARP1 = UINT8_TO_16(Msg_in[31], Msg_in[32]);
            SHARP2 = UINT8_TO_16(Msg_in[33], Msg_in[34]);
            PEAK1 = UINT8_TO_16(Msg_in[35], Msg_in[36]);
            PEAK2 = UINT8_TO_16(Msg_in[37], Msg_in[38]);
            PEAK3 = UINT8_TO_16(Msg_in[39], Msg_in[40]);
            SHOULDER1 = UINT8_TO_16(Msg_in[41], Msg_in[42]);
            SHOULDER2 = UINT8_TO_16(Msg_in[43], Msg_in[44]);
            SHOULDER3 = UINT8_TO_16(Msg_in[45], Msg_in[46]);
            OFF = UINT8_TO_16(Msg_in[47], Msg_in[48]);
            N_SM = UINT8_TO_16(Msg_in[49], Msg_in[50]);
            sm_index = UINT8_TO_16(Msg_in[51], Msg_in[52]);

            //Update flash memory
            zclSmartMeter_nvWriteParam();
            
            zclSmartMeter_parameterInit();
            // send the current parameter value to send over the air to Coordinator
            zclSmartMeter_SendParam();
        }

        else if ((COMMAND == TIME_SET))
        {
            // set the current parameter sent over the air from the Coordinator
            YEAR = UINT8_TO_16(Msg_in[13], Msg_in[14]);
            MONTH = UINT8_TO_16(Msg_in[15], Msg_in[16]);
            DAY = UINT8_TO_16(Msg_in[17], Msg_in[18]);
            HOUR = UINT8_TO_16(Msg_in[19], Msg_in[20]);
            MINUTE = UINT8_TO_16(Msg_in[21], Msg_in[22]);
            SECOND = UINT8_TO_16(Msg_in[23], Msg_in[24]);
            sm_index = UINT8_TO_16(Msg_in[25], Msg_in[26]);

            //timeReg[0] = YEAR;
            //timeReg[1] = MONTH;
            //timeReg[2] = DAY;
            //timeReg[3] = HOUR;
            //timeReg[4] = MINUTE;
            //timeReg[5] = SECOND;
            HalLcdWriteString( "timesetvalue", HAL_LCD_LINE_7 );
            //UARTprintf(" YEAR: %d\n", YEAR);
            //UARTprintf(" MONTH: %d\n", MONTH);
            //UARTprintf(" DAY: %d\n", DAY);
            //UARTprintf(" HOUR: %d\n", HOUR);
            //UARTprintf(" MINUTE: %d\n", MINUTE);
            //UARTprintf(" SECOND: %d\n", SECOND);
            //UARTprintf(" sm_index: %d\n", sm_index);
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

            //UARTprintf(" TimeStruct.seconds: %d\n", TimeStruct.seconds);
            //UARTprintf(" TimeStruct.minutes: %d\n", TimeStruct.minutes);
            //UARTprintf(" TimeStruct.hour: %d\n", TimeStruct.hour);
            //UARTprintf(" TimeStruct.day: %d\n", TimeStruct.day);
            //UARTprintf(" TimeStruct.month: %d\n", TimeStruct.month);
            //UARTprintf(" TimeStruct.year: %d\n", TimeStruct.year);
            sys_secold = osal_ConvertUTCSecs( &TimeStruct );
            //UARTprintf(" sys_secold: %d\n", sys_secold);
            zclSmartMeter_SendTime();
            //osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_CLOCK_EVT, 1000 );  //continues 1s
        }

        else if ((COMMAND == RESET))
        {
            flagreset = OPERATION;
            ENERGY_RESET_VALUE_1 = UINT8_TO_16(Msg_in[15], Msg_in[16]);
            ENERGY_RESET_VALUE_0 = UINT8_TO_16(Msg_in[17], Msg_in[18]);
            ENERGY_RESET_VALUE = BUILD_UINT32_16(ENERGY_RESET_VALUE_1, ENERGY_RESET_VALUE_0);
            //energy_display[0] = (double)ENERGY_RESET_VALUE;
            uint8 i = 0;
            for(i = 0 ; i < 8; i++)
            {
                energyVal[i] = (uint64)ENERGY_RESET_VALUE * 100000; //ENERGY_RESET_VALUE in J
                Energy[i] = (double)((double)energyVal[i] / 1000 / 3600 / 100000); //energy magnified in kWh
            }
            
            /*
            start = 0;
            //reset all the energy calculation related varibles
            rmsTemp_V1 = 0;
            overflow_num_V1 = 0;
            rmsTemp_V2 = 0;
            overflow_num_V2 = 0;
            rmsTemp_V3 = 0;
            overflow_num_V3 = 0;

            //uint8 i = 0;
            for(i = 0; i < 8; i++)
            {
                rmsTemp_I1[i] = 0;
                overflow_num_I1[i] = 0;
                rmsTemp_I2[i] = 0;
                overflow_num_I2[i] = 0;
                rmsTemp_I3[i] = 0;
                overflow_num_I3[i] = 0;

                accu_GEN_INPUT1[i] = 0;
                accu_GEN_INPUT2[i] = 0;                
            }

            l_nSamples = 0;
            enecal_timeperiod = 0;
            
            flagrelay = 1;
            //GPIOPinWrite(GPIO_C_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0x01);
            GPIOPinWrite(GPIO_C_BASE, GPIO_PIN_0, 0x01);
            start = 0; 
            flaginc = 1;
            */
            
            //UARTprintf(" ENERGY_RESET_VALUE: %d\n", ENERGY_RESET_VALUE);
            // send the current flagreset and ENERGY_RESET_VALUE to send over the air to Coordinator
            zclSmartMeter_SendReset();
            flagreset = 0;
        }


        else if ((COMMAND == USR_RX_GET) && (OPERATION == ACK_SUCCESS))
        {
#ifdef LCD_SUPPORTED
            HalLcdWriteString( "SendAck SUCCESS", HAL_LCD_LINE_2 );
#endif
        }

        else if ((COMMAND == USR_RX_GET) && (OPERATION == COM_CAL))
        {
            zclSmartMeter_SendCalPara();
        }

        else if ((COMMAND == CALIBRATE))
        {
            start = 1;
            //HalLcdWriteString( "cali0", HAL_LCD_LINE_4 );
            V_CAL = UINT8_TO_16(Msg_in[13], Msg_in[14]);
            I_CAL = UINT8_TO_16(Msg_in[15], Msg_in[16]);
            T_CAL = UINT8_TO_16(Msg_in[17], Msg_in[18]);
            N_CAL = UINT8_TO_16(Msg_in[19], Msg_in[20]);
            INPUT_1_CAL = UINT8_TO_16(Msg_in[21], Msg_in[22]);
            INPUT_2_CAL = UINT8_TO_16(Msg_in[23], Msg_in[24]);

            time_old = osal_GetSystemClock();
            time_new = time_old;


            l_nSamples = 0;
            VrmsTemp[0] = 0;
            IrmsTemp[0] = 0;
            //sprintf((char *)lcdString, "%d %d %d %d %d %d", V_CAL, I_CAL, T_CAL, N_CAL, INPUT_1_CAL, INPUT_2_CAL);
            //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
            if(V_CAL != 0 && I_CAL == 0)
            {

                CAL_OPT = CAL_VOL;
                MAG_V1 = 0;
                //sprintf((char *)lcdString, "%d %d %d %d", V_CAL, I_CAL, T_CAL, N_CAL );
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
            }
            else if(V_CAL == 0 && I_CAL != 0)
            {
                CAL_OPT = CAL_CUR;
                MAG_I1 = 0;
            }
            else if(V_CAL != 0 && I_CAL != 0)
            {
                CAL_OPT = CAL_ENE;
                enecal_cycle = 0;
            }
            else if(INPUT_1_CAL != 0 && N_CAL != 0)
            {
                CAL_OPT = CAL_GEN_1;
            }

            else if(INPUT_2_CAL != 0 && N_CAL != 0)
            {
                CAL_OPT = CAL_GEN_2;
            }

        }

        else if (COMMAND == COM_CONFIG)
        {
          
            SM_CONFIG_5 = Msg_in[13];
            SM_CONFIG_4 = Msg_in[14];
            SM_CONFIG_3 = Msg_in[15];
            SM_CONFIG_2 = Msg_in[16];
            SM_CONFIG_1 = Msg_in[17];
            SM_CONFIG_0 = Msg_in[18];
            
            
            
          
            zclSmartMeter_WriteConfigReg();
            /*
            for (uint8 i = 0; i < 3; i++)
                ConfigReg[i] = 0;
            zclSmartMeter_ReadConfigReg();
            */
/*
            SM_CONFIG_5 = (uint8)((ConfigReg[0] & 0xff00) >> 8);
            SM_CONFIG_4 = (uint8)((ConfigReg[0] & 0x00ff));
            SM_CONFIG_3 = (uint8)((ConfigReg[1] & 0xff00) >> 8);
            SM_CONFIG_2 = (uint8)((ConfigReg[1] & 0x00ff));
            SM_CONFIG_1 = (uint8)((ConfigReg[2] & 0xff00) >> 8);
            SM_CONFIG_0 = (uint8)((ConfigReg[2] & 0x00ff));
            */
            // Initialize SmartMeter parameters
            //zclSmartMeter_parameterInit();
            //Initialize SmartMeter data register
            //zclSmartMeter_dataRegInit();
            Configuration_Reg_Process();
            
            zclSmartMeter_SendConfigAck();
        }

    }
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

    for (i = 0; i < 24; i++)
        calReg[i] = 0;
    zclSmartMeter_calReadParam();
    /*
        SM_CONFIG_5 = (uint8)((calReg[4] & 0xff00) >> 8);
        SM_CONFIG_4 = (uint8)((calReg[4] & 0x00ff));
        SM_CONFIG_3 = (uint8)((calReg[5] & 0xff00) >> 8);
        SM_CONFIG_2 = (uint8)((calReg[5] & 0x00ff));
        SM_CONFIG_1 = (uint8)((calReg[6] & 0xff00) >> 8);
        SM_CONFIG_0 = (uint8)((calReg[6] & 0x00ff));
        */
    for(i = 0; i < 8; i++)
    {
        MAG_V[i] = calReg[7 + i * 4];
        MAG_I[i] = calReg[8 + i * 4];
        MAG_GEN_INPUT1[i] = calReg[9 + i * 4];
        MAG_GEN_INPUT2[i] = calReg[10 + i * 4];
    }
    T_EFF = calReg[39];

    for (i = 0; i < 3; i++)
        ConfigReg[i] = 0;
    zclSmartMeter_ReadConfigReg();

    SM_CONFIG_5 = (uint8)((ConfigReg[0] & 0xff00) >> 8);
    SM_CONFIG_4 = (uint8)((ConfigReg[0] & 0x00ff));
    SM_CONFIG_3 = (uint8)((ConfigReg[1] & 0xff00) >> 8);
    SM_CONFIG_2 = (uint8)((ConfigReg[1] & 0x00ff));
    SM_CONFIG_1 = (uint8)((ConfigReg[2] & 0xff00) >> 8);
    SM_CONFIG_0 = (uint8)((ConfigReg[2] & 0x00ff));
    time_old = osal_GetSystemClock();
    flaginc = 1;
}


//functions realted to FFT
static  void   initW()
{
    int   i;
    //W = (complex *)malloc(sizeof(complex)   *   FFT_N);
    for(i = 0; i < FFT_N; i++)
    {
        W[i].real = cos(2 * PI / FFT_N * i);
        W[i].img = -1 * sin(2 * PI / FFT_N * i);
    }
}

static  void   fft()
{
    int   i = 0, j = 0, k = 0, l = 0;
    complex   up, down, product;
    change();
    for(i = 0; i <   log(FFT_N) / log(2)   ; i++)
    {
        l = 1 << i;
        for(j = 0; j < FFT_N; j +=   2 * l   )
        {
            for(k = 0; k < l; k++)
            {
                mul(x[j + k + l], W[FFT_N * k / 2 / l], &product);
                add(x[j + k], product, &up);
                sub(x[j + k], product, &down);
                x[j + k] = up;
                x[j + k + l] = down;
            }
        }
    }
}

static void   change()
{
    complex   temp;
    unsigned   short   i = 0, j = 0, k = 0;
    double   t;
    for(i = 0; i < FFT_N; i++)
    {
        k = i;
        j = 0;
        t = (log(FFT_N) / log(2));
        while(   (t--) > 0   )
        {
            j = j << 1;
            j |= (k   &   1);
            k = k >> 1;
        }
        if(j > i)
        {
            temp = x[i];
            x[i] = x[j];
            x[j] = temp;
        }
    }
}


static void   add(complex   a, complex   b, complex   *c)
{
    c->real = a.real + b.real;
    c->img = a.img + b.img;
}

static void   mul(complex   a, complex   b, complex   *c)
{
    c->real = a.real * b.real   -   a.img * b.img;
    c->img = a.real * b.img   +   a.img * b.real;
}
static void   sub(complex   a, complex   b, complex   *c)
{
    c->real = a.real - b.real;
    c->img = a.img - b.img;
}
static void   divi(complex   a, complex   b, complex   *c)
{
    c->real = (   a.real * b.real + a.img * b.img   ) / (
                  b.real * b.real + b.img * b.img);
    c->img = (   a.img * b.real - a.real * b.img) / (b.real * b.real + b.img * b.img);
}
//functions realted to FFT


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
    char lcdString[15];

    if(flaginc == 0 && start == 1)  //modified by xu, check if the start value is 1(start/stop power calculation flag)
    {
        uint8 j = 0;
        uint8 i = 0;
        uint8 m = 0;

        uint32 ana_time_old = osal_GetSystemClock();

        senValue[0] = analogRead(SOCADC_AIN0); //get ADC voltage value on channel 7
        senValue[1] = analogRead(SOCADC_AIN1); //get ADC current value on channel 6
        senValue[2] = analogRead(SOCADC_AIN2); //get ADC voltage value on channel 5
        senValue[3] = analogRead(SOCADC_AIN3); //get ADC current value on channel 4
        senValue[4] = analogRead(SOCADC_AIN4); //get ADC voltage value on channel 3
        senValue[5] = analogRead(SOCADC_AIN5); //get ADC current value on channel 2
        senValue[6] = analogRead(SOCADC_AIN6); //get ADC voltage value on channel 1
        senValue[7] = analogRead(SOCADC_AIN7); //get ADC current value on channel 0
        uint32 phase_time_now = osal_GetSystemClock();

        uint32 ana_time_new = osal_GetSystemClock();

        //uint8 uart0show[3] = {0};
        //uart0show[0] = 0xee;
        //uart0show[1] = (uint8)(ana_time_new - ana_time_old);
        //uart0show[2] = 0xfe;
        //HalUART0Write ( HAL_UART_PORT_0, uart0show, 3);
        
        uint8 uart0show[6] = {0};
        uart0show[0] = 0xee;
        uart0show[1] = (uint8)((senValue[5] & 0xff00) >> 8);
        uart0show[2] = (uint8)(senValue[5] & 0x00ff);
        uart0show[3] = (uint8)((senValue[6] & 0xff00) >> 8);
        uart0show[4] = (uint8)(senValue[6]);
        uart0show[5] = 0xff;
        
        //HalUART0Write ( HAL_UART_PORT_0, uart0show, 6);
        //sprintf((char *)lcdString, " %d %d",  uart0show[3], uart0show[4] );
        //HalLcdWriteString( lcdString, HAL_LCD_LINE_5 );


        l_nSamples ++;

        if(CAL_OPT == 0)
        {
            if(l_nSamples % 1000 == 0 && l_nSamples <= 10000)
            {
                time_new = osal_GetSystemClock();

                //sprintf((char *)lcdString, "T_EFF: %d", (time_new - time_old) );
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_5 );
                run_T_EFF = (uint16)(time_new - time_old) ;

                time_old = time_new;
            }
            if(Num_phase[1])
            {
                REAL_V1 = (int16)(zclSmartMeter_map((int16)senValue[j++], (int16)MIN_ADC,
                                                    (int16)MAX_ADC, (int16)((float)(MIN_V) * sqrt(2) * (-1)), (int16)((float)MAX_V * sqrt(2)))); //map to real voltage
                
                
                for(i = 0; i < FFT_N - 1; i++)
                {
                     VOI_sample[m * FFT_N + i] = VOI_sample[m * FFT_N + i + 1];                     
                }
                     VOI_sample[m * FFT_N + FFT_N - 1] = REAL_V1;
                
                
                m++;
                rmsTemp_V1 += (REAL_V1 * REAL_V1);

                if (rmsTemp_V1 >= SUM_SQUR_LIMIT)
                {
                    rmsTemp_V1 = rmsTemp_V1 - SUM_SQUR_LIMIT;
                    overflow_num_V1++;
                }
                
                
                for(i = 0; i < Num_phase[1]; i++)
                {
                    REAL_I1[i] = (int16)(zclSmartMeter_map((int16)senValue[j++], (int16)MIN_ADC, (int16)MAX_ADC,
                                                           (int16)((float)(MIN_I * 100) * sqrt(2) * (-1) ), (int16)(MAX_I * 100 * sqrt(2)))); //map to real current, mag by MAG
                    
                    uint8 i_fft = 0;
                    for(i_fft = 0; i_fft < FFT_N - 1; i_fft++)
                    {
                         VOI_sample[m * FFT_N + i_fft] = VOI_sample[m * FFT_N + i_fft + 1];
                    }
                         VOI_sample[m * FFT_N + FFT_N - 1] = REAL_I1[i];
                    
                    m++;
                    rmsTemp_I1[i] += REAL_I1[i] * REAL_I1[i];
                    if (rmsTemp_I1[i] >= SUM_SQUR_LIMIT)
                    {
                        rmsTemp_I1[i] = rmsTemp_I1[i] - SUM_SQUR_LIMIT;
                        overflow_num_I1[i]++;
                    }
                }
            }		
            
            if(Num_phase[2])
            {
                REAL_V2 = (int16)(zclSmartMeter_map((int16)senValue[j++], (int16)MIN_ADC,
                                                    (int16)MAX_ADC, (int16)((float)(MIN_V) * sqrt(2) * (-1)), (int16)((float)MAX_V * sqrt(2)))); //map to real voltage
                
                
                for(i = 0; i < FFT_N - 1; i++)
                {
                     VOI_sample[m * FFT_N + i] = VOI_sample[m * FFT_N + i + 1];
                }
                     VOI_sample[m * FFT_N + FFT_N - 1] = REAL_V2;
                               
                m++;
                rmsTemp_V2 += (REAL_V2 * REAL_V2);
		totalTemp_V2 += REAL_V2;////0928

                if (rmsTemp_V2 >= SUM_SQUR_LIMIT)
                {
                    rmsTemp_V2 = rmsTemp_V2 - SUM_SQUR_LIMIT;
                    overflow_num_V2++;
                }
                
                
                for(i = 0; i < Num_phase[2]; i++)
                {
                    REAL_I2[i] = (int16)(zclSmartMeter_map((int16)senValue[j++], (int16)MIN_ADC, (int16)MAX_ADC,
                                                           (int16)((float)(MIN_I * 100) * sqrt(2) * (-1) ), (int16)(MAX_I * 100 * sqrt(2)))); //map to real current, mag by MAG
                    
                    uint8 i_fft = 0;
                    for(i_fft = 0; i_fft < FFT_N - 1; i_fft++)
                    {
                         VOI_sample[m * FFT_N + i_fft] = VOI_sample[m * FFT_N + i_fft + 1];
                    }
                         VOI_sample[m * FFT_N + FFT_N - 1] = REAL_I2[i];
                    
                    m++;
                    rmsTemp_I2[i] += REAL_I2[i] * REAL_I2[i];
		    totalTemp_I2[i] += REAL_I2[i];////0928
                    if (rmsTemp_I2[i] >= SUM_SQUR_LIMIT)
                    {
                        rmsTemp_I2[i] = rmsTemp_I2[i] - SUM_SQUR_LIMIT;
                        overflow_num_I2[i]++;
                    }
                }
            }

            if(Num_phase[3])
            {
                REAL_V3 = (int16)(zclSmartMeter_map((int16)senValue[j++], (int16)MIN_ADC,
                                                    (int16)MAX_ADC, (int16)((float)(MIN_V) * sqrt(2) * (-1)), (int16)((float)MAX_V * sqrt(2)))); //map to real voltage
                
                
                for(i = 0; i < FFT_N - 1; i++)
                {
                     VOI_sample[m * FFT_N + i] = VOI_sample[m * FFT_N + i + 1];                    
                }
                     VOI_sample[m * FFT_N + FFT_N - 1] = REAL_V3;
                
                
                m++;
                rmsTemp_V3 += (REAL_V3 * REAL_V3);

                if (rmsTemp_V3 >= SUM_SQUR_LIMIT)
                {
                    rmsTemp_V3 = rmsTemp_V3 - SUM_SQUR_LIMIT;
                    overflow_num_V3++;
                }
                
                
                for(i = 0; i < Num_phase[3]; i++)
                {
                    REAL_I3[i] = (int16)(zclSmartMeter_map((int16)senValue[j++], (int16)MIN_ADC, (int16)MAX_ADC,
                                                           (int16)((float)(MIN_I * 100) * sqrt(2) * (-1) ), (int16)(MAX_I * 100 * sqrt(2)))); //map to real current, mag by MAG
                    
                    uint8 i_fft = 0;
                    for(i_fft = 0; i_fft < FFT_N - 1; i_fft++)
                    {
                         VOI_sample[m * FFT_N + i_fft] = VOI_sample[m * FFT_N + i_fft + 1];
                    }
                         VOI_sample[m * FFT_N + FFT_N - 1] = REAL_I3[i];
                    
                    m++;
                    rmsTemp_I3[i] += REAL_I3[i] * REAL_I3[i];
                    if (rmsTemp_I3[i] >= SUM_SQUR_LIMIT)
                    {
                        rmsTemp_I3[i] = rmsTemp_I3[i] - SUM_SQUR_LIMIT;
                        overflow_num_I3[i]++;
                    }
                }
            }			

            if(Num_phase[0])
            {

                for (i = 0;  i < Num_phase[0]; i++)
                {
                    REAL_GEN_INPUT1[i] = senValue[j++];
                    accu_GEN_INPUT1[i] += REAL_GEN_INPUT1[i];
                }
            }

            if(Num_phase[4])
            {

                for (i = 0;  i < Num_phase[4]; i++)
                {
                    REAL_GEN_INPUT2[i] = senValue[j++];
                    accu_GEN_INPUT2[i] += REAL_GEN_INPUT2[i];
                }
            }

        }
        
        else if(CAL_OPT == CAL_VOL)
        {

            //sprintf((char *)lcdString, "sample: %d", l_nSamples );
            //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

            if (l_nSamples == N_CAL * 2)
            {
                time_new = osal_GetSystemClock();

                //sprintf((char *)lcdString, "time1: %d", (time_new - time_old) );
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
                time_old = time_new;
                for(uint8 i = 0; i < 8; i++)
                {
                    //CRMS_V[i] = (uint16)sqrt((double)rmsTemp_calV[i] / l_nSamples - ((double)totalTemp_calV[i] / l_nSamples)*((double)totalTemp_calV[i] / l_nSamples)); //get RMS voltage
                    CRMS_V[i] = (uint16)sqrt((rmsTemp_calV[i] / l_nSamples) - (uint32)((abs(totalTemp_calV[i]) / l_nSamples)*(abs(totalTemp_calV[i]) / l_nSamples))); //get RMS voltage
                    
                    //CRMS_V[i] = (uint16)sqrt(rmsTemp_calV[i] / l_nSamples);
                    rmsTemp_calV[i] = 0;
		    totalTemp_calV[i] = 0;
                    MAG_V[i] = (uint16)(V_CAL * 100 / CRMS_V[i]);
                }
                
                
                l_nSamples = 0;

                zclSmartMeter_calWriteParam();
                zclSmartMeter_SendCalibrate();
                CAL_OPT = 0;
            }
            else
            {
                //l_nSamples++;
                for(uint8 i = 0; i < 8; i++)
                {
                    Cal_Vreal[i] = (int16)(zclSmartMeter_map((int16)senValue[i], (int16)MIN_ADC,
                                           (int16)MAX_ADC, (int16)((float)(MIN_V) * sqrt(2) * (-1)), (int16)((float)MAX_V * sqrt(2)))); //map to real voltage
                    rmsTemp_calV[i] += (Cal_Vreal[i] * Cal_Vreal[i]);
		    totalTemp_calV[i] += Cal_Vreal[i];
                }
            }
        }

        else if(CAL_OPT == CAL_CUR)
        {
            //sprintf((char *)lcdString, "sample: %d", l_nSamples );
            //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

            if (l_nSamples == N_CAL * 2)
            {
                time_new = osal_GetSystemClock();

                //sprintf((char *)lcdString, "time2: %d", (time_new - time_old) );
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
                time_old = time_new;
                for(uint8 i = 0; i < 8; i++)
                {
                    CRMS_I[i] = (uint16)sqrt(rmsTemp_calI[i] / l_nSamples - (uint32)((abs(totalTemp_calI[i]) / l_nSamples)*(abs(totalTemp_calI[i]) / l_nSamples))); //get RMS voltage
                    
                    //CRMS_I[i] = (uint16)sqrt((double)rmsTemp_calI[i] / l_nSamples - ((double)totalTemp_calI[i] / l_nSamples) * ((double)totalTemp_calI[i] / l_nSamples)); //get RMS voltage
                    //CRMS_I[i] = (uint16)sqrt(rmsTemp_calI[i] / l_nSamples);
                    rmsTemp_calI[i] = 0;
		    totalTemp_calI[i] = 0;
                    MAG_I[i] = (uint16)(I_CAL * 10000 / CRMS_I[i]);
                }

                l_nSamples = 0;
                zclSmartMeter_calWriteParam();
                // send the calibration value over the air to Coordinator
                zclSmartMeter_SendCalibrate();
                CAL_OPT = 0;
            }
            else
            {
                //l_nSamples++;
                for(uint8 i = 0; i < 8; i++)
                {
                    Cal_Ireal[i] = (int16)(zclSmartMeter_map((int16)senValue[i], (int16)MIN_ADC, (int16)MAX_ADC,
                                           (int16)((float)(MIN_I * 100) * sqrt(2) * (-1) ), (int16)(MAX_I * 100 * sqrt(2)))); //map to real current, mag by MAG
                    rmsTemp_calI[i] += (Cal_Ireal[i] * Cal_Ireal[i]);
		    totalTemp_calI[i] += Cal_Ireal[i];
                }
            }
        }
/*
        else if(CAL_OPT == CAL_VOL)
        {

            sprintf((char *)lcdString, "sample: %d", l_nSamples );
            HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

            if (l_nSamples == N_CAL * 4)
            {
                time_new = osal_GetSystemClock();

                sprintf((char *)lcdString, "time1: %d", (time_new - time_old) );
                HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
                time_old = time_new;
                for(uint8 i = 0; i < 8; i++)
                {
                    CRMS_V[i] = (uint16)sqrt(rmsTemp_calV[i] / l_nSamples); //get RMS voltage
                    rmsTemp_calV[i] = 0;
                    MAG_V[i] = (uint16)(V_CAL * 100 / CRMS_V[i]);
                }
                l_nSamples = 0;

                zclSmartMeter_calWriteParam();
                zclSmartMeter_SendCalibrate();
                CAL_OPT = 0;
            }
            else
            {
                //l_nSamples++;
                for(uint8 i = 0; i < 8; i++)
                {
                    Cal_Vreal[i] = (int16)(zclSmartMeter_map((int16)senValue[i], (int16)MIN_ADC,
                                           (int16)MAX_ADC, (int16)((float)(MIN_V) * sqrt(2) * (-1)), (int16)((float)MAX_V * sqrt(2)))); //map to real voltage
                    rmsTemp_calV[i] += (Cal_Vreal[i] * Cal_Vreal[i]);
                }
            }
        }

        else if(CAL_OPT == CAL_CUR)
        {
            sprintf((char *)lcdString, "sample: %d", l_nSamples );
            HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

            if (l_nSamples == N_CAL * 4)
            {
                time_new = osal_GetSystemClock();

                sprintf((char *)lcdString, "time2: %d", (time_new - time_old) );
                HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
                time_old = time_new;
                for(uint8 i = 0; i < 8; i++)
                {
                    CRMS_I[i] = (uint16)sqrt(rmsTemp_calI[i] / l_nSamples); //get RMS voltage
                    rmsTemp_calI[i] = 0;
                    MAG_I[i] = (uint16)(I_CAL * 10000 / CRMS_I[i]);
                }

                l_nSamples = 0;
                zclSmartMeter_calWriteParam();
                // send the calibration value over the air to Coordinator
                zclSmartMeter_SendCalibrate();
                CAL_OPT = 0;
            }
            else
            {
                //l_nSamples++;
                for(uint8 i = 0; i < 8; i++)
                {
                    Cal_Ireal[i] = (int16)(zclSmartMeter_map((int16)senValue[i], (int16)MIN_ADC, (int16)MAX_ADC,
                                           (int16)((float)(MIN_I * 100) * sqrt(2) * (-1) ), (int16)(MAX_I * 100 * sqrt(2)))); //map to real current, mag by MAG
                    rmsTemp_calI[i] += (Cal_Ireal[i] * Cal_Ireal[i]);
                }
            }
        }
*/
        else if(CAL_OPT == CAL_GEN_1)
        {
            //sprintf((char *)lcdString, "samplegen1: %d", l_nSamples );
            //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

            if (l_nSamples == N_CAL * 4)
            {
                time_new = osal_GetSystemClock();

                //sprintf((char *)lcdString, "timeGEN1: %d", (time_new - time_old) );
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
                time_old = time_new;
                for(uint8 i = 0; i < 8; i++)
                {
                    MAG_GEN_INPUT1[i] = (uint16)((uint32)INPUT_1_CAL * 100 * l_nSamples / accu_GEN_INPUT1[i]);
                    accu_GEN_INPUT1[i] = 0;
                }

                l_nSamples = 0;
                zclSmartMeter_calWriteParam();
                // send the calibration value over the air to Coordinator
                zclSmartMeter_SendCalibrate();
                CAL_OPT = 0;
            }
            else
            {
                //l_nSamples++;
                for(uint8 i = 0; i < 8; i++)
                {
                    REAL_GEN_INPUT1[i] = senValue[i];
                    accu_GEN_INPUT1[i] += REAL_GEN_INPUT1[i];
                }
            }
        }

        else if(CAL_OPT == CAL_GEN_2)
        {
            //sprintf((char *)lcdString, "samplegen2: %d", l_nSamples );
            //HalLcdWriteString( lcdString, HAL_LCD_LINE_3 );

            if (l_nSamples == N_CAL * 4)
            {
                time_new = osal_GetSystemClock();

                //sprintf((char *)lcdString, "timeGEN2: %d", (time_new - time_old) );
                //HalLcdWriteString( lcdString, HAL_LCD_LINE_6 );
                time_old = time_new;
                for(uint8 i = 0; i < 8; i++)
                {
                    MAG_GEN_INPUT2[i] = (uint16)((uint32)INPUT_2_CAL * 100 * l_nSamples / accu_GEN_INPUT2[i]);
                    accu_GEN_INPUT2[i] = 0;
                }

                l_nSamples = 0;
                zclSmartMeter_calWriteParam();
                // send the calibration value over the air to Coordinator
                zclSmartMeter_SendCalibrate();
                CAL_OPT = 0;
            }
            else
            {
                //l_nSamples++;
                for(uint8 i = 0; i < 8; i++)
                {
                    REAL_GEN_INPUT2[i] = senValue[i];
                    accu_GEN_INPUT2[i] += REAL_GEN_INPUT2[i];
                }
            }
        }

        //else
        //    time_new = osal_GetSystemClock();

    }
    else if (flaginc == 1 && start == 1)   //check if the start value is 1(start/stop power calculation flag)
    {
        //sprintf((char *)lcdString, "s: %d %d %d %d %d", Num_phase[0], Num_phase[1], Num_phase[2], Num_phase[3], len_DataReg );
        //HalLcdWriteString( lcdString, HAL_LCD_LINE_4 );

        for(uint8 i = 0; i < 8; i++)
            phase_dec_flag[i] = 1;

        enecal_timenew = osal_GetSystemClock();
        enecal_timeperiod = enecal_timenew - enecal_timeold;
        enecal_timeold = enecal_timenew;

        flaginc = 0;

        uint8 j = 0;
        uint8 k = 0;  //data Reg count
        uint8 count_MAG = 0;  //count MAG_V and MAG_I
        if(Num_phase[1])
        {
            uint8 i = 0;
            for(i = 0; i < FFT_N; i++)
            {
                x[i].real = VOI_sample[count_MAG * FFT_N + i];
                x[i].img = 0;
            }           

            initW();    
            fft();  	           
            
            double compare[FFT_N] = {0};
            uint8 max_index = 0;
            //for(i = 0; i < FFT_N; i++)
            for(i = 1; i <= FFT_N/2; i++) //IGNORE DC PART
            {
                compare[i] = x[i].real * x[i].real + x[i].img * x[i].img;
                
                if(compare[i] > compare[max_index])
                        max_index = i;
            }
            
            result_PHASE[0] = (uint16)(atan2 (x[max_index].img, x[max_index].real) * 180 / PI + 180);  // 0~360
            
            
            RMS_V1 = (uint16)((sqrt(rmsTemp_V1 / l_nSamples + SUM_SQUR_LIMIT / l_nSamples * overflow_num_V1) * MAG_V[count_MAG++]) / 100); //get RMS voltage
            
           
            
            VIT_dataReg[k++] = RMS_V1;
            
                               
            rmsTemp_V1 = 0;
            overflow_num_V1 = 0;

            for(uint8 i = 0; i < Num_phase[1]; i++)
            {
                /////////////////////////////////////////////////
                uint8 i_fft = 0;
                for(i_fft = 0; i_fft < FFT_N; i_fft++)
                {
                    x[i_fft].real = VOI_sample[count_MAG * FFT_N + i_fft];
                    x[i_fft].img = 0;
                }
                
                initW();    
                fft();  	
                
                double compare[FFT_N] = {0};
                uint8 max_index = 0;
                //for(i_fft = 0; i_fft < FFT_N; i_fft++)
                for(i_fft = 1; i_fft <= FFT_N/2; i_fft++) //IGNORE DC PART
                {
                    compare[i_fft] = x[i_fft].real * x[i_fft].real + x[i_fft].img * x[i_fft].img;
                    
                    if(compare[i_fft] > compare[max_index])
                            max_index = i_fft;
                }                
                
                Theta1[i] = (int16)(result_PHASE[0]) - (int16)(atan2(x[max_index].img, x[max_index].real) * 180 / PI + 180 + (float)7 * (i + 1)) ;  // 0~360  3.4????????+ (float)3.4 * (i + 1)
                                
                if(Theta1[i] < 0)
                    Theta1[i] += 360;

              
                RMS_I1[i] = (uint16)((sqrt(rmsTemp_I1[i] / l_nSamples + SUM_SQUR_LIMIT / l_nSamples * overflow_num_I1[i]) * MAG_I[count_MAG++]) / 100); //get RMS current

                if(RMS_I1[i] < 15)////////////////////////////////////////////////////////
                    power_flag = 0;
                else
                    power_flag = 1;
                
                VIT_dataReg[k++] = RMS_I1[i] * power_flag;

                VIT_dataReg[k++] = Theta1[i] * power_flag;

                powerVal[j] = RMS_V1 * RMS_I1[i];                     //get power
                energyVal[j] += (uint64)(( powerVal[j] * enecal_timeperiod * TIMEINDEX ));  //energy in W.s  * 100000
                Energy[j] = (double)((double)energyVal[j] / 1000 / 3600 / 100000); //energy magnified in kWh
                CurDisplay[j] = (float)((float)(RMS_I1[i] ) / 100);
                PowerDisplay[j] = (float)((float)(powerVal[j]) / 100);
                j++;

                rmsTemp_I1[i] = 0;
                overflow_num_I1[i] = 0;
            }
        }	

        if(Num_phase[2])
        {
            /////////////////////////////////////////////////
            uint8 i = 0;
            for(i = 0; i < FFT_N; i++)
            {
                x[i].real = VOI_sample[count_MAG * FFT_N + i];
                x[i].img = 0;
            }           

            initW();    
            fft();  	            
            
            double compare[FFT_N] = {0};
            uint8 max_index = 0;
            //for(i = 0; i < FFT_N; i++)
            for(i = 1; i <= FFT_N/2; i++) //IGNORE DC PART
            {
                compare[i] = x[i].real * x[i].real + x[i].img * x[i].img;
                
                if(compare[i] > compare[max_index])
                        max_index = i;
            }
            
            result_PHASE[1] = (uint16)(atan2 (x[max_index].img, x[max_index].real) * 180 / PI + 180);  // 0~360

            ///0928
            RMS_V2 = (uint16)((sqrt((rmsTemp_V2 / l_nSamples + SUM_SQUR_LIMIT / l_nSamples * overflow_num_V2)-(uint32)((abs(totalTemp_V2) / l_nSamples)*(abs(totalTemp_V2) / l_nSamples))) * MAG_V[count_MAG++]) / 100); //get RMS voltage
                       
            
            VIT_dataReg[k++] = RMS_V2;

            rmsTemp_V2 = 0;
            overflow_num_V2 = 0;
	    totalTemp_V2 = 0; ///0928

            for(uint8 i = 0; i < Num_phase[2]; i++)
            {
                /////////////////////////////////////////////////
                uint8 i_fft = 0;
                for(i_fft = 0; i_fft < FFT_N; i_fft++)
                {
                    x[i_fft].real = VOI_sample[count_MAG * FFT_N + i_fft];
                    x[i_fft].img = 0;
                }
                
                initW();    
                fft();  	
                
                double compare[FFT_N] = {0};
                uint8 max_index = 0;
                //for(i_fft = 0; i_fft < FFT_N; i_fft++)
                for(i_fft = 1; i_fft <= FFT_N/2; i_fft++) //IGNORE DC PART
                {
                    compare[i_fft] = x[i_fft].real * x[i_fft].real + x[i_fft].img * x[i_fft].img;
                    
                    if(compare[i_fft] > compare[max_index])
                            max_index = i_fft;
                }
                
                
                Theta2[i] = (int16)(result_PHASE[1]) - (int16)(atan2(x[max_index].img, x[max_index].real) * 180 / PI + 180 + (float)7 * (i + 1)) ;  // 0~360  3.4????????+ (float)3.4 * (i + 1)
                
                
                if(Theta2[i] < 0)
                    Theta2[i] += 360;
              
			    ///0928
                RMS_I2[i] = (uint16)((sqrt((double)rmsTemp_I2[i] / l_nSamples + (double)SUM_SQUR_LIMIT / l_nSamples * overflow_num_I2[i] -(uint32)((abs(totalTemp_I2[i]) / l_nSamples)*(abs(totalTemp_I2[i]) / l_nSamples))) * MAG_I[count_MAG++]) / 100); //get RMS current

                if(RMS_I2[i] < 15)////////////////////////////////////////////////////////
                    power_flag = 0;
                else
                    power_flag = 1;
                
                VIT_dataReg[k++] = RMS_I2[i] * power_flag;

                VIT_dataReg[k++] = Theta2[i] * power_flag;

                powerVal[j] = RMS_V2 * RMS_I2[i];                     //get power
                energyVal[j] += (uint64)(( powerVal[j] * enecal_timeperiod * TIMEINDEX ));  //energy in W.s  * 100000
                Energy[j] = (double)((double)energyVal[j] / 1000 / 3600 / 100000); //energy magnified in kWh
                CurDisplay[j] = (float)((float)(RMS_I2[i] ) / 100);
                PowerDisplay[j] = (float)((float)(powerVal[j]) / 100);
                j++;

                rmsTemp_I2[i] = 0;
		totalTemp_I2[i] = 0;////0928
                overflow_num_I2[i] = 0;
            }
        }

        if(Num_phase[3])
        {
            uint8 i = 0;
            for(i = 0; i < FFT_N; i++)
            {
                x[i].real = VOI_sample[count_MAG * FFT_N + i];
                x[i].img = 0;
            }           

            initW();    
            fft();  	           
            
            double compare[FFT_N] = {0};
            uint8 max_index = 0;
            //for(i = 0; i < FFT_N; i++)
            for(i = 1; i <= FFT_N/2; i++) //IGNORE DC PART
            {
                compare[i] = x[i].real * x[i].real + x[i].img * x[i].img;
                
                if(compare[i] > compare[max_index])
                        max_index = i;
            }
            
            result_PHASE[2] = (uint16)(atan2 (x[max_index].img, x[max_index].real) * 180 / PI + 180);  // 0~360
            
            
            RMS_V3 = (uint16)((sqrt(rmsTemp_V3 / l_nSamples + SUM_SQUR_LIMIT / l_nSamples * overflow_num_V3) * MAG_V[count_MAG++]) / 100); //get RMS voltage
            
           
            
            VIT_dataReg[k++] = RMS_V3;
            
                               
            rmsTemp_V3 = 0;
            overflow_num_V3 = 0;

            for(uint8 i = 0; i < Num_phase[3]; i++)
            {
                /////////////////////////////////////////////////
                uint8 i_fft = 0;
                for(i_fft = 0; i_fft < FFT_N; i_fft++)
                {
                    x[i_fft].real = VOI_sample[count_MAG * FFT_N + i_fft];
                    x[i_fft].img = 0;
                }
                
                initW();    
                fft();  	
                
                double compare[FFT_N] = {0};
                uint8 max_index = 0;
                //for(i_fft = 0; i_fft < FFT_N; i_fft++)
                for(i_fft = 1; i_fft <= FFT_N/2; i_fft++) //IGNORE DC PART
                {
                    compare[i_fft] = x[i_fft].real * x[i_fft].real + x[i_fft].img * x[i_fft].img;
                    
                    if(compare[i_fft] > compare[max_index])
                            max_index = i_fft;
                }                
                
                Theta3[i] = (int16)(result_PHASE[2]) - (int16)(atan2(x[max_index].img, x[max_index].real) * 180 / PI + 180 + (float)7 * (i + 1)) ;  // 0~360  3.4????????+ (float)3.4 * (i + 1)
                                
                if(Theta3[i] < 0)
                    Theta3[i] += 360;

              
                RMS_I3[i] = (uint16)((sqrt(rmsTemp_I3[i] / l_nSamples + SUM_SQUR_LIMIT / l_nSamples * overflow_num_I3[i]) * MAG_I[count_MAG++]) / 100); //get RMS current

                if(RMS_I3[i] < 15)////////////////////////////////////////////////////////
                    power_flag = 0;
                else
                    power_flag = 1;
                
                VIT_dataReg[k++] = RMS_I3[i] * power_flag;

                VIT_dataReg[k++] = Theta3[i] * power_flag;

                powerVal[j] = RMS_V3 * RMS_I3[i];                     //get power
                energyVal[j] += (uint64)(( powerVal[j] * enecal_timeperiod * TIMEINDEX ));  //energy in W.s  * 100000
                Energy[j] = (double)((double)energyVal[j] / 1000 / 3600 / 100000); //energy magnified in kWh
                CurDisplay[j] = (float)((float)(RMS_I3[i] ) / 100);
                PowerDisplay[j] = (float)((float)(powerVal[j]) / 100);
                j++;

                rmsTemp_I3[i] = 0;
                overflow_num_I3[i] = 0;
            }
        }		

        if(Num_phase[0])
        {
            for (uint8 i = 0;  i < Num_phase[0]; i++)
            {
                double Temp;
                uint16 RawADC = 0;



                //RawADC = (uint16)(accu_GEN_INPUT1[i] * MAG_GEN_INPUT1[count_MAG++] / (l_nSamples * 100));
                RawADC = (uint16)(accu_GEN_INPUT1[i]  / l_nSamples);
                uint8 uart0show[6] = {0};
                uart0show[0] = 0x11;
                uart0show[1] = (uint8)((RawADC & 0xff00) >> 8);
                uart0show[2] = (uint8)(RawADC & 0x00ff);
                uart0show[3] = (uint8)((senValue[7] & 0xff00) >> 8);
                uart0show[4] = (uint8)(senValue[7] & 0x00ff);
                uart0show[5] = 0x88;
                //HalUART0Write ( HAL_UART_PORT_0, uart0show, 6);

                Temp = log(10000.0 * ((1024.0 / RawADC - 1)));
                Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
                Temp = Temp - 273.15;
                VIT_dataReg[k++] = (uint16)Temp;
                accu_GEN_INPUT1[i] = 0;
            }

        }

        if(Num_phase[4])
        {
            for (uint8 i = 0;  i < Num_phase[4]; i++)
            {
                VIT_dataReg[k++] = (uint16)(accu_GEN_INPUT2[i] * MAG_GEN_INPUT2[count_MAG++] / (l_nSamples * 100));
                accu_GEN_INPUT2[i] = 0;
            }

        }

        //len_DataReg = k;
        l_nSamples = 0;
        enecal_timeperiod = 0;

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
    return((int32)((float)(senValue * (MAX_PEAK - MIN_PEAK)) / MAX_ADC) + MIN_PEAK);
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
/*
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
*/
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


    //GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_0);
    //GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_1);
    
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_7);
    
    //GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_3);
    //GPIOPinTypeGPIOInput(GPIO_A_BASE, GPIO_PIN_4);

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
    for (i = 0; i < 30; i++)
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
    lcdBufferPrintIntAligned(0, RMS_V2, eLcdAlignCenter, eLcdPage2);
    lcdBufferPrintStringAligned(0, "V", eLcdAlignRight, eLcdPage2);
    //Write RMS_I to default buffer
    lcdBufferPrintStringAligned(0, "Current:", eLcdAlignLeft, eLcdPage3);
    lcdBufferPrintFloatAligned(0, CurDisplay[4], 2, eLcdAlignCenter, eLcdPage3);
    lcdBufferPrintStringAligned(0, "A", eLcdAlignRight, eLcdPage3);
    //Write powerVal to default buffer
    lcdBufferPrintStringAligned(0, "Power:", eLcdAlignLeft, eLcdPage4);
    lcdBufferPrintFloatAligned(0, PowerDisplay[4], 2, eLcdAlignCenter, eLcdPage4);
    lcdBufferPrintStringAligned(0, "W", eLcdAlignRight, eLcdPage4);
    //Write EnergyVal to default buffer
    lcdBufferPrintStringAligned(0, "Energy:", eLcdAlignLeft, eLcdPage5);
    lcdBufferPrintFloatAligned(0, Energy[4], 6, eLcdAlignCenter, eLcdPage5);
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
    /*
    IOCPinConfigPeriphOutput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_TXD,
                             IOC_MUX_OUT_SEL_UART0_TXD);
    GPIOPinTypeUARTOutput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_TXD);
    IOCPinConfigPeriphInput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_RXD,
                            IOC_UARTRXD_UART0);
    GPIOPinTypeUARTInput(SMARTMETER_GPIO_UART_BASE, SMARTMETER_PIN_UART_RXD);
    // Initialize the UART (UART0) for console I/O.
    UARTStdioInit(0);
    */
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

static uint8 recognise_sm_id(void)
{

    if(ADD_3 == 0x0012 && ADD_2 == 0x4B00 && ADD_1 == 0x040F && ADD_0 == 0x1A3C)
        return 0x00;
    else if(ADD_3 == 0x0012 && ADD_2 == 0x4B00 && ADD_1 == 0x040F && ADD_0 == 0x1C77)
        return 0x01;
    else if(ADD_3 == 0x0012 && ADD_2 == 0x4B00 && ADD_1 == 0x040E && ADD_0 == 0xF19E)
        return 0x02;
    else if(ADD_3 == 0x0012 && ADD_2 == 0x4B00 && ADD_1 == 0x040F && ADD_0 == 0x05B3)
        return 0x03;

    else
        return 0xff;
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
     HalLcdWriteString ( "CB ENTERED", HAL_LCD_LINE_4 );
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
                HalLcdWriteString ( pStr, HAL_LCD_LINE_4 );
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
                HalLcdWriteString ( pStr, HAL_LCD_LINE_4 );
            }
        }
#endif  // LCD_SUPPORTED
        osal_start_timerEx( zclSmartMeter_TaskID, SmartMeter_ADC_SEND_EVT, 2000 );
    }
}
#endif // ZCL_EZMODE

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

    AESLoadKey((uint8_t*)pui8Key, ui8KeyLocation);
    AESECBStart(pui8Buf, pui8Buf, ui8KeyLocation, ui8Encrypt, false);
    
    //
    // wait for completion of the operation
    //
    do
    {
        ASM_NOP;
    }while(!(AESECBCheckResult()));
    
    AESECBGetResult();

    return (AES_SUCCESS);
}
/****************************************************************************
****************************************************************************/