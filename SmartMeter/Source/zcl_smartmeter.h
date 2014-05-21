/**************************************************************************************************
  Filename:       zcl_SmartMeter.h
  Revised:        $Date: 2013-04-22 14:49:05 -0700 (Mon, 22 Apr 2013) $
  Revision:       $Revision: 33994 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.



**************************************************************************************************/

#ifndef ZCL_SmartMeter_H
#define ZCL_SmartMeter_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
#define SmartMeter_ENDPOINT            8

#define SmartMeter_MAX_ATTRIBUTES      17

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define SmartMeter_IDENTIFY_TIMEOUT_EVT		     0x0001
#define SmartMeter_EZMODE_TIMEOUT_EVT           0x0002
#define SmartMeter_EZMODE_NEXTSTATE_EVT         0x0004
#define SmartMeter_MAIN_SCREEN_EVT              0x0008
#define SmartMeter_TEMP_SEND_EVT                0x0010

#define SmartMeter_ADC_SEND_EVT                 0x0012
#define SmartMeter_RESET_SEND_EVT               0x0014
#define SmartMeter_RELAY_SEND_EVT              0x0016
#define SmartMeter_NWKDISCOV_SEND_EVT         0x0018


// Application Display Modes
#define TEMPSENSE_MAINMODE         0x00
#define TEMPSENSE_HELPMODE         0x01

// lhy  
#define ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT            0x0410
#define ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT                 0x0411
#define ZCL_CLUSTER_ID_MS_RESET_MEASUREMENT                0x0412
#define ZCL_CLUSTER_ID_MS_RELAY_MEASUREMENT                0x0413
#define ZCL_CLUSTER_ID_MS_START_MEASUREMENT                0x0414
#define ZCL_CLUSTER_ID_MS_ACK_MEASUREMENT                  0x0415  

#define ZCL_CLUSTER_ID_MS_RESTART_MEASUREMENT              0x0416
#define ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT                  0x0417
#define ZCL_CLUSTER_ID_MS_COM_MEASUREMENT                  0x0418  
  
#define ATTRID_MS_PARAMETER_MEASURED_VALUE                 0x0020 
#define ATTRID_MS_DATA_MEASURED_VALUE                      0x0021
#define ATTRID_MS_RESET_MEASURED_VALUE                     0x0022
#define ATTRID_MS_RELAY_MEASURED_VALUE                     0x0023
#define ATTRID_MS_START_MEASURED_VALUE                     0x0024
#define ATTRID_MS_ACK_MEASURED_VALUE                       0x0025
#define ATTRID_MS_RESTART_MEASURED_VALUE                   0x0026
#define ATTRID_MS_ADD_MEASURED_VALUE                       0x0027
#define ATTRID_MS_COM_MEASURED_VALUE                       0x0028
//  
  
  
// Application Parameter Modes 
// For test purpose only
// To be replaced by osal_snv_write( )

  
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSmartMeter_SimpleDesc;

extern CONST zclAttrRec_t zclSmartMeter_Attrs[];

extern uint8  zclSmartMeter_OnOff;

extern uint16 zclSmartMeter_IdentifyTime;

// Temperature Measurement Cluster
// extern int16 zclSmartMeter_MeasuredValue;
// extern const int16 zclSmartMeter_MinMeasuredValue;
// extern const uint16 zclSmartMeter_MaxMeasuredValue;


/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSmartMeter_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSmartMeter_event_loop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SmartMeter_H */
