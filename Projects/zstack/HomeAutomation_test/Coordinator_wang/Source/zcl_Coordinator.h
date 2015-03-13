/**************************************************************************************************
  Filename:       zcl_Coordinator.h
  Revised:        $Date: 2013-04-22 14:49:05 -0700 (Mon, 22 Apr 2013) $
  Revision:       $Revision: 33994 $
  Description:    This device will act as a Coordinator.
**************************************************************************************************/
#ifndef ZCL_Coordinator_H
#define ZCL_Coordinator_H
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
#define Coordinator_ENDPOINT            8
#define Coordinator_MAX_ATTRIBUTES      15
#define LIGHT_OFF                       0x00
#define WIRELESS_CONNECTION             0x0001
#define WIRED_CONNECTION                0x0002
// Application Events
// The number of events should not be more than 15
#define Coordinator_IDENTIFY_TIMEOUT_EVT     	         0x0001
#define TIME_RUNNING_EVT                                 0x0002
#define Coordinator_EZMODE_TIMEOUT_EVT                   0x0004
#define Coordinator_EZMODE_NEXTSTATE_EVT                 0x0008
//#define Coordinator_UART_EVT                             0x0010
#define Coordinator_USB_EVT                              0x0200
#define Coordinator_WAIT_SERIES_EVT                      0x0020
#define ACK_WAIT_EVT                                     0x0040  
#define ACK_CS_EVT                                       0x0080
#define DATA_CL_EVT                                      0x0100

//branch of ACK_WAIT_EVT, this is not event
#define Coordinator_ProcessParaSet                       0x0041 
#define Coordinator_RelaySet                             0x0042  
#define DATA_WAIT                                        0x0043
#define Network_WAIT                                     0x0044
#define Calibration_WAIT                                 0x0045
#define Coordinator_EnergyResetWait                      0x0046
#define GETCAL_WAIT                                      0x0047

// Application Display Modes
#define THERMOSTAT_MAINMODE         0x00
#define THERMOSTAT_HELPMODE         0x01
#define THERMOSTAT_HEATMODE         0x02
#define THERMOSTAT_COOLMODE         0x03
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
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
 * VARIABLES
 */
extern uint16 paramReg[];
extern uint16 dataReg[];
extern SimpleDescriptionFormat_t zclCoordinator_SimpleDesc;
extern CONST zclAttrRec_t zclCoordinator_Attrs[];
extern uint16 zclCoordinator_IdentifyTime;
extern int16 zclCoordinator_LocalTemperature;
extern uint8 zclCoordinator_ControlSequenceOfOperation;
extern uint8 zclCoordinator_SystemMode;
// Initialization for the task
extern void zclCoordinator_Init( byte task_id );
// Event Process for the task
extern UINT16 zclCoordinator_event_loop( byte task_id, UINT16 events );
/************************************************************e*********
 * FUNCTIONS
 */
/*********************************************************************
*********************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* ZCL_Coordinator_H */
