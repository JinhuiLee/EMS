/**************************************************************************************************
  Filename:       zcl_Coordinator.h
  Revised:        $Date: 2014-10-24 11:49:27 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 1 $
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
// Application Events
// The number of events should not be more than 15
#define Coordinator_IDENTIFY_TIMEOUT_EVT     	          0x0001
#define Coordinator_EZMODE_TIMEOUT_EVT                    0x0004
#define Coordinator_EZMODE_NEXTSTATE_EVT                  0x0008
#define Coordinator_UART_EVT                              0x0010
#define Coordinator_ProcessParaSet_EVT                    0x0020 
#define Coordinator_RelaySet_EVT                          0x0040  
// Application Display Modes
#define COORDINATOR_MAINMODE         0x00
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
extern SimpleDescriptionFormat_t zclCoordinator_SimpleDesc;
extern CONST zclAttrRec_t zclCoordinator_Attrs[];
extern uint16 zclCoordinator_IdentifyTime;
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
