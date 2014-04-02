/**************************************************************************************************
  Filename:       zcl_Coordinator.h
  Revised:        $Date: 2013-04-22 14:49:05 -0700 (Mon, 22 Apr 2013) $
  Revision:       $Revision: 33994 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#define Coordinator_MAX_ATTRIBUTES      22

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define Coordinator_IDENTIFY_TIMEOUT_EVT     	  0x0001
#define Coordinator_POLL_CONTROL_TIMEOUT_EVT     0x0002
#define Coordinator_EZMODE_TIMEOUT_EVT           0x0004
#define Coordinator_EZMODE_NEXTSTATE_EVT         0x0008
#define Coordinator_MAIN_SCREEN_EVT              0x0010
  
#define  Coordinator_DATA_SEND_EVT                0x0012
#define  Coordinator_NWKDISCOV_SEND_EVT          0x0014
#define Coordinator_RESET_SEND_EVT               0x0018
#define Coordinator_RELAY_SEND_EVT               0x0020
#define Coordinator_PARAM_SEND_EVT               0x0022
#define Coordinator_PARAM_SET_EVT               0x0024
#define Coordinator_CONTROL_SEND_EVT             0x0026
#define Coordinator_RTABLE_SEND_EVT              0x0028
#define Coordinator_CONTROL_SET_EVT             0x0030

// Application Display Modes
#define THERMOSTAT_MAINMODE         0x00
#define THERMOSTAT_HELPMODE         0x01
#define THERMOSTAT_HEATMODE         0x02
#define THERMOSTAT_COOLMODE         0x03

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern unit16 paramReg[];

extern unit16 dataReg[];
   
extern SimpleDescriptionFormat_t zclCoordinator_SimpleDesc;

extern CONST zclAttrRec_t zclCoordinator_Attrs[];

extern uint8  zclCoordinator_OnOff;

extern uint16 zclCoordinator_IdentifyTime;

extern int16 zclCoordinator_OccupiedCoolingSetpoint;

extern int16 zclCoordinator_OccupiedHeatingSetpoint;

extern int16 zclCoordinator_LocalTemperature;

extern int16 zclCoordinator_MinHeatSetpointLimit;

extern int16 zclCoordinator_MaxHeatSetpointLimit;

extern int16 zclCoordinator_MinCoolSetpointLimit;

extern int16 zclCoordinator_MaxCoolSetpointLimit;

extern uint8 zclCoordinator_CoolingDemand;

extern uint8 zclCoordinator_HeatingDemand;

extern uint8 zclCoordinator_ControlSequenceOfOperation;

extern uint8 zclCoordinator_SystemMode;

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclCoordinator_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclCoordinator_event_loop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_Coordinator_H */
