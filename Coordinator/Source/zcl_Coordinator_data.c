/**************************************************************************************************
  Filename:       zcl_Coordinator_data.c
  Revised:        $Date: 2013-10-18 17:02:21 -0700 (Fri, 18 Oct 2013) $
  Revision:       $Revision: 35724 $


  Description:    Zigbee Cluster Library - sample device application.


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

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_poll_control.h"
#include "zcl_electrical_measurement.h"
#include "zcl_diagnostic.h"
#include "zcl_meter_identification.h"
#include "zcl_appliance_identification.h"
#include "zcl_appliance_events_alerts.h"
#include "zcl_power_profile.h"
#include "zcl_appliance_control.h"
#include "zcl_appliance_statistics.h"
#include "zcl_hvac.h"
#include "zcl_ezmode.h"

#include "zcl_Coordinator.h"

/*********************************************************************
 * CONSTANTS
 */

#define Coordinator_DEVICE_VERSION     0
#define Coordinator_FLAGS              0

#define Coordinator_HWVERSION          1
#define Coordinator_ZCLVERSION         1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Basic Cluster
const uint8 zclCoordinator_HWRevision = Coordinator_HWVERSION;
const uint8 zclCoordinator_ZCLVersion = Coordinator_ZCLVERSION;
const uint8 zclCoordinator_ManufacturerName[] = { 16, 'T','e','x','a','s','I','n','s','t','r','u','m','e','n','t','s' };
const uint8 zclCoordinator_ModelId[] = { 16, 'T','I','0','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclCoordinator_DateCode[] = { 16, '2','0','0','6','0','8','3','1',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclCoordinator_PowerSource = POWER_SOURCE_MAINS_1_PHASE;

uint8 zclCoordinator_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclCoordinator_PhysicalEnvironment = 0;
uint8 zclCoordinator_DeviceEnable = DEVICE_ENABLED;

// Identify Cluster
uint16 zclCoordinator_IdentifyTime = 0;

// On/Off Cluster
uint8  zclCoordinator_OnOff = COMMAND_OFF;

// HVAC Thermostat Cluster
int16 zclCoordinator_LocalTemperature = NULL;
int16 zclCoordinator_MinHeatSetpointLimit = 1700;  // 17.00C
int16 zclCoordinator_MaxHeatSetpointLimit = 2700;  // 27.00C
int16 zclCoordinator_MinCoolSetpointLimit = 1700;  // 17.00C
int16 zclCoordinator_MaxCoolSetpointLimit = 2700;  // 27.00C
int16 zclCoordinator_OccupiedHeatingSetpoint = 2000; // 20.00C
int16 zclCoordinator_OccupiedCoolingSetpoint = 2400; // 24.00C
uint8 zclCoordinator_HeatingDemand = 100;   // 100% heating demanded of heating device
uint8 zclCoordinator_CoolingDemand = 100;   // 100% cooling demanded of cooling device
uint8 zclCoordinator_ControlSequenceOfOperation = HVAC_THERMOSTAT_CTRL_SEQ_OF_OPER_COOLING_HEATING;    // Both heating and cooling is possible
uint8 zclCoordinator_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_OFF;

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclCoordinator_Attrs[Coordinator_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclCoordinator_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclCoordinator_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclCoordinator_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclCoordinator_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclCoordinator_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclCoordinator_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclCoordinator_DeviceEnable
    }
  },

  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclCoordinator_IdentifyTime
    }
  },

  // *** On/Off Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_OnOff
    }
  },

  // *** HVAC Thermostat Cluster Attributes *** //
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_OCCUPIED_COOLING_SETPOINT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclCoordinator_OccupiedCoolingSetpoint
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_OCCUPIED_HEATING_SETPOINT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclCoordinator_OccupiedHeatingSetpoint
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_LOCAL_TEMPERATURE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_LocalTemperature
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_MinHeatSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_MaxHeatSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_MinCoolSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_MaxCoolSetpointLimit
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_PI_COOLING_DEMAND,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_CoolingDemand
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_PI_HEATING_DEMAND,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclCoordinator_HeatingDemand
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_CTRL_SEQ_OF_OPER,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclCoordinator_ControlSequenceOfOperation
    }
  },
  {
    ZCL_CLUSTER_ID_HVAC_THERMOSTAT,
    { // Attribute record
      ATTRID_HVAC_THERMOSTAT_SYSTEM_MODE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE,
      (void *)&zclCoordinator_SystemMode
    }
  },
  // *** Parameter Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_PARAMETER_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter__Parameter_MeasuredValue
    }
  },  
  
// *** Data Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_DATA_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter__Data_MeasuredValue
    }
  },    
// *** Power Reset Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_RESET_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_RESET_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter__Reset_MeasuredValue
    }
  },  
 // *** Relay Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_RELAY_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_RELAY_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter__Relay_MeasuredValue
    }
  },     
// *** Start Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_START_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_START_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter__Start_MeasuredValue
    }
  },    
// *** Start Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_ACK_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_ACK_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter__Ack_MeasuredValue
    }
  },        
};

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
#define ZCLCoordinator_MAX_INCLUSTERS       3
const cId_t zclCoordinator_InClusterList[ZCLCoordinator_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_HVAC_THERMOSTAT
};

#define ZCLCoordinator_MAX_OUTCLUSTERS       1
const cId_t zclCoordinator_OutClusterList[ZCLCoordinator_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
    
};

SimpleDescriptionFormat_t zclCoordinator_SimpleDesc =
{
  Coordinator_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                          //  uint16 AppProfId[2];
  ZCL_HA_DEVICEID_DIMMABLE_LIGHT,             //  uint16 AppDeviceId[2];
  Coordinator_DEVICE_VERSION,            //  int   AppDevVer:4;
  Coordinator_FLAGS,                     //  int   AppFlags:4;
  ZCLCoordinator_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclCoordinator_InClusterList, //  byte *pAppInClusterList;
  ZCLCoordinator_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclCoordinator_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/


