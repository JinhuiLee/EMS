/**************************************************************************************************
  Filename:       zcl_Coordinator_data.c
  Revised:        $Date: 2013-10-18 17:02:21 -0700 (Fri, 18 Oct 2013) $
  Revision:       $Revision: 35724 $
  Description:    This device will act as a Coordinator.
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
const uint8 zclCoordinator_ManufacturerName[] = { 16, 'I','T','U',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclCoordinator_ModelId[] = { 16, 'I','T','U','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclCoordinator_DateCode[] = { 16, '2','0','1','4','0','5','1','3',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclCoordinator_PowerSource = POWER_SOURCE_MAINS_1_PHASE;
uint8 zclCoordinator_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclCoordinator_PhysicalEnvironment = 0;
uint8 zclCoordinator_DeviceEnable = DEVICE_ENABLED;
// Identify Cluster
uint16 zclCoordinator_IdentifyTime = 0;

// HVAC Thermostat Cluster
int16 zclCoordinator_LocalTemperature = NULL;
uint8 zclCoordinator_ControlSequenceOfOperation = HVAC_THERMOSTAT_CTRL_SEQ_OF_OPER_COOLING_HEATING;    // Both heating and cooling is possible
uint8 zclCoordinator_SystemMode = HVAC_THERMOSTAT_SYSTEM_MODE_OFF;        
uint8 zclCoordinator_Data_MeasuredValue = 0;
uint8 zclCoordinator_Add_MeasuredValue = 0;
uint8 zclCoordinator_Com_MeasuredValue = 0;
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

  // *** HVAC Thermostat Cluster Attributes *** //
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
#define ZCLCoordinator_MAX_OUTCLUSTERS       5
const cId_t zclCoordinator_OutClusterList[ZCLCoordinator_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
  // Add for smart meter
  ZCL_CLUSTER_ID_MS_PARAMETER_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_COM_MEASUREMENT
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