/**************************************************************************************************
  Filename:       zcl_SmartMeter_data.c
  Revised:        $Date: 2013-10-18 11:49:27 -0700 (Fri, 18 Oct 2013) $
  Revision:       $Revision: 35718 $
  Description:    This device will act as a smart meter.
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
#include "zcl_ms.h"
#include "zcl_ezmode.h"
#include "zcl_smartmeter.h"
/*********************************************************************
 * CONSTANTS
 */
#define SmartMeter_DEVICE_VERSION     0
#define SmartMeter_FLAGS              0
#define SmartMeter_HWVERSION          1
#define SmartMeter_ZCLVERSION         1
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
const uint8 zclSmartMeter_HWRevision = SmartMeter_HWVERSION;
const uint8 zclSmartMeter_ZCLVersion = SmartMeter_ZCLVERSION;
const uint8 zclSmartMeter_ManufacturerName[] = { 16, 'I','T','U',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSmartMeter_ModelId[] = { 16, 'I','T','U','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSmartMeter_DateCode[] = { 16, '2','0','1','4','0','5','1','3',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSmartMeter_PowerSource = POWER_SOURCE_MAINS_1_PHASE;
uint8 zclSmartMeter_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclSmartMeter_PhysicalEnvironment = 0;
uint8 zclSmartMeter_DeviceEnable = DEVICE_ENABLED;
// Identify Cluster
uint16 zclSmartMeter_IdentifyTime = 0;
// Smart meter Clusters
uint16 zclSmartMeter_Data_MeasuredValue =0;
uint16 zclSmartMeter_Add_MeasuredValue =0;
uint16 zclSmartMeter_Com_MeasuredValue =0;
/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclSmartMeter_Attrs[SmartMeter_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSmartMeter_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSmartMeter_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSmartMeter_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSmartMeter_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclSmartMeter_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartMeter_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartMeter_DeviceEnable
    }
  },
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartMeter_IdentifyTime
    }
  },
// *** Data Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_DATA_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_DATA_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter_Data_MeasuredValue
    }
  },  
//  *** Address Attributes
  {
    ZCL_CLUSTER_ID_MS_ADD_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_ADD_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter_Add_MeasuredValue
    }
  },     
// *** Power Reset Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_COM_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_COM_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartMeter_Com_MeasuredValue
    }
  },  
 // *** Relay Attriubtes ***
};
/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
#define ZCLSmartMeter_MAX_INCLUSTERS       3
const cId_t zclSmartMeter_InClusterList[ZCLSmartMeter_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
};
#define ZCLSmartMeter_MAX_OUTCLUSTERS       1
const cId_t zclSmartMeter_OutClusterList[ZCLSmartMeter_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};
SimpleDescriptionFormat_t zclSmartMeter_SimpleDesc =
{
  SmartMeter_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId[2];
  ZCL_HA_DEVICEID_DIMMABLE_LIGHT,        //  uint16 AppDeviceId[2];
  SmartMeter_DEVICE_VERSION,            //  int   AppDevVer:4;
  SmartMeter_FLAGS,                     //  int   AppFlags:4;
  ZCLSmartMeter_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSmartMeter_InClusterList, //  byte *pAppInClusterList;
  ZCLSmartMeter_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSmartMeter_OutClusterList //  byte *pAppInClusterList;
};
/*********************************************************************
 * GLOBAL FUNCTIONS
 */
/*********************************************************************
 * LOCAL FUNCTIONS
 */
/****************************************************************************
****************************************************************************/