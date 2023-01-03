/**************************************************************************************************
  Filename:       zcl_SmartSwitch_data.c
  Revised:        $Date: 2014-05-12 13:14:02 -0700 (Mon, 12 May 2014) $
  Revision:       $Revision: 38502 $


  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "zcl_ms.h"
/* SMARTSWITCH_TODO: Include any of the header files below to access specific cluster data
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
*/

#include "zcl_SmartSwitch.h"

#include "battery.h"

/*********************************************************************
 * CONSTANTS
 */

#define SMARTSWITCH_DEVICE_VERSION     1
#define SMARTSWITCH_FLAGS              0

#define SMARTSWITCH_HWVERSION          1
#define SMARTSWITCH_ZCLVERSION         1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
bool zclSmartSwitch_OnOff[4] = {SW_OFF};
uint8 zclSmartSwitch_PowerOnBehavior[4] = {SW_OFF};
uint8 zclSmartSwitch_ErorRuntime[4] = {false};
// Global attributes
const uint16 zclSmartSwitch_clusterRevision_all = 0x0001; 

// Basic Cluster
const uint8 zclSmartSwitch_HWRevision = SMARTSWITCH_HWVERSION;
const uint8 zclSmartSwitch_ZCLVersion = SMARTSWITCH_ZCLVERSION;
const uint8 zclSmartSwitch_ManufacturerName[] = { 9, 'W', 'a', 'r', 'm', 'H', 'o', 'u', 's', 'e' };
const uint8 zclSmartSwitch_ModelId[] = { 10, 'W', 'H', '_', 'S', 'W', 'I', 'T', 'C', 'H', '4'};
const uint8 zclSmartSwitch_DateCode[] = { 8, '2','0','2','2','1','2','2','8' };
const uint8 zclSmartSwitch_PowerSource = POWER_SOURCE_MAINS_1_PHASE;

uint8 zclSmartSwitch_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclSmartSwitch_PhysicalEnvironment = 0;
uint8 zclSmartSwitch_DeviceEnable = DEVICE_ENABLED;

// Identify Cluster
uint16 zclSmartSwitch_IdentifyTime;

/* SMARTSWITCH_TODO: declare attribute variables here. If its value can change,
 * initialize it in zclSmartSwitch_ResetAttributesToDefaultValues. If its
 * value will not change, initialize it here.
 */

#if ZCL_DISCOVER
CONST zclCommandRec_t zclSmartSwitch_Cmds[] =
{
  {
   BASIC,
    COMMAND_BASIC_RESET_FACT_DEFAULT,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_OFF,
    CMD_DIR_SERVER_RECEIVED
  },
  {
    ZCL_CLUSTER_ID_GEN_ON_OFF,
    COMMAND_ON,
    CMD_DIR_SERVER_RECEIVED
  },
};

CONST uint8 zclCmdsArraySize = ( sizeof(zclSmartSwitch_Cmds) / sizeof(zclSmartSwitch_Cmds[0]) );
#endif // ZCL_DISCOVER

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclSmartSwitch_Attrs0[] =
{
  // *** General Basic Cluster Attributes ***
  {
   BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSmartSwitch_HWRevision  // Pointer to attribute variable
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_ZCLVersion
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSmartSwitch_ManufacturerName
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSmartSwitch_ModelId
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSmartSwitch_DateCode
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_PowerSource
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclSmartSwitch_LocationDescription
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartSwitch_PhysicalEnvironment
    }
  },
  {
   BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartSwitch_DeviceEnable
    }
  },

#ifdef ZCL_IDENTIFY
  // *** Identify Cluster Attribute ***
  {
   IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartSwitch_IdentifyTime
    }
  },
#endif
  {
   BASIC,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_clusterRevision_all
    }
  },
  {
   IDENTIFY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_clusterRevision_all
    }
  },
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_OnOff[0]
    }
  },
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF_ERROR_RUNTIME,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_ErorRuntime[0]
    }
  },
  {
    ZCL_CLUSTER_POWER_ON_BEHAVIOR,
    { // Attribute record
      ATTRID_POWER_ON_BEHAVIOR,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartSwitch_PowerOnBehavior[0]
    }
  }
};

uint8 CONST zclSmartSwitch_NumAttributes0 = ( sizeof(zclSmartSwitch_Attrs0) / sizeof(zclSmartSwitch_Attrs0[0]) );


/*  endpoint 2 */
CONST zclAttrRec_t zclSmartSwitch_Attrs1[] =
{
  // *** General Basic Cluster Attributes ***
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_OnOff[1]
    }
  },
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF_ERROR_RUNTIME,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_ErorRuntime[1]
    }
  },
  {
    ZCL_CLUSTER_POWER_ON_BEHAVIOR,
    { // Attribute record
      ATTRID_POWER_ON_BEHAVIOR,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartSwitch_PowerOnBehavior[1]
    }
  }
};

uint8 CONST zclSmartSwitch_NumAttributes1 = ( sizeof(zclSmartSwitch_Attrs1) / sizeof(zclSmartSwitch_Attrs1[0]) );



/*  endpoint 2 */
CONST zclAttrRec_t zclSmartSwitch_Attrs2[] =
{
  // *** General Basic Cluster Attributes ***
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_OnOff[2]
    }
  },
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF_ERROR_RUNTIME,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_ErorRuntime[2]
    }
  },
  {
    ZCL_CLUSTER_POWER_ON_BEHAVIOR,
    { // Attribute record
      ATTRID_POWER_ON_BEHAVIOR,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartSwitch_PowerOnBehavior[2]
    }
  }
};

uint8 CONST zclSmartSwitch_NumAttributes2 = ( sizeof(zclSmartSwitch_Attrs2) / sizeof(zclSmartSwitch_Attrs2[0]) );



/*  endpoint 3 */
CONST zclAttrRec_t zclSmartSwitch_Attrs3[] =
{
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF,
      ZCL_DATATYPE_BOOLEAN,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_OnOff[3]
    }
  },
  {
  ZCL_CLUSTER_ID_GEN_ON_OFF,
    { // Attribute record
      ATTRID_ON_OFF_ERROR_RUNTIME,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSmartSwitch_ErorRuntime[3]
    }
  },
  {
    ZCL_CLUSTER_POWER_ON_BEHAVIOR,
    { // Attribute record
      ATTRID_POWER_ON_BEHAVIOR,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSmartSwitch_PowerOnBehavior[3]
    }
  }
};

uint8 CONST zclSmartSwitch_NumAttributes3 = ( sizeof(zclSmartSwitch_Attrs3) / sizeof(zclSmartSwitch_Attrs3[0]) );
/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
const cId_t zclSmartSwitch_InClusterList[] =
{
 BASIC,
 IDENTIFY,
 ZCL_CLUSTER_ID_GEN_ON_OFF,
 ZCL_CLUSTER_POWER_ON_BEHAVIOR
//  ZCL_CLUSTER_ID_GEN_ON_OFF
  // SMARTSWITCH_TODO: Add application specific Input Clusters Here. 
  //       See zcl.h for Cluster ID definitions
  
};
#define ZCLSMARTSWITCH_MAX_INCLUSTERS   (sizeof(zclSmartSwitch_InClusterList) / sizeof(zclSmartSwitch_InClusterList[0]))


const cId_t zclSmartSwitch_OutClusterList[] =
{
 BASIC,
  // SMARTSWITCH_TODO: Add application specific Output Clusters Here. 
  //       See zcl.h for Cluster ID definitions
  ZCL_CLUSTER_ID_GEN_ON_OFF,
  ZCL_CLUSTER_POWER_ON_BEHAVIOR
};
#define ZCLSMARTSWITCH_MAX_OUTCLUSTERS  (sizeof(zclSmartSwitch_OutClusterList) / sizeof(zclSmartSwitch_OutClusterList[0]))


SimpleDescriptionFormat_t zclSmartSwitch_SimpleDesc[4] = {
  {
    SMARTSWITCH_ENDPOINT,                  //  int Endpoint;
    ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
    // SMARTSWITCH_TODO: Replace ZCL_HA_DEVICEID_ON_OFF_LIGHT with application specific device ID
    ZCL_HA_DEVICEID_ON_OFF_SWITCH,          //  uint16 AppDeviceId; 
    SMARTSWITCH_DEVICE_VERSION,            //  int   AppDevVer:4;
    SMARTSWITCH_FLAGS,                     //  int   AppFlags:4;
    ZCLSMARTSWITCH_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_InClusterList, //  byte *pAppInClusterList;
    ZCLSMARTSWITCH_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_OutClusterList //  byte *pAppInClusterList;
  },
  {
    SMARTSWITCH_ENDPOINT+1,                  //  int Endpoint;
    ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
    // SMARTSWITCH_TODO: Replace ZCL_HA_DEVICEID_ON_OFF_LIGHT with application specific device ID
    ZCL_HA_DEVICEID_ON_OFF_SWITCH,          //  uint16 AppDeviceId; 
    SMARTSWITCH_DEVICE_VERSION,            //  int   AppDevVer:4;
    SMARTSWITCH_FLAGS,                     //  int   AppFlags:4;
    ZCLSMARTSWITCH_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_InClusterList, //  byte *pAppInClusterList;
    ZCLSMARTSWITCH_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_OutClusterList //  byte *pAppInClusterList;
  },
  {
    SMARTSWITCH_ENDPOINT+2,                  //  int Endpoint;
    ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
    // SMARTSWITCH_TODO: Replace ZCL_HA_DEVICEID_ON_OFF_LIGHT with application specific device ID
    ZCL_HA_DEVICEID_ON_OFF_SWITCH,          //  uint16 AppDeviceId; 
    SMARTSWITCH_DEVICE_VERSION,            //  int   AppDevVer:4;
    SMARTSWITCH_FLAGS,                     //  int   AppFlags:4;
    ZCLSMARTSWITCH_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_InClusterList, //  byte *pAppInClusterList;
    ZCLSMARTSWITCH_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_OutClusterList //  byte *pAppInClusterList;
  },
  {
    SMARTSWITCH_ENDPOINT+3,                  //  int Endpoint;
    ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId;
    // SMARTSWITCH_TODO: Replace ZCL_HA_DEVICEID_ON_OFF_LIGHT with application specific device ID
    ZCL_HA_DEVICEID_ON_OFF_SWITCH,          //  uint16 AppDeviceId; 
    SMARTSWITCH_DEVICE_VERSION,            //  int   AppDevVer:4;
    SMARTSWITCH_FLAGS,                     //  int   AppFlags:4;
    ZCLSMARTSWITCH_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_InClusterList, //  byte *pAppInClusterList;
    ZCLSMARTSWITCH_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
    (cId_t *)zclSmartSwitch_OutClusterList //  byte *pAppInClusterList;
  },
};

// Added to include ZLL Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
bdbTLDeviceInfo_t tlSmartSwitch_DeviceInfo =
{
  SMARTSWITCH_ENDPOINT,                  //uint8 endpoint;
  ZCL_HA_PROFILE_ID,                        //uint16 profileID;
  // SMARTSWITCH_TODO: Replace ZCL_HA_DEVICEID_ON_OFF_LIGHT with application specific device ID
  ZCL_HA_DEVICEID_SIMPLE_SENSOR,          //uint16 deviceID;
  SMARTSWITCH_DEVICE_VERSION,                    //uint8 version;
  SMARTSWITCH_NUM_GRPS                   //uint8 grpIdCnt;
};
#endif

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
  
void zclSmartSwitch_ResetAttributesToDefaultValues(void)
{
  int i;
  
  zclSmartSwitch_LocationDescription[0] = 16;
  for (i = 1; i <= 16; i++)
  {
    zclSmartSwitch_LocationDescription[i] = ' ';
  }
  
  zclSmartSwitch_PhysicalEnvironment = PHY_UNSPECIFIED_ENV;
  zclSmartSwitch_DeviceEnable = DEVICE_ENABLED;
  
#ifdef ZCL_IDENTIFY
  zclSmartSwitch_IdentifyTime = 0;
#endif
  
  /* SMARTSWITCH_TODO: initialize cluster attribute variables. */
}

/****************************************************************************
****************************************************************************/


