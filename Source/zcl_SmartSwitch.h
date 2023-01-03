/**************************************************************************************************
  Filename:       zcl_genericapp.h
  Revised:        $Date: 2014-06-19 08:38:22 -0700 (Thu, 19 Jun 2014) $
  Revision:       $Revision: 39101 $

  Description:    This file contains the ZigBee Cluster Library Home
                  Automation Sample Application.


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

#ifndef ZCL_SMARTSWITCH_H
#define ZCL_SMARTSWITCH_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"


// Added to include ZLL Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  #include "zcl_general.h"
  #include "bdb_tlCommissioning.h"
#endif

/*********************************************************************
 * CONSTANTS
 */
#define SMARTSWITCH_ENDPOINT            1
// Added to include ZLL Target functionality
#define SMARTSWITCH_NUM_GRPS            2


// Application Events
//#define SMARTSWITCH_MAIN_SCREEN_EVT          0x0001
//#define SMARTSWITCH_EVT_LONG                 0x0002
//#define SMARTSWITCH_END_DEVICE_REJOIN_EVT    0x0004  
  
/* SMARTSWITCH_TODO: define app events here */
  
#define SMARTSWITCH_REPORT_EVT                 0x0001
#define SMARTSWITCH_READ_SENSORS_EVT           0x0002
#define SMARTSWITCH_CHECK_KEY_RELEASE_EVT        0x0004
#define SMARTSWITCH_OLED_OFF_EVT              0x0008  
#define SMARTSWITCH_FIRST_REPORT_EVT              0x0010  
// Application Display Modes
#define GENERIC_MAINMODE      0x00
#define GENERIC_HELPMODE      0x01
  
#define SMARTSWITCH_END_DEVICE_REJOIN_DELAY 10000

/*********************************************************************
 * MACROS
 */
  
#define BASIC       ZCL_CLUSTER_ID_GEN_BASIC
#define POWER_CFG   ZCL_CLUSTER_ID_GEN_POWER_CFG
#define TEMPERATURE        ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
#define HUMIDITY    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY
//#define PRESSURE    ZCL_CLUSTER_ID_MS_PRESSURE_MEASUREMENT
#define IDENTIFY     ZCL_CLUSTER_ID_GEN_IDENTIFY
#ifdef OUTDOOR_LONG_RANGE
#define SMARTSWITCH_REPORT_DELAY ((uint32) 300000) //5 minutes
#else
#define SMARTSWITCH_REPORT_DELAY ((uint32) 1800000) //1 minute  
#endif  
  
#define SW_OFF                       0x00
#define SW_ON                        0x01
#define SW_PREVIOUS                  0x02
   
#define BEHAVIOR                        0x00
#define STATE_ONOFF                     0x01
  
#define ZCL_CLUSTER_POWER_ON_BEHAVIOR                       0xE001
#define ATTRID_POWER_ON_BEHAVIOR                       0xD010

#define ATTRID_ON_OFF_ERROR_RUNTIME                       0x4004
   
#define NV_SWITCH_STATE_ID          0x04FF
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */

extern bool zclSmartSwitch_OnOff[4];
extern uint8 zclSmartSwitch_PowerOnBehavior[4];
extern uint8 zclSmartSwitch_ErorRuntime[4];

// Added to include ZLL Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  extern bdbTLDeviceInfo_t tlSmartSwitch_DeviceInfo;
#endif

extern SimpleDescriptionFormat_t zclSmartSwitch_SimpleDesc[];

extern CONST zclCommandRec_t zclSmartSwitch_Cmds[];

extern CONST uint8 zclCmdsArraySize;

// attribute list
extern CONST zclAttrRec_t zclSmartSwitch_Attrs0[];
extern CONST zclAttrRec_t zclSmartSwitch_Attrs1[];
extern CONST zclAttrRec_t zclSmartSwitch_Attrs2[];
extern CONST zclAttrRec_t zclSmartSwitch_Attrs3[];
extern CONST uint8 zclSmartSwitch_NumAttributes0;
extern CONST uint8 zclSmartSwitch_NumAttributes1;
extern CONST uint8 zclSmartSwitch_NumAttributes2;
extern CONST uint8 zclSmartSwitch_NumAttributes3;

// Identify attributes
extern uint16 zclSmartSwitch_IdentifyTime;
extern uint8  zclSmartSwitch_IdentifyCommissionState;

// SMARTSWITCH_TODO: Declare application specific attributes here


/*********************************************************************
 * FUNCTIONS
 */
//static void zclSmartSwitch_OnOffCB(uint8);
 /*
  * Initialization for the task
  */
extern void zclSmartSwitch_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSmartSwitch_event_loop( byte task_id, UINT16 events );

/*
 *  Reset all writable attributes to their default values.
 */
extern void zclSmartSwitch_ResetAttributesToDefaultValues(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SMARTSWITCH_H */
