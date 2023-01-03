#include <stdio.h>
#include <stdlib.h>

#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_SYS.h"

#include "nwk_util.h"

#include "zcl_ms.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_diagnostic.h"
#include "zcl_SmartSwitch.h"

#include "bdb.h"
#include "bdb_interface.h"
#include "gp_interface.h"



#if defined ( INTER_PAN )
#if defined ( BDB_TL_INITIATOR )
#include "bdb_touchlink_initiator.h"
#endif // BDB_TL_INITIATOR
#if defined ( BDB_TL_TARGET )
#include "bdb_touchlink_target.h"
#endif // BDB_TL_TARGET
#endif // INTER_PAN

#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
#include "bdb_touchlink.h"
#endif

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

// my library
#include "uart.h"
#include "Debug.h"
#include "bitmasks.h"
#include "delay.h"
#include "hal_i2c.h"

//#include "sht20.h"
//#include "ssd1306.h"
//#include "ssd1306_tests.h"
//#include "BH1750.h"
#include "utils.h"
#include "PCF8574.h"
//#include "battery.h"
#include "commissioning.h"
#include "factory_reset.h"
/*********************************************************************
* MACROS
*/
#define POWER_ON_SENSORS()                                                                                                                 \
    do {                                                                                                                                   \
    } while (0)
#define POWER_OFF_SENSORS()                                                                                                                \
    do {                                                                                                                                   \
    } while (0) 

/*********************************************************************
* CONSTANTS
*/


/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
//extern bool requestNewTrustCenterLinkKey;
byte zclSmartSwitch_TaskID;
//int16 zclSmartSwitch_MeasuredValue;
//afAddrType_t zclSmartSwitch_DstAddr;


/*********************************************************************
* GLOBAL FUNCTIONS
*/
/*********************************************************************
* LOCAL VARIABLES
*/
uint16 SeqNum = 0;

uint8 giGenAppScreenMode = GENERIC_MAINMODE;   // display the main screen mode first

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclSmartSwitch_NwkState = DEV_INIT;

afAddrType_t zclSmartSwitch_BindDstAddr;
byte p0LastState = 0xFF;
byte p0CurrentState = 0xFF;
//static int8 firstReport = 0;
//volatile bool isPressFeatureButton = false;
uint8 stateSwitch[8];
uint8 count = 0;
afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};
/*********************************************************************
* LOCAL FUNCTIONS
*/

//static void zclSmartSwitch_ReadSHT20( void );
//static void zclSmartSwitch_Report(void) ;
//static void zclSmartSwitch_ReadSensors(void);
static void zclSmartSwitch_HandleKeys( byte shift, byte keys );
static void zclSmartSwitch_BasicResetCB( void );
//static void zclSmartSwitch_ProcessIdentifyTimeChange( uint8 endpoint );
static void zclSmartSwitch_BindNotification( bdbBindNotificationData_t *data );
static void zclSmartSwitch_OnOffCB(uint8 cmd);
#if ( defined ( BDB_TL_TARGET ) && (BDB_TOUCHLINK_CAPABILITY_ENABLED == TRUE) )
static void zclSmartSwitch_ProcessTouchlinkTargetEnable( uint8 enable );
#endif

static void zclSmartSwitch_ControlRelay(uint8 relayID, bool state, bool notReport);
static void zclSmartSwitch_ReportSwitchState( uint8 endpoint , bool notReport);
static void zclSmartSwitch_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);

static void zclSmartSwitch_RestoreAttributesFromNV(void);
static void zclSmartSwitch_SaveAttributesToNV(uint8 endpoint, uint8 data, uint8 type);
// app display functions
//static void zclSmartSwitch_LcdDisplayUpdate( void );
#ifdef LCD_SUPPORTED
static void zclSmartSwitch_LcdDisplayMainMode( void );
static void zclSmartSwitch_LcdDisplayHelpMode( void );
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSmartSwitch_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSmartSwitch_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSmartSwitch_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSmartSwitch_ProcessInWriteCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSmartSwitch_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
//#ifdef ZCL_DISCOVER
//static uint8 zclSmartSwitch_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
//static uint8 zclSmartSwitch_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
//static uint8 zclSmartSwitch_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
//#endif

//static void zclSampleApp_BatteryWarningCB( uint8 voltLevel);

/*********************************************************************
* STATUS STRINGS
*/
#ifdef LCD_SUPPORTED
const char sDeviceName[]   = "  Generic App";
const char sClearLine[]    = " ";
const char sSwSmartSwitch[]      = "SW1:GENAPP_TODO";  // SMARTSWITCH_TODO
const char sSwBDBMode[]     = "SW2: Start BDB";
char sSwHelp[]             = "SW4: Help       ";  // last character is * if NWK open
#endif

/*********************************************************************
* ZCL General Profile Callback table
*/
static zclGeneral_AppCallbacks_t zclSmartSwitch_CmdCallbacks =
{
  zclSmartSwitch_BasicResetCB,             // Basic Cluster Reset command
  NULL,                                   // Identify Trigger Effect command
  zclSmartSwitch_OnOffCB,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
* SMARTSWITCH_TODO: Add other callback structures for any additional application specific 
*       Clusters being used, see available callback structures below.
*
*       bdbTL_AppCallbacks_t 
*       zclApplianceControl_AppCallbacks_t 
*       zclApplianceEventsAlerts_AppCallbacks_t 
*       zclApplianceStatistics_AppCallbacks_t 
*       zclElectricalMeasurement_AppCallbacks_t 
*       zclGeneral_AppCallbacks_t 
*       zclGp_AppCallbacks_t 
*       zclHVAC_AppCallbacks_t 
*       zclLighting_AppCallbacks_t 
*       zclMS_AppCallbacks_t 
*       zclPollControl_AppCallbacks_t 
*       zclPowerProfile_AppCallbacks_t 
*       zclSS_AppCallbacks_t  
*
*/

/*********************************************************************
* @fn          zclSmartSwitch_Init
*
* @brief       Initialization function for the zclGeneral layer.
*
* @param       none
*
* @return      none
*/
void zclSmartSwitch_Init( byte task_id )
{
//  P0SEL &= ~(b11111111);                           
//  P1SEL &= ~(b11111111);                          
//  P2SEL &= ~(b00000111);                           
//  P0INP |= b01111101;                           
//  P1INP &= ~(b11111111);                           
//  P2INP &= ~(BV(4)|BV(3)|BV(2)|BV(1)|BV(0));                           
//  P2INP |= BV(7)|BV(6);
//  UART_Init();
  HalI2CInit();
//  requestNewTrustCenterLinkKey = FALSE;
  zclSmartSwitch_TaskID = task_id;
  
  
    // Set destination address to indirect for bindings
//  zclSmartSwitch_BindDstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
//  zclSmartSwitch_BindDstAddr.endPoint = 1;
//  zclSmartSwitch_BindDstAddr.addr.shortAddr = 0;
  
  // This app is part of the Home Automation Profile
  bdb_RegisterSimpleDescriptor( &zclSmartSwitch_SimpleDesc[0] );
  bdb_RegisterSimpleDescriptor( &zclSmartSwitch_SimpleDesc[1] );
  bdb_RegisterSimpleDescriptor( &zclSmartSwitch_SimpleDesc[2] );
  bdb_RegisterSimpleDescriptor( &zclSmartSwitch_SimpleDesc[3] );
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SMARTSWITCH_ENDPOINT, &zclSmartSwitch_CmdCallbacks );
  zclGeneral_RegisterCmdCallbacks( SMARTSWITCH_ENDPOINT+1, &zclSmartSwitch_CmdCallbacks );
  zclGeneral_RegisterCmdCallbacks( SMARTSWITCH_ENDPOINT+2, &zclSmartSwitch_CmdCallbacks );
  zclGeneral_RegisterCmdCallbacks( SMARTSWITCH_ENDPOINT+3, &zclSmartSwitch_CmdCallbacks );
  
  // SMARTSWITCH_TODO: Register other cluster command callbacks here
  
  // Register the application's attribute list
  zcl_registerAttrList( SMARTSWITCH_ENDPOINT, zclSmartSwitch_NumAttributes0, zclSmartSwitch_Attrs0 );
  zcl_registerAttrList( SMARTSWITCH_ENDPOINT+1, zclSmartSwitch_NumAttributes1, zclSmartSwitch_Attrs1 );
  zcl_registerAttrList( SMARTSWITCH_ENDPOINT+2, zclSmartSwitch_NumAttributes2, zclSmartSwitch_Attrs2 );
  zcl_registerAttrList( SMARTSWITCH_ENDPOINT+3, zclSmartSwitch_NumAttributes3, zclSmartSwitch_Attrs3 );
  
  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSmartSwitch_TaskID );
  
//#ifdef ZCL_DISCOVER
//  // Register the application's command list
//  zcl_registerCmdList( SMARTSWITCH_ENDPOINT, zclCmdsArraySize, zclSmartSwitch_Cmds );
//#endif
  
  // Register low voltage NV memory protection application callback
//  RegisterVoltageWarningCB( zclSampleApp_BatteryWarningCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSmartSwitch_TaskID );
  
  bdb_RegisterCommissioningStatusCB( zclSmartSwitch_ProcessCommissioningStatus );
//  bdb_RegisterIdentifyTimeChangeCB( zclSmartSwitch_ProcessIdentifyTimeChange );
  bdb_RegisterBindNotificationCB( zclSmartSwitch_BindNotification );
  
#if ( defined ( BDB_TL_TARGET ) && (BDB_TOUCHLINK_CAPABILITY_ENABLED == TRUE) )
  bdb_RegisterTouchlinkTargetEnableCB( zclSmartSwitch_ProcessTouchlinkTargetEnable );
#endif
  
#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SMARTSWITCH_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );
  
  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif
  
  
#ifdef LCD_SUPPORTED
  HalLcdWriteString ( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif  // LCD_SUPPORTED
  

  
  

//  //  Interrupt begin
//  P0SEL &= ~(BV(1)|BV(7));
//  P0DIR &= ~(BV(1)|BV(7));
//  P0INP |= BV(7); //tri-state
//  P0INP &= ~BV(1);
//  //  P2INP |= BV(5);
//  
//  PICTL |= BV(0); //falling
//  P0IEN |= BV(1)|BV(7);
//  IEN1 |= BV(5);
//  P0IFG = 0;
//  // Interrupt
  
//  P0SEL |= b00001100;                    // 0=GPIO 1=Peripheral (ADC, UART)

//  P1DIR |= BV(0)|BV(1);
//  P1 |= BV(1);

//  P0SEL &= ~(BV(1));
//  P0DIR &= ~(BV(1));
  
//  HalI2CInit();
  
#ifdef PWR_OFF_DETECT  
  // Power off interrupt
  P2SEL &= ~BV(0);
  P2DIR &= (~BV(0));
  P2INP &= ~BV(0);
  P2INP |= BV(7);
  
  PICTL |= BV(3);
  P2IEN |= BV(0);
  IEN2 |= BV(1);
  P2IFG = 0;
#endif
  
  P1SEL |= b11000000;
  UART_Init();
  UART_String("start");
  zclSmartSwitch_RestoreAttributesFromNV();

//  write8(b00001001);
//  BH1750_Init(ONE_TIME_HIGH_RES_MODE);
//  zclBattery_Report();
//  zclSmartSwitch_ReadSHT20();
//  
//  ssd1306_Init();
//  ssd1306_SetContrast(150);
//  ssd1306_DisplaySensorFrame(TemperatureValue, HumidityValue, getConnected(), getBattPercentValue());
//  ssd1306_SetDisplayOn(0);
  //UART_String("start");
  
  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_FORMATION|BDB_COMMISSIONING_MODE_NWK_STEERING| BDB_COMMISSIONING_MODE_FINDING_BINDING);
//  osal_start_reload_timer(zclSmartSwitch_TaskID, SMARTSWITCH_FIRST_REPORT_EVT, 500);
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
uint16 zclSmartSwitch_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSmartSwitch_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
      case ZCL_INCOMING_MSG:
        // Incoming ZCL Foundation command/response messages
        zclSmartSwitch_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
//        if (((zclIncomingMsg_t *)MSGpkt)->attrCmd) {
//          osal_mem_free(((zclIncomingMsg_t *)MSGpkt)->attrCmd);
//        }
        break;
        
      case KEY_CHANGE:
        zclSmartSwitch_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;
        
//      case ZDO_STATE_CHANGE:
//        zclSmartSwitch_NwkState = (devStates_t)(MSGpkt->hdr.status);
//        
//        // now on the network
//        if ( (zclSmartSwitch_NwkState == DEV_ZB_COORD) ||
//            (zclSmartSwitch_NwkState == DEV_ROUTER)   ||
//              (zclSmartSwitch_NwkState == DEV_END_DEVICE) )
//        {
//          giGenAppScreenMode = GENERIC_MAINMODE;
//          //            zclSmartSwitch_LcdDisplayUpdate();
//        }
//        break;
        
      default:
        break;
      }
      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
 
//  if ( events & SMARTSWITCH_REPORT_EVT )
//  {
////    ssd1306_SetDisplayOn(1);
////    zclSmartSwitch_ReportTemp();
//    
////    BH1750_SetMode(ONE_TIME_HIGH_RES_MODE);
////    waitMeasurementReady(true);
////    BH1750_ReadLight(&BH1750_lux);
////    sprintf(BH1750Str, "%d lux", (uint16)(BH1750_lux));
////    //UART_String(BH1750Str);  
////    ssd1306_SetDisplayOn(0);
//    //LREPMaster("SMARTSWITCH_REPORT_EVT\r\n");
//    //UART_String("Report ne");
//    zclSmartSwitch_Report();
//    return ( events ^ SMARTSWITCH_REPORT_EVT );
//  }
//  if (events & SMARTSWITCH_READ_SENSORS_EVT) {
//    //LREPMaster("SMARTSWITCH_READ_SENSORS_EVT\r\n");
//    zclSmartSwitch_ReadSensors();
//    pushBut = true;
//    return (events ^ SMARTSWITCH_READ_SENSORS_EVT);
//  }
  if (events & SMARTSWITCH_CHECK_KEY_RELEASE_EVT) {
//    zclSmartSwitchCheckKeyState(HAL_KEY_P0_INPUT_PINS);
    p0CurrentState = P0;
  
//      //LREP("p0state=0x%X\r\n", p0LastState);
    if ((p0CurrentState & BV(0)) != (p0LastState & BV(0))){  
      OnBoard_SendKeys(BV(0), HAL_KEY_RELEASE | HAL_KEY_PORT0);
    }
    if ((p0CurrentState & BV(1)) != (p0LastState & BV(1))){  
      OnBoard_SendKeys(BV(1), HAL_KEY_RELEASE | HAL_KEY_PORT0);
    }
    if ((p0CurrentState & BV(2)) != (p0LastState & BV(2))){  
      OnBoard_SendKeys(BV(2), HAL_KEY_RELEASE | HAL_KEY_PORT0);
    }
    if ((p0CurrentState & BV(3)) != (p0LastState & BV(3))){  
      OnBoard_SendKeys(BV(3), HAL_KEY_RELEASE | HAL_KEY_PORT0);
    }

    
  
    if (p0LastState == HAL_KEY_P0_INPUT_PINS){
      osal_stop_timerEx(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT);       
    }
    p0LastState = p0CurrentState & HAL_KEY_P0_INPUT_PINS;
    return (events ^ SMARTSWITCH_CHECK_KEY_RELEASE_EVT);
  }
//  if ( events & SMARTSWITCH_OLED_OFF_EVT )
//  {
//    ////UART_String("Oled2");
////    ssd1306_SetDisplayOn(0);
//    return ( events ^ SMARTSWITCH_OLED_OFF_EVT );
//  }
  
  if ( events & SMARTSWITCH_FIRST_REPORT_EVT )
  {
//    switch(zclSmartSwitch_PowerOnBehavior0){
//    case SW_ON:
      UART_String("1 SW_ON");
//      break;
//    case SW_OFF:
//      UART_String("1 SW_OFF");
//      break;
//    case SW_PREVIOUS:
//      UART_String("1 SW_PREVIOUS");
//      break;
//    default:
//      break;
//    }
//    
//    switch(zclSmartSwitch_PowerOnBehavior1){
//    case SW_ON:
//      UART_String("2 SW_ON");
//      break;
//    case SW_OFF:
//      UART_String("2 SW_OFF");
//      break;
//    case SW_PREVIOUS:
//      UART_String("2 SW_PREVIOUS");
//      break;
//    default:
//      break;
//    }
//    
//    switch(zclSmartSwitch_PowerOnBehavior2){
//    case SW_ON:
//      UART_String("3 SW_ON");
//      break;
//    case SW_OFF:
//      UART_String("3 SW_OFF");
//      break;
//    case SW_PREVIOUS:
//      UART_String("3 SW_PREVIOUS");
//      break;
//    default:
//      break;
//    }
//    
//    switch(zclSmartSwitch_PowerOnBehavior3){
//    case SW_ON:
//      UART_String("4 SW_ON");
//      break;
//    case SW_OFF:
//      UART_String("4 SW_OFF");
//      break;
//    case SW_PREVIOUS:
//      UART_String("4 SW_PREVIOUS");
//      break;
//    default:
//      break;
//    }
//    if(bdbAttributes.bdbNodeIsOnANetwork){
//      firstReport++;
//      zclSmartSwitch_Report();
//    }
//    if(firstReport < 2){
//      osal_start_timerEx(zclSmartSwitch_TaskID, SMARTSWITCH_FIRST_REPORT_EVT, 5000);
//    }else{
//      osal_start_reload_timer( zclSmartSwitch_TaskID, SMARTSWITCH_REPORT_EVT, SMARTSWITCH_REPORT_DELAY );
//    }
    return ( events ^ SMARTSWITCH_FIRST_REPORT_EVT );
  }
  
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
* @fn      zclSmartSwitch_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_5
*                 HAL_KEY_SW_4
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void zclSmartSwitch_HandleKeys( byte portAndAction, byte keyCode )
{
//  //LREP("portAndAction=0x%X keyCode=0x%X\r\n", portAndAction, keyCode);
  if(portAndAction & HAL_KEY_PORT0)
  {  
    p0LastState = P0 & HAL_KEY_P0_INPUT_PINS; // 0010 & 0011
    if(keyCode & BV(1))
    {
      zclFactoryResetter_HandleKeys(portAndAction, keyCode);
//      zclCommissioning_HandleKeys(portAndAction, keyCode);
      if (portAndAction & HAL_KEY_RELEASE) 
      {
        UART_String("Key 1 release");
      }
      if (portAndAction & HAL_KEY_PRESS) 
      {
        zclSmartSwitch_ControlRelay(1, COMMAND_TOGGLE, false);
        UART_String("Key 1 press"); 
        osal_stop_timerEx(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT);
        osal_start_reload_timer(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT, 100);
      }
    }
    if(keyCode & BV(2))
    {
      if (portAndAction & HAL_KEY_RELEASE) 
      {
        UART_String("Key 2 release");
      };
      if (portAndAction & HAL_KEY_PRESS) 
      {
        zclSmartSwitch_ControlRelay(2, COMMAND_TOGGLE, false);
        UART_String("Key 2 press");
        osal_stop_timerEx(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT);
        osal_start_reload_timer(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT, 100);
      }
    }
     if(keyCode & BV(0))
    {
      if (portAndAction & HAL_KEY_RELEASE) 
      {
        UART_String("Key 3 release");
      };
      if (portAndAction & HAL_KEY_PRESS) 
      {
        zclSmartSwitch_ControlRelay(3, COMMAND_TOGGLE, false);
        UART_String("Key 3 press");
        osal_stop_timerEx(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT);
        osal_start_reload_timer(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT, 100);
      }
    }
    if(keyCode & BV(3))
    {
      if (portAndAction & HAL_KEY_RELEASE) 
      {
        UART_String("Key 4 release");
      };
      if (portAndAction & HAL_KEY_PRESS) 
      {
        zclSmartSwitch_ControlRelay(4, COMMAND_TOGGLE, false);
        UART_String("Key 4 press");
        osal_stop_timerEx(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT);
        osal_start_reload_timer(zclSmartSwitch_TaskID, SMARTSWITCH_CHECK_KEY_RELEASE_EVT, 100);
      }
    }
  }
//  if(portAndAction & HAL_KEY_PORT2)
//  {  
//    UART_String("drop");
//    if(keyCode & BV(0)){
//      zclSmartSwitch_SaveAttributesToNV();
//    }
//  }
}


/*********************************************************************
* @fn      zclSmartSwitch_ProcessCommissioningStatus
*
* @brief   Callback in which the status of the commissioning process are reported
*
* @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
*
* @return  none
*/
static void zclSmartSwitch_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
  case BDB_COMMISSIONING_FORMATION:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
    {
      //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
      bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
    }
    else
    {
      //Want to try other channels?
      //try with bdb_setChannelAttribute
    }
    break;
  case BDB_COMMISSIONING_NWK_STEERING:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
    {
      //YOUR JOB:
      //We are on the nwk, what now?
//        NLME_SetPollRate(0);
//        NLME_SetQueuedPollRate(0);
//        NLME_SetResponseRate(0);
//        osal_start_timerEx( zclSmartSwitch_TaskID, SMARTSWITCH_REPORTING_EVT, 10 );
    }
    else
    {
      //See the possible errors for nwk steering procedure
      //No suitable networks found
      //Want to try other channels?
      //try with bdb_setChannelAttribute
    }
    break;
  case BDB_COMMISSIONING_FINDING_BINDING:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
    {
      //YOUR JOB:
    }
    else
    {
      //YOUR JOB:
      //retry?, wait for user interaction?
    }
    break;
  case BDB_COMMISSIONING_INITIALIZATION:
    //Initialization notification can only be successful. Failure on initialization
    //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
    
    //YOUR JOB:
    //We are on a network, what now?
    
    break;
#if ZG_BUILD_ENDDEVICE_TYPE    
  case BDB_COMMISSIONING_PARENT_LOST:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
    {
      //We did recover from losing parent
    }
    else
    {
      //Parent not found, attempt to rejoin again after a fixed delay
      osal_start_timerEx(zclSmartSwitch_TaskID, SMARTSWITCH_END_DEVICE_REJOIN_EVT, SMARTSWITCH_END_DEVICE_REJOIN_DELAY);
    }
    break;
#endif 
  }
}

/*********************************************************************
* @fn      zclSmartSwitch_ProcessIdentifyTimeChange
*
* @brief   Called to process any change to the IdentifyTime attribute.
*
* @param   endpoint - in which the identify has change
*
* @return  none
*/
//static void zclSmartSwitch_ProcessIdentifyTimeChange( uint8 endpoint )
//{
//  (void) endpoint;
//  
//  if ( zclSmartSwitch_IdentifyTime > 0 )
//  {
//    HalLedBlink ( HAL_LED_2, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
//  }
//  else
//  {
//    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
//  }
//}

/*********************************************************************
* @fn      zclSmartSwitch_BindNotification
*
* @brief   Called when a new bind is added.
*
* @param   data - pointer to new bind data
*
* @return  none
*/
static void zclSmartSwitch_BindNotification( bdbBindNotificationData_t *data )
{
  // SMARTSWITCH_TODO: process the new bind information
}


/*********************************************************************
* @fn      zclSmartSwitch_ProcessTouchlinkTargetEnable
*
* @brief   Called to process when the touchlink target functionality
*          is enabled or disabled
*
* @param   none
*
* @return  none
*/
//#if ( defined ( BDB_TL_TARGET ) && (BDB_TOUCHLINK_CAPABILITY_ENABLED == TRUE) )
//static void zclSmartSwitch_ProcessTouchlinkTargetEnable( uint8 enable )
//{
//  if ( enable )
//  {
//    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
//  }
//  else
//  {
//    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
//  }
//}
//#endif

/*********************************************************************
* @fn      zclSmartSwitch_BasicResetCB
*
* @brief   Callback from the ZCL General Cluster Library
*          to set all the Basic Cluster attributes to default values.
*
* @param   none
*
* @return  none
*/
static void zclSmartSwitch_BasicResetCB( void )
{
  
  /* SMARTSWITCH_TODO: remember to update this function with any
  application-specific cluster attribute variables */
  
  zclSmartSwitch_ResetAttributesToDefaultValues();
  
}
/*********************************************************************
* @fn      zclSampleApp_BatteryWarningCB
*
* @brief   Called to handle battery-low situation.
*
* @param   voltLevel - level of severity
*
* @return  none
*/
//void zclSampleApp_BatteryWarningCB( uint8 voltLevel )
//{
//  if ( voltLevel == VOLT_LEVEL_CAUTIOUS )
//  {
//    // Send warning message to the gateway and blink LED
//  }
//  else if ( voltLevel == VOLT_LEVEL_BAD )
//  {
//    // Shut down the system
//  }
//}

/******************************************************************************
*
*  Functions for processing ZCL Foundation incoming Command/Response messages
*
*****************************************************************************/

/*********************************************************************
* @fn      zclSmartSwitch_ProcessIncomingMsg
*
* @brief   Process ZCL Foundation incoming message
*
* @param   pInMsg - pointer to the received message
*
* @return  none
*/
static void zclSmartSwitch_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  char eventStr[20];
  sprintf(eventStr, "%d lux", (uint16)(pInMsg->zclHdr.commandID));
  UART_String(eventStr);
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
  case ZCL_CMD_READ_RSP:
    zclSmartSwitch_ProcessInReadRspCmd( pInMsg );
    break;
#endif
#ifdef ZCL_WRITE
  case ZCL_CMD_WRITE_RSP:
    zclSmartSwitch_ProcessInWriteRspCmd( pInMsg );
    break;
  case ZCL_CMD_WRITE:
    zclSmartSwitch_ProcessInWriteCmd(pInMsg);
    break;
#endif
  case ZCL_CMD_CONFIG_REPORT:
  case ZCL_CMD_CONFIG_REPORT_RSP:
  case ZCL_CMD_READ_REPORT_CFG:
  case ZCL_CMD_READ_REPORT_CFG_RSP:
  case ZCL_CMD_REPORT:
    //bdb_ProcessIncomingReportingMsg( pInMsg );
    break;
    
  case ZCL_CMD_DEFAULT_RSP:
    zclSmartSwitch_ProcessInDefaultRspCmd( pInMsg );
    break;
#ifdef ZCL_DISCOVER
  case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
    zclSmartSwitch_ProcessInDiscCmdsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
    zclSmartSwitch_ProcessInDiscCmdsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_ATTRS_RSP:
    zclSmartSwitch_ProcessInDiscAttrsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
    zclSmartSwitch_ProcessInDiscAttrsExtRspCmd( pInMsg );
    break;
#endif
  default:
    break;
  }
  
  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
* @fn      zclSmartSwitch_ProcessInReadRspCmd
*
* @brief   Process the "Profile" Read Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSmartSwitch_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;
  UART_String("read");
  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }
  
  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
* @fn      zclSmartSwitch_ProcessInWriteRspCmd
*
* @brief   Process the "Profile" Write Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSmartSwitch_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;
  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }
  
  return ( TRUE );
}

static uint8 zclSmartSwitch_ProcessInWriteCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteCmd_t *writeCmd;
  uint8 i;

  writeCmd = (zclWriteCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeCmd->numAttr; i++ )
  {
    zclAttrRec_t attrRec;
    zclWriteRec_t *statusRec = &(writeCmd->attrList[i]);

    if ( zclFindAttrRec( pInMsg->endPoint, pInMsg->clusterId,
                         statusRec->attrID, &attrRec ) )
    {
      if ( statusRec->dataType == attrRec.attr.dataType )
      {
//        char eventStr[20];
        switch(attrRec.clusterID){
        case ZCL_CLUSTER_POWER_ON_BEHAVIOR:
          if(attrRec.attr.attrId == ATTRID_POWER_ON_BEHAVIOR)
          {
            zclSmartSwitch_SaveAttributesToNV(pInMsg->endPoint, zclSmartSwitch_PowerOnBehavior[pInMsg->endPoint-1], BEHAVIOR);
            if(zclSmartSwitch_PowerOnBehavior[pInMsg->endPoint-1] == SW_PREVIOUS){
              zclSmartSwitch_SaveAttributesToNV(pInMsg->endPoint, 
                                      zclSmartSwitch_OnOff[pInMsg->endPoint-1],
                                      STATE_ONOFF);
            }
//            switch(pInMsg->endPoint){
//            case 1:
//              sprintf(eventStr, "enpoint 1 %d", (uint16)(zclSmartSwitch_PowerOnBehavior0));
//              UART_String(eventStr);
//              break;
//            case 2:
//              sprintf(eventStr, "enpoint 2 %d", (uint16)(zclSmartSwitch_PowerOnBehavior1));
//              UART_String(eventStr);
//              break;
//            case 3:
//              sprintf(eventStr, "enpoint 3 %d", (uint16)(zclSmartSwitch_PowerOnBehavior2));
//              UART_String(eventStr);
//              break;
//            case 4:
//              sprintf(eventStr, "enpoint 4 %d", (uint16)(zclSmartSwitch_PowerOnBehavior3));
//              UART_String(eventStr);
//              break;
//            default:
//              break;
//            }
          }
          break;
        default:
          break;
        }
          
          
//          status = zclWriteAttrData( pInMsg->msg->endPoint, &(pInMsg->msg->srcAddr),
//                                       &attrRec, statusRec );
      }
    }
  } // for loop
  
  
  
  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
* @fn      zclSmartSwitch_ProcessInDefaultRspCmd
*
* @brief   Process the "Profile" Default Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSmartSwitch_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  
  // Device is notified of the Default Response command.
  (void)pInMsg;
  
  return ( TRUE );
}

//#ifdef ZCL_DISCOVER
/*********************************************************************
* @fn      zclSmartSwitch_ProcessInDiscCmdsRspCmd
*
* @brief   Process the Discover Commands Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
//static uint8 zclSmartSwitch_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
//{
//  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
//  uint8 i;
//  
//  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
//  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
//  {
//    // Device is notified of the result of its attribute discovery command.
//  }
//  
//  return ( TRUE );
//}

/*********************************************************************
* @fn      zclSmartSwitch_ProcessInDiscAttrsRspCmd
*
* @brief   Process the "Profile" Discover Attributes Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
//static uint8 zclSmartSwitch_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
//{
//  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
//  uint8 i;
//  
//  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
//  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
//  {
//    // Device is notified of the result of its attribute discovery command.
//  }
//  
//  return ( TRUE );
//}

/*********************************************************************
* @fn      zclSmartSwitch_ProcessInDiscAttrsExtRspCmd
*
* @brief   Process the "Profile" Discover Attributes Extended Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
//static uint8 zclSmartSwitch_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
//{
//  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
//  uint8 i;
//  
//  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
//  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
//  {
//    // Device is notified of the result of its attribute discovery command.
//  }
//  
//  return ( TRUE );
//}
//#endif // ZCL_DISCOVER

static void zclSmartSwitch_ControlRelay(uint8 relayID, bool state, bool notReport){
  uint8 portValue = valueOut();
  uint8 ledIndicatorPin, ctrlRelayPin;
  switch (relayID){
  case 1:
    ledIndicatorPin = 1;
    ctrlRelayPin = 5;
    zclSmartSwitch_OnOff[0] ^= BV(0);
    zclSmartSwitch_ErorRuntime[0] = true;
    break;
  case 2:
    ledIndicatorPin = 0;
    ctrlRelayPin = 4;
    zclSmartSwitch_OnOff[1] ^= BV(0);
    zclSmartSwitch_ErorRuntime[1] = false;
    break;
  case 3:
    ledIndicatorPin = 3;
    ctrlRelayPin = 7;
    zclSmartSwitch_OnOff[2] ^= BV(0);
    zclSmartSwitch_ErorRuntime[2] = false;
    break;
  case 4:
    ledIndicatorPin = 2;
    ctrlRelayPin = 6;
    zclSmartSwitch_OnOff[3] ^= BV(0);
    zclSmartSwitch_ErorRuntime[3]= true;
    break;
  default:
    return;
  }
  if(state == COMMAND_ON){
    portValue |= BV(ledIndicatorPin);
    portValue &= ~BV(ctrlRelayPin);
  }
  else if(state == COMMAND_OFF){
    portValue &= ~BV(ledIndicatorPin);
    portValue |= BV(ctrlRelayPin);
  }
  else if(state == COMMAND_TOGGLE){
    portValue ^= BV(ledIndicatorPin)|BV(ctrlRelayPin);
  }
  

  if(zclSmartSwitch_PowerOnBehavior[SMARTSWITCH_ENDPOINT + relayID - 2] == SW_PREVIOUS){
    zclSmartSwitch_SaveAttributesToNV(SMARTSWITCH_ENDPOINT + relayID - 1, 
                                      zclSmartSwitch_OnOff[SMARTSWITCH_ENDPOINT + relayID - 2],
                                      STATE_ONOFF);
  }
  
  write8(portValue);
  zclSmartSwitch_ReportSwitchState(SMARTSWITCH_ENDPOINT + relayID - 1, notReport);
}

static void zclSmartSwitch_OnOffCB(uint8 cmd){
  afIncomingMSGPacket_t *pPtr = zcl_getRawAFMsg();
  if (pPtr->endPoint == SMARTSWITCH_ENDPOINT) //button 1
  {
    zclSmartSwitch_ControlRelay(1, cmd, true);
  }
  else if (pPtr->endPoint == SMARTSWITCH_ENDPOINT+1) //button 2
  {
    zclSmartSwitch_ControlRelay(2, cmd, true);
  }
  else if (pPtr->endPoint == SMARTSWITCH_ENDPOINT+2) //button 3
  {
    zclSmartSwitch_ControlRelay(3, cmd, true);
  }
  else if (pPtr->endPoint == SMARTSWITCH_ENDPOINT+3) //button 4
  {
    zclSmartSwitch_ControlRelay(4, cmd, true);
  }
}

static void zclSmartSwitch_ReportSwitchState( uint8 endpoint , bool notReport)
{
  // ?????? ???????????
  const uint8 NUM_ATTRIBUTES = (notReport)?1:2;
  zclReportCmd_t *pReportCmd;

  pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) +
                              (NUM_ATTRIBUTES * sizeof(zclReport_t)));
  if (pReportCmd != NULL) {
    pReportCmd->numAttr = NUM_ATTRIBUTES;

    pReportCmd->attrList[0].attrID = ATTRID_ON_OFF_ERROR_RUNTIME;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT8;
    pReportCmd->attrList[0].attrData = (void *)(&zclSmartSwitch_ErorRuntime[endpoint-1]);
    if(!notReport){
      pReportCmd->attrList[1].attrID = ATTRID_ON_OFF;
      pReportCmd->attrList[1].dataType = ZCL_DATATYPE_BOOLEAN;
      pReportCmd->attrList[1].attrData = (void *)(&zclSmartSwitch_OnOff[endpoint-1]);
    }
    zclSmartSwitch_BindDstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSmartSwitch_BindDstAddr.addr.shortAddr = 0;
    zclSmartSwitch_BindDstAddr.endPoint = 1;

    zcl_SendReportCmd(endpoint, &zclSmartSwitch_BindDstAddr,
                      ZCL_CLUSTER_ID_GEN_ON_OFF, pReportCmd,
                      ZCL_FRAME_CLIENT_SERVER_DIR, false, SeqNum++);
  }

  osal_mem_free(pReportCmd);
}

static void zclSmartSwitch_RestoreAttributesFromNV(void) {
  UART_String("Restore");
//  if (osal_nv_item_init(NV_SWITCH_STATE_ID, 1, &count) == ZSuccess) {
//    if(osal_nv_read(NV_SWITCH_STATE_ID, 0, 1, &count)  == ZSuccess){
//      switch(count){
//      case 0:
//        UART_String("fail");
//        break;
//      case 1:
//        UART_String("Ok1");
//        break;
//      case 2:
//        UART_String("Ok2");
//        break;
//      default:
//        UART_String("Ok");
//        break;
//      }
//    }
//  }

  uint8 portValue = b11110000;
  if (osal_nv_item_init(NV_SWITCH_STATE_ID, 8, stateSwitch) == ZSuccess) {
    if (osal_nv_read(NV_SWITCH_STATE_ID, 0, 8, stateSwitch) == ZSuccess) {
      zclSmartSwitch_OnOff[0] = stateSwitch[0];
      zclSmartSwitch_PowerOnBehavior[0] = stateSwitch[1];
      zclSmartSwitch_OnOff[1] = stateSwitch[2];
      zclSmartSwitch_PowerOnBehavior[1] = stateSwitch[3];
      zclSmartSwitch_OnOff[2] = stateSwitch[4];
      zclSmartSwitch_PowerOnBehavior[2] = stateSwitch[5];
      zclSmartSwitch_OnOff[3] = stateSwitch[6];
      zclSmartSwitch_PowerOnBehavior[3] = stateSwitch[7];

      switch(zclSmartSwitch_PowerOnBehavior[0]){
      case SW_ON:
        portValue |= BV(1);
        portValue &= ~BV(5); 
        break;
      case SW_PREVIOUS:
        if(zclSmartSwitch_OnOff[0] == COMMAND_ON){
          portValue |= BV(1);
          portValue &= ~BV(5); 
        }
        else if(zclSmartSwitch_OnOff[0] == COMMAND_OFF){
          portValue &= ~BV(1);
          portValue |= BV(5);
        }
       break;
      case SW_OFF: 
      default:
        portValue &= ~BV(1);
        portValue |= BV(5);
      }
      
      switch(zclSmartSwitch_PowerOnBehavior[1]){
      case SW_ON:
        portValue |= BV(0);
        portValue &= ~BV(4); 
        break;
      case SW_PREVIOUS:
        if(zclSmartSwitch_OnOff[1] == COMMAND_ON){
          portValue |= BV(0);
          portValue &= ~BV(4); 
        }
        else if(zclSmartSwitch_OnOff[1] == COMMAND_OFF){
          portValue &= ~BV(0);
          portValue |= BV(4);
        }
       break;
      case SW_OFF: 
      default:
        portValue &= ~BV(0);
        portValue |= BV(4);
      }
      
      switch(zclSmartSwitch_PowerOnBehavior[2]){
      case SW_ON:
        portValue |= BV(3);
        portValue &= ~BV(7); 
        break;
      case SW_PREVIOUS:
        if(zclSmartSwitch_OnOff[2] == COMMAND_ON){
          portValue |= BV(3);
          portValue &= ~BV(7); 
        }
        else if(zclSmartSwitch_OnOff[2] == COMMAND_OFF){
          portValue &= ~BV(3);
          portValue |= BV(7);
        }
       break;
      case SW_OFF: 
      default:
        portValue &= ~BV(3);
        portValue |= BV(7);
      }
      
      switch(zclSmartSwitch_PowerOnBehavior[3]){
      case SW_ON:
        portValue |= BV(2);
        portValue &= ~BV(6); 
        break;
      case SW_PREVIOUS:
        if(zclSmartSwitch_OnOff[3] == COMMAND_ON){
          portValue |= BV(2);
          portValue &= ~BV(6); 
        }
        else if(zclSmartSwitch_OnOff[3] == COMMAND_OFF){
          portValue &= ~BV(2);
          portValue |= BV(6);
        }
       break;
      case SW_OFF: 
      default:
        portValue &= ~BV(2);
        portValue |= BV(6);
      }
      
      zclSmartSwitch_OnOff[0]= (portValue & BV(1)) >> 1;
      zclSmartSwitch_OnOff[1] = (portValue & BV(0)) >> 0;
      zclSmartSwitch_OnOff[2] = (portValue & BV(3)) >> 3;
      zclSmartSwitch_OnOff[3] = (portValue & BV(2)) >> 2;
      
    }
  }
  write8(portValue);
}
static void zclSmartSwitch_SaveAttributesToNV(uint8 endpoint, uint8 data, uint8 type) {
  switch(type){
  case BEHAVIOR:
    if (osal_nv_item_init(NV_SWITCH_STATE_ID, 1, &data) == ZSuccess) {
      osal_nv_write(NV_SWITCH_STATE_ID, endpoint*2 - 1 , 1, &data);
    }
    break;
  case STATE_ONOFF:
    if (osal_nv_item_init(NV_SWITCH_STATE_ID, 1, &data) == ZSuccess) {
      osal_nv_write(NV_SWITCH_STATE_ID, endpoint*2 - 2 , 1, &data);
    }
    break;
  default:
    break;
  }
//  count++;
//  if (osal_nv_item_init(NV_SWITCH_STATE_ID, 1, &count) == ZSuccess) {
//      osal_nv_write(NV_SWITCH_STATE_ID, 0, 1, &count);
//  }
  
//  UART_String("save");
//  stateSwitch[0] = (uint8) zclSmartSwitch_OnOff0;
//  stateSwitch[1] = zclSmartSwitch_PowerOnBehavior0;
//  stateSwitch[2] = (uint8) zclSmartSwitch_OnOff1;
//  stateSwitch[3] = zclSmartSwitch_PowerOnBehavior1;
//  stateSwitch[4] = (uint8) zclSmartSwitch_OnOff2;
//  stateSwitch[5] = zclSmartSwitch_PowerOnBehavior2;
//  stateSwitch[6] = (uint8) zclSmartSwitch_OnOff3;
//  stateSwitch[7] = zclSmartSwitch_PowerOnBehavior3;
//  if (osal_nv_item_init(NV_SWITCH_STATE_ID, 8, stateSwitch) == ZSuccess) {
//      osal_nv_write(NV_SWITCH_STATE_ID, 0, 8, stateSwitch);
//  }
}
/****************************************************************************
****************************************************************************/

//HAL_ISR_FUNCTION( MyKeyPort0Isr, P0INT_VECTOR )
//{
//  HAL_ENTER_ISR();
//
////  if (P0IFG & BV(1))
////  {
////    if(!P0_1)
////      osal_start_timerEx( zclSmartSwitch_TaskID, SMARTSWITCH_PIR_EVT, 20 );
////  }
////  
////  if (P0IFG & BV(7))
////  {
////    osal_start_timerEx( zclSmartSwitch_TaskID, SMARTSWITCH_KEY_FEATURE_EVT, 20 );
////  }
//  /*
//    Clear the CPU interrupt flag for Port_0
//    PxIFG has to be cleared before PxIF
//  */
//  P0IFG = 0;
//  P0IF = 0;
//  
//  CLEAR_SLEEP_MODE();
//  HAL_EXIT_ISR();
//}

#ifdef PWR_OFF_DETECT
HAL_ISR_FUNCTION(halKeyPort2Isr, P2INT_VECTOR) {
    HAL_ENTER_ISR();
    UART_String("zo");
    if (P2IFG & BV(0)) {
      UART_String("x");
      zclSmartSwitch_SaveAttributesToNV();
      SystemResetSoft();
    }

    P2IFG = 0; //&= ~HAL_KEY_P2_INPUT_PINS;
    P2IF = 0;

    CLEAR_SLEEP_MODE();
    HAL_EXIT_ISR();
}
#endif