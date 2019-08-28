#include "_global.h"
#include "rf24l01.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "ProtocolParser.h"
#include "timer_4.h"
#include "timer_3.h"
#include "Uart2Dev.h"
#include "FlashDataStorage.h"
#include "wwdg.h"
#include "I_collect.h"
/*
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PC3 -> CE
  PC4 -> CSN
  PC7 -> MISO
  PC6 -> MOSI
  PC5 -> SCK
  PC2 -> IRQ

*/

#ifdef TEST
void testio()
{
  GPIO_Init(GPIOB , GPIO_PIN_5 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_4 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_3 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_2 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_1 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD , GPIO_PIN_1 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD , GPIO_PIN_7 , GPIO_MODE_OUT_PP_LOW_SLOW);
}
#endif

// Starting Flash block number of backup config
#define BACKUP_CONFIG_BLOCK_NUM         2
#define BACKUP_CONFIG_ADDRESS           (FLASH_DATA_START_PHYSICAL_ADDRESS + BACKUP_CONFIG_BLOCK_NUM * FLASH_BLOCK_SIZE)
#define STATUS_DATA_NUM                 4
#define STATUS_DATA_ADDRESS             (FLASH_DATA_START_PHYSICAL_ADDRESS + STATUS_DATA_NUM * FLASH_BLOCK_SIZE)

#define EQ_REPORT_INTERVAL              3000   // 30s
#define CURRENT_SND_INTERVAL            300    // 3s
#define CURRENT_SND_MAX_INTERVAL        6000   // about 60s (6000 * 10ms)
#define CURRENT_CHANGE_THRESHOLD        5      // 0.05A

// Unique ID
#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
  #define     UNIQUE_ID_ADDRESS         (0x48CD)
#endif
#if defined(STM8S103) || defined(STM8S003) ||  defined(STM8S903)
  #define     UNIQUE_ID_ADDRESS         (0x4865)
#endif

// Public variables
Config_t gConfig;
MyMessage_t sndMsg, rcvMsg;
uint8_t *psndMsg = (uint8_t *)&sndMsg;
uint8_t *prcvMsg = (uint8_t *)&rcvMsg;
bool gIsConfigChanged = FALSE;
bool gNeedSaveBackup = FALSE;
bool gIsStatusChanged = FALSE;
bool gResetRF = FALSE;
bool gResetNode = FALSE;
uint8_t _uniqueID[UNIQUE_ID_LEN];

// Moudle variables
uint8_t mStatus = SYS_INIT;
bool mGotNodeID = FALSE;
uint8_t mutex = 0;

// Keep Alive Timer
uint16_t mTimerKeepAlive = 0;
uint8_t m_cntRFReset = 0;
uint8_t m_cntRFSendFailed = 0;
// current report interval
uint16_t m_ICollectTick = 0;
// EQ report interval
uint16_t m_EQCollectTick = 0;
// EQ cal interval
uint16_t m_EQCalTick = 0;
uint16_t lastAccEQIndex = 0;
uint16_t eqIndex = 0;
#define SAVESTATUS_INTERVAL  10 // 10min
uint16_t save_tick = 0;

// sensor data interval
uint16_t m_SensorDataTick = 0;
bool bAdjusting = FALSE;

void tmrCalPower();

uint8_t *Read_UniqueID(uint8_t *UniqueID, uint16_t Length)  
{
  Flash_ReadBuf(UNIQUE_ID_ADDRESS, UniqueID, Length);
  return UniqueID;
}

bool isIdentityEmpty(const UC *pId, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId[i] > 0) return FALSE; }
  return TRUE;
}

bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId1[i] != pId2[i]) return FALSE; }
  return TRUE;
}

bool isNodeIdRequired()
{
  return( isIdentityEmpty(gConfig.NetworkID, ADDRESS_WIDTH) || isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) );
}

void UpdateNodeAddress(uint8_t _tx) {
  memcpy(rx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  rx_addr[0] = gConfig.nodeID;
  memcpy(tx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  if( _tx == NODEID_RF_SCANNER ) {
    tx_addr[0] = NODEID_RF_SCANNER;
  } else {
    tx_addr[0] = (isNodeIdRequired() ? BASESERVICE_ADDRESS : NODEID_GATEWAY);
  }
  RF24L01_setup(gConfig.rfChannel, gConfig.rfDataRate, gConfig.rfPowerLevel, BROADCAST_ADDRESS);
}

bool NeedUpdateRFAddress(uint8_t _dest) {
  bool rc = FALSE;
  if( sndMsg.header.destination == NODEID_RF_SCANNER && tx_addr[0] != NODEID_RF_SCANNER ) {
    UpdateNodeAddress(NODEID_RF_SCANNER);
    rc = TRUE;
  } else if( sndMsg.header.destination != NODEID_RF_SCANNER && tx_addr[0] != NODEID_GATEWAY ) {
    UpdateNodeAddress(NODEID_GATEWAY);
    rc = TRUE;
  }
  return rc;
}

// reset rf
void ResetRFModule()
{
  if(gResetRF)
  {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF=FALSE;
  }
  if(gResetNode)
  {
    mStatus = SYS_RESET;
    gResetNode=FALSE;
  }
}

// Save config to Flash
void SaveBackupConfig()
{
  if( gNeedSaveBackup ) {
    // back config FLASH
    if(Flash_WriteDataBlock(BACKUP_CONFIG_BLOCK_NUM, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gNeedSaveBackup = FALSE;
    }
  }
}

// Save status to Flash
void SaveStatusData()
{
  // status data contain nodeid subid and networkid
  if(gIsStatusChanged)
  {
    gNeedSaveBackup = TRUE;
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint8_t *)(&gConfig.rfChannel) - (uint8_t *)(&gConfig);
    memcpy(pData, (uint8_t *)&gConfig, nLen);
    if(Flash_WriteDataBlock(STATUS_DATA_NUM, pData, nLen))
    {
      gIsStatusChanged = FALSE;
    }
  }
}

// Save config data to Flash(can't be called at working time)
void SaveConfig()
{
  if( gIsConfigChanged ) {
    gIsStatusChanged = TRUE;
    // Overwrite entire config FLASH 
    uint8_t Attmpts = 0;
    while(++Attmpts <= 3) {
      if(Flash_WriteDataBlock(0, (uint8_t *)&gConfig, sizeof(gConfig)))
      {
        gIsConfigChanged = FALSE;
        break;
      }
    }
  }
  if(!gIsConfigChanged)
  { // ensure rf info is up to date
    ResetRFModule();
  }
  SaveStatusData();
}

// Initialize Node Address and look forward to being assigned with a valid NodeID by the SmartController
void InitNodeAddress() {
  // Whether has preset node id
  gConfig.nodeID = XLA_PRODUCT_NODEID;
  memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
}

bool IsStatusInvalid() {
  // gConfig.aircondStatus[2] > 32 temp more than 32
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT);
}

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || gConfig.nodeID == 0 || !IS_AC_NODEID(gConfig.nodeID)
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

bool isNodeIdInvalid(uint8_t nodeid)
{
  return( !IS_AC_NODEID(nodeid)  );
}

// Load config from Flash
void LoadConfig()
{
    // Load the config area
  Flash_ReadBuf(FLASH_DATA_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
  uint16_t nStatusLen = (uint8_t *)(&gConfig.nodeID) - (uint8_t *)(&gConfig);
  if( IsConfigInvalid() ) {
    // If config isn't OK, then try to load config from backup area
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS+nStatusLen, (uint8_t *)&gConfig.nodeID, sizeof(gConfig)-nStatusLen);
    bool backupInvalid = IsConfigInvalid();
    InitNodeAddress();
    if( backupInvalid ) {
      // If neither valid, then initialize config with default settings
        memset(&gConfig, 0x00, sizeof(gConfig));
        gConfig.version = XLA_VERSION;
        InitNodeAddress();
        gConfig.subID = 0;
        gConfig.type = XLA_PRODUCT_Type;
        gConfig.rptTimes = 1;
        gConfig.rfChannel = RF24_CHANNEL;
        gConfig.rfPowerLevel = RF24_PA_MAX;
        gConfig.rfDataRate = RF24_250KBPS;
        gConfig.coefficient = ADJUST_K;
        gConfig.constant = ADJUST_B;
    }
    gIsConfigChanged = TRUE;
    SaveConfig();
  } else {
    uint8_t bytVersion;
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&bytVersion, sizeof(bytVersion));
    if( bytVersion != gConfig.version ) gNeedSaveBackup = TRUE;
  }
  // Load the most recent status from FLASH
  uint8_t pData[50];
  memset(pData,0x00,sizeof(pData));
  uint16_t nLen = (uint8_t *)(&gConfig.rfChannel) - (uint8_t *)(&gConfig);
  Flash_ReadBuf(STATUS_DATA_ADDRESS, pData, nLen);
  if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
  { // status data valid    
    memcpy(&gConfig,pData,nStatusLen);
    if(isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) && !isNodeIdInvalid(gConfig.nodeID) )
    { // valid nodeid but with default network config,can covered by status data or back data if they are valid
      uint16_t networkOffset = (uint8_t *)(&gConfig.NetworkID) - (uint8_t *)(&gConfig);
      if( !isIdentityEmpty(pData+networkOffset,sizeof(gConfig.NetworkID)) )
      {
        memcpy(gConfig.NetworkID,pData+networkOffset,sizeof(gConfig.NetworkID));
      } 
    } 
  }
  else
  { // load backup area for status data
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, pData, nLen);
    if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
    { // status data valid 
      memcpy(&gConfig,pData,nStatusLen);
      if(isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) && !isNodeIdInvalid(gConfig.nodeID))
      { // valid nodeid but with default network config,can covered by status data or back data if they are valid
        uint16_t networkOffset = (uint8_t *)(&gConfig.NetworkID) - (uint8_t *)(&gConfig);
        if( !isIdentityEmpty(pData+networkOffset,sizeof(gConfig.NetworkID)) )
        {
          memcpy(gConfig.NetworkID,pData+networkOffset,sizeof(gConfig.NetworkID));
        }        
      }
    }
  }
 
  if(IsStatusInvalid())
  {
    // default status value
    gConfig.version = XLA_VERSION;
    gConfig.totalEQ = 0;
    gConfig.totalEQreset = 1;
  }
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
    feed_wwdg();
  }
  return FALSE;
}

// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
      
    uint8_t lv_tried = 0;
    uint16_t delay;
    while (lv_tried++ <= gConfig.rptTimes ) {
      
      mutex = 0;
      if(RF24L01_set_mode_TX_timeout() == -1) 
        break;
      if(RF24L01_write_payload_timeout(psndMsg, PLOAD_WIDTH) == -1) 
        break;
      WaitMutex(0x1FFFF);
      if (mutex == 1) {
        m_cntRFSendFailed = 0;
        m_cntRFReset = 0;
        break; // sent sccessfully
      } else {
        m_cntRFSendFailed++;
        if( m_cntRFSendFailed >= MAX_RF_FAILED_TIME ) {
          m_cntRFSendFailed = 0;
          m_cntRFReset++;
          if( m_cntRFReset >= 3 ) {
            // Cold Reset
            WWDG->CR = 0x80;       
            m_cntRFReset = 0;
            break;
          } else if( m_cntRFReset >= 2 ) {
            // Reset whole node
            mStatus = SYS_RESET;
            break;
          }

          // Reset RF module
          //RF24L01_DeInit();
          delay = 0x1FFF;
          while(delay--)feed_wwdg();
          RF24L01_init();
          NRF2401_EnableIRQ();
          UpdateNodeAddress(NODEID_GATEWAY);
          continue;
        }
      }
      
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
      // Repeat the message if necessary
      delay = 0xFFF;
      while(delay--)feed_wwdg();
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
    
    // Reset Keep Alive Timer
    mTimerKeepAlive = 0;
  }

  return(mutex > 0);
}

void GotNodeID() {
  mGotNodeID = TRUE;
  UpdateNodeAddress(NODEID_GATEWAY);
  gNeedSaveBackup = TRUE;
}

void GotPresented() {
  mStatus = SYS_RUNNING;
  gConfig.swTimes = 0;
  gIsStatusChanged = TRUE;
}
int main( void ) {
  static uint8_t bRestart = 1;
  //After reset, the device restarts by default with the HSI clock divided by 8.
  //CLK_DeInit();
  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);  // now: HSI=16M prescale = 1; sysclk = 16M

  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  // Load config from Flash
  LoadConfig();
  if(((gConfig.totalEQ >> 8)^0xFFFFFF) == 0)
  {
    gConfig.totalEQ = 0;
    gConfig.totalEQreset = 1;    
  }
  
  // Init Watchdog
  wwdg_init();
  
  gIsStatusChanged = TRUE;
  SaveConfig(); 

  init_ADC();
  uart2_config(9600);
  printlog("start...\r\n");
  // Init timer
  TIM4_10ms_handler = tmrProcess;
  Time4_Init();
  TIM3_500ms_handler = tmrCalPower;
  Time3_Init();
  //TIM2_Init();
  GPIO_Init(GPIOE , GPIO_PIN_5 , GPIO_MODE_OUT_PP_LOW_SLOW);
#ifdef TEST
   testio();
#endif
  uint16_t pre_current = 0;
  while(1) {
    // Go on only if NRF chip is presented
    disableInterrupts();
    gConfig.present = 0;
    RF24L01_init();
    u16 timeoutRFcheck = 0;
    while(!NRF24L01_Check()) {
      if( timeoutRFcheck > 50 ) {
        break;
      }
      feed_wwdg();
      timeoutRFcheck++;
    }
    // IRQ
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    
    // Send Presentation Message
    if( bRestart == 1 )
    {
      Msg_Presentation();
      bRestart = 0;
    } 
    SendMyMessage();
    RF24L01_set_mode_RX();
    mStatus = SYS_RUNNING;

    while (mStatus == SYS_RUNNING) {
      
      // Feed the Watchdog
      feed_wwdg();

      if(bAdjusting)
      {
        if(m_SensorDataTick >= CURRENT_SND_INTERVAL)
        {
          m_SensorDataTick = 0;
          Msg_CurrentChange(GetSensorData(),1);
          SendMyMessage();   
        }
      }
      else
      {
        // report eq
        if(m_EQCollectTick >= EQ_REPORT_INTERVAL)
        {
           m_EQCollectTick = 0;
           Msg_TotalEQReport(eqIndex,GetCurrent());
        }
        SendMyMessage();
        // report current 
        if(m_ICollectTick >= CURRENT_SND_INTERVAL)
        {
           uint16_t current = GetCurrent();
           uint16_t threshold = 0;
           if(current >= pre_current) 
           {
             threshold = current - pre_current;
           }
           else 
           {
             threshold = pre_current - current;
           }
           if(threshold >= CURRENT_CHANGE_THRESHOLD || m_ICollectTick>=CURRENT_SND_MAX_INTERVAL)
           {
             m_ICollectTick = 0;
             pre_current = current;
             Msg_CurrentChange(current,0);
           }
        }
        SendMyMessage();        
      }
      ////////////rfscanner process///////////////////////////////
      ProcessOutputCfgMsg(); 
      ////////////rfscanner process/////////////////////////////// 
      ResetRFModule();
      SaveConfig();
      
      // ToDo: Check heartbeats
      // mStatus = SYS_RESET, if timeout or received a value 3 times consecutively
    }
  }
}

// Execute timer operations
void tmrProcess() {
   m_ICollectTick++;
   m_EQCollectTick++; 
   m_SensorDataTick++;
  // Save config into backup area
   SaveBackupConfig();
   if(runLEDTick >0)
   {
     runLEDTick--;
     if(runLEDTick == 0)
     {
       GPIO_WriteHigh(GPIOE , GPIO_PIN_5);
     }
   }
}

void tmrCalPower() {
  //GPIO_WriteReverse(GPIOD , GPIO_PIN_4);
   m_EQCalTick++; 
   if(m_EQCalTick >= 120)
   {//1min interval
     m_EQCalTick = 0;
     save_tick++;
     eqIndex = (eqIndex+1)%10000;
     uint16_t eqeverymin = GetMinuteEQ();
     gConfig.totalEQ += eqeverymin;
     if(save_tick >= SAVESTATUS_INTERVAL)
     {
       gIsStatusChanged = TRUE;
       save_tick = 0;
     } 
   }
}


INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5) {
#ifdef TEST
  PD7_High;
#endif
  if(RF24L01_is_data_available()) {
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(prcvMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
#ifdef TEST
    PD7_Low;
#endif
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info;
#ifdef TEST
    PD7_Low;
#endif    
    return;
  }

   RF24L01_clear_interrupts();
#ifdef TEST
   PD7_Low;
#endif   
}
