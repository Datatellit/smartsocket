#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "publicDefine.h"
#include "stm8sheader.h"

#define ADJUST_K   5656
#define ADJUST_B   0

// Keep alive message interval, around 6 seconds
#define RTE_TM_KEEP_ALIVE               500    // about 5s (500 * 10ms)
#define MAX_RF_FAILED_TIME              10      // Reset RF module when reach max failed times of sending
//#define MAX_RF_RESET_TIME               3      // Reset Node when reach max times of RF module consecutive reset

// Xlight Application Identification
#define XLA_VERSION               0x20
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM

#define XLA_MIN_VER_REQUIREMENT   0x20
typedef struct
{
  // Static & status parameters
  UC version                  :8;           // Data version, other than 0xFF
  UC present                  :1;           // 0 - not present; 1 - present
  UC state                    :1;           // SuperSensor On/Off
  UC swTimes                  :4;           // On/Off times
  UC reserved0                :2;
  UL totalEQ;
  UC totalEQreset;
  
  // Configurable parameters
  UC nodeID;                                // Node ID for this device
  UC subID;                                 // SubID
  UC NetworkID[6];
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC reserved1                :2;
  UC type;                                  // Type of SuperSensor
  US token;
  UC reserved2                :8;
  US senMap                   :16;
  int16_t coefficient         :16;          // current formula(need magnify 100 times for avoid decimals)
  int16_t constant            :16;          // current constant(need magnify 100 times for avoid decimals)
} Config_t;

extern Config_t gConfig;
extern bool gIsConfigChanged;
extern bool gNeedSaveBackup;
extern bool gIsStatusChanged;
extern bool gResetRF;
extern bool gResetNode;
extern uint8_t _uniqueID[UNIQUE_ID_LEN];
extern bool bAdjusting;

bool isNodeIdInvalid(uint8_t nodeid);
bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen);
void GotNodeID();
void GotPresented();
bool SendMyMessage();

void tmrProcess();
void idleProcess();

void ResetNodeToRegister();

#define XLA_PRODUCT_NODEID              NODEID_MIN_AC
#define XLA_PRODUCT_Type                ZEN_TARGET_SMARTSOCKET

#endif /* __GLOBAL_H */
