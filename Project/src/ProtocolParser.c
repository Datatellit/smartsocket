#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "rf24l01.h"
#include "xliNodeConfig.h"

#define DEVICE_SW_OFF 0
#define DEVICE_SW_ON  1

uint16_t delaySendTick = 0;
bool bDelaySend = FALSE;
uint8_t runLEDTick = 0;

uint8_t ParseProtocol(){
  if( rcvMsg.header.destination != gConfig.nodeID && rcvMsg.header.destination != BROADCAST_ADDRESS ) return 0;
  
  uint8_t ret = ParseCommonProtocol();
  if(ret) return 1;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = rcvMsg.header.sender;  // The original sender
  uint8_t _type = rcvMsg.header.type;
  uint8_t _sensor = rcvMsg.header.sensor;
  uint8_t _lenPayl = miGetLength();
  bool _needAck = (bool)miGetRequestAck();
  bool _isAck = (bool)miGetAck();
  
  bDelaySend = FALSE;
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
        case NCF_DEV_SET_SENSOR:
        {
            uint8_t onoff = rcvMsg.payload.data[0];
            if(onoff == DEVICE_SW_ON)
            {
               bAdjusting = TRUE;
            }
            else if(onoff == DEVICE_SW_OFF)
            {
              if(_lenPayl >= 7)
              {                     
                gConfig.coefficient = (rcvMsg.payload.data[2]<<8 | rcvMsg.payload.data[3]);
                gConfig.constant = (rcvMsg.payload.data[5]<<8 | rcvMsg.payload.data[6]);
                if(rcvMsg.payload.data[4] == 0)
                {
                  gConfig.constant = gConfig.constant*(-1);
                }     
              }
              bAdjusting = FALSE;
            }
        }
        break; 
      }
    }
    break;
  case C_REQ:
    if(IS_MINE_SUBID(_sensor))
    {
      if( _type == V_STATUS ) {
        GPIO_WriteLow(GPIOE , GPIO_PIN_5);
        runLEDTick = 200;
      }
      if(_isAck)
      {
        if( _type == V_KWH )
        { // receive reset ack
          gConfig.totalEQreset = 0;
        }
      }
      if( _needAck ) {
        if( IS_MINE_SUBID(_sensor) ) {
        }
      }  
    }  
    break;
  default:
    break;
  }
  
  return 0;
}

void Msg_RequestNodeID() {
  // Request NodeID for device
  build(BASESERVICE_ADDRESS, NODE_TYP_LAMP, C_INTERNAL, I_ID_REQUEST, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
  build(NODEID_GATEWAY, S_POWER, C_PRESENTATION, gConfig.type, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// eq report message
void Msg_EQReport(uint16_t eq,uint16_t index,uint16_t current) {
  build(NODEID_GATEWAY, gConfig.subID, C_REQ, V_KWH, 0, 1);
  moSetPayloadType(P_BYTE);
  moSetLength(6);
  sndMsg.payload.data[0] = eq % 256;
  sndMsg.payload.data[1] = eq / 256;
  sndMsg.payload.data[2] = index % 256;
  sndMsg.payload.data[3] = index / 256;
  sndMsg.payload.data[4] = current % 256;
  sndMsg.payload.data[5] = current / 256;
  bMsgReady = 1;
}

void Msg_TotalEQReport(uint16_t index,uint16_t current)
{
  uint8_t bNeedack = 0;
  if(gConfig.totalEQreset == 1)
  {
    bNeedack = 1;
  }
  build(NODEID_GATEWAY, gConfig.subID, C_REQ, V_KWH, bNeedack,!bNeedack );
  moSetPayloadType(P_BYTE);
  moSetLength(9);
  //memcpy(sndMsg.payload.data,&gConfig.totalEQ,sizeof(gConfig.totalEQ));
  sndMsg.payload.data[0] = gConfig.totalEQ &0xFF;
  sndMsg.payload.data[1] = (gConfig.totalEQ >> 8)&0xFF;
  sndMsg.payload.data[2] = (gConfig.totalEQ >> 16)&0xFF;
  sndMsg.payload.data[3] = gConfig.totalEQ >> 24;
  sndMsg.payload.data[8] = gConfig.totalEQreset;
  sndMsg.payload.data[4] = index % 256;
  sndMsg.payload.data[5] = index / 256;
  sndMsg.payload.data[6] = current % 256;
  sndMsg.payload.data[7] = current / 256;
  sndMsg.payload.data[8] = gConfig.totalEQreset;
  bMsgReady = 1;
}

// current change message
void Msg_CurrentChange(uint16_t current,bool isOriginal) {
  build(NODEID_GATEWAY, gConfig.subID, C_REQ, V_CURRENT, 0, 1);
  moSetPayloadType(P_BYTE);
  uint8_t len = 2;
  sndMsg.payload.data[0] = current % 256;
  sndMsg.payload.data[1] = current / 256;
  if(isOriginal)
  {
    sndMsg.payload.data[2] = 1;
    len++;
  } 
  moSetLength(len);
  bMsgReady = 1;
}

/*// sensor data message
void Msg_SensorData(uint16_t sensor) {
  build(NODEID_GATEWAY, gConfig.subID, C_REQ, V_CURRENT, 0, 1);
  moSetPayloadType(P_UINT16);
  moSetLength(2);
  sndMsg.payload.data[0] = sensor % 256;
  sndMsg.payload.data[1] = sensor / 256;
  bMsgReady = 1;
}*/

