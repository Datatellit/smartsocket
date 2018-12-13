#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"

#define DEVICE_SW_OFF 0
#define DEVICE_SW_ON  1

uint8_t bMsgReady = 0;
bool bDelaySend = FALSE;

void MsgScanner_ProbeAck();
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid);
void Process_SetConfig(u8 _len);
void Process_SetDevConfig(u8 _len);
void Process_SetupRF(const UC *rfData,uint8_t rflen);

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid);
typedef struct
{
  uint8_t offset;
  uint8_t size;
  uint8_t byUniqueid;  // whether getconfig by uniqueid
}CfgBlock;
#define OUT_CFG_MESSAGE_LEN           16
CfgBlock out_cfg_msg_buf[OUT_CFG_MESSAGE_LEN];
u8 cfg_msg_out_buf_read_ptr = 0;
u8 cfg_msg_out_buf_write_ptr = 0;

bool AddCfgOutputBuf(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {  
  CfgBlock cfgblock;
  cfgblock.offset = offset;
  cfgblock.size = size;
  cfgblock.byUniqueid = isNeedUniqueid;
  out_cfg_msg_buf[cfg_msg_out_buf_write_ptr++] = cfgblock;
  cfg_msg_out_buf_write_ptr %= OUT_CFG_MESSAGE_LEN;
  return TRUE;
}

bool ProcessOutputCfgMsg() {
  // Send output Cfg msg
  while( cfg_msg_out_buf_read_ptr != cfg_msg_out_buf_write_ptr) {   
    CfgBlock cfgblock = out_cfg_msg_buf[cfg_msg_out_buf_read_ptr++];
    SendCfgBlock(cfgblock.offset,cfgblock.size,TRUE);
    cfg_msg_out_buf_read_ptr %= OUT_CFG_MESSAGE_LEN;
  }
  return TRUE;
}

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {
  // Send output Cfg msg  
    build(NODEID_RF_SCANNER, gConfig.subID, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);
    // Common payload
    sndMsg.payload.data[0] = SCANNER_GETDEV_CONFIG;
    sndMsg.payload.data[1] = offset;
    uint8_t custom_playload = 2;
    if(isNeedUniqueid != 0) 
    {
      memcpy(sndMsg.payload.data + 2,_uniqueID, UNIQUE_ID_LEN);
      custom_playload += UNIQUE_ID_LEN;
    }  
    memcpy(sndMsg.payload.data + custom_playload, (void *)((uint16_t)(&gConfig) + offset), size);
    moSetLength(size+custom_playload);
    moSetPayloadType(P_CUSTOM);
    bMsgReady = 1;
    SendMyMessage();
    return TRUE;
}
// Assemble message
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck)
{
    sndMsg.header.version_length = PROTOCOL_VERSION;
    sndMsg.header.sender = gConfig.nodeID;
    sndMsg.header.destination = _destination;
    sndMsg.header.sensor = _sensor;
    sndMsg.header.type = _type;
    moSetCommand(_command);
    moSetRequestAck(_enableAck);
    moSetAck(_isAck);
}

uint8_t ParseProtocol(){
  if( rcvMsg.header.destination != gConfig.nodeID && rcvMsg.header.destination != BROADCAST_ADDRESS ) return 0;
  
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
    if( _type == I_REBOOT ) {
      if( IS_MINE_SUBID(_sensor) ) {
          WWDG->CR = 0x80;
        return 0;
      }
    }else if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
      case NCF_QUERY:
        // Inform controller with version & NCF data
        Msg_NodeConfigData(_sender);
        return 1;
        break;     
      case NCF_MAP_SENSOR:
        gConfig.senMap = rcvMsg.payload.data[0] + rcvMsg.payload.data[1] * 256;
        break;
      case NCF_DEV_SET_SUBID:
          gConfig.subID = rcvMsg.payload.data[0];
        break;
      case NCF_DEV_MAX_NMRT:
        gConfig.rptTimes = rcvMsg.payload.data[0];
        break;
      case NCF_DEV_SET_RF:
        if( _lenPayl >= 1)
        {
           // Node base rf info Config
          uint8_t newNid = rcvMsg.payload.data[0];
          if(!isNodeIdInvalid(newNid))
          {
            gConfig.nodeID = newNid;
            gResetRF = TRUE;
          }
        }
        if(_lenPayl >= 2) 
        {
          uint8_t rfchannel = rcvMsg.payload.data[1];
          if(rfchannel != 0)
          {
            gConfig.rfChannel = rfchannel;
            gResetRF = TRUE;
          }
        }
        if(_lenPayl >= 3)
        {
          uint8_t subid = rcvMsg.payload.data[2];
          gConfig.subID = subid;
        }
        if(_lenPayl >= 4)
        {
          uint8_t netlen = _lenPayl - 3;
          uint8_t bnetvalid = 0;
          for(uint8_t i = 0;i<netlen;i++)
          {
            if(rcvMsg.payload.data[3+i] != 0) 
            {
              bnetvalid = 1;
              break;
            }
          }
          if(bnetvalid)
          {
            memset(gConfig.NetworkID,0x00,sizeof(gConfig.NetworkID));
            for(uint8_t j = 0;j<netlen;j++)
            {
              gConfig.NetworkID[4-j] = rcvMsg.payload.data[3+j];
            }
            gResetRF = TRUE;
          }
        }
        break;
      case NCF_DEV_SET_SENSOR:
        uint8_t onoff = rcvMsg.payload.data[0];
        if(onoff == DEVICE_SW_ON)
        {
           bAdjusting = TRUE;
        }
        else if(onoff == DEVICE_SW_OFF)
        {
          gConfig.coefficient = (rcvMsg.payload.data[2]<<8 | rcvMsg.payload.data[3]);
          gConfig.constant = (rcvMsg.payload.data[5]<<8 | rcvMsg.payload.data[6]);
          if(rcvMsg.payload.data[4] == 0)
          {
            gConfig.constant = gConfig.constant*(-1);
          }     
          bAdjusting = FALSE;
        }
      }
      gIsConfigChanged = TRUE;
      Msg_NodeConfigAck(_sender, _sensor);
      return 1;
    }else if( _type == I_GET_NONCE ) {
      // RF Scanner Probe
        if( _sender == NODEID_RF_SCANNER ) {
          uint8_t _lenPayl = miGetLength();
          if( rcvMsg.payload.data[0] == SCANNER_PROBE ) {      
            MsgScanner_ProbeAck();
          } else if( rcvMsg.payload.data[0] == SCANNER_SETUP_RF ) {
            if(!IS_MINE_SUBID(_sensor)) return 0;  
            Process_SetupRF(rcvMsg.payload.data + 1,_lenPayl-1);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETUPDEV_RF ) {
            if(!isIdentityEqual(rcvMsg.payload.data + 1,_uniqueID,UNIQUE_ID_LEN)) return 0;
            Process_SetupRF(rcvMsg.payload.data + 1 + UNIQUE_ID_LEN,_lenPayl-1 - UNIQUE_ID_LEN);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETCONFIG ) {
            
            if(!IS_MINE_SUBID(_sensor)) return 0;          
            uint8_t cfg_len = _lenPayl - 2;
            Process_SetConfig(cfg_len);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_SETDEV_CONFIG ) {  
            if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
            uint8_t cfg_len = _lenPayl - 10;
            Process_SetDevConfig(cfg_len);
          }
          else if( rcvMsg.payload.data[0] == SCANNER_GETDEV_CONFIG ) {  
            uint8_t offset = rcvMsg.payload.data[1];
            uint8_t cfgblock_len = rcvMsg.payload.data[10];
            if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
            MsgScanner_ConfigAck(offset,cfgblock_len,TRUE); 
          }
          else if( rcvMsg.payload.data[0] == SCANNER_GETCONFIG ) { 
            if(!IS_MINE_SUBID(_sensor)) return 0;  
            uint8_t offset = rcvMsg.payload.data[1];
            uint8_t cfgblock_len = rcvMsg.payload.data[2];
            MsgScanner_ConfigAck(offset,cfgblock_len,FALSE);
          }
          return 1;
        }      
      }
    break;   
  case C_REQ:
    if(IS_MINE_SUBID(_sensor))
    {
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

void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf) {
  build(_to, _ncf, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[0] = 1;      // OK
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  bMsgReady = 1;
}

// Prepare NCF query ack message
void Msg_NodeConfigData(uint8_t _to) {
  uint8_t payl_len = 0;
  build(_to, NCF_QUERY, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.subID;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = gConfig.senMap % 256;
  sndMsg.payload.data[payl_len++] = gConfig.senMap / 256;
  sndMsg.payload.data[payl_len++] = gConfig.rptTimes;
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
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

//----------------------------------------------
// RF Scanner Messages
//----------------------------------------------
// Probe ack message
void MsgScanner_ProbeAck() {
  uint8_t payl_len = UNIQUE_ID_LEN + 1;
  build(NODEID_RF_SCANNER, 0x00, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);

  // Common payload
  sndMsg.payload.data[0] = SCANNER_PROBE;
  memcpy(sndMsg.payload.data + 1, _uniqueID, UNIQUE_ID_LEN);
  
  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = gConfig.nodeID;
  sndMsg.payload.data[payl_len++] = gConfig.subID;
  sndMsg.payload.data[payl_len++] = gConfig.rfChannel;
  sndMsg.payload.data[payl_len++] = (gConfig.rfDataRate << 2) + gConfig.rfPowerLevel;
  memcpy(sndMsg.payload.data + payl_len, gConfig.NetworkID, sizeof(gConfig.NetworkID));
  payl_len += sizeof(gConfig.NetworkID);
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;
//    uint8_t uniqueid[8];
//    UC ConfigBlock[15];
//}MyMsgPayload_t
#define CFGBLOCK_SIZE    15
#define CFGBLOCK_NO_UNIQUEID_SIZE CFGBLOCK_SIZE+UNIQUE_ID_LEN
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid) {
  uint8_t cfg_end_offset = cfglen;
  if(cfglen == 0) cfg_end_offset = sizeof(Config_t)-1;
  else
  {
    cfg_end_offset = offset + cfglen > sizeof(Config_t)-1?sizeof(Config_t)-1:offset + cfglen;
  }  
  while( offset < cfg_end_offset )
  {
    uint8_t left_len = cfg_end_offset - offset;
    uint8_t payl_len = left_len < CFGBLOCK_SIZE ? left_len : CFGBLOCK_SIZE;
    if(_isByUniqueid) AddCfgOutputBuf(offset,payl_len,1);
    else  
    {
      payl_len = left_len < CFGBLOCK_NO_UNIQUEID_SIZE ? left_len : CFGBLOCK_NO_UNIQUEID_SIZE;
      AddCfgOutputBuf(offset,payl_len,0);
    }
    offset+=payl_len;
    offset %= sizeof(Config_t);
  }
}

//////set config by nodeid&subid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;  //config offset
//    UC ConfigBlock[23];
//}MyMsgPayload_t
//////set config by nodeid&subid data struct/////////////////////
void Process_SetConfig(u8 _len) {
  uint8_t offset = rcvMsg.payload.data[1];
  memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2,_len);
  gIsConfigChanged = TRUE;
}
//////set config by uniqueid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;   //config offset
//    uint8_t uniqueid[8];
//    
//    UC ConfigBlock[15];
//}MyMsgPayload_t
//////set config by uniqueid data struct/////////////////////
void Process_SetDevConfig(u8 _len) {
    uint8_t offset = rcvMsg.payload.data[1];
    memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2+UNIQUE_ID_LEN,_len);
    gIsConfigChanged = TRUE;
}
bool IsNodeidValid(uint8_t nodeid)
{
  return IS_AC_NODEID(nodeid);
}
//////set rf /////////////////////////////////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t uniqueid[8];
//    uint8_t channel;
//    uint8_t datarate;
//    uint8_t powerlevel;
//    uint8_t network[6];
//    uint8_t nodeid;        //unnecessary data field��has this field��need change nodeid��0 indicate ignore this parameter
//    uint8_t subid;         //unnecessary data field��has this field��need change subid
//}MyMsgPayload_t
//////set rf /////////////////////////////////////////////////
void Process_SetupRF(const UC *rfData,uint8_t rflen)
{
  bool bNeedChangeCfg = FALSE;
  if(rflen > 0 &&(*rfData)>=0 && (*rfData)<=127)
  {
    if(gConfig.rfChannel != (*rfData))
    {
      gConfig.rfChannel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 1 &&(*rfData)>=RF24_1MBPS && (*rfData)<= RF24_250KBPS)
  {
    if(gConfig.rfDataRate != (*rfData))
    {
      gConfig.rfDataRate = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 2 &&(*rfData)>=RF24_PA_MIN && (*rfData)<= RF24_PA_ERROR)
  {
    if(gConfig.rfPowerLevel != (*rfData))
    {
      gConfig.rfPowerLevel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  bool bValidNet = FALSE;
  bool newNetwork[6] = {0};
  if(rflen > 8)
  {  
    for(uint8_t i = 0;i<6;i++)
    {
      if(*(rfData+i) != 0)
      {
        bValidNet=TRUE;
        break;
      }
    }
    if(isIdentityEqual(rfData,gConfig.NetworkID,sizeof(gConfig.NetworkID)))
    {
      bValidNet=FALSE;
    }
    else
    {
      memcpy(newNetwork,rfData,sizeof(newNetwork));
    }
  }
  rfData = rfData + sizeof(gConfig.NetworkID);
  bool bNeedResetNode = FALSE;
  if(rflen > 9 && (* rfData) != 0)
  {
    if(gConfig.nodeID != (* rfData))
    {
      if(IsNodeidValid(*rfData))
      {
        gConfig.nodeID = (* rfData);
        bNeedResetNode = TRUE;
      }    
    }
  }
  rfData++; 
  if(rflen > 10)
  {
    if(gConfig.subID != (* rfData ))
    {
      gConfig.subID = (*rfData);
      bNeedChangeCfg = TRUE;
    }
  }
  if(bValidNet)
  {// nodeid is valid,allow change networkid
    if(IsNodeidValid(gConfig.nodeID))
    {
      memcpy(gConfig.NetworkID,newNetwork,sizeof(gConfig.NetworkID));
      bNeedResetNode = TRUE;
    }
  }
  if(bNeedResetNode)
    gResetNode = TRUE;
  if(gResetNode || gResetRF || bNeedChangeCfg)
  {
    gIsConfigChanged = TRUE;
  }
}
//----------------------------------------------