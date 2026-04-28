#ifndef PARSE_H
#define PARSE_H
#include <iostream>
#include <vector>

enum MessageType {
  GRANT = 0,
  STATUS = 1,
  UPDATE = 2,
  CONTROL_CHANNEL = 3,
  REGISTRATION = 4,
  DEREGISTRATION = 5,
  AFFILIATION = 6,
  SYSID = 7,
  ACKNOWLEDGE = 8,
  LOCATION = 9,
  PATCH_ADD = 10,
  PATCH_DELETE = 11,
  DATA_GRANT = 12,
  UU_ANS_REQ = 13,
  UU_V_GRANT = 14,
  UU_V_UPDATE = 15,
  INVALID_CC_MESSAGE = 16,
  TDULC = 17,
  CALL_ALERT = 18,
  UNKNOWN = 99
};

struct PatchData {
  unsigned long sg;
  unsigned long ga1;
  unsigned long ga2;
  unsigned long ga3;
};

struct TrunkMessage {
  MessageType message_type;
  std::string meta;
  double freq;
  long talkgroup;
  bool encrypted;
  bool emergency;
  bool duplex;
  bool mode;
  int priority;
  int tdma_slot;
  bool phase2_tdma;
  long source;
  int sys_num;
  unsigned long sys_id;
  int sys_rfss;
  int sys_site_id;
  unsigned long nac;
  unsigned long wacn;
  PatchData patch_data;
  unsigned long opcode;
  
};

class TrunkParser {
  std::vector<TrunkMessage> parse_message(std::string s);
};
#endif
