#ifndef SMARTNET_PARSE_H
#define SMARTNET_PARSE_H
#include "parser.h"
#include "system.h"
#include "system_impl.h"
#include <iostream>
#include <vector>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/tokenizer.hpp>
#include <stdio.h>

#include <boost/log/trivial.hpp>

// OSW commands range from $000 to $3ff
// Within this range are 4 ranges for channel indicators
// 		$000-$2f7 (760 channels)
// 		$32f-$33f ( 17 channels)
//    $3be      (  1 channel )
//    $3c1-$3fe ( 62 channels)
// Channels are pre-defined for 8/9 trunks. If an OBT trunk, the first range gets
// used to to indicate both inbound and outbound channels, with radio programming
// determining frequencies.
//    from $000-$2f7 (760 channels)
//         $000-$17b (380 channels) OBT inbound channels
//         $17c-$2f7 (380 channels) OBT outbound channels
#define OSW_MIN 0x000
#define OSW_CHAN_BAND_1_MIN 0x000 // Bandplan Range 1, also used for OBT channelization
#define OSW_CHAN_BAND_1_MAX 0x2f7 // Bandplan Range 1, also used for OBT channelization

#define OSW_BACKGROUND_IDLE 0x2f8
#define OSW_FIRST_CODED_PC 0x304
#define OSW_FIRST_NORMAL 0x308
#define OSW_FIRST_TY2AS1 0x309 // Unused - we don't support Type I trunks (or in this case, Type IIi)
#define OSW_EXTENDED_FCN 0x30b
#define OSW_AFFIL_FCN 0x30d
#define OSW_TY2_AFFILIATION 0x310
#define OSW_TY1_STATUS_MIN 0x310 // Unused - we don't support Type I trunks
#define OSW_TY2_MESSAGE 0x311
#define OSW_TY1_STATUS_MAX 0x317 // Unused - we don't support Type I trunks
#define OSW_TY1_ALERT 0x318      // Unused - we don't support Type I trunks
#define OSW_TY1_EMERGENCY 0x319  // Unused - we don't support Type I trunks
#define OSW_TY2_CALL_ALERT 0x319
#define OSW_SECOND_NORMAL 0x320
#define OSW_FIRST_ASTRO 0x321
#define OSW_SYSTEM_CLOCK 0x322
#define OSW_SCAN_MARKER 0x32b
#define OSW_EMERG_ANNC 0x32e

#define OSW_CHAN_BAND_2_MIN 0x32f // Bandplan Range 2
#define OSW_CHAN_BAND_2_MAX 0x33f // Bandplan Range 2

// The following command range ($340-$3bd) wastes 64 commands for AMSS site #
#define OSW_AMSS_ID_MIN 0x360
#define OSW_AMSS_ID_MAX 0x39f

#define OSW_CW_ID 0x3a0

#define OSW_CHAN_BAND_3 0x3be // Bandplan "Range" 3

#define OSW_SYS_NETSTAT 0x3bf
#define OSW_SYS_STATUS 0x3c0

#define OSW_CHAN_BAND_4_MIN 0x3c1 // Bandplan Range 4
#define OSW_CHAN_BAND_4_MAX 0x3fe // Bandplan Range 4

#define OSW_MAX 0x3ff

struct osw_stru {
  unsigned short cmd;
  unsigned short id;
  int status;
  int full_address;
  long address;
  int grp;
};

class SmartnetParser : public TrunkParser {
  int lastcmd;
  long lastaddress;

public:
  struct osw_stru stack[5];
  short numStacked;
  short numConsumed;
  SmartnetParser();
  void print_osw(std::string s);
  double getfreq(int cmd, System *system);
  bool is_chan_outbound(int cmd, System *system);
  bool is_chan_inbound_obt(int cmd, System *system);
  bool is_first_normal(int cmd, System *system);
  std::vector<TrunkMessage> parse_message(std::string s, System *system);
};
#endif
