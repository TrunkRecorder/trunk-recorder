#ifndef DMR_PARSER_H
#define DMR_PARSER_H

#include <stdint.h>
#include <vector>
#include <gnuradio/message.h>

#include "parser.h"

class System;

// DmrParser turns the OP25 DMR messages emitted by frame_assembler (CACH SLC,
// CSBK, MBC, VLC, TLC, ELC, PI) into trunk-recorder TrunkMessages.
//
// It is *stateless across messages* — anything that has to persist (current
// CC frequency, rest-channel LCN, observed variant) lives on System. The
// parser only translates one wire-format event at a time.
//
// Variants understood today:
//   * ETSI Tier III / MOTOTRBO Capacity Max: standard CSBKOs 0x19, 0x28,
//     0x30, 0x31, 0x32, 0x39 with FID=0x00; Motorola Cap Max extensions
//     CSBKO 0x19/0x21/0x22 with FID=0x10.
//   * MOTOTRBO Connect Plus: CSBKO 0x01/0x03/0x10..0x18 with FID=0x06.
//   * MOTOTRBO Capacity Plus: CSBKO 0x3B/0x3D/0x3E with FID=0x10 + CACH SLC
//     SLCO=15 (rest channel announcement).
//
// Anything we don't recognise is logged at debug level as a hex dump for
// later analysis; an UNKNOWN TrunkMessage is *not* emitted (those would
// pollute call manager state).
class DmrParser : public TrunkParser {
public:
  DmrParser();
  std::vector<TrunkMessage> parse_message(gr::message::sptr msg, System *system);

private:
  std::vector<TrunkMessage> decode_csbk(const uint8_t *csbk, int slot, int rxid, System *system);
  std::vector<TrunkMessage> decode_cach_slc(const uint8_t *slc, int rxid, System *system);
  std::vector<TrunkMessage> decode_vlc(const uint8_t *lc, int slot, System *system, bool terminator);

  // Map an LCN id to a Hz frequency using the system's lcn_freq_table.
  // Returns 0 if the LCN is not mapped (caller should drop the grant).
  double lcn_to_freq(System *system, int lcn);

  TrunkMessage blank_message(System *system);
};

#endif // DMR_PARSER_H
