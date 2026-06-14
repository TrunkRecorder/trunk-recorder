#include "dmr_parser.h"

#include "../formatter.h"
#include "system.h"
#include <boost/log/trivial.hpp>
#include <ctype.h>
#include <iomanip>
#include <sstream>

// OP25 m_type message identifiers (mirror of op25_repeater/lib/op25_msg_types.h
// which is not part of the installed include set). Keep these in sync.
static const int16_t M_DMR_TIMEOUT      = -1;
static const int16_t M_DMR_CACH_SLC     =  0;
static const int16_t M_DMR_CACH_CSBK    =  1;
static const int16_t M_DMR_SLOT_PI      =  2;
static const int16_t M_DMR_SLOT_VLC     =  3;
static const int16_t M_DMR_SLOT_TLC     =  4;
static const int16_t M_DMR_SLOT_CSBK    =  5;
static const int16_t M_DMR_SLOT_MBC     =  6;
static const int16_t M_DMR_SLOT_ELC     =  7;
static const int16_t M_DMR_SLOT_ERC     =  8;
static const int16_t M_DMR_SLOT_ESB     =  9;

static const int16_t PROTOCOL_DMR        = 1;

// Helpers --------------------------------------------------------------------

static uint32_t u24_be(const uint8_t *p) {
  return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)p[2];
}

static std::string hex_dump(const uint8_t *p, size_t n) {
  std::ostringstream os;
  os << std::hex << std::setfill('0');
  for (size_t i = 0; i < n; i++) {
    os << std::setw(2) << (int)p[i];
  }
  return os.str();
}

DmrParser::DmrParser() {}

TrunkMessage DmrParser::blank_message(System *system) {
  TrunkMessage m;
  m.message_type = UNKNOWN;
  m.meta = "";
  m.freq = 0;
  m.talkgroup = 0;
  m.encrypted = false;
  m.emergency = false;
  m.duplex = false;
  m.mode = false;
  m.priority = 0;
  m.tdma_slot = 0;
  m.phase2_tdma = false;
  m.source = -1;
  m.sys_num = system ? system->get_sys_num() : 0;
  m.sys_id = 0;
  m.sys_rfss = 0;
  m.sys_site_id = 0;
  m.nac = 0;
  m.wacn = 0;
  m.opcode = 0;
  m.patch_data.sg = 0;
  m.patch_data.ga1 = 0;
  m.patch_data.ga2 = 0;
  m.patch_data.ga3 = 0;
  return m;
}

double DmrParser::lcn_to_freq(System *system, int lcn) {
  if (!system) return 0;
  return system->get_lcn_freq(lcn);
}

std::vector<TrunkMessage> DmrParser::parse_message(gr::message::sptr msg, System *system) {
  std::vector<TrunkMessage> out;

  // OP25 packs:  type() = (m_proto << 16) | m_type   (both signed 16-bit)
  //              arg1() = (rxid << 1) | slot
  int16_t m_proto = (int16_t)((msg->type() >> 16) & 0xFFFF);
  int16_t m_type  = (int16_t)( msg->type()        & 0xFFFF);
  int rxid = ((int)msg->arg1()) >> 1;
  int slot = ((int)msg->arg1()) & 0x1;
  std::string buf = msg->to_string();
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(buf.data());
  size_t nbytes = buf.size();

  // DMR proto is 1. Timeout messages come through with m_proto=0 / m_type=-1.
  if (m_proto != PROTOCOL_DMR && m_type != M_DMR_TIMEOUT) {
    return out;
  }
  if (m_type == M_DMR_TIMEOUT) {
    BOOST_LOG_TRIVIAL(debug) << "[" << (system ? system->get_short_name() : "dmr") << "] sync timeout";
    return out;
  }

  switch (m_type) {
    case M_DMR_CACH_SLC:
      if (nbytes >= 4) {
        return decode_cach_slc(bytes, rxid, system);
      }
      break;
    case M_DMR_SLOT_CSBK:
    case M_DMR_SLOT_MBC:
      if (nbytes >= 10) {
        return decode_csbk(bytes, slot, rxid, system);
      }
      break;
    case M_DMR_SLOT_VLC:
      if (nbytes >= 9) {
        return decode_vlc(bytes, slot, system, /*terminator=*/false);
      }
      break;
    case M_DMR_SLOT_TLC:
      if (nbytes >= 9) {
        return decode_vlc(bytes, slot, system, /*terminator=*/true);
      }
      break;
    case M_DMR_SLOT_ELC:
      // Embedded LC during a voice transmission — same layout as VLC, mostly
      // useful for refreshing src on long calls.
      if (nbytes >= 9) {
        return decode_vlc(bytes, slot, system, /*terminator=*/false);
      }
      break;
    case M_DMR_SLOT_PI:
      // PI header: byte[0]=algId, byte[2]=keyId, byte[3..6]=MI, byte[7..9]=dst.
      // For now we just log; an UPDATE message could mark the active call
      // encrypted when we wire it up to the call manager.
      if (nbytes >= 10) {
        BOOST_LOG_TRIVIAL(debug) << "[" << (system ? system->get_short_name() : "dmr")
                                 << "] PI slot " << slot << " algId 0x" << std::hex
                                 << (int)bytes[0] << " keyId 0x" << (int)bytes[2]
                                 << " dst " << std::dec << u24_be(bytes + 7);
      }
      break;
    default:
      break;
  }
  return out;
}

// ----------------------------------------------------------------------------
// CACH SLC (Slow Link Control) — one of the strongest variant fingerprints
// we have. SLCO=9 / 10 mark a Connect Plus voice/control channel; SLCO=15
// announces the current Capacity Plus rest LCN.
//
// Wire layout (4 bytes):
//   byte[0] = SLCO
//   byte[1..3] = SLC data (interpretation depends on SLCO)
std::vector<TrunkMessage> DmrParser::decode_cach_slc(const uint8_t *slc, int rxid, System *system) {
  std::vector<TrunkMessage> out;
  uint8_t slco = slc[0];

  if (slco == 0) {
    // Idle slot — common, not worth a message.
    return out;
  }

  if (slco == 1) {
    // Activity update: byte[0]=SLCO, byte[1] = TS1/TS2 act nibbles,
    // byte[2..3] = hashes. Pure status; no TrunkMessage.
    return out;
  }

  if (slco == 9 || slco == 10) {
    // Connect Plus: byte[1..2] = netId(12)+siteId(8) (packed across bytes)
    int netId  = ((int)slc[1] << 4) | (slc[2] >> 4);
    int siteId = (((int)slc[2] & 0xF) << 4) | (slc[3] >> 4);
    TrunkMessage m = blank_message(system);
    m.message_type = (slco == 10) ? CONTROL_CHANNEL : STATUS;
    m.sys_id = ((unsigned long)netId << 16) | (unsigned long)siteId;
    m.sys_site_id = siteId;
    std::ostringstream os;
    os << "CACH SLC " << (slco == 10 ? "CC" : "VC")
       << " ConnectPlus netId=" << netId << " siteId=" << siteId;
    m.meta = os.str();
    BOOST_LOG_TRIVIAL(debug) << "[" << (system ? system->get_short_name() : "dmr") << "] " << os.str();
    if (system) system->set_dmr_variant("connect_plus");
    out.push_back(m);
    return out;
  }

  if (slco == 15) {
    // Capacity Plus rest channel announcement: byte[1] = current rest LCN.
    int rest_lcn = slc[1];
    TrunkMessage m = blank_message(system);
    m.message_type = STATUS;
    m.opcode = 15;
    std::ostringstream os;
    os << "CACH SLC CapPlus rest_lcn=" << rest_lcn;
    m.meta = os.str();
    BOOST_LOG_TRIVIAL(debug) << "[" << (system ? system->get_short_name() : "dmr") << "] " << os.str();
    if (system) {
      system->set_dmr_variant("capacity_plus");
      system->set_dmr_rest_lcn(rest_lcn);
    }
    out.push_back(m);
    return out;
  }

  BOOST_LOG_TRIVIAL(debug) << "[" << (system ? system->get_short_name() : "dmr")
                           << "] CACH SLC SLCO=" << (int)slco
                           << " data=" << hex_dump(slc, 4);
  return out;
}

// ----------------------------------------------------------------------------
// CSBK / MBC decoding. Wire layout (10 bytes / 80 bits):
//   byte[0] = LB(1)|PF(1)|CSBKO(6)
//   byte[1] = FID
//   byte[2..9] = 64-bit DATA
std::vector<TrunkMessage> DmrParser::decode_csbk(const uint8_t *csbk, int slot, int rxid, System *system) {
  std::vector<TrunkMessage> out;
  uint8_t csbko = csbk[0] & 0x3F;
  uint8_t fid   = csbk[1];
  uint16_t key = ((uint16_t)csbko << 8) | fid;
  const uint8_t *d = csbk + 2;          // 8 bytes of payload data
  std::string short_name = system ? system->get_short_name() : std::string("dmr");

  switch (key) {
    // ---- MOTOTRBO Connect Plus (FID=0x06) --------------------------------
    case 0x0106: {  // Neighbors — control channel only
      TrunkMessage m = blank_message(system);
      m.message_type = STATUS;
      m.opcode = key;
      std::ostringstream os;
      os << "ConnectPlus Neighbors:";
      // five 6-bit neighbour site IDs packed in d[0..4]
      for (int i = 0; i < 5; i++) os << " " << (int)d[i];
      m.meta = os.str();
      BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] " << os.str();
      if (system) system->set_dmr_variant("connect_plus");
      out.push_back(m);
      return out;
    }
    case 0x0306: {  // Connect Plus Channel Grant
      uint32_t src   = u24_be(d + 0);        // bits 16..39
      uint32_t group = u24_be(d + 3);        // bits 40..63
      uint8_t  lcn   = (d[6] >> 4) & 0xF;    // bits 64..67
      uint8_t  tslot = (d[6] >> 3) & 0x1;    // bit 68
      double freq = lcn_to_freq(system, lcn);
      TrunkMessage m = blank_message(system);
      m.message_type = GRANT;
      m.opcode = key;
      m.talkgroup = group;
      m.source = src;
      m.tdma_slot = tslot;
      m.freq = freq;
      std::ostringstream os;
      os << "ConnectPlus Grant src=" << src << " tg=" << group
         << " lcn=" << (int)lcn << " slot=" << (int)tslot
         << " freq=" << format_freq(freq);
      m.meta = os.str();
      BOOST_LOG_TRIVIAL(info) << "[" << short_name << "] " << os.str();
      if (freq == 0) {
        BOOST_LOG_TRIVIAL(warning) << "[" << short_name << "] dropping grant: LCN "
                                   << (int)lcn << " not in lcn_freq_table";
        return std::vector<TrunkMessage>();
      }
      if (system) system->set_dmr_variant("connect_plus");
      out.push_back(m);
      return out;
    }
    case 0x1006: case 0x1106: case 0x1206: case 0x1806: {
      // Registration / Affiliation: log only — call manager has no use yet
      BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] ConnectPlus aff/reg op=0x"
                               << std::hex << (int)csbko
                               << " data=" << hex_dump(d, 8);
      return out;
    }

    // ---- MOTOTRBO Capacity Plus (FID=0x10) -------------------------------
    case 0x3B10: {  // Sys/Sites/TS — carries rest LCN announcement
      uint8_t rest = d[0] & 0x1F;
      uint8_t bcn  = (d[1] >> 7) & 0x1;
      uint8_t site = (d[1] >> 3) & 0xF;
      TrunkMessage m = blank_message(system);
      m.message_type = STATUS;
      m.opcode = key;
      m.sys_site_id = site;
      std::ostringstream os;
      os << "CapPlus Sys/Sites rest_lcn=" << (int)rest
         << " beacon=" << (int)bcn << " site=" << (int)site;
      m.meta = os.str();
      BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] " << os.str();
      if (system) {
        system->set_dmr_variant("capacity_plus");
        system->set_dmr_rest_lcn(rest);
      }
      out.push_back(m);
      return out;
    }
    case 0x3D10:  // Preamble
    case 0x3E10:  // Site Status
      BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] CapPlus op=0x"
                               << std::hex << (int)csbko
                               << " data=" << hex_dump(d, 8);
      return out;

    // ---- MOTOTRBO Capacity Max extensions (FID=0x10) ---------------------
    case 0x1910: {  // Cap Max ALOHA — control-channel beacon
      TrunkMessage m = blank_message(system);
      m.message_type = STATUS;
      m.opcode = key;
      m.meta = "CapMax ALOHA";
      if (system) system->set_dmr_variant("capacity_max");
      BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] " << m.meta;
      out.push_back(m);
      return out;
    }
    case 0x2110: case 0x2210: {
      // Cap Max Voice Channel Update — Open Mode / Advantage Mode. Layout
      // per SDRTrunk:
      //   d[0..2] = TG/dst (24-bit)
      //   d[3..5] = src (24-bit)
      //   d[6] high nibble = LCN; d[6] bit 3 = slot
      uint32_t dst = u24_be(d + 0);
      uint32_t src = u24_be(d + 3);
      uint8_t  lcn = (d[6] >> 4) & 0xF;
      uint8_t  tslot = (d[6] >> 3) & 0x1;
      double freq = lcn_to_freq(system, lcn);
      TrunkMessage m = blank_message(system);
      m.message_type = (key == 0x2210) ? UPDATE : GRANT;
      m.opcode = key;
      m.talkgroup = dst;
      m.source = src;
      m.tdma_slot = tslot;
      m.freq = freq;
      std::ostringstream os;
      os << ((key == 0x2210) ? "CapMax VC-Update " : "CapMax VC-Grant ")
         << "src=" << src << " tg=" << dst << " lcn=" << (int)lcn
         << " slot=" << (int)tslot << " freq=" << format_freq(freq);
      m.meta = os.str();
      BOOST_LOG_TRIVIAL(info) << "[" << short_name << "] " << os.str();
      if (freq == 0) {
        BOOST_LOG_TRIVIAL(warning) << "[" << short_name
                                   << "] dropping Cap Max event: LCN "
                                   << (int)lcn << " not mapped";
        return std::vector<TrunkMessage>();
      }
      if (system) system->set_dmr_variant("capacity_max");
      out.push_back(m);
      return out;
    }

    // ---- ETSI Tier III standard (FID=0x00) -------------------------------
    case 0x1900: {  // ALOHA — control channel beacon
      TrunkMessage m = blank_message(system);
      m.message_type = STATUS;
      m.opcode = key;
      m.meta = "Tier III ALOHA";
      BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] " << m.meta;
      out.push_back(m);
      return out;
    }
    case 0x3100: case 0x3200: {  // Talkgroup Voice Grant / Broadcast TG Voice Grant
      uint32_t dst = u24_be(d + 0);
      uint32_t src = u24_be(d + 3);
      uint8_t  lcn = (d[6] >> 4) & 0xF;
      uint8_t  tslot = (d[6] >> 3) & 0x1;
      double freq = lcn_to_freq(system, lcn);
      TrunkMessage m = blank_message(system);
      m.message_type = GRANT;
      m.opcode = key;
      m.talkgroup = dst;
      m.source = src;
      m.tdma_slot = tslot;
      m.freq = freq;
      std::ostringstream os;
      os << "Tier III TG Grant src=" << src << " tg=" << dst
         << " lcn=" << (int)lcn << " slot=" << (int)tslot
         << " freq=" << format_freq(freq);
      m.meta = os.str();
      BOOST_LOG_TRIVIAL(info) << "[" << short_name << "] " << os.str();
      if (freq == 0) return std::vector<TrunkMessage>();
      out.push_back(m);
      return out;
    }
    case 0x3000: {  // Private Voice Channel Grant — unit to unit
      uint32_t dst = u24_be(d + 0);
      uint32_t src = u24_be(d + 3);
      uint8_t  lcn = (d[6] >> 4) & 0xF;
      uint8_t  tslot = (d[6] >> 3) & 0x1;
      double freq = lcn_to_freq(system, lcn);
      TrunkMessage m = blank_message(system);
      m.message_type = UU_V_GRANT;
      m.opcode = key;
      m.talkgroup = dst;
      m.source = src;
      m.tdma_slot = tslot;
      m.freq = freq;
      std::ostringstream os;
      os << "Tier III Private Voice Grant src=" << src << " dst=" << dst
         << " lcn=" << (int)lcn << " slot=" << (int)tslot
         << " freq=" << format_freq(freq);
      m.meta = os.str();
      BOOST_LOG_TRIVIAL(info) << "[" << short_name << "] " << os.str();
      if (freq == 0) return std::vector<TrunkMessage>();
      out.push_back(m);
      return out;
    }
    case 0x3900: {  // Move TSCC — CC frequency change
      TrunkMessage m = blank_message(system);
      m.message_type = STATUS;
      m.opcode = key;
      m.meta = "Tier III Move TSCC";
      BOOST_LOG_TRIVIAL(info) << "[" << short_name << "] " << m.meta;
      out.push_back(m);
      return out;
    }

    default:
      BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] CSBK csbko=0x"
                               << std::hex << (int)csbko << " fid=0x" << (int)fid
                               << " data=" << hex_dump(d, 8);
      break;
  }

  return out;
}

// ----------------------------------------------------------------------------
// Voice Link Control (VLC) and Terminator Link Control (TLC) — these ride
// on the voice channel itself and refresh src/dst during a call.
//
// Wire layout: 9 bytes ([0]=PF/FLCO, [1]=FID, [2]=SVCOPT, [3..5]=dst,
// [6..8]=src). Same shape across all variants.
std::vector<TrunkMessage> DmrParser::decode_vlc(const uint8_t *lc, int slot, System *system, bool terminator) {
  std::vector<TrunkMessage> out;
  uint8_t flco = lc[0] & 0x3F;
  uint8_t fid  = lc[1];
  uint8_t svc  = lc[2];
  uint32_t dst = u24_be(lc + 3);
  uint32_t src = u24_be(lc + 6);

  // TDULC in TrunkMessage's enum triggers a control-channel retune, which is
  // not what a DMR voice terminator means. Treat both VLC and TLC as plain
  // UPDATEs — the recorder pipeline detects end-of-call via its own slot
  // state machine on the voice channel, not via the trunking control channel.
  TrunkMessage m = blank_message(system);
  m.message_type = UPDATE;
  m.talkgroup = dst;
  m.source = src;
  m.tdma_slot = slot;
  m.encrypted = (svc & 0x80) != 0;
  m.emergency = (svc & 0x40) != 0;
  m.opcode = ((uint16_t)flco << 8) | fid;

  std::ostringstream os;
  os << (terminator ? "TLC" : "VLC") << " slot=" << slot
     << " flco=0x" << std::hex << (int)flco << " fid=0x" << (int)fid
     << " src=" << std::dec << src << " dst=" << dst;
  if (m.encrypted) os << " ENCRYPTED";
  if (m.emergency) os << " EMERGENCY";
  m.meta = os.str();

  std::string short_name = system ? system->get_short_name() : std::string("dmr");
  BOOST_LOG_TRIVIAL(debug) << "[" << short_name << "] " << os.str();
  out.push_back(m);
  return out;
}
