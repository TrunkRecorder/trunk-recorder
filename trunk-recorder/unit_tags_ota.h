#ifndef UNIT_TAGS_OTA_H
#define UNIT_TAGS_OTA_H

#include <string>
#include <vector>
#include <array>
#include <cstdint>

// Result structure for OTA alias decode containing radio ID, alias text, and source
struct OTAAlias {
  bool success;           // Whether decode was successful
  long radio_id;          // Radio ID from the alias payload
  std::string alias;      // Decoded alias text
  std::string source;     // Decoder source (e.g. "MotoP25_TDMA", "MotoP25_FDMA", etc..)
  std::string wacn;       // WACN
  std::string sys;        // System ID
  long talkgroup_id;      // Talkgroup ID where alias was broadcast
  
  OTAAlias() : success(false), radio_id(0), alias(""), source(""), wacn(""), sys(""), talkgroup_id(0) {}
  OTAAlias(long id, const std::string& text, const std::string& src = "", 
           const std::string& w = "", const std::string& s = "", long tg = 0) 
    : success(true), radio_id(id), alias(text), source(src), wacn(w), sys(s), talkgroup_id(tg) {}
};

class UnitTagsOTA {
public:
  // Motorola OTA (Over-The-Air) alias decoding
  static OTAAlias decode_motorola_alias(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages);
  static OTAAlias decode_motorola_alias_p2(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages);

private:
  // Helper functions for Motorola alias decoding
  static std::string assemble_payload(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages);
  static std::string assemble_payload_p2(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages);
  static bool validate_crc(const std::string& payload_hex, const std::string& checksum_hex);
  static std::string decode_mot_alias(const std::vector<int8_t>& encoded_data);
  static std::string uint8_vector_to_hex_string(const std::vector<uint8_t>& v);
  static std::vector<uint8_t> hex_string_to_uint8_vector(const std::string& hex_str);
};

#endif // UNIT_TAGS_OTA_H
