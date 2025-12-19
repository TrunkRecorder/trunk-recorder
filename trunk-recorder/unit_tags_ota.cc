// Handling for OTA (Over-The-Air) unit tags
// Motorola Alias Algorithm decode_mot_alias() - Copyright 2024 Ilya Smirnov / @ilyacodes. All rights reserved.

#include "unit_tags_ota.h"

#include <boost/log/trivial.hpp>
#include <sstream>
#include <iomanip>
#include <map>
#include <array>

// OTA Motorola alias handling tools

OTAAlias UnitTagsOTA::decode_motorola_alias(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages) {
  // Validate input
  if (messages <= 0 || messages >= (int)alias_buffer.size()) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: invalid message count " << messages;
    return OTAAlias();
  }

  // Assemble the payload from all message fragments
  std::string payload_hex = assemble_payload(alias_buffer, messages);
  if (payload_hex.length() < 32) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: assembled payload too short (" << payload_hex.length() << " chars)";
    return OTAAlias();
  }

  // Extract components
  std::string tg = payload_hex.substr(0, 4);           // Talkgroup ID
  std::string msg_count = payload_hex.substr(4, 2);    // # of Data blocks
  std::string unknown_4 = payload_hex.substr(6, 4);    // [unknown] - format?
  std::string sequence_id = payload_hex.substr(10, 1); // Sequence ID char
  std::string unknown_3 = payload_hex.substr(11, 3);   // [unknown] - checksum?
  std::string wacn = payload_hex.substr(14, 5);        // WACN 
  std::string sys = payload_hex.substr(19, 3);         // System ID
  std::string radio = payload_hex.substr(22, 6);       // Radio ID
  std::string length_code = payload_hex.substr(28, 2); // Alias length code (first 2 chars of alias)
  
  static const std::map<std::string, int> length_lookup = {
    {"94", 1}, {"32", 2}, {"95", 3}, {"9d", 4}, {"1b", 5}, {"77", 6}, {"b5", 7},
    {"6e", 8}, {"24", 9}, {"61", 10}, {"2d", 11}, {"7d", 12}, {"83", 13}, {"29", 14}
  };

  auto it = length_lookup.find(length_code);
  int alias_len = (it != length_lookup.end()) ? it->second : 0;

  if (alias_len == 0) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: unknown alias length code: " << length_code;
    return OTAAlias();
  }

  size_t required_len = 28 + alias_len * 4 + 4;
  if (payload_hex.length() < required_len) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: payload too short for alias length " << alias_len;
    return OTAAlias();
  }

  std::string alias_code = payload_hex.substr(28, alias_len * 4);
  std::string checksum = payload_hex.substr(28 + alias_len * 4, 4);

  // Convert radio ID to decimal
  unsigned long radio_decimal = std::stoul(radio, nullptr, 16);
  unsigned long tg_decimal = std::stoul(tg, nullptr, 16);

  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: WACN: " << wacn << ", SYS: " << sys << ", Radio: " << radio_decimal << " (0x" << radio << ")" << ", TG: " << tg_decimal << " (0x" << tg << ")";
  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: Alias Length: " << alias_len << ", Code: " << alias_code << ", Checksum: " << checksum;

  // Validate CRC
  std::string crc_payload = wacn + sys + radio + alias_code;
  if (!validate_crc(crc_payload, checksum)) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: CRC-16/GSM check failed";
    return OTAAlias();
  }

  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: CRC-16/GSM check passed";

  // Convert hex string to bytes
  std::vector<uint8_t> payload_bytes = hex_string_to_uint8_vector(crc_payload);
  if (payload_bytes.empty()) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: failed to parse hex payload";
    return OTAAlias();
  }

  // Need at least 8 bytes (7 byte offset + 1 byte minimum encoded data)
  if (payload_bytes.size() < 8) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: payload too short for decoding (" << payload_bytes.size() << " bytes)";
    return OTAAlias();
  }

  // Extract encoded portion (skip first 7 bytes)
  std::vector<int8_t> encoded(payload_bytes.begin() + 7, payload_bytes.end());

  std::string alias = decode_mot_alias(encoded);

  if (!alias.empty()) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: Decoded alias: '" << alias << "' for radio " << radio_decimal << " (0x" << radio << ")" << ", TG: " << tg_decimal << " (0x" << tg << ")";
    return OTAAlias(radio_decimal, alias, "MotoP25_FDMA", wacn, sys, tg_decimal);
  }

  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: Decrypt returned empty alias";
  return OTAAlias();
}

// Phase 1 FDMA: assemble payload hex string from message buffer
std::string UnitTagsOTA::assemble_payload(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages) {
  std::stringstream ss;

  if (alias_buffer[0].size() < 9) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: header too small";
    return "";
  }

  // Header block - extract bytes 2-8 (7 bytes) - discarding opcode/mfr (bytes 0-1)
  std::vector<uint8_t> header_slice(alias_buffer[0].begin() + 2, alias_buffer[0].begin() + 9);
  ss << uint8_vector_to_hex_string(header_slice);

  for (int i = 1; i <= messages; i++) {
    if (alias_buffer[i].size() < 9) {
      BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: alias_buffer[" << i << "] too small";
      return "";
    }

    // Bytes 3-8: extract all, then skip first char (upper nibble of byte 3)
    std::vector<uint8_t> data_slice(alias_buffer[i].begin() + 3, alias_buffer[i].begin() + 9);
    ss << uint8_vector_to_hex_string(data_slice).substr(1);
  }

  return ss.str();
}

// Phase 2 TDMA: Assemble payload from 17-byte MAC PDU messages
std::string UnitTagsOTA::assemble_payload_p2(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages) {
  std::stringstream ss;

  if (alias_buffer[0].size() < 17) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: header too small (" << alias_buffer[0].size() << " bytes)";
    return "";
  }

  // Header block - extract bytes 3-16 (14 bytes)
  std::vector<uint8_t> header_slice(alias_buffer[0].begin() + 3, alias_buffer[0].begin() + 17);
  ss << uint8_vector_to_hex_string(header_slice);

  // Data blocks - extract bytes 4-16 (13 bytes each)
  for (int i = 1; i <= messages; i++) {
    if (alias_buffer[i].size() < 17) {
      BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: alias_buffer[" << i << "] too small (" << alias_buffer[i].size() << " bytes)";
      return "";
    }
    // Bytes 4-16: extract all, then skip first char (sequence ID upper nibble)
    std::vector<uint8_t> data_slice(alias_buffer[i].begin() + 4, alias_buffer[i].begin() + 17);
    ss << uint8_vector_to_hex_string(data_slice).substr(1);
  }

  return ss.str();
}

// Phase 2 TDMA alias decode entry point
OTAAlias UnitTagsOTA::decode_motorola_alias_p2(const std::array<std::vector<uint8_t>, 10>& alias_buffer, int messages) {
  // Validate input
  if (messages <= 0 || messages >= (int)alias_buffer.size()) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: invalid message count " << messages;
    return OTAAlias();
  }

  // Assemble the payload from all message fragments using P2 format
  std::string payload_hex = assemble_payload_p2(alias_buffer, messages);
  if (payload_hex.length() < 30) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: assembled payload too short (" << payload_hex.length() << " chars)";
    return OTAAlias();
  }

  // Extract components
  std::string tg = payload_hex.substr(0, 4);           // Talkgroup ID
  std::string msg_count = payload_hex.substr(4, 2);    // # of Data blocks
  std::string unknown_4 = payload_hex.substr(6, 4);    // [unknown] - format?
  std::string sequence_id = payload_hex.substr(10, 1); // Sequence ID char
  std::string unknown_1 = payload_hex.substr(11, 1);   // [unknown] - check digit?
  std::string wacn = payload_hex.substr(12, 5);        // WACN
  std::string sys = payload_hex.substr(17, 3);         // System ID
  std::string radio = payload_hex.substr(20, 6);       // Radio ID
  std::string length_code = payload_hex.substr(26, 2); // Alias length code (first 2 chars of alias)

  static const std::map<std::string, int> length_lookup = {
    {"94", 1}, {"32", 2}, {"95", 3}, {"9d", 4}, {"1b", 5}, {"77", 6}, {"b5", 7},
    {"6e", 8}, {"24", 9}, {"61", 10}, {"2d", 11}, {"7d", 12}, {"83", 13}, {"29", 14}
  };

  auto it = length_lookup.find(length_code);
  int alias_len = (it != length_lookup.end()) ? it->second : 0;

  if (alias_len == 0) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: unknown alias length code: " << length_code;
    return OTAAlias();
  }

  size_t required_len = 26 + alias_len * 4 + 4;
  if (payload_hex.length() < required_len) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: payload too short for alias length " << alias_len;
    return OTAAlias();
  }

  std::string alias_code = payload_hex.substr(26, alias_len * 4);
  std::string checksum = payload_hex.substr(26 + alias_len * 4, 4);

  unsigned long radio_decimal = std::stoul(radio, nullptr, 16);
  unsigned long tg_decimal = std::stoul(tg, nullptr, 16);

  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: WACN: " << wacn << ", SYS: " << sys << ", Radio: " << radio_decimal << " (0x" << radio << ")" << ", TG: " << tg_decimal << " (0x" << tg << ")";
  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: Alias Length: " << alias_len << ", Code: " << alias_code << ", Checksum: " << checksum;

  // Validate CRC
  std::string crc_payload = wacn + sys + radio + alias_code;
  if (!validate_crc(crc_payload, checksum)) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: CRC-16/GSM check failed for " << crc_payload;
    return OTAAlias();
  }

  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: CRC-16/GSM check passed";

  // Convert hex string to bytes
  std::vector<uint8_t> payload_bytes = hex_string_to_uint8_vector(crc_payload);
  if (payload_bytes.empty()) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: failed to parse hex payload";
    return OTAAlias();
  }

  // Need at least 8 bytes (7 byte offset + 1 byte minimum encoded data)
  if (payload_bytes.size() < 8) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: payload too short for decoding (" << payload_bytes.size() << " bytes)";
    return OTAAlias();
  }

  // Extract encoded portion (skip first 7 bytes)
  std::vector<int8_t> encoded(payload_bytes.begin() + 7, payload_bytes.end());

  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: Encoded data size: " << encoded.size() << " bytes";

  std::string alias = decode_mot_alias(encoded);

  if (!alias.empty()) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: Decoded alias: '" << alias << "' for radio " << radio_decimal << " (0x" << radio << ")" << ", TG: " << tg_decimal << " (0x" << tg << ")";
    return OTAAlias(radio_decimal, alias, "MotoP25_TDMA", wacn, sys, tg_decimal);
  }

  BOOST_LOG_TRIVIAL(debug) << "MOTOROLA P2: Decrypt returned empty alias";
  return OTAAlias();
}

// Validate CRC-16/GSM checksum
bool UnitTagsOTA::validate_crc(const std::string& payload_hex, const std::string& checksum_hex) {
  // Convert hex string to bytes
  std::vector<uint8_t> payload_bytes = hex_string_to_uint8_vector(payload_hex);
  if (payload_bytes.empty()) return false;

  // Calculate CRC-16/GSM
  uint16_t crc = 0x0000;
  for (size_t j = 0; j < payload_bytes.size(); ++j) {
    uint8_t byte = static_cast<uint8_t>(payload_bytes[j]);
    crc ^= byte << 8;

    for (int i = 0; i < 8; i++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  uint16_t crc_calc = ~crc & 0xFFFF;

  // Parse expected checksum
  uint16_t expected_crc = 0;
  try {
    expected_crc = std::stoi(checksum_hex, nullptr, 16);
  } catch (const std::exception& e) {
    BOOST_LOG_TRIVIAL(debug) << "MOTOROLA: invalid checksum format: " << checksum_hex;
    return false;
  }

  return (crc_calc == expected_crc);
}

// De-obfuscate Motorola alias using custom algorithm
std::string UnitTagsOTA::decode_mot_alias(const std::vector<int8_t>& encoded) {

  static const uint8_t SUBSTITUTION_TABLE[256] = {
    0xd2, 0xf6, 0xd4, 0x2b, 0x63, 0x49, 0x94, 0x5e, 0xa7, 0x5c, 0x70, 0x69, 0xf7, 0x08, 0xb1, 0x7d,
    0x38, 0xcf, 0xcc, 0xd8, 0x51, 0x8f, 0xd5, 0x93, 0x6a, 0xf3, 0xef, 0x7e, 0xfb, 0x64, 0xf4, 0x35,
    0x27, 0x07, 0x31, 0x14, 0x87, 0x98, 0x76, 0x34, 0xca, 0x92, 0x33, 0x1b, 0x4f, 0x8c, 0x09, 0x40,
    0x32, 0x36, 0x77, 0x12, 0xd3, 0xc3, 0x01, 0xab, 0x72, 0x81, 0x95, 0xc9, 0xc0, 0xe9, 0x65, 0x52,
    0x24, 0x30, 0x1c, 0xdb, 0x88, 0xe8, 0x97, 0x9d, 0x58, 0x26, 0x04, 0x39, 0xac, 0x2a, 0x9e, 0xaa,
    0x25, 0xd7, 0xce, 0xeb, 0x96, 0xf5, 0x0e, 0x8d, 0xdc, 0xa9, 0x2f, 0xdd, 0x1f, 0xea, 0x91, 0xb7,
    0xd6, 0x89, 0x8b, 0xd1, 0xb0, 0x99, 0x13, 0x7a, 0xe7, 0x9a, 0xb5, 0x86, 0xff, 0x46, 0x85, 0xb2,
    0x73, 0xda, 0xbf, 0xd0, 0x71, 0xcb, 0x4d, 0x80, 0x15, 0x67, 0x16, 0x1a, 0x20, 0x8e, 0x45, 0x3e,
    0xf2, 0x2e, 0x66, 0x90, 0x74, 0x8a, 0x6f, 0x78, 0xbb, 0x53, 0x03, 0x11, 0x68, 0xcd, 0x44, 0x17,
    0x28, 0x5f, 0x1e, 0x84, 0x75, 0x79, 0x6e, 0x9b, 0x2c, 0xbe, 0x62, 0x2d, 0xf1, 0x7c, 0xb8, 0x83,
    0xd9, 0x4e, 0x6d, 0x02, 0x61, 0x3d, 0xa8, 0x06, 0xb9, 0xf8, 0x9c, 0x37, 0x3a, 0x23, 0xc1, 0x50,
    0xed, 0x9f, 0xaf, 0x3b, 0xbd, 0x82, 0xba, 0xa0, 0xdf, 0xc2, 0x47, 0x22, 0xf0, 0xee, 0xa1, 0xfe,
    0xa2, 0x10, 0x5b, 0x48, 0x57, 0xa3, 0x05, 0x60, 0x7b, 0x0d, 0xf9, 0x6c, 0xb3, 0x56, 0x4c, 0xbc,
    0x29, 0xa4, 0x0f, 0xec, 0xb6, 0xa5, 0xa6, 0x3c, 0x7f, 0x6b, 0xb4, 0x21, 0xad, 0xae, 0xc4, 0xc8,
    0xc5, 0x5d, 0xde, 0xe0, 0x1d, 0x19, 0x4b, 0xc6, 0x0c, 0x3f, 0x5a, 0xc7, 0xe1, 0x59, 0x55, 0x54,
    0x4a, 0x43, 0x42, 0xe2, 0xe3, 0xfa, 0x00, 0xe4, 0xe5, 0x18, 0x41, 0x0b, 0x0a, 0xe6, 0xfc, 0xfd,
  };

  // Precomputed inverses to avoid runtime calculation for each byte
  static const uint8_t MODULAR_INVERSE_ODD[128] = {
    0x01, 0xab, 0xcd, 0xb7, 0x39, 0xa3, 0xc5, 0xef, 0xf1, 0x1b, 0x3d, 0xa7, 0x29, 0x13, 0x35, 0xdf,
    0xe1, 0x8b, 0xad, 0x97, 0x19, 0x83, 0xa5, 0xcf, 0xd1, 0xfb, 0x1d, 0x87, 0x09, 0xf3, 0x15, 0xbf,
    0xc1, 0x6b, 0x8d, 0x77, 0xf9, 0x63, 0x85, 0xaf, 0xb1, 0xdb, 0xfd, 0x67, 0xe9, 0xd3, 0xf5, 0x9f,
    0xa1, 0x4b, 0x6d, 0x57, 0xd9, 0x43, 0x65, 0x8f, 0x91, 0xbb, 0xdd, 0x47, 0xc9, 0xb3, 0xd5, 0x7f,
    0x81, 0x2b, 0x4d, 0x37, 0xb9, 0x23, 0x45, 0x6f, 0x71, 0x9b, 0xbd, 0x27, 0xa9, 0x93, 0xb5, 0x5f,
    0x61, 0x0b, 0x2d, 0x17, 0x99, 0x03, 0x25, 0x4f, 0x51, 0x7b, 0x9d, 0x07, 0x89, 0x73, 0x95, 0x3f,
    0x41, 0xeb, 0x0d, 0xf7, 0x79, 0xe3, 0x05, 0x2f, 0x31, 0x5b, 0x7d, 0xe7, 0x69, 0x53, 0x75, 0x1f,
    0x21, 0xcb, 0xed, 0xd7, 0x59, 0xc3, 0xe5, 0x0f, 0x11, 0x3b, 0x5d, 0xc7, 0x49, 0x33, 0x55, 0xff,
  };

  std::vector<uint8_t> decoded(encoded.size(), 0);

  uint16_t accumulator = static_cast<uint16_t>(decoded.size());

  for (size_t i = 0; i < decoded.size(); ++i) {
    uint8_t encoded_byte = static_cast<uint8_t>(encoded[i]);
    
    uint16_t lcg_value = accumulator * 293 + 0x72E9;
    uint8_t substituted = SUBSTITUTION_TABLE[encoded_byte];
    uint8_t intermediate = substituted - static_cast<uint8_t>(lcg_value >> 8);

    uint8_t modulus = static_cast<uint8_t>(lcg_value | 0x1);
    uint8_t inverse = MODULAR_INVERSE_ODD[modulus >> 1];
    
    decoded[i] = intermediate * inverse;

    accumulator += encoded_byte + 1;
  }

  std::string alias;
  alias.reserve(decoded.size() / 2);
  
  for (size_t i = 0; i + 1 < decoded.size(); i += 2) {
    uint16_t codepoint = (static_cast<uint16_t>(decoded[i]) << 8) | decoded[i + 1];
    
    if (codepoint > 31 && codepoint < 128) {
      alias += static_cast<char>(codepoint);
    }
  }

  return alias;
}

// Convert vector of uint8_t to hex string
std::string UnitTagsOTA::uint8_vector_to_hex_string(const std::vector<uint8_t>& v)
{
    std::string result;
    result.reserve(v.size() * 2);

    static constexpr char hex[] = "0123456789abcdef";

    for (uint8_t c : v)
    {
        result.push_back(hex[c / 16]);
        result.push_back(hex[c % 16]);
    }

    return result;
}

// Convert hex string to vector of int8_t
std::vector<uint8_t> UnitTagsOTA::hex_string_to_uint8_vector(const std::string& hex_str) {
    std::vector<uint8_t> result;
    result.reserve(hex_str.length() / 2);
    for (size_t i = 0; i + 1 < hex_str.length(); i += 2) {
        int hi = (hex_str[i] <= '9') ? hex_str[i] - '0' : (hex_str[i] & 0xF) + 9;
        int lo = (hex_str[i+1] <= '9') ? hex_str[i+1] - '0' : (hex_str[i+1] & 0xF) + 9;
        result.push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return result;
}