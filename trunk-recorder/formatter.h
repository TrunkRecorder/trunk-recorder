#ifndef FORMATTER_H
#define FORMATTER_H

#include "state.h"
#include <boost/format.hpp>
#include <string>

// ANSI color codes
namespace Color {
  constexpr const char* RST = "\033[0m";     // Reset
  constexpr const char* BLK = "\033[0;30m";  // Black
  constexpr const char* RED = "\033[0;31m";  // Red
  constexpr const char* GRN = "\033[0;32m";  // Green
  constexpr const char* YEL = "\033[0;33m";  // Yellow
  constexpr const char* BLU = "\033[0;34m";  // Blue
  constexpr const char* MAG = "\033[0;35m";  // Magenta
  constexpr const char* CYN = "\033[0;36m";  // Cyan
  constexpr const char* WHT = "\033[0;37m";  // White
  
  constexpr const char* BBLK = "\033[0;90m"; // Bright Black (Gray)
  constexpr const char* BRED = "\033[0;91m"; // Bright Red
  constexpr const char* BGRN = "\033[0;92m"; // Bright Green
  constexpr const char* BYEL = "\033[0;93m"; // Bright Yellow
  constexpr const char* BBLU = "\033[0;94m"; // Bright Blue
  constexpr const char* BMAG = "\033[0;95m"; // Bright Magenta
  constexpr const char* BCYN = "\033[0;96m"; // Bright Cyan
  constexpr const char* BWHT = "\033[0;97m"; // Bright White
}

extern boost::format format_freq(double f);
extern boost::format FormatSamplingRate(float f);
extern boost::format format_time(float f);
extern std::string format_state(State state, MonitoringState monitoringState = UNSPECIFIED);
std::string get_frequency_format();
extern std::string log_header(std::string short_name,long call_num, std::string talkgroup_display, double freq);
extern int frequency_format;
extern bool statusAsString;

#endif // FORMATTER_H
