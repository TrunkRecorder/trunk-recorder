#include "formatter.h"
#include <boost/lexical_cast.hpp>

int frequency_format = 0;
bool statusAsString = true;

boost::format format_freq(double f) {
  if (frequency_format == 1)
    return boost::format("%10.6f MHz") % (f / 1000000.0);
  else if (frequency_format == 2)
    return boost::format("%.0f Hz") % f;
  else
    return boost::format("%e") % f;
}

std::string get_frequency_format() {
  if (frequency_format == 1)
    return "mhz";
  else if (frequency_format == 2)
    return "hz";
  else
    return "exp";
}

boost::format FormatSamplingRate(float f) {
  return boost::format("%.0f") % f;
}

boost::format format_time(float f) {
  return boost::format("%5.2f") % f;
}

std::string format_state(State state) {
  if (statusAsString) {
    if (state == MONITORING)    // Cyan
      return "\033[0;36mMonitoring\033[0m";
    else if (state == RECORDING)// Red
      return "\033[0;31mRecording\033[0m";
    else if (state == INACTIVE) // Blue
      return "\033[0;34mInactive\033[0m";
    else if (state == ACTIVE)   // Yellow
      return "\033[0;33mActive\033[0m";
    else if (state == IDLE)
      return "idle";
    else if (state == STOPPED)  // Purple
      return "\033[0;35mStopped\033[0m";
    else if (state == AVAILABLE)// Green
      return "\033[0;32mAvailable\033[0m";
    else if (state == IGNORE)
      return "Ignored";
    return "Unknown";
  }
  return boost::lexical_cast<std::string>(state);
}

std::string format_state(State state, MonitoringState monitoringState) {
  if (statusAsString) {
    if (state == MONITORING) {
      if (monitoringState == UNSPECIFIED)
        return "monitoring";
      else if (monitoringState == UNKNOWN_TG)
        return "monitoring : UNKNOWN TG";
      else if (monitoringState == IGNORED_TG)
        return "monitoring : IGNORED TG";
      else if (monitoringState == NO_SOURCE)
        return "monitoring : NO SOURCE COVERING FREQ";
      else if (monitoringState == NO_RECORDER)
        return "monitoring : NO RECORDER AVAILABLE";
      else if (monitoringState == ENCRYPTED)
        return "monitoring : ENCRYPTED";
      else if (monitoringState == DUPLICATE)
        return "monitoring : DUPLICATE";
      else if (monitoringState == SUPERSEDED)
        return "monitoring : SUPERSEDED";
      else
        return "monitoring";
    } else if (state == RECORDING)
      return "recording";
    else if (state == INACTIVE)
      return "inactive";
    else if (state == ACTIVE)
      return "active";
    else if (state == IDLE)
      return "idle";
    else if (state == STOPPED)
      return "stopped";
    else if (state == AVAILABLE)
      return "available";
    return "Unknown";
  }
  return boost::lexical_cast<std::string>(state);
}

std::string log_header(std::string short_name,long call_num, std::string talkgroup_display, double freq) {
  std::stringstream ss;
  ss << "[" << short_name << "]\t\033[0;34m" << call_num << "C\033[0m\tTG: " << talkgroup_display << "\tFreq: " << format_freq(freq) << "\033[0m\t";
  std::string s = ss.str();
  return s;
}
