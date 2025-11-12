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

std::string format_state(State state, MonitoringState monitoringState) {
  if (statusAsString) {
    std::stringstream ss;
    switch (state) {
      case MONITORING: ss << Color::CYN << "Monitoring" << Color::RST;
        switch (monitoringState) {
          case UNKNOWN_TG:   ss << ": UNKNOWN TG"; break;
          case IGNORED_TG:   ss << ": IGNORED TG"; break;
          case NO_SOURCE:    ss << ": " << Color::YEL << "NO SOURCE COVERING FREQ" << Color::RST; break;
          case NO_RECORDER:  ss << ": " << Color::YEL << "NO RECORDER AVAILABLE" << Color::RST; break;
          case ENCRYPTED:    ss << ": " << Color::RED << "ENCRYPTED" << Color::RST; break;
          case DUPLICATE:    ss << ": " << Color::CYN << "DUPLICATE" << Color::RST; break;
          case SUPERSEDED:   ss << ": " << Color::CYN << "SUPERSEDED" << Color::RST; break;
          default: break;  // UNSPECIFIED
        }
        break;

      case RECORDING:  ss << Color::RED << "Recording"  << Color::RST; break;
      case INACTIVE:   ss << Color::BLU << "Inactive"   << Color::RST; break;
      case ACTIVE:     ss << Color::YEL << "Active"     << Color::RST; break;
      case IDLE:       ss << "Idle"; break;
      case STOPPED:    ss << Color::MAG << "Stopped"    << Color::RST; break;
      case AVAILABLE:  ss << Color::GRN << "Available"  << Color::RST; break;
      case IGNORE:     ss << "Ignored"; break;
      default:         ss << "Unknown"; break;
    }
    return ss.str();
  }
  return boost::lexical_cast<std::string>(state);
}

std::string log_header(std::string short_name,long call_num, std::string talkgroup_display, double freq) {
  std::stringstream ss;
  ss << "[" << short_name << "]\t" << Color::BLU << call_num << "C" << Color::RST 
     << "\tTG: " << talkgroup_display << "\tFreq: " << format_freq(freq) << "\t";
  return ss.str();
}
