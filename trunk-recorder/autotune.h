#ifndef AUTOTUNE_H
#define AUTOTUNE_H

#include <boost/log/trivial.hpp>
#include <cmath>
#include <deque>
#include <mutex>

class Source;
class System;

class AutotuneManager {
private:
  Source *parent_source;
  std::mutex history_mutex;

  std::deque<int> error_history; // Last 20 error measurements (in Hz)
  int average_error = 0;         // Running average correction value (in Hz)

  static const size_t MAX_HISTORY = 20;               // Maximum number of error measurements to keep
  static constexpr double PPM_THRESHOLD = 3.5;        // Warning threshold in PPM
  static constexpr int SUGGESTED_ERROR_ROUNDING = 10; // Round suggested error to nearest X Hz

public:
  explicit AutotuneManager(Source *source);

  void add_error_measurement(int observed_error, int current_offset);
  int get_average_error() const;
  std::string get_status_string() const;
  void reset();
};

void autotune_control_channel(System *system, bool store_measurement = true);

#endif // AUTOTUNE_H
