#include "autotune.h"
#include "source.h"
#include "systems/system.h"
#include <iomanip>
#include <sstream>

/**
 * AutotuneManager
 *
 * This tracks the last 20 tuning errors as reported by the FLL band-edge filter and
 * calculates a running average for corrective offsets.  Operation is automatic and only
 * requires the config.json enables autoTune for the source(s).
 *
 * While the FFL filter can already handle minor corrections during a transmission, this
 * attempts to assist it by providing a more accurate starting point at the beginning of 
 * each call.
 *
 * Hz error readings are taken from the FLL filter.  Measurements represent the difference
 * between the detected signal and the center frequency of the filter.  (i.e. When the average
 * value is positive, Autotune will adjust the source "down".  When it is negative, Autotune
 * will adjust the source "up".)
 *
 * Each status display will show the current averages per source, as well as a suggested
 * "error" value for the config file to pre-correct future runs.
 *
 * Operation is currently limited to P25 voice channels (conventional/trunked) and P25
 * control channels, but may be expanded to other modes once it is verified that the
 * underlying FLL error readings are accurate.
 *
 * General Operation:
 * 1) When enabled, AutotuneManager collects error measurements for each source.
 * 2) Average tuning error is calculated from the last 20 measurements.
 * 3) When a recorder starts, the average error is applied as a frequency offset.
 * 4) When a recorder stops, the running average is updated with observed error measurements.
 * 5) On each status display, source parameters and suggested error values are shown.
 *
 * Control Channels:
 * 1) On each status display, a control channel is autotuned if it uses an enabled source.
 * 2) The average error for the source is used to calculate a new offset.
 * 3) The control channel is adjusted with finetune_control_freq() to avoid disruption.
 * 4) If a system tunes a new control channel, it will be autotuned without storing measurements.
 *
 */

/**
 * Constructor
 *
 * @param source Pointer to parent Source object
 */
AutotuneManager::AutotuneManager(Source *source)
    : parent_source(source) {
}

/**
 * Add a new frequency error measurement to the history and recalculate the average
 *
 * @param observed_error Frequency error from FLL band-edge filter (Hz)
 * @param current_offset Current autotune offset being applied (Hz)
 */
void AutotuneManager::add_error_measurement(int observed_error, int current_offset) {
  std::lock_guard<std::mutex> lock(history_mutex);

  // Add the total error (observed + current offset) to history, removing the oldest value
  int total_error = observed_error + current_offset;
  error_history.push_front(total_error);

  if (error_history.size() > MAX_HISTORY) {
    error_history.pop_back();
  }

  // Recalculate the average
  int total_errors = 0;
  std::ostringstream debug_errors;

  for (const auto &error : error_history) {
    total_errors += error;
    debug_errors << error << " ";
  }

  average_error = total_errors / static_cast<int>(error_history.size());
  BOOST_LOG_TRIVIAL(debug) << "Source: " << parent_source->get_num() << " - Errors: " << debug_errors.str() << " Avg: " << average_error;

  // Warn if calculated error offset > PPM_THRESHOLD
  double center_freq = parent_source->get_center();
  if (center_freq != 0.0) {
    double ppm_correction = static_cast<double>(average_error) / (center_freq / 1000000.0);
    if (std::abs(ppm_correction) > PPM_THRESHOLD) {
      BOOST_LOG_TRIVIAL(warning) << "Source " << parent_source->get_num()
                                 << " - AutoTune offset: " << average_error
                                 << " Hz exceeds " << PPM_THRESHOLD << " PPM (based on center freq "
                                 << center_freq / 1e6 << " MHz). "
                                 << "Verify initial offset used in config file.";
    }
  }
}

/**
 * Get the cached average frequency error
 *
 * @return Average error in Hz
 */
int AutotuneManager::get_average_error() const {
  return average_error;
}

/**
 * Get formatted status string for display in logging
 *
 * @return string showing autotune correction and suggested error value
 */
std::string AutotuneManager::get_status_string() const {
  int autotune_correction = get_average_error();
  double initial_error = parent_source->get_error();
  double total_error = initial_error - autotune_correction;
  int suggested_error = static_cast<int>(std::round(total_error / SUGGESTED_ERROR_ROUNDING) * SUGGESTED_ERROR_ROUNDING);

  std::ostringstream status;
  status << " \u001b[36mAutoTune: " << std::showpos << autotune_correction << std::noshowpos
         << " Hz, \"error\": " << suggested_error << "\u001b[0m";

  return status.str();
}

/**
 * Clear all error measurements
 */
void AutotuneManager::reset() {
  std::lock_guard<std::mutex> lock(history_mutex);
  error_history.clear();
}

/**
 * Coordinate autotune corrections for an active control channel
 *
 * @param system Pointer to the System managing the control channel
 * @param store_measurement Whether to store the current measurement in history (default: true)
 *                          Set to false when retuning to avoid storing junk values from retuned channels
 */
void autotune_control_channel(System *system, bool store_measurement) {
  Source *source = system->get_source();
  double control_channel_freq = system->get_current_control_channel();

  // Query the system control channel for the current frequency error and offset
  int fll_error = system->get_freq_error();
  int current_offset = system->get_autotune_offset();

  // Only add error measurements if requested (skip for retuned control channels)
  if (store_measurement) {
    source->autotune_manager->add_error_measurement(fll_error, current_offset);
  } else {
    BOOST_LOG_TRIVIAL(debug) << "Autotune: Skipping measurement storage for retuned control channel";
  }

  // Use the average for the next offset
  int new_offset = source->autotune_manager->get_average_error();

  BOOST_LOG_TRIVIAL(info) << "\t [\u001b[36m AutoTune:" << std::setw(5) << std::showpos << current_offset
                          << " Hz  TuningErr:" << std::setw(5) << fll_error
                          << " Hz  \u001b[0m->\u001b[36m  Next AutoTune:" << std::setw(5) << new_offset << std::noshowpos << " Hz \u001b[0m]";

  // Apply the corrected frequency and store the new offset
  double corrected_freq = control_channel_freq - new_offset;
  system->finetune_control_freq(corrected_freq);
  system->set_autotune_offset(new_offset);
}
