/**
 * Example Plugin: Dynamic Configuration
 * 
 * See example_config_plugin.h for documentation.
 */

#include "example_config_plugin.h"
#include <boost/log/trivial.hpp>

// Factory function
boost::shared_ptr<Plugin_Api> create_plugin() {
  return boost::make_shared<ExampleConfigPlugin>();
}

ExampleConfigPlugin::ExampleConfigPlugin()
    : m_auto_gain_enabled(false),
      m_target_signal_level(-30.0),
      m_gain_adjustment_step(1.0),
      m_poll_interval_ms(5000),
      m_config(nullptr),
      m_avg_signal_level(0.0),
      m_signal_sample_count(0) {
  m_last_adjustment_time = std::chrono::steady_clock::now();
}

int ExampleConfigPlugin::parse_config(json config_data) {
  // Parse plugin-specific configuration
  // Example config.json section:
  // {
  //   "name": "example_config_plugin",
  //   "library": "libexample_config_plugin.so",
  //   "autoGainEnabled": true,
  //   "targetSignalLevel": -25.0,
  //   "gainAdjustmentStep": 0.5,
  //   "pollIntervalMs": 10000
  // }
  
  m_auto_gain_enabled = config_data.value("autoGainEnabled", false);
  m_target_signal_level = config_data.value("targetSignalLevel", -30.0);
  m_gain_adjustment_step = config_data.value("gainAdjustmentStep", 1.0);
  m_poll_interval_ms = config_data.value("pollIntervalMs", 5000);
  
  BOOST_LOG_TRIVIAL(info) << "ExampleConfigPlugin: Auto-gain " 
                          << (m_auto_gain_enabled ? "enabled" : "disabled")
                          << ", target signal level: " << m_target_signal_level << " dB";
  
  return 0;
}

int ExampleConfigPlugin::init(Config *config, std::vector<Source *> sources, std::vector<System *> systems) {
  m_config = config;
  m_sources = sources;
  m_systems = systems;
  
  BOOST_LOG_TRIVIAL(info) << "ExampleConfigPlugin: Initialized with " 
                          << sources.size() << " sources and " 
                          << systems.size() << " systems";
  
  // Log initial configuration values
  log_current_config();
  
  return 0;
}

int ExampleConfigPlugin::start() {
  BOOST_LOG_TRIVIAL(info) << "ExampleConfigPlugin: Started";
  
  // Example: Register for configuration change notifications
  // (This is already done automatically via the plugin manager)
  
  return 0;
}

int ExampleConfigPlugin::stop() {
  BOOST_LOG_TRIVIAL(info) << "ExampleConfigPlugin: Stopped";
  return 0;
}

int ExampleConfigPlugin::poll_one() {
  // Check if enough time has passed since last adjustment
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_last_adjustment_time);
  
  if (elapsed.count() < m_poll_interval_ms) {
    return 0;
  }
  
  m_last_adjustment_time = now;
  
  // Example: Periodic gain adjustment based on average signal level
  if (m_auto_gain_enabled && m_signal_sample_count > 0) {
    for (size_t i = 0; i < m_sources.size(); i++) {
      adjust_gain_for_source(static_cast<int>(i));
    }
    
    // Reset signal tracking
    m_avg_signal_level = 0.0;
    m_signal_sample_count = 0;
  }
  
  return 0;
}

int ExampleConfigPlugin::on_config_change(const ConfigChangeInfo& change) {
  // React to configuration changes made by other plugins or external sources
  
  std::string change_type;
  switch (change.type) {
    case ConfigCommandType::SET_SOURCE_GAIN:
      change_type = "Source Gain";
      break;
    case ConfigCommandType::SET_SOURCE_ERROR:
      change_type = "Source Error";
      break;
    case ConfigCommandType::SET_SYSTEM_SQUELCH_DB:
      change_type = "System Squelch";
      break;
    case ConfigCommandType::SET_CALL_TIMEOUT:
      change_type = "Call Timeout";
      break;
    default:
      change_type = "Other";
      break;
  }
  
  BOOST_LOG_TRIVIAL(debug) << "ExampleConfigPlugin: Configuration changed - "
                           << change_type << " (target=" << change.target_id << ")"
                           << " by " << change.requester;
  
  // Example: If squelch is changed, we might want to adjust gain accordingly
  if (change.type == ConfigCommandType::SET_SYSTEM_SQUELCH_DB) {
    // Plugin could react to squelch changes here
  }
  
  return 0;
}

int ExampleConfigPlugin::call_end(Call_Data_t call_info) {
  // Track signal quality for auto-gain adjustment
  if (m_auto_gain_enabled && call_info.signal != 0) {
    // Update running average
    m_avg_signal_level = ((m_avg_signal_level * m_signal_sample_count) + call_info.signal) 
                         / (m_signal_sample_count + 1);
    m_signal_sample_count++;
    
    BOOST_LOG_TRIVIAL(trace) << "ExampleConfigPlugin: Signal sample " 
                             << call_info.signal << " dB, avg: " 
                             << m_avg_signal_level << " dB";
  }
  
  return 0;
}

void ExampleConfigPlugin::adjust_gain_for_source(int source_num) {
  ConfigurationService* config_service = get_config_service();
  if (!config_service) {
    BOOST_LOG_TRIVIAL(warning) << "ExampleConfigPlugin: ConfigurationService not available";
    return;
  }
  
  // Get current gain
  auto current_gain = config_service->get_source_gain(source_num);
  if (!current_gain.has_value()) {
    return;
  }
  
  double gain = current_gain.value();
  double signal_diff = m_avg_signal_level - m_target_signal_level;
  
  // Adjust gain based on signal difference
  // If signal is too weak (below target), increase gain
  // If signal is too strong (above target), decrease gain
  double new_gain = gain;
  
  if (signal_diff < -3.0) {
    // Signal too weak, increase gain
    new_gain = gain + m_gain_adjustment_step;
  } else if (signal_diff > 3.0) {
    // Signal too strong, decrease gain
    new_gain = gain - m_gain_adjustment_step;
  } else {
    // Signal within acceptable range
    return;
  }
  
  // Clamp gain to valid range
  new_gain = std::max(0.0, std::min(100.0, new_gain));
  
  if (new_gain == gain) {
    return;  // No change needed
  }
  
  BOOST_LOG_TRIVIAL(info) << "ExampleConfigPlugin: Adjusting source " << source_num 
                          << " gain from " << gain << " to " << new_gain
                          << " (avg signal: " << m_avg_signal_level << " dB)";
  
  // Submit the gain change request
  config_service->set_source_gain(source_num, new_gain, "ExampleConfigPlugin",
    [source_num](ConfigResult result, const std::string& message) {
      if (result == ConfigResult::SUCCESS) {
        BOOST_LOG_TRIVIAL(debug) << "ExampleConfigPlugin: Gain change for source " 
                                 << source_num << " applied successfully";
      } else {
        BOOST_LOG_TRIVIAL(warning) << "ExampleConfigPlugin: Gain change failed: " << message;
      }
    });
}

void ExampleConfigPlugin::log_current_config() {
  ConfigurationService* config_service = get_config_service();
  if (!config_service) {
    return;
  }
  
  BOOST_LOG_TRIVIAL(info) << "ExampleConfigPlugin: Current configuration snapshot:";
  BOOST_LOG_TRIVIAL(info) << "  Call timeout: " << config_service->get_call_timeout() << " seconds";
  
  for (size_t i = 0; i < m_sources.size(); i++) {
    auto gain = config_service->get_source_gain(static_cast<int>(i));
    auto error = config_service->get_source_error(static_cast<int>(i));
    
    BOOST_LOG_TRIVIAL(info) << "  Source " << i << ": gain=" 
                            << (gain.has_value() ? std::to_string(gain.value()) : "N/A")
                            << ", error=" 
                            << (error.has_value() ? std::to_string(error.value()) : "N/A");
  }
  
  for (size_t i = 0; i < m_systems.size(); i++) {
    System* sys = m_systems[i];
    auto squelch = config_service->get_system_squelch(sys->get_sys_num());
    
    BOOST_LOG_TRIVIAL(info) << "  System " << sys->get_short_name() << ": squelch=" 
                            << (squelch.has_value() ? std::to_string(squelch.value()) : "N/A") << " dB";
  }
}

