#include "config_service.h"
#include "formatter.h"
#include "source.h"
#include "systems/system.h"

#include <sstream>

// Global configuration service instance
ConfigurationService* g_config_service = nullptr;

void init_config_service(Config* config, std::vector<Source*>* sources, std::vector<System*>* systems) {
  if (g_config_service == nullptr) {
    g_config_service = new ConfigurationService();
  }
  g_config_service->init(config, sources, systems);
}

void shutdown_config_service() {
  if (g_config_service != nullptr) {
    delete g_config_service;
    g_config_service = nullptr;
  }
}

ConfigurationService::ConfigurationService()
    : m_config(nullptr), m_sources(nullptr), m_systems(nullptr), m_initialized(false) {
}

ConfigurationService::~ConfigurationService() {
}

void ConfigurationService::init(Config* config, std::vector<Source*>* sources, std::vector<System*>* systems) {
  m_config = config;
  m_sources = sources;
  m_systems = systems;
  m_initialized = true;
  BOOST_LOG_TRIVIAL(info) << "ConfigurationService initialized with " << sources->size() << " sources and " << systems->size() << " systems";
}

// ============================================================
// Command submission
// ============================================================

bool ConfigurationService::submit_command(const ConfigCommand& cmd) {
  if (!m_initialized) {
    BOOST_LOG_TRIVIAL(error) << "ConfigurationService: Cannot submit command - not initialized";
    return false;
  }

  std::lock_guard<std::mutex> lock(m_queue_mutex);
  m_pending_commands.push(cmd);
  return true;
}

// Convenience methods for Source parameters

bool ConfigurationService::set_source_gain(int source_num, double gain, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SOURCE_GAIN;
  cmd.target_id = source_num;
  cmd.value = gain;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_source_gain_by_name(int source_num, const std::string& name, double gain, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SOURCE_GAIN_BY_NAME;
  cmd.target_id = source_num;
  cmd.param_name = name;
  cmd.value = gain;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_source_error(int source_num, double error, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SOURCE_ERROR;
  cmd.target_id = source_num;
  cmd.value = error;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_source_ppm(int source_num, double ppm, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SOURCE_PPM;
  cmd.target_id = source_num;
  cmd.value = ppm;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_source_gain_mode(int source_num, bool agc, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SOURCE_GAIN_MODE;
  cmd.target_id = source_num;
  cmd.value = agc;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_source_antenna(int source_num, const std::string& antenna, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SOURCE_ANTENNA;
  cmd.target_id = source_num;
  cmd.value = antenna;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_source_signal_detector_threshold(int source_num, double threshold, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SOURCE_SIGNAL_DETECTOR_THRESHOLD;
  cmd.target_id = source_num;
  cmd.value = threshold;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

// Convenience methods for System parameters

bool ConfigurationService::set_system_squelch(int sys_num, double squelch_db, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_SQUELCH_DB;
  cmd.target_id = sys_num;
  cmd.value = squelch_db;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_analog_levels(int sys_num, double levels, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_ANALOG_LEVELS;
  cmd.target_id = sys_num;
  cmd.value = levels;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_digital_levels(int sys_num, double levels, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_DIGITAL_LEVELS;
  cmd.target_id = sys_num;
  cmd.value = levels;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_min_duration(int sys_num, double duration, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_MIN_DURATION;
  cmd.target_id = sys_num;
  cmd.value = duration;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_max_duration(int sys_num, double duration, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_MAX_DURATION;
  cmd.target_id = sys_num;
  cmd.value = duration;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_min_tx_duration(int sys_num, double duration, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_MIN_TX_DURATION;
  cmd.target_id = sys_num;
  cmd.value = duration;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_record_unknown(int sys_num, bool record, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_RECORD_UNKNOWN;
  cmd.target_id = sys_num;
  cmd.value = record;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_hide_encrypted(int sys_num, bool hide, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_HIDE_ENCRYPTED;
  cmd.target_id = sys_num;
  cmd.value = hide;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_hide_unknown(int sys_num, bool hide, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_HIDE_UNKNOWN;
  cmd.target_id = sys_num;
  cmd.value = hide;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_conversation_mode(int sys_num, bool mode, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_CONVERSATION_MODE;
  cmd.target_id = sys_num;
  cmd.value = mode;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_tau(int sys_num, float tau, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_TAU;
  cmd.target_id = sys_num;
  cmd.value = static_cast<double>(tau);
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_max_dev(int sys_num, int max_dev, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_MAX_DEV;
  cmd.target_id = sys_num;
  cmd.value = max_dev;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_system_filter_width(int sys_num, double width, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_SYSTEM_FILTER_WIDTH;
  cmd.target_id = sys_num;
  cmd.value = width;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

bool ConfigurationService::set_call_timeout(double timeout, const std::string& requester, ConfigCallback callback) {
  ConfigCommand cmd;
  cmd.type = ConfigCommandType::SET_CALL_TIMEOUT;
  cmd.target_id = -1;  // Global config
  cmd.value = timeout;
  cmd.requester = requester;
  cmd.callback = callback;
  return submit_command(cmd);
}

// ============================================================
// Getters
// ============================================================

Source* ConfigurationService::find_source(int source_num) {
  if (!m_sources) return nullptr;
  for (Source* src : *m_sources) {
    if (src->get_num() == source_num) {
      return src;
    }
  }
  return nullptr;
}

System* ConfigurationService::find_system(int sys_num) {
  if (!m_systems) return nullptr;
  for (System* sys : *m_systems) {
    if (sys->get_sys_num() == sys_num) {
      return sys;
    }
  }
  return nullptr;
}

Source* ConfigurationService::get_source(int source_num) {
  return find_source(source_num);
}

System* ConfigurationService::get_system(int sys_num) {
  return find_system(sys_num);
}

System* ConfigurationService::get_system_by_short_name(const std::string& short_name) {
  if (!m_systems) return nullptr;
  for (System* sys : *m_systems) {
    if (sys->get_short_name() == short_name) {
      return sys;
    }
  }
  return nullptr;
}

std::optional<double> ConfigurationService::get_source_gain(int source_num) {
  Source* src = find_source(source_num);
  if (!src) return std::nullopt;
  return src->get_gain();
}

std::optional<double> ConfigurationService::get_source_error(int source_num) {
  Source* src = find_source(source_num);
  if (!src) return std::nullopt;
  return src->get_error();
}

std::optional<bool> ConfigurationService::get_source_gain_mode(int source_num) {
  Source* src = find_source(source_num);
  if (!src) return std::nullopt;
  return src->get_gain_mode();
}

std::optional<std::string> ConfigurationService::get_source_antenna(int source_num) {
  Source* src = find_source(source_num);
  if (!src) return std::nullopt;
  return src->get_antenna();
}

std::optional<double> ConfigurationService::get_system_squelch(int sys_num) {
  System* sys = find_system(sys_num);
  if (!sys) return std::nullopt;
  return sys->get_squelch_db();
}

std::optional<double> ConfigurationService::get_system_analog_levels(int sys_num) {
  System* sys = find_system(sys_num);
  if (!sys) return std::nullopt;
  return sys->get_analog_levels();
}

std::optional<double> ConfigurationService::get_system_digital_levels(int sys_num) {
  System* sys = find_system(sys_num);
  if (!sys) return std::nullopt;
  return sys->get_digital_levels();
}

std::optional<double> ConfigurationService::get_system_min_duration(int sys_num) {
  System* sys = find_system(sys_num);
  if (!sys) return std::nullopt;
  return sys->get_min_duration();
}

std::optional<double> ConfigurationService::get_system_max_duration(int sys_num) {
  System* sys = find_system(sys_num);
  if (!sys) return std::nullopt;
  return sys->get_max_duration();
}

std::optional<bool> ConfigurationService::get_system_record_unknown(int sys_num) {
  System* sys = find_system(sys_num);
  if (!sys) return std::nullopt;
  return sys->get_record_unknown();
}

std::optional<bool> ConfigurationService::get_system_conversation_mode(int sys_num) {
  System* sys = find_system(sys_num);
  if (!sys) return std::nullopt;
  return sys->get_conversation_mode();
}

double ConfigurationService::get_call_timeout() {
  if (!m_config) return 3.0;  // Default
  return m_config->call_timeout;
}

// ============================================================
// Change notification
// ============================================================

void ConfigurationService::register_change_listener(ConfigChangeListener listener) {
  std::lock_guard<std::mutex> lock(m_listeners_mutex);
  m_change_listeners.push_back(listener);
}

void ConfigurationService::notify_listeners(const ConfigChangeInfo& change) {
  std::lock_guard<std::mutex> lock(m_listeners_mutex);
  for (auto& listener : m_change_listeners) {
    try {
      listener(change);
    } catch (const std::exception& e) {
      BOOST_LOG_TRIVIAL(error) << "ConfigurationService: Listener threw exception: " << e.what();
    }
  }
}

// ============================================================
// Validation
// ============================================================

ConfigResult ConfigurationService::validate_command(const ConfigCommand& cmd) {
  // Validate target exists
  if (cmd.type >= ConfigCommandType::SET_SOURCE_GAIN && cmd.type <= ConfigCommandType::SET_SOURCE_SIGNAL_DETECTOR_THRESHOLD) {
    if (!find_source(cmd.target_id)) {
      return ConfigResult::INVALID_TARGET;
    }
  } else if (cmd.type >= ConfigCommandType::SET_SYSTEM_SQUELCH_DB && cmd.type <= ConfigCommandType::SET_SYSTEM_FILTER_WIDTH) {
    if (!find_system(cmd.target_id)) {
      return ConfigResult::INVALID_TARGET;
    }
  }

  // Validate value ranges based on command type
  switch (cmd.type) {
    case ConfigCommandType::SET_SOURCE_GAIN:
    case ConfigCommandType::SET_SOURCE_GAIN_BY_NAME: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double gain = std::get<double>(cmd.value);
      if (gain < 0 || gain > 100) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SOURCE_ERROR: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      // Error can be positive or negative, reasonable range check
      double error = std::get<double>(cmd.value);
      if (error < -1000000 || error > 1000000) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SOURCE_PPM: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double ppm = std::get<double>(cmd.value);
      if (ppm < -1000 || ppm > 1000) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SOURCE_GAIN_MODE: {
      if (!std::holds_alternative<bool>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      break;
    }
    case ConfigCommandType::SET_SOURCE_ANTENNA: {
      if (!std::holds_alternative<std::string>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      break;
    }
    case ConfigCommandType::SET_SOURCE_SIGNAL_DETECTOR_THRESHOLD: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double threshold = std::get<double>(cmd.value);
      if (threshold < -200 || threshold > 0) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SYSTEM_SQUELCH_DB: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double squelch = std::get<double>(cmd.value);
      if (squelch < -200 || squelch > 0) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SYSTEM_ANALOG_LEVELS:
    case ConfigCommandType::SET_SYSTEM_DIGITAL_LEVELS: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double levels = std::get<double>(cmd.value);
      if (levels < 0 || levels > 100) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SYSTEM_MIN_DURATION:
    case ConfigCommandType::SET_SYSTEM_MAX_DURATION:
    case ConfigCommandType::SET_SYSTEM_MIN_TX_DURATION: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double duration = std::get<double>(cmd.value);
      if (duration < 0 || duration > 86400) return ConfigResult::INVALID_VALUE;  // Max 24 hours
      break;
    }
    case ConfigCommandType::SET_SYSTEM_RECORD_UNKNOWN:
    case ConfigCommandType::SET_SYSTEM_HIDE_ENCRYPTED:
    case ConfigCommandType::SET_SYSTEM_HIDE_UNKNOWN:
    case ConfigCommandType::SET_SYSTEM_CONVERSATION_MODE: {
      if (!std::holds_alternative<bool>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      break;
    }
    case ConfigCommandType::SET_SYSTEM_TAU: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double tau = std::get<double>(cmd.value);
      if (tau < 0 || tau > 1) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SYSTEM_MAX_DEV: {
      if (!std::holds_alternative<int>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      int max_dev = std::get<int>(cmd.value);
      if (max_dev < 0 || max_dev > 50000) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_SYSTEM_FILTER_WIDTH: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double width = std::get<double>(cmd.value);
      if (width < 0.1 || width > 10) return ConfigResult::INVALID_VALUE;
      break;
    }
    case ConfigCommandType::SET_CALL_TIMEOUT: {
      if (!std::holds_alternative<double>(cmd.value)) return ConfigResult::TYPE_MISMATCH;
      double timeout = std::get<double>(cmd.value);
      if (timeout < 0.1 || timeout > 300) return ConfigResult::INVALID_VALUE;
      break;
    }
  }

  return ConfigResult::SUCCESS;
}

// ============================================================
// Command execution
// ============================================================

ConfigResult ConfigurationService::execute_command(const ConfigCommand& cmd, ConfigValue& old_value) {
  switch (cmd.type) {
    // Source commands
    case ConfigCommandType::SET_SOURCE_GAIN: {
      Source* src = find_source(cmd.target_id);
      if (!src) return ConfigResult::INVALID_TARGET;
      old_value = src->get_gain();
      src->set_gain(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SOURCE_GAIN_BY_NAME: {
      Source* src = find_source(cmd.target_id);
      if (!src) return ConfigResult::INVALID_TARGET;
      old_value = static_cast<double>(src->get_gain_by_name(cmd.param_name));
      src->set_gain_by_name(cmd.param_name, std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SOURCE_ERROR: {
      Source* src = find_source(cmd.target_id);
      if (!src) return ConfigResult::INVALID_TARGET;
      old_value = src->get_error();
      src->set_error(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SOURCE_PPM: {
      Source* src = find_source(cmd.target_id);
      if (!src) return ConfigResult::INVALID_TARGET;
      old_value = 0.0;  // No getter for PPM
      src->set_freq_corr(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SOURCE_GAIN_MODE: {
      Source* src = find_source(cmd.target_id);
      if (!src) return ConfigResult::INVALID_TARGET;
      old_value = src->get_gain_mode();
      src->set_gain_mode(std::get<bool>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SOURCE_ANTENNA: {
      Source* src = find_source(cmd.target_id);
      if (!src) return ConfigResult::INVALID_TARGET;
      old_value = src->get_antenna();
      src->set_antenna(std::get<std::string>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SOURCE_SIGNAL_DETECTOR_THRESHOLD: {
      Source* src = find_source(cmd.target_id);
      if (!src) return ConfigResult::INVALID_TARGET;
      old_value = 0.0;  // No getter currently
      src->set_signal_detector_threshold(static_cast<float>(std::get<double>(cmd.value)));
      break;
    }

    // System commands
    case ConfigCommandType::SET_SYSTEM_SQUELCH_DB: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_squelch_db();
      sys->set_squelch_db(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_ANALOG_LEVELS: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_analog_levels();
      sys->set_analog_levels(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_DIGITAL_LEVELS: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_digital_levels();
      sys->set_digital_levels(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_MIN_DURATION: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_min_duration();
      sys->set_min_duration(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_MAX_DURATION: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_max_duration();
      sys->set_max_duration(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_MIN_TX_DURATION: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_min_tx_duration();
      sys->set_min_tx_duration(std::get<double>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_RECORD_UNKNOWN: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_record_unknown();
      sys->set_record_unknown(std::get<bool>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_HIDE_ENCRYPTED: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_hideEncrypted();
      sys->set_hideEncrypted(std::get<bool>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_HIDE_UNKNOWN: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_hideUnknown();
      sys->set_hideUnknown(std::get<bool>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_CONVERSATION_MODE: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_conversation_mode();
      sys->set_conversation_mode(std::get<bool>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_TAU: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = static_cast<double>(sys->get_tau());
      sys->set_tau(static_cast<float>(std::get<double>(cmd.value)));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_MAX_DEV: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_max_dev();
      sys->set_max_dev(std::get<int>(cmd.value));
      break;
    }
    case ConfigCommandType::SET_SYSTEM_FILTER_WIDTH: {
      System* sys = find_system(cmd.target_id);
      if (!sys) return ConfigResult::INVALID_TARGET;
      old_value = sys->get_filter_width();
      sys->set_filter_width(std::get<double>(cmd.value));
      break;
    }

    // Config commands
    case ConfigCommandType::SET_CALL_TIMEOUT: {
      if (!m_config) return ConfigResult::INTERNAL_ERROR;
      old_value = m_config->call_timeout;
      m_config->call_timeout = std::get<double>(cmd.value);
      break;
    }
  }

  return ConfigResult::SUCCESS;
}

// ============================================================
// Main loop integration
// ============================================================

void ConfigurationService::process_pending_changes() {
  std::queue<ConfigCommand> commands_to_process;

  // Quickly move commands to local queue to minimize lock time
  {
    std::lock_guard<std::mutex> lock(m_queue_mutex);
    std::swap(commands_to_process, m_pending_commands);
  }

  // Process each command
  while (!commands_to_process.empty()) {
    ConfigCommand cmd = commands_to_process.front();
    commands_to_process.pop();

    // Validate
    ConfigResult result = validate_command(cmd);
    std::string message;

    if (result == ConfigResult::SUCCESS) {
      // Execute
      ConfigValue old_value;
      result = execute_command(cmd, old_value);

      if (result == ConfigResult::SUCCESS) {
        // Build change info for notification
        ConfigChangeInfo change;
        change.type = cmd.type;
        change.target_id = cmd.target_id;
        change.param_name = cmd.param_name;
        change.old_value = old_value;
        change.new_value = cmd.value;
        change.requester = cmd.requester;

        // Notify listeners
        notify_listeners(change);

        message = "Change applied successfully";
      } else {
        message = "Execution failed: " + result_to_string(result);
      }
    } else {
      message = "Validation failed: " + result_to_string(result);
    }

    // Log the change
    log_change(cmd, result, message);

    // Invoke callback if provided
    if (cmd.callback) {
      try {
        cmd.callback(result, message);
      } catch (const std::exception& e) {
        BOOST_LOG_TRIVIAL(error) << "ConfigurationService: Callback threw exception: " << e.what();
      }
    }
  }
}

size_t ConfigurationService::pending_count() {
  std::lock_guard<std::mutex> lock(m_queue_mutex);
  return m_pending_commands.size();
}

// ============================================================
// Logging helpers
// ============================================================

void ConfigurationService::log_change(const ConfigCommand& cmd, ConfigResult result, const std::string& message) {
  std::stringstream ss;
  ss << "ConfigService: [" << cmd.requester << "] ";
  ss << command_type_to_string(cmd.type);

  if (cmd.target_id >= 0) {
    ss << " (target=" << cmd.target_id;
    if (!cmd.param_name.empty()) {
      ss << ", param=" << cmd.param_name;
    }
    ss << ")";
  }

  ss << " -> " << result_to_string(result);

  if (result == ConfigResult::SUCCESS) {
    // Log value for successful changes
    if (std::holds_alternative<double>(cmd.value)) {
      ss << " [value=" << std::get<double>(cmd.value) << "]";
    } else if (std::holds_alternative<int>(cmd.value)) {
      ss << " [value=" << std::get<int>(cmd.value) << "]";
    } else if (std::holds_alternative<bool>(cmd.value)) {
      ss << " [value=" << (std::get<bool>(cmd.value) ? "true" : "false") << "]";
    } else if (std::holds_alternative<std::string>(cmd.value)) {
      ss << " [value=" << std::get<std::string>(cmd.value) << "]";
    }
    BOOST_LOG_TRIVIAL(info) << ss.str();
  } else {
    ss << ": " << message;
    BOOST_LOG_TRIVIAL(warning) << ss.str();
  }
}

std::string ConfigurationService::command_type_to_string(ConfigCommandType type) {
  switch (type) {
    case ConfigCommandType::SET_SOURCE_GAIN: return "SET_SOURCE_GAIN";
    case ConfigCommandType::SET_SOURCE_GAIN_BY_NAME: return "SET_SOURCE_GAIN_BY_NAME";
    case ConfigCommandType::SET_SOURCE_ERROR: return "SET_SOURCE_ERROR";
    case ConfigCommandType::SET_SOURCE_PPM: return "SET_SOURCE_PPM";
    case ConfigCommandType::SET_SOURCE_GAIN_MODE: return "SET_SOURCE_GAIN_MODE";
    case ConfigCommandType::SET_SOURCE_ANTENNA: return "SET_SOURCE_ANTENNA";
    case ConfigCommandType::SET_SOURCE_SIGNAL_DETECTOR_THRESHOLD: return "SET_SOURCE_SIGNAL_DETECTOR_THRESHOLD";
    case ConfigCommandType::SET_SYSTEM_SQUELCH_DB: return "SET_SYSTEM_SQUELCH_DB";
    case ConfigCommandType::SET_SYSTEM_ANALOG_LEVELS: return "SET_SYSTEM_ANALOG_LEVELS";
    case ConfigCommandType::SET_SYSTEM_DIGITAL_LEVELS: return "SET_SYSTEM_DIGITAL_LEVELS";
    case ConfigCommandType::SET_SYSTEM_MIN_DURATION: return "SET_SYSTEM_MIN_DURATION";
    case ConfigCommandType::SET_SYSTEM_MAX_DURATION: return "SET_SYSTEM_MAX_DURATION";
    case ConfigCommandType::SET_SYSTEM_MIN_TX_DURATION: return "SET_SYSTEM_MIN_TX_DURATION";
    case ConfigCommandType::SET_SYSTEM_RECORD_UNKNOWN: return "SET_SYSTEM_RECORD_UNKNOWN";
    case ConfigCommandType::SET_SYSTEM_HIDE_ENCRYPTED: return "SET_SYSTEM_HIDE_ENCRYPTED";
    case ConfigCommandType::SET_SYSTEM_HIDE_UNKNOWN: return "SET_SYSTEM_HIDE_UNKNOWN";
    case ConfigCommandType::SET_SYSTEM_CONVERSATION_MODE: return "SET_SYSTEM_CONVERSATION_MODE";
    case ConfigCommandType::SET_SYSTEM_TAU: return "SET_SYSTEM_TAU";
    case ConfigCommandType::SET_SYSTEM_MAX_DEV: return "SET_SYSTEM_MAX_DEV";
    case ConfigCommandType::SET_SYSTEM_FILTER_WIDTH: return "SET_SYSTEM_FILTER_WIDTH";
    case ConfigCommandType::SET_CALL_TIMEOUT: return "SET_CALL_TIMEOUT";
    default: return "UNKNOWN";
  }
}

std::string ConfigurationService::result_to_string(ConfigResult result) {
  switch (result) {
    case ConfigResult::SUCCESS: return "SUCCESS";
    case ConfigResult::INVALID_TARGET: return "INVALID_TARGET";
    case ConfigResult::INVALID_VALUE: return "INVALID_VALUE";
    case ConfigResult::INVALID_PARAM_NAME: return "INVALID_PARAM_NAME";
    case ConfigResult::TYPE_MISMATCH: return "TYPE_MISMATCH";
    case ConfigResult::INTERNAL_ERROR: return "INTERNAL_ERROR";
    default: return "UNKNOWN";
  }
}

