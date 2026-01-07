#ifndef CONFIG_SERVICE_H
#define CONFIG_SERVICE_H

#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <variant>
#include <vector>
#include <optional>

#include <boost/log/trivial.hpp>

#include "global_structs.h"

// Forward declarations to avoid circular includes
class Source;
class System;

// Value types that can be set via the configuration service
using ConfigValue = std::variant<double, int, bool, std::string>;

// Command types for all modifiable parameters
enum class ConfigCommandType {
  // Source parameters
  SET_SOURCE_GAIN,
  SET_SOURCE_GAIN_BY_NAME,
  SET_SOURCE_ERROR,
  SET_SOURCE_PPM,
  SET_SOURCE_GAIN_MODE,
  SET_SOURCE_ANTENNA,
  SET_SOURCE_SIGNAL_DETECTOR_THRESHOLD,

  // System parameters
  SET_SYSTEM_SQUELCH_DB,
  SET_SYSTEM_ANALOG_LEVELS,
  SET_SYSTEM_DIGITAL_LEVELS,
  SET_SYSTEM_MIN_DURATION,
  SET_SYSTEM_MAX_DURATION,
  SET_SYSTEM_MIN_TX_DURATION,
  SET_SYSTEM_RECORD_UNKNOWN,
  SET_SYSTEM_HIDE_ENCRYPTED,
  SET_SYSTEM_HIDE_UNKNOWN,
  SET_SYSTEM_CONVERSATION_MODE,
  SET_SYSTEM_TAU,
  SET_SYSTEM_MAX_DEV,
  SET_SYSTEM_FILTER_WIDTH,

  // Config parameters
  SET_CALL_TIMEOUT
};

// Result of a configuration change
enum class ConfigResult {
  SUCCESS,
  INVALID_TARGET,      // Source/System not found
  INVALID_VALUE,       // Value out of range or wrong type
  INVALID_PARAM_NAME,  // For gain_by_name with unknown name
  TYPE_MISMATCH,       // Wrong variant type for command
  INTERNAL_ERROR       // Unexpected error
};

// Callback type for async notification
using ConfigCallback = std::function<void(ConfigResult result, const std::string& message)>;

// Configuration change command
struct ConfigCommand {
  ConfigCommandType type;
  int target_id;              // source_num or sys_num (-1 for global config)
  std::string param_name;     // Used for SET_SOURCE_GAIN_BY_NAME
  ConfigValue value;
  ConfigCallback callback;    // Optional callback for result notification
  std::string requester;      // Name of plugin/entity requesting change

  ConfigCommand() : type(ConfigCommandType::SET_SOURCE_GAIN), target_id(-1), callback(nullptr) {}
};

// Information about a configuration change (for notifications)
struct ConfigChangeInfo {
  ConfigCommandType type;
  int target_id;
  std::string param_name;
  ConfigValue old_value;
  ConfigValue new_value;
  std::string requester;
};

// Listener callback type for change notifications
using ConfigChangeListener = std::function<void(const ConfigChangeInfo& change)>;

class ConfigurationService {
public:
  ConfigurationService();
  ~ConfigurationService();

  // Initialize the service with references to sources, systems, and config
  void init(Config* config, std::vector<Source*>* sources, std::vector<System*>* systems);

  // ============================================================
  // Command submission (thread-safe, called by plugins)
  // ============================================================

  // Submit a configuration change command
  // Returns immediately; actual change happens in process_pending_changes()
  bool submit_command(const ConfigCommand& cmd);

  // Convenience methods for common operations
  bool set_source_gain(int source_num, double gain, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_source_gain_by_name(int source_num, const std::string& name, double gain, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_source_error(int source_num, double error, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_source_ppm(int source_num, double ppm, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_source_gain_mode(int source_num, bool agc, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_source_antenna(int source_num, const std::string& antenna, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_source_signal_detector_threshold(int source_num, double threshold, const std::string& requester, ConfigCallback callback = nullptr);

  bool set_system_squelch(int sys_num, double squelch_db, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_analog_levels(int sys_num, double levels, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_digital_levels(int sys_num, double levels, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_min_duration(int sys_num, double duration, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_max_duration(int sys_num, double duration, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_min_tx_duration(int sys_num, double duration, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_record_unknown(int sys_num, bool record, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_hide_encrypted(int sys_num, bool hide, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_hide_unknown(int sys_num, bool hide, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_conversation_mode(int sys_num, bool mode, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_tau(int sys_num, float tau, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_max_dev(int sys_num, int max_dev, const std::string& requester, ConfigCallback callback = nullptr);
  bool set_system_filter_width(int sys_num, double width, const std::string& requester, ConfigCallback callback = nullptr);

  bool set_call_timeout(double timeout, const std::string& requester, ConfigCallback callback = nullptr);

  // ============================================================
  // Getters (thread-safe snapshots)
  // ============================================================

  std::optional<double> get_source_gain(int source_num);
  std::optional<double> get_source_error(int source_num);
  std::optional<bool> get_source_gain_mode(int source_num);
  std::optional<std::string> get_source_antenna(int source_num);

  std::optional<double> get_system_squelch(int sys_num);
  std::optional<double> get_system_analog_levels(int sys_num);
  std::optional<double> get_system_digital_levels(int sys_num);
  std::optional<double> get_system_min_duration(int sys_num);
  std::optional<double> get_system_max_duration(int sys_num);
  std::optional<bool> get_system_record_unknown(int sys_num);
  std::optional<bool> get_system_conversation_mode(int sys_num);

  double get_call_timeout();

  // Get source/system by ID
  Source* get_source(int source_num);
  System* get_system(int sys_num);
  System* get_system_by_short_name(const std::string& short_name);

  // ============================================================
  // Change notification
  // ============================================================

  // Register a listener to be notified of all configuration changes
  void register_change_listener(ConfigChangeListener listener);

  // ============================================================
  // Main loop integration
  // ============================================================

  // Process all pending configuration changes
  // Called from the main loop (single-threaded context)
  void process_pending_changes();

  // Get the number of pending changes
  size_t pending_count();

private:
  // Find source by number
  Source* find_source(int source_num);

  // Find system by number
  System* find_system(int sys_num);

  // Validate command before execution
  ConfigResult validate_command(const ConfigCommand& cmd);

  // Execute a single command
  ConfigResult execute_command(const ConfigCommand& cmd, ConfigValue& old_value);

  // Notify all listeners of a change
  void notify_listeners(const ConfigChangeInfo& change);

  // Log a configuration change
  void log_change(const ConfigCommand& cmd, ConfigResult result, const std::string& message);

  // Get string representation of command type
  static std::string command_type_to_string(ConfigCommandType type);

  // Get string representation of result
  static std::string result_to_string(ConfigResult result);

  // Members
  Config* m_config;
  std::vector<Source*>* m_sources;
  std::vector<System*>* m_systems;

  std::mutex m_queue_mutex;
  std::queue<ConfigCommand> m_pending_commands;

  std::mutex m_listeners_mutex;
  std::vector<ConfigChangeListener> m_change_listeners;

  bool m_initialized;
};

// Global configuration service instance
extern ConfigurationService* g_config_service;

// Initialize the global configuration service
void init_config_service(Config* config, std::vector<Source*>* sources, std::vector<System*>* systems);

// Shutdown the global configuration service
void shutdown_config_service();

#endif // CONFIG_SERVICE_H

