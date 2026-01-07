/**
 * Example Plugin: Dynamic Configuration
 * 
 * This example plugin demonstrates how to use the ConfigurationService
 * to dynamically modify Trunk Recorder's configuration at runtime.
 * 
 * Features demonstrated:
 * - Adjusting source gain based on signal quality
 * - Modifying system squelch levels
 * - Reacting to configuration changes made by other plugins
 * - Using callbacks for async notification of change results
 */

#ifndef EXAMPLE_CONFIG_PLUGIN_H
#define EXAMPLE_CONFIG_PLUGIN_H

#include "../plugin_manager/plugin_api.h"
#include <chrono>

class ExampleConfigPlugin : public Plugin_Api {
public:
  ExampleConfigPlugin();
  
  // Plugin lifecycle
  int parse_config(json config_data) override;
  int init(Config *config, std::vector<Source *> sources, std::vector<System *> systems) override;
  int start() override;
  int stop() override;
  
  // Called every poll cycle - use for periodic adjustments
  int poll_one() override;
  
  // Called when any configuration parameter changes
  int on_config_change(const ConfigChangeInfo& change) override;
  
  // Optional: React to call events for signal-based gain adjustment
  int call_end(Call_Data_t call_info) override;

private:
  // Configuration from plugin config
  bool m_auto_gain_enabled;
  double m_target_signal_level;
  double m_gain_adjustment_step;
  int m_poll_interval_ms;
  
  // State
  std::vector<Source*> m_sources;
  std::vector<System*> m_systems;
  Config* m_config;
  std::chrono::steady_clock::time_point m_last_adjustment_time;
  
  // Track signal quality for auto-gain
  double m_avg_signal_level;
  int m_signal_sample_count;
  
  // Helper methods
  void adjust_gain_for_source(int source_num);
  void log_current_config();
};

// Factory function for plugin loading
extern "C" {
  boost::shared_ptr<Plugin_Api> create_plugin();
}

#endif // EXAMPLE_CONFIG_PLUGIN_H

