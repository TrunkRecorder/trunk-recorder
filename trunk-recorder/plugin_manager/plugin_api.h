#ifndef PLUGIN_API_H
#define PLUGIN_API_H

#include "../call.h"
#include "../global_structs.h"
#include "../gr_blocks/decoder_wrapper.h"
#include "../systems/system.h"
#include "../systems/parser.h"
#include "../formatter.h"
#include "../config_service.h"

#include <json.hpp>

typedef enum {
  PLUGIN_UNKNOWN,
  PLUGIN_INITIALIZED,
  PLUGIN_RUNNING,
  PLUGIN_FAILED,
  PLUGIN_STOPPED,
  PLUGIN_DISABLED
} plugin_state_t;

using json = nlohmann::json;

class Plugin_Api {
public:
  virtual int init(Config *config, std::vector<Source *> sources, std::vector<System *> systems) { frequency_format = config->frequency_format; return 0; };
  virtual int parse_config(json config_data) { return 0; }; // const { BOOST_LOG_TRIVIAL(info) << "plugin_api created!";return 0; };
  virtual int start() { return 0; };
  virtual int stop() { return 0; };
  virtual int poll_one() { return 0; };
  virtual int signal(long unitId, const char *signaling_type, gr::blocks::SignalType sig_type, Call *call, System *system, Recorder *recorder) { return 0; };
  virtual int audio_stream(Call *call, Recorder *recorder, int16_t *samples, int sampleCount) { return 0; };
  virtual int trunk_message(std::vector<TrunkMessage> messages, System *system) { return 0; };
  virtual int call_start(Call *call) { return 0; };
  virtual int call_end(Call_Data_t call_info) { return 0; }; //= 0; //{ BOOST_LOG_TRIVIAL(info) << "plugin_api call_end"; return 0; };
  virtual int calls_active(std::vector<Call *> calls) { return 0; };
  virtual int setup_recorder(Recorder *recorder) { return 0; };
  virtual int setup_system(System *system) { return 0; };
  virtual int setup_systems(std::vector<System *> systems) { return 0; };
  virtual int setup_sources(std::vector<Source *> sources) { return 0; };
  virtual int setup_config(std::vector<Source *> sources, std::vector<System *> systems) { return 0; };
  virtual int system_rates(std::vector<System *> systems, float timeDiff) { return 0; };
  virtual int unit_registration(System *sys, long source_id) { return 0; };
  virtual int unit_deregistration(System *sys, long source_id) { return 0; };
  virtual int unit_acknowledge_response(System *sys, long source_id) { return 0; };
  virtual int unit_group_affiliation(System *sys, long source_id, long talkgroup_num) { return 0; };
  virtual int unit_data_grant(System *sys, long source_id) { return 0; };
  virtual int unit_answer_request(System *sys, long source_id, long talkgroup) { return 0; };
  virtual int unit_location(System *sys, long source_id, long talkgroup_num) { return 0; };
  
  // Called when a configuration parameter is changed via the ConfigurationService
  // Allows plugins to react to configuration changes made by other plugins or external sources
  virtual int on_config_change(const ConfigChangeInfo& change) { return 0; };
  
  // Get a pointer to the global configuration service (for making changes)
  ConfigurationService* get_config_service() { return g_config_service; }
  
  //void set_frequency_format(int f) { frequencyFormat = f; }
  virtual ~Plugin_Api(){};
};

#endif
