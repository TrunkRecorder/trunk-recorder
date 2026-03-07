#ifndef PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_H

#include "../call_concluder/call_concluder.h"
#include "../config_service.h"
#include "../recorders/recorder.h"
#include "../source.h"
#include "../systems/system.h"
#include "../systems/system_impl.h"

#include "plugin_api.h"
#if GNURADIO_VERSION >= 0x030a00
#include <boost/function.hpp>
#endif
#include <boost/optional/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <stdlib.h>
#include <vector>

typedef boost::shared_ptr<Plugin_Api>(pluginapi_create_t)();

struct Plugin {
  boost::function<pluginapi_create_t> creator;
  boost::shared_ptr<Plugin_Api> api;
  plugin_state_t state;
  std::string name;
};

void initialize_plugins(json config_data, Config *config, std::vector<Source *> sources, std::vector<System *> systems);
void add_internal_plugin(std::string name, std::string library, json config_data);
void start_plugins(std::vector<Source *> sources, std::vector<System *> systems);
void stop_plugins();

void plugman_poll_one();
void plugman_audio_callback(Call *call, Recorder *recorder, int16_t *samples, int sampleCount);
int plugman_signal(long unitId, const char *signaling_type, gr::blocks::SignalType sig_type, Call *call, System *system, Recorder *recorder);
int plugman_trunk_message(std::vector<TrunkMessage> messages, System *system);
int plugman_call_start(Call *call);
int plugman_call_end(Call_Data_t& call_info);
int plugman_calls_active(std::vector<Call *> calls);
void plugman_setup_recorder(Recorder *recorder);
void plugman_setup_system(System *system);
void plugman_setup_systems(std::vector<System *> systems);
void plugman_setup_sources(std::vector<Source *> sources);
void plugman_setup_config(std::vector<Source *> sources, std::vector<System *> systems);
void plugman_system_rates(std::vector<System *> systems, float timeDiff);
void plugman_unit_registration(System *system, long source_id);
void plugman_unit_deregistration(System *system, long source_id);
void plugman_unit_acknowledge_response(System *system, long source_id);
void plugman_unit_group_affiliation(System *system, long source_id, long talkgroup_num);
void plugman_unit_data_grant(System *system, long source_id);
void plugman_unit_answer_request(System *system, long source_id, long talkgroup);
void plugman_unit_location(System *system, long source_id, long talkgroup_num);
void plugman_config_change(const ConfigChangeInfo& change);

// Register the plugin manager as a listener for configuration changes
void plugman_register_config_listener();

#endif // PLUGIN_MANAGER_H
