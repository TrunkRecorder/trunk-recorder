#include "plugin_manager.h"

#include "../global_structs.h"
#include <boost/dll/import.hpp> // for import_alias
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <stdlib.h>
#include <vector>

std::vector<Plugin *> plugins;

Plugin *setup_plugin(std::string plugin_lib, std::string plugin_name) {
  BOOST_LOG_TRIVIAL(info) << "Setting up plugin -  Name: " << plugin_name << "\t Library file: " << plugin_lib;
  // Plugin *plugin = plugin_new(plugin_lib == "" ? NULL : plugin_lib.c_str(), plugin_name.c_str());

  // Based on factory plugin method from Boost: https://www.boost.org/doc/libs/1_64_0/doc/html/boost_dll/tutorial.html#boost_dll.tutorial.factory_method_in_plugin
  boost::filesystem::path lib_path("./");
  Plugin *plugin = new Plugin();
  plugin->creator = boost::dll::import_alias<pluginapi_create_t>( // type of imported symbol must be explicitly specified
      plugin_lib,                                                 // path to library
      "create_plugin",                                            // symbol to import
      boost::dll::load_mode::append_decorations                   // do append extensions and prefixes
          | boost::dll::load_mode::search_system_folders);

  plugin->api = plugin->creator();
  plugin->name = plugin_name;
  plugins.push_back(plugin);

  return plugin;
}

void initialize_plugins(json config_data, Config *config, std::vector<Source *> sources, std::vector<System *> systems) {

  bool plugins_exists = config_data.contains("plugins");

  if (plugins_exists) {
    for (json element : config_data["plugins"]) {
      std::string plugin_lib = element.value("library", "");
      std::string plugin_name = element.value("name", "");
      
      // If name is not provided, derive it from the library filename
      if (plugin_name.empty() && !plugin_lib.empty()) {
        plugin_name = boost::filesystem::path(plugin_lib).stem().string();
        if (plugin_name.size() > 3 && plugin_name.substr(0, 3) == "lib") {
          plugin_name = plugin_name.substr(3);
        }
      }
      
      bool plugin_enabled = element.value("enabled", true);
      if (plugin_enabled) {
        Plugin *plugin = setup_plugin(plugin_lib, plugin_name);
        plugin->api->parse_config(element);
      }
    }

    if (plugins.size() == 1) {
      BOOST_LOG_TRIVIAL(info) << "Loaded " << plugins.size() << " Plugin";
    }

    if (plugins.size() > 1) {
      BOOST_LOG_TRIVIAL(info) << "Loaded " << plugins.size() << " Plugins";
    }
  } else {
    BOOST_LOG_TRIVIAL(info) << "No plugins configured";
  }

  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    int ret = plugin->api->init(config, sources, systems);
    if (ret < 0) {
      plugin->state = PLUGIN_FAILED;
    } else {
      plugin->state = PLUGIN_INITIALIZED;
      ret = 0;
    }
  }
}

void add_internal_plugin(std::string name, std::string library, json config_data) {

  Plugin *plugin = setup_plugin(library, name);
  plugin->api->parse_config(config_data);
}

void start_plugins(std::vector<Source *> sources, std::vector<System *> systems) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;

    /* ----- Plugin Start ----- */
    if (plugin->state == PLUGIN_INITIALIZED) {
      int err = plugin->api->start();
      if (err != 0) {
        plugin->state = PLUGIN_FAILED;
        continue;
      }
    }
    plugin->state = PLUGIN_RUNNING;

    /* ----- Plugin Setup Sources ----- */
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->setup_sources(sources);
    }

    /* ----- Plugin Setup Systems ----- */
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->setup_systems(systems);
    }
  }
}

void stop_plugins() {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      int err = plugin->api->stop();
      if (err != 0) {
        plugin->state = PLUGIN_FAILED;
        continue;
      }
    }
    plugin->state = PLUGIN_STOPPED;
  }
}

void plugman_poll_one() {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->poll_one();
    }
  }
}

void plugman_audio_callback(Call *call, Recorder *recorder, int16_t *samples, int sampleCount) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->audio_stream(call, recorder, samples, sampleCount);
    }
  }
}

int plugman_signal(long unitId, const char *signaling_type, gr::blocks::SignalType sig_type, Call *call, System *system, Recorder *recorder) {
  int error = 0;
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->signal(unitId, signaling_type, sig_type, call, system, recorder);
    }
  }
  return error;
}

int plugman_trunk_message(std::vector<TrunkMessage> messages, System *system) {
  int error = 0;
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->trunk_message(messages, system);
    }
  }
  return error;
}

int plugman_call_start(Call *call) {
  int error = 0;
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->call_start(call);
    }
  }
  return error;
}

int plugman_call_end(Call_Data_t& call_info) {
  std::vector<int> plugin_retry_list;
  
  std::stringstream logstream;
  logstream << "[" << call_info.short_name << "]\t\033[0;34m" << call_info.call_num << "C\033[0m\tTG: " << call_info.talkgroup_display << "\tFreq: " << format_freq(call_info.freq) << "\t";
  std::string loghdr = logstream.str();

  // On INITIAL, run call_end for all active plugins and note failues 
  if (call_info.status == INITIAL)
  {
    for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
      Plugin *plugin = *it;
      if (plugin->state == PLUGIN_RUNNING) {
        int plugin_error = plugin->api->call_end(call_info);
        if (plugin_error) {
          BOOST_LOG_TRIVIAL(error) << loghdr << "Plugin Manager: call_end -  " << plugin->name << " failed.";
          int plugin_index = std::distance(plugins.begin(), it );
          plugin_retry_list.push_back(plugin_index);
        }
      }
    }
  } 
  // On RETRY, run call_end only for plugins reporting previous failue
  else if (call_info.status == RETRY)
  {
    for (std::vector<int>::iterator it = call_info.plugin_retry_list.begin(); it != call_info.plugin_retry_list.end(); it++) {
      Plugin *plugin = plugins[*it];
      if (plugin->state == PLUGIN_RUNNING) {
        BOOST_LOG_TRIVIAL(info) << loghdr << "Plugin Manager: call_end - retry (" << call_info.retry_attempt << "/" << Call_Concluder::MAX_RETRY << ") - " << plugin->name;
        int plugin_error = plugin->api->call_end(call_info);
        if (plugin_error) {
          BOOST_LOG_TRIVIAL(error) << loghdr << "Plugin Manager: call_end - retry (" << call_info.retry_attempt << "/" << Call_Concluder::MAX_RETRY << ") - " << plugin->name << " failed.";
          plugin_retry_list.push_back(*it);
        }
      }
    }
  }

  if (plugin_retry_list.size() == 0) {
    return 0;
  } else {
    call_info.plugin_retry_list = plugin_retry_list;
    return 1;
  }
}

int plugman_calls_active(std::vector<Call *> calls) {
  int error = 0;
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->calls_active(calls);
    }
  }
  return error;
}

void plugman_setup_recorder(Recorder *recorder) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->setup_recorder(recorder);
    }
  }
}

void plugman_setup_system(System *system) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->setup_system(system);
    }
  }
}

void plugman_setup_systems(std::vector<System *> systems) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->setup_systems(systems);
    }
  }
}

void plugman_setup_sources(std::vector<Source *> sources) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->setup_sources(sources);
    }
  }
}

void plugman_setup_config(std::vector<Source *> sources, std::vector<System *> systems) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->setup_config(sources, systems);
    }
  }
}

void plugman_system_rates(std::vector<System *> systems, float timeDiff) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->system_rates(systems, timeDiff);
    }
  }
}

void plugman_unit_registration(System *system, long source_id) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->unit_registration(system, source_id);
    }
  }
}
void plugman_unit_deregistration(System *system, long source_id) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->unit_deregistration(system, source_id);
    }
  }
}
void plugman_unit_acknowledge_response(System *system, long source_id) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->unit_acknowledge_response(system, source_id);
    }
  }
}
void plugman_unit_group_affiliation(System *system, long source_id, long talkgroup_num) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->unit_group_affiliation(system, source_id, talkgroup_num);
    }
  }
}
void plugman_unit_data_grant(System *system, long source_id) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->unit_data_grant(system, source_id);
    }
  }
}
void plugman_unit_answer_request(System *system, long source_id, long talkgroup) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->unit_answer_request(system, source_id, talkgroup);
    }
  }
}
void plugman_unit_location(System *system, long source_id, long talkgroup_num) {
  for (std::vector<Plugin *>::iterator it = plugins.begin(); it != plugins.end(); it++) {
    Plugin *plugin = *it;
    if (plugin->state == PLUGIN_RUNNING) {
      plugin->api->unit_location(system, source_id, talkgroup_num);
    }
  }
}
