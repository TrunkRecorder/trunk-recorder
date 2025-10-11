#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/foreach.hpp>
#include <boost/intrusive_ptr.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/tokenizer.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <utility>

#include "./global_structs.h"
#include "config.h"
#include "recorder_globals.h"
#include "source.h"

#include "recorders/analog_recorder.h"
#include "recorders/p25_recorder.h"
#include "recorders/recorder.h"

#include "call.h"
#include "call_conventional.h"

#include "systems/p25_trunking.h"
#include "systems/parser.h"

#include "./setup_systems.h"
#include "monitor_systems.h"
#include "systems/smartnet_trunking.h"
#include "systems/system.h"
#include "systems/system_impl.h"

#include <osmosdr/source.h>

#include "formatter.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/message.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uhd/usrp_source.h>

#include "plugin_manager/plugin_manager.h"

#include "cmake.h"
#include "git.h"

using namespace std;
namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;

std::vector<Source *> sources;
std::vector<System *> systems;
std::vector<Call *> calls;
std::vector<Call *> monitored_calls;

gr::top_block_sptr tb;

Config config;

int main(int argc, char **argv) {
  // BOOST_STATIC_ASSERT(true) __attribute__((unused));
  int exit_code = EXIT_SUCCESS;
  logging::core::get()->set_filter(logging::trivial::severity >= logging::trivial::info);

  boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");

  boost::log::add_common_attributes();
  boost::log::core::get()->add_global_attribute("Scope", boost::log::attributes::named_scope());
  boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);

  boost::program_options::options_description desc("Options");
  desc.add_options()("help,h", "Help screen")("config,c", boost::program_options::value<string>()->default_value("./config.json"), "Config File")("version,v", "Version Information");

  boost::program_options::variables_map vm;
  boost::program_options::store(parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("version")) {
    GitMetadata::VersionInfo();
    exit(0);
  }

  if (vm.count("help")) {
    std::cout << "Usage: trunk-recorder [options]\n";
    std::cout << desc;
    exit(0);
  }
  string config_file = vm["config"].as<string>();

  tb = gr::make_top_block("Trunking");

  if (!load_config(config_file, config, tb, sources, systems)) {
    exit(1);
  }

  start_plugins(sources, systems);

  if (setup_systems(config, tb, sources, systems, calls)) {

    tb->start();

    exit_code = monitor_messages(config, tb, sources, systems, calls);

    // ------------------------------------------------------------------
    // -- stop flow graph execution
    // ------------------------------------------------------------------
    BOOST_LOG_TRIVIAL(info) << "stopping flow graph" << std::endl;
    tb->stop();
    tb->wait();

    BOOST_LOG_TRIVIAL(info) << "stopping plugins" << std::endl;
    stop_plugins();
  } else {
    BOOST_LOG_TRIVIAL(error) << "Unable to setup a System to record, exiting..." << std::endl;
  }

  return exit_code;
}
