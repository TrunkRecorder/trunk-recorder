#include <time.h>
#include <vector>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

//#include "../../trunk-recorder/global_structs.h"
//#include "../../trunk-recorder/recorders/recorder.h"
#include "../../trunk-recorder/source.h"
//#include "../../trunk-recorder/systems/system.h"
#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../trunk-recorder/gr_blocks/decoder_wrapper.h"
#include <boost/dll/alias.hpp> // for BOOST_DLL_ALIAS  

#include <json.hpp>

typedef struct stat_plugin_t stat_plugin_t;

struct stat_plugin_t {
  std::vector<Source *> sources;
  std::vector<System *> systems;
  std::vector<Call *> calls;
  Config* config;
};


class Stat_Socket : public Plugin_Api {

  typedef websocketpp::client<websocketpp::config::asio_client> client;
  //typedef websocketpp::retry_client_endpoint<websocketpp::retry_config<websocketpp::config::asio_client>> client;
  typedef websocketpp::lib::lock_guard<websocketpp::lib::mutex> scoped_lock;

  client m_client;
  websocketpp::connection_hdl m_hdl;
  websocketpp::lib::mutex m_lock;
  int retry_attempt;
  time_t reconnect_time;
  bool m_reconnect;
  bool m_open;
  bool m_done;
  bool m_config_sent;
  std::vector<Source *> sources;
  std::vector<System *> systems;
  std::vector<Call *> calls;
  Config* config;

public:
  /**
 * The telemetry client connects to a WebSocket server and sends a message every
 * second containing an integer count. This example can be used as the basis for
 * programs where a client connects and pushes data for logging, stress/load
 * testing, etc.
 */
  int system_rates(std::vector<System *> systems, float timeDiff) {
    this->systems = systems;
    if (m_open == false)
      return 0;
      
    boost::property_tree::ptree nodes;

    for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); it++) {
      System *system = *it;
      nodes.push_back(std::make_pair("", system->get_stats_current(timeDiff)));
    }
    return send_object(nodes, "rates", "rates");
  }

  Stat_Socket() : m_open(false), m_done(false), m_config_sent(false) {
    // set up access channels to only log interesting things
    m_client.clear_access_channels(websocketpp::log::alevel::all);
    m_client.set_access_channels(websocketpp::log::alevel::connect);
    m_client.set_access_channels(websocketpp::log::alevel::disconnect);
    m_client.set_access_channels(websocketpp::log::alevel::app);
    m_client.set_access_channels(websocketpp::log::alevel::message_payload);

    // Initialize the Asio transport policy
    m_client.init_asio();

    // Bind the handlers we are using
    using websocketpp::lib::bind;
    using websocketpp::lib::placeholders::_1;
    using websocketpp::lib::placeholders::_2;
    m_client.set_open_handler(bind(&Stat_Socket::on_open, this, _1));
    m_client.set_close_handler(bind(&Stat_Socket::on_close, this, _1));
    m_client.set_fail_handler(bind(&Stat_Socket::on_fail, this, _1));
    m_client.set_message_handler(bind(&Stat_Socket::on_message, this, _1, _2));
  }

  void send_config(std::vector<Source *> sources, std::vector<System *> systems) {

    if (m_open == false)
      return;

    if (config_sent())
      return;

    boost::property_tree::ptree root;
    boost::property_tree::ptree systems_node;
    boost::property_tree::ptree sources_node;

    for (std::vector<Source *>::iterator it = sources.begin(); it != sources.end(); it++) {
      Source *source = *it;
      std::vector<Gain_Stage_t> gain_stages;
      boost::property_tree::ptree source_node;
      source_node.put("source_num", source->get_num());
      source_node.put("antenna", source->get_antenna());

      source_node.put("silence_frames", source->get_silence_frames());

      source_node.put("min_hz", source->get_min_hz());
      source_node.put("max_hz", source->get_max_hz());
      source_node.put("center", source->get_center());
      source_node.put("rate", source->get_rate());
      source_node.put("driver", source->get_driver());
      source_node.put("device", source->get_device());
      source_node.put("error", source->get_error());
      source_node.put("gain", source->get_gain());
      gain_stages = source->get_gain_stages();
      for (std::vector<Gain_Stage_t>::iterator gain_it = gain_stages.begin(); gain_it != gain_stages.end(); gain_it++) {
        source_node.put(gain_it->stage_name + "_gain", gain_it->value);
      }
      source_node.put("antenna", source->get_antenna());
      source_node.put("analog_recorders", source->analog_recorder_count());
      source_node.put("digital_recorders", source->digital_recorder_count());
      source_node.put("debug_recorders", source->debug_recorder_count());
      source_node.put("sigmf_recorders", source->sigmf_recorder_count());
      sources_node.push_back(std::make_pair("", source_node));
    }

    for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); ++it) {
      System *sys = (System *)*it;

      boost::property_tree::ptree sys_node;
      boost::property_tree::ptree channels_node;
      sys_node.put("audioArchive", sys->get_audio_archive());
      sys_node.put("systemType", sys->get_system_type());
      sys_node.put("shortName", sys->get_short_name());
      sys_node.put("sysNum", sys->get_sys_num());
      sys_node.put("recordUnkown", sys->get_record_unknown());
      sys_node.put("callLog", sys->get_call_log());
      sys_node.put("talkgroupsFile", sys->get_talkgroups_file());
      sys_node.put("analog_levels", sys->get_analog_levels());
      sys_node.put("digital_levels", sys->get_digital_levels());
      sys_node.put("qpsk", sys->get_qpsk_mod());
      sys_node.put("squelch_db", sys->get_squelch_db());
      std::vector<double> channels;

      if ((sys->get_system_type() == "conventional") || (sys->get_system_type() == "conventionalP25") || (sys->get_system_type() == "conventionalDMR") || (sys->get_system_type() == "conventionalSIGMF") ) {
        channels = sys->get_channels();
      } else {
        channels = sys->get_control_channels();
      }

      //std::cout << "starts: " << std::endl;

      for (std::vector<double>::iterator chan_it = channels.begin(); chan_it != channels.end(); chan_it++) {
        double channel = *chan_it;
        boost::property_tree::ptree channel_node;
        //std::cout << "Hello: " << channel << std::endl;
        channel_node.put("", channel);

        // Add this node to the list.
        channels_node.push_back(std::make_pair("", channel_node));
      }
      sys_node.add_child("channels", channels_node);

      if (sys->get_system_type() == "smartnet") {
        sys_node.put("bandplan", sys->get_bandplan());
        sys_node.put("bandfreq", sys->get_bandfreq());
        sys_node.put("bandplan_base", sys->get_bandplan_base());
        sys_node.put("bandplan_high", sys->get_bandplan_high());
        sys_node.put("bandplan_spacing", sys->get_bandplan_spacing());
        sys_node.put("bandplan_offset", sys->get_bandplan_offset());
      }
      systems_node.push_back(std::make_pair("", sys_node));
    }
    root.add_child("sources", sources_node);
    root.add_child("systems", systems_node);
    root.put("captureDir", this->config->capture_dir);
    root.put("uploadServer", this->config->upload_server);

    // root.put("defaultMode", default_mode);
    root.put("callTimeout", this->config->call_timeout);
    root.put("logFile", this->config->log_file);
    root.put("instanceId", this->config->instance_id);
    root.put("instanceKey", this->config->instance_key);
    root.put("logFile", this->config->log_file);
    root.put("type", "config");

    if (this->config->broadcast_signals == true) {
      root.put("broadcast_signals", this->config->broadcast_signals);
    }

    std::stringstream stats_str;
    boost::property_tree::write_json(stats_str, root);

    // std::cout << stats_str;
    send_stat(stats_str.str());
    m_config_sent = true;
  }

  int send_systems(std::vector<System *> systems) {
    if (m_open == false)
      return 0;
    boost::property_tree::ptree node;

    for (std::vector<System *>::iterator it = systems.begin(); it != systems.end(); it++) {
      System *system = *it;
      node.push_back(std::make_pair("", system->get_stats()));
    }
    return send_object(node, "systems", "systems");
  }

  int send_system(System *system) {
    if (m_open == false)
      return 0;

    return send_object(system->get_stats(), "system", "system");

  }

  int calls_active(std::vector<Call *> calls) {
    if (m_open == false)
      return 0;
    boost::property_tree::ptree node;

    for (std::vector<Call *>::iterator it = calls.begin(); it != calls.end(); it++) {
      Call *call = *it;
      //if (call->get_state() == RECORDING) {
        node.push_back(std::make_pair("", call->get_stats()));
      //}
    }

    return send_object(node, "calls", "calls_active");
  }

  int send_recorders(std::vector<Recorder *> recorders) {

    if (m_open == false)
      return 0;
    boost::property_tree::ptree node;

    for (std::vector<Recorder *>::iterator it = recorders.begin(); it != recorders.end(); it++) {
      Recorder *recorder = *it;
      node.push_back(std::make_pair("", recorder->get_stats()));
    }

    return send_object(node, "recorders", "recorders");
  }

  int call_start(Call *call) {
    if (m_open == false)
      return 0;

    return send_object(call->get_stats(), "call", "call_start");

  }

  int call_end(Call_Data_t call_info) {
    if (m_open == false)
      return 0;
    return 0;
    //send_object(call->get_stats(), "call", "call_end");
  }

  int send_recorder(Recorder *recorder) {
    if (m_open == false)
      return 0;

    return send_object(recorder->get_stats(), "recorder", "recorder");
  }

  int send_object(boost::property_tree::ptree data, std::string name, std::string type) {
    if (m_open == false)
      return 0;
    boost::property_tree::ptree root;

    root.add_child(name, data);
    root.put("type", type);
    root.put("instanceId", this->config->instance_id);
    root.put("instanceKey", this->config->instance_key);
    std::stringstream stats_str;
    boost::property_tree::write_json(stats_str, root);

    // std::cout << stats_str;
    return send_stat(stats_str.str());
  }



  void reopen_stat() {
    m_client.reset();
    m_reconnect = false;
    open_stat();
  }
  // This method will block until the connection is complete
  void open_stat() {
    // Create a new connection to the given URI

    if (this->config->status_server == "") {
      return;
    }

    websocketpp::lib::error_code ec;
    client::connection_ptr con = m_client.get_connection(this->config->status_server, ec);

    if (ec) {
      m_client.get_alog().write(websocketpp::log::alevel::app, "open_stat: Get WebSocket Connection Error: " + ec.message());
      return;
    }

    // Grab a handle for this connection so we can talk to it in a thread
    // safe manor after the event loop starts.
    m_hdl = con->get_handle();

    // Queue the connection. No DNS queries or network connections will be
    // made until the io_service event loop is run.
    m_client.connect(con);

    // Create a thread to run the ASIO io_service event loop
    // websocketpp::lib::thread asio_thread(&client::run, &m_client);

    // Create a thread to run the telemetry loop
    // websocketpp::lib::thread telemetry_thread(&stat_socket::telemetry_loop,this);

    // asio_thread.join();
    // telemetry_thread.join();
  }

  int poll_one() {
    if (m_reconnect && (reconnect_time - time(NULL) < 0)) {
      reopen_stat();
    }
    m_client.poll_one();
    return 0;
  }

  bool is_open() {
    scoped_lock guard(m_lock);

    return m_open;
  }

  bool config_sent() {
    scoped_lock guard(m_lock);

    return m_config_sent;
  }
  // The open handler will signal that we are ready to start sending telemetry
  void on_open(websocketpp::connection_hdl) {
    m_client.get_alog().write(websocketpp::log::alevel::app,
                              "on_open: WebSocket Connection opened, starting telemetry!");

    {
      // de scope the lock before calling the callback
      scoped_lock guard(m_lock);
      m_open = true;
      retry_attempt = 0;
    }
    send_config(this->sources, this->systems);
    send_systems(this->systems);
    std::vector<Recorder *> recorders;

    for (std::vector<Source *>::iterator it = this->sources.begin(); it != this->sources.end(); it++) {
      Source *source = *it;

      std::vector<Recorder *> sourceRecorders = source->get_recorders();

      recorders.insert(recorders.end(), sourceRecorders.begin(), sourceRecorders.end());
    }

    send_recorders(recorders);
  }

  // The close handler will signal that we should stop sending telemetry
  void on_close(websocketpp::connection_hdl) {
    std::stringstream stream_num;
    std::string str_num;
    m_client.get_alog().write(websocketpp::log::alevel::app,
                              "on_close: WebSocket Connection closed, stopping telemetry!");

    scoped_lock guard(m_lock);
    m_open = false;
    m_done = true;
    m_config_sent = false;
    m_reconnect = true;
    retry_attempt++;
    long reconnect_delay = (6 * retry_attempt + (rand() % 30));
    stream_num << reconnect_delay;
    stream_num >> str_num;
    reconnect_time = time(NULL) + reconnect_delay;
    m_client.get_alog().write(websocketpp::log::alevel::app, "on_close: Will try to reconnect in:  " + str_num);
  }

  // The fail handler will signal that we should stop sending telemetry
  void on_fail(websocketpp::connection_hdl) {
    std::stringstream stream_num;
    std::string str_num;
    m_client.get_alog().write(websocketpp::log::alevel::app, "on_fail: WebSocket Connection failed, stopping telemetry!");

    scoped_lock guard(m_lock);
    m_open = false;
    m_done = true;
    m_config_sent = false;
    if (!m_reconnect) {
      m_reconnect = true;
      retry_attempt++;
      long reconnect_delay = (6 * retry_attempt + (rand() % 30));
      stream_num << reconnect_delay;
      stream_num >> str_num;
      reconnect_time = time(NULL) + reconnect_delay;
      m_client.get_alog().write(websocketpp::log::alevel::app, "on_fail: Will try to reconnect in:  " + str_num);
    }
  }

  void on_message(websocketpp::connection_hdl, client::message_ptr msg) {
    //Need to receive the message so they don't build up. TrunkPlayer sends a message to acknowledge what TrunkRecorder sends.
  }

  int send_stat(std::string val) {
    websocketpp::lib::error_code ec;
    if (m_open) {
      m_client.send(m_hdl, val, websocketpp::frame::opcode::text, ec);

      // The most likely error that we will get is that the connection is
      // not in the right state. Usually this means we tried to send a
      // message to a connection that was closed or in the process of
      // closing. While many errors here can be easily recovered from,
      // in this simple example, we'll stop the telemetry loop.
      if (ec) {
        m_client.get_alog().write(websocketpp::log::alevel::app, "Error Sending : " + ec.message());
        return 1;
      }
      return 0;
    } else {
      return 1;
    }
  }

  int signal(long unitId, const char *signaling_type, gr::blocks::SignalType sig_type, Call *call, System *system, Recorder *recorder) {
    if (m_open == false || this->config->broadcast_signals == false)
      return 1;

    boost::property_tree::ptree signal;
    signal.put("unit_id", unitId);
    //signal.put("signal_system_type", signaling_type);
    //signal.put("signal_type", sig_type);

    if (call != NULL) {
      signal.add_child("call", call->get_stats());
    }

    if (recorder != NULL) {
      signal.add_child("recorder", recorder->get_stats());
    }

    if (system != NULL) {
      signal.add_child("system", system->get_stats());
    }

    return send_object(signal, "signal", "signaling");
  }

  int init(Config *config, std::vector<Source *> sources, std::vector<System *> systems) {

    this->sources = sources;
    this->systems = systems;
    this->config = config;

    return 0;
  }

  int start() {
    open_stat();
    return 0;
  }

  int setup_recorder( Recorder *recorder) {
    this->send_recorder(recorder);
    return 0;
  }

  int setup_system( System *system) {
    this->send_system(system);
    return 0;
  }

  int setup_systems( std::vector<System *> systems) {
    this->systems = systems;

    this->send_systems(systems);
    return 0;
  }

  int setup_config( std::vector<Source *> sources, std::vector<System *> systems) {

    this->sources = sources;
    this->systems = systems;

    send_config(sources, systems);
    return 0;
  }


    // Factory method
    static boost::shared_ptr<Stat_Socket> create() {
        return boost::shared_ptr<Stat_Socket>(
            new Stat_Socket()
        );
    }

 int parse_config(json config_data ){ return 0; }
   int stop() { return 0; }
   int setup_sources(std::vector<Source *> sources) { return 0; }

};

BOOST_DLL_ALIAS(
   Stat_Socket::create, // <-- this function is exported with...
    create_plugin                               // <-- ...this alias name
)
