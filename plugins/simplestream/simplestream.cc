#include "../../trunk-recorder/plugin_manager/plugin_api.h"
#include "../../trunk-recorder/recorders/recorder.h"
#include <boost/dll/alias.hpp> // for BOOST_DLL_ALIAS
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>

using namespace boost::asio;

typedef struct plugin_t plugin_t;
typedef struct stream_t stream_t;
std::vector<stream_t> streams;
io_service my_tcp_io_service;
long max_tcp_index = 0;

struct plugin_t {
  Config* config;
};

struct stream_t {
  long TGID;
  long tcp_index;
  std::string address;
  std::string short_name;
  long port;
  ip::udp::endpoint remote_endpoint;
  ip::tcp::socket *tcp_socket;
  bool sendTGID = false;
  bool sendJSON = false;
  bool sendCallStart = false;
  bool sendCallEnd = false;
  bool tcp = false;
};

class Simple_Stream : public Plugin_Api {
  typedef boost::asio::io_service io_service;
  io_service my_io_service;
  ip::udp::endpoint remote_endpoint;
  ip::udp::socket my_socket{my_io_service};
  public:
  
  Simple_Stream(){
      
  }

 int parse_config(json config_data) {
    for (json element : config_data["streams"]) {
      stream_t stream;
      stream.TGID = element["TGID"];
      stream.address = element["address"];
      stream.port = element["port"];
      stream.remote_endpoint = ip::udp::endpoint(ip::address::from_string(stream.address), stream.port);
      stream.sendTGID = element.value("sendTGID",false);
      stream.sendJSON = element.value("sendJSON",false);
      stream.sendCallStart = element.value("sendCallStart",false);
      stream.sendCallEnd = element.value("sendCallEnd",false);
      stream.tcp = element.value("useTCP",false);
      stream.short_name = element.value("shortName", "");
      BOOST_LOG_TRIVIAL(info) << "simplestreamer will stream audio from TGID " <<stream.TGID << " on System " <<stream.short_name << " to " << stream.address <<" on port " << stream.port << " tcp is "<<stream.tcp;
      streams.push_back(stream);
    }
    return 0;
  }
  
  int audio_stream(Call *call, Recorder *recorder, int16_t *samples, int sampleCount){
    //Call local_call = *call;
    System *call_system = call->get_system();
    int32_t call_tgid = call->get_talkgroup();
    int32_t call_src = call->get_current_source_id();
    uint32_t call_freq = call->get_freq();
    std::string call_short_name = call->get_short_name();
    std::string call_src_tag = call_system->find_unit_tag(call_src);
    std::vector<unsigned long> unsigned_patched_talkgroups = call_system->get_talkgroup_patch(call_tgid);
    std::vector<long> patched_talkgroups;
    // Convert unsigned long to signed long, preserving negative values
    for (auto tgid : unsigned_patched_talkgroups) {
      patched_talkgroups.push_back(static_cast<long>(tgid));
    }

    if(call_src == -1){
      if(call->get_transmissions().size() > 0){
        // Get the source from the most recent transmission
        auto transmissions = call->get_transmissions();
        call_src = transmissions.back().source;
        BOOST_LOG_TRIVIAL(debug) << "using source " << call_src << " from most recent transmission";
      }
      else{
        BOOST_LOG_TRIVIAL(debug) << "no source found for call - leaving src as -1";
      }
    }

    Recorder& local_recorder = *recorder;
    int recorder_id = local_recorder.get_num();
    long wav_hz = local_recorder.get_wav_hz();
    boost::system::error_code error;
    BOOST_FOREACH (auto stream, streams){
      if (0==stream.short_name.compare(call_short_name) || (0==stream.short_name.compare(""))){ //Check if shortName matches or is not specified
        if (patched_talkgroups.size() == 0){
          patched_talkgroups.push_back(call_tgid);  //call_info.talkgroup may be negative - we cast stream.TGID to signed for comparison
        }
        BOOST_FOREACH (auto TGID, patched_talkgroups){
          if ((TGID==static_cast<long>(stream.TGID)) || stream.TGID==0){  //setting TGID to 0 in the config file will stream everything
            BOOST_LOG_TRIVIAL(debug) << "got " <<sampleCount <<" samples - " <<sampleCount*2<<" bytes from recorder "<<recorder_id<<" for TGID "<<TGID;
            json json_object;
            std::string json_string;
            std::vector<boost::asio::const_buffer> send_buffer;
            if (stream.sendJSON==true){
              //create JSON metadata
              json_object = {
                 {"src", call_src},
                 {"src_tag",call_src_tag},
                 {"talkgroup", TGID},
                 {"patched_talkgroups",patched_talkgroups},
                 {"freq", call_freq},
                 {"short_name", call_short_name},
                 {"audio_sample_rate",wav_hz},
                 {"event","audio"},
              };
              json_string = json_object.dump();
              uint32_t json_length = json_string.length();  //determine length in bytes
              //BOOST_LOG_TRIVIAL(debug) << "json_length is " <<json_length <<" bytes";
              send_buffer.push_back(buffer(&json_length,4));  //prepend length of the json data
              send_buffer.push_back(buffer(json_string));  //prepend json data
              //BOOST_LOG_TRIVIAL(debug) << "json_string is " <<json_string;
            }
            else if (stream.sendTGID==true){
              send_buffer.push_back(buffer(&TGID,4));  //prepend 4 byte long tgid to the audio data
            }
            send_buffer.push_back(buffer(samples, sampleCount*2));
            if(stream.tcp == true){
              stream.tcp_socket->send(send_buffer);
            }
            else{
              my_socket.send_to(send_buffer, stream.remote_endpoint, 0, error);
            }
          }
        }
      }
    }
    return 0;
  }

  int call_start(Call *call){
    boost::system::error_code error;
    System *call_system = call->get_system();
    int32_t call_tgid = call->get_talkgroup();
    int32_t call_src = call->get_current_source_id();
    uint32_t call_freq = call->get_freq();
    std::string call_short_name = call->get_short_name();
    std::string call_src_tag = call_system->find_unit_tag(call_src);
    std::string call_tgid_tag = call->get_talkgroup_tag();
    std::vector<unsigned long> unsigned_patched_talkgroups = call_system->get_talkgroup_patch(call_tgid);
    std::vector<long> patched_talkgroups;
    // Convert unsigned long to signed long, preserving negative values
    for (auto tgid : unsigned_patched_talkgroups) {
      patched_talkgroups.push_back(static_cast<long>(tgid));
    }

    if(call_src == -1){
      if(call->get_transmissions().size() > 0){
        // Get the source from the most recent transmission
        auto transmissions = call->get_transmissions();
        call_src = transmissions.back().source;
      }
    }
    
    BOOST_FOREACH (auto stream, streams){
      if (stream.sendJSON == true && stream.sendCallStart == true){
        if (0==stream.short_name.compare(call_short_name) || (0==stream.short_name.compare(""))){ //Check if shortName matches or is not specified
          if (patched_talkgroups.size() == 0){
            patched_talkgroups.push_back(call_tgid);  //call_info.talkgroup may be negative - we cast stream.TGID to signed for comparison
          }
          std::vector<std::string> patched_talkgroup_tags;
          BOOST_FOREACH (auto TGID, patched_talkgroups){
            Talkgroup* this_tg = call_system->find_talkgroup(static_cast<unsigned long>(TGID));
            if (this_tg != nullptr) {
              patched_talkgroup_tags.push_back(this_tg->alpha_tag);
            }
            if ((TGID==static_cast<long>(stream.TGID)) || stream.TGID==0){  //setting TGID to 0 in the config file will stream everything
              json json_object;
              std::string json_string;
              std::vector<boost::asio::const_buffer> send_buffer;
              if (stream.sendJSON==true){
                //create JSON metadata
                json_object = {
                   {"src", call_src},
                   {"src_tag", call_src_tag},
                   {"talkgroup", call_tgid},
                   {"talkgroup_tag",call_tgid_tag},
                   {"patched_talkgroups",patched_talkgroups},
                   {"patched_talkgroup_tags",patched_talkgroup_tags},
                   {"freq", call_freq},
                   {"short_name", call_short_name},
                   {"event","call_start"},
                };
                json_string = json_object.dump();
                uint32_t json_length = json_string.length();  //determine length in bytes
                send_buffer.push_back(buffer(&json_length,4));  //prepend length of the json data
                send_buffer.push_back(buffer(json_string));  //prepend json data
              }
              if(stream.tcp == true){
                stream.tcp_socket->send(send_buffer);
              }
              else{
                my_socket.send_to(send_buffer, stream.remote_endpoint, 0, error);
              }
            }
          }
        }
      }
    }
    return 0;
  }

  int call_end(Call_Data_t call_info) {
    boost::system::error_code error;
    BOOST_FOREACH (auto stream, streams){
      if (stream.sendJSON == true && stream.sendCallEnd == true){
        if (0==stream.short_name.compare(call_info.short_name) || (0==stream.short_name.compare(""))){ //Check if shortName matches or is not specified
          std::vector<long> patched_talkgroups;
          if (call_info.patched_talkgroups.size() == 0){
            patched_talkgroups.push_back(call_info.talkgroup); //call_info.talkgroup may be negative - we cast stream.TGID to signed for comparison
          } else {
            // Convert existing unsigned long values to signed long
            for (auto tgid : call_info.patched_talkgroups) {
              patched_talkgroups.push_back(static_cast<long>(tgid));
            }
          }
          BOOST_FOREACH (auto TGID, patched_talkgroups){
            if ((TGID == static_cast<long>(stream.TGID)) || stream.TGID==0){  //setting TGID to 0 in the config file will stream everything
              json json_object;
              std::string json_string;
              std::vector<boost::asio::const_buffer> send_buffer;
              if (stream.sendJSON==true){
                //create JSON metadata
                json_object = {
                   {"talkgroup", call_info.talkgroup},
                   {"patched_talkgroups",patched_talkgroups},
                   {"freq", call_info.freq},
                   {"short_name", call_info.short_name},
                   {"event","call_end"},
                };
                json_string = json_object.dump();
                uint32_t json_length = json_string.length();  //determine length in bytes
                send_buffer.push_back(buffer(&json_length,4));  //prepend length of the json data
                send_buffer.push_back(buffer(json_string));  //prepend json data
              }
              if(stream.tcp == true){
                stream.tcp_socket->send(send_buffer);
              }
              else{
                my_socket.send_to(send_buffer, stream.remote_endpoint, 0, error);
              }
            }
          }
        }
      }
    }
    return 0;
  }

  int start(){
    BOOST_FOREACH (auto& stream, streams){
      if (stream.tcp == true){
        ip::tcp::socket *my_tcp_socket = new ip::tcp::socket{my_tcp_io_service};
        stream.tcp_socket = my_tcp_socket;
        stream.tcp_socket->connect(ip::tcp::endpoint( boost::asio::ip::address::from_string(stream.address), stream.port ));
      }
    }
    my_socket.open(ip::udp::v4());
    return 0;
  }
  
  int stop(){
    BOOST_FOREACH (auto& stream, streams){
      if (stream.tcp == true){
        stream.tcp_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        stream.tcp_socket->close();
      }
    }
    my_socket.close();
    return 0;
  }

  static boost::shared_ptr<Simple_Stream> create() {
    return boost::shared_ptr<Simple_Stream>(
        new Simple_Stream());
  }
};

BOOST_DLL_ALIAS(
    Simple_Stream::create, // <-- this function is exported with...
    create_plugin             // <-- ...this alias name
)
