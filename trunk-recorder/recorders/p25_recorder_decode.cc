
#include "p25_recorder_decode.h"
#include "../gr_blocks/plugin_wrapper_impl.h"
#include "../plugin_manager/plugin_manager.h"
#include "../systems/system_impl.h"
#include "../formatter.h"
#include "../unit_tags_ota.h"

p25_recorder_decode_sptr make_p25_recorder_decode(Recorder *recorder, int silence_frames, bool d_soft_vocoder) {
  p25_recorder_decode *decoder = new p25_recorder_decode(recorder);
  decoder->initialize(silence_frames, d_soft_vocoder);
  return gnuradio::get_initial_sptr(decoder);
}

p25_recorder_decode::p25_recorder_decode(Recorder *recorder)
    : gr::hier_block2("p25_recorder_decode",
                      gr::io_signature::make(1, 1, sizeof(float)),
                      gr::io_signature::make(0, 0, sizeof(float))) {
  d_recorder = recorder;
}

p25_recorder_decode::~p25_recorder_decode() {
}

void p25_recorder_decode::stop() {
  wav_sink->stop_recording();
  d_call = NULL;
}

void p25_recorder_decode::start(Call *call) {
  levels->set_k(call->get_system()->get_digital_levels());

  if(call->get_phase2_tdma()){
    wav_sink->start_recording(call, call->get_tdma_slot());
  } else {
    wav_sink->start_recording(call);
  }
  
  d_call = call;
}

void p25_recorder_decode::set_xor_mask(const char *mask) {
  op25_frame_assembler->set_xormask(mask);
}

void p25_recorder_decode::set_source(long src) {
  wav_sink->set_source(src);
}

std::vector<Transmission> p25_recorder_decode::get_transmission_list() {
  return wav_sink->get_transmission_list();
}

void p25_recorder_decode::set_tdma_slot(int slot) {

  tdma_slot = slot;
  op25_frame_assembler->set_slotid(tdma_slot);
}
double p25_recorder_decode::get_current_length() {
  return wav_sink->total_length_in_seconds();
}

State p25_recorder_decode::get_state() {
  return wav_sink->get_state();
}

double p25_recorder_decode::since_last_write() {
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - wav_sink->get_last_write_time();
  return diff.count();
}

void p25_recorder_decode::switch_tdma(bool phase2_tdma) {
  op25_frame_assembler->set_phase2_tdma(phase2_tdma);
}

void p25_recorder_decode::initialize(int silence_frames, bool d_soft_vocoder) {
  // OP25 Slicer
  const float l[] = {-2.0, 0.0, 2.0, 4.0};
  std::vector<float> slices(l, l + sizeof(l) / sizeof(l[0]));
  const int msgq_id = 0;
  const int debug = 0;
  slicer = gr::op25_repeater::fsk4_slicer_fb::make(msgq_id, debug, slices);
  wav_sink = gr::blocks::transmission_sink::make(1, 8000, 16);
  // recorder->initialize(src);

  bool use_streaming = d_recorder->get_enable_audio_streaming();

  // OP25 Frame Assembler
  traffic_queue = gr::msg_queue::make(2);
  rx_queue = gr::msg_queue::make(100);

  int udp_port = 0;
  int verbosity = 0; // 10 = lots of debug messages
  const char *udp_host = "127.0.0.1";
  bool do_imbe = 1;
  bool do_output = 1;
  bool do_msgq = 1;
  bool do_audio_output = 1;
  bool do_tdma = 0;
  bool do_nocrypt = 1;

  op25_frame_assembler = gr::op25_repeater::p25_frame_assembler::make(silence_frames, d_soft_vocoder, udp_host, udp_port, verbosity, do_imbe, do_output, do_msgq, rx_queue, do_audio_output, do_tdma, do_nocrypt);
  levels = gr::blocks::multiply_const_ss::make(1);

  if (use_streaming) {
    plugin_sink = gr::blocks::plugin_wrapper_impl::make(std::bind(&p25_recorder_decode::plugin_callback_handler, this, std::placeholders::_1, std::placeholders::_2));
  }

  connect(self(), 0, slicer, 0);
  connect(slicer, 0, op25_frame_assembler, 0);
  connect(op25_frame_assembler, 0, levels, 0);

  if (use_streaming) {
    connect(levels, 0, plugin_sink, 0);
  }
  connect(levels, 0, wav_sink, 0);
}

void p25_recorder_decode::plugin_callback_handler(int16_t *samples, int sampleCount) {
  if (d_call) {
    plugman_audio_callback(d_call, d_recorder, samples, sampleCount);
  }
}

double p25_recorder_decode::get_output_sample_rate() {
  return 8000;
}

// This lead to weird Segfaults. The concept is trying to clear out the buffers for a new call
void p25_recorder_decode::reset_block(gr::basic_block_sptr block) {
  gr::block_detail_sptr detail;
  gr::block_sptr grblock = cast_to_block_sptr(block);
  detail = grblock->detail();
  //detail->reset_nitem_counters();
  detail->clear_tags();
}

void p25_recorder_decode::reset() {
  /*reset_block(op25_frame_assembler);
  reset_block(slicer);
  reset_block(levels);
  reset_block(wav_sink);*/
  op25_frame_assembler->clear();
}

gr::op25_repeater::p25_frame_assembler::sptr p25_recorder_decode::get_transmission_sink() {
  return op25_frame_assembler;
}

void p25_recorder_decode::handle_alias_message(const nlohmann::json& j) {
  int messages = j["messages"];
  std::array<std::vector<uint8_t>, 10> alias_buffer;
  
  // Decode hex strings back to binary
  for (auto& [key, value] : j["blocks"].items()) {
    int idx = std::stoi(key);
    if (idx >= 0 && idx < 10) {
      std::string hex_str = value.get<std::string>();
      alias_buffer[idx].clear();
      alias_buffer[idx].reserve(hex_str.length() / 2);
      for (size_t i = 0; i + 1 < hex_str.length(); i += 2) {
        uint8_t byte = static_cast<uint8_t>(std::stoul(hex_str.substr(i, 2), nullptr, 16));
        alias_buffer[idx].push_back(byte);
      }
    }
  }
  
  OTAAlias result;
  if (j["type"] == "motorola_alias_p2") {
    result = UnitTagsOTA::decode_motorola_alias_p2(alias_buffer, messages);
  } else {
    result = UnitTagsOTA::decode_motorola_alias(alias_buffer, messages);
  }
  
  if (result.success && !result.alias.empty()) {
    std::string loghdr = log_header(d_call->get_short_name(),d_call->get_call_num(),d_call->get_talkgroup_display(),d_call->get_freq());
    
    BOOST_LOG_TRIVIAL(debug) << loghdr << "Alias OTA: " << result.radio_id << " = \"" << result.alias << "\" [" << result.source << "]";
    
    System *sys = d_call->get_system();
    if (sys) {
      System_impl *sys_impl = dynamic_cast<System_impl*>(sys);
      if (sys_impl && sys_impl->unit_tags) {
        bool added = sys_impl->unit_tags->add_ota(result);
        if (added) {
          BOOST_LOG_TRIVIAL(info) << loghdr << Color::BMAG << "New " << result.source << " alias: " << Color::RST 
                                  << result.radio_id << " (" << Color::BLU << result.alias << Color::RST << ")"; 
        } else {
          BOOST_LOG_TRIVIAL(debug) << loghdr << "Alias for " << result.radio_id << " already exists";
        }
      }
    }
  }
}

void p25_recorder_decode::check_message_queue() {
  if (!rx_queue || !d_call) {
    return;
  }

  gr::message::sptr msg;
  while ((msg = rx_queue->delete_head_nowait())) {
    long msg_type = msg->type();
    
    if (msg_type == -3) { // M_P25_JSON_DATA
      std::string msg_str(msg->to_string());
      
      try {
        auto j = nlohmann::json::parse(msg_str);
        
        if (j.contains("type")) {
          std::string json_msg_type = j["type"];
          
          if (json_msg_type == "motorola_alias_p1" || json_msg_type == "motorola_alias_p2") {
            handle_alias_message(j);
          }
          // Add some more JSON handlers as we find other things to decode!
        }
        
      } catch (const std::exception& e) {
        BOOST_LOG_TRIVIAL(debug) << "Malformed P25 JSON message: " << e.what();
      }
    }
  }
}