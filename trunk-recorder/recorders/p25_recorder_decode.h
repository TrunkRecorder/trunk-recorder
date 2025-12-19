#ifndef P25_RECORDER_DECODE_H
#define P25_RECORDER_DECODE_H

#include <boost/shared_ptr.hpp>
#include <gnuradio/block.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/blocks/short_to_float.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/msg_queue.h>

#include <op25_repeater/fsk4_slicer_fb.h>
#include <op25_repeater/costas_loop_cc.h>
#include <op25_repeater/gardner_cc.h>
#include <op25_repeater/include/op25_repeater/fsk4_demod_ff.h>
#include <op25_repeater/include/op25_repeater/p25_frame_assembler.h>
#include <op25_repeater/include/op25_repeater/rx_status.h>
#include <op25_repeater/vocoder.h>

#if GNURADIO_VERSION < 0x030800
#include <gnuradio/blocks/multiply_const_ss.h>
#else
#include <gnuradio/blocks/multiply_const.h>
#endif

#include "../gr_blocks/plugin_wrapper.h"
#include "../gr_blocks/transmission_sink.h"
#include "recorder.h"

class p25_recorder_decode;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<p25_recorder_decode> p25_recorder_decode_sptr;
#else
typedef std::shared_ptr<p25_recorder_decode> p25_recorder_decode_sptr;
#endif

p25_recorder_decode_sptr make_p25_recorder_decode(Recorder *recorder, int silence_frames, bool d_soft_vocoder);

class p25_recorder_decode : public gr::hier_block2 {
  friend p25_recorder_decode_sptr make_p25_recorder_decode(Recorder *recorder, int silence_frames, bool d_soft_vocoder);

protected:
  virtual void initialize(int silence_frames, bool d_soft_vocoder);
  Recorder *d_recorder;
  Call *d_call;
  gr::op25_repeater::p25_frame_assembler::sptr op25_frame_assembler;
  gr::msg_queue::sptr traffic_queue;
  gr::msg_queue::sptr rx_queue;
  gr::op25_repeater::fsk4_slicer_fb::sptr slicer;
  gr::blocks::short_to_float::sptr converter;
  gr::blocks::multiply_const_ss::sptr levels;
  gr::blocks::transmission_sink::sptr wav_sink;
  gr::blocks::plugin_wrapper::sptr plugin_sink;

public:
  p25_recorder_decode(Recorder *recorder);
  void set_tdma_slot(int slot);
  std::vector<Transmission> get_transmission_list();
  void set_source(long src);
  void set_xor_mask(const char *mask);
  void switch_tdma(bool phase2_tdma);
  void start(Call *call);
  double since_last_write();
  void stop();
  void reset();
  void reset_block(gr::basic_block_sptr block); 
  int tdma_slot;
  bool delay_open;
  virtual ~p25_recorder_decode();
  double get_current_length();
  void plugin_callback_handler(int16_t *samples, int sampleCount);
  double get_output_sample_rate();
  State get_state();
  gr::op25_repeater::p25_frame_assembler::sptr get_transmission_sink();
  void check_message_queue();

private:
  void handle_alias_message(const nlohmann::json& j);
};
#endif