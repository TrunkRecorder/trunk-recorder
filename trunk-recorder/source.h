#ifndef SOURCE_H
#define SOURCE_H
#include "./global_structs.h"
#include "./gr_blocks/selector.h"
#include "./gr_blocks/signal_detector_cvf.h"
#include "./autotune.h"
#include "recorders/analog_recorder.h"
#include "recorders/debug_recorder.h"
#include "recorders/dmr_recorder.h"
#include "recorders/p25_recorder.h"
#include "recorders/sigmf_recorder.h"
#include "sources/iq_file_source.h"
#include <gnuradio/basic_block.h>
#include <gnuradio/top_block.h>
#include <gnuradio/uhd/usrp_source.h>
#ifdef GnuradioIIO_FOUND
  #include <gnuradio/iio/fmcomms2_source.h>
#endif
#include <iostream>
#include <numeric>
#include <osmosdr/source.h>

#include <json.hpp>

struct Gain_Stage_t {
  std::string stage_name;
  double value;
};

class Source {

  int src_num;
  double min_hz;
  double max_hz;
  double center;
  double rate;
  double actual_rate;
  double error;
  double ppm;
  bool attached_detector;
  bool attached_selector;
  bool gain_mode;
  double gain;
  double bb_gain;
  double if_gain;
  double lna_gain;
  double tia_gain;
  double pga_gain;
  double mix_gain;
  double vga1_gain;
  double vga2_gain;
  int max_digital_recorders;
  int max_debug_recorders;
  int max_sigmf_recorders;
  int max_analog_recorders;
  int debug_recorder_port;
  int next_selector_port;
  int silence_frames;
  unsigned long bufLength;
  Config *config;
  bool autotune_source;

  std::vector<p25_recorder_sptr> digital_recorders;
  std::vector<p25_recorder_sptr> digital_conv_recorders;
  std::vector<debug_recorder_sptr> debug_recorders;
  std::vector<sigmf_recorder_sptr> sigmf_recorders;
  std::vector<sigmf_recorder_sptr> sigmf_conv_recorders;
  std::vector<analog_recorder_sptr> analog_recorders;
  std::vector<analog_recorder_sptr> analog_conv_recorders;
  std::vector<dmr_recorder_sptr> dmr_conv_recorders;
  std::vector<Gain_Stage_t> gain_stages;
  std::string driver;
  std::string device;
  std::string antenna;
  gr::basic_block_sptr source_block;
  gr::blocks::selector::sptr recorder_selector;
  signal_detector_cvf::sptr signal_detector;

  void add_gain_stage(std::string stage_name, double value);

public:
  int get_num();
  Config *get_config();
  Source(double c, double r, double e, std::string driver, std::string device, Config *cfg);
  Source(std::string sigmf_meta, std::string sigmf_data, bool repeat, Config *cfg);
  Source(std::string iq_file, bool repeat, double center, double rate, Config *cfg);
  void set_iq_source(std::string iq_file, bool repeat, double center, double rate);
  gr::basic_block_sptr get_src_block();
  void attach_detector(gr::top_block_sptr tb);
  void attach_selector(gr::top_block_sptr tb);
  double get_min_hz();
  double get_max_hz();
  void set_min_max();

  void set_silence_frames(int m);
  int get_silence_frames();

  double get_center();
  double get_rate();
  bool got_samples();
  std::string get_driver();
  std::string get_device();
  void set_antenna(std::string ant);
  std::string get_antenna();
  void set_error(double e);
  double get_error();
  void set_freq_corr(double p);

  /* -- Gain -- */
  void set_if_gain(double i);
  double get_if_gain();
  void set_gain_mode(bool m);
  bool get_gain_mode();
  void set_gain(double r);
  std::vector<Gain_Stage_t> get_gain_stages();
  int get_gain_by_name(std::string name);
  void set_gain_by_name(std::string name, double r);
  double get_gain();
  double get_bb_gain();
  double get_mix_gain();
  double get_lna_gain();
  double get_tia_gain();
  double get_pga_gain();
  double get_vga1_gain();
  double get_vga2_gain();

  /* -- Recorders -- */
  void print_recorders();
  void tune_digital_recorders();
  int debug_recorder_count();
  int get_debug_recorder_port();
  int sigmf_recorder_count();
  int digital_recorder_count();
  int analog_recorder_count();
  int get_num_available_analog_recorders();
  int get_num_available_digital_recorders();
  void set_signal_detector_threshold(float t);
  std::vector<Recorder *> find_conventional_recorders_by_freq(Detected_Signal ds);
  void enable_detected_recorders();
  void set_selector_port_enabled(unsigned int port, bool enabled);
  bool is_selector_port_enabled(unsigned int port);
  void create_debug_recorder(gr::top_block_sptr tb, int source_num);
  void create_sigmf_recorders(gr::top_block_sptr tb, int r);
  void create_analog_recorders(gr::top_block_sptr tb, int r);
  void create_digital_recorders(gr::top_block_sptr tb, int r);

  analog_recorder_sptr create_conventional_recorder(gr::top_block_sptr tb);
  analog_recorder_sptr create_conventional_recorder(gr::top_block_sptr tb, float tone_freq);
  sigmf_recorder_sptr create_sigmf_conventional_recorder(gr::top_block_sptr tb);
  p25_recorder_sptr create_digital_conventional_recorder(gr::top_block_sptr tb);
  dmr_recorder_sptr create_dmr_conventional_recorder(gr::top_block_sptr tb);

  Recorder *get_digital_recorder(Call *call);
  Recorder *get_digital_recorder(Talkgroup *talkgroup, int priority, Call *call);
  Recorder *get_analog_recorder(Call *call);
  Recorder *get_analog_recorder(Talkgroup *talkgroup, int priority, Call *call);
  Recorder *get_debug_recorder();
  Recorder *get_sigmf_recorder();
  std::vector<Recorder *> get_recorders();

  AutotuneManager *autotune_manager;
  void set_autotune_source(bool m);
  bool get_autotune_source();
  void add_autotune_error_measurement(int error, int offset);
  int get_source_error();

#if GNURADIO_VERSION < 0x030900
  inline osmosdr::source::sptr cast_to_osmo_sptr(gr::basic_block_sptr p) {
    return boost::dynamic_pointer_cast<osmosdr::source, gr::basic_block>(p);
  }
  inline gr::uhd::usrp_source::sptr cast_to_usrp_sptr(gr::basic_block_sptr p) {
    return boost::dynamic_pointer_cast<gr::uhd::usrp_source, gr::basic_block>(p);
  }
#ifdef GnuradioIIO_FOUND
  inline gr::iio::fmcomms2_source<gr_complex>::sptr cast_to_iio_sptr(gr::basic_block_sptr p) {
      return boost::dynamic_pointer_cast<gr::iio::fmcomms2_source<gr_complex>, gr::basic_block>(p);
  }
#endif // GnuradioIIO_FOUND
#else
  inline osmosdr::source::sptr cast_to_osmo_sptr(gr::basic_block_sptr p) {
    return std::dynamic_pointer_cast<osmosdr::source, gr::basic_block>(p);
  }
  inline gr::uhd::usrp_source::sptr cast_to_usrp_sptr(gr::basic_block_sptr p) {
    return std::dynamic_pointer_cast<gr::uhd::usrp_source, gr::basic_block>(p);
  }
#ifdef GnuradioIIO_FOUND
  inline gr::iio::fmcomms2_source<gr_complex>::sptr cast_to_iio_sptr(gr::basic_block_sptr p) {
      return std::dynamic_pointer_cast<gr::iio::fmcomms2_source<gr_complex>, gr::basic_block>(p);
  }
#endif // GnuradioIIO_FOUND

#endif
};
#endif
