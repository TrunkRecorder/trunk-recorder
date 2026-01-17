#include "source.h"
#include "formatter.h"

using json = nlohmann::json;

static int src_counter = 0;

int Source::get_num() {
  return src_num;
};

gr::basic_block_sptr Source::get_src_block() {
  return source_block;
}

Config *Source::get_config() {
  return config;
}

void Source::set_min_max() {
  long s = rate;
  long if_freqs[] = {24000, 25000, 32000};
  long decim = 24000;
  for (int i = 0; i < 3; i++) {
    long if_freq = if_freqs[i];
    if (s % if_freq != 0) {
      continue;
    }
    long q = s / if_freq;
    if (q & 1) {
      continue;
    }

    if ((q >= 40) && ((q & 3) == 0)) {
      decim = q / 4;
    } else {
      decim = q / 2;
    }
  }
  long if1 = rate / decim;
  min_hz = center - ((rate / 2) - (if1 / 2));
  max_hz = center + ((rate / 2) - (if1 / 2));
}

Source::Source(double c, double r, double e, std::string drv, std::string dev, Config *cfg) {
  rate = r;
  center = c;
  error = e;
  bufLength = 0;
  set_min_max();
  driver = drv;
  device = dev;
  config = cfg;
  gain = 0;
  lna_gain = 0;
  tia_gain = 0;
  pga_gain = 0;
  mix_gain = 0;
  if_gain = 0;
  src_num = src_counter++;
  max_digital_recorders = 0;
  max_debug_recorders = 0;
  max_sigmf_recorders = 0;
  max_analog_recorders = 0;
  debug_recorder_port = 0;
  attached_detector = false;
  attached_selector = false;
  next_selector_port = 0;
  autotune_source = false;
  autotune_manager = new AutotuneManager(this);


  recorder_selector = gr::blocks::selector::make(sizeof(gr_complex), 0, 0);

  // parameters for signal_detector_cvf
  float threshold_sensitivity = 0.9;
  bool auto_threshold = true;
  float threshold = -45;
  int fft_len = 1024;
  float average = 0.8;
  float quantization = 0.01;
  float min_bw = 0.0;
  float max_bw = 50000;

  signal_detector = signal_detector_cvf::make(rate, fft_len, 0, threshold, threshold_sensitivity, auto_threshold, average, quantization, min_bw, max_bw, "");
  BOOST_LOG_TRIVIAL(info) << "Made the Signal Detector";

  if (driver == "osmosdr") {
    osmosdr::source::sptr osmo_src;
    std::vector<std::string> gain_names;
    if (dev == "") {
      BOOST_LOG_TRIVIAL(info) << "Source Device not specified";
      osmo_src = osmosdr::source::make();
    } else {
      std::ostringstream msg;

      if (isdigit(dev[0])) {  // Assume this is a serial number and fail back
                              // to using rtl as default
        msg << "rtl=" << dev; // <<  ",buflen=32764,buffers=8";
        BOOST_LOG_TRIVIAL(info) << "Source device name missing, defaulting to rtl device";
      } else {
        msg << dev; // << ",buflen=32764,buffers=8";
      }
      BOOST_LOG_TRIVIAL(info) << "Source Device: " << msg.str();
      osmo_src = osmosdr::source::make(msg.str());
    }
    BOOST_LOG_TRIVIAL(info) << "SOURCE TYPE OSMOSDR (osmosdr)";
    BOOST_LOG_TRIVIAL(info) << "Setting sample rate to: " << FormatSamplingRate(rate);
    osmo_src->set_sample_rate(rate);
    actual_rate = osmo_src->get_sample_rate();
    rate = round(actual_rate);
    BOOST_LOG_TRIVIAL(info) << "Actual sample rate: " << FormatSamplingRate(actual_rate);
    BOOST_LOG_TRIVIAL(info) << "Tuning to " << format_freq(center + error);
    osmo_src->set_center_freq(center + error, 0);
    gain_names = osmo_src->get_gain_names();
    std::string gain_list;
    for (std::vector<std::string>::iterator it = gain_names.begin(); it != gain_names.end(); it++) {
      std::string gain_name = *it;
      osmosdr::gain_range_t range = osmo_src->get_gain_range(gain_name);
      std::vector<double> gains = range.values();
      std::string gain_opt_str;
      for (std::vector<double>::iterator gain_it = gains.begin(); gain_it != gains.end(); gain_it++) {
        double gain_opt = *gain_it;
        std::ostringstream ss;
        // gain_opt = floor(gain_opt * 10) / 10;
        ss << gain_opt << " ";

        gain_opt_str += ss.str();
      }
      BOOST_LOG_TRIVIAL(info) << "Gain Stage: " << gain_name << " supported values: " << gain_opt_str;
    }

    source_block = osmo_src;
  }

  if (driver == "usrp") {
    gr::uhd::usrp_source::sptr usrp_src;
    usrp_src = gr::uhd::usrp_source::make(device, uhd::stream_args_t("fc32"));

    BOOST_LOG_TRIVIAL(info) << "SOURCE TYPE USRP (UHD)";

    BOOST_LOG_TRIVIAL(info) << "Setting sample rate to: " << FormatSamplingRate(rate);
    usrp_src->set_samp_rate(rate);
    actual_rate = usrp_src->get_samp_rate();
    BOOST_LOG_TRIVIAL(info) << "Actual sample rate: " << FormatSamplingRate(actual_rate);
    BOOST_LOG_TRIVIAL(info) << "Tuning to " << format_freq(center + error);
    usrp_src->set_center_freq(center + error, 0);

    source_block = usrp_src;
  }

#ifdef GnuradioIIO_FOUND
  if (driver == "iio") {
    std::vector<bool> enable_channels{1,1,0,0};
    BOOST_LOG_TRIVIAL(info) << "SOURCE TYPE IIO";

    // check to see if device string has bufferLength, and if so, split them
    std::string dev = device;
    bufLength = 32768;

    if (device.find(",")) {
      dev = device.substr(0,device.find(","));
      bufLength = std::stoul(device.substr(device.find(",") + 1, device.length()));
    }

    BOOST_LOG_TRIVIAL(info) << "Device: " << dev;
    BOOST_LOG_TRIVIAL(info) << "Buffer Length: " << bufLength;

    gr::iio::fmcomms2_source<gr_complex>::sptr iio_src;
    iio_src = gr::iio::fmcomms2_source<gr_complex>::make(dev, enable_channels, bufLength);
    iio_src->set_len_tag_key("");
    iio_src->set_gain_mode(0, "manual");
    iio_src->set_gain(0, gain);

    BOOST_LOG_TRIVIAL(info) << "Tuning to " << format_freq(center + error);
    iio_src->set_frequency(center + error);

    
    BOOST_LOG_TRIVIAL(info) << "Setting sample rate to: " << FormatSamplingRate(rate);
    iio_src->set_samplerate(rate);
    actual_rate = rate;
    
    iio_src->set_quadrature(true);
    iio_src->set_rfdc(true);
    iio_src->set_bbdc(true);
    iio_src->set_filter_params("Auto", "", 0.0, 0.0);

    

    source_block = iio_src;
  }
#else // GnuradioIIO_FOUND
  BOOST_LOG_TRIVIAL(fatal) << "Trunk-recorder was not compiled with IIO enabled.";
  exit(1);
#endif //GnuradioIIO_FOUND
}

void Source::set_iq_source(std::string iq_file, bool repeat, double center, double rate) {
  this->rate = rate;
  this->center = center;
  error = 0;
  min_hz = center - ((rate / 2));
  max_hz = center + ((rate / 2));
  driver = "iqfile";
  device = "";
  gain = 0;
  lna_gain = 0;
  tia_gain = 0;
  pga_gain = 0;
  mix_gain = 0;
  if_gain = 0;
  src_num = src_counter++;
  max_digital_recorders = 0;
  max_debug_recorders = 0;
  max_sigmf_recorders = 0;
  max_analog_recorders = 0;
  debug_recorder_port = 0;
  attached_detector = false;
  attached_selector = false;
  next_selector_port = 0;
  autotune_source = false;
  autotune_manager = new AutotuneManager(this);

  iq_file_source::sptr iq_file_src;
  iq_file_src = iq_file_source::make(iq_file, this->rate, repeat);

  BOOST_LOG_TRIVIAL(info) << "SOURCE TYPE IQ FILE";
  BOOST_LOG_TRIVIAL(info) << "Setting Center to: " << FormatSamplingRate(center);
  BOOST_LOG_TRIVIAL(info) << "Setting sample rate to: " << FormatSamplingRate(rate);

  source_block = iq_file_src;
}

Source::Source(std::string sigmf_meta, std::string sigmf_data, bool repeat, Config *cfg) {
  json data;
  std::cout << sigmf_meta << std::endl;
  try {
    std::ifstream f(sigmf_meta);
    data = json::parse(f);
  } catch (const json::parse_error &e) {
    // output exception information
    std::cout << "message: " << e.what() << '\n'
              << "exception id: " << e.id << '\n'
              << "byte position of error: " << e.byte << std::endl;
    exit(1);
  }

  std::cout << data.dump(4) << std::endl;
  json global = data["global"];

  config = cfg;
  this->rate = global["core:sample_rate"];

  json capture = data["captures"][0];
  this->center = capture["core:frequency"];
  std::cout << "Rate: " << rate << "Center: " << center << std::endl;
  set_iq_source(sigmf_data, repeat, center, rate);
}

Source::Source(std::string iq_file, bool repeat, double center, double rate, Config *cfg) {
  config = cfg;
  set_iq_source(iq_file, repeat, center, rate);
}

void Source::set_selector_port_enabled(unsigned int port, bool enabled) {
  recorder_selector->set_port_enabled(port, enabled);
}

bool Source::is_selector_port_enabled(unsigned int port) {
  return recorder_selector->is_port_enabled(port);
}

void Source::attach_selector(gr::top_block_sptr tb) {
  if (!attached_selector) {
    attached_selector = true;
    recorder_selector = gr::blocks::selector::make(sizeof(gr_complex), 0, 0);
    tb->connect(source_block, 0, recorder_selector, 0);
  }
}

void Source::attach_detector(gr::top_block_sptr tb) {
  if (!attached_detector) {
    attached_detector = true;
    tb->connect(source_block, 0, signal_detector, 0);
  }
}

void Source::set_antenna(std::string ant) {
  antenna = ant;

  if (driver == "osmosdr") {
    cast_to_osmo_sptr(source_block)->set_antenna(antenna, 0);
    BOOST_LOG_TRIVIAL(info) << "Setting antenna to [" << cast_to_osmo_sptr(source_block)->get_antenna() << "]";
  }

  if (driver == "usrp") {
    BOOST_LOG_TRIVIAL(info) << "Setting antenna to [" << antenna << "]";
    cast_to_usrp_sptr(source_block)->set_antenna(antenna, 0);
  }
}

std::string Source::get_antenna() {
  return antenna;
}

void Source::set_silence_frames(int m) {
  silence_frames = m;
}

int Source::get_silence_frames() {
  return silence_frames;
}

double Source::get_min_hz() {
  return min_hz;
}

double Source::get_max_hz() {
  return max_hz;
}

double Source::get_center() {
  return center;
}

double Source::get_rate() {
  return rate;
}

bool Source::got_samples() {
  if (attached_selector) {
    return recorder_selector->got_samples();
  }
  return true;
}

std::string Source::get_driver() {
  return driver;
}

std::string Source::get_device() {
  return device;
}

void Source::set_freq_corr(double p) {
  ppm = p;

  if (driver == "osmosdr") {
    cast_to_osmo_sptr(source_block)->set_freq_corr(ppm);
    BOOST_LOG_TRIVIAL(info) << "PPM set to: " << cast_to_osmo_sptr(source_block)->get_freq_corr();
  }
}

void Source::set_error(double e) {
  error = e;
}

double Source::get_error() {
  return error;
}

/* -- Gain -- */

void Source::set_gain(double r) {
  if (driver == "osmosdr") {
    gain = r;
    cast_to_osmo_sptr(source_block)->set_gain(gain);
    double current_gain = cast_to_osmo_sptr(source_block)->get_gain();
    if (current_gain != gain) {
      BOOST_LOG_TRIVIAL(error) << "Requested Gain of " << gain << " not supported, driver using: " << current_gain;
    } 
    BOOST_LOG_TRIVIAL(info) << "Gain set to: " << current_gain;
  }

  if (driver == "usrp") {
    gain = r;
    cast_to_usrp_sptr(source_block)->set_gain(gain);
  }

#ifdef GnuradioIIO_FOUND
  if (driver == "iio") {
    gain = r;
    cast_to_iio_sptr(source_block)->set_gain(0, gain);
    BOOST_LOG_TRIVIAL(info) << "Gain set to: " << gain;
  }
#else // GnuradioIIO_FOUND
  BOOST_LOG_TRIVIAL(fatal) << "Trunk-recorder was not compiled with IIO enabled.";
  exit(1);
#endif //GnuradioIIO_FOUND
}

void Source::add_gain_stage(std::string stage_name, double value) {
  Gain_Stage_t stage = {stage_name, value};
  gain_stages.push_back(stage);
}

std::vector<Gain_Stage_t> Source::get_gain_stages() {
  return gain_stages;
}

void Source::set_gain_by_name(std::string name, double new_gain) {
  if (driver == "osmosdr") {
    cast_to_osmo_sptr(source_block)->set_gain(new_gain, name);
    double current_gain = cast_to_osmo_sptr(source_block)->get_gain(name);
    if (current_gain != new_gain) {
      BOOST_LOG_TRIVIAL(error) << "Requested " << name << " Gain of " << new_gain << " not supported, driver using: " << current_gain;
    }
    BOOST_LOG_TRIVIAL(info) << name << " Gain set to: " << current_gain;
    add_gain_stage(name, new_gain);
  } else {
    BOOST_LOG_TRIVIAL(error) << "Unable to set Gain by Name for SDR driver: " << driver;
  }
}

int Source::get_gain_by_name(std::string name) {
  if (driver == "osmosdr") {
    try {
      return cast_to_osmo_sptr(source_block)->get_gain(name, 0);
    } catch (std::exception &e) {
      BOOST_LOG_TRIVIAL(error) << name << " Gain unsupported or other error: " << e.what();
    }
  } else {
    BOOST_LOG_TRIVIAL(error) << "Unable to get Gain by Name for SDR drive: " << driver;
  }
  return -1;
}

double Source::get_gain() {
  return gain;
}

void Source::set_gain_mode(bool m) {
  if (driver == "osmosdr") {
    gain_mode = m;
    cast_to_osmo_sptr(source_block)->set_gain_mode(gain_mode);
    if (cast_to_osmo_sptr(source_block)->get_gain_mode()) {
      BOOST_LOG_TRIVIAL(info) << "Auto gain control is ON";
    } else {
      BOOST_LOG_TRIVIAL(info) << "Auto gain control is OFF";
    }

  }
#ifdef GnuradioIIO_FOUND
 else if (driver == "iio") {
    gain_mode = m;
    if (gain_mode) {
      cast_to_iio_sptr(source_block)->set_gain_mode(0, "fast_attack");
    } else {
      cast_to_iio_sptr(source_block)->set_gain_mode(0, "manual");
    }
  }
#else // GnuradioIIO_FOUND
  BOOST_LOG_TRIVIAL(fatal) << "Trunk-recorder was not compiled with IIO enabled.";
  exit(1);
#endif
}

double Source::get_if_gain() {
  return if_gain;
}

double Source::get_bb_gain() {
  return bb_gain;
}

double Source::get_mix_gain() {
  return mix_gain;
}

double Source::get_lna_gain() {
  return lna_gain;
}

double Source::get_tia_gain() {
  return tia_gain;
}

double Source::get_pga_gain() {
  return pga_gain;
}

double Source::get_vga1_gain() {
  return vga1_gain;
}

double Source::get_vga2_gain() {
  return vga2_gain;
}

void Source::add_autotune_error_measurement(int error, int offset){
  autotune_manager->add_error_measurement(error, offset);
}

int Source::get_source_error(){
  return autotune_manager->get_average_error();
}

void Source::set_autotune_source(bool m){
  autotune_source = m;
}

bool Source::get_autotune_source() {
  return autotune_source;
}

/* -- Recorders -- */

std::vector<Recorder *> Source::find_conventional_recorders_by_freq(Detected_Signal signal) {
  double freq = center + signal.center_freq;

  std::vector<Recorder *> recorders;
  long max_freq_diff = 12500;
  for (std::vector<p25_recorder_sptr>::iterator it = digital_conv_recorders.begin(); it != digital_conv_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;
    double recorder_freq = rx->get_freq();

    if (std::abs(freq - recorder_freq) < max_freq_diff) {
      recorders.push_back((Recorder *)rx.get());
    }
  }

  for (std::vector<dmr_recorder_sptr>::iterator it = dmr_conv_recorders.begin(); it != dmr_conv_recorders.end(); it++) {
    dmr_recorder_sptr rx = *it;
    double recorder_freq = rx->get_freq();

    if (std::abs(freq - recorder_freq) < max_freq_diff) {
      recorders.push_back((Recorder *)rx.get());
    }
  }

  for (std::vector<analog_recorder_sptr>::iterator it = analog_conv_recorders.begin(); it != analog_conv_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;
    double recorder_freq = rx->get_freq();

    if (std::abs(freq - recorder_freq) < max_freq_diff) {
      recorders.push_back((Recorder *)rx.get());
    }
  }

  return recorders;
}

void Source::enable_detected_recorders() {
  std::vector<Detected_Signal> signals = signal_detector->get_detected_signals();

  for (std::vector<Detected_Signal>::iterator it = signals.begin(); it != signals.end(); it++) {
    Detected_Signal signal = *it;

    float rssi = signal.max_rssi;
    float threshold = signal.threshold;

    std::vector<Recorder *> recorders = find_conventional_recorders_by_freq(signal);
    for (std::vector<Recorder *>::iterator it = recorders.begin(); it != recorders.end(); it++) {
      Recorder *recorder = *it;
      if (!recorder->is_enabled()) {
        recorder->set_enabled(true);
        BOOST_LOG_TRIVIAL(info) << "\t[ " << recorder->get_num() << " ] " << recorder->get_type_string() << "\tEnabled - Freq: " << format_freq(recorder->get_freq()) << "\t Detected Signal: " << floor(rssi) << "dBM (Threshold: " << floor(threshold) << "dBM)";
      }
    }
  }
}

void Source::set_signal_detector_threshold(float threshold) {
BOOST_LOG_TRIVIAL(info) << " - Setting Signal Detector Threshold to: " << threshold;
  signal_detector->set_threshold(threshold);
}

void Source::create_analog_recorders(gr::top_block_sptr tb, int r) {
  if (r > 0) {
    attach_selector(tb);
  }
  max_analog_recorders = r;

  for (int i = 0; i < max_analog_recorders; i++) {
    analog_recorder_sptr log = make_analog_recorder(this, ANALOG);
    analog_recorders.push_back(log);
    log->set_selector_port(next_selector_port);
    tb->connect(recorder_selector, next_selector_port, log, 0);
    next_selector_port++;
  }
}

void Source::create_digital_recorders(gr::top_block_sptr tb, int r) {

  if (r > 0) {
    attach_selector(tb);
  }
  max_digital_recorders = r;

  for (int i = 0; i < max_digital_recorders; i++) {
    p25_recorder_sptr log = make_p25_recorder(this, P25);
    digital_recorders.push_back(log);
    log->set_selector_port(next_selector_port);
    tb->connect(recorder_selector, next_selector_port, log, 0);
    next_selector_port++;
  }
}

void Source::create_sigmf_recorders(gr::top_block_sptr tb, int r) {
  max_sigmf_recorders = r;

  if (r > 0) {
    attach_selector(tb);
  }
  for (int i = 0; i < max_sigmf_recorders; i++) {
    sigmf_recorder_sptr log = make_sigmf_recorder(this, SIGMF);

    sigmf_recorders.push_back(log);
    log->set_selector_port(next_selector_port);
    tb->connect(recorder_selector, next_selector_port, log, 0);
  }
}

analog_recorder_sptr Source::create_conventional_recorder(gr::top_block_sptr tb, float tone_freq) {
  // Not adding it to the vector of analog_recorders. We don't want it to be available for trunk recording.
  // Conventional recorders are tracked seperately in analog_conv_recorders
  attach_detector(tb);
  attach_selector(tb);

  analog_recorder_sptr log = make_analog_recorder(this, ANALOGC, tone_freq);
  analog_conv_recorders.push_back(log);
  log->set_selector_port(next_selector_port);
  tb->connect(recorder_selector, next_selector_port, log, 0);
  next_selector_port++;
  return log;
}

analog_recorder_sptr Source::create_conventional_recorder(gr::top_block_sptr tb) {
  // Not adding it to the vector of analog_recorders. We don't want it to be available for trunk recording.
  // Conventional recorders are tracked seperately in analog_conv_recorders
  attach_detector(tb);
  attach_selector(tb);

  analog_recorder_sptr log = make_analog_recorder(this, ANALOGC);
  analog_conv_recorders.push_back(log);
  log->set_selector_port(next_selector_port);
  tb->connect(recorder_selector, next_selector_port, log, 0);
  next_selector_port++;
  return log;
}
sigmf_recorder_sptr Source::create_sigmf_conventional_recorder(gr::top_block_sptr tb) {
  // Not adding it to the vector of digital_recorders. We don't want it to be available for trunk recording.
  // Conventional recorders are tracked seperately in digital_conv_recorders
  attach_detector(tb);
  attach_selector(tb);
  sigmf_recorder_sptr log = make_sigmf_recorder(this, SIGMFC);
  sigmf_conv_recorders.push_back(log);
  log->set_selector_port(next_selector_port);
  tb->connect(recorder_selector, next_selector_port, log, 0);
  next_selector_port++;
  return log;
}

p25_recorder_sptr Source::create_digital_conventional_recorder(gr::top_block_sptr tb) {
  // Not adding it to the vector of digital_recorders. We don't want it to be available for trunk recording.
  // Conventional recorders are tracked seperately in digital_conv_recorders
  attach_detector(tb);
  attach_selector(tb);

  p25_recorder_sptr log = make_p25_recorder(this, P25C);
  digital_conv_recorders.push_back(log);
  log->set_selector_port(next_selector_port);
  tb->connect(recorder_selector, next_selector_port, log, 0);
  next_selector_port++;
  return log;
}

dmr_recorder_sptr Source::create_dmr_conventional_recorder(gr::top_block_sptr tb) {
  // Not adding it to the vector of digital_recorders. We don't want it to be available for trunk recording.
  // Conventional recorders are tracked seperately in digital_conv_recorders
  attach_detector(tb);
  attach_selector(tb);

  dmr_recorder_sptr log = make_dmr_recorder(this, DMR);
  dmr_conv_recorders.push_back(log);
  log->set_selector_port(next_selector_port);
  tb->connect(recorder_selector, next_selector_port, log, 0);
  next_selector_port++;
  return log;
}

void Source::create_debug_recorder(gr::top_block_sptr tb, int source_num) {
  max_debug_recorders = 1;
  debug_recorder_port = config->debug_recorder_port + source_num;
  debug_recorder_sptr log = make_debug_recorder(this, config->debug_recorder_address, debug_recorder_port);
  debug_recorders.push_back(log);
  tb->connect(source_block, 0, log, 0);
}

Recorder *Source::get_analog_recorder(Talkgroup *talkgroup, int priority, Call *call) {
  int num_available_recorders = get_num_available_analog_recorders();
  std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
  if (talkgroup && (priority == -1)) {
    call->set_state(MONITORING);
    call->set_monitoring_state(IGNORED_TG);
    BOOST_LOG_TRIVIAL(info) << loghdr << "Not recording talkgroup. Priority is -1.";
    return NULL;
  }

  if (talkgroup && priority > num_available_recorders) { // a high priority is bad. You need at least the number of availalbe recorders to your priority
    call->set_state(MONITORING);
    call->set_monitoring_state(NO_RECORDER);
    BOOST_LOG_TRIVIAL(error) << loghdr << "Not recording talkgroup. Priority is " << priority << " but only " << num_available_recorders << " recorders are available.";
    return NULL;
  }

  return get_analog_recorder(call);
}

Recorder *Source::get_analog_recorder(Call *call) {
  for (std::vector<analog_recorder_sptr>::iterator it = analog_recorders.begin();
       it != analog_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;

    if (rx->get_state() == AVAILABLE) {
      return (Recorder *)rx.get();

      break;
    }
  }
  std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
  BOOST_LOG_TRIVIAL(error) << loghdr << "[ " << device << " ] No Analog Recorders Available.";
  return NULL;
}

Recorder *Source::get_digital_recorder(Talkgroup *talkgroup, int priority, Call *call) {
  int num_available_recorders = get_num_available_digital_recorders();
  std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());

  if (talkgroup && (priority == -1)) {
    call->set_state(MONITORING);
    call->set_monitoring_state(IGNORED_TG);
    BOOST_LOG_TRIVIAL(info) << loghdr << "Not recording talkgroup. Priority is -1.";
    return NULL;
  }

  if (talkgroup && priority > num_available_recorders) { // a high priority is bad. You need at least the number of availalbe recorders to your priority
    call->set_state(MONITORING);
    call->set_monitoring_state(NO_RECORDER);
    BOOST_LOG_TRIVIAL(error) << loghdr << "Not recording talkgroup. Priority is " << priority << " but only " << num_available_recorders << " recorders are available.";
    return NULL;
  }

  return get_digital_recorder(call);
}

Recorder *Source::get_digital_recorder(Call *call) {
  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    if (rx->get_state() == AVAILABLE) {
      return (Recorder *)rx.get();

      break;
    }
  }
  std::string loghdr = log_header( call->get_short_name(), call->get_call_num(), call->get_talkgroup_display(), call->get_freq());
  BOOST_LOG_TRIVIAL(error) << loghdr << "[ " << device << " ] No Digital Recorders Available.";

  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;
    BOOST_LOG_TRIVIAL(info) << "[ " << rx->get_num() << " ] State: " << format_state(rx->get_state()) << " Freq: " << rx->get_freq();
  }
  return NULL;
}

Recorder *Source::get_debug_recorder() {
  for (std::vector<debug_recorder_sptr>::iterator it = debug_recorders.begin();
       it != debug_recorders.end(); it++) {
    debug_recorder_sptr rx = *it;

    if (rx->get_state() == INACTIVE) {
      return (Recorder *)rx.get();

      break;
    }
  }
  return NULL;
}

int Source::get_debug_recorder_port() {
  return debug_recorder_port;
}

Recorder *Source::get_sigmf_recorder() {
  for (std::vector<sigmf_recorder_sptr>::iterator it = sigmf_recorders.begin();
       it != sigmf_recorders.end(); it++) {
    sigmf_recorder_sptr rx = *it;

    if (rx->get_state() == INACTIVE) {
      return (Recorder *)rx.get();

      break;
    }
  }
  return NULL;
}

void Source::print_recorders() {
  // If autotune is enabled, show the average correction being applied for this source
  std::string autotune_status;
  if (autotune_source) {
    autotune_status = autotune_manager->get_status_string();
  }

  BOOST_LOG_TRIVIAL(info) << "[ Source " << src_num << ": " << format_freq(center) << " ] " << device << autotune_status;

  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    BOOST_LOG_TRIVIAL(info) << "\t[ " << std::setw(2) << rx->get_num() << " ] " << rx->get_type_string() << "\tState: " << format_state(rx->get_state());
  }

  for (std::vector<p25_recorder_sptr>::iterator it = digital_conv_recorders.begin();
       it != digital_conv_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    BOOST_LOG_TRIVIAL(info) << "\t[ " << std::setw(2) << rx->get_num() << " ] " << rx->get_type_string() << "\tState: " << format_state(rx->get_state());
  }

  for (std::vector<dmr_recorder_sptr>::iterator it = dmr_conv_recorders.begin();
       it != dmr_conv_recorders.end(); it++) {
    dmr_recorder_sptr rx = *it;

    BOOST_LOG_TRIVIAL(info) << "\t[ " << std::setw(2) << rx->get_num() << " ] " << rx->get_type_string() << "\tState: " << format_state(rx->get_state());
  }

  for (std::vector<analog_recorder_sptr>::iterator it = analog_recorders.begin();
       it != analog_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;

    BOOST_LOG_TRIVIAL(info) << "\t[ " << std::setw(2) << rx->get_num() << " ] " << rx->get_type_string() << "\tState: " << format_state(rx->get_state());
  }

  for (std::vector<analog_recorder_sptr>::iterator it = analog_conv_recorders.begin();
       it != analog_conv_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;

    BOOST_LOG_TRIVIAL(info) << "\t[ " << std::setw(2) << rx->get_num() << " ] " << rx->get_type_string() << "\tState: " << format_state(rx->get_state());
  }
}

int Source::digital_recorder_count() {
  return digital_recorders.size() + digital_conv_recorders.size() + dmr_conv_recorders.size();
}

int Source::analog_recorder_count() {
  return analog_recorders.size() + analog_conv_recorders.size();
}

int Source::debug_recorder_count() {
  return debug_recorders.size();
}

int Source::sigmf_recorder_count() {
  return sigmf_recorders.size();
}

int Source::get_num_available_digital_recorders() {
  int num_available_recorders = 0;

  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    if (rx->get_state() == AVAILABLE) {
      num_available_recorders++;
    }
  }
  return num_available_recorders;
}

int Source::get_num_available_analog_recorders() {
  int num_available_recorders = 0;

  for (std::vector<analog_recorder_sptr>::iterator it = analog_recorders.begin(); it != analog_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;

    if (rx->get_state() == AVAILABLE) {
      num_available_recorders++;
    }
  }
  return num_available_recorders;
}

std::vector<Recorder *> Source::get_recorders() {

  std::vector<Recorder *> recorders;

  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin(); it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;
    recorders.push_back((Recorder *)rx.get());
  }

  for (std::vector<p25_recorder_sptr>::iterator it = digital_conv_recorders.begin(); it != digital_conv_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;
    recorders.push_back((Recorder *)rx.get());
  }

  for (std::vector<dmr_recorder_sptr>::iterator it = dmr_conv_recorders.begin(); it != dmr_conv_recorders.end(); it++) {
    dmr_recorder_sptr rx = *it;
    recorders.push_back((Recorder *)rx.get());
  }

  for (std::vector<analog_recorder_sptr>::iterator it = analog_recorders.begin(); it != analog_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;
    recorders.push_back((Recorder *)rx.get());
  }

  for (std::vector<analog_recorder_sptr>::iterator it = analog_conv_recorders.begin(); it != analog_conv_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;
    recorders.push_back((Recorder *)rx.get());
  }

  for (std::vector<debug_recorder_sptr>::iterator it = debug_recorders.begin(); it != debug_recorders.end(); it++) {
    debug_recorder_sptr rx = *it;
    recorders.push_back((Recorder *)rx.get());
  }

  for (std::vector<sigmf_recorder_sptr>::iterator it = sigmf_recorders.begin(); it != sigmf_recorders.end(); it++) {
    sigmf_recorder_sptr rx = *it;
    recorders.push_back((Recorder *)rx.get());
  }
  return recorders;
}
