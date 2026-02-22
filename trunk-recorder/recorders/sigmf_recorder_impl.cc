
#include "sigmf_recorder_impl.h"
#include <boost/log/trivial.hpp>
#include <cmath>

// static int rec_counter=0;



sigmf_recorder_sptr make_sigmf_recorder(Source *src, Recorder_Type type) {
  sigmf_recorder *recorder = new sigmf_recorder_impl(src, type);

  return gnuradio::get_initial_sptr(recorder);
}

sigmf_recorder_impl::sigmf_recorder_impl(Source *src, Recorder_Type type)
    : gr::hier_block2("sigmf_recorder",
                      gr::io_signature::make(1, 1, sizeof(gr_complex)),
                      gr::io_signature::make(0, 0, sizeof(float))),
      Recorder(type) {

        if (type == SIGMFC) {
          conventional = true;
        } else if (type == SIGMF) {
          conventional = false;
        } else {
          BOOST_LOG_TRIVIAL(error) << "Trying to SIGMF Recorder to another type of recorder";
          exit(1);
        }
  source = src;
  freq = source->get_center();
  center = source->get_center();
  config = source->get_config();
  silence_frames = source->get_silence_frames();
  squelch_db = 0;
  input_rate = source->get_rate();
  talkgroup = 0;
  recording_count = 0;
  recording_duration = 0;

  rec_num = rec_counter++;

  state = INACTIVE;

  // double symbol_rate         = 4800;

  timestamp = time(NULL);
  starttime = time(NULL);



  // tm *ltm = localtime(&starttime);

  filename = std::to_string(talkgroup) + "-" + std::to_string(starttime) + "_" + std::to_string(freq) + ".raw";
  raw_sink = gr::blocks::file_sink::make(sizeof(gr_complex), filename.c_str());

  //initialize_prefilter();
  //initialize_prefilter_xlat();
  
  prefilter = xlat_channelizer::make(input_rate, channelizer::phase1_samples_per_symbol, channelizer::phase1_symbol_rate, xlat_channelizer::channel_bandwidth, center, conventional);
  set_enabled(false);
  connect(squelch, 0, raw_sink, 0);
}

int sigmf_recorder_impl::get_num() {
  return rec_num;
}

bool sigmf_recorder_impl::is_enabled() {
  return source->is_selector_port_enabled(selector_port);
}

void sigmf_recorder_impl::set_enabled(bool enabled) {
  source->set_selector_port_enabled(selector_port, enabled);
}

bool sigmf_recorder_impl::is_active() {
  if (state == ACTIVE) {
    return true;
  } else {
    return false;
  }
}

double sigmf_recorder_impl::get_freq() {
  return freq;
}

int sigmf_recorder_impl::get_freq_error() { // get frequency error from FLL and convert to Hz
  return prefilter->get_freq_error();
}

double sigmf_recorder_impl::get_current_length() {
  return 0;
}

int sigmf_recorder_impl::lastupdate() {
  return time(NULL) - timestamp;
}

long sigmf_recorder_impl::elapsed() {
  return time(NULL) - starttime;
}
/*
void sigmf_recorder_impl::tune_offset(double f) {
  // have to flip this for 3.7
  // BOOST_LOG_TRIVIAL(info) << "Offset set to: " << offset_amount << " Freq: "
  //  << freq;
  freq_xlat->set_center_freq(-f);
}*/

State sigmf_recorder_impl::get_state() {
  return state;
}

void sigmf_recorder_impl::stop() {
  if (state == ACTIVE) {
    std::string loghdr = log_header(this->call->get_short_name(),this->call->get_call_num(),this->call->get_talkgroup_display(),freq);
    BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[32mStopping SigMF Recorder Num [" << rec_num << "]\u001b[0m";

    state = INACTIVE;
    set_enabled(false);
    raw_sink->close();
  } else {
    BOOST_LOG_TRIVIAL(error) << "sigmf_recorder.cc: Trying to Stop an Inactive Logger!!!";
  }
}

bool sigmf_recorder_impl::start(Call *call) {
  if (state == INACTIVE) {
    timestamp = time(NULL);
    starttime = time(NULL);
    tm *ltm = localtime(&starttime);
    this->call = call;
    System *system = call->get_system();
    talkgroup = call->get_talkgroup();
    freq = call->get_freq();
    
    int offset_amount = (center - freq);
    prefilter->tune_offset(offset_amount);
    
    //freq_xlat->set_center_freq(-offset_amount);
    std::string loghdr = log_header(this->call->get_short_name(),this->call->get_call_num(),this->call->get_talkgroup_display(),freq);
    BOOST_LOG_TRIVIAL(info) << loghdr << "\u001b[32mStarting SigMF Recorder Num [" << rec_num << "]\u001b[0m";

    std::stringstream path_stream;

    // Found some good advice on Streams and Strings here: https://blog.sensecodons.com/2013/04/dont-let-stdstringstreamstrcstr-happen.html
    path_stream << call->get_temp_dir() << "/" << call->get_short_name() << "/" << 1900 + ltm->tm_year << "/" << 1 + ltm->tm_mon << "/" << ltm->tm_mday;
    std::string path_string = path_stream.str();
    boost::filesystem::create_directories(path_string);

    filename = path_string + "/" + std::to_string(talkgroup) + "-" + std::to_string(starttime) + "_" +
               std::to_string(static_cast<long>(std::llround(call->get_freq()))) + "-call_" +
               std::to_string(call->get_call_num()) + ".sigmf-data";

    raw_sink->open(filename.c_str());
    state = ACTIVE;

  if (conventional) {
    Call_conventional *conventional_call = dynamic_cast<Call_conventional *>(call);
    squelch_db = conventional_call->get_squelch_db();
    if (conventional_call->get_signal_detection()) {
      set_enabled(false);
    } else {
      set_enabled(true); // If signal detection is not being used, open up the Value/Selector from the start
    }
  } else {
    squelch_db = system->get_squelch_db();
    set_enabled(true);
  }
  prefilter->set_squelch_db(squelch_db);

    std::string src_description = source->get_driver() + ": " + source->get_device() + " - " + source->get_antenna();
    time_t now;
    time(&now);
    char buf[sizeof "2011-10-08T07:07:09Z"];
    strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
    std::string start_time(buf);
    nlohmann::json j = {
      {"global", {
        {"core:datatype", "cf32_le"},
        {"core:sample_rate", channelizer::phase1_samples_per_symbol * channelizer::phase1_symbol_rate},
        {"core:hw", src_description},
        {"core:recorder", "Trunk Recorder"},
        {"core:version", "1.0.0"}
      }},
      {"captures", nlohmann::json::array(
        { nlohmann::json::object({
          {"core:sample_start", 0},
          {"core:frequency", freq},
          {"core:datetime", start_time}
        })
        }
      )},
      {"annotations", nlohmann::json::array({})}
    };

    filename = path_string + "/" + std::to_string(talkgroup) + "-" + std::to_string(starttime) + "_" +
               std::to_string(static_cast<long>(std::llround(call->get_freq()))) + "-call_" +
               std::to_string(call->get_call_num()) + ".sigmf-meta";
    std::ofstream o(filename);
    o << std::setw(4) << j << std::endl;
    o.close();

  } else {
    BOOST_LOG_TRIVIAL(error) << "sigmf_recorder.cc: Trying to Start an already Active Logger!!!";
  }
  return true;
}
