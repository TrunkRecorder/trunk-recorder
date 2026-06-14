#include "iq_file_source.h"
#include <cstdlib>


iq_file_source::sptr
iq_file_source::make(std::string filename,  double rate, bool repeat=false) {
  return gnuradio::get_initial_sptr(new iq_file_source(filename, rate, repeat));
}

iq_file_source::iq_file_source(std::string filename,  double rate, bool repeat=false)
    : gr::hier_block2("iq_file_source",
                 gr::io_signature::make(0, 0, 0),
                 gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_filename(filename),
      d_rate(rate),
      d_repeat(repeat) {

    file_source = gr::blocks::file_source::make(sizeof(gr_complex), filename.c_str(), repeat);
    // Allow IQ playback faster than wall-clock by overriding throttle. Useful
    // for offline analysis on short captures; setting TR_IQ_FILE_THROTTLE_X to
    // 10 plays the file 10x real time.
    const char *thx = std::getenv("TR_IQ_FILE_THROTTLE_X");
    double throttle_x = thx ? std::atof(thx) : 1.0;
    throttle = gr::blocks::throttle::make(sizeof(gr_complex), rate * throttle_x);
    connect(file_source, 0, throttle, 0);
    connect(throttle, 0, self(), 0);
}


  