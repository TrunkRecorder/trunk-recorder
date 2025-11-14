/* -*- c++ -*- */

/*
 * Copyright 2004,2006-2011,2013 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "transmission_sink.h"
#include "../../trunk-recorder/call.h"
#include <boost/filesystem.hpp>
#include <boost/math/special_functions/round.hpp>
#include <climits>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/thread/thread.h>
#include <stdexcept>
#include <stdio.h>

// win32 (mingw/msvc) specific
#ifdef HAVE_IO_H
#include <io.h>
#endif // ifdef HAVE_IO_H
#ifdef O_BINARY
#define OUR_O_BINARY O_BINARY
#else // ifdef O_BINARY
#define OUR_O_BINARY 0
#endif // ifdef O_BINARY

// should be handled via configure
#ifdef O_LARGEFILE
#define OUR_O_LARGEFILE O_LARGEFILE
#else // ifdef O_LARGEFILE
#define OUR_O_LARGEFILE 0
#endif // ifdef O_LARGEFILE

namespace gr {
namespace blocks {
transmission_sink::sptr
transmission_sink::make(int n_channels, unsigned int sample_rate, int bits_per_sample) {
  return gnuradio::get_initial_sptr(new transmission_sink(n_channels, sample_rate, bits_per_sample));
}

transmission_sink::transmission_sink(int n_channels, unsigned int sample_rate, int bits_per_sample)
    : sync_block("transmission_sink",
                 io_signature::make(1, n_channels, sizeof(int16_t)),
                 io_signature::make(0, 0, 0)),
      d_sample_rate(sample_rate),
      d_nchans(n_channels),
      d_current_call(NULL),
      d_fp(0) {

  if ((bits_per_sample != 8) && (bits_per_sample != 16)) {
    throw std::runtime_error("Invalid bits per sample (supports 8 and 16)");
  }
  d_bytes_per_sample = bits_per_sample / 8;
  d_sample_count = 0;
  d_slot = -1;
  d_termination_flag = false;
  state = AVAILABLE;
}

void transmission_sink::create_filename() {
  time_t work_start_time = d_start_time;
  std::stringstream temp_path_stream;
  // Found some good advice on Streams and Strings here: https://blog.sensecodons.com/2013/04/dont-let-stdstringstreamstrcstr-happen.html

  temp_path_stream << d_current_call_temp_dir << "/" << d_current_call_short_name;
  std::string temp_path_string = temp_path_stream.str();
  boost::filesystem::create_directories(temp_path_string);

  int nchars;

  if (d_slot == -1) {
    nchars = snprintf(current_filename, 255, "%s/%ld-%ld_%.0f.wav", temp_path_string.c_str(), d_current_call_talkgroup, work_start_time, d_current_call_freq);
  } else {
    // this is for the case when it is a P25P2 TDMA or DMR recorder and 2 wav files are created, the slot is needed to keep them separate.
    nchars = snprintf(current_filename, 255, "%s/%ld-%ld_%.0f.%d.wav", temp_path_string.c_str(), d_current_call_talkgroup, work_start_time, d_current_call_freq, d_slot);
  }
  if (nchars >= 255) {
    BOOST_LOG_TRIVIAL(error) << "Call: Path longer than 255 charecters";
  }
}

char *transmission_sink::get_filename() {
  return current_filename;
}

bool transmission_sink::start_recording(Call *call, int slot) {
  this->d_slot = slot;
  this->start_recording(call);
  return true;
}

bool transmission_sink::start_recording(Call *call) {
  gr::thread::scoped_lock guard(d_mutex);
  if (d_current_call && d_fp) {
    BOOST_LOG_TRIVIAL(trace) << "Start() - Current_Call & fp are not null! current_filename is: " << current_filename << " Length: " << d_sample_count << std::endl;
  }
  d_current_call = call;
  d_current_call_num = call->get_call_num();
  d_current_call_freq = call->get_freq();
  d_conventional = call->is_conventional();
  if (d_conventional && (call->get_system_type() == "conventionalDMR")) {
    BOOST_LOG_TRIVIAL(debug) << "transmission_sink::start_recording - Conventional DMR - dynamically assigning talkgroups";
    d_current_call_talkgroup = 0;
    d_current_call_talkgroup_display = "N/A";
    d_current_call_talkgroup_encoded = 0;
  } else {
    d_current_call_talkgroup = call->get_talkgroup();
    d_current_call_talkgroup_display = call->get_talkgroup_display();
    if (call->get_system_type() == "smartnet") {
      d_current_call_talkgroup_encoded = (call->get_talkgroup() >> 4);
    } else {
      d_current_call_talkgroup_encoded = call->get_talkgroup();
    }
  }
  d_current_call_short_name = call->get_short_name();
  d_current_call_temp_dir = call->get_temp_dir();
  d_prior_transmission_length = 0;
  d_error_count = 0;
  d_spike_count = 0;
  d_current_color_code = -1;
  d_last_write_time = std::chrono::steady_clock::now(); // we want to make sure the call doesn't get cleaned up before data starts coming in.

  this->clear_transmission_list();


  curr_src_id = d_current_call->get_current_source_id();
  d_sample_count = 0;

  // when a wav_sink first gets associated with a call, set its lifecycle to idle;
  state = IDLE;
  /* Should reset more variables here */
  std::string loghdr = log_header(d_current_call_short_name,d_current_call_num,d_current_call_talkgroup_display,d_current_call_freq);
  BOOST_LOG_TRIVIAL(trace) << loghdr << "Starting wavfile sink SRC ID: " << curr_src_id << " Conventional: " << d_conventional;

  return true;
}

bool transmission_sink::open_internal(const char *filename) {
  // we use the open system call to get access to the O_LARGEFILE flag.
  //  O_APPEND|
  int fd;

  if ((fd = ::open(filename,
                   O_RDWR | O_CREAT | OUR_O_LARGEFILE | OUR_O_BINARY,
                   0664)) < 0) {
    perror(filename);
    BOOST_LOG_TRIVIAL(error) << "wav error opening: " << filename << std::endl;
    return false;
  }

  if (d_fp) { // if we've already got a new one open, close it
    BOOST_LOG_TRIVIAL(trace) << "File pointer already open, closing " << d_fp << " more" << current_filename << " for " << filename << std::endl;

    // fclose(d_fp);
    // d_fp = NULL;
  }

  if (strlen(filename) >= 255) {
    BOOST_LOG_TRIVIAL(error) << "transmission_sink: Error! filename longer than 255";
  }

  if ((d_fp = fdopen(fd, "rb+")) == NULL) {
    perror(filename);
    ::close(fd); // don't leak file descriptor if fdopen fails.
    BOOST_LOG_TRIVIAL(error) << "wav open failed" << std::endl;
    return false;
  }
  if (std::setvbuf(d_fp, nullptr, _IOFBF, 1000000) != 0) {
    BOOST_LOG_TRIVIAL(error) << "setvbuf failed"; // POSIX version sets errno
  }
  d_sample_count = 0;

  if (!wavheader_write(d_fp, d_sample_rate, d_nchans, d_bytes_per_sample)) {
    fprintf(stderr, "[%s] could not write to WAV file\n", __FILE__);
    return false;
  }

  if (d_bytes_per_sample == 1) {
    d_max_sample_val = UCHAR_MAX;
    d_min_sample_val = 0;
    d_normalize_fac = d_max_sample_val / 2;
    d_normalize_shift = 1;
  } else if (d_bytes_per_sample == 2) {
    d_max_sample_val = SHRT_MAX;
    d_min_sample_val = SHRT_MIN;
    d_normalize_fac = d_max_sample_val;
    d_normalize_shift = 0;
  }

  return true;
}

void transmission_sink::set_source(long src) {
  std::string loghdr = log_header(d_current_call_short_name,d_current_call_num,d_current_call_talkgroup_display,d_current_call_freq);
  if (curr_src_id == -1) {

    BOOST_LOG_TRIVIAL(info) << loghdr << "Unit ID set via Control Channel, ext: " << src << "\tcurrent: " << curr_src_id << "\t samples: " << d_sample_count;

    curr_src_id = src;
  }
  else if (d_conventional && (src != curr_src_id)) {
    if ((state == RECORDING) && (d_sample_count > 0)) {
        gr::thread::scoped_lock guard(d_mutex);
        BOOST_LOG_TRIVIAL(error) << loghdr << "Unit ID externally set, ext: " << src << "\tcurrent: " << curr_src_id << "\t samples: " << d_sample_count;
        end_transmission();
        state = IDLE;
        curr_src_id = src;
    }

  }
}

void transmission_sink::end_transmission() {
  if (d_sample_count > 0) {
    if (d_fp) {
      close_wav(false);
    } else {
      BOOST_LOG_TRIVIAL(error) << "Ending transmission, sample_count is greater than 0 but d_fp is null" << std::endl;
    }
    // if an Transmission has ended, send it to Call.
    Transmission transmission;
    transmission.source = curr_src_id;      // Source ID for the Call
    transmission.start_time = d_start_time; // Start time of the Call
    transmission.stop_time = d_stop_time;   // when the Call eneded
    transmission.sample_count = d_sample_count;
    transmission.spike_count = d_spike_count;
    transmission.error_count = d_error_count;
    transmission.slot = d_slot;
    transmission.color_code = d_current_color_code;
    transmission.length = length_in_seconds(); // length in seconds
    d_prior_transmission_length = d_prior_transmission_length + transmission.length;
    strcpy(transmission.filename, current_filename); // Copy the filename
    transmission.talkgroup = d_current_call_talkgroup;

    BOOST_LOG_TRIVIAL(debug) << "Adding transmission: " << transmission.filename << " Slot: " << transmission.slot << " Talkgroup: " << transmission.talkgroup << " Length: " << transmission.length << " Samples: " << d_sample_count;
    this->add_transmission(transmission);

    // Reset the recorder to be ready to record the next Transmission
    state = IDLE;
    d_sample_count = 0;
    d_error_count = 0;
    d_spike_count = 0;
    curr_src_id = -1;
    d_current_color_code = -1;

 
  } else {
    BOOST_LOG_TRIVIAL(error) << "Trying to end a Transmission, but the sample_count is 0" << std::endl;
  }
}

void transmission_sink::stop_recording() {
  gr::thread::scoped_lock guard(d_mutex);

  if (state == RECORDING) {
    BOOST_LOG_TRIVIAL(trace) << "stop_recording() - stopping wavfile sink but recorder state is: " << state << " Sample Count is: " << d_sample_count << std::endl;
  }

  if (d_sample_count > 0) {
    end_transmission();
  }

  d_current_call = NULL;
  d_termination_flag = false;
  state = AVAILABLE;
}

void transmission_sink::close_wav(bool close_call) {
  unsigned int byte_count = d_sample_count * d_bytes_per_sample;
  wavheader_complete(d_fp, byte_count);
  fclose(d_fp);
  d_fp = NULL;
}

transmission_sink::~transmission_sink() {
  stop_recording();
}

bool transmission_sink::stop() {
  return true;
}

State transmission_sink::get_state() {
  return this->state;
}

int transmission_sink::work(int noutput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) {

  gr::thread::scoped_lock guard(d_mutex); // hold mutex for duration of this function
  std::string loghdr = log_header(d_current_call_short_name,d_current_call_num,d_current_call_talkgroup_display,d_current_call_freq);
  
  // it is possible that we could get part of a transmission after a call has stopped. We shouldn't do any recording if this happens.... this could mean that we miss part of the recording though
  if (!d_current_call) {
    time_t now = time(NULL);
    double its_been = difftime(now, d_stop_time);

    // It is possible the P25 Frame Assembler passes a TDU after the call has timed out.
    // In this case, the termination tag will be transferred on a blank sample and can safely be ignored.
    if (noutput_items == 1) {
      BOOST_LOG_TRIVIAL(trace) << loghdr << "Dropping " << noutput_items << " samples - current_call is null\t Rec State: " << format_state(this->state) << "\tSince close: " << its_been;
    } else {
      BOOST_LOG_TRIVIAL(error) << loghdr << "Dropping " << noutput_items << " samples - current_call is null\t Rec State: " << format_state(this->state) << "\tSince close: " << its_been;
    }

    return noutput_items;
  }

  // it is possible that we could get part of a transmission after a call has stopped. We shouldn't do any recording if this happens.... this could mean that we miss part of the recording though
  if ((state == STOPPED) || (state == AVAILABLE)) {
    if (noutput_items > 1) {

      BOOST_LOG_TRIVIAL(error) << loghdr << "Dropping " << noutput_items << " samples - Recorder state is: " << format_state(this->state);

      // BOOST_LOG_TRIVIAL(info) << "WAV - state is: " << format_state(this->state) << "\t Dropping samples: " << noutput_items << " Since close: " << its_been << std::endl;
    }
    return noutput_items;
  }

  std::vector<gr::tag_t> tags;
  pmt::pmt_t src_id_key(pmt::intern("src_id")); // This is the src id from Phase 1, Phase 2 and DMR
  pmt::pmt_t grp_id_key(pmt::intern("grp_id")); // This is the talkgroup id from Phase 1, Phase 2 and DMR
  pmt::pmt_t cc_key(pmt::intern("cc"));         // This is the channel color code from DMR
  pmt::pmt_t terminate_key(pmt::intern("terminate"));
  pmt::pmt_t spike_count_key(pmt::intern("spike_count"));
  pmt::pmt_t error_count_key(pmt::intern("error_count"));

  // pmt::pmt_t squelch_key(pmt::intern("squelch_eob"));
  // get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + noutput_items);
  get_tags_in_window(tags, 0, 0, noutput_items);
  unsigned pos = 0;
  // long curr_src_id = 0;

  for (unsigned int i = 0; i < tags.size(); i++) {
    // BOOST_LOG_TRIVIAL(info) << "TAG! " << tags[i].key;
    if (pmt::eq(grp_id_key, tags[i].key)) {
      long grp_id = pmt::to_long(tags[i].value);

      if ((state == RECORDING) || (state == IDLE)) {
        if (d_current_call_talkgroup_encoded != grp_id) {
          if (!d_conventional) {
            BOOST_LOG_TRIVIAL(info) << loghdr << "GROUP MISMATCH -  Recorder TG: " << d_current_call_talkgroup_encoded << " Received TG: " << grp_id << " Recorder state: " << format_state(state) << " incoming: " << noutput_items;
            if (d_sample_count > 0) {
              BOOST_LOG_TRIVIAL(info) << loghdr << "Ending Transmission and IGNORING Rest - count: " << d_sample_count;
              end_transmission();
            }
            state = IGNORE;
          } else {
            if (d_current_call_talkgroup != grp_id) {
              if (d_current_call_talkgroup != 0) {
                BOOST_LOG_TRIVIAL(debug) << loghdr << "Conventional Call - TALKGROUP MISMATCH - Talkgroup already set - Recorder TG: " << d_current_call_talkgroup << " Received TG: " << grp_id << " Recorder state: " << format_state(state) << " incoming: " << noutput_items;
                // this is where we would conclude the current call and start a new one.
              }
              BOOST_LOG_TRIVIAL(debug) << loghdr << "Conventional Call - TALKGROUP set via Control Channel - Recorder TG: " << d_current_call_talkgroup << " Received TG: " << grp_id << " Recorder state: " << format_state(state) << " incoming: " << noutput_items;
              // Retain the OTA talkgroup for conventional systems, only apply it for DMR
              d_current_call_talkgroup_encoded = grp_id;
              if (d_current_call->get_system_type() == "conventionalDMR") {
                d_current_call_talkgroup = grp_id;
                d_current_call_talkgroup_display = std::to_string(grp_id);
              }
            }
          }
        }
      }
    }
    if (pmt::eq(cc_key, tags[i].key)) {
      long cc = pmt::to_long(tags[i].value);

      if ((state == RECORDING) || (state == IDLE)) {
        if (cc != d_current_color_code) {
          if (d_current_call->get_system_type() == "conventionalDMR") {
            d_current_color_code = cc;
            BOOST_LOG_TRIVIAL(info) << loghdr << "DMR Color Code set to: " << d_current_color_code << " Recorder state: " << format_state(state);
          } 
        }
      }
    }
    if (pmt::eq(src_id_key, tags[i].key)) {
      long src_id = pmt::to_long(tags[i].value);
      pos = d_sample_count + (tags[i].offset - nitems_read(0));

      if (curr_src_id == -1) {
        // BOOST_LOG_TRIVIAL(info) << "Updated Voice Channel source id: " << src_id << " pos: " << pos << " offset: " << tags[i].offset - nitems_read(0);

        curr_src_id = src_id;
      } else if (src_id != curr_src_id) {
        if (state == RECORDING) {

          // BOOST_LOG_TRIVIAL(info) << "ENDING TRANSMISSION from TAGS Voice Channel mismatch source id - current: "<< curr_src_id << " new: " << src_id << " pos: " << pos << " offset: " << tags[i].offset - nitems_read(0);
          /*
              if (d_conventional && (d_sample_count > 0)) {
                  end_transmission();
                  state = IDLE;
              }
            state = STOPPED;
            if (!record_more_transmissions) {
              state = STOPPED;
            } else {
              state = IDLE;
            }*/

          curr_src_id = src_id;
        }
        // BOOST_LOG_TRIVIAL(info) << "Updated Voice Channel source id: " << src_id << " pos: " << pos << " offset: " << tags[i].offset - nitems_read(0);
      }
    }

    if (pmt::eq(terminate_key, tags[i].key)) {
      d_termination_flag = true;
      pos = d_sample_count + (tags[i].offset - nitems_read(0));

      // BOOST_LOG_TRIVIAL(info) << "[" << d_current_call_short_name << "]\t\033[0;34m" << d_current_call_num << "C\033[0m\tTG: " << d_current_call_talkgroup_display << "\tFreq: " << format_freq(d_current_call_freq) << "\tTermination - rec sample count " << d_sample_count << " pos: " << pos << " offset: " << tags[i].offset;

      // BOOST_LOG_TRIVIAL(info) << "TERMINATOR!!";
    }

    // Only process Spike and Error Count tags if the sink is currently recording
    if (state == RECORDING) {
      if (pmt::eq(spike_count_key, tags[i].key)) {
        d_spike_count = pmt::to_long(tags[i].value);

        BOOST_LOG_TRIVIAL(trace) << loghdr << "Spike Count: " << d_spike_count << " pos: " << pos << " offset: " << tags[i].offset;
      }
      if (pmt::eq(error_count_key, tags[i].key)) {
        d_error_count = pmt::to_long(tags[i].value);

        BOOST_LOG_TRIVIAL(trace) << loghdr << "Error Count: " << d_error_count << " pos: " << pos << " offset: " << tags[i].offset;
      }
    }
  }
  tags.clear();

  // if the System for this call is in Transmission Mode, and we have a recording and we got a flag that a Transmission ended...
  int nwritten = dowork(noutput_items, input_items, output_items);

  return nwritten;
}

time_t transmission_sink::get_start_time() {
  return d_start_time;
}

time_t transmission_sink::get_stop_time() {
  return d_stop_time;
}

std::chrono::time_point<std::chrono::steady_clock> transmission_sink::get_last_write_time() {
  return d_last_write_time;
}

void transmission_sink::add_transmission(Transmission t) {
  transmission_list.push_back(t);
}

void transmission_sink::clear_transmission_list() {
  transmission_list.clear();
  transmission_list.shrink_to_fit();
}

std::vector<Transmission> transmission_sink::get_transmission_list() {
  return transmission_list;
}

int transmission_sink::dowork(int noutput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) {
  // block
  int n_in_chans = input_items.size();
  int16_t sample_buf_s;
  int nwritten = 0;
  bool terminate_after_write = false;
  std::string loghdr = log_header(d_current_call_short_name,d_current_call_num,d_current_call_talkgroup_display,d_current_call_freq);

  if (state == STOPPED) {
    return noutput_items;
  }

  // A Termination Tag was receive
  if (d_termination_flag) {
    d_termination_flag = false;

    if (d_current_call == NULL) {
      BOOST_LOG_TRIVIAL(error) << "wav - no current call, but in termination loop";
      state = STOPPED;

      return noutput_items;
    }

    if (state == IGNORE) {
      BOOST_LOG_TRIVIAL(trace) << loghdr << "Resetting state from IGNORE to IDLE: " << noutput_items;
      state = IDLE;

      return noutput_items;
    }

    // The TDU can come in with voice samples. Write the voice samples and then end the transmission.
    if (d_sample_count > 0 && noutput_items > 1) {
      BOOST_LOG_TRIVIAL(trace) << loghdr << "Terminator received with items. Ending transmission after writing. Sample Count: " << d_sample_count << " Noutput Items: " << noutput_items;
      terminate_after_write = true;
      // Handle the case of a terminator coming in without voice samples. End the transmission immediately.
    } else if (d_sample_count > 0) {
      BOOST_LOG_TRIVIAL(trace) << loghdr << "Terminator received without items. Ending transmission immediately. " << d_sample_count << " Noutput Items: " << noutput_items;
      end_transmission();
      return noutput_items;
    } else {
      BOOST_LOG_TRIVIAL(trace) << loghdr << "TERM - skipped....   - count: " << d_sample_count;
      return noutput_items;
    }
  }

  if (state == IGNORE) {
    BOOST_LOG_TRIVIAL(trace) << loghdr << "IGNORE missing count: " << noutput_items;
    return noutput_items;
  }

  if (state == IDLE) {
    // BOOST_LOG_TRIVIAL(info) << loghdr << "IDLE but haven't seen Group ID yet, missing count: " << noutput_items;
    // return noutput_items;
    if (d_fp) {
      // if we are already recording a file for this call, close it before starting a new one.
      BOOST_LOG_TRIVIAL(info) << "WAV - Weird! we have an existing FP, but STATE was IDLE:  " << current_filename << std::endl;

      close_wav(false);
    }

    time_t current_time = time(NULL);
    if (current_time == d_start_time) {
      d_start_time = current_time + 1;
    } else {
      d_start_time = current_time;
    }

    // create a new filename, based on the current time and source.
    create_filename();
    if (!open_internal(current_filename)) {
      BOOST_LOG_TRIVIAL(error) << "can't open file";
      return noutput_items;
    }

    BOOST_LOG_TRIVIAL(trace) << loghdr << "Starting new Transmission \tSrc ID:  " << curr_src_id;

    // curr_src_id = d_current_call->get_current_source_id();
    state = RECORDING;
  }

  if (!d_fp) // drop output on the floor
  {
    BOOST_LOG_TRIVIAL(error) << "Wav - Dropping items, no fp or Current Call: " << noutput_items << " Filename: " << current_filename << " Current sample count: " << d_sample_count << std::endl;
    return noutput_items;
  }

  if (state == RECORDING) {
    for (nwritten = 0; nwritten < noutput_items; nwritten++) {
      for (int chan = 0; chan < d_nchans; chan++) {
        // Write zeros to channels which are in the WAV file
        // but don't have any inputs here
        if (chan < n_in_chans) {
          int16_t **in = (int16_t **)&input_items[0];
          sample_buf_s = in[chan][nwritten];
        } else {
          sample_buf_s = 0;
        }

        wav_write_sample(d_fp, sample_buf_s, d_bytes_per_sample);

        d_sample_count++;
      }
    }

    if (terminate_after_write) {
      end_transmission();
    }
  }

  d_stop_time = time(NULL);
  d_last_write_time = std::chrono::steady_clock::now();

  if (nwritten < noutput_items) {
    BOOST_LOG_TRIVIAL(error) << loghdr << "Failed to Write! Wrote: " << nwritten << " of " << noutput_items;
  } else {
    BOOST_LOG_TRIVIAL(trace) << loghdr << "Wrote: " << nwritten << " of " << noutput_items;
  }
  return noutput_items;
}

void transmission_sink::set_bits_per_sample(int bits_per_sample) {
  gr::thread::scoped_lock guard(d_mutex);

  if ((bits_per_sample == 8) || (bits_per_sample == 16)) {
    d_bytes_per_sample = bits_per_sample / 8;
  }
}

void transmission_sink::set_sample_rate(unsigned int sample_rate) {
  gr::thread::scoped_lock guard(d_mutex);

  d_sample_rate = sample_rate;
}

int transmission_sink::bits_per_sample() {
  return d_bytes_per_sample * 8;
}

unsigned int
transmission_sink::sample_rate() {
  return d_sample_rate;
}

double transmission_sink::total_length_in_seconds() {
  return this->length_in_seconds() + d_prior_transmission_length;
}

double transmission_sink::length_in_seconds() {
  return (double)d_sample_count / (double)d_sample_rate;
}

void transmission_sink::do_update() {}
} /* namespace blocks */
} /* namespace gr */
