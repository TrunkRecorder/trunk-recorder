/* -*- c++ -*- */
/* 
 * Copyright 2010, 2011, 2012, 2013, 2014 Max H. Parke KA1RBI 
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "p25_frame_assembler_impl.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <sys/time.h>

namespace gr {
  namespace op25_repeater {

    /* This is for the TPS Analog decoder */
    void p25_frame_assembler_impl::p25p2_queue_msg(int duid) {
      static const unsigned char wbuf[2] = {0xff, 0xff}; // dummy NAC
      if (!d_do_msgq)
        return;
      if (d_msg_queue->full_p())
        return;
      gr::message::sptr msg = gr::message::make_from_string(std::string((const char *)wbuf, 2), duid, 0);
      d_msg_queue->insert_tail(msg);
    }

    void p25_frame_assembler_impl::set_xormask(const char*p) {
		p2tdma.set_xormask(p);
    }

    void p25_frame_assembler_impl::set_slotid(int slotid) {
		p2tdma.set_slotid(slotid);
    }

    void p25_frame_assembler_impl::set_slotkey(int key) {
    }

    void p25_frame_assembler_impl::set_nac(int nac) {
		p1fdma.set_nac(nac);
        p2tdma.set_nac(nac);
    }

    void p25_frame_assembler_impl::reset_timer() {
		p1fdma.reset_timer();
    }

    void p25_frame_assembler_impl::set_debug(int debug) {
		op25audio.set_debug(debug);
		p1fdma.set_debug(debug);
		p2tdma.set_debug(debug);
    }

    p25_frame_assembler::sptr
    p25_frame_assembler::make(int silence_frames, bool soft_vocoder, const char *udp_host, int port, int debug, bool do_imbe, bool do_output, bool do_msgq, gr::msg_queue::sptr queue, bool do_audio_output, bool do_phase2_tdma, bool do_nocrypt) {
      return gnuradio::get_initial_sptr(new p25_frame_assembler_impl(silence_frames, soft_vocoder, udp_host, port, debug, do_imbe, do_output, do_msgq, queue, do_audio_output, do_phase2_tdma, do_nocrypt));
    }

    /*
     * The private constructor
     */

    /*
     * Our virtual destructor.
     */
    p25_frame_assembler_impl::~p25_frame_assembler_impl()
    {
    }

static const int MIN_IN = 1;	// mininum number of input streams
static const int MAX_IN = 1;	// maximum number of input streams

/*
 * The private constructor
 */
  p25_frame_assembler_impl::p25_frame_assembler_impl(int silence_frames,  bool soft_vocoder, const char *udp_host, int port, int debug, bool do_imbe, bool do_output, bool do_msgq, gr::msg_queue::sptr queue, bool do_audio_output, bool do_phase2_tdma, bool do_nocrypt)
      : gr::block("p25_frame_assembler",
		   gr::io_signature::make (MIN_IN, MAX_IN, sizeof (char)),
		   gr::io_signature::make ((do_output) ? 1 : 0, (do_output) ? 1 : 0, (do_audio_output && do_output) ? sizeof(int16_t) : ((do_output) ? sizeof(char) : 0 ))),
	d_do_imbe(do_imbe),
	d_do_output(do_output),
  p1fdma(op25audio, logts, debug, do_imbe, do_output, do_msgq, queue, output_queue, do_audio_output, soft_vocoder),
	d_do_audio_output(do_audio_output),
	d_do_phase2_tdma(do_phase2_tdma),
	p2tdma(op25audio, logts,  0, debug, do_msgq, queue, output_queue, do_audio_output, soft_vocoder),
	d_do_msgq(do_msgq),
	d_msg_queue(queue),
	output_queue(),
	op25audio(udp_host, port, debug),
  d_input_rate(4800),
  d_tag_src(pmt::intern(name())), 
  d_silence_frames(silence_frames)
{
      if (d_do_audio_output && !d_do_output)
        fprintf(stderr, "p25_frame_assembler: error: do_output must be enabled if do_audio_output is enabled\n");

      if (d_do_audio_output && !d_do_imbe)
        fprintf(stderr, "p25_frame_assembler: error: do_imbe must be enabled if do_audio_output is enabled\n");

      if (d_do_phase2_tdma && !d_do_audio_output)
        fprintf(stderr, "p25_frame_assembler: error: do_audio_output must be enabled if do_phase2_tdma is enabled\n");

      if (d_do_audio_output)
        set_output_multiple(864);

      if (!d_do_audio_output && !d_do_imbe)
        set_output_multiple(160);
    }


    void p25_frame_assembler_impl::clear() {
      // clear out the SRC and GRP IDs
      long clear;
      clear = p1fdma.get_curr_src_id();
      clear = p1fdma.get_curr_grp_id();
      clear = p2tdma.get_ptt_src_id(); 
      clear = p2tdma.get_ptt_grp_id(); 
      // p1fdma.clear(); //All this does is Clear the Vocoder - I am nervous about doing this because there are some memsets...
    }

void p25_frame_assembler_impl::send_grp_src_id() {
          long tdma_src_id = -1;
          long tdma_grp_id = -1;
          long fdma_src_id = -1;
          long fdma_grp_id = -1;

          tdma_src_id = p2tdma.get_ptt_src_id(); 
          tdma_grp_id = p2tdma.get_ptt_grp_id(); 
          fdma_src_id = p1fdma.get_curr_src_id();
          fdma_grp_id = p1fdma.get_curr_grp_id();

          // If a SRC wasn't received on the voice channel since the last check, it will be -1
          if (fdma_src_id > 0) {
            add_item_tag(0, nitems_written(0), pmt::intern("src_id"), pmt::from_long(fdma_src_id), d_tag_src);
          }

          if (tdma_src_id > 0) {
            add_item_tag(0, nitems_written(0), pmt::intern("src_id"), pmt::from_long(tdma_src_id), d_tag_src);
          }

          if ((tdma_src_id > 0) && (fdma_src_id > 0)) {
            BOOST_LOG_TRIVIAL(info) << " Both TDMA and FDMA SRC IDs are set. TDMA: " << tdma_src_id << " FDMA: " << fdma_src_id;
          }

          if (fdma_grp_id > 0) {
            add_item_tag(0, nitems_written(0), pmt::intern("grp_id"), pmt::from_long(fdma_grp_id), d_tag_src);
          }

          if (tdma_grp_id > 0) {
            add_item_tag(0, nitems_written(0), pmt::intern("grp_id"), pmt::from_long(tdma_grp_id), d_tag_src);
          }

          if ((tdma_grp_id > 0) && (fdma_grp_id > 0)) {
            BOOST_LOG_TRIVIAL(info) << " Both TDMA and FDMA GRP IDs are set. TDMA: " << tdma_grp_id << " FDMA: " << fdma_grp_id;
          }
}


int 
p25_frame_assembler_impl::general_work (int noutput_items,
                               gr_vector_int &ninput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
{

  const uint8_t *in = (const uint8_t *) input_items[0];
  std::pair<bool,long> terminate_call = std::make_pair(false, 0);

  p1fdma.rx_sym(in, ninput_items[0]);
  if(d_do_phase2_tdma) {
    for (int i = 0; i < ninput_items[0]; i++) {
      if(p2tdma.rx_sym(in[i])) {
        int rc = p2tdma.handle_frame();
        terminate_call = p2tdma.get_call_terminated();
        if (terminate_call.first) {
          p2tdma.reset_call_terminated();
        }
        
        if (rc > -1) {
          p25p2_queue_msg(rc);
          p1fdma.reset_timer(); // prevent P1 timeouts due to long TDMA transmissions
        }
      }
    }
  } else {
    terminate_call = p1fdma.get_call_terminated();
    if (terminate_call.first) {
      p1fdma.reset_call_terminated();
    }
  }

  int amt_produce = 0;

      // If this block is being used for Trunking, then you want to skip all of this.
      if (d_do_audio_output) {
        amt_produce = output_queue.size();
        int16_t *out = (int16_t *)output_items[0];

        //BOOST_LOG_TRIVIAL(trace) << "P25 Frame Assembler -  output_queue: " << output_queue.size() << " noutput_items: " <<  noutput_items << " ninput_items: " << ninput_items[0];

        if (amt_produce > 0) {
            if (amt_produce >= 32768) {
              BOOST_LOG_TRIVIAL(error) << "P25 Frame Assembler -  output_queue size: " << output_queue.size() << " max size: " << output_queue.max_size() << " limiting amt_produce to  32767 ";
              
              amt_produce = 32767; // buffer limit is 32768, see gnuradio/gnuradio-runtime/lib/../include/gnuradio/buffer.h:186
            }

            for (int i = 0; i < amt_produce; i++) {
              out[i] = output_queue[i];
            }
            output_queue.erase(output_queue.begin(), output_queue.begin() + amt_produce);

            send_grp_src_id();

            BOOST_LOG_TRIVIAL(trace) << "setting silence_frame_count " << silence_frame_count << " to d_silence_frames: " << d_silence_frames << std::endl;
            silence_frame_count = d_silence_frames;
        } else {
          if (silence_frame_count > 0) {
            std::fill(out, out + noutput_items, 0);
            amt_produce = noutput_items;
            silence_frame_count--;
          }
        }
        
        if (amt_produce == 0 && terminate_call.first) {
          std::fill(out, out + 1, 0);
          amt_produce = 1;
        }

        if (terminate_call.first) {
            BOOST_LOG_TRIVIAL(trace) << "P25 Frame Assembler: Applying TDU." << " Amount: " << amt_produce << " nitems_written(0): " << nitems_written(0);
            if ((amt_produce != 1) && (terminate_call.second < amt_produce)) {
              BOOST_LOG_TRIVIAL(info) << "P25 Frame Assembler: TDU was not at the end of the data. TDU: " << terminate_call.second << " Amount: " << amt_produce << " nitems_written(0): " << nitems_written(0);  
            }
            add_item_tag(0, nitems_written(0), pmt::intern("terminate"), pmt::from_long(1), d_tag_src );
            
            Rx_Status status = p1fdma.get_rx_status();
            
            // If something was recorded, send the number of Errors and Spikes that were counted during that period
            if (status.total_len > 0 ) {
              add_item_tag(0, nitems_written(0), pmt::intern("spike_count"), pmt::from_long(status.spike_count), d_tag_src);
              add_item_tag(0, nitems_written(0), pmt::intern("error_count"), pmt::from_long(status.error_count), d_tag_src);
              p1fdma.reset_rx_status();
            }
            if (silence_frame_count > 0) {
              std::fill(out, out + noutput_items, 0);
              amt_produce = noutput_items;
              silence_frame_count--;
            }
          }
        
        
      }
  consume_each(ninput_items[0]);
  // Tell runtime system how many output items we actually produced.
  return amt_produce;
}

    void p25_frame_assembler_impl::clear_silence_frame_count() {
      silence_frame_count = 0;
    }

    void p25_frame_assembler_impl::set_phase2_tdma(bool p) {
      d_do_phase2_tdma = p;

      BOOST_LOG_TRIVIAL(debug) << "Setting TDMA to: " << p;
      if (d_do_audio_output) {
        if (d_do_phase2_tdma) {
          d_input_rate = 6000;
        } else {
          d_input_rate = 4800;
        }
      }
    }
  } /* namespace op25_repeater */
} /* namespace gr */
