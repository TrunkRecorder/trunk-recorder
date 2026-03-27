/* -*- c++ -*- */
/* 
 * Copyright 2009, 2010, 2011, 2012, 2013, 2014 Max H. Parke KA1RBI
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

#ifndef INCLUDED_OP25_REPEATER_P25_FRAME_ASSEMBLER_IMPL_H
#define INCLUDED_OP25_REPEATER_P25_FRAME_ASSEMBLER_IMPL_H

#include "../include/op25_repeater/p25_frame_assembler.h"
#include "../include/op25_repeater/rx_status.h"
#include <gnuradio/msg_queue.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <deque>
#include <boost/log/trivial.hpp>

#include "p25p1_fdma.h"
#include "p25p2_tdma.h"
#include "op25_audio.h"
#include "log_ts.h"

typedef std::deque<uint8_t> dibit_queue;

namespace gr {
  namespace op25_repeater {

    class p25_frame_assembler_impl : public p25_frame_assembler
    {
     private:
	bool d_do_imbe;
	bool d_do_output;
	p25p1_fdma p1fdma;
	bool d_do_audio_output;
	bool d_do_phase2_tdma;
	bool d_do_nocrypt;
	p25p2_tdma p2tdma;
	bool d_do_msgq;
	gr::msg_queue::sptr d_msg_queue;

  int d_input_rate;
  int d_silence_frames;
  int silence_frame_count;
  long total_produced;
  pmt::pmt_t d_tag_key;
  pmt::pmt_t d_tag_src;
  // internal functions

    void send_grp_src_id();
    void set_xormask(const char*p) ;
    void set_nac(int nac) ;
    void set_slotid(int slotid) ;
    void set_slotkey(int key) ;
    void set_debug(int debug) ;
    void reset_timer() ;
	typedef std::vector<bool> bit_vector;
	std::deque<int16_t> output_queue;

  void p25p2_queue_msg(int duid);
  void set_phase2_tdma(bool p);

public:
      p25_frame_assembler_impl(int silence_frames, bool soft_vocoder, const char* udp_host, int port, int debug, bool do_imbe, bool do_output, bool do_msgq, gr::msg_queue::sptr queue, bool do_audio_output, bool do_phase2_tdma, bool do_nocrypt);
      ~p25_frame_assembler_impl();

      op25_audio op25audio;

      // Where all the action really happens

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);

      void clear_silence_frame_count();
      void clear();
      void crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key);
      void crypt_reset();
      log_ts logts;
    };

  } // namespace op25_repeater
} // namespace gr

#endif /* INCLUDED_OP25_REPEATER_P25_FRAME_ASSEMBLER_IMPL_H */