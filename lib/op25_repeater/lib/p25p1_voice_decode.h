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

#ifndef INCLUDED_OP25_REPEATER_P25P1_VOICE_DECODE_H
#define INCLUDED_OP25_REPEATER_P25P1_VOICE_DECODE_H

#include <sys/time.h>
#include <stdint.h>
#include <vector>
#include <deque>

#include "op25_audio.h"
#include "imbe_vocoder/imbe_vocoder.h"

#include "imbe_decoder.h"
#include "software_imbe_decoder.h"

namespace gr {
  namespace op25_repeater {

    typedef std::vector<bool> bit_vector;
    class p25p1_voice_decode
    {
     private:
      // Nothing to declare in this block.

     public:
      p25p1_voice_decode(bool verbose_flag, const op25_audio& udp, std::deque<int16_t> &_output_queue);
      ~p25p1_voice_decode();
	void rxframe(const voice_codeword& cw);
	void rxframe(const uint32_t u[]);
	void rxchar(const char* c, int len);
  void clear();

  private:
	static const int RXBUF_MAX = 80;

	/* data items */
	int write_bufp;
	char write_buf[512];
	char rxbuf[RXBUF_MAX];
	int rxbufp ;
	imbe_vocoder vocoder;
	software_imbe_decoder software_decoder;
	bool d_software_imbe_decoder;
        const op25_audio& op25audio;

	std::deque<int16_t> &output_queue;

	bool opt_verbose;
	/* local methods */
    };

  } // namespace op25_repeater
} // namespace gr

#endif /* INCLUDED_OP25_REPEATER_P25P1_VOICE_DECODE_H */
