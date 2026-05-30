/* -*- c++ -*- */
/* 
 * GNU Radio interface for Pavel Yazev's Project 25 IMBE Encoder/Decoder
 * 
 * Copyright 2009 Pavel Yazev E-mail: pyazev@gmail.com
 * Copyright 2009, 2010, 2011, 2012, 2013, 2014 KA1RBI
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
#include "vocoder_impl.h"

#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace gr {
  namespace op25_repeater {

    static const int FRAGMENT_SIZE = 864;

    vocoder::sptr
    vocoder::make(bool encode_flag, bool verbose_flag, int stretch_amt, char* udp_host, int udp_port, bool raw_vectors_flag)
    {
      return gnuradio::get_initial_sptr
        (new vocoder_impl(encode_flag, verbose_flag, stretch_amt, udp_host, udp_port, raw_vectors_flag));
    }

//////////////////////////////////////////////////////////////
//	enc/dec		udp	operation
//	dec		no	byte input; short output
//	dec		yes	byte input; null output
//	enc		no	short input; char output
//	enc		yes	short input; null output

#define M_IN(encode_flag, udp_port)  (1)
#define M_OUT(encode_flag, udp_port) ((udp_port) ? 0 : 1)
#define S_IN(encode_flag, udp_port)  ((encode_flag) ? sizeof(int16_t) : sizeof(uint8_t))
#define S_OUT(encode_flag, udp_port) ((udp_port) ? 0 : ((encode_flag) ? sizeof(uint8_t) : sizeof(int16_t)))

    /*
     * The private constructor
     */
    vocoder_impl::vocoder_impl(bool encode_flag, bool verbose_flag, int stretch_amt, char* udp_host, int udp_port, bool raw_vectors_flag)
      : gr::block("vocoder",
              gr::io_signature::make (M_IN(encode_flag, udp_port), M_IN(encode_flag, udp_port), S_IN(encode_flag, udp_port)),
              gr::io_signature::make (M_OUT(encode_flag, udp_port), M_OUT(encode_flag, udp_port), S_OUT(encode_flag, udp_port))),
    output_queue(),
    output_queue_decode(),
    opt_udp_port(udp_port),
    opt_encode_flag(encode_flag),
    op25audio(udp_host, udp_port, 0),
    p1voice_encode(verbose_flag, stretch_amt, op25audio, raw_vectors_flag, output_queue),
    p1voice_decode(verbose_flag, op25audio, output_queue_decode)
    {
	if (opt_encode_flag)
		set_output_multiple(FRAGMENT_SIZE);
    }

    /*
     * Our virtual destructor.
     */
    vocoder_impl::~vocoder_impl()
    {
    }

void
vocoder_impl::forecast(int nof_output_items, gr_vector_int &nof_input_items_reqd)
{
   /* When encoding, this block consumes 8000 symbols/s and produces 4800
    * samples/s. That's a sampling rate of 5/3 or 1.66667.
    *
    * When decoding, the block consumes one line of text per voice codeword.
    * Each line of text is exactly 32 bytes.  It outputs 160 samples for each 
    * codeword; the ratio is thus 32/160 = 0.2.
    *
    * Thanks to Matt Mills for catching a bug where this value wasn't set correctly
    */
   const int nof_samples_reqd = (opt_encode_flag) ? (1.66667 * nof_output_items) : (0.2 * nof_output_items);
   std::fill(nof_input_items_reqd.begin(), nof_input_items_reqd.end(), nof_samples_reqd);
}

int 
vocoder_impl::general_work_decode (int noutput_items,
			       gr_vector_int &ninput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
  const char *in = (const char *) input_items[0];

  p1voice_decode.rxchar(in, ninput_items[0]);

  // Tell runtime system how many input items we consumed on
  // each input stream.

  consume_each (ninput_items[0]);

  int16_t *out = reinterpret_cast<int16_t*>(output_items[0]);
  const int n = std::min(static_cast<int>(output_queue_decode.size()), noutput_items);
  if(0 < n) {
     copy(output_queue_decode.begin(), output_queue_decode.begin() + n, out);
     output_queue_decode.erase(output_queue_decode.begin(), output_queue_decode.begin() + n);
  }
  // Tell runtime system how many output items we produced.
  return n;
}

int 
vocoder_impl::general_work_encode (int noutput_items,
			       gr_vector_int &ninput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
  const short *in = (const short *) input_items[0];
  const int noutput_fragments = noutput_items / FRAGMENT_SIZE;
  const int fragments_available = output_queue.size() / FRAGMENT_SIZE;
  const int nsamples_consume = std::min(ninput_items[0], std::max(0,(noutput_fragments - fragments_available) * 9 * 160));

  if (nsamples_consume > 0) {
    p1voice_encode.compress_samp(in, nsamples_consume);

    // Tell runtime system how many input items we consumed on
    // each input stream.

    consume_each (nsamples_consume);
  }

  if (op25audio.enabled())		// in udp option, we are a gr sink only
    return 0;

  uint8_t *out = reinterpret_cast<uint8_t*>(output_items[0]);
  const int n = std::min(static_cast<int>(output_queue.size()), noutput_items);
  if(0 < n) {
     copy(output_queue.begin(), output_queue.begin() + n, out);
     output_queue.erase(output_queue.begin(), output_queue.begin() + n);
  }
  // Tell runtime system how many output items we produced.
  return n;
}

int 
vocoder_impl::general_work (int noutput_items,
			       gr_vector_int &ninput_items,
			       gr_vector_const_void_star &input_items,
			       gr_vector_void_star &output_items)
{
	if (opt_encode_flag)
		return general_work_encode(noutput_items, ninput_items, input_items, output_items);
	else
		return general_work_decode(noutput_items, ninput_items, input_items, output_items);
}

void
vocoder_impl::set_gain_adjust(float gain_adjust) {
	p1voice_encode.set_gain_adjust(gain_adjust);
}

  } /* namespace op25_repeater */
} /* namespace gr */
