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
#include "p25p1_voice_decode.h"

#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "imbe_vocoder/imbe_vocoder.h"
#include "p25_frame.h"
#include "op25_imbe_frame.h"

namespace gr {
  namespace op25_repeater {

static void clear_bits(bit_vector& v) {
	for (size_t i=0; i<v.size(); i++) {
		v[i]=0;
	}
}

p25p1_voice_decode::p25p1_voice_decode(bool verbose_flag, const op25_audio& udp, std::deque<int16_t> &_output_queue) :
	write_bufp(0),
	rxbufp(0),
	d_er(0.0f),
	d_rpt_ctr(0),
	op25audio(udp),
	output_queue(_output_queue),
	opt_verbose(verbose_flag)
    {
	memset(d_last_snd, 0, sizeof(d_last_snd));
	const char *p = getenv("IMBE");
	if (p && strcasecmp(p, "soft") == 0)
		d_software_imbe_decoder = true;
	else
		d_software_imbe_decoder = false;
    }

    /*
     * Our virtual destructor.
     */
    p25p1_voice_decode::~p25p1_voice_decode()
    {
    }

void p25p1_voice_decode::clear() {
  vocoder.clear();
  d_er = 0.0f;
  d_rpt_ctr = 0;
  memset(d_last_snd, 0, sizeof(d_last_snd));
}
// more-optimized version of rxframe() used by p25p1_fdma
void p25p1_voice_decode::rxframe(const voice_codeword& cw)
{
	int16_t snd[FRAME];
	uint32_t u[8], E0, ET;
	imbe_header_decode(cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], E0, ET);

	// TIA-102.BABA-A §7.7-7.8 muting and frame-repeat policy, applied uniformly
	// to both the fixed-point and float decoders. The float decoder repeats the
	// same logic internally; gating here keeps the fixed-point path honest.
	d_er = (0.95f * d_er) + (0.000365f * (float)ET);
	int b0 = ((u[0] >> 4) & 0xfc) | ((u[7] >> 1) & 0x3);
	bool muted = false;
	bool repeated = false;
	if (d_er > 0.0875f) {
		muted = true;
	} else if (b0 > 207 || E0 >= 2 || ET >= (int)(10.0f + 40.0f * d_er)) {
		if (++d_rpt_ctr >= 4) {
			muted = true;
		} else {
			repeated = true;
		}
	} else {
		d_rpt_ctr = 0;
	}

	if (muted) {
		memset(snd, 0, sizeof(snd));
	} else if (repeated) {
		memcpy(snd, d_last_snd, sizeof(snd));
	} else if (d_software_imbe_decoder) {
		software_decoder.decode(snd, cw);
		memcpy(d_last_snd, snd, sizeof(snd));
	} else {
		int16_t frame_vector[8];
		for (int i = 0; i < 8; i++) {
			frame_vector[i] = u[i];
		}
		frame_vector[7] >>= 1;
		vocoder.imbe_decode(frame_vector, snd);
		memcpy(d_last_snd, snd, sizeof(snd));
	}

	if (op25audio.enabled()) {
		op25audio.send_audio(snd, FRAME * sizeof(int16_t));
	} else {
		// add generated samples to output queue
		for (int i = 0; i < FRAME; i++) {
			output_queue.push_back(snd[i]);
		}
	}
}

// this version of rxframe() not normally used except by rxchar()
void p25p1_voice_decode::rxframe(const uint32_t u[])
{
	int16_t snd[FRAME];
	int16_t frame_vector[8];
	// decode 88 bits, outputs 160 sound samples (8000 rate)
	if (d_software_imbe_decoder) {
		voice_codeword cw(voice_codeword_sz);
		imbe_header_encode(cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
		software_decoder.decode(snd, cw);
	} else {
		for (int i=0; i < 8; i++) {
			frame_vector[i] = u[i];
		}
/* TEST*/	frame_vector[7] >>= 1;
		vocoder.imbe_decode(frame_vector, snd);					// Does anyone still use this version of the codec?
	}
	if (op25audio.enabled()) {
		op25audio.send_audio(snd, FRAME * sizeof(int16_t));
	} else {
		// add generated samples to output queue
		for (int i = 0; i < FRAME; i++) {
			output_queue.push_back(snd[i]);
		}
	}
}

void p25p1_voice_decode::rxchar(const char* c, int len)
{
	uint32_t u[8];

	for (int i = 0; i < len; i++ ) {
		if (c[i] < ' ') {
			if (c[i] == '\n') {
				rxbuf[rxbufp] = 0;
				sscanf(rxbuf, "%x %x %x %x %x %x %x %x", &u[0], &u[1], &u[2], &u[3], &u[4], &u[5], &u[6], &u[7]);
				rxbufp = 0;
				rxframe(u);
			}
			continue;
		}
		rxbuf[rxbufp++] = c[i];
		if (rxbufp >= RXBUF_MAX) {
			rxbufp = RXBUF_MAX - 1;
		}
	} /* end of for() */
}

  } /* namespace op25_repeater */
} /* namespace gr */
