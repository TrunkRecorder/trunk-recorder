// P25 Decoder (C) Copyright 2013, 2014, 2015, 2016, 2017 Max H. Parke KA1RBI
//             (C) Copyright 2019, 2020, 2021, 2022 Graham J. Norbury
// 
// This file is part of OP25
// 
// OP25 is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3, or (at your option)
// any later version.
// 
// OP25 is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with OP25; see the file COPYING. If not, write to the Free
// Software Foundation, Inc., 51 Franklin Street, Boston, MA
// 02110-1301, USA.

#ifndef INCLUDED_RX_SYNC_H
#define INCLUDED_RX_SYNC_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <deque>
#include <assert.h>
#include <gnuradio/msg_queue.h>

#include "bit_utils.h"
#include "check_frame_sync.h"

#include "frame_sync_magics.h"
#include "p25p1_fdma.h"
#include "p25p2_tdma.h"
#include "p25p2_vf.h"
#include "mbelib.h"
#include "ambe.h"
#include "imbe_vocoder/imbe_vocoder.h"

#include "ysf_const.h"
#include "dmr_const.h"
#include "dmr_cai.h"
#include "p25_frame.h"
#include "op25_timer.h"
#include "op25_imbe_frame.h"
#include "software_imbe_decoder.h"
#include "op25_audio.h"
#include "log_ts.h"

#include "rx_base.h"

namespace gr{
    namespace op25_repeater{

enum rx_types {
	RX_TYPE_NONE=0,
	RX_TYPE_P25P1,
	RX_TYPE_P25P2,
	RX_TYPE_DMR,
	RX_TYPE_DSTAR,
	RX_TYPE_YSF,
	RX_N_TYPES
};   // also used as array index

static const struct _mode_data {
	const char * type;
	int sync_len;
	int sync_offset;
	int fragment_len;   // symbols
	int expiration;
} MODE_DATA[RX_N_TYPES] = {
	{"NONE",   0,0,0,0},
	{"P25P1",  48,0,57,1728},
	{"P25P2",  40,0,180,2160},
	{"DMR",    48,66,144,1728},
	{"DSTAR",  48,72,96,2016*2},
	{"YSF",    40,0,480,480*2}
};   // index order must match rx_types enum

static const int KNOWN_MAGICS = 13;
static const struct _sync_magic {
	int type;
	uint64_t magic;
} SYNC_MAGIC[KNOWN_MAGICS] = {
	{RX_TYPE_P25P1, P25_FRAME_SYNC_MAGIC},
	{RX_TYPE_P25P2, P25P2_FRAME_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_BS_VOICE_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_BS_DATA_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_MS_VOICE_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_MS_DATA_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_MS_RC_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_T1_VOICE_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_T1_DATA_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_T2_VOICE_SYNC_MAGIC},
	{RX_TYPE_DMR, DMR_T2_DATA_SYNC_MAGIC},
	{RX_TYPE_DSTAR, DSTAR_FRAME_SYNC_MAGIC},
	{RX_TYPE_YSF, YSF_FRAME_SYNC_MAGIC}
}; // maps sync patterns to protocols

enum codeword_types {
	CODEWORD_P25P1,
	CODEWORD_P25P2,
	CODEWORD_DMR,
	CODEWORD_DSTAR,
	CODEWORD_YSF_FULLRATE,
	CODEWORD_YSF_HALFRATE
};

class rx_sync : public rx_base {
public:
	void rx_sym(const uint8_t sym);
	void sync_reset(void);
	void reset_timer(void);
	void call_end(void);
	void crypt_reset(void);
	void crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key);
	void set_slot_mask(int mask);
	void set_slot_key(int mask);
	void set_xormask(const char* p);
	void set_nac(int nac);
	void set_debug(int debug);
	int get_src_id(int slot);
	std::pair<bool,long> get_terminated(int slot);
	rx_sync(const char * options, log_ts& logger, int debug, int msgq_id, gr::msg_queue::sptr queue, std::array<std::deque<int16_t>, 2> &output_queue, bool d_soft_vocoder);
	~rx_sync();

private:
	void sync_timeout(rx_types proto);
	void sync_established(rx_types proto);
	void cbuf_insert(const uint8_t c);
	void ysf_sync(const uint8_t dibitbuf[], bool& ysf_fullrate, bool& unmute);
	void codeword(const uint8_t* cw, const enum codeword_types codeword_type, int slot_id);
	void output(int16_t * samp_buf, const ssize_t slot_id);
	static const int CBUF_SIZE=864;
	static const int NSAMP_OUTPUT = 160;

	op25_timer sync_timer;
	unsigned int d_symbol_count;
	uint64_t d_sync_reg;
	uint64_t d_fs;
	uint8_t d_cbuf[CBUF_SIZE*2];
	unsigned int d_cbuf_idx;
	enum rx_types d_current_type;
	int d_fragment_len;
	int d_threshold;
	int d_rx_count;
	unsigned int d_expires;
	int d_shift_reg;
	int d_slot_mask;
	int d_slot_key;
	unsigned int d_unmute_until[2];
	p25p1_fdma p25fdma;
	p25p2_tdma p25tdma;
	p25p2_vf interleaver;
	mbe_parms cur_mp[2];
	mbe_parms prev_mp[2];
	mbe_parms enh_mp[2];
	mbe_errs errs_mp[2];
	mbe_tone tone_mp[2];
	int mbe_err_cnt[2];
	bool d_soft_vocoder;
	software_imbe_decoder d_software_decoder[2];
	imbe_vocoder d_imbe_vocoder[2];
	std::deque<int16_t> d_output_queue[2];
	dmr_cai dmr;
	int d_msgq_id;
	gr::msg_queue::sptr d_msg_queue;
	bool d_stereo;
	int d_debug;
	op25_audio d_audio;
	log_ts& logts;
	std::array<std::deque<int16_t>, 2> &output_queue;
	int src_id[2];
};

    } // end namespace op25_repeater
} // end namespace gr
#endif // INCLUDED_RX_SYNC_H
