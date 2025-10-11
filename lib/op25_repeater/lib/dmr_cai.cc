//
// DMR Protocol Decoder (C) Copyright 2019 Graham J. Norbury
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <deque>
#include <errno.h>
#include <unistd.h>

#include "dmr_cai.h"

#include "op25_msg_types.h"
#include "bit_utils.h"
#include "dmr_const.h"
#include "hamming.h"
#include "crc16.h"

dmr_cai::dmr_cai(log_ts& logger, int debug, int msgq_id, gr::msg_queue::sptr queue) :
	d_slot{dmr_slot(0, logger, debug, msgq_id, queue), dmr_slot(1, logger, debug, msgq_id, queue)},
	d_slot_mask(3),
	d_chan(0),
	d_shift_reg(0),
	d_debug(debug),
	d_msgq_id(msgq_id),
	d_msg_queue(queue),
	logts(logger)
{
	d_cach_sig.clear();
	memset(d_frame, 0, sizeof(d_frame));
}

dmr_cai::~dmr_cai() {
}

void
dmr_cai::set_slot_mask(int mask) {
	d_slot_mask = mask;
	d_slot[0].set_slot_mask(mask);
	d_slot[1].set_slot_mask(mask);
}

void
dmr_cai::set_debug(int debug) {
    d_debug = debug;
    d_slot[0].set_debug(debug);
    d_slot[1].set_debug(debug);
}

std::pair<bool,long> dmr_cai::get_terminated(int slot) {
	if ((slot == 0) || (slot == 1)) {
		return d_slot[slot].get_terminated();
	}
	fprintf(stderr, "Error, Slot given is not 0 or 1\n");
	return std::make_pair(false, 0);
}

int dmr_cai::get_src_id(int slot) {
	if ((slot == 0) || (slot == 1)) {
		return d_slot[slot].get_src_id();
	}
	fprintf(stderr, "Error, Slot given is not 0 or 1\n");
	return -1;
}

int dmr_cai::get_dst_id(int slot) {
	if ((slot == 0) || (slot == 1)) {
		return d_slot[slot].get_dst_id();
	}
	fprintf(stderr, "Error, Slot given is not 0 or 1\n");
	return -1;
}

int dmr_cai::get_cc(int slot) {
	if ((slot == 0) || (slot == 1)) {
		return d_slot[slot].get_cc();
	}
	fprintf(stderr, "Error, Slot given is not 0 or 1\n");
	return -1;
}

void
dmr_cai::send_msg(const std::string& m_buf, const int m_type) {
	if ((d_msgq_id < 0) || (d_msg_queue->full_p()))
		return;

	gr::message::sptr msg = gr::message::make_from_string(m_buf, get_msg_type(PROTOCOL_DMR, m_type), (d_msgq_id << 1), logts.get_ts());
	if (!d_msg_queue->full_p())
	    d_msg_queue->insert_tail(msg);
}

bool
dmr_cai::load_frame(const uint8_t fr_sym[], bool& unmute) {
	dibits_to_bits(d_frame, fr_sym, FRAME_SIZE >> 1);

	// Check to see if burst contains SYNC identifying it as Voice or Data
	// SYNC pattern may not match exactly due to received bit errors
	// but the question is how many bit errors is too many...
	bool sync_rxd = false;
	uint64_t sl_sync = load_reg64(d_frame + SYNC_EMB + 24, 48);
	for (unsigned int i = 0; i < DMR_SYNC_MAGICS_COUNT; i ++) {
		if (__builtin_popcountll(sl_sync ^ DMR_SYNC_MAGICS[i]) <= DMR_SYNC_THRESHOLD) {
			sl_sync = DMR_SYNC_MAGICS[i];
			sync_rxd = true;
			break;
		}
	}

	// determine channel id either explicitly or incrementally
	unmute = false;
	if (sync_rxd) {
		switch(sl_sync) {
			case DMR_BS_VOICE_SYNC_MAGIC:
				extract_cach_fragment();
				break;
			case DMR_BS_DATA_SYNC_MAGIC:
				extract_cach_fragment();
				break;
			case DMR_T1_VOICE_SYNC_MAGIC:
				d_shift_reg = 0;
				d_chan = 0;
				break;
			case DMR_T2_VOICE_SYNC_MAGIC:
				d_shift_reg = 1;
				d_chan = 1;
				break;
		}
	} else {
		sl_sync = 0;
		d_shift_reg = (d_shift_reg << 1) + ((d_chan + 1) % 2);
		d_chan = slot_ids[d_shift_reg & 7];	
	}

	// decode the slot data
	unmute = d_slot[d_chan].load_slot(d_frame + 24, sl_sync);

	return sync_rxd;
}

void
dmr_cai::extract_cach_fragment() {
	int tact, tact_tc, tact_lcss;
	uint8_t tactbuf[sizeof(cach_tact_bits)];

	for (size_t i=0; i<sizeof(cach_tact_bits); i++)
		tactbuf[i] = d_frame[CACH + cach_tact_bits[i]];
	tact = hamming_7_4_decode[load_i(tactbuf, 7)];
	tact_tc   = (tact>>2) & 1; // TDMA Channel
	tact_lcss = tact & 3;      // Link Control Start/Stop
	d_shift_reg = (d_shift_reg << 1) + tact_tc;
	d_chan = slot_ids[d_shift_reg & 7];

	switch(tact_lcss) {
		case 0: // Begin CSBK
			// TODO: do something useful
			break;
		case 1: // Begin Short_LC
			d_cach_sig.clear();
			for (size_t i=0; i<sizeof(cach_payload_bits); i++)
				d_cach_sig.push_back(d_frame[CACH + cach_payload_bits[i]]);
			break;
		case 2: // End Short_LC or CSBK
			for (size_t i=0; i<sizeof(cach_payload_bits); i++)
				d_cach_sig.push_back(d_frame[CACH + cach_payload_bits[i]]);
			decode_shortLC();
			break;
		case 3: // Continue Short_LC or CSBK
			for (size_t i=0; i<sizeof(cach_payload_bits); i++)
				d_cach_sig.push_back(d_frame[CACH + cach_payload_bits[i]]);
			break;
	}
}

bool
dmr_cai::decode_shortLC()
{
	bool slc[68];
	bool hmg_result = true;

	// deinterleave
	int i, src;
	for (i = 0; i < 67; i++) {
		src = (i * 4) % 67;
		slc[i] = d_cach_sig[src];
	}
	slc[i] = d_cach_sig[i];

	// apply error correction
	hmg_result &= CHamming::decode17123(slc + 0);
	hmg_result &= CHamming::decode17123(slc + 17);
	hmg_result &= CHamming::decode17123(slc + 34);

	// check hamming results for unrecoverable errors
	if (!hmg_result)
		return false;

	// parity check
	for (i = 0; i < 17; i++) {
		if (slc[i+51] != (slc[i+0] ^ slc[i+17] ^ slc[i+34]))
			return false;
	}

	// remove hamming and leave 36 bits of Short LC
	for (i = 17; i < 29; i++) {
		slc[i-5] = slc[i];
	}
	for (i = 34; i < 46; i++) {
		slc[i-10] = slc[i];
	}

	// validate CRC8
	// are earlier checks even necessary as long as this passes?
	if (crc8((uint8_t*)slc, 36) != 0)
		return false;

	// extract useful data
	uint8_t slco, d0, d1, d2;
	slco = 0; d0 = 0; d1 = 0; d2 = 0;
	for (i = 0; i < 4; i++) {
		slco <<= 1;
		slco |= slc[i];
	}
	for (i = 0; i < 8; i++) {
		d0 <<= 1;
		d0 |= slc[i+4];
	}
	for (i = 0; i < 8; i++) {
		d1 <<= 1;
		d1 |= slc[i+12];
	}
	for (i = 0; i < 8; i++) {
		d2 <<= 1;
		d2 |= slc[i+20];
	}

	// send up the stack for further processing
	std::string slc_msg(4,0);
        slc_msg[0] = slco;
	slc_msg[1] = d0;
	slc_msg[2] = d1;
	slc_msg[3] = d2;
	send_msg(slc_msg, M_DMR_CACH_SLC);

	// decode a little further for logging purposes
	switch(slco) {
		case 0x0: { // Nul_Msg
			if (d_debug >= 10)
				fprintf(stderr, "%s SLCO=0x%x, NULL MSG\n", logts.get(d_msgq_id), slco);
			break;
		}

		case 0x1: { // Act_Updt
			uint8_t ts1_act = d0 >> 4;
			uint8_t ts2_act = d0 & 0xf;
			if (d_debug >= 10)
				fprintf(stderr, "%s SLCO=0x%x, ACTIVITY UPDATE TS1(%x), TS2(%x), HASH1(%02x), HASH2(%02x)\n", logts.get(d_msgq_id), slco, ts1_act, ts2_act, d1, d2);
			break;
		}

		case 0x2: { // Sys_Parms
			uint8_t model = d0 >> 6;
			uint8_t reg = (d1 >> 1) & 0x1;
			uint16_t cs_ctr = ((d1 << 8) + d2) & 0x1ff;
			uint16_t net;
			uint16_t site;
			switch(model) {
				case 0x0: // tiny
					net = ((d0 << 3) + (d1 >> 5)) & 0x1ff;
					site = (d1 >> 2) & 0x7;
					break;
				case 0x1: // small
					net = ((d0 << 1) + (d1 >> 7)) & 0x7f;
					site = (d1 >> 2) & 0x1f;
					break;
				case 0x2: // large
					net = (d0 >> 2) & 0xf;
					site = ((d0 << 6) + (d1 >> 2)) & 0xff;
					break;
				case 0x3: // huge
					net = (d0 >> 5) & 0x3;
					site = ((d0 << 6) + (d1 >> 2)) & 0x3f;
					break;
			}
			if (d_debug >= 10)
				fprintf(stderr, "%s SLCO=0x%x, C_SYS_PARM model(%d), net(%x), size(%x), reg(%d), cs_ctr(%x)\n", logts.get(d_msgq_id), slco, model, net, site, reg, cs_ctr);
			break;
		}

		case 0x9: { // Connect Plus Voice Channel
			uint16_t netId = (d0 << 4) + (d1 >> 4);
			uint8_t siteId = ((d1 & 0xf) << 4) + (d2 >> 4);
			if (d_debug >= 10)
				fprintf(stderr, "%s SLCO=0x%x, CONNECT PLUS VOICE CHANNEL netId(%03x), siteId(%02x)\n", logts.get(d_msgq_id), slco, netId, siteId);
			break;
		}

		case 0xa: { // Connect Plus Control Channel
			uint16_t netId = (d0 << 4) + (d1 >> 4);
			uint8_t siteId = ((d1 & 0xf) << 4) + (d2 >> 4);
			if (d_debug >= 10)
				fprintf(stderr, "%s SLCO=0x%x, CONNECT PLUS CONTROL CHANNEL netId(%03x), siteId(%02x)\n", logts.get(d_msgq_id), slco, netId, siteId);
			break;
		}

		case 0xf: { // Capacity Plus
			uint8_t lcn = d1 & 0xf;
			if (d_debug >= 10)
				fprintf(stderr, "%s SLCO=0x%x, CAPACITY PLUS REST CHANNEL LCN(%x)\n", logts.get(d_msgq_id), slco, lcn);
			break;
		}

		default: {
			if (d_debug >= 10)
				fprintf(stderr, "%s SLCO=0x%x, DATA=%02x %02x %02x\n", logts.get(d_msgq_id), slco, d0, d1, d2);
		}
	}

	return true;
}
