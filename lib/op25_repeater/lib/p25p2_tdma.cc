// P25 TDMA Decoder (C) Copyright 2013, 2014, 2021 Max H. Parke KA1RBI
// Copyright 2017-2021 Graham J. Norbury (modularization rewrite, additional messages)
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

#include <stdint.h>
#include <map>
#include <string.h>
#include <string>
#include <iostream>
#include <assert.h>
#include <errno.h>
#include <sys/time.h>

#include "imbe_decoder.h"
#include "op25_msg_types.h"
#include "p25p2_duid.h"
#include "p25p2_sync.h"
#include "p25p2_tdma.h"
#include "p25p2_vf.h"
#include "p25_crypt_algs.h"
#include "mbelib.h"
#include "ambe.h"
#include "crc16.h"

static const int BURST_SIZE = 180;
static const int SUPERFRAME_SIZE = (12*BURST_SIZE);

static uint16_t crc12(const uint8_t bits[], unsigned int len) {
	uint16_t crc=0;
	static const unsigned int K = 12;
	static const uint8_t poly[K+1] = {1,1,0,0,0,1,0,0,1,0,1,1,1}; // p25 p2 crc 12 poly
	uint8_t buf[256];
	if (len+K > sizeof(buf)) {
		fprintf (stderr, "crc12: buffer length %u exceeds maximum %lu\n", len+K, sizeof(buf));
		return 0;
	}
	memset (buf, 0, sizeof(buf));
	for (unsigned int i=0; i<len; i++){
		buf[i] = bits[i];
	}
	for (unsigned int i=0; i<len; i++)
		if (buf[i])
			for (unsigned int j=0; j<K+1; j++)
				buf[i+j] ^= poly[j];
	for (unsigned int i=0; i<K; i++){
		crc = (crc << 1) + buf[len + i];
	}
	return crc ^ 0xfff;
}

static bool crc12_ok(const uint8_t bits[], unsigned int len) {
	uint16_t crc = 0;
	for (unsigned int i=0; i < 12; i++) {
		crc = (crc << 1) + bits[len+i];
	}
	return (crc == crc12(bits,len));
}

static const uint8_t mac_msg_len[256] = {
	 0,  7,  8,  7,  0, 16,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
	 0, 14, 15,  0,  0, 15,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
	 5,  7,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
	 9,  7,  9,  0,  9,  8,  9,  0, 10, 10,  9,  0, 10,  0,  0,  0, 
	 0,  0,  0,  0,  9,  7,  0,  0, 10,  0,  7,  0, 10,  8, 14,  7, 
	 9,  9,  0,  0,  9,  0,  0,  9, 10,  0,  7, 10, 10,  7,  0,  9, 
	 9, 29,  9,  9,  9,  9, 10, 13,  9,  9,  9, 11,  9,  9,  0,  0, 
	 8,  0,  0,  7, 11,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
	16,  0,  0, 11, 13, 11, 11, 11, 10,  0,  0,  0,  0,  0,  0,  0, 
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 
	11,  0,  0,  8, 15, 12, 15, 32, 12, 12,  0, 27, 14, 29, 29, 32, 
	 0,  0,  0,  0,  0,  0,  9,  0, 14, 29, 11, 27, 14,  0, 40, 11, 
	28,  0,  0, 14, 17, 14,  0,  0, 16,  8, 11,  0, 13, 19,  0,  0, 
	 0,  0, 16, 14,  0,  0, 12,  0, 22,  0, 11, 13, 11,  0, 15,  0 };

p25p2_tdma::p25p2_tdma(const op25_audio& udp, log_ts& logger, int slotid, int debug, bool do_msgq, gr::msg_queue::sptr queue, std::deque<int16_t> &qptr, bool do_audio_output, bool soft_vocoder, int msgq_id) :	// constructor
	tdma_xormask(new uint8_t[SUPERFRAME_SIZE]),
	symbols_received(0),
	packets(0),
	write_bufp(0),
	d_slotid(slotid),
	d_tdma_slot_first_4v(-1),
	mbe_err_cnt(0),
	tone_frame(false),
	d_msg_queue(queue),
	output_queue_decode(qptr),
	d_do_msgq(do_msgq),
	d_msgq_id(msgq_id),
	d_do_audio_output(do_audio_output),
	d_soft_vocoder(soft_vocoder),
	op25audio(udp),
    logts(logger),
	d_nac(0),
	d_debug(debug),
	burst_id(-1),
	burst_type(-1),
	ESS_A(28,0),
	ESS_B(16,0),
	ess_keyid(0),
	ess_algid(0x80),
	next_keyid(0),
	next_algid(0x80),
	p2framer(),
    crypt_algs(logger, debug, msgq_id)
{
	assert (slotid == 0 || slotid == 1);
	mbe_initMbeParms (&cur_mp, &prev_mp, &enh_mp);
	mbe_initToneParms (&tone_mp);
	mbe_initErrParms (&errs_mp);
}

bool p25p2_tdma::rx_sym(uint8_t sym)
{
	symbols_received++;
	return p2framer.rx_sym(sym);
}

void p25p2_tdma::set_slotid(int slotid)
{
	assert (slotid == 0 || slotid == 1);
	d_slotid = slotid;
}

void p25p2_tdma::call_end() {
    op25audio.send_audio_flag(op25_audio::DRAIN);
	reset_ess();
	reset_vb();
	d_tdma_slot_first_4v = -1;
	burst_type = -1;
}

void p25p2_tdma::crypt_reset() {
	crypt_algs.reset();
}

void p25p2_tdma::crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
	crypt_algs.key(keyid, algid, key);
}

void p25p2_tdma::reset_call_terminated() {
	terminate_call = std::pair<bool,long>(false, 0);
}

std::pair<bool,long> p25p2_tdma::get_call_terminated() {
	return terminate_call;
}

long p25p2_tdma::get_ptt_src_id() {
	long id = src_id;
	src_id = -1;
	return id;
}

long p25p2_tdma::get_ptt_grp_id() {
	long addr = grp_id;
    grp_id = -1;
	return addr;
}

p25p2_tdma::~p25p2_tdma()	// destructor
{
	delete[](tdma_xormask);
}

void
p25p2_tdma::set_xormask(const char*p) {
	for (int i=0; i<SUPERFRAME_SIZE; i++)
		tdma_xormask[i] = p[i] & 3;
}

int p25p2_tdma::process_mac_pdu(const uint8_t byte_buf[], const unsigned int len, const int rs_errs) 
{
    unsigned int offset = (byte_buf[0] >> 2) & 0x7;
    unsigned int opcode = (byte_buf[0] >> 5) & 0x7;

    if (d_debug >= 10) {
        fprintf(stderr, "%s process_mac_pdu: opcode %d offset %d len %d\n", logts.get(d_msgq_id), opcode, offset, len);
    }

    switch (opcode)
    {
        case 0: // MAC_SIGNAL
            handle_mac_signal(byte_buf, len, rs_errs);
            break;

        case 1: // MAC_PTT
            handle_mac_ptt(byte_buf, len, rs_errs);
            // capture the offset field
            // PT.1 has offset=1, PT.0 has offset=0
            // add offset + 1 to the current TDMA slot to get the first 4V position
            // normalize TDMA slot 0-9 to ch0/ch1 slot 0-4
            d_tdma_slot_first_4v = ((sync.tdma_slotid() >> 1) + offset + 1) % 5; //TODO: handle offset > 4
            break;

        case 2: // MAC_END_PTT
            handle_mac_end_ptt(byte_buf, len, rs_errs);
            break;

        case 3: // MAC_IDLE
            handle_mac_idle(byte_buf, len, rs_errs);
            break;

        case 4: // MAC_ACTIVE
            handle_mac_active(byte_buf, len, rs_errs);
            // also capture the offset field here
            // it can be captured directly as for non-PTT PDUs it simply stores the offset to the first 4V
            d_tdma_slot_first_4v = (offset > 4) ? 0 : offset;
            break;

        case 6: // MAC_HANGTIME
            handle_mac_hangtime(byte_buf, len, rs_errs);
            break;
    }
	// maps sacch opcodes into phase I duid values 
	static const int opcode_map[8] = {3, 5, 15, 15, 5, 3, 3, 3};
	return opcode_map[opcode];
}

void p25p2_tdma::handle_mac_signal(const uint8_t byte_buf[], const unsigned int len, const int rs_errs) 
{
        int nac;
        nac = (byte_buf[19] << 4) + ((byte_buf[20] >> 4) & 0xf);
        if (d_debug >= 10) {
                fprintf(stderr, "%s MAC_SIGNAL: colorcd=0x%03x, ", logts.get(d_msgq_id), nac);
        }
        decode_mac_msg(byte_buf, 18, nac);
        if (d_debug >= 10)
                fprintf(stderr, ", rs_errs=%d\n", rs_errs);
}

void p25p2_tdma::handle_mac_ptt(const uint8_t byte_buf[], const unsigned int len, const int rs_errs) 
{
	    std::string pdu;
		pdu.assign(len+2, 0);
		pdu[0] = 0xff; pdu[1] = 0xff;
		for (unsigned int i = 0; i < len; i++) {
			pdu[2 + i] = byte_buf[1 + i];
		}
		send_msg(pdu, M_P25_MAC_PTT);
        uint32_t srcaddr = (byte_buf[13] << 16) + (byte_buf[14] << 8) + byte_buf[15];
        uint16_t grpaddr = (byte_buf[16] << 8) + byte_buf[17];

        if (d_debug >= 10) {
                fprintf(stderr, "%s MAC_PTT: srcaddr=%u, grpaddr=%u, TDMA slot ID=%u", logts.get(d_msgq_id), srcaddr, grpaddr, sync.tdma_slotid());
        }

        for (int i = 0; i < 9; i++) {
                ess_mi[i] = byte_buf[i+1];
        }
        ess_algid = byte_buf[10];
        ess_keyid = (byte_buf[11] << 8) + byte_buf[12];
        if (d_debug >= 10) {
                fprintf(stderr, ", algid=%x, keyid=%x, mi=%02x %02x %02x %02x %02x %02x %02x %02x %02x, rs_errs=%d\n",
			ess_algid, ess_keyid,
			ess_mi[0], ess_mi[1], ess_mi[2], ess_mi[3], ess_mi[4], ess_mi[5],ess_mi[6], ess_mi[7], ess_mi[8],
			rs_errs);
        }
		if (encrypted()) {
			crypt_algs.prepare(ess_algid, ess_keyid, PT_P25_PHASE2, ess_mi);
		}

		src_id = srcaddr;
		grp_id = grpaddr;

		std::string s = "{\"srcaddr\" : " + std::to_string(srcaddr) + ", \"grpaddr\": " + std::to_string(grpaddr) + "}";
        send_msg(s, -3);
		reset_vb();
}

void p25p2_tdma::handle_mac_end_ptt(const uint8_t byte_buf[], const unsigned int len, const int rs_errs) 
{
	    std::string pdu;
		pdu.assign(len+2, 0);
		pdu[0] = 0xff; pdu[1] = 0xff;
		for (unsigned int i = 0; i < len; i++) {
			pdu[2 + i] = byte_buf[1 + i];
		}
		send_msg(pdu, M_P25_MAC_END_PTT);

        uint16_t colorcd = ((byte_buf[1] & 0x0f) << 8) + byte_buf[2];
        uint32_t srcaddr = (byte_buf[13] << 16) + (byte_buf[14] << 8) + byte_buf[15];
        uint16_t grpaddr = (byte_buf[16] << 8) + byte_buf[17];

		//src_id = srcaddr; // the decode for Source Address is not correct
		grp_id = grpaddr;

        if (d_debug >= 10)
                fprintf(stderr, "%s MAC_END_PTT: colorcd=0x%03x, srcaddr=%u, grpaddr=%u, rs_errs=%d\n", logts.get(d_msgq_id), colorcd, srcaddr, grpaddr, rs_errs);

        op25audio.send_audio_flag(op25_audio::DRAIN);
		terminate_call = std::pair<bool,long>(true, output_queue_decode.size());
		// reset crypto parameters
        reset_ess();
}

void p25p2_tdma::handle_mac_idle(const uint8_t byte_buf[], const unsigned int len, const int rs_errs) 
{
        if (d_debug >= 10)
                fprintf(stderr, "%s MAC_IDLE: ", logts.get(d_msgq_id));

        decode_mac_msg(byte_buf, len);
        op25audio.send_audio_flag(op25_audio::DRAIN);

        if (d_debug >= 10)
                fprintf(stderr, ", rs_errs=%d\n", rs_errs);
}

void p25p2_tdma::handle_mac_active(const uint8_t byte_buf[], const unsigned int len, const int rs_errs) 
{
        if (d_debug >= 10)
                fprintf(stderr, "%s MAC_ACTIVE: ", logts.get(d_msgq_id));

        decode_mac_msg(byte_buf, len);

        if (d_debug >= 10)
                fprintf(stderr, ", rs_errs=%d\n", rs_errs);
}

void p25p2_tdma::handle_mac_hangtime(const uint8_t byte_buf[], const unsigned int len, const int rs_errs) 
{
        if (d_debug >= 10)
                fprintf(stderr, "%s MAC_HANGTIME: ", logts.get(d_msgq_id));

        decode_mac_msg(byte_buf, len);
        op25audio.send_audio_flag(op25_audio::DRAIN);

        if (d_debug >= 10)
                fprintf(stderr, ", rs_errs=%d\n", rs_errs);
}


void p25p2_tdma::decode_mac_msg(const uint8_t byte_buf[], const unsigned int len, const uint16_t nac) 
{
	std::string s;
	std::string pdu;
	uint8_t b1b2, mco, op, mfid, msg_ptr, msg_len, len_remaining;
    uint16_t colorcd;

	colorcd = nac;
	for (msg_ptr = 1; msg_ptr < len; )
	{
		len_remaining = len - msg_ptr;
        b1b2 = byte_buf[msg_ptr] >> 6;
        mco  = byte_buf[msg_ptr] & 0x3f;
        op   = (b1b2 << 6) + mco;
		mfid = 0;

		// Find message length using opcode handlers or lookup table
		switch (op) {
			case 0x00: // Null Information
				msg_len = len_remaining;
				break;
			case 0x08: // Null Avoid Zero Bias Message
				msg_len = byte_buf[msg_ptr+1] & 0x3f;
				break;
			case 0x11: // Indirect Group Paging without Priority
				msg_len = (((byte_buf[msg_ptr+1] & 0x3) + 1) * 2) + 2;
				break;
			case 0x12: // Individual Paging with Priority
				msg_len = (((byte_buf[msg_ptr+1] & 0x3) + 1) * 3) + 2;
				break;
			default:
				if (b1b2 == 0x2) {				// Manufacturer-specific ops have len field
					mfid = byte_buf[msg_ptr+1];
					msg_len = byte_buf[msg_ptr+2] & 0x3f;
				} else {
					msg_len = mac_msg_len[op];	// Lookup table for everything else
				}
		}

		if (d_debug >= 10) {
			fprintf(stderr, "mco=%01x/%02x(0x%02x), len=%d", b1b2, mco, op, msg_len);
		}

		// Generic message processing
		if (b1b2 == 0x1) {
			// Derived from FDMA CAI abbreviated format; convert to TSBK
			convert_abbrev_msg(byte_buf+msg_ptr, colorcd, mfid);
		} else if ((op != 0x00) && (op != 0x08) && (msg_len != 0)) {
			// Unique TDMA CAI message
			// Manufacturer specific message
			// Derived from FDMA CAI extended or explicit format
			pdu.assign(msg_len+2, 0);
			pdu[0] = colorcd >> 8; pdu[1] = colorcd & 0xff;
			for (int i = 0; i < msg_len; i++) {
				pdu[2 + i] = byte_buf[msg_ptr + i];
			}
			send_msg(pdu, M_P25_MAC_PDU);
		} else {
			// Discard Null, Null-Avoid-Zero-Bias, and messages with unknown length
		}

		msg_ptr = (msg_len == 0) ? len : (msg_ptr + msg_len);

		if ((d_debug >= 10) && (msg_ptr < len)) {
			fprintf(stderr,", ");
		}
	}
}

void p25p2_tdma::convert_abbrev_msg(const uint8_t byte_buf[], const uint16_t nac, const uint8_t mfid) 
{
    if ((byte_buf[0] & 0xc0) != 0x40) // b1b2=0x1 (abbreviated form messages only)
		return;

	std::string tsbk(12,0);
	tsbk[0] = nac >> 8; tsbk[1] = nac & 0xff;
	tsbk[2] = 0x80 + (byte_buf[0] & 0x3f); // opcode with LB bit set
	tsbk[3] = 0x00;                        // mfrid
	for (int i = 4; i <= 11; i++) {
		tsbk[i] = byte_buf[i-3];
	}
	send_msg(tsbk, M_P25_DUID_TSBK);
}

int p25p2_tdma::handle_acch_frame(const uint8_t dibits[], bool fast, bool is_lcch) 
{
	int rc, rs_errs = 0;
	unsigned int i, j = 0;
	uint8_t bits[512];
	std::vector<uint8_t> HB(63,0);
	std::vector<int> Erasures;
	uint8_t byte_buf[32];
	unsigned int bufl=0;
	unsigned int len=0;
	if (fast) {
		for (i=11; i < 11+36; i++) {
			bits[bufl++] = (dibits[i] >> 1) & 1;
			bits[bufl++] = dibits[i] & 1;
		}
		for (i=48; i < 48+31; i++) {
			bits[bufl++] = (dibits[i] >> 1) & 1;
			bits[bufl++] = dibits[i] & 1;
		}
		for (i=100; i < 100+32; i++) {
			bits[bufl++] = (dibits[i] >> 1) & 1;
			bits[bufl++] = dibits[i] & 1;
		}
		for (i=133; i < 133+36; i++) {
			bits[bufl++] = (dibits[i] >> 1) & 1;
			bits[bufl++] = dibits[i] & 1;
		}
	} else {
		for (i=11; i < 11+36; i++) {
			bits[bufl++] = (dibits[i] >> 1) & 1;
			bits[bufl++] = dibits[i] & 1;
		}
		for (i=48; i < 48+84; i++) {
			bits[bufl++] = (dibits[i] >> 1) & 1;
			bits[bufl++] = dibits[i] & 1;
		}
		for (i=133; i < 133+36; i++) {
			bits[bufl++] = (dibits[i] >> 1) & 1;
			bits[bufl++] = dibits[i] & 1;
		}
	}

	// Reed-Solomon
	if (fast) {
		j = 9;
		len = 270;
		Erasures = {0,1,2,3,4,5,6,7,8,54,55,56,57,58,59,60,61,62};
	} else {
		j = 5;
		len = 312;
		Erasures = {0,1,2,3,4,57,58,59,60,61,62};
	}

	for (i = 0; i < len; i += 6) { // convert bits to hexbits
		HB[j] = (bits[i] << 5) + (bits[i+1] << 4) + (bits[i+2] << 3) + (bits[i+3] << 2) + (bits[i+4] << 1) + bits[i+5];
		j++;
	}
	rs_errs = rs28.decode(HB, Erasures);
	if (rs_errs < 0) // Don't need upper limit on corrections because pdu is crc12 checked before subsequent processing
		return -1;

	// Adjust FEC error counter to eliminate erasures
	if (rs_errs >= (int)Erasures.size())
		rs_errs -= (int)Erasures.size();

	if (fast) {
		j = 9;
		len = 144;
	} else {
		j = 5;
		len = (is_lcch) ? 180 : 168;
	}
	for (i = 0; i < len; i += 6) { // convert hexbits back to bits
		bits[i]   = (HB[j] & 0x20) >> 5;
		bits[i+1] = (HB[j] & 0x10) >> 4;
		bits[i+2] = (HB[j] & 0x08) >> 3;
		bits[i+3] = (HB[j] & 0x04) >> 2;
		bits[i+4] = (HB[j] & 0x02) >> 1;
		bits[i+5] = (HB[j] & 0x01);
		j++;
	}

	bool crc_ok = (is_lcch) ? (crc16(bits, len) == 0) : crc12_ok(bits, len);
	unsigned int olen = (is_lcch) ? 23 : len/8;
	rc = -1;
	if (crc_ok) { // TODO: rewrite crc12 so we don't have to do so much bit manipulation
		for (i=0; i<olen; i++) {
			byte_buf[i] = (bits[i*8 + 0] << 7) + (bits[i*8 + 1] << 6) + (bits[i*8 + 2] << 5) + (bits[i*8 + 3] << 4) + (bits[i*8 + 4] << 3) + (bits[i*8 + 5] << 2) + (bits[i*8 + 6] << 1) + (bits[i*8 + 7] << 0);
		}
		rc = process_mac_pdu(byte_buf, olen, rs_errs);
	}
	return rc;
}

void p25p2_tdma::handle_voice_frame(const uint8_t dibits[], int slot, int voice_subframe)
{
	int16_t samples_buf[IMBE_SAMPLES_PER_FRAME];
	packed_codeword p_cw;
    bool audio_valid = !encrypted();
	int u[4];
	int b[9];
	size_t errs;
	int16_t snd;
	int K;
	int rc = -1;
	frame_type fr_type;

	// Deinterleave and figure out frame type:
	errs = vf.process_vcw(&errs_mp, dibits, b, u);
	if (d_debug >= 9) {
		char log_str[40];
		vf.pack_cw(p_cw, u);
		strcpy(log_str, logts.get(d_msgq_id)); // param eval order not guaranteed; force timestamp computation first
		fprintf(stderr, "%s AMBE %02x %02x %02x %02x %02x %02x %02x errs %lu err_rate %f, dt %f\n",
			    log_str,
			    p_cw[0], p_cw[1], p_cw[2], p_cw[3], p_cw[4], p_cw[5], p_cw[6], errs, errs_mp.ER,
				logts.get_tdiff());            // dt is time in seconds since last AMBE frame processed
		logts.mark_ts();
	}

	// Pass encrypted traffic through the decryption algorithms
	if (encrypted()) {
		switch (slot) {
			case 0:
				fr_type = FT_4V_0;
				break;
			case 1:
				fr_type = FT_4V_1;
				break;
			case 2:
				fr_type = FT_4V_2;
				break;
			case 3:
				fr_type = FT_4V_3;
				break;
			case 4:
				fr_type = FT_2V;
				break;
		}
		vf.pack_cw(p_cw, u);
		audio_valid = crypt_algs.process(p_cw, fr_type, voice_subframe);
		if (!audio_valid)
			return;
        vf.unpack_cw(p_cw, u);  // unpack plaintext codewords
        vf.unpack_b(b, u);      // for unencrypted traffic this is done inside vf.process_vcw()
	}

	rc = mbe_dequantizeAmbeTone(&tone_mp, &errs_mp, u);
	if (rc >= 0) {					// Tone Frame
		if (rc == 0) {                  // Valid Tone
			tone_frame = true;
			mbe_err_cnt = 0;
		} else {                        // Tone Erasure with Frame Repeat
			if ((++mbe_err_cnt < 4) && tone_frame) {
				mbe_useLastMbeParms(&cur_mp, &prev_mp);
				rc = 0;
			} else {
				tone_frame = false;     // Mute audio output after 3 successive Frame Repeats
			}
        }
	} else {
		rc = mbe_dequantizeAmbe2250Parms (&cur_mp, &prev_mp, &errs_mp, b);
		if (rc == 0) {				// Voice Frame
			tone_frame = false;
			mbe_err_cnt = 0;
		} else if ((++mbe_err_cnt < 4) && !tone_frame) {// Erasure with Frame Repeat per TIA-102.BABA.5.6
			mbe_useLastMbeParms(&cur_mp, &prev_mp);
			rc = 0;
		} else {
			tone_frame = false;         // Mute audio output after 3 successive Frame Repeats
		}
	}

	// Synthesize tones or speech as long as dequantization was successful and overall error rate is below threshold
	if ((rc == 0) && (errs_mp.ER <= 0.096)) {
		if (tone_frame) {
			software_decoder.decode_tone(samples_buf, tone_mp.ID, tone_mp.AD, &tone_mp.n);
		} else if(d_soft_vocoder) {
			K = 12;
			if (cur_mp.L <= 36)
				K = int(float(cur_mp.L + 2.0) / 3.0);
			software_decoder.decode_tap(samples_buf, cur_mp.L, K, cur_mp.w0, &cur_mp.Vl[1], &cur_mp.Ml[1]);
		} else {
			vocoder.decode_tap(samples_buf, cur_mp.L, cur_mp.w0, &cur_mp.Vl[1], &cur_mp.Ml[1]);
		}
	}

	// Populate output buffer with either audio samples or silence
	write_bufp = 0;
	for (int i=0; i < IMBE_SAMPLES_PER_FRAME; i++) {
		snd = samples_buf[i];
		output_queue_decode.push_back(snd); // outputs the sound
		write_buf[write_bufp++] = snd & 0xFF ;
		write_buf[write_bufp++] = snd >> 8;
	}
	if (d_do_audio_output && (write_bufp >= 0)) { 
		op25audio.send_audio(write_buf, write_bufp);
		write_bufp = 0;
	}

	mbe_moveMbeParms (&cur_mp, &prev_mp);
	mbe_moveMbeParms (&cur_mp, &enh_mp);
}

int p25p2_tdma::handle_frame(void)
{
	uint8_t dibits[180];
	int rc;
	for (size_t i=0; i<sizeof(dibits); i++)
		dibits[i] = p2framer.d_frame_body[i*2+1] + (p2framer.d_frame_body[i*2] << 1);
	rc = handle_packet(dibits, p2framer.get_fs());
	return rc;
}

/* returns true if in sync and slot matches current active slot d_slotid */
int p25p2_tdma::handle_packet(uint8_t dibits[], const uint64_t fs) 
{
	// descramble and process the frame
	int rc = -1;
	static const int which_slot[] = {0,1,0,1,0,1,0,1,0,1,1,0};
	packets++;
	sync.check_confidence(dibits);
	if (!sync.in_sync())
		return -1;
	const uint8_t* burstp = &dibits[10];
	uint8_t xored_burst[BURST_SIZE - 10];
	burst_type = duid.duid_lookup(duid.extract_duid(burstp));
	if ((burst_type != 13) && (which_slot[sync.tdma_slotid()] != d_slotid)) // only permit control channel or active slot
		return -1;
	for (int i=0; i<BURST_SIZE - 10; i++) {
		xored_burst[i] = burstp[i] ^ tdma_xormask[sync.tdma_slotid() * BURST_SIZE + i];
	}
	if (burst_type == 0 || burst_type == 6)	{       // 4V or 2V burst
		// normalize current TDMA slot from 0-9 to ch0/ch1 slot 0-4
		int current_slot = sync.tdma_slotid() >> 1;

		track_vb();

		// update "first 4V" variable here as well as it always follows 2V
		// in case we missed both PTTs
		if (burst_type == 6 && sync.last_rc() >= 0)
			d_tdma_slot_first_4v = (current_slot + 1) % 5;

		if (d_tdma_slot_first_4v >= 0 && sync.last_rc() >= 0) {
			// now let's see if the voice frame received is the one we expected to get.
			// shift the range from [0, 4] to [first_4V, first_4V+4]
			if (current_slot < (int) d_tdma_slot_first_4v)
				current_slot += 5;

			// translate [first_4V, first_4V+1] to the burst_id we would expect to be in this slot
			current_slot -= d_tdma_slot_first_4v;

			if (current_slot != burst_id && current_slot > burst_id) {
				int need_to_skip = current_slot - burst_id;
				// XXX determine if the 2V frame was missed?
				if (d_debug >= 10) {
					fprintf(stderr, "%i voice frame(s) missing; expecting %uV_%u but got %uV_%u. ISCH rc=%d\n", need_to_skip, (burst_id == 4 ? 2 : 4), burst_id, (burst_type == 6 ? 2 : 4), current_slot, sync.last_rc());
				}
				burst_id = current_slot;
			}
		}

		handle_4V2V_ess(&xored_burst[84]);
		handle_voice_frame(&xored_burst[11], current_slot, 0);
		handle_voice_frame(&xored_burst[48], current_slot, 1);
		if (burst_type == 0) {
			handle_voice_frame(&xored_burst[96], current_slot, 2);
			handle_voice_frame(&xored_burst[133], current_slot, 3);
		} else /* if (burst_type == 6) */ {
			// promote next set of encryption parameters AFTER we get the full ESS & process the 2V frame
			ess_algid = next_algid;
			ess_keyid = next_keyid;
			memcpy(ess_mi, next_mi, sizeof(ess_mi));
			if (encrypted()) {
				crypt_algs.prepare(ess_algid, ess_keyid, PT_P25_PHASE2, ess_mi);
			}
			std::string s = "{\"encrypted\": " + std::to_string((encrypted()) ? 1 : 0) + ", \"algid\": " + std::to_string(ess_algid) + ", \"keyid\": " + std::to_string(ess_keyid) + "}";
			send_msg(s, M_P25_JSON_DATA);
		}
		return -1;
	} else if (burst_type == 3) {                   // scrambled sacch
		rc = handle_acch_frame(xored_burst, 0, false);
	} else if (burst_type == 4) {                   // scrambled lcch
		// rc = handle_acch_frame(xored_burst, 0, true); // only used for IECI (not supported)
		return -1;
	} else if (burst_type == 9) {                   // scrambled facch
		rc = handle_acch_frame(xored_burst, 1, false);
	} else if (burst_type == 12) {                  // unscrambled sacch
		rc = handle_acch_frame(burstp, 0, false);
	} else if (burst_type == 13) {                  // unscrambled lcch / TDMA CC OECI
		rc = handle_acch_frame(burstp, 0, true);
	} else if (burst_type == 15) {                  // unscrambled facch
		rc = handle_acch_frame(burstp, 1, false);
	} else {
		// unsupported type duid
		return -1;
	}

	if (rc > -1)
		send_msg(std::string(2, 0xff), rc);
	return rc;
}

void p25p2_tdma::handle_4V2V_ess(const uint8_t dibits[])
{
	int ec = 0;

	if (d_debug >= 10) {
		fprintf(stderr, "%s %s_BURST(%d) TDMA slot ID=%u ", logts.get(d_msgq_id), (burst_type == 0) ? "4V" : "2V", burst_id, sync.tdma_slotid());
	}

	if (burst_id < 4) {
		for (int i=0; i < 12; i += 3) { // ESS-B is 4 hexbits / 12 dibits
			ESS_B[(4 * burst_id) + (i / 3)] = (uint8_t) ((dibits[i] << 4) + (dibits[i+1] << 2) + dibits[i+2]);
		}
	} else {
		int i, j;

		j = 0;
		for (i = 0; i < 28; i++) { // ESS-A is 28 hexbits / 84 dibits
			ESS_A[i] = (uint8_t) ((dibits[j] << 4) + (dibits[j+1] << 2) + dibits[j+2]);
			j = (i == 15) ? (j + 4) : (j + 3);  // skip dibit containing DUID#3
		}

		ec = rs28.decode(ESS_B, ESS_A);

		if ((ec >= 0) && (ec <= 14)) { // upper limit 14 corrections
			next_algid = (ESS_B[0] << 2) + (ESS_B[1] >> 4);
			next_keyid = ((ESS_B[1] & 15) << 12) + (ESS_B[2] << 6) + ESS_B[3]; 

			j = 0;
			for (i = 0; i < 9;) {
				next_mi[i++] = (uint8_t)  (ESS_B[j+4]         << 2) + (ESS_B[j+5] >> 4);
				next_mi[i++] = (uint8_t) ((ESS_B[j+5] & 0x0f) << 4) + (ESS_B[j+6] >> 2);
				next_mi[i++] = (uint8_t) ((ESS_B[j+6] & 0x03) << 6) +  ESS_B[j+7];
				j += 4;
			}
		}
	}     

	if (d_debug >= 10 && burst_id == 4) {
		fprintf(stderr, "ESS: algid=%x, keyid=%x, mi=%02x %02x %02x %02x %02x %02x %02x %02x %02x, rs_errs=%d\n",
			    next_algid, next_keyid,
			    next_mi[0], next_mi[1], next_mi[2], next_mi[3], next_mi[4], next_mi[5],next_mi[6], next_mi[7], next_mi[8],
			    ec);        
	} else if (d_debug >= 10) {
		fprintf(stderr, "ESS: (partial)\n");
	}
}

void p25p2_tdma::send_msg(const std::string msg_str, long msg_type)
{
	if (!d_do_msgq || d_msg_queue->full_p())
		return;

	gr::message::sptr msg = gr::message::make_from_string(msg_str, msg_type, 0, 0);           
    if (!d_msg_queue->full_p())
    	d_msg_queue->insert_tail(msg);
}
