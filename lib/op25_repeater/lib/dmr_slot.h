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

#ifndef INCLUDED_DMR_SLOT_H
#define INCLUDED_DMR_SLOT_H

#include <stdint.h>
#include <vector>
#include <gnuradio/msg_queue.h>

#include "frame_sync_magics.h"
#include "dmr_const.h"
#include "bptc19696.h"
#include "trellis.h"
#include "ezpwd/rs"
#include "log_ts.h"

typedef std::vector<bool> bit_vector;
typedef std::vector<uint8_t> byte_vector;

enum data_state { DATA_INVALID = 0, DATA_VALID, DATA_INCOMPLETE };

static const unsigned int DMR_SYNC_THRESHOLD       = 6;
static const unsigned int DMR_SYNC_MAGICS_COUNT    = 9;
static const uint64_t     DMR_SYNC_MAGICS[]        = {DMR_BS_VOICE_SYNC_MAGIC,
						      DMR_BS_DATA_SYNC_MAGIC,
						      DMR_MS_VOICE_SYNC_MAGIC,
						      DMR_MS_DATA_SYNC_MAGIC,
						      DMR_MS_RC_SYNC_MAGIC,
						      DMR_T1_VOICE_SYNC_MAGIC,
						      DMR_T1_DATA_SYNC_MAGIC,
						      DMR_T2_VOICE_SYNC_MAGIC,
						      DMR_T2_DATA_SYNC_MAGIC};

static const uint8_t VOICE_LC_HEADER_CRC_MASK[]    = {1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0}; // 0x969696
static const uint8_t TERMINATOR_WITH_LC_CRC_MASK[] = {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1}; // 0x999999
static const uint8_t PI_HEADER_CRC_MASK[]          = {0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1};                 // 0x6969
static const uint8_t DATA_HEADER_CRC_MASK[]        = {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0};                 // 0xCCCC
static const uint8_t CSBK_CRC_MASK[]               = {1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1};                 // 0xA5A5
static const uint8_t MBC_HEADER_CRC_MASK[]         = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0};                 // 0xAAAA

static const unsigned int SLOT_SIZE                = 264; // size in bits
static const unsigned int PAYLOAD_L                =   0; // starting position in bits
static const unsigned int PAYLOAD_R                = 156;
static const unsigned int SYNC_EMB                 = 108;
static const unsigned int SLOT_L                   =  98;
static const unsigned int SLOT_R                   = 156;

class dmr_slot {
public:
	dmr_slot(const int chan, log_ts& logger, const int debug, int msgq_id, gr::msg_queue::sptr queue);
	~dmr_slot();
	inline void set_debug(const int debug) { d_debug = debug; };
	bool load_slot(const uint8_t slot[], uint64_t sl_type);
	inline void set_slot_mask(int mask) { d_slot_mask = mask; };

	/* For getting Src and Terminator for DMR Recorder */
	int get_src_id(); 
	int get_dst_id();
	int get_cc();
	std::pair<bool,long> get_terminated();

private:
	uint8_t     d_slot[SLOT_SIZE];	// array of bits comprising the current slot
	bit_vector  d_slot_type;
	byte_vector d_emb;		// last received Embedded data
	byte_vector d_mbc;		// last received MBC data
	byte_vector d_dhdr;		// last received Data Header data
	byte_vector d_pdp;		// last received PDP data
	byte_vector d_lc;		// last received LC data
	uint8_t     d_rc;		// last received RC data
	uint16_t    d_sb;		// last received SB data
	byte_vector d_pi;
	uint8_t     d_pdp_bf;
	uint8_t     d_pdp_poc;
	data_state  d_mbc_state;
	data_state  d_dhdr_state;
	data_state  d_pdp_state;
	bool        d_lc_valid;		// flag indicating if LC data is valid
	bool        d_rc_valid;		// flag indicating if RC data is valid
	bool        d_sb_valid;		// flag indicating if SB data is valid
	bool        d_pi_valid;		// flag indicating if PI data is valid
	bool        d_dhdr_valid;	// flag indicating if DHDR data is valid
	uint64_t    d_type;
	uint8_t     d_cc;
	int         d_msgq_id;
	int         d_debug;
	int         d_chan;
	int         d_slot_mask;
	int			d_src_id;
	int			d_dst_id;
	std::pair<bool,long>		d_terminated;
    log_ts&      logts;
	CBPTC19696  bptc;
	CDMRTrellis trellis;
	ezpwd::RS<255,252> rs12;	// Reed-Solomon(12,9) object for Link Control decode
	gr::msg_queue::sptr d_msg_queue;

	void send_msg(const std::string& m_buf, const int m_type);
	bool decode_slot_type();
	bool decode_csbk(uint8_t* csbk);
	bool decode_mbc_header(uint8_t* mbc);
	bool decode_mbc_continue(uint8_t* mbc);
	bool decode_pdp_header(uint8_t* dhdr);
	bool decode_pdp_12data(uint8_t* pdp);
	bool decode_pdp_34data(uint8_t* pdp);
	bool decode_vlch(uint8_t* vlch);
	bool decode_tlc(uint8_t* tlc);
	bool decode_lc(uint8_t* lc, int* rs_errs = NULL);
	bool decode_pinf(uint8_t* pinf);
	bool decode_emb();
	bool decode_embedded_lc();
	bool decode_embedded_sbrc(bool _pi);

	inline uint8_t  get_lc_pf()      { return d_lc_valid ? ((d_lc[0] & 0x80) >> 7) : 0; };
	inline uint8_t  get_lc_flco()    { return d_lc_valid ? (d_lc[0] & 0x3f) : 0; };
	inline uint8_t  get_lc_fid()     { return d_lc_valid ? d_lc[1] : 0; };
	inline uint8_t  get_lc_svcopt()  { return d_lc_valid ? d_lc[2] : 0; };
	inline uint32_t get_lc_dstaddr() { return d_lc_valid ? ((d_lc[3] << 16) + (d_lc[4] << 8) + d_lc[5]) : 0; };
	inline uint32_t get_lc_srcaddr() { return d_lc_valid ? ((d_lc[6] << 16) + (d_lc[7] << 8) + d_lc[8]) : 0; };

	inline uint8_t  get_pi_algid()   { return d_pi_valid ? d_pi[0] : 0; };
	inline uint8_t  get_pi_keyid()   { return d_pi_valid ? d_pi[2] : 0; };
	inline uint32_t get_pi_mi()      { return d_pi_valid ? ((d_pi[3] << 24) + (d_pi[4] << 16) + (d_pi[5] << 8) + d_pi[6]) : 0; };
	inline uint32_t get_pi_dstaddr() { return d_pi_valid ? ((d_pi[7] << 16) + (d_pi[8] << 8) + d_pi[9]) : 0; };

	inline uint8_t  get_rc()         { return d_rc_valid ? d_rc : 0; };
	inline uint8_t  get_sb()         { return d_sb_valid ? d_sb : 0; };

	inline uint8_t  get_slot_cc()    { return d_slot_type.size() ? ((d_slot_type[0] << 3) + (d_slot_type[1] << 2) + (d_slot_type[2] << 1) + d_slot_type[3]) : 0xf; };
	inline uint8_t  get_data_type()  { return d_slot_type.size() ? ((d_slot_type[4] << 3) + (d_slot_type[5] << 2) + (d_slot_type[6] << 1) + d_slot_type[7]) : 0x9; };

	inline uint8_t  get_dhdr_dpf()   { return d_dhdr_valid ? (d_dhdr[0] & 0xf) : 0; };
	inline uint8_t  get_dhdr_sap()   { return d_dhdr_valid ? (d_dhdr[1] >> 4) & 0xf : 0; };
	inline uint32_t get_dhdr_dst()   { return d_dhdr_valid ? ((d_dhdr[2] << 16) + (d_dhdr[3] << 8) + d_dhdr[4]) : 0; };
	inline uint32_t get_dhdr_src()   { return d_dhdr_valid ? ((d_dhdr[5] << 16) + (d_dhdr[6] << 8) + d_dhdr[7]) : 0; };

};

#endif /* INCLUDED_DMR_SLOT_H */
