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

#ifndef INCLUDED_DMR_CAI_H
#define INCLUDED_DMR_CAI_H

#include <stdint.h>
#include <vector>
#include <gnuradio/msg_queue.h>

#include "dmr_slot.h"
#include "log_ts.h"

typedef std::vector<bool> bit_vector;

static const unsigned int slot_ids[] = {0, 1, 0, 0, 1, 1, 0, 1};

class dmr_cai {
public:
	dmr_cai(log_ts& logger, int debug, int msgq_id, gr::msg_queue::sptr queue);
	~dmr_cai();
	bool load_frame(const uint8_t fr_sym[], bool& unmute);
    void set_debug(int debug);
	inline int chan() { return d_chan; };
	inline void set_slot_mask(int mask);
	int get_src_id(int slot);
	int get_dst_id(int slot);
	int get_cc(int slot);
	std::pair<bool,long> get_terminated(int slot);

private:
	static const int FRAME_SIZE = 288; // frame length in bits
	static const int CACH       =   0; // position of CACH in frame

	uint8_t d_frame[FRAME_SIZE];       // array of bits comprising the current frame
	dmr_slot d_slot[2];
	bit_vector d_cach_sig;
	int d_slot_mask;
	int d_chan;
	int d_shift_reg;
	int d_debug;
	int d_msgq_id;
	gr::msg_queue::sptr d_msg_queue;
	log_ts& logts;

	void extract_cach_fragment();
	bool decode_shortLC();
	void send_msg(const std::string& m_buf, const int m_type);
};

#endif /* INCLUDED_DMR_CAI_H */
