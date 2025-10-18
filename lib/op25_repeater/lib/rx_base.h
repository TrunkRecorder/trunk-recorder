// P25 Decoder (C) Copyright 2020 Graham J. Norbury
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

#ifndef INCLUDED_RX_BASE_H
#define INCLUDED_RX_BASE_H

#include "log_ts.h"

namespace gr{
    namespace op25_repeater{

        class rx_base {
            public:
                virtual void rx_sym(const uint8_t sym) = 0;
                virtual void sync_reset(void) = 0;
                virtual void call_end(void) = 0;
                virtual void crypt_reset(void) = 0;
                virtual void crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) = 0;
                virtual void reset_timer(void) = 0;
                virtual void set_slot_mask(int mask) = 0;
                virtual void set_slot_key(int mask) = 0;
                virtual void set_nac(int nac) = 0;
                virtual void set_debug(int debug) = 0;
                virtual void set_xormask(const char* p) = 0;
                virtual int get_src_id(int slot) { return -1;};
                virtual int get_dst_id(int slot) { return -1;};
                virtual int get_cc(int slot) { return -1;};
	            virtual std::pair<bool,long> get_terminated(int slot) { return std::pair<bool,long>(false,0);};
                rx_base(const char * options, log_ts& logger, int debug, int msgq_id, gr::msg_queue::sptr queue) { };
                rx_base() {}; // default constructor called by derived classes
                virtual ~rx_base() {};
        };

    } // end namespace op25_repeater
} // end namespace gr
#endif // INCLUDED_RX_BASE_H
