/* -*- c++ -*- */
/*
 * Copyright 2025 Graham J. Norbury
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

#ifndef P25_CRYPT_ALG_H
#define P25_CRYPT_ALG_H

#include <gnuradio/msg_queue.h>
#include <unordered_map>

#include "p25_crypt.h"
#include "log_ts.h"

// Base class for implementation of individual decryption algorithms
class p25_crypt_alg
{
    protected:
        log_ts& logts;
        int d_debug;
        int d_msgq_id;
        std::unordered_map<uint16_t, key_info> d_keys;
        std::unordered_map<uint16_t, key_info>::const_iterator d_key_iter;

    public:
        virtual bool prepare(uint16_t keyid, protocol_type pr_type, uint8_t *MI) = 0;
        virtual bool process(packed_codeword& PCW, frame_type fr_type, int voice_subframe) = 0;

        inline p25_crypt_alg(log_ts& logger, int debug, int msgq_id) : logts(logger), d_debug(debug), d_msgq_id(msgq_id) { }
        inline virtual ~p25_crypt_alg() { }

        inline virtual void reset(void) { d_keys.clear(); }
        inline virtual void set_debug(int debug) { d_debug = debug; }
        inline virtual uint8_t key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
            if ((keyid == 0) || (algid == ALG_UNENCRYPTED))
                return 0;
            d_keys[keyid] = key_info(algid, key);
            return algid;
        }

};

#endif /* P25_CRYPT_ALG_H  */
