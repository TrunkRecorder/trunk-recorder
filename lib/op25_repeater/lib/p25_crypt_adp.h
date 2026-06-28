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

#ifndef P25_CRYPT_ADP_H
#define P25_CRYPT_ADP_H

#include <gnuradio/msg_queue.h>
#include "log_ts.h"
#include "p25_crypt.h"
#include "p25_crypt_alg.h"

// ADP RC4 decryption algorithm
class p25_crypt_adp : public p25_crypt_alg
{
    private:
        protocol_type d_pr_type;
        uint8_t d_mi[9];
        uint8_t d_keystream[469];
        uint32_t d_position;

    public:
        virtual bool prepare(uint16_t keyid, protocol_type pr_type, uint8_t *MI);
        virtual bool process(packed_codeword& PCW, frame_type fr_type, int voice_subframe);

        p25_crypt_adp(log_ts& logger, int debug, int msgq_id);
        ~p25_crypt_adp();

        inline uint8_t key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
            if (algid != ALG_ADP_RC4)
                return 0;
            p25_crypt_alg::key(keyid, algid, key);
            return algid;
        }

        inline void adp_swap(uint8_t *S, uint32_t i, uint32_t j) {
            uint8_t temp = S[i];
            S[i] = S[j];
            S[j] = temp;
        }
};

#endif /* P25_CRYPT_ADP_H  */
