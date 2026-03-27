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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "p25_crypt_adp.h"

// constructor
p25_crypt_adp::p25_crypt_adp(log_ts& logger, int debug, int msgq_id) :
    p25_crypt_alg(logger, debug, msgq_id) {

    fprintf(stderr, "%s p25_crypt_adp::p25_crypt_adp: loading ADP RC4 module\n", logts.get(d_msgq_id));
}

// destructor
p25_crypt_adp::~p25_crypt_adp() {

}

// prepare routine entry point
bool
p25_crypt_adp::prepare(uint16_t keyid, protocol_type pr_type, uint8_t *MI) {
    d_pr_type = pr_type;
    memcpy(d_mi, MI, sizeof(d_mi));

    d_key_iter = d_keys.find(keyid);
    if (d_key_iter == d_keys.end()) {
        if (d_debug >= 10) {
            fprintf(stderr, "%s p25_crypt_adp::prepare: keyid[0x%x] not found\n", logts.get(d_msgq_id), keyid);
        }
        return false;
    }
    if (d_debug >= 10) {
        fprintf(stderr, "%s p25_crypt_adp::prepare: keyid[0x%x] found\n", logts.get(d_msgq_id), keyid);
    }
    d_position = 0;
    d_pr_type = pr_type;

    // Find key value from keyid and set up to create keystream
    uint8_t adp_key[13], S[256], K[256];
    uint32_t i, j, k;
    std::vector<uint8_t>::const_iterator kval_iter = d_key_iter->second.key.begin();
    for (i = 0; i < (uint32_t)std::max(5-(int)(d_key_iter->second.key.size()), 0); i++) {
        adp_key[i] = 0;             // pad with leading 0 if supplied key too short
    }
    for ( ; i < 5; i++) {
        adp_key[i] = *kval_iter++;  // copy up to 5 bytes into key array
    }

    j = 0;
    for (i = 5; i < 13; ++i) {
        adp_key[i] = d_mi[i - 5];   // append MI bytes
    }

    for (i = 0; i < 256; ++i) {
        K[i] = adp_key[i % 13];
    }

    for (i = 0; i < 256; ++i) {
        S[i] = i;
    }

    for (i = 0; i < 256; ++i) {
        j = (j + S[i] + K[i]) & 0xFF;
        adp_swap(S, i, j);
    }

    i = j = 0;

    for (k = 0; k < 469; ++k) {
        i = (i + 1) & 0xFF;
        j = (j + S[i]) & 0xFF;
        adp_swap(S, i, j);
        d_keystream[k] = S[(S[i] + S[j]) & 0xFF];
    }

    return true;
}

// process routine entry point
bool
p25_crypt_adp::process(packed_codeword& PCW, frame_type fr_type, int voice_subframe) {
    if (d_key_iter == d_keys.end())
        return false;

    bool rc = true;
    size_t offset = 256;
    switch (fr_type) {
        case FT_LDU1:
            offset = 0;
            break;
        case FT_LDU2:
            offset = 101;
            break;
        case FT_4V_0:
            offset += 7 * voice_subframe;
            break;
        case FT_4V_1:
            offset += 7 * (voice_subframe + 4);
            break;
        case FT_4V_2:
            offset += 7 * (voice_subframe + 8);
            break;
        case FT_4V_3:
            offset += 7 * (voice_subframe + 12);
            break;
        case FT_2V:
            offset += 7 * (voice_subframe + 16);
            break;
        default:
            rc = false;
            break;
    }
    if (d_pr_type == PT_P25_PHASE1) {
        //FDMA
        offset += (d_position * 11) + 267 + ((d_position < 8) ? 0 : 2); // voice only; skip LCW and LSD
        d_position = (d_position + 1) % 9;
        for (int j = 0; j < 11; ++j) {
            PCW[j] = d_keystream[j + offset] ^ PCW[j];
        }
    } else if (d_pr_type == PT_P25_PHASE2) {
        //TDMA
        for (int j = 0; j < 7; ++j) {
            PCW[j] = d_keystream[j + offset] ^ PCW[j];
        }
        PCW[6] &= 0x80; // mask everything except the MSB of the final codeword
    }

    return rc;
}
