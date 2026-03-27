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

#include <vector>
#include <string>

#include "p25_crypt.h"
#include "p25_crypt_algs.h"
#include "p25_crypt_adp.h"
#include "p25_crypt_des.h"
#include "p25_crypt_aes.h"

#include "op25_msg_types.h"

// constructor
p25_crypt_algs::p25_crypt_algs(log_ts& logger, int debug, int msgq_id) :
    logts(logger),
    d_debug(debug),
    d_msgq_id(msgq_id),
    d_alg_iter(d_algs.end()) {
}

// destructor
p25_crypt_algs::~p25_crypt_algs() {
    // clean up dynamically allocated decryption algorithm objects
    d_alg_iter = d_algs.begin();
    while (d_alg_iter != d_algs.end()) {
        delete d_alg_iter->second;
        d_algs.erase(d_alg_iter);
    }
}

// remove all stored keys
void p25_crypt_algs::reset(void) {
    for (auto& it : d_algs) {
        it.second->reset();
    }
}

// add or update a key
void p25_crypt_algs::key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
    if ((keyid == 0) || (algid == 0x80))
        return;

    // find appropriate decryption object based on algid
    d_alg_iter = d_algs.find(algid);
    if (d_alg_iter == d_algs.end()) {
        // TODO:
        // This really should be re-written as a self-registering factory class,
        // but for now you have to manually add a 'case' for each new algid to be supported
        switch (algid) {
            case ALG_DES_OFB:
                d_alg_iter = d_algs.insert({algid, new p25_crypt_des(logts, d_debug, d_msgq_id)}).first;
                break;
            case ALG_AES_256:
                d_alg_iter = d_algs.insert({algid, new p25_crypt_aes(logts, d_debug, d_msgq_id)}).first;
                break;
            case ALG_ADP_RC4:
                d_alg_iter = d_algs.insert({algid, new p25_crypt_adp(logts, d_debug, d_msgq_id)}).first;
                break;
        }
        if (d_alg_iter == d_algs.end()) {
            return; // d_algs insert failed
        }
    }
    // Register the key with the algorithm object
    if (d_alg_iter->second->key(keyid, algid, key) == algid) {
        if (d_debug >= 10) {
            fprintf(stderr, "%s p25_crypt_algs::key: loaded key keyId:%04x, algId:%02x\n", logts.get(d_msgq_id), keyid, algid);
        }
    }
}

// generic entry point to prepare for decryption
bool p25_crypt_algs::prepare(uint8_t algid, uint16_t keyid, protocol_type pr_type, uint8_t *MI) {
    d_alg_iter = d_algs.find(algid);
    if (d_alg_iter == d_algs.end()) {
        if (d_debug >= 10) {
            fprintf(stderr, "%s p25_crypt_algs::prepare: algid[0x%x] algorithm module not found\n", logts.get(d_msgq_id), algid);
        }
        return false;
    }
    return d_alg_iter->second->prepare(keyid, pr_type, MI);
}

// generic entry point to perform decryption
bool p25_crypt_algs::process(packed_codeword& PCW, frame_type fr_type, int voice_subframe) {
    if (d_alg_iter == d_algs.end()) {
        if (d_debug >= 10) {
            fprintf(stderr, "%s p25_crypt_algs::process: internal error (no algorithm module)\n", logts.get(d_msgq_id));
        }
        return false;
    }
    return d_alg_iter->second->process(PCW, fr_type, voice_subframe);
}

// P25 variant of LFSR routine
// Returns MSB before shift takes place
uint64_t p25_crypt_algs::step_p25_lfsr(uint64_t &lfsr) {
    // Polynomial is C(x) = x^64 + x^62 + x^46 + x^38 + x^27 + x^15 + 1
    uint64_t ov_bit = (lfsr >> 63) & 0x1;
    uint64_t fb_bit = ((lfsr >> 63) ^ (lfsr >> 61) ^ (lfsr >> 45) ^ (lfsr >> 37) ^ (lfsr >> 26) ^ (lfsr >> 14)) & 0x1;
    lfsr =  (lfsr << 1) | (fb_bit);
    return ov_bit;
}

// Use P25 LFSR to replace current MI with the next one in the sequence
void p25_crypt_algs::cycle_p25_mi(uint8_t *MI) {
    uint64_t lfsr = 0;
    for (int i=0; i<8; i++) {
        lfsr = (lfsr << 8) + MI[i];
    }

    for(uint8_t cnt=0; cnt<64; cnt++) {
        step_p25_lfsr(lfsr);
    }

    for (int i=7; i>=0; i--) {
        MI[i] = lfsr & 0xFF;
        lfsr >>= 8;
    }
    MI[8] = 0;
}

// Expand MI to 128 bits for use as an Initialization Vector
void p25_crypt_algs::expand_mi_to_128(uint8_t *MI, uint8_t *IV) {
    // copy first 64 bits of MI into LFSR
    uint64_t lfsr = 0;
    for (int i=0; i<8; i++) {
        lfsr = (lfsr << 8) + MI[i];
    }

    // use LFSR routine to compute the expansion
    uint64_t overflow = 0;
    for (int i = 0; i < 64; i++) {
        overflow = (overflow << 1) | step_p25_lfsr(lfsr);
    }

    // copy expansion and lfsr to IV
    for (int i=7; i>=0; i--) {
        IV[i] = overflow & 0xFF;
        overflow >>= 8;
    }
    for (int i=15; i>=8; i--) {
        IV[i] = lfsr & 0xFF;
        lfsr >>= 8;
    }
}
