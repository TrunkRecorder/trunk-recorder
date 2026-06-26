/* -*- c++ -*- */
/*
 * Copyright 2025 Graham J. Norbury
 * Developed from code originally written by Ali Elmasry
 * and adapted for op25 by Joey Absi
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

#ifndef P25_CRYPT_DES_H
#define P25_CRYPT_DES_H

#include <gnuradio/msg_queue.h>
#include <string>
#include <vector>
#include "log_ts.h"
#include "p25_crypt.h"
#include "p25_crypt_alg.h"

// DES OFB decryption algorithm
class p25_crypt_des : public p25_crypt_alg
{
    private:
        protocol_type d_pr_type;
        uint8_t d_keystream[224];
        uint32_t d_position;

        // DES manipulation methods
        std::string hex2bin(std::string s);
        std::string bin2hex(std::string s);
        std::string permute(std::string k, int* arr, int n);
        std::string shift_left(std::string k, int shifts);
        std::string xor_(std::string a, std::string b);
        std::string encrypt(std::string pt, std::vector<std::string> rkb, std::vector<std::string> rk);
        std::string byteArray2string(uint8_t array[]);
        void        string2ByteArray(const std::string& s, uint8_t array[], int offset);

    public:
        virtual bool prepare(uint16_t keyid, protocol_type pr_type, uint8_t *MI);
        virtual bool process(packed_codeword& PCW, frame_type fr_type, int voice_subframe);

        p25_crypt_des(log_ts& logger, int debug, int msgq_id);
        ~p25_crypt_des();

        inline uint8_t key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
            if (algid != ALG_DES_OFB)
                return 0;
            p25_crypt_alg::key(keyid, algid, key);
            return algid;
        }
};

#endif /* P25_CRYPT_DES_H  */
