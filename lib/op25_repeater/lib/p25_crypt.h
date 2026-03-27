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

#ifndef P25_CRYPT_H
#define P25_CRYPT_H

#include <stdint.h>
#include <vector>

enum algid_type : uint8_t   {
    ALG_UNENCRYPTED   = 0x80,
    ALG_DES_OFB       = 0x81,
    ALG_AES_256       = 0x84,
    ALG_ADP_RC4       = 0xAA
};

enum frame_type       { FT_UNK = 0, FT_LDU1, FT_LDU2, FT_2V, FT_4V_0, FT_4V_1, FT_4V_2, FT_4V_3 };
enum protocol_type    { PT_UNK = 0, PT_P25_PHASE1, PT_P25_PHASE2 };

typedef std::vector<uint8_t> packed_codeword;

struct key_info {
    key_info() : algid(0), key() {}
    key_info(uint8_t a, const std::vector<uint8_t> &k) : algid(a), key(k) {}
    uint8_t algid;
    std::vector<uint8_t> key;
};

#endif /* P25_CRYPT_H  */
