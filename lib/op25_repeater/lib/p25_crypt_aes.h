/* -*- c++ -*- */
/*
 * Copyright 2025 Graham J. Norbury
 * Adapted from aes.h/aes.c from github.com:/lwvmobile/tinier-aes
 * which in turn was derived from https://github.com/kokke/tiny-AES-c
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

#ifndef P25_CRYPT_AES_H
#define P25_CRYPT_AES_H

#include <gnuradio/msg_queue.h>
#include <string>
#include <vector>
#include "log_ts.h"
#include "p25_crypt.h"
#include "p25_crypt_alg.h"

#define AES_BLOCKLEN 16

// DES OFB decryption algorithm
class p25_crypt_aes : public p25_crypt_alg
{
    private:
        protocol_type d_pr_type;
        uint8_t d_keystream[240]; // 16 bytes per block x 15 blocks (FDMA), or 9 blocks (TDMA)
        uint32_t d_position;

        unsigned Nb = 4;
        unsigned Nk = 8;
        unsigned Nr = 14;

    public:
        virtual bool prepare(uint16_t keyid, protocol_type pr_type, uint8_t *MI);
        virtual bool process(packed_codeword& PCW, frame_type fr_type, int voice_subframe);

        p25_crypt_aes(log_ts& logger, int debug, int msgq_id);
        ~p25_crypt_aes();

        inline uint8_t key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
            if (algid != ALG_AES_256)
                return 0;
            p25_crypt_alg::key(keyid, algid, key);
            return algid;
        }

    private:
        // everything that follows is derived from tinier-aes
        struct AES_ctx
        {
            uint8_t RoundKey[240];
            uint8_t Iv[16];
        };

        typedef uint8_t state_t[4][4];

        static const uint8_t sbox[256];
        static const uint8_t rsbox[256];
        static const uint8_t Rcon[11];

        // bit and byte utility prototyes
        uint64_t convert_bits_into_output(uint8_t * input, int len);
        void pack_bit_array_into_byte_array (uint8_t * input, uint8_t * output, int len);
        void unpack_byte_array_into_bit_array (uint8_t * input, uint8_t * output, int len);
        uint8_t xtime(uint8_t x);
        void XorWithIv(uint8_t* buf, const uint8_t* Iv);

        // aes function prototypes, convenience wrapper functions
        void aes_ctr_bitwise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * payload, int type);
        void aes_ctr_bytewise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * payload, int type);
        void aes_ofb_keystream_output (uint8_t * iv, uint8_t * key, uint8_t * output, int type, int nblocks);
        void aes_cfb_bytewise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * in, uint8_t * out, int type, int nblocks, int de);
        void aes_cbc_bytewise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * in, uint8_t * out, int type, int nblocks, int de);
        void aes_cbc_mac_generator (uint8_t * key, uint8_t * in, uint8_t * out, int type, int nblocks);
        void aes_ecb_bytewise_payload_crypt (uint8_t * input, uint8_t * key, uint8_t * output, int type, int de);

        // internal function prototypes
        void Cipher(state_t* state, const uint8_t* RoundKey);
        void InvCipher(state_t* state, const uint8_t* RoundKey);
        void KeyExpansion(uint8_t* RoundKey, const uint8_t* Key);
        void AddRoundKey(uint8_t round, state_t* state, const uint8_t* RoundKey);
        void SubBytes(state_t* state);
        void ShiftRows(state_t* state);
        void MixColumns(state_t* state);
        void InvMixColumns(state_t* state);
        void InvSubBytes(state_t* state);
        void InvShiftRows(state_t* state);
        void AES_init_ctx(struct AES_ctx* ctx, const uint8_t* key);
        void AES_init_ctx_iv(struct AES_ctx* ctx, const uint8_t* key, const uint8_t* iv);
        void AES_ctx_set_iv(struct AES_ctx* ctx, const uint8_t* iv);
        void AES_ECB_encrypt(const struct AES_ctx* ctx, uint8_t* buf);
        void AES_ECB_decrypt(const struct AES_ctx* ctx, uint8_t* buf);
        void AES_CBC_encrypt_buffer(struct AES_ctx* ctx, uint8_t* buf, size_t length);
        void AES_CBC_decrypt_buffer(struct AES_ctx* ctx, uint8_t* buf, size_t length);
        void AES_CTR_xcrypt_buffer(struct AES_ctx* ctx, uint8_t* buf, size_t length);

};

#endif /* P25_CRYPT_AES_H  */
