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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string>
#include <vector>
#include <bitset>
#include <unordered_map>
#include <sstream>
#include <iomanip>

#include "p25_crypt_aes.h"
#include "p25_crypt_algs.h"

const uint8_t p25_crypt_aes::sbox[256] = {
    //0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16 };

const uint8_t p25_crypt_aes::rsbox[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d };

const uint8_t p25_crypt_aes::Rcon[11] = {
    0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36 };

#define getSBoxValue(num) (p25_crypt_aes::sbox[(num)])
#define getSBoxInvert(num) (p25_crypt_aes::rsbox[(num)])
#define Multiply(x, y)                                \
    (  ((y & 1) * x) ^                              \
       ((y>>1 & 1) * p25_crypt_aes::xtime(x)) ^                       \
       ((y>>2 & 1) * p25_crypt_aes::xtime(p25_crypt_aes::xtime(x))) ^                \
       ((y>>3 & 1) * p25_crypt_aes::xtime(p25_crypt_aes::xtime(p25_crypt_aes::xtime(x)))) ^         \
       ((y>>4 & 1) * p25_crypt_aes::xtime(p25_crypt_aes::xtime(p25_crypt_aes::xtime(p25_crypt_aes::xtime(x))))))   \

void
p25_crypt_aes::KeyExpansion(uint8_t* RoundKey, const uint8_t* Key)
{
    unsigned i, j, k;
    uint8_t tempa[4]; // Used for the column/row operations

    // The first round key is the key itself.
    for (i = 0; i < Nk; ++i)
    {
        RoundKey[(i * 4) + 0] = Key[(i * 4) + 0];
        RoundKey[(i * 4) + 1] = Key[(i * 4) + 1];
        RoundKey[(i * 4) + 2] = Key[(i * 4) + 2];
        RoundKey[(i * 4) + 3] = Key[(i * 4) + 3];
    }

    // All other round keys are found from the previous round keys.
    for (i = Nk; i < Nb * (Nr + 1); ++i)
    {
        {
            k = (i - 1) * 4;
            tempa[0]=RoundKey[k + 0];
            tempa[1]=RoundKey[k + 1];
            tempa[2]=RoundKey[k + 2];
            tempa[3]=RoundKey[k + 3];
        }

        if (i % Nk == 0)
        {
            // This function shifts the 4 bytes in a word to the left once.
            // [a0,a1,a2,a3] becomes [a1,a2,a3,a0]

            // Function RotWord()
            {
                const uint8_t u8tmp = tempa[0];
                tempa[0] = tempa[1];
                tempa[1] = tempa[2];
                tempa[2] = tempa[3];
                tempa[3] = u8tmp;
            }

            // SubWord() is a function that takes a four-byte input word and
            // applies the S-box to each of the four bytes to produce an output word.

            // Function Subword()
            {
                tempa[0] = getSBoxValue(tempa[0]);
                tempa[1] = getSBoxValue(tempa[1]);
                tempa[2] = getSBoxValue(tempa[2]);
                tempa[3] = getSBoxValue(tempa[3]);
            }

            tempa[0] = tempa[0] ^ Rcon[i/Nk];
        }

        if (Nk == 8) //only run if using AES256 (Nk == 8)
        {
            if (i % Nk == 4)
            {
                // Function Subword()
                {
                    tempa[0] = getSBoxValue(tempa[0]);
                    tempa[1] = getSBoxValue(tempa[1]);
                    tempa[2] = getSBoxValue(tempa[2]);
                    tempa[3] = getSBoxValue(tempa[3]);
                }
            }
        }

        j = i * 4; k=(i - Nk) * 4;
        RoundKey[j + 0] = RoundKey[k + 0] ^ tempa[0];
        RoundKey[j + 1] = RoundKey[k + 1] ^ tempa[1];
        RoundKey[j + 2] = RoundKey[k + 2] ^ tempa[2];
        RoundKey[j + 3] = RoundKey[k + 3] ^ tempa[3];
    }
}

//input bit array, return output as up to a 64-bit value
uint64_t
p25_crypt_aes::convert_bits_into_output(uint8_t * input, int len)
{
    int i;
    uint64_t output = 0;
    for(i = 0; i < len; i++)
    {
        output <<= 1;
        output |= (uint64_t)(input[i] & 1);
    }
    return output;
}

//take x amount of bits and pack into len amount of bytes (symmetrical)
void
p25_crypt_aes::pack_bit_array_into_byte_array (uint8_t * input, uint8_t * output, int len)
{
    int i;
    for (i = 0; i < len; i++)
        output[i] = (uint8_t)convert_bits_into_output(&input[i*8], 8);
}

//take len amount of bytes and unpack back into a bit array
void
p25_crypt_aes::unpack_byte_array_into_bit_array (uint8_t * input, uint8_t * output, int len)
{
    int i = 0, k = 0;
    for (i = 0; i < len; i++)
    {
        output[k++] = (input[i] >> 7) & 1;
        output[k++] = (input[i] >> 6) & 1;
        output[k++] = (input[i] >> 5) & 1;
        output[k++] = (input[i] >> 4) & 1;
        output[k++] = (input[i] >> 3) & 1;
        output[k++] = (input[i] >> 2) & 1;
        output[k++] = (input[i] >> 1) & 1;
        output[k++] = (input[i] >> 0) & 1;
    }
}

void
p25_crypt_aes::AES_init_ctx(struct AES_ctx* ctx, const uint8_t* key)
{
    KeyExpansion(ctx->RoundKey, key);
}

void
p25_crypt_aes::AES_init_ctx_iv(struct AES_ctx* ctx, const uint8_t* key, const uint8_t* iv)
{
    KeyExpansion(ctx->RoundKey, key);
    memcpy (ctx->Iv, iv, AES_BLOCKLEN);
}

void
p25_crypt_aes::AES_ctx_set_iv(struct AES_ctx* ctx, const uint8_t* iv)
{
    memcpy (ctx->Iv, iv, AES_BLOCKLEN);
}

void
p25_crypt_aes::AddRoundKey(uint8_t round, state_t* state, const uint8_t* RoundKey)
{
    uint8_t i,j;
    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            (*state)[i][j] ^= RoundKey[(round * Nb * 4) + (i * Nb) + j];
        }
    }
}

void
p25_crypt_aes::SubBytes(state_t* state)
{
    uint8_t i, j;
    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            (*state)[j][i] = getSBoxValue((*state)[j][i]);
        }
    }
}

void
p25_crypt_aes::ShiftRows(state_t* state)
{
    uint8_t temp;

    // Rotate first row 1 columns to left
    temp           = (*state)[0][1];
    (*state)[0][1] = (*state)[1][1];
    (*state)[1][1] = (*state)[2][1];
    (*state)[2][1] = (*state)[3][1];
    (*state)[3][1] = temp;

    // Rotate second row 2 columns to left
    temp           = (*state)[0][2];
    (*state)[0][2] = (*state)[2][2];
    (*state)[2][2] = temp;

    temp           = (*state)[1][2];
    (*state)[1][2] = (*state)[3][2];
    (*state)[3][2] = temp;

    // Rotate third row 3 columns to left
    temp           = (*state)[0][3];
    (*state)[0][3] = (*state)[3][3];
    (*state)[3][3] = (*state)[2][3];
    (*state)[2][3] = (*state)[1][3];
    (*state)[1][3] = temp;
}

uint8_t
p25_crypt_aes::xtime(uint8_t x)
{
    return ((x<<1) ^ (((x>>7) & 1) * 0x1b));
}

void
p25_crypt_aes::MixColumns(state_t* state)
{
    uint8_t i;
    uint8_t Tmp, Tm, t;
    for (i = 0; i < 4; ++i)
    {
        t   = (*state)[i][0];
        Tmp = (*state)[i][0] ^ (*state)[i][1] ^ (*state)[i][2] ^ (*state)[i][3] ;
        Tm  = (*state)[i][0] ^ (*state)[i][1] ; Tm = xtime(Tm);  (*state)[i][0] ^= Tm ^ Tmp ;
        Tm  = (*state)[i][1] ^ (*state)[i][2] ; Tm = xtime(Tm);  (*state)[i][1] ^= Tm ^ Tmp ;
        Tm  = (*state)[i][2] ^ (*state)[i][3] ; Tm = xtime(Tm);  (*state)[i][2] ^= Tm ^ Tmp ;
        Tm  = (*state)[i][3] ^ t ;              Tm = xtime(Tm);  (*state)[i][3] ^= Tm ^ Tmp ;
    }
}

void
p25_crypt_aes::InvMixColumns(state_t* state)
{
    int i;
    uint8_t a, b, c, d;
    for (i = 0; i < 4; ++i)
    {
        a = (*state)[i][0];
        b = (*state)[i][1];
        c = (*state)[i][2];
        d = (*state)[i][3];

        (*state)[i][0] = Multiply(a, 0x0e) ^ Multiply(b, 0x0b) ^ Multiply(c, 0x0d) ^ Multiply(d, 0x09);
        (*state)[i][1] = Multiply(a, 0x09) ^ Multiply(b, 0x0e) ^ Multiply(c, 0x0b) ^ Multiply(d, 0x0d);
        (*state)[i][2] = Multiply(a, 0x0d) ^ Multiply(b, 0x09) ^ Multiply(c, 0x0e) ^ Multiply(d, 0x0b);
        (*state)[i][3] = Multiply(a, 0x0b) ^ Multiply(b, 0x0d) ^ Multiply(c, 0x09) ^ Multiply(d, 0x0e);
    }
}

void
p25_crypt_aes::InvSubBytes(state_t* state)
{
    uint8_t i, j;
    for (i = 0; i < 4; ++i)
    {
        for (j = 0; j < 4; ++j)
        {
            (*state)[j][i] = getSBoxInvert((*state)[j][i]);
        }
    }
}

void
p25_crypt_aes::InvShiftRows(state_t* state)
{
    uint8_t temp;

    // Rotate first row 1 columns to right
    temp = (*state)[3][1];
    (*state)[3][1] = (*state)[2][1];
    (*state)[2][1] = (*state)[1][1];
    (*state)[1][1] = (*state)[0][1];
    (*state)[0][1] = temp;

    // Rotate second row 2 columns to right
    temp = (*state)[0][2];
    (*state)[0][2] = (*state)[2][2];
    (*state)[2][2] = temp;

    temp = (*state)[1][2];
    (*state)[1][2] = (*state)[3][2];
    (*state)[3][2] = temp;

    // Rotate third row 3 columns to right
    temp = (*state)[0][3];
    (*state)[0][3] = (*state)[1][3];
    (*state)[1][3] = (*state)[2][3];
    (*state)[2][3] = (*state)[3][3];
    (*state)[3][3] = temp;
}

// Cipher is the main function that encrypts the PlainText,
// or produces a keystream, depending on application.
void
p25_crypt_aes::Cipher(state_t* state, const uint8_t* RoundKey)
{
    uint8_t round = 0;

    // Add the First round key to the state before starting the rounds.
    AddRoundKey(0, state, RoundKey);

    // There will be Nr rounds.
    // The first Nr-1 rounds are identical.
    // These Nr rounds are executed in the loop below.
    // Last one without MixColumns()
    for (round = 1; ; ++round)
    {
        SubBytes(state);
        ShiftRows(state);
        if (round == Nr) {
            break;
        }
        MixColumns(state);
        AddRoundKey(round, state, RoundKey);
    }
    // Add round key to last round
    AddRoundKey(Nr, state, RoundKey);
}

void
p25_crypt_aes::InvCipher(state_t* state, const uint8_t* RoundKey)
{
    uint8_t round = 0;

    // Add the First round key to the state before starting the rounds.
    AddRoundKey(Nr, state, RoundKey);

    // There will be Nr rounds.
    // The first Nr-1 rounds are identical.
    // These Nr rounds are executed in the loop below.
    // Last one without InvMixColumn()
    for (round = (Nr - 1); ; --round)
    {
        InvShiftRows(state);
        InvSubBytes(state);
        AddRoundKey(round, state, RoundKey);
        if (round == 0) {
            break;
        }
        InvMixColumns(state);
    }
}

void
p25_crypt_aes::AES_ECB_encrypt(const struct AES_ctx* ctx, uint8_t* buf)
{
    // The next function call encrypts the PlainText with the Key using AES algorithm.
    Cipher((state_t*)buf, ctx->RoundKey);
}

void
p25_crypt_aes::AES_ECB_decrypt(const struct AES_ctx* ctx, uint8_t* buf)
{
    // The next function call decrypts the PlainText with the Key using AES algorithm.
    InvCipher((state_t*)buf, ctx->RoundKey);
}

void
p25_crypt_aes::XorWithIv(uint8_t* buf, const uint8_t* Iv)
{
    uint8_t i;
    for (i = 0; i < AES_BLOCKLEN; ++i)
    {
        buf[i] ^= Iv[i];
    }
}

void
p25_crypt_aes::AES_CBC_encrypt_buffer(struct AES_ctx *ctx, uint8_t* buf, size_t length)
{
    size_t i;
    uint8_t *Iv = ctx->Iv;
    for (i = 0; i < length; i += AES_BLOCKLEN)
    {
        XorWithIv(buf, Iv);
        Cipher((state_t*)buf, ctx->RoundKey);
        Iv = buf;
        buf += AES_BLOCKLEN;
    }
    /* store Iv in ctx for next call */
    memcpy(ctx->Iv, Iv, AES_BLOCKLEN);
}

void
p25_crypt_aes::AES_CBC_decrypt_buffer(struct AES_ctx* ctx, uint8_t* buf, size_t length)
{
    size_t i;
    uint8_t storeNextIv[AES_BLOCKLEN];
    for (i = 0; i < length; i += AES_BLOCKLEN)
    {
        memcpy(storeNextIv, buf, AES_BLOCKLEN);
        InvCipher((state_t*)buf, ctx->RoundKey);
        XorWithIv(buf, ctx->Iv);
        memcpy(ctx->Iv, storeNextIv, AES_BLOCKLEN);
        buf += AES_BLOCKLEN;
    }

}

/* Symmetrical operation: same function for encrypting as for decrypting. Note any IV/nonce should never be reused with the same key */
void
p25_crypt_aes::AES_CTR_xcrypt_buffer(struct AES_ctx* ctx, uint8_t* buf, size_t length)
{
    uint8_t buffer[AES_BLOCKLEN];

    size_t i;
    int bi;
    for (i = 0, bi = AES_BLOCKLEN; i < length; ++i, ++bi)
    {
        if (bi == AES_BLOCKLEN) /* we need to regen xor compliment in buffer */
        {
            memcpy(buffer, ctx->Iv, AES_BLOCKLEN);
            Cipher((state_t*)buffer,ctx->RoundKey);

            /* Increment Iv and handle overflow */
            for (bi = (AES_BLOCKLEN - 1); bi >= 0; --bi)
            {
                /* inc will overflow */
                if (ctx->Iv[bi] == 255)
                {
                    ctx->Iv[bi] = 0;
                    continue;
                }
                ctx->Iv[bi] += 1;
                break;
            }
            bi = 0;
        }
        buf[i] = (buf[i] ^ buffer[bi]);
    }
}

//byte-wise output of AES OFB Keystream
//input iv is a 16-byte uint8_t array of initialization vector
//input key is up to 32-byte uint8_t array of key value
//input type is the type/key len of AES required (0-128, 1-192, 2-256)
//input nblocks is the number of rounds of 16-byte keystream output blocks requried
//output is a uint8_t bytewise array, each round filled with 16-bytes from aes keystream output
void
p25_crypt_aes::aes_ofb_keystream_output (uint8_t * iv, uint8_t * key, uint8_t * output, int type, int nblocks)
{
    int i;
    uint8_t input_register[16]; //OFB Input Register
    memset (input_register, 0, sizeof(input_register));

    //Set values specific to type (128/192/256)
    if (type == 0) //128
    {
        Nb = 4;
        Nk = 4;
        Nr = 10;
    }
    else if (type == 1) //192
    {
        Nb = 4;
        Nk = 6;
        Nr = 12;
    }
    else //if (type == 2) //256
    {
        Nb = 4;
        Nk = 8;
        Nr = 14;
    }

    struct AES_ctx ctx;

    //load first round of input_register with received IV (OFB First Input Register)
    memcpy (input_register, iv, 16*sizeof(uint8_t) );

    //initialize the key variable for the Cipher function
    memset (ctx.RoundKey, 0, 240*sizeof(uint8_t));
    KeyExpansion(ctx.RoundKey, key);

    //execute the cipher function, and copy ciphered input_register to output for required number of rounds
    for (i = 0; i < nblocks; i++)
    {
        Cipher((state_t*)input_register, ctx.RoundKey); //input_register is returned as output, and is put back in as object feedback
        memcpy (output+(i*16), input_register, 16*sizeof(uint8_t) ); //copy ciphered input_register to output
    }
}

//byte-wise AES CFB (Cipher Feedback) Payload operating in 128-bit block mode
//input in is a uint8_t bytewise array,is the input to be ciphered (encrypted or decrypted)
//input iv is a 16-byte uint8_t array of initialization vector
//input key is up to 32-byte uint8_t array of key value
//input type is the type/key len of AES required (0-128, 1-192, 2-256)
//input nblocks is the number of rounds of 16-byte payload blocks requried (last block will need padding if not flush)
//output out is a uint8_t bytewise array, each round filled with 16-bytes of cfb ciphered output (encrypted or decrypted)
//de is a bit-flag signalling to run Cipher (encrypt) on 1, or InvCipher (decrypt) on 0
void
p25_crypt_aes::aes_cfb_bytewise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * in, uint8_t * out, int type, int nblocks, int de)
{
    int i, j;
    uint8_t input_register[16]; //Input Register
    memset (input_register, 0, sizeof(input_register));

    //Set values specific to type (128/192/256)
    if (type == 0) //128
    {
        Nb = 4;
        Nk = 4;
        Nr = 10;
    }
    else if (type == 1) //192
    {
        Nb = 4;
        Nk = 6;
        Nr = 12;
    }
    else //if (type == 2) //256
    {
        Nb = 4;
        Nk = 8;
        Nr = 14;
    }

    struct AES_ctx ctx;

    //load first round of input_register with received IV (CFB First Input Register)
    memcpy (input_register, iv, 16*sizeof(uint8_t) );

    //initialize the key variable for the Cipher function
    memset (ctx.RoundKey, 0, 240*sizeof(uint8_t));
    KeyExpansion(ctx.RoundKey, key);

    //execute the cipher function, and copy ciphered input_register to output for required number of rounds
    for (i = 0; i < nblocks; i++)
    {
        //the cipher is always run in the foward, or encryption mode
        Cipher((state_t*)input_register, ctx.RoundKey);

        //xor the current input 'in' to the current state of the input_register for cipher feedback
        for (j = 0; j < 16; j++)
            input_register[j] ^= in[j+(i*16)];

        //copy ciphered/xor'd input_register to output 'out'
        memcpy (out+(i*16), input_register, 16*sizeof(uint8_t) );

        //if running in decryption mode, we feed in the next round of input
        if (!de)
            memcpy(input_register, in+(i*16), 16*sizeof(uint8_t));
    }
}

//byte-wise AES CBC (Cipher Block Chaining) //Yeah, I know this already exists, but wanted a custom convenience wrapper version
//input in is a uint8_t bytewise array,is the input to be ciphered (encrypted or decrypted)
//input iv is a 16-byte uint8_t array of initialization vector
//input key is up to 32-byte uint8_t array of key value
//input type is the type/key len of AES required (0-128, 1-192, 2-256)
//input nblocks is the number of rounds of 16-byte payload blocks requried (last block will need padding if not flush)
//output out is a uint8_t bytewise array, each round filled with 16-bytes of cfb ciphered output (encrypted or decrypted)
//de is a bit-flag signalling to run Cipher (encrypt) on 1, or InvCipher (decrypt) on 0
void
p25_crypt_aes::aes_cbc_bytewise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * in, uint8_t * out, int type, int nblocks, int de)
{
    int i, j;
    uint8_t input_register[16]; //Input Register
    memset (input_register, 0, sizeof(input_register));

    //Set values specific to type (128/192/256)
    if (type == 0) //128
    {
        Nb = 4;
        Nk = 4;
        Nr = 10;
    }
    else if (type == 1) //192
    {
        Nb = 4;
        Nk = 6;
        Nr = 12;
    }
    else //if (type == 2) //256
    {
        Nb = 4;
        Nk = 8;
        Nr = 14;
    }

    struct AES_ctx ctx;

    //load first round of input_register accordingly
    if (de)
        memcpy (input_register, iv, 16*sizeof(uint8_t) );
    else memcpy (input_register, in, 16*sizeof(uint8_t) );

    //initialize the key variable for the Cipher function
    memset (ctx.RoundKey, 0, 240*sizeof(uint8_t));
    KeyExpansion(ctx.RoundKey, key);

    //
    for (i = 0; i < nblocks; i++)
    {
        //run encryption or decryption depending on de value
        if (de) //encrypt
        {
            //xor the current input 'in' pt to the current state of the input_register for cbc feedback
            for (j = 0; j < 16; j++)
                input_register[j] ^= in[j+(i*16)];

            Cipher((state_t*)input_register, ctx.RoundKey);

            //copy ciphered input_register to output 'out'
            memcpy (out+(i*16), input_register, 16*sizeof(uint8_t) );

        }

        else   //decrypt
        {
            InvCipher((state_t*)input_register, ctx.RoundKey);

            //copy ciphered input_register to output 'out'
            memcpy (out+(i*16), input_register, 16*sizeof(uint8_t) );

            //xor the current output by IV, or by last received CT
            if (i == 0)
            {
                for (j = 0; j < 16; j++)
                    out[j] ^= iv[j];
            }
            else
            {
                for (j = 0; j < 16; j++)
                    out[j+(i*16)] ^= in[j+((i-1)*16)];
            }

            //copy in next segment for input_register (if not last)
            if (i < nblocks)
                memcpy(input_register, in+((i+1)*16), 16*sizeof(uint8_t) );
        }
    }
}

//byte-wise AES CBC_MAC (Cipher Block Chaining Message Authentication) //This is slightly different than above, no IV is present,
//but if iv is desireable, it will need to be pre-XOR'd with the first plaintext input block by the calling function
//input in is a uint8_t bytewise array, is the input to be ciphered.
//input key is up to 32-byte uint8_t array of key value
//input type is the type/key len of AES required (0-128, 1-192, 2-256)
//input nblocks is the number of rounds of 16-byte payload blocks requried (last block will need padding if not flush)
//output out is a uint8_t bytewise array, with only the final round output as the MAC octets
//NOTE: When doing a cbc_mac, you should only run it in the forward (encryption) mode to get the mac bytes
void
p25_crypt_aes::aes_cbc_mac_generator (uint8_t * key, uint8_t * in, uint8_t * out, int type, int nblocks)
{
    int i, j;
    uint8_t input_register[16]; //Input Register
    memset (input_register, 0, sizeof(input_register));

    //Set values specific to type (128/192/256)
    if (type == 0) //128
    {
        Nb = 4;
        Nk = 4;
        Nr = 10;
    }
    else if (type == 1) //192
    {
        Nb = 4;
        Nk = 6;
        Nr = 12;
    }
    else //if (type == 2) //256
    {
        Nb = 4;
        Nk = 8;
        Nr = 14;
    }

    struct AES_ctx ctx;

    //initialize the key variable for the Cipher function
    memset (ctx.RoundKey, 0, 240*sizeof(uint8_t));
    KeyExpansion(ctx.RoundKey, key);

    //
    for (i = 0; i < nblocks; i++)
    {
        //xor the current input 'in' pt to the current state of the input_register for cbc feedback
        //if this is the first iteration, this will load the first round plain text instead
        for (j = 0; j < 16; j++)
            input_register[j] ^= in[j+((i+0)*16)];

        Cipher((state_t*)input_register, ctx.RoundKey);

        //debug, load out all intermediate output register values
        // memcpy (out+(i*16), input_register, 16*sizeof(uint8_t) );

    }

    //copy final ciphered input_register to output 'out', user will determine how many bytes of output they want for MAC
    memcpy (out, input_register, 16*sizeof(uint8_t) );
}

//byte-wise output of AES ECB Ciphering/Deciphering
//input is uint8_t byte-wise (16-bytes) data to be ciphered or deciphered
//input key is up to 32-byte uint8_t array of key value
//input type is the type/len of AES required (0-128, 1-192, 2-256)
//output is a uint8_t bytewise array of ciphered or deciphered input
//de is a bit-flag signalling to run Cipher (encrypt) on 1, or InvCipher (decrypt) on 0
void
p25_crypt_aes::aes_ecb_bytewise_payload_crypt (uint8_t * input, uint8_t * key, uint8_t * output, int type, int de)
{
    uint8_t input_register[16]; //ECB Input Register
    memset (input_register, 0, sizeof(input_register));

    //Set values specific to type (128/192/256)
    if (type == 0) //128
    {
        Nb = 4;
        Nk = 4;
        Nr = 10;
    }
    else if (type == 1) //192
    {
        Nb = 4;
        Nk = 6;
        Nr = 12;
    }
    else //if (type == 2) //256
    {
        Nb = 4;
        Nk = 8;
        Nr = 14;
    }

    struct AES_ctx ctx;

    //load input_register with received input (ECB Payload)
    memcpy (input_register, input, 16*sizeof(uint8_t) );

    //initialize the key variable for the Cipher function
    memset (ctx.RoundKey, 0, 240*sizeof(uint8_t));
    KeyExpansion(ctx.RoundKey, key);

    //run encryption or decryption depending on de value
    if (de) //encrypt
        Cipher((state_t*)input_register, ctx.RoundKey);
    else   //decrypt
        InvCipher((state_t*)input_register, ctx.RoundKey);

    //copy ciphered/deciphered input_register to output
    memcpy (output, input_register, 16*sizeof(uint8_t) );
}

//symmetrical ctr mode payload encryption and decryption
void
p25_crypt_aes::aes_ctr_bitwise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * payload, int type)
{
    //Set values specific to type (128/192/256)
    if (type == 0) //128
    {
        Nb = 4;
        Nk = 4;
        Nr = 10;
    }
    else if (type == 1) //192
    {
        Nb = 4;
        Nk = 6;
        Nr = 12;
    }
    else //if (type == 2) //256
    {
        Nb = 4;
        Nk = 8;
        Nr = 14;
    }

    struct AES_ctx ctx;

    //init and set the iv and key variables
    memset (ctx.RoundKey, 0, 240*sizeof(uint8_t));
    memset (ctx.Iv, 0, 16*sizeof(uint8_t));

    KeyExpansion(ctx.RoundKey, key);
    memcpy (ctx.Iv, iv, AES_BLOCKLEN);

    //pack input bit-wise payload to byte array
    uint8_t payload_bytes[16];
    memset (payload_bytes, 0, sizeof(payload_bytes));
    pack_bit_array_into_byte_array (payload, payload_bytes, 16);

    //pass to internal CTR handler for payload
    AES_CTR_xcrypt_buffer(&ctx, payload_bytes, 16);

    //unpack output bytes back to bits
    unpack_byte_array_into_bit_array(payload_bytes, payload, 16);
}

//symmetrical ctr mode payload encryption and decryption
void
p25_crypt_aes::aes_ctr_bytewise_payload_crypt (uint8_t * iv, uint8_t * key, uint8_t * payload, int type)
{
    //Set values specific to type (128/192/256)
    if (type == 0) //128
    {
        Nb = 4;
        Nk = 4;
        Nr = 10;
    }
    else if (type == 1) //192
    {
        Nb = 4;
        Nk = 6;
        Nr = 12;
    }
    else //if (type == 2) //256
    {
        Nb = 4;
        Nk = 8;
        Nr = 14;
    }

    struct AES_ctx ctx;

    //init and set the iv and key variables
    memset (ctx.RoundKey, 0, 240*sizeof(uint8_t));
    memset (ctx.Iv, 0, 16*sizeof(uint8_t));

    KeyExpansion(ctx.RoundKey, key);
    memcpy (ctx.Iv, iv, AES_BLOCKLEN);

    //pass to internal CTR handler for payload
    AES_CTR_xcrypt_buffer(&ctx, payload, 16);
}

// constructor
p25_crypt_aes::p25_crypt_aes(log_ts& logger, int debug, int msgq_id) :
    p25_crypt_alg(logger, debug, msgq_id) {

        fprintf(stderr, "%s p25_crypt_aes::p25_crypt_aes: loading AES 256 (OFB) module\n", logts.get(d_msgq_id));
    }

// destructor
p25_crypt_aes::~p25_crypt_aes() {

}

// prepare routine entry point
bool
p25_crypt_aes::prepare(uint16_t keyid, protocol_type pr_type, uint8_t *MI) {
    d_pr_type = pr_type;
    d_key_iter = d_keys.find(keyid);
    if (d_key_iter == d_keys.end()) {
        if (d_debug >= 10) {
            fprintf(stderr, "%s p25_crypt_aes::prepare: keyid[0x%x] not found\n", logts.get(d_msgq_id), keyid);
        }
        return false;
    }
    if (d_debug >= 10) {
        fprintf(stderr, "%s p25_crypt_aes::prepare: keyid[0x%x] found\n", logts.get(d_msgq_id), keyid);
    }

    // Expand MI to create proper IV
    uint8_t IV[16];
    p25_crypt_algs::expand_mi_to_128(MI, IV);

	// Find key value from keyid and set up to create keystream
    uint8_t Key[32];
    uint32_t i;
	std::vector<uint8_t>::const_iterator kval_iter = d_key_iter->second.key.begin();
	for (i = 0; i < (uint32_t)std::max(32 - (int)(d_key_iter->second.key.size()), 0); i++) {
		Key[i] = 0;             // pad with leading 0 if supplied key is too short
	}
	for (; i < 32; i++) {
		Key[i] = *kval_iter++;  // copy up to 32 bytes into key array
	}

    // Run the crypt routine to create a keystream long enough for the whole superframe
    // Length is dependent on protocol: FDMA=15 blocks, TDMA=9 blocks.
    aes_ofb_keystream_output (IV, Key, d_keystream, 2, ((pr_type == PT_P25_PHASE2) ? 9 : 15));
    d_position = 0;

    return true;
}

// process routine entry point
bool
p25_crypt_aes::process(packed_codeword& PCW, frame_type fr_type, int voice_subframe) {
    if (d_key_iter == d_keys.end())
        return false;

    bool rc = true;
    size_t offset = 16; //initial offset is 16 (AES-OFB discard round)

    switch (fr_type) {
        /* FDMA */
        case FT_LDU1:
            offset += 0; //additional offset for FDMA is handled below
            break;
        case FT_LDU2:
            offset += 101; //additional offset for FDMA is handled below
            break;
            /* TDMA */
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
        offset += (d_position * 11) + 11 + ((d_position < 8) ? 0 : 2); // voice only; skip 9 LCW bytes, 2 reserved bytes, and LSD between 7,8 and 16,17
        d_position = (d_position + 1) % 9;
        for (int j = 0; j < 11; ++j) {
            PCW[j] = d_keystream[j + offset] ^ PCW[j];
        }

#if 0
        //debug, print keystream values and track offset
        if (d_debug >= 10) {
            fprintf (stderr, "%s AES KS: ", logts.get(d_msgq_id));
            for (int j = 0; j < 7; ++j) {
                fprintf (stderr,  "%02X", d_keystream[j + offset]);
            }
            fprintf (stderr, " Offset: %ld; \n", offset);
        }
#endif

    } else if (d_pr_type == PT_P25_PHASE2) {
        //TDMA - Experimental
        for (int j = 0; j < 7; ++j) {
            PCW[j] = d_keystream[j + offset] ^ PCW[j];
        }
        PCW[6] &= 0x80; // mask everything except the MSB of the final codeword

#if 0
        //debug, print keystream values and track offset
        if (d_debug >= 10) {
            fprintf (stderr, "%s AES KS: ", logts.get(d_msgq_id));
            for (int j = 0; j < 7; ++j) {
                fprintf (stderr,  "%02X", d_keystream[j + offset]);
            }
            fprintf (stderr, " Offset: %ld; \n", offset);
        }
#endif

    }
    return rc;
}
