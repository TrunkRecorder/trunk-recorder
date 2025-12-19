/* -*- c++ -*- */
/* 
 * Copyright 2010, 2011, 2012, 2013, 2014 Max H. Parke KA1RBI 
 * Copyright 2017, 2018, 2019, 2020 Graham J. Norbury
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

#include "p25p1_fdma.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include "bch.h"
#include "op25_msg_types.h"
#include "op25_imbe_frame.h"
#include "p25_frame.h"
#include "p25_framer.h"
#include "rs.h"
#include "p25_crypt_algs.h"
#include "imbe_vocoder/imbe_vocoder.h"

namespace gr {
    namespace op25_repeater {

        static const int64_t TIMEOUT_THRESHOLD = 1000000;

        p25p1_fdma::~p25p1_fdma()
        {
            delete framer;
        }

        static uint16_t crc16(uint8_t buf[], int len) {
            if (buf == 0)
                return -1;
            uint32_t poly = (1<<12) + (1<<5) + (1<<0);
            uint32_t crc = 0;
            for(int i=0; i<len; i++) {
                uint8_t bits = buf[i];
                for (int j=0; j<8; j++) {
                    uint8_t bit = (bits >> (7-j)) & 1;
                    crc = ((crc << 1) | bit) & 0x1ffff;
                    if (crc & 0x10000)
                        crc = (crc & 0xffff) ^ poly;
                }
            }
            crc = crc ^ 0xffff;
            return crc & 0xffff;
        }

        /* translated from p25craft.py Michael Ossmann <mike@ossmann.com>  */
        static uint32_t crc32(uint8_t buf[], int len) {	/* length is nr. of bits */
            uint32_t g = 0x04c11db7;
            uint64_t crc = 0;
            for (int i = 0; i < len; i++) {
                crc <<= 1;
                int b = ( buf [i / 8] >> (7 - (i % 8)) ) & 1;
                if (((crc >> 32) ^ b) & 1)
                    crc ^= g;
            }
            crc = (crc & 0xffffffff) ^ 0xffffffff;
            return crc;
        }

        /* find_min is from wireshark/plugins/p25/packet-p25cai.c */
        /* Copyright 2008, Michael Ossmann <mike@ossmann.com>  */
        /* return the index of the lowest value in a list */
        static int find_min(uint8_t list[], int len) {
            int min = list[0];	
            int index = 0;	
            int unique = 1;	
            int i;

            for (i = 1; i < len; i++) {
                if (list[i] < min) {
                    min = list[i];
                    index = i;
                    unique = 1;
                } else if (list[i] == min) {
                    unique = 0;
                }
            }
            /* return -1 if a minimum can't be found */
            if (!unique)
                return -1;

            return index;
        }

        /* count_bits is from wireshark/plugins/p25/packet-p25cai.c */
        /* Copyright 2008, Michael Ossmann <mike@ossmann.com>  */
        /* count the number of 1 bits in an int */
        static int count_bits(unsigned int n) {
            int i = 0;
            for (i = 0; n != 0; i++)
                n &= n - 1;
            return i;
        }

        /* adapted from wireshark/plugins/p25/packet-p25cai.c */
        /* Copyright 2008, Michael Ossmann <mike@ossmann.com>  */
        /* deinterleave and trellis1_2 decoding */
        /* buf is assumed to be a buffer of 12 bytes */
        static int block_deinterleave(bit_vector& bv, unsigned int start, uint8_t* buf) {
            static const uint16_t deinterleave_tb[] = {
                0,  1,  2,  3,  52, 53, 54, 55, 100,101,102,103, 148,149,150,151,
                4,  5,  6,  7,  56, 57, 58, 59, 104,105,106,107, 152,153,154,155,
                8,  9, 10, 11,  60, 61, 62, 63, 108,109,110,111, 156,157,158,159,
                12, 13, 14, 15,  64, 65, 66, 67, 112,113,114,115, 160,161,162,163,
                16, 17, 18, 19,  68, 69, 70, 71, 116,117,118,119, 164,165,166,167,
                20, 21, 22, 23,  72, 73, 74, 75, 120,121,122,123, 168,169,170,171,
                24, 25, 26, 27,  76, 77, 78, 79, 124,125,126,127, 172,173,174,175,
                28, 29, 30, 31,  80, 81, 82, 83, 128,129,130,131, 176,177,178,179,
                32, 33, 34, 35,  84, 85, 86, 87, 132,133,134,135, 180,181,182,183,
                36, 37, 38, 39,  88, 89, 90, 91, 136,137,138,139, 184,185,186,187,
                40, 41, 42, 43,  92, 93, 94, 95, 140,141,142,143, 188,189,190,191,
                44, 45, 46, 47,  96, 97, 98, 99, 144,145,146,147, 192,193,194,195,
                48, 49, 50, 51 };

            uint8_t hd[4];
            int b, d, j;
            int state = 0;
            uint8_t codeword;

            static const uint8_t next_words[4][4] = {
                {0x2, 0xC, 0x1, 0xF},
                {0xE, 0x0, 0xD, 0x3},
                {0x9, 0x7, 0xA, 0x4},
                {0x5, 0xB, 0x6, 0x8}
            };

            memset(buf, 0, 12);

            for (b=0; b < 98*2; b += 4) {
                codeword = (bv[start+deinterleave_tb[b+0]] << 3) + 
                    (bv[start+deinterleave_tb[b+1]] << 2) + 
                    (bv[start+deinterleave_tb[b+2]] << 1) + 
                    bv[start+deinterleave_tb[b+3]]     ;

                /* try each codeword in a row of the state transition table */
                for (j = 0; j < 4; j++) {
                    /* find Hamming distance for candidate */
                    hd[j] = count_bits(codeword ^ next_words[state][j]);
                }
                /* find the dibit that matches the most codeword bits (minimum Hamming distance) */
                state = find_min(hd, 4);
                /* error if minimum can't be found */
                if(state == -1)
                    return -1;	// decode error, return failure
                /* It also might be nice to report a condition where the minimum is
                 * non-zero, i.e. an error has been corrected.  It probably shouldn't
                 * be a permanent failure, though.
                 *
                 * DISSECTOR_ASSERT(hd[state] == 0);
                 */

                /* append dibit onto output buffer */
                d = b >> 2;	// dibit ctr
                if (d < 48) {
                    buf[d >> 2] |= state << (6 - ((d%4) * 2));
                }
            }
            return 0;
        }

        void p25p1_fdma::set_debug(int debug)
        {
            d_debug = debug;
            framer->set_debug(debug);
        }

        void p25p1_fdma::set_nac(int nac)
        {
            d_nac = nac;
            framer->set_nac(nac);
            if (d_debug >= 10)
                fprintf(stderr, "%s p25p1_fdma::set_nac: 0x%03x\n", logts.get(d_msgq_id), d_nac);
        }

        p25p1_fdma::p25p1_fdma(const op25_audio& udp, log_ts& logger, int debug, bool do_imbe, bool do_output, bool do_msgq, gr::msg_queue::sptr queue, std::deque<int16_t> &output_queue, bool do_audio_output, bool soft_vocoder, int msgq_id) :
            write_bufp(0),
            d_debug(debug),
            d_do_imbe(do_imbe),
            d_do_output(do_output),
            d_do_msgq(do_msgq),
            d_msgq_id(msgq_id),
            d_do_audio_output(do_audio_output),
            d_nac(0),
            d_msg_queue(queue),
            d_soft_vocoder(soft_vocoder),
            output_queue(output_queue),
            framer(new p25_framer(logger, debug, msgq_id)),
            qtimer(op25_timer(TIMEOUT_THRESHOLD)),
            op25audio(udp),
            logts(logger),
            crypt_algs(logger, debug, msgq_id),
            ess_keyid(0),
            curr_src_id(-1),
            curr_grp_id(-1),
            ess_algid(0x80),
            vf_tgid(0),
			terminate_call(std::pair<bool,long>(false,0)),
            p1voice_decode((debug > 0), udp, output_queue)
        {
			rx_status.error_count = 0;
			rx_status.total_len = 0;
			rx_status.spike_count = 0;
			rx_status.last_update = time(NULL);
			for (int i=0; i<20; i++)
				error_history[i] = -1;
		}

		void p25p1_fdma::reset_rx_status() {
			rx_status.error_count = 0;
			rx_status.total_len = 0;
			rx_status.spike_count = 0;
			rx_status.last_update = 0;
			/*for (int i=0; i<20; i++)
				error_history[i] = -1;*/
		}

		Rx_Status p25p1_fdma::get_rx_status() {
			return rx_status;
		}

        void p25p1_fdma::reset_call_terminated() {
            terminate_call = std::pair<bool,long>(false,0);
        }
		std::pair<bool,long> p25p1_fdma::get_call_terminated() {
			return terminate_call;
		}
        long p25p1_fdma::get_curr_grp_id() {
            long addr = curr_grp_id;
            curr_grp_id = -1;
            // This makes it easy to tell when a new Src Address has been received, all other times it will be -1
			return addr;
		}

		long p25p1_fdma::get_curr_src_id() {
            long addr = curr_src_id;
            curr_src_id = -1;
            // This makes it easy to tell when a new Src Address has been received, all other times it will be -1
			return addr;
		}
		void p25p1_fdma::clear() {
			p1voice_decode.clear();
		}

        void p25p1_fdma::process_duid(uint32_t const duid, uint32_t const nac, const uint8_t* buf, const int len) {
            char wbuf[256];
            int p = 0;
            if (!d_do_msgq)
                return;
            assert (len+2 <= (int)sizeof(wbuf));
            wbuf[p++] = (nac >> 8) & 0xff;
            wbuf[p++] = nac & 0xff;
            if (buf) {
                memcpy(&wbuf[p], buf, len);	// copy data
                p += len;
            }
            send_msg(std::string(wbuf, p), duid);
            qtimer.reset();
        }

        void p25p1_fdma::process_HDU(const bit_vector& A) {
            if (d_debug >= 10) {
                fprintf (stderr, "%s NAC 0x%03x HDU:  ", logts.get(d_msgq_id), framer->nac);
            }

            uint32_t MFID;
            int i, j, k, ec;
            size_t errs = 0, gly_errs = 0;
            std::vector<uint8_t> HB(63,0); // hexbit vector
            k = 0;
            for (i = 0; i < 36; i ++) {
                uint32_t CW = 0;
                for (j = 0; j < 18; j++) {  // 18 bits / cw
                    CW = (CW << 1) + A [ hdu_codeword_bits[k++] ];
                }
                HB[27 + i] = gly24128Dec(CW, &errs) & 63;
                gly_errs += errs;
            }
            ec = rs16.decode(HB); // Reed Solomon (36,20,17) error correction

            if ((ec >= 0) && (ec <= 8)) { // upper limit of 8 corrections
                j = 27;												// 72 bit MI
                for (i = 0; i < 9;) {
                    ess_mi[i++] = (uint8_t)  (HB[j  ]         << 2) + (HB[j+1] >> 4);
                    ess_mi[i++] = (uint8_t) ((HB[j+1] & 0x0f) << 4) + (HB[j+2] >> 2);
                    ess_mi[i++] = (uint8_t) ((HB[j+2] & 0x03) << 6) +  HB[j+3];
                    j += 4;
                }
                MFID      =  (HB[j  ]         <<  2) + (HB[j+1] >> 4);						// 8 bit MfrId
                ess_algid = ((HB[j+1] & 0x0f) <<  4) + (HB[j+2] >> 2);						// 8 bit AlgId
                ess_keyid = ((HB[j+2] & 0x03) << 14) + (HB[j+3] << 8) + (HB[j+4] << 2) + (HB[j+5] >> 4);	// 16 bit KeyId
                vf_tgid   = ((HB[j+5] & 0x0f) << 12) + (HB[j+6] << 6) +  HB[j+7];				// 16 bit TGID

                curr_grp_id = vf_tgid;
                if (d_debug >= 10) {
                    fprintf (stderr, "ESS: tgid=%d, mfid=%x, algid=%x, keyid=%x, mi=%02x %02x %02x %02x %02x %02x %02x %02x %02x",
                            vf_tgid, MFID, ess_algid, ess_keyid,
                            ess_mi[0], ess_mi[1], ess_mi[2], ess_mi[3], ess_mi[4], ess_mi[5],ess_mi[6], ess_mi[7], ess_mi[8]);
                }
            }

            if (d_debug >= 10) {
                fprintf (stderr, ", gly_errs=%lu, rs_errs=%d\n", gly_errs, ec);
            }
        }

        void p25p1_fdma::process_LLDU(const bit_vector& A, std::vector<uint8_t>& HB) {
            process_duid(framer->duid, framer->nac, NULL, 0);

            int i, j, k;
            k = 0;
            for (i = 0; i < 24; i ++) { // 24 10-bit codewords
                uint32_t CW = 0;
                for (j = 0; j < 10; j++) {  // 10 bits / cw
                    CW = (CW << 1) + A[ imbe_ldu_ls_data_bits[k++] ];
                }
                HB[39 + i] = hmg1063Dec( CW >> 4, CW & 0x0f );
            }
        }

        void p25p1_fdma::process_LDU1(const bit_vector& A) {
            if (d_debug >= 10) {
                fprintf (stderr, "%s NAC 0x%03x LDU1: ", logts.get(d_msgq_id), framer->nac);
            }

            std::vector<uint8_t> HB(63,0); // hexbit vector
            process_LLDU(A, HB);
            process_LCW(HB);

            if (d_debug >= 10) {
                fprintf (stderr, "\n");
            }

            process_voice(A, FT_LDU1);
        }

        void p25p1_fdma::process_LDU2(const bit_vector& A) {
            uint16_t next_keyid;
            uint8_t  next_algid;
            uint8_t  next_mi[9] = {0};
            bool next_ess_valid = false;

            if (d_debug >= 10) {
                fprintf (stderr, "%s NAC 0x%03x LDU2: ", logts.get(d_msgq_id), framer->nac);
            }

            std::vector<uint8_t> HB(63,0); // hexbit vector
            process_LLDU(A, HB);

            int i, j, ec;
            ec = rs8.decode(HB); // Reed Solomon (24,16,9) error correction
            if ((ec >= 0) && (ec <= 4)) {	// upper limit of 4 corrections
                j = 39;                                                             // 72 bit MI
                for (i = 0; i < 9;) {
                    next_mi[i++] = (uint8_t)  (HB[j  ]         << 2) + (HB[j+1] >> 4);
                    next_mi[i++] = (uint8_t) ((HB[j+1] & 0x0f) << 4) + (HB[j+2] >> 2);
                    next_mi[i++] = (uint8_t) ((HB[j+2] & 0x03) << 6) +  HB[j+3];
                    j += 4;
                }
                next_algid =  (HB[j  ]         <<  2) + (HB[j+1] >> 4);             //  8 bit AlgId
                next_keyid = ((HB[j+1] & 0x0f) << 12) + (HB[j+2] << 6) + HB[j+3];   // 16 bit KeyId
                next_ess_valid = true;

                if (d_debug >= 10) {
                    fprintf (stderr, "ESS: algid=%x, keyid=%x, mi=%02x %02x %02x %02x %02x %02x %02x %02x %02x, rs_errs=%d\n",
                            next_algid, next_keyid,
                            next_mi[0], next_mi[1], next_mi[2], next_mi[3], next_mi[4], next_mi[5], next_mi[6], next_mi[7], next_mi[8],
                            ec); 
                }
            }

            process_voice(A, FT_LDU2);

            // replace existing ess with newly received data now that voice processing is complete
            if (next_ess_valid) {
                ess_algid = next_algid;
                ess_keyid = next_keyid;
                memcpy(ess_mi, next_mi, sizeof(next_mi));
            }
        }

        void p25p1_fdma::process_TTDU() {
            process_duid(framer->duid, framer->nac, NULL, 0);
            reset_ess();

            if ((d_do_imbe || d_do_audio_output) && (framer->duid == 0x3 || framer->duid == 0xf)) {  // voice termination
                op25audio.send_audio_flag(op25_audio::DRAIN);
				terminate_call = std::pair<bool,long>(true, output_queue.size());
            }
        }

        void p25p1_fdma::process_TDU3() {
            if (d_debug >= 10) {
                fprintf (stderr, "%s NAC 0x%03x TDU3:  ", logts.get(d_msgq_id), framer->nac);
            }

            process_TTDU();

            if (d_debug >= 10) {
                fprintf (stderr, "\n");
            }
        }

        void p25p1_fdma::process_TDU15(const bit_vector& A) {
            if (d_debug >= 10) {
                fprintf (stderr, "%s NAC 0x%03x TDU15:  ", logts.get(d_msgq_id), framer->nac);
            }

            process_TTDU();

            int i, j, k;
            size_t gly_errs = 0, errs = 0;
            std::vector<uint8_t> HB(63,0); // hexbit vector
            k = 0;
            for (i = 0; i <= 22; i += 2) {
                uint32_t CW = 0;
                for (j = 0; j < 12; j++) {   // 12 24-bit codewords
                    CW = (CW << 1) + A [ hdu_codeword_bits[k++] ];
                    CW = (CW << 1) + A [ hdu_codeword_bits[k++] ];
                }
                uint32_t D = gly24128Dec(CW, &errs);
                HB[39 + i] = D >> 6;
                HB[40 + i] = D & 63;
            }
            process_LCW(HB);

            if (d_debug >= 10) {
                fprintf (stderr, ", gly_errs=%lu\n", gly_errs);
            }
        }

        void p25p1_fdma::process_LCW(std::vector<uint8_t>& HB) {
            int ec = rs12.decode(HB); // Reed Solomon (24,12,13) error correction
            if ((ec < 0) || (ec > 6)) // upper limit of 6 corrections
                return; // failed CRC

            int i, j;
            std::vector<uint8_t> lcw(9,0); // Convert hexbits to bytes
            j = 0;
            for (i = 0; i < 9;) {
                lcw[i++] = (uint8_t)  (HB[j+39]         << 2) + (HB[j+40] >> 4);
                lcw[i++] = (uint8_t) ((HB[j+40] & 0x0f) << 4) + (HB[j+41] >> 2);
                lcw[i++] = (uint8_t) ((HB[j+41] & 0x03) << 6) +  HB[j+42];
                j += 4;
            }

            std::string pdu(11,0);
            pdu[0] = (framer->nac >> 8) & 0xff; pdu[1] = framer->nac & 0xff;
            for (int i = 0; i < 9; i++) {
                pdu[2+i] = lcw[i];
            }
            //send_msg(pdu, M_P25_FDMA_LCW); // This messages is not being parsed, not sure what it is for

            int pb =   (lcw[0] >> 7);
            int sf =  ((lcw[0] & 0x40) >> 6);
            int lco =   lcw[0] & 0x3f;
            std::string s = "";

            if (d_debug >= 10) {
                fprintf(stderr, "LCW: ec=%d, pb=%d, sf=%d, lco=%d : %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                        ec, pb, sf, lco, lcw[0], lcw[1], lcw[2], lcw[3], lcw[4], lcw[5], lcw[6], lcw[7], lcw[8]);
            }

            if (pb == 0) { // only decode if unencrypted
                if ((sf == 0) && ((lcw[1] == 0x00) || (lcw[1] == 0x01) || (lcw[1] == 0x90))) {	// sf=0, explicit MFID in standard or Motorola format
                    switch (lco) {
                        case 0x00: { // Group Voice Channel User
                            uint16_t grpaddr = (lcw[4] << 8) + lcw[5];
                            uint32_t srcaddr = (lcw[6] << 16) + (lcw[7] << 8) + lcw[8];

                            curr_src_id = srcaddr;
                            curr_grp_id = grpaddr;
                            s = "{\"srcaddr\" : " + std::to_string(srcaddr) + ", \"grpaddr\": " + std::to_string(grpaddr) + "}";
                            send_msg(s, -3);
                            if (d_debug >= 10)
                                fprintf(stderr, ", srcaddr=%d, grpaddr=%d", srcaddr, grpaddr);
                            break;
                        }
                        case 0x15: { // Motorola alias header
                            if (lcw[1] == 0x90) {
                                int messages = lcw[4] & 0xf;
                                int id_sequence = lcw[7] >> 4;

                                alias_buffer[0] = lcw;
                                for (int i = 1; i <= messages; i++) {
                                    alias_buffer[i] = std::vector<uint8_t>(9, 0);
                                }
                            }
                            break;
                        }
                        case 0x17: { // Motorola alias messages
                            if (lcw[1] == 0x90) {
                                // get message count
                                int messages = 0;
                                int header_sequence = -1;

                                if (alias_buffer[0].size() >= 8) {
                                    messages = alias_buffer[0][4] & 0xf;
                                    header_sequence = alias_buffer[0][7] >> 4;
                                }

                                int message = lcw[2] & 0xf;
                                int msg_sequence = lcw[3] >> 4;

                                // Validate message index is within bounds
                                if ((message > 0 && message < 10) && (message <= messages) && (msg_sequence == header_sequence)) {
                                    alias_buffer[message] = lcw;

                                    // When all messages received, send raw buffer to recorder for decoding
                                    if (message == messages && messages > 0 && messages < (int)alias_buffer.size()) {
                                        std::string msg = "{\"type\": \"motorola_alias_p1\", \"messages\": " + std::to_string(messages) + ", \"blocks\": {";
                                        for (int i = 0; i <= messages && i < 10; i++) {
                                            if (!alias_buffer[i].empty()) {
                                                if (i > 0) msg += ", ";
                                                msg += "\"" + std::to_string(i) + "\": \"" + uint8_vector_to_hex_string(alias_buffer[i]) + "\"";
                                            }
                                        }
                                        msg += "}}";
                                        send_msg(msg, M_P25_JSON_DATA);
                                    }
                                }
                            }
                            break;
                        }
                    }
                } else if (sf == 1) {						// sf=1, implicit MFID
                    switch (lco) {
                        case 0x02: { // Group Voice Channel Update
                            std::string tsbk(12,0);
                            uint16_t ch_A  = (lcw[1] << 8) + lcw[2];
                            uint16_t grp_A = (lcw[3] << 8) + lcw[4];
                            uint16_t ch_B  = (lcw[5] << 8) + lcw[6];
                            uint16_t grp_B = (lcw[7] << 8) + lcw[8];
                            if (d_debug >= 10)
                                fprintf(stderr, ", ch_A=%d, grp_A=%d, ch_B=%d, grp_B=%d", ch_A, grp_A, ch_B, grp_B);
                            tsbk[0] = 0xff; tsbk[1] = 0xff;
                            tsbk[2] = 0x82;
                            tsbk[3] = 0x00;
                            tsbk[4] = ch_A >> 8; tsbk[5] = ch_A & 0xff;
                            tsbk[6] = grp_A >> 8; tsbk[7] = grp_A & 0xff;
                            tsbk[8] = ch_B >> 8; tsbk[9] = ch_B & 0xff;
                            tsbk[10] = grp_B >> 8; tsbk[11] = grp_B & 0xff;
                            send_msg(tsbk, 7);
                            break;
                        }
                        case 0x04: { // Group Voice Channel Update Explicit
                            std::string tsbk(12,0);
                            uint8_t  svcopts = (lcw[2]     )         ;
                            uint16_t grpaddr = (lcw[3] << 8) + lcw[4];
                            uint16_t ch_T    = (lcw[5] << 8) + lcw[6];
                            uint16_t ch_R    = (lcw[7] << 8) + lcw[8];

                            // Don't use this grpaddr, you can get Updates for Talkgroups that are on other frequencies, the GRP here may not be the one the being recordered
                            if (d_debug >= 10)
                                fprintf(stderr, ", svcopts=0x%02x, grpaddr=%d, ch_T=%d, ch_R=%d", svcopts, grpaddr, ch_T, ch_R);
                            tsbk[0] = 0xff; tsbk[1] = 0xff;
                            tsbk[2] = 0x83;
                            tsbk[3] = 0x00;
                            tsbk[4] = svcopts;
                            tsbk[5] = 0x00;
                            tsbk[6] = ch_T >> 8; tsbk[7] = ch_T & 0xff;
                            tsbk[8] = ch_R >> 8; tsbk[9] = ch_R & 0xff;
                            tsbk[10] = grpaddr >> 8; tsbk[11] = grpaddr & 0xff;
                            send_msg(tsbk, 7);
                            break;
                        }

                    }
                }
            }

        }

        void p25p1_fdma::process_TSBK(const bit_vector& fr, uint32_t fr_len) {
            uint8_t op, lb = 0;
            block_vector deinterleave_buf;
            if (process_blocks(fr, fr_len, deinterleave_buf) == 0) {
                for (size_t j = 0; (j < deinterleave_buf.size()) && (lb == 0); j++) {
                    if (crc16(deinterleave_buf[j].data(), 12) != 0) // validate CRC
                        return;

                    lb = deinterleave_buf[j][0] >> 7;	// last block flag
                    op = deinterleave_buf[j][0] & 0x3f;	// opcode
                    process_duid(framer->duid, framer->nac, deinterleave_buf[j].data(), 10);

                    if (d_debug >= 10) {
                        fprintf (stderr, "%s NAC 0x%03x TSBK: op=%02x : %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                logts.get(d_msgq_id), framer->nac, op,
                                deinterleave_buf[j][0], deinterleave_buf[j][1], deinterleave_buf[j][2], deinterleave_buf[j][3],
                                deinterleave_buf[j][4], deinterleave_buf[j][5], deinterleave_buf[j][6], deinterleave_buf[j][7],
                                deinterleave_buf[j][8], deinterleave_buf[j][9], deinterleave_buf[j][10], deinterleave_buf[j][11]);
                    }
                }
            }
        }

        void p25p1_fdma::process_PDU(const bit_vector& fr, uint32_t fr_len) {
            uint8_t fmt, sap, blks, op = 0;
            block_vector deinterleave_buf;
            if ((process_blocks(fr, fr_len, deinterleave_buf) == 0) &&
                    (deinterleave_buf.size() > 0)) {			// extract all blocks associated with this PDU
                if (crc16(deinterleave_buf[0].data(), 12) != 0) // validate PDU header
                    return;

                fmt =  deinterleave_buf[0][0] & 0x1f;
                sap =  deinterleave_buf[0][1] & 0x3f;
                blks = deinterleave_buf[0][6] & 0x7f;

                if ((sap == 61) && ((fmt == 0x17) || (fmt == 0x15))) { // Multi Block Trunking messages
                    if ((blks > deinterleave_buf.size()) || (deinterleave_buf.size() == 1))
                        return; // insufficient blocks available

                    uint32_t crc1 = crc32(deinterleave_buf[1].data(), ((blks * 12) - 4) * 8);
                    uint32_t crc2 = (deinterleave_buf[blks][8] << 24) + (deinterleave_buf[blks][9] << 16) +
                        (deinterleave_buf[blks][10] << 8) + deinterleave_buf[blks][11];

                    if (crc1 != crc2)
                        return; // payload crc check failed

                    process_duid(framer->duid, framer->nac, deinterleave_buf[0].data(), ((blks + 1) * 12) - 4);

                    if (d_debug >= 10) {
                        if (fmt == 0x15) {
                            op =   deinterleave_buf[1][0] & 0x3f; // Unconfirmed MBT format
                        } else if (fmt == 0x17) {
                            op =   deinterleave_buf[0][7] & 0x3f; // Alternate MBT format
                        }

                        char s0[40], s1[40], s2[40], s3[40];
                        sprintf(s0, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[0][0], deinterleave_buf[0][1], deinterleave_buf[0][2], deinterleave_buf[0][3],
                                deinterleave_buf[0][4], deinterleave_buf[0][5], deinterleave_buf[0][6], deinterleave_buf[0][7],
                                deinterleave_buf[0][8], deinterleave_buf[0][9], deinterleave_buf[0][10], deinterleave_buf[0][11]);
                        sprintf(s1, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[1][0], deinterleave_buf[1][1], deinterleave_buf[1][2], deinterleave_buf[1][3],
                                deinterleave_buf[1][4], deinterleave_buf[1][5], deinterleave_buf[1][6], deinterleave_buf[1][7],
                                deinterleave_buf[1][8], deinterleave_buf[1][9], deinterleave_buf[1][10], deinterleave_buf[1][11]);
                        sprintf(s2, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[2][0], deinterleave_buf[2][1], deinterleave_buf[2][2], deinterleave_buf[2][3],
                                deinterleave_buf[2][4], deinterleave_buf[2][5], deinterleave_buf[2][6], deinterleave_buf[2][7],
                                deinterleave_buf[2][8], deinterleave_buf[2][9], deinterleave_buf[2][10], deinterleave_buf[2][11]);
                        sprintf(s3, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                deinterleave_buf[3][0], deinterleave_buf[3][1], deinterleave_buf[3][2], deinterleave_buf[3][3],
                                deinterleave_buf[3][4], deinterleave_buf[3][5], deinterleave_buf[3][6], deinterleave_buf[3][7],
                                deinterleave_buf[3][8], deinterleave_buf[3][9], deinterleave_buf[3][10], deinterleave_buf[3][11]);
                        fprintf (stderr, "%s NAC 0x%03x PDU:  fmt=%02x, op=0x%02x : %s %s %s %s\n",
                                logts.get(d_msgq_id), framer->nac, fmt, op, s0, s1, s2, s3);
                    }
                } else if (d_debug >= 10) {
                    fprintf(stderr, "%s NAC 0x%03x PDU:  non-MBT message ignored\n", logts.get(d_msgq_id), framer->nac);
                }

            }
        }

        int p25p1_fdma::process_blocks(const bit_vector& fr, uint32_t& fr_len, block_vector& dbuf) {
            bit_vector bv;
            bv.reserve(fr_len >> 1);
            for (unsigned int d=0; d < fr_len >> 1; d++) {	  // eliminate status bits from frame
                if ((d+1) % 36 == 0)
                    continue;
                bv.push_back(fr[d*2]);
                bv.push_back(fr[d*2+1]);
            }

            int bl_cnt = 0;
            int bl_len = (bv.size() - (48+64)) / 196;
            for (bl_cnt = 0; bl_cnt < bl_len; bl_cnt++) { // deinterleave,  decode trellis1_2, save 12 byte block
                dbuf.push_back({0,0,0,0,0,0,0,0,0,0,0,0});
                if(block_deinterleave(bv, 48+64+bl_cnt*196, dbuf[bl_cnt].data()) != 0) {
                    dbuf.pop_back();
                    return -1;
                }
            }
            return (bl_cnt > 0) ? 0 : -1;
        }

        void p25p1_fdma::process_voice(const bit_vector& A, const frame_type fr_type) {
            if (d_do_imbe || d_do_audio_output) {
                if (encrypted())
                    crypt_algs.prepare(ess_algid, ess_keyid, PT_P25_PHASE1, ess_mi);

                for(size_t i = 0; i < nof_voice_codewords; ++i) {
                    voice_codeword cw(voice_codeword_sz);
                    uint32_t E0, ET;
                    uint32_t u[8];
                    char s[128];
                    bool audio_valid = !encrypted();
                    size_t errs = 0;
                    imbe_deinterleave(A, cw, i);
                    uint16_t imbe_error = 0;

                    errs = imbe_header_decode(cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], E0, ET);

                    if (d_debug >= 9) {
                        packed_codeword p_cw;
                        imbe_pack(p_cw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                        sprintf(s,"%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                p_cw[0], p_cw[1], p_cw[2], p_cw[3], p_cw[4], p_cw[5],
                                p_cw[6], p_cw[7], p_cw[8], p_cw[9], p_cw[10]);
                        fprintf(stderr, "%s IMBE %s errs %lu\n", logts.get(d_msgq_id), s, errs); // print to log in one operation
                    }

                    if (encrypted()) {
                        packed_codeword ciphertext;
                        imbe_pack(ciphertext, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                        audio_valid = crypt_algs.process(ciphertext, fr_type, i);
                        imbe_unpack(ciphertext, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                    }


					imbe_error = (uint16_t) errs;	 
					// this section calculates if the IMBE Error is greater than std deviation. If so it adds to spike count
					// this is probably not neccesary.
					for (int j=19; j>0; j--) {
						error_history[j] = error_history[j-1];
					}
					error_history[0] = imbe_error;
					double error_history_total=0;
					double error_history_len=0;
					for (int j=0; j<20; j++) {
						if (error_history[j]>=0) {
						error_history_len++;
						error_history_total += error_history[j];
						}
					}
					double error_history_avg = error_history_total / error_history_len;

					double error_history_sqrt;
					for (int j=0; j<error_history_len; j++){
						error_history_sqrt += pow((error_history[j] - error_history_avg), 2);
					}
					float std_dev = sqrt(error_history_sqrt/error_history_len);

					if (imbe_error >  std_dev) {
						rx_status.spike_count++;
						if (d_debug >= 10) {
						fprintf(stderr, "SPIKE! Errors: %d \tStd Dev: %f \tAvg: %f \tLimit: %f\n", imbe_error, std_dev, error_history_avg, std_dev + error_history_avg );
						}
					}
					rx_status.error_count += imbe_error;
					rx_status.total_len += 144;
					//rx_status.last_update = time(NULL); //uncomment to ignore post-voice frames
					// output one 32-byte msg per 0.020 sec.
					// also, 32*9 = 288 byte pkts (for use via UDP)
					sprintf(s, "%03x %03x %03x %03x %03x %03x %03x %03x\n", u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);

                    if (d_do_audio_output) {
                        if ( !encrypted()) {
                            // This is the Vocoder that OP25 currently uses.

                            if (d_soft_vocoder) {
                                // This is vocoder that is for half-rate
                                software_decoder.decode_fullrate(snd, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], E0, ET);
                            } else {

                                // This is the older, fullrate vocoder
                                // it was copied from p25p1_voice_decode.cc
                                int16_t frame_vector[8];

                                for (int i=0; i < 8; i++) { // Ugh. For compatibility convert imbe params from uint32_t to int16_t
                                    frame_vector[i] = u[i];
                                }
                                frame_vector[7] >>= 1;
                                vocoder.imbe_decode(frame_vector, snd);
                            }

                            if (op25audio.enabled()) {      // decoded audio goes out via UDP (normal code path)
                                op25audio.send_audio(snd, SND_FRAME * sizeof(int16_t));
                            } else {                        // decoded audio back to gnuradio (still supported?)
                                for (int i = 0; i < SND_FRAME; i++) {
                                    output_queue.push_back(snd[i]);
                                }
                            }
                        } else {
		                    // For encrypted voice without a valid key, push silent audio frames
                            // If monitoring for metadata, this will allow tags to pass and preserve call flow
                            if (!op25audio.enabled()) {
                                for (int i = 0; i < SND_FRAME; i++) {
                                    output_queue.push_back(0);  // Silent frame
                                }
                            }
                            std::string encr = "{\"encrypted\": " + std::to_string(1) + ", \"algid\": " + std::to_string(ess_algid) + ", \"keyid\": " + std::to_string(ess_keyid) + "}";
                            send_msg(encr, M_P25_JSON_DATA);
                        }
                    }

                    if (d_do_output && !d_do_audio_output) { // ugh! - legacy wireshark support
                        sprintf(s, "%03x %03x %03x %03x %03x %03x %03x %03x\n", u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                        for (size_t j=0; j < strlen(s); j++) {
                            output_queue.push_back(s[j]);
                        }
                    }
                }
            }
        }

        void p25p1_fdma::reset_timer() {
            qtimer.reset();
        }

        void p25p1_fdma::call_end() {
            if (d_do_audio_output)
                op25audio.send_audio_flag(op25_audio::DRAIN);
            reset_ess();
        }

        void p25p1_fdma::crypt_reset() {
            crypt_algs.reset();
        }

        void p25p1_fdma::crypt_key(uint16_t keyid, uint8_t algid, const std::vector<uint8_t> &key) {
            crypt_algs.key(keyid, algid, key);
        }

        void p25p1_fdma::send_msg(const std::string msg_str, long msg_type) {
            if (!d_do_msgq)
                return;

            gr::message::sptr msg = gr::message::make_from_string(msg_str, msg_type);     

            if (!d_msg_queue->full_p())
                d_msg_queue->insert_tail(msg);
        }

        void p25p1_fdma::process_frame() {
            // extract additional signalling information and voice codewords
            switch(framer->duid) {
                case 0x00:
                    process_HDU(framer->frame_body);
                    break;
                case 0x03:
                    process_TDU3();
                    break;
                case 0x05:
                    process_LDU1(framer->frame_body);
                    break;
                case 0x07:
                    process_TSBK(framer->frame_body, framer->frame_size);
                    break;
                case 0x0a:
                    process_LDU2(framer->frame_body);
                    break;
                case 0x0c:
                    process_PDU(framer->frame_body, framer->frame_size);
                    break;
                case 0x0f:
                    process_TDU15(framer->frame_body);
                    break;
            }

            if (!d_do_imbe) { // send raw frame to wireshark
                // pack the bits into bytes, MSB first
                size_t obuf_ct = 0;
                uint8_t obuf[P25_VOICE_FRAME_SIZE/2];
                for (uint32_t i = 0; i < framer->frame_size; i += 8) {
                    uint8_t b = 
                        (framer->frame_body[i+0] << 7) +
                        (framer->frame_body[i+1] << 6) +
                        (framer->frame_body[i+2] << 5) +
                        (framer->frame_body[i+3] << 4) +
                        (framer->frame_body[i+4] << 3) +
                        (framer->frame_body[i+5] << 2) +
                        (framer->frame_body[i+6] << 1) +
                        (framer->frame_body[i+7]     );
                    obuf[obuf_ct++] = b;
                }
                op25audio.send_to(obuf, obuf_ct);

                if (d_do_output) {
                    for (size_t j=0; j < obuf_ct; j++) {
                        output_queue.push_back(obuf[j]);
                    }
                }
            }
        }

        // Construct a frame one symbol at a time (used by rx.py)
        void p25p1_fdma::rx_sym (const uint8_t *syms, int nsyms) {
            for (int i1 = 0; i1 < nsyms; i1++) {
                if(framer->rx_sym(syms[i1])) {   // complete frame was detected
                    if (framer->nac == 0) {  // discard frame if NAC is invalid
                        continue;
                    }
					rx_status.error_count += framer->bch_errors;
					rx_status.total_len += 64;
					rx_status.last_update = time(NULL); //comment/remove if you don't care about non-voice frames
					terminate_call = std::pair<bool,long>(false,0);

                    process_frame();
                }  // end of complete frame
            }
            check_timeout();
        }

        // Load a frame starting with NID block (used by multi_rx.py)
        uint32_t p25p1_fdma::load_nid(const uint8_t *syms, int nsyms, const uint64_t fs) {
            uint32_t fr_len = framer->load_nid(syms, nsyms, fs);
            check_timeout();
            return fr_len;
        }

        // Load remainder of frame body (used by multi_rx.py)
        bool p25p1_fdma::load_body(const uint8_t *syms, int nsyms) {
            bool rc = framer->load_body(syms, nsyms);
            if (rc) {
                process_frame();
            }
            check_timeout();
            return rc;
        }

        // Check for timer expiry
        void p25p1_fdma::check_timeout() {
            if (d_do_msgq) {
                // check for timeout
                if (qtimer.expired()) {
                    if (d_debug >= 10)
                        fprintf(stderr, "%s p25p1_fdma::check_timeout: expired\n", logts.get(d_msgq_id));

                    if (d_do_audio_output) {
                        op25audio.send_audio_flag(op25_audio::DRAIN);
                    }

                    qtimer.reset();
                    gr::message::sptr msg = gr::message::make(get_msg_type(PROTOCOL_P25, M_P25_TIMEOUT), (d_msgq_id << 1), logts.get_ts());
                    if (!d_msg_queue->full_p())
                        d_msg_queue->insert_tail(msg);
                }
            }
        }

    }  // namespace
}  // namespace
