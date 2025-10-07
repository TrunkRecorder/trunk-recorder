/*
 * Project 25 IMBE Encoder/Decoder Fixed-Point implementation
 * Developed by Pavel Yazev E-mail: pyazev@gmail.com
 * Version 1.0 (c) Copyright 2009
 * 
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * The software is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this; see the file COPYING.  If not, write to the Free
 * Software Foundation, Inc., 51 Franklin Street, Boston, MA
 * 02110-1301, USA.
 */

#include "typedef.h"
#include "globals.h"
#include "imbe.h"
#include "ch_decode.h"
#include "sa_decode.h"
#include "sa_enh.h"
#include "v_synt.h"
#include "uv_synt.h"
#include "basic_op.h"
#include "aux_sub.h"
#include "encode.h"
#include "dsp_sub.h"
#include "imbe_vocoder.h"


#include <string.h>
#include <math.h>



void imbe_vocoder::decode_init(IMBE_PARAM *imbe_param)
{
	v_synt_init();
	uv_synt_init();
	sa_decode_init();

	// Make previous frame for the first frame
	memset((char *)imbe_param, 0, sizeof(IMBE_PARAM));
	imbe_param->fund_freq = 0x0cf6474a;
	imbe_param->num_harms = 9;
	imbe_param->num_bands = 3;

}


void imbe_vocoder::decode(IMBE_PARAM *imbe_param, Word16 *frame_vector, Word16 *snd)
{
	Word16 snd_tmp[FRAME];
	Word16 j;

	decode_frame_vector(imbe_param, frame_vector);
	v_uv_decode(imbe_param);
	sa_decode(imbe_param);
	sa_enh(imbe_param);
	v_synt(imbe_param, snd);
	uv_synt(imbe_param, snd_tmp);

	for(j = 0; j < FRAME; j++)
		snd[j] = add(snd[j], snd_tmp[j]);
}


void imbe_vocoder::decode_tap(Word16 *snd, int L, float w0, const int *Vl, const float *Ml)
{
	Word16 snd_v[FRAME];
    Word16 snd_uv[FRAME];
    int i;

	//
	// Convert floating-point parameters into fixed-point IMBE_PARAM struct
	//

	// Number of Harmonics (L) -> num_harms
	my_imbe_param.num_harms = L;

	// Fundamental Frequency (w0) -> fund_freq
	// w0 is float angular frequency, convert to frequency
	float fund_freq_f = w0 / M_PI;
	// Convert fundamental frequency to fixed-point Q1.31 format
	my_imbe_param.fund_freq = (Word32)(fund_freq_f * (UWord32)(1LL << 31));

	// Zero out harmonic vectors so only used harmonics have values
	memset(my_imbe_param.v_uv_dsn, 0, sizeof(my_imbe_param.v_uv_dsn));
	memset(my_imbe_param.sa, 0, sizeof(my_imbe_param.sa));

	// Voicing Vector (Vl) -> v_uv_dsn
    for (i = 0; i < L; i++) {
        my_imbe_param.v_uv_dsn[i] = (Word16)Vl[i];
    }

	// Spectral Amplitudes (Ml) -> sa
    for (int i = 0; i < L; i++) {
        my_imbe_param.sa[i] = (Word16)Ml[i];
    }

	//
	// Pre-calculate values needed by synthesis that is typically done during sa decode
	//
	
	my_imbe_param.div_one_by_num_harm_sh = norm_s(my_imbe_param.num_harms);
	my_imbe_param.div_one_by_num_harm = div_s(0x4000, my_imbe_param.num_harms << my_imbe_param.div_one_by_num_harm_sh);
    my_imbe_param.l_uv = 0;
    for (i = 0; i < my_imbe_param.num_harms; i++) {
        if (my_imbe_param.v_uv_dsn[i] == 0)
			my_imbe_param.l_uv++;
    }

	//
	// Perform fixed-point synthesis
	//

    sa_enh(&my_imbe_param);
    v_synt(&my_imbe_param, snd_v);
    uv_synt(&my_imbe_param, snd_uv);

    for(i = 0; i < FRAME; i++)
		snd[i] = add(snd_v[i], snd_uv[i]);
}