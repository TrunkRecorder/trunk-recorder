/* -*- C++ -*- */

/*
 * Copyright 2008-2009 Steve Glass
 *
 * This file is part of OP25.
 *
 * OP25 is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or(at your option)
 * any later version.
 *
 * OP25 is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OP25; see the file COPYING. If not, write to the Free
 * Software Foundation, Inc., 51 Franklin Street, Boston, MA
 * 02110-1301, USA.
 */

#ifndef INCLUDED_SOFTWARE_IMBE_DECODER_H
#define INCLUDED_SOFTWARE_IMBE_DECODER_H

#include "imbe_decoder.h"

#include <stdint.h>

/**
 * A software implementation of the imbe_decoder interface.
 */
class software_imbe_decoder : public imbe_decoder {
public:

	/**
	 * Default constructor for the software_imbe_decoder.
	 */
	software_imbe_decoder();

	/**
	 * Destructor for the software_imbe_decoder.
	 */
	virtual ~software_imbe_decoder();

	/**
	 * Decode the compressed audio.
	 *
	 * \cw in IMBE codeword (including parity check bits).
	 */
	virtual void decode(int16_t samples[IMBE_SAMPLES_PER_FRAME], const voice_codeword& cw);

	void decode_fullrate(int16_t samples[IMBE_SAMPLES_PER_FRAME], uint32_t u0, uint32_t u1, uint32_t u2, uint32_t u3, uint32_t u4, uint32_t u5, uint32_t u6, uint32_t u7, uint32_t E0, uint32_t ET);
	void decode_tap(int16_t samples[IMBE_SAMPLES_PER_FRAME], int _L, int _K, float _w0, const int * _v, const float * _mu);
	void decode_tone(int16_t samples[IMBE_SAMPLES_PER_FRAME], int _ID, int _AD, int * _n);
private:

	//NOTE: Single-letter variable names are upper case only; Lower
	//				  case if needed is spelled. e.g. L, ell

	float ER;					// BER Estimate
	int rpt_ctr;				// Frame repeat counter

	int bee[58];				// Encoded Spectral Amplitudes
	float M[57][2];				// Enhanced Spectral Amplitudes
	float Mu[57][2];			// Unenhanced Spectral Amplitudes
	int vee[57][2];				// V/UV decisions
	float suv[160];				// Unvoiced samples
	float sv[160];				// Voiced samples
	float log2Mu[58][2];
	float Olduw[256];
	float psi1;
	float phi[57][2];
	uint32_t u[211];

	int Old;
	int New;
	int L;
	int OldL;
	float w0;
	float Oldw0;
	float Luv;						//number of unvoiced spectral amplitudes

	char sym_b[4096];
	char RxData[4096];
	int sym_bp;
	int ErFlag;

	uint32_t pngen15(uint32_t& pn);
	uint32_t pngen23(uint32_t& pn);
	uint32_t next_u(uint32_t u);
	void decode_spectral_amplitudes(int, int );
	void decode_vuv(int );
	void adaptive_smoothing(float, float );
	void fft(float i[], float q[]);
	void enhance_spectral_amplitudes(float&);
	void ifft(float i[], float q[], float[]);
	uint16_t rearrange(uint32_t u0, uint32_t u1, uint32_t u2, uint32_t u3, uint32_t u4, uint32_t u5, uint32_t u6, uint32_t u7);
	void synth_unvoiced();
	void synth_voiced();
	void unpack(uint8_t *buf, uint32_t& u0, uint32_t& u1, uint32_t& u2, uint32_t& u3, uint32_t& u4, uint32_t& u5, uint32_t& u6, uint32_t& u7, uint32_t& E0, uint32_t& ET);
	int repeat_last();
};


#endif /* INCLUDED_SOFTWARE_IMBE_DECODER_H */
