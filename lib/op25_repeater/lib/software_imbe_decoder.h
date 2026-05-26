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
 * Runtime-tunable knobs for the trunk-recorder-specific quality improvements
 * layered on the TIA-102.BABA-A IMBE reference decoder. Defaults match the
 * "best so far" values established empirically; pass a modified instance to
 * software_imbe_decoder::set_params() to override at runtime (used by the
 * imbe_tune utility for parameter sweeps).
 *
 * See docs/Notes/VOCODER-IMPROVEMENTS.md and the AUDIO TUNING PARAMETERS
 * comment block at the top of software_imbe_decoder.cc for the patent-derived
 * rationale, target ranges, and effect of each field.
 */
struct VocoderParams {
	// Formant postfilter (US5241650, expired ~2009).
	//   alpha 0=off, 0.25 mild, 0.5 aggressive; w = (2*w+1)-tap smoothing.
	float fmt_alpha             = 0.22f;
	int   fmt_w                 = 5;

	// Voiced phase regeneration (US5701390, expired Feb 2015).
	float phase_c_env           = 0.90f;
	float phase_w_rand          = 0.15f;
	float phase_low_blend       = 0.85f;
	int   phase_kernel_d        = 19;
	float phase_kernel_gamma    = 0.72f;

	// Voicing-decision median smoothing (US6912496, expired Mar 2023).
	int   voicing_smooth_taps   = 3;

	// UV->V phase reset (US6963833, expired Mar 2022).
	bool  uv_to_v_reset         = true;

	// Subframe-style amp/freq/phase interpolation (US6131084, expired ~2017).
	int   interp_max_l          = 12;
	float interp_pitch_tol      = 0.15f;

	// Repeated-frame amplitude decay (compounds across repeats).
	float repeat_amplitude_decay = 0.85f;
};

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
	 * Reset all cross-frame state so a new call starts from a clean slate.
	 * Must be called between calls; otherwise stale ER, vee_history, phase,
	 * and spectral state from the previous call leak in and can leave the
	 * gating stuck in mute (producing fully-silent output files).
	 */
	void clear();

	/**
	 * Replace the tuning parameters used by synthesis. Does not reset
	 * cross-frame state; call clear() too if you want a clean slate. Safe
	 * to call between frames; takes effect on the next decoded frame.
	 */
	void set_params(const VocoderParams& p) { params_ = p; }
	const VocoderParams& get_params() const { return params_; }

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
	uint32_t voiced_phase_seed;	// xorshift32 state for per-frame voiced phase regen
	int vee_history[57][4];		// past voicing decisions per harmonic (newest at [0])
	VocoderParams params_;		// runtime tuning; defaults set in struct

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
	void apply_formant_postfilter();
	void smooth_voicing_decisions();
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
