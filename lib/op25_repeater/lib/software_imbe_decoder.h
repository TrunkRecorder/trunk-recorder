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
 * layered on the TIA-102.BABA-A IMBE reference decoder.
 *
 * Defaults are mathematically-principled starting points that match the
 * post-cleanup implementation (proper Hilbert kernel, DC-removed log-magnitude
 * input, voiced-only postfilter, ER-gated voicing smoothing). They are not
 * necessarily the perceptual sweet spot for any given system - run
 * utils/imbe_tune against captured .imbe files to find the best values for
 * your audio.
 *
 * See docs/Notes/VOCODER-IMPROVEMENTS.md for the patent-derived rationale,
 * target ranges, and effect of each field. Re-tuning notes for the post-
 * cleanup implementation are in the same doc under "Re-tuning note".
 */
struct VocoderParams {
	// -- Formant postfilter (US5241650, expired ~2009) ------------------------
	// Magnitude-domain contrast emphasis on voiced harmonics:
	//   M'[l] = M[l] * 2^(alpha * (log2 M[l] - log2 M_smooth[l]))
	// Applied only to voiced bands; edge harmonics use symmetric reflection.

	// Emphasis strength. 0 = off, 0.25 = mild (recommended baseline),
	// 0.35 = noticeable formant shape, 0.5+ = tinny / sibilant.
	float fmt_alpha             = 0.40f;
	// Half-width of the smoothing window (window = 2W+1 harmonics).
	// 3 = 7-tap (narrow), 5 = 11-tap = ~1 formant wide (recommended), 7 = wider.
	int   fmt_w                 = 5;

	// -- Voiced phase regeneration (US5701390, expired Feb 2015) ---------------
	// Phase from discrete Hilbert transform of log-magnitude:
	//   phi(l) = c_env * Sum_{m odd, 1..D} (2/(pi*m)) * (B[l+m] - B[l-m])
	// where B = log2(M) - mean(log2 M).

	// Envelope-phase scaling. The kernel already includes 2/pi; this is an
	// additional multiplier. 0 = falls back to pure linear phase (buzzy).
	// 0.65 = patent's natural scale (recommended baseline), 1.0+ = stronger
	// shaping (risks "echoey" on sustained vowels).
	float phase_c_env           = 1.00f;
	// Residual TIA-eq.142 random phase weight. 0 = pure deterministic per
	// the patent (recommended baseline). Raise to 0.05-0.15 if some random
	// jitter helps on locally-flat spectra.
	float phase_w_rand          = 0.10f;
	// Envelope-phase weight on LOW harmonics (l <= L/4). Low harmonics carry
	// glottal-pulse alignment that makes voiced speech sound "alive".
	//   0   = pure linear phase, most glottal/buzzy
	//   0.5 = balanced (recommended baseline)
	//   1.0 = same envelope weight as high harmonics, least glottal
	float phase_low_blend       = 0.85f;
	// Kernel half-length (full length = 2*D+1 taps). Patent's preferred.
	int   phase_kernel_d        = 13;
	// Boundary-extension geometric decay outside [1, L]. Patent value.
	float phase_kernel_gamma    = 0.72f;

	// -- Voicing-decision median smoothing (US6912496, expired Mar 2023) ------
	// N-tap majority median on vee[l][New] using current + past frames.
	// Only fires when smoothed ER >= threshold, so clean audio keeps snappy
	// V/UV transitions and only noisy audio gets de-chattered.

	// Filter length. 1 = off, 3 = drop single-frame outliers (recommended),
	// 5 = smoother but voicing-state changes lag 2 frames.
	int   voicing_smooth_taps        = 3;
	// Minimum smoothed ER for smoothing to fire. 0 = always-on, 0.01 = light
	// gating (recommended), 0.03 = only fire on quite noisy frames.
	float voicing_smooth_er_threshold = 0.01f;

	// -- UV->V phase reset (US6963833, expired Mar 2022) ----------------------
	// On fully-unvoiced -> voiced transition, reset psi1 to 0 and pre-seed
	// phi[l][Old] from a 56-entry table of deliberately misaligned per-
	// harmonic offsets so onset frames never start with all-aligned phases.
	bool  uv_to_v_reset         = true;

	// -- Subframe-style interpolation (US6131084, expired ~2017) --------------
	// Loosens the TIA fine-transition gate (ell < 8, |dw0|/w0 < 0.1) so the
	// quadratic-phase + linear-amplitude smooth path runs on more frames.

	// Max harmonic eligible for fine transition. 8 = TIA spec, 12 =
	// recommended (smoother sustained vowels), 16 = smoothest (may smear
	// fast consonant transitions).
	int   interp_max_l          = 12;
	// Max |w0 - Oldw0| / w0 ratio for fine transition. 0.10 = TIA spec,
	// 0.15 = recommended (catches normal pitch wobble), 0.20 = includes
	// vibrato (risks smearing real pitch jumps).
	float interp_pitch_tol      = 0.15f;

	// -- Repeated-frame amplitude decay ---------------------------------------
	// On the repeat path, M[l][New] = decay * M[l][Old] each frame.
	// Compounds across consecutive repeats so a tail of marginal frames
	// fades before the 4-frame mute kicks in.
	//   1.00 = no decay (sustained tones)
	//   0.85 = recommended (~61% after 3 repeats)
	//   0.70 = aggressive (~34% after 3)
	float repeat_amplitude_decay = 0.95f;
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
	 * Multi-pass / offline support: capture the per-band voicing decision
	 * of the last decoded frame (post-smoothing, post-override). out[1..56]
	 * are populated; out[0] = 0.
	 */
	void get_decoded_voicing(int out[57]) const;

	/**
	 * Multi-pass / offline support: pre-set the voicing decision for the
	 * NEXT call to decode_fullrate. Override is applied AFTER
	 * smooth_voicing_decisions and BEFORE compute_envelope_phases /
	 * postfilter / synth, so it controls what the synthesizer sees.
	 * Consumed (cleared) after one decode call. Use in a pass-2 decode
	 * with externally-smoothed voicing from a pass-1 capture.
	 *
	 * in[1..56] are read; in[0] is ignored.
	 */
	void set_voicing_override(const int in[57]);

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
	uint32_t unvoiced_noise_state;	// full 32-bit xorshift32 state for unvoiced excitation
	int vee_history[57][4];		// past voicing decisions per harmonic (newest at [0])
	int vee_override_[57];		// one-shot override of vee[][New] for offline multi-pass
	bool vee_override_active_;	// one-shot flag; consumed by decode_fullrate
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
	void compute_envelope_phases();
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
