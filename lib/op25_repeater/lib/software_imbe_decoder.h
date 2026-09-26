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

	// Emphasis strength. 0 = off (default). The TIA spectral enhancement in
	// enhance_spectral_amplitudes() is already a formant postfilter; stacking
	// this on top lowered PESQ-NB and DNSMOS at every alpha/W tried
	// (alpha 0.1..0.45, W 3..7).
	float fmt_alpha             = 0.0f;
	// Half-width of the smoothing window (window = 2W+1 harmonics).
	// 3 = 7-tap (narrow), 5 = 11-tap = ~1 formant wide, 7 = wider (only used when fmt_alpha > 0).
	int   fmt_w                 = 7;

	// -- Voiced phase regeneration (US5701390, expired Feb 2015) ---------------
	// Phase from discrete Hilbert transform of log-magnitude:
	//   phi(l) = c_env * Sum_{m odd, 1..D} (2/(pi*m)) * (B[l+m] - B[l-m])
	// where B = log2(M) - mean(log2 M).

	// Envelope-phase scaling. The kernel already includes 2/pi; this is an
	// additional multiplier. 0 = falls back to pure linear phase (buzzy).
	// Envelope phase measured ~+0.03 PESQ-NB over linear phase; 0.7-1.3 are
	// indistinguishable on PESQ/DNSMOS, 2.0 is measurably worse.
	float phase_c_env           = 1.30f;
	// Residual TIA-eq.142 random phase weight, scaled by the unvoiced
	// fraction of the frame. 0 = deterministic per the patent; 0..0.6 made
	// no measurable difference.
	float phase_w_rand          = 0.25f;
	// Envelope-phase weight on LOW harmonics (l <= L/4). Low harmonics carry
	// glottal-pulse alignment that makes voiced speech sound "alive".
	//   0   = pure linear phase, most glottal/buzzy
	//   0.5 = balanced
	//   1.0 = same envelope weight as high harmonics (measured worse)
	float phase_low_blend       = 0.4f;
	// Kernel form. 0 = odd-only discrete Hilbert 2/(pi*m) on mean-removed
	// B with geometric extension past L; 1 = US5701390 as written: 1/m for
	// all m on B = log2(M), B[l>L] = phase_kernel_gamma * B[L], B[0] = 0,
	// B[-l] = B[l]; phase_c_env is then the patent's scale (0.44).
	int   phase_kernel          = 0;
	// Kernel half-length (full length = 2*D+1 taps). Patent's preferred.
	int   phase_kernel_d        = 19;
	// Boundary-extension geometric decay outside [1, L]. Patent value.
	float phase_kernel_gamma    = 0.6f;

	// -- Voicing-decision median smoothing (US6912496, expired Mar 2023) ------
	// N-tap majority median on vee[l][New] using current + past frames.
	// Only fires when smoothed ER >= threshold, so clean audio keeps snappy
	// V/UV transitions and only noisy audio gets de-chattered.

	// Filter length. 1 = off (default), 3 = drop single-frame outliers,
	// 5 = smoother but voicing-state changes lag 2 frames. Measured harmful:
	// 3 taps cost ~0.3 PESQ-NB at 2% BER even when ER-gated.
	int   voicing_smooth_taps        = 1;
	// Minimum smoothed ER for smoothing to fire. 0 = always-on, 0.01 = light
	// gating, 0.03 = only fire on quite noisy frames.
	float voicing_smooth_er_threshold = 0.00f;

	// -- UV->V phase reset (US6963833, expired Mar 2022) ----------------------
	// On fully-unvoiced -> voiced transition, reset psi1 to 0. No measurable
	// effect: onset frames are synthesized from phi[New] alone.
	bool  uv_to_v_reset         = true;

	// -- Subframe-style interpolation (US6131084, expired ~2017) --------------
	// Loosens the TIA fine-transition gate (ell < 8, |dw0|/w0 < 0.1) so the
	// quadratic-phase + linear-amplitude smooth path runs on more frames.

	// Max harmonic eligible for fine transition. 8 = TIA spec (default),
	// 12 = smoother sustained vowels, 16 = smoothest (may smear fast
	// consonant transitions).
	int   interp_max_l          = 8;
	// Max |w0 - Oldw0| / w0 ratio for fine transition. 0.10 = TIA spec
	// (default), 0.15 = catches normal pitch wobble, 0.20 = includes vibrato
	// (risks smearing real pitch jumps).
	float interp_pitch_tol      = 0.1f;

	// -- Repeated-frame amplitude decay ---------------------------------------
	// On the repeat path, M[l][New] = decay * M[l][Old] each frame.
	// Compounds across consecutive repeats so a tail of marginal frames
	// fades before the 4-frame mute kicks in.
	//   1.00 = no decay (sustained tones)
	//   0.85 = ~61% after 3 repeats (no measurable PESQ difference)
	//   0.70 = aggressive (~34% after 3)
	float repeat_amplitude_decay = 1.0f;

	// -- Frame repeat / mute thresholds (TIA-102.BABA-A §7.7-7.8) -------------
	// Repeat when E0 >= repeat_e0 or ET >= repeat_et_base + repeat_et_slope*ER;
	// mute when smoothed ER > mute_er or after max_repeats consecutive repeats.
	float mute_er         = 0.0875f;
	// TIA uses E0 >= 2. Golay(23,12) is a perfect code, so E0 == 2 is almost
	// always a correct decode; repeating it measured worse on both random and
	// fading channels than repeating only at E0 >= 3.
	int   repeat_e0       = 3;
	float repeat_et_base  = 10.0f;
	float repeat_et_slope = 40.0f;
	int   max_repeats     = 4;
};

/**
 * What the decoder did with the last frame, for reception statistics
 * (vocoder_monitor). Shared by the float and fixed-point decoders.
 */
struct ImbeFrameInfo {
	enum Status { DECODED = 0, REPEATED = 1, MUTED = 2 };
	enum Cause {                // why a frame was repeated or muted (bitmask)
		CAUSE_B0     = 1,   // b0 > 207 (invalid pitch / silence / tone)
		CAUSE_E0     = 2,   // Golay errors in u0 at the repeat threshold
		CAUSE_ET     = 4,   // total corrected errors over ET threshold
		CAUSE_ER     = 8,   // smoothed error rate over mute threshold
		CAUSE_MAXREP = 16   // too many consecutive repeats
	};
	int status = DECODED;
	int cause = 0;
	float w0 = 0.0f;            // fundamental, radians/sample (0 if unknown)
	int L = 0;                  // harmonics
	int n_voiced = -1;          // voiced harmonics (-1 if unknown)
	float er = 0.0f;            // smoothed error rate after this frame
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
	 * What happened to the frame most recently passed to decode_fullrate().
	 */
	const ImbeFrameInfo& last_frame_info() const { return last_info_; }

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
	float SE;					// TIA-102.BABA S_E: smoothed spectral energy, persists across frames
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
	VocoderParams params_;
	ImbeFrameInfo last_info_;		// runtime tuning; defaults set in struct

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
