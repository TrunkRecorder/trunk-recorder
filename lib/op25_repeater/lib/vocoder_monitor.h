/*
 * vocoder_monitor: per-transmission reception and vocoder statistics for P25
 * Phase 1 voice, plus optional raw IMBE frame capture for offline replay.
 *
 * Enabled by environment variables (both optional, independent):
 *   OP25_VOCODER_STATS=<file>      append one JSON line per transmission
 *   OP25_IMBE_CAPTURE_DIR=<dir>    write each transmission's frames to
 *                                  <dir>/p25imbe_tg<tg>_<epoch_ms>.imbe
 *                                  (u[0..7], E0, ET per frame, for offline replay)
 *
 * All methods run on the decoder's own thread. A transmission ends on a voice
 * terminator, a talkgroup change, or a gap of more than 1 s between frames.
 */
#ifndef INCLUDED_OP25_VOCODER_MONITOR_H
#define INCLUDED_OP25_VOCODER_MONITOR_H

#include <stdint.h>
#include <stdio.h>
#include <string>

#include "software_imbe_decoder.h"

class vocoder_monitor
{
public:
	vocoder_monitor();
	~vocoder_monitor();

	bool enabled() const { return d_stats_on || d_capture_on; }

	// One call per IMBE frame. u/E0/ET as returned by imbe_header_decode()
	// (after decryption). info/snd are null for frames that were not decoded
	// (encrypted without a key).
	void frame(uint16_t tgid, uint32_t nac, long src, const uint32_t u[8], uint32_t E0, uint32_t ET,
	           bool encrypted, const ImbeFrameInfo *info, const int16_t *snd, int nsnd);

	// Voice terminator seen: close out the current transmission.
	void end_transmission();

private:
	void begin(uint16_t tgid, uint32_t nac, double now);
	void write_stats();

	bool d_stats_on;
	bool d_capture_on;
	std::string d_capture_dir;

	// current transmission
	bool d_active;
	double d_t_start, d_t_last;
	uint16_t d_tgid;
	uint32_t d_nac;
	long d_src;
	FILE *d_capture;
	std::string d_capture_name;

	uint32_t d_frames, d_enc_frames;
	uint32_t d_e0_hist[4];
	uint32_t d_et_hist[17];          // 0..15, 16+
	uint64_t d_et_sum;
	uint32_t d_status[3];            // decoded, repeated, muted
	uint32_t d_cause[5];             // b0, e0, et, er, maxrep (frames with that bit)
	uint32_t d_mute_runs, d_cur_mute_run, d_max_mute_run;
	uint32_t d_cur_rep_run, d_max_rep_run;
	uint32_t d_burst_hist[6];        // bad-frame run lengths: 1, 2, 3, 4-7, 8-15, 16+
	uint32_t d_cur_burst;
	uint32_t d_b0_silence, d_b0_invalid;
	double d_er_sum, d_er_max;

	// pitch / voicing (decoded frames)
	uint32_t d_pitch_n;
	double d_f0_sum, d_f0_sumsq;
	uint32_t d_pitch_jumps, d_octave_jumps;
	double d_prev_f0;
	bool d_prev_voiced;
	double d_vf_sum, d_vf_change, d_prev_vf;
	uint32_t d_vf_n;
	double d_L_sum;

	// output audio
	uint32_t d_active_frames;
	double d_active_sumsq;
	uint64_t d_active_samples;
	int d_peak;
	uint32_t d_clip_samples;
};

#endif /* INCLUDED_OP25_VOCODER_MONITOR_H */
