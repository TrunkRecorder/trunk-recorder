/*
 * vocoder_monitor: per-transmission reception and vocoder statistics for P25
 * Phase 1 voice, plus optional raw IMBE frame capture. See vocoder_monitor.h.
 */

#include "vocoder_monitor.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <mutex>

namespace {

// One stats file per process, shared by every recorder's decoder thread.
std::mutex g_stats_mutex;
FILE *g_stats_file = nullptr;
bool g_stats_checked = false;

FILE *stats_file() {
	// caller holds g_stats_mutex
	if (!g_stats_checked) {
		g_stats_checked = true;
		const char *p = getenv("OP25_VOCODER_STATS");
		if (p && p[0]) {
			g_stats_file = fopen(p, "a");
			if (!g_stats_file)
				fprintf(stderr, "[vocoder stats] cannot open %s: %s\n", p, strerror(errno));
		}
	}
	return g_stats_file;
}

double now_s() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec + tv.tv_usec / 1e6;
}

// Frames with this much FEC activity count toward an error burst.
bool bad_frame(uint32_t E0, uint32_t ET) { return E0 >= 2 || ET >= 6; }

void add_burst(uint32_t hist[6], uint32_t len) {
	if (len == 0)
		return;
	int bin = len == 1 ? 0 : len == 2 ? 1 : len == 3 ? 2 : len < 8 ? 3 : len < 16 ? 4 : 5;
	hist[bin]++;
}

} // namespace

vocoder_monitor::vocoder_monitor()
	: d_stats_on(false), d_capture_on(false), d_active(false), d_capture(nullptr)
{
	const char *s = getenv("OP25_VOCODER_STATS");
	d_stats_on = (s && s[0]);
	const char *c = getenv("OP25_IMBE_CAPTURE_DIR");
	if (c && c[0]) {
		d_capture_on = true;
		d_capture_dir = c;
	}
}

vocoder_monitor::~vocoder_monitor()
{
	end_transmission();
}

void vocoder_monitor::begin(uint16_t tgid, uint32_t nac, double now)
{
	d_active = true;
	d_t_start = d_t_last = now;
	d_tgid = tgid;
	d_nac = nac;
	d_src = -1;
	d_frames = d_enc_frames = 0;
	memset(d_e0_hist, 0, sizeof(d_e0_hist));
	memset(d_et_hist, 0, sizeof(d_et_hist));
	d_et_sum = 0;
	memset(d_status, 0, sizeof(d_status));
	memset(d_cause, 0, sizeof(d_cause));
	d_mute_runs = d_cur_mute_run = d_max_mute_run = 0;
	d_cur_rep_run = d_max_rep_run = 0;
	memset(d_burst_hist, 0, sizeof(d_burst_hist));
	d_cur_burst = 0;
	d_b0_silence = d_b0_invalid = 0;
	d_er_sum = d_er_max = 0;
	d_pitch_n = 0;
	d_f0_sum = d_f0_sumsq = 0;
	d_pitch_jumps = d_octave_jumps = 0;
	d_prev_f0 = 0;
	d_prev_voiced = false;
	d_vf_sum = d_vf_change = d_prev_vf = 0;
	d_vf_n = 0;
	d_L_sum = 0;
	d_active_frames = 0;
	d_active_sumsq = 0;
	d_active_samples = 0;
	d_peak = 0;
	d_clip_samples = 0;

	d_capture_name.clear();
	if (d_capture_on) {
		char fname[1024];
		snprintf(fname, sizeof(fname), "%s/p25imbe_tg%u_%llu.imbe", d_capture_dir.c_str(), (unsigned)tgid,
		         (unsigned long long)(now * 1000.0));
		d_capture = fopen(fname, "wb");
		if (d_capture) {
			const char magic[8] = {'P', '2', '5', 'I', 'M', 'B', 'E', '\0'};
			uint32_t ver = 1, reserved = 0;
			fwrite(magic, 1, 8, d_capture);
			fwrite(&ver, 4, 1, d_capture);
			fwrite(&reserved, 4, 1, d_capture);
			const char *base = strrchr(fname, '/');
			d_capture_name = base ? base + 1 : fname;
		} else {
			fprintf(stderr, "[IMBE capture] cannot open %s: %s - capture disabled\n", fname, strerror(errno));
			d_capture_on = false;
		}
	}
}

void vocoder_monitor::frame(uint16_t tgid, uint32_t nac, long src, const uint32_t u[8], uint32_t E0, uint32_t ET,
                            bool encrypted, const ImbeFrameInfo *info, const int16_t *snd, int nsnd)
{
	if (!enabled())
		return;
	double now = now_s();
	if (d_active && (tgid != d_tgid || now - d_t_last > 1.0))
		end_transmission();
	if (!d_active)
		begin(tgid, nac, now);
	d_t_last = now;
	if (src > 0)
		d_src = src;
	d_frames++;

	if (encrypted) {
		d_enc_frames++;
		return;
	}

	if (d_capture) {
		uint32_t rec[10] = {u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7], E0, ET};
		fwrite(rec, 4, 10, d_capture);
	}
	if (!d_stats_on)
		return;

	// channel
	d_e0_hist[E0 > 3 ? 3 : E0]++;
	d_et_hist[ET > 16 ? 16 : ET]++;
	d_et_sum += ET;
	if (bad_frame(E0, ET)) {
		d_cur_burst++;
	} else {
		add_burst(d_burst_hist, d_cur_burst);
		d_cur_burst = 0;
	}
	// u7 from imbe_header_decode() is shifted left by one
	int b0 = ((u[0] >> 4) & 0xfc) | ((u[7] >> 2) & 0x3);
	if (b0 >= 216 && b0 <= 219)
		d_b0_silence++;
	else if (b0 > 207)
		d_b0_invalid++;

	if (!info)
		return;

	// concealment
	int st = info->status < 0 || info->status > 2 ? 0 : info->status;
	d_status[st]++;
	for (int i = 0; i < 5; i++)
		if (info->cause & (1 << i))
			d_cause[i]++;
	if (st == ImbeFrameInfo::MUTED) {
		if (d_cur_mute_run == 0)
			d_mute_runs++;
		d_cur_mute_run++;
		if (d_cur_mute_run > d_max_mute_run)
			d_max_mute_run = d_cur_mute_run;
	} else {
		d_cur_mute_run = 0;
	}
	if (st == ImbeFrameInfo::REPEATED) {
		d_cur_rep_run++;
		if (d_cur_rep_run > d_max_rep_run)
			d_max_rep_run = d_cur_rep_run;
	} else {
		d_cur_rep_run = 0;
	}
	d_er_sum += info->er;
	if (info->er > d_er_max)
		d_er_max = info->er;

	// pitch and voicing, from frames that were actually decoded
	if (st == ImbeFrameInfo::DECODED && b0 <= 207) {
		double f0 = 16000.0 / (b0 + 39.5); // w0 = 4*pi/(b0+39.5) rad/sample at 8 kHz
		bool voiced = true;
		if (info->n_voiced >= 0 && info->L > 0) {
			double vf = (double)info->n_voiced / info->L;
			if (d_vf_n > 0)
				d_vf_change += fabs(vf - d_prev_vf);
			d_prev_vf = vf;
			d_vf_sum += vf;
			d_vf_n++;
			d_L_sum += info->L;
			voiced = vf >= 0.5;
		}
		if (voiced) {
			d_pitch_n++;
			d_f0_sum += f0;
			d_f0_sumsq += f0 * f0;
			if (d_prev_voiced && d_prev_f0 > 0) {
				double r = f0 / d_prev_f0;
				if (fabs(r - 1.0) > 0.25)
					d_pitch_jumps++;
				if (fabs(r - 2.0) < 0.15 || fabs(r - 0.5) < 0.075)
					d_octave_jumps++;
			}
			d_prev_f0 = f0;
		}
		d_prev_voiced = voiced;
	} else {
		d_prev_voiced = false;
	}

	// output audio
	if (snd && nsnd > 0) {
		double ss = 0;
		for (int i = 0; i < nsnd; i++) {
			int v = snd[i] < 0 ? -snd[i] : snd[i];
			if (v > d_peak)
				d_peak = v;
			if (v >= 32767)
				d_clip_samples++;
			ss += (double)snd[i] * snd[i];
		}
		if (sqrt(ss / nsnd) > 100.0) { // about -50 dBFS
			d_active_frames++;
			d_active_sumsq += ss;
			d_active_samples += nsnd;
		}
	}
}

void vocoder_monitor::end_transmission()
{
	if (!d_active)
		return;
	add_burst(d_burst_hist, d_cur_burst);
	d_cur_burst = 0;
	if (d_capture) {
		fclose(d_capture);
		d_capture = nullptr;
	}
	if (d_stats_on)
		write_stats();
	d_active = false;
}

void vocoder_monitor::write_stats()
{
	double dur = d_frames * 0.02;
	double f0_mean = d_pitch_n ? d_f0_sum / d_pitch_n : 0.0;
	double f0_sd = d_pitch_n > 1 ? sqrt(fmax(0.0, d_f0_sumsq / d_pitch_n - f0_mean * f0_mean)) : 0.0;
	double rms_db = d_active_samples ? 20.0 * log10(sqrt(d_active_sumsq / d_active_samples) / 32768.0) : -99.0;
	uint32_t decoded_n = d_status[0] + d_status[1] + d_status[2];

	char buf[2048];
	int n = snprintf(buf, sizeof(buf),
		"{\"t\":%.3f,\"t_end\":%.3f,\"tg\":%u,\"nac\":%u,\"src\":%ld,\"frames\":%u,\"enc_frames\":%u,"
		"\"e0\":[%u,%u,%u,%u],"
		"\"et\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],\"et_sum\":%llu,"
		"\"bursts\":[%u,%u,%u,%u,%u,%u],"
		"\"decoded\":%u,\"repeated\":%u,\"muted\":%u,"
		"\"cause\":{\"b0\":%u,\"e0\":%u,\"et\":%u,\"er\":%u,\"maxrep\":%u},"
		"\"mute_runs\":%u,\"max_mute_run\":%u,\"max_rep_run\":%u,"
		"\"b0_silence\":%u,\"b0_invalid\":%u,\"er_mean\":%.5f,\"er_max\":%.5f,"
		"\"f0_mean\":%.1f,\"f0_sd\":%.1f,\"pitch_frames\":%u,\"pitch_jumps\":%u,\"octave_jumps\":%u,"
		"\"voiced_frac\":%.3f,\"vuv_change_per_s\":%.3f,\"L_mean\":%.1f,"
		"\"rms_dbfs\":%.1f,\"active_frames\":%u,\"peak\":%d,\"clip_samples\":%u,\"capture\":\"%s\"}\n",
		d_t_start, d_t_last + 0.02, (unsigned)d_tgid, (unsigned)d_nac, d_src, d_frames, d_enc_frames,
		d_e0_hist[0], d_e0_hist[1], d_e0_hist[2], d_e0_hist[3],
		d_et_hist[0], d_et_hist[1], d_et_hist[2], d_et_hist[3], d_et_hist[4], d_et_hist[5], d_et_hist[6],
		d_et_hist[7], d_et_hist[8], d_et_hist[9], d_et_hist[10], d_et_hist[11], d_et_hist[12], d_et_hist[13],
		d_et_hist[14], d_et_hist[15], d_et_hist[16], (unsigned long long)d_et_sum,
		d_burst_hist[0], d_burst_hist[1], d_burst_hist[2], d_burst_hist[3], d_burst_hist[4], d_burst_hist[5],
		d_status[0], d_status[1], d_status[2],
		d_cause[0], d_cause[1], d_cause[2], d_cause[3], d_cause[4],
		d_mute_runs, d_max_mute_run, d_max_rep_run,
		d_b0_silence, d_b0_invalid, decoded_n ? d_er_sum / decoded_n : 0.0, d_er_max,
		f0_mean, f0_sd, d_pitch_n, d_pitch_jumps, d_octave_jumps,
		d_vf_n ? d_vf_sum / d_vf_n : 0.0, dur > 0 ? d_vf_change / dur : 0.0, d_vf_n ? d_L_sum / d_vf_n : 0.0,
		rms_db, d_active_frames, d_peak, d_clip_samples, d_capture_name.c_str());
	if (n <= 0)
		return;

	std::lock_guard<std::mutex> lock(g_stats_mutex);
	FILE *f = stats_file();
	if (f) {
		fwrite(buf, 1, n < (int)sizeof(buf) ? n : sizeof(buf) - 1, f);
		fflush(f);
	}
}
