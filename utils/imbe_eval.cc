// imbe_eval: offline IMBE encode -> P25 channel -> decode harness, for
// measuring vocoder changes against a clean reference.
//
//   imbe_eval enc in.wav out.imbe [CHANNEL] [seed]
//       Encode 8 kHz mono 16-bit WAV with the OP25 fixed-point IMBE encoder,
//       apply the P25 voice FEC (Golay/Hamming + PN), corrupt the 144-bit
//       codewords, FEC-decode, and write u[0..7], E0, ET records in the same
//       .imbe format that OP25_IMBE_CAPTURE_DIR produces from live traffic.
//       CHANNEL is a bit error rate (e.g. 0.02) or a Gilbert-Elliott fading
//       channel "ge:g,b,pgb,pbg": each 20 ms frame is in a good (BER g) or
//       bad (BER b) state, moving good->bad with probability pgb and
//       bad->good with probability pbg per frame. Default: 0 (no errors).
//
//   imbe_eval dec in.imbe out.wav fixed|float
//       Decode a .imbe file (synthetic or live capture) with the fixed-point
//       imbe_vocoder (with frame repeat/mute) or the software_imbe_decoder,
//       exactly as p25p1_fdma calls them. VocoderParams fields can be
//       overridden with VP_<field>=value environment variables.
//
// Build: cmake target imbe_eval. Score the output with utils/score_vocoder.py.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "imbe_vocoder/imbe_vocoder.h"
#include "op25_imbe_frame.h"
#include "software_imbe_decoder.h"

struct Rec {
  uint32_t u[8], E0, ET;
};

static bool read_wav(const char *fn, std::vector<int16_t> &out) {
  FILE *f = fopen(fn, "rb");
  if (!f)
    return false;
  char id[4];
  uint32_t sz;
  if (fread(id, 1, 4, f) != 4 || fread(&sz, 4, 1, f) != 1 || fread(id, 1, 4, f) != 4) {
    fclose(f);
    return false;
  }
  uint16_t ch = 0, bits = 0;
  uint32_t rate = 0;
  while (fread(id, 1, 4, f) == 4 && fread(&sz, 4, 1, f) == 1) {
    if (!memcmp(id, "fmt ", 4)) {
      std::vector<uint8_t> b(sz);
      if (fread(b.data(), 1, sz, f) != sz)
        break;
      memcpy(&ch, &b[2], 2);
      memcpy(&rate, &b[4], 4);
      memcpy(&bits, &b[14], 2);
    } else if (!memcmp(id, "data", 4)) {
      if (bits != 16 || ch != 1 || rate != 8000) {
        fprintf(stderr, "%s: need 8 kHz mono 16-bit (got %u Hz, %u ch, %u bits)\n", fn, rate, ch, bits);
        fclose(f);
        return false;
      }
      out.resize(sz / 2);
      size_t n = fread(out.data(), 2, out.size(), f);
      out.resize(n);
      fclose(f);
      return true;
    } else {
      fseek(f, sz + (sz & 1), SEEK_CUR);
    }
  }
  fclose(f);
  return false;
}

static void write_wav(const char *fn, const std::vector<int16_t> &s) {
  FILE *f = fopen(fn, "wb");
  if (!f)
    return;
  uint32_t data = s.size() * 2, riff = 36 + data, fmtsz = 16, rate = 8000, br = 16000;
  uint16_t pcm = 1, ch = 1, ba = 2, bits = 16;
  fwrite("RIFF", 1, 4, f);
  fwrite(&riff, 4, 1, f);
  fwrite("WAVEfmt ", 1, 8, f);
  fwrite(&fmtsz, 4, 1, f);
  fwrite(&pcm, 2, 1, f);
  fwrite(&ch, 2, 1, f);
  fwrite(&rate, 4, 1, f);
  fwrite(&br, 4, 1, f);
  fwrite(&ba, 2, 1, f);
  fwrite(&bits, 2, 1, f);
  fwrite("data", 1, 4, f);
  fwrite(&data, 4, 1, f);
  fwrite(s.data(), 2, s.size(), f);
  fclose(f);
}

static int do_enc(const char *in, const char *out, const char *channel, unsigned seed) {
  double ber = 0, ge_g = -1, ge_b = 0, ge_pgb = 0, ge_pbg = 0;
  if (!strncmp(channel, "ge:", 3)) {
    if (sscanf(channel + 3, "%lf,%lf,%lf,%lf", &ge_g, &ge_b, &ge_pgb, &ge_pbg) != 4) {
      fprintf(stderr, "bad channel spec %s\n", channel);
      return 2;
    }
  } else {
    ber = atof(channel);
  }
  std::vector<int16_t> pcm;
  if (!read_wav(in, pcm))
    return 1;
  FILE *f = fopen(out, "wb");
  if (!f)
    return 1;
  const char magic[8] = {'P', '2', '5', 'I', 'M', 'B', 'E', '\0'};
  uint32_t ver = 1, reserved = 0;
  fwrite(magic, 1, 8, f);
  fwrite(&ver, 4, 1, f);
  fwrite(&reserved, 4, 1, f);

  imbe_vocoder enc;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  size_t nfr = pcm.size() / 160, flips = 0;
  bool bad = false;
  for (size_t i = 0; i < nfr; i++) {
    if (ge_g >= 0) {
      bad = bad ? (U(rng) >= ge_pbg) : (U(rng) < ge_pgb);
      ber = bad ? ge_b : ge_g;
    }
    int16_t fv[8], snd[160];
    memcpy(snd, &pcm[i * 160], sizeof(snd));
    enc.imbe_encode(fv, snd);
    // The encoder's frame_vector[7] holds the 7 unprotected bits as sent;
    // imbe_header_encode() expects them shifted left by one.
    voice_codeword cw(voice_codeword_sz);
    imbe_header_encode(cw, fv[0], fv[1], fv[2], fv[3], fv[4], fv[5], fv[6], ((uint32_t)fv[7] & 0x7f) << 1);
    if (ber > 0) {
      for (size_t b = 0; b < cw.size(); b++) {
        if (U(rng) < ber) {
          cw[b] = !cw[b];
          flips++;
        }
      }
    }
    Rec r;
    imbe_header_decode(cw, r.u[0], r.u[1], r.u[2], r.u[3], r.u[4], r.u[5], r.u[6], r.u[7], r.E0, r.ET);
    fwrite(&r, sizeof(r), 1, f);
  }
  fclose(f);
  fprintf(stderr, "%zu frames, %zu bit errors (BER %.4f)\n", nfr, flips, nfr ? (double)flips / (nfr * 144.0) : 0.0);
  return 0;
}

static void apply_param_overrides(software_imbe_decoder &dec) {
  VocoderParams vp = dec.get_params();
  auto F = [](const char *n, float &v) { if (const char *e = getenv(n)) v = atof(e); };
  auto I = [](const char *n, int &v) { if (const char *e = getenv(n)) v = atoi(e); };
  F("VP_fmt_alpha", vp.fmt_alpha);
  I("VP_fmt_w", vp.fmt_w);
  F("VP_phase_c_env", vp.phase_c_env);
  F("VP_phase_w_rand", vp.phase_w_rand);
  F("VP_phase_low_blend", vp.phase_low_blend);
  I("VP_phase_kernel", vp.phase_kernel);
  I("VP_phase_kernel_d", vp.phase_kernel_d);
  F("VP_phase_kernel_gamma", vp.phase_kernel_gamma);
  I("VP_voicing_smooth_taps", vp.voicing_smooth_taps);
  F("VP_voicing_smooth_er_threshold", vp.voicing_smooth_er_threshold);
  if (const char *e = getenv("VP_uv_to_v_reset"))
    vp.uv_to_v_reset = atoi(e) != 0;
  I("VP_interp_max_l", vp.interp_max_l);
  F("VP_interp_pitch_tol", vp.interp_pitch_tol);
  F("VP_phase_track", vp.phase_track);
  F("VP_amp_smooth", vp.amp_smooth);
  F("VP_repeat_amplitude_decay", vp.repeat_amplitude_decay);
  F("VP_mute_er", vp.mute_er);
  I("VP_repeat_e0", vp.repeat_e0);
  F("VP_repeat_et_base", vp.repeat_et_base);
  F("VP_repeat_et_slope", vp.repeat_et_slope);
  I("VP_max_repeats", vp.max_repeats);
  I("VP_uv_synth_mode", vp.uv_synth_mode);
  F("VP_uv_smooth_gain", vp.uv_smooth_gain);
  I("VP_uv_xfade", vp.uv_xfade);
  I("VP_onset_ramp_mode", vp.onset_ramp_mode);
  dec.set_params(vp);
}

static int do_dec(const char *in, const char *out, const std::string &mode) {
  if (mode != "fixed" && mode != "float") {
    fprintf(stderr, "mode must be fixed or float\n");
    return 2;
  }
  FILE *f = fopen(in, "rb");
  if (!f)
    return 1;
  char hdr[16];
  if (fread(hdr, 1, 16, f) != 16 || memcmp(hdr, "P25IMBE", 7)) {
    fprintf(stderr, "%s: not a .imbe file\n", in);
    fclose(f);
    return 1;
  }
  software_imbe_decoder sw;
  apply_param_overrides(sw);
  imbe_vocoder fx;
  std::vector<int16_t> pcm;
  Rec r;
  while (fread(&r, sizeof(r), 1, f) == 1) {
    int16_t snd[160];
    if (mode == "float") {
      sw.decode_fullrate(snd, r.u[0], r.u[1], r.u[2], r.u[3], r.u[4], r.u[5], r.u[6], r.u[7], r.E0, r.ET);
    } else {
      int16_t fv[8];
      for (int i = 0; i < 8; i++)
        fv[i] = r.u[i] & 0xFFFF;
      fv[7] >>= 1;
      fx.imbe_decode_checked(fv, r.E0, r.ET, snd);
    }
    pcm.insert(pcm.end(), snd, snd + 160);
  }
  fclose(f);
  write_wav(out, pcm);
  return 0;
}

int main(int argc, char **argv) {
  if (argc >= 4 && !strcmp(argv[1], "enc"))
    return do_enc(argv[2], argv[3], argc > 4 ? argv[4] : "0", argc > 5 ? atoi(argv[5]) : 1);
  if (argc >= 5 && !strcmp(argv[1], "dec"))
    return do_dec(argv[2], argv[3], argv[4]);
  fprintf(stderr, "usage: imbe_eval enc in.wav out.imbe [BER|ge:g,b,pgb,pbg] [seed]\n"
                  "       imbe_eval dec in.imbe out.wav fixed|float\n");
  return 2;
}
