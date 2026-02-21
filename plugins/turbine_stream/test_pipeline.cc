// Test program: reads an 8kHz 16-bit mono WAV from trunk-recorder,
// runs it through the same upsampler + Opus encode pipeline as the plugin,
// then decodes back to PCM and writes an output WAV for comparison.
//
// Build: g++ -o test_pipeline test_pipeline.cc -lopus -lm
// Usage: ./test_pipeline input.wav output.wav

#include <opus/opus.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- Upsampler (identical to plugin) ----
struct Upsampler {
  int factor;
  int taps_per_phase;
  std::vector<std::vector<float>> coeffs;
  std::vector<float> history;

  Upsampler() : factor(1), taps_per_phase(0) {}

  void init(int upsample_factor, int num_taps_per_phase = 24) {
    factor = upsample_factor;
    if (factor <= 1) { factor = 1; return; }
    taps_per_phase = num_taps_per_phase;
    int total_taps = factor * taps_per_phase;

    std::vector<float> h(total_taps);
    float center = (total_taps - 1) / 2.0f;
    for (int n = 0; n < total_taps; n++) {
      float x = n - center;
      float sinc_val;
      if (fabsf(x) < 1e-6f) {
        sinc_val = 1.0f;
      } else {
        sinc_val = sinf((float)M_PI * x / factor) / ((float)M_PI * x / factor);
      }
      float t = 2.0f * (float)M_PI * n / (total_taps - 1);
      float window = 0.42f - 0.5f * cosf(t) + 0.08f * cosf(2.0f * t);
      h[n] = (float)factor * sinc_val * window;
    }

    coeffs.resize(factor);
    for (int p = 0; p < factor; p++) {
      coeffs[p].resize(taps_per_phase);
      for (int t = 0; t < taps_per_phase; t++) {
        coeffs[p][t] = h[p + t * factor];
      }
    }
    history.assign(taps_per_phase, 0.0f);
  }

  void process(const int16_t *input, int count, std::vector<float> &output) {
    if (factor <= 1) {
      for (int i = 0; i < count; i++)
        output.push_back(input[i] / 32768.0f);
      return;
    }
    output.reserve(output.size() + count * factor);
    for (int i = 0; i < count; i++) {
      for (int t = 0; t < taps_per_phase - 1; t++)
        history[t] = history[t + 1];
      history[taps_per_phase - 1] = input[i] / 32768.0f;

      for (int p = 0; p < factor; p++) {
        float sum = 0.0f;
        for (int t = 0; t < taps_per_phase; t++)
          sum += coeffs[p][t] * history[taps_per_phase - 1 - t];
        output.push_back(sum);
      }
    }
  }
};

// ---- Minimal WAV reader/writer ----
struct WavHeader {
  char riff[4];
  uint32_t file_size;
  char wave[4];
  char fmt[4];
  uint32_t fmt_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data[4];
  uint32_t data_size;
};

bool read_wav(const char *path, std::vector<int16_t> &samples, uint32_t &sample_rate) {
  FILE *f = fopen(path, "rb");
  if (!f) { perror(path); return false; }

  WavHeader hdr;
  if (fread(&hdr, sizeof(hdr), 1, f) != 1) { fclose(f); return false; }

  if (memcmp(hdr.riff, "RIFF", 4) || memcmp(hdr.wave, "WAVE", 4)) {
    fprintf(stderr, "Not a WAV file\n");
    fclose(f);
    return false;
  }

  // Skip to 'data' chunk if fmt chunk is larger than expected
  if (hdr.fmt_size > 16) {
    fseek(f, 20 + hdr.fmt_size, SEEK_SET); // skip past fmt chunk
    // Read data chunk header
    char chunk_id[4];
    uint32_t chunk_size;
    while (fread(chunk_id, 4, 1, f) == 1 && fread(&chunk_size, 4, 1, f) == 1) {
      if (memcmp(chunk_id, "data", 4) == 0) {
        hdr.data_size = chunk_size;
        break;
      }
      fseek(f, chunk_size, SEEK_CUR);
    }
  }

  sample_rate = hdr.sample_rate;
  int num_samples = hdr.data_size / (hdr.bits_per_sample / 8);
  samples.resize(num_samples);
  fread(samples.data(), sizeof(int16_t), num_samples, f);
  fclose(f);

  printf("Read %s: %d samples, %u Hz, %d-bit\n", path, num_samples, sample_rate, hdr.bits_per_sample);
  return true;
}

bool write_wav(const char *path, const std::vector<float> &samples, uint32_t sample_rate) {
  FILE *f = fopen(path, "wb");
  if (!f) { perror(path); return false; }

  uint32_t num_samples = samples.size();
  uint32_t data_size = num_samples * 2;

  WavHeader hdr;
  memcpy(hdr.riff, "RIFF", 4);
  hdr.file_size = 36 + data_size;
  memcpy(hdr.wave, "WAVE", 4);
  memcpy(hdr.fmt, "fmt ", 4);
  hdr.fmt_size = 16;
  hdr.audio_format = 1; // PCM
  hdr.num_channels = 1;
  hdr.sample_rate = sample_rate;
  hdr.byte_rate = sample_rate * 2;
  hdr.block_align = 2;
  hdr.bits_per_sample = 16;
  memcpy(hdr.data, "data", 4);
  hdr.data_size = data_size;

  fwrite(&hdr, sizeof(hdr), 1, f);

  for (uint32_t i = 0; i < num_samples; i++) {
    float s = samples[i];
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    int16_t v = (int16_t)(s * 32767.0f);
    fwrite(&v, 2, 1, f);
  }

  fclose(f);
  printf("Wrote %s: %u samples at %u Hz\n", path, num_samples, sample_rate);
  return true;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s input.wav output.wav [encode_rate]\n", argv[0]);
    fprintf(stderr, "  encode_rate: 8000 or 24000 (default: 24000)\n");
    return 1;
  }

  int encode_rate = 24000;
  if (argc >= 4) encode_rate = atoi(argv[3]);

  // Read input WAV
  std::vector<int16_t> input_samples;
  uint32_t input_rate;
  if (!read_wav(argv[1], input_samples, input_rate)) return 1;

  // Determine upsample factor
  int upsample_factor = 1;
  if (encode_rate > (int)input_rate && encode_rate % input_rate == 0) {
    upsample_factor = encode_rate / input_rate;
  } else if (encode_rate != (int)input_rate) {
    fprintf(stderr, "Cannot resample %u -> %d\n", input_rate, encode_rate);
    return 1;
  }

  printf("Pipeline: %u Hz -> %dx upsample -> Opus %d Hz -> decode -> WAV\n",
         input_rate, upsample_factor, encode_rate);

  // Upsample
  Upsampler up;
  up.init(upsample_factor);

  std::vector<float> upsampled;
  up.process(input_samples.data(), input_samples.size(), upsampled);
  printf("Upsampled: %zu samples at %d Hz (%.2f seconds)\n",
         upsampled.size(), encode_rate, (double)upsampled.size() / encode_rate);

  // Also write the upsampled (pre-encode) audio for comparison
  {
    std::string pre_path = std::string(argv[2]) + ".pre.wav";
    write_wav(pre_path.c_str(), upsampled, encode_rate);
  }

  // Create Opus encoder (same settings as plugin)
  int err;
  OpusEncoder *enc = opus_encoder_create(encode_rate, 1, OPUS_APPLICATION_VOIP, &err);
  if (err != OPUS_OK) {
    fprintf(stderr, "Encoder create failed: %s\n", opus_strerror(err));
    return 1;
  }
  opus_encoder_ctl(enc, OPUS_SET_BITRATE(OPUS_AUTO));
  opus_encoder_ctl(enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
  opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(20));
  opus_encoder_ctl(enc, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_NARROWBAND));

  // Create Opus decoder at the same rate
  OpusDecoder *dec = opus_decoder_create(encode_rate, 1, &err);
  if (err != OPUS_OK) {
    fprintf(stderr, "Decoder create failed: %s\n", opus_strerror(err));
    return 1;
  }

  int frame_ms = 40;
  int frame_size = encode_rate * frame_ms / 1000;
  printf("Frame size: %d samples (%d ms)\n", frame_size, frame_ms);

  // Encode + decode frame by frame
  std::vector<float> decoded_all;
  std::vector<unsigned char> opus_buf(4000);
  std::vector<float> decode_buf(frame_size);

  int total_frames = 0;
  int total_bytes = 0;
  size_t pos = 0;

  while (pos + frame_size <= upsampled.size()) {
    int encoded = opus_encode_float(enc, &upsampled[pos], frame_size,
                                    opus_buf.data(), opus_buf.size());
    if (encoded < 0) {
      fprintf(stderr, "Encode error at frame %d: %s\n", total_frames, opus_strerror(encoded));
      pos += frame_size;
      continue;
    }

    int decoded = opus_decode_float(dec, opus_buf.data(), encoded,
                                    decode_buf.data(), frame_size, 0);
    if (decoded < 0) {
      fprintf(stderr, "Decode error at frame %d: %s\n", total_frames, opus_strerror(decoded));
      pos += frame_size;
      continue;
    }

    decoded_all.insert(decoded_all.end(), decode_buf.begin(), decode_buf.begin() + decoded);
    total_frames++;
    total_bytes += encoded;
    pos += frame_size;
  }

  printf("Encoded %d frames, avg %.1f bytes/frame (%.1f kbps)\n",
         total_frames, (double)total_bytes / total_frames,
         (double)total_bytes * 8.0 / ((double)total_frames * frame_ms / 1000.0) / 1000.0);

  // Write output WAV
  write_wav(argv[2], decoded_all, encode_rate);

  opus_encoder_destroy(enc);
  opus_decoder_destroy(dec);

  printf("\nCompare:\n");
  printf("  Original:   %s (%u Hz)\n", argv[1], input_rate);
  printf("  Upsampled:  %s.pre.wav (%d Hz)\n", argv[2], encode_rate);
  printf("  Roundtrip:  %s (%d Hz)\n", argv[2], encode_rate);

  return 0;
}
