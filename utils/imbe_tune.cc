// imbe_tune.cc — sweep IMBE vocoder tuning parameters against a captured
// .imbe file and produce one .wav per parameter combination.
//
// Pair with the OP25_IMBE_CAPTURE_DIR feature in p25p1_fdma: capture a real
// call's IMBE frames once, then re-decode them N times here with different
// VocoderParams. Eliminates the live-audio variance that makes A/B tuning
// impossible in production.
//
// Usage:
//   imbe_tune --input call.imbe --sweep sweep.json --output-dir DIR
//
// sweep.json (JSON object: knob name -> list of values):
//   {
//     "fmt_alpha":       [0.15, 0.18, 0.22, 0.25],
//     "phase_low_blend": [0.55, 0.70, 0.85],
//     "phase_w_rand":    [0.0,  0.10, 0.20]
//   }
//
// Output:
//   <output-dir>/combo_NNNN.wav   - audio for combo NNNN
//   <output-dir>/combo_NNNN.json  - param values that produced it
//   <output-dir>/summary.csv      - one row per combo
//
// Recommended follow-up:
//   python3 utils/analyze_vocoder.py <output-dir>/*.wav

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "software_imbe_decoder.h"

extern "C" {
// nlohmann::json is vendored at lib/json.hpp in trunk-recorder.
}
#include "json.hpp"

using json = nlohmann::json;

// -----------------------------------------------------------------------------
// .imbe file I/O — format defined in p25p1_fdma.cc
// header (16 bytes): "P25IMBE\0" + uint32 version + uint32 reserved
// record (40 bytes): u[0..7] (8 u32) + E0 (u32) + ET (u32)
struct ImbeFrame {
    uint32_t u[8];
    uint32_t E0;
    uint32_t ET;
};

static bool read_imbe_file(const std::string& path, std::vector<ImbeFrame>& out)
{
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        std::cerr << "error: cannot open " << path << "\n";
        return false;
    }
    char magic[8];
    if (std::fread(magic, 1, 8, f) != 8 || std::memcmp(magic, "P25IMBE\0", 8) != 0) {
        std::cerr << "error: " << path << " is not a P25IMBE file\n";
        std::fclose(f);
        return false;
    }
    uint32_t version = 0, reserved = 0;
    std::fread(&version, 4, 1, f);
    std::fread(&reserved, 4, 1, f);
    if (version != 1) {
        std::cerr << "warning: unexpected version " << version << "; trying anyway\n";
    }
    ImbeFrame fr;
    while (std::fread(fr.u, 4, 8, f) == 8) {
        if (std::fread(&fr.E0, 4, 1, f) != 1) break;
        if (std::fread(&fr.ET, 4, 1, f) != 1) break;
        out.push_back(fr);
    }
    std::fclose(f);
    return true;
}

// -----------------------------------------------------------------------------
// Minimal 8 kHz 16-bit mono WAV writer.
static void write_wav(const std::string& path, const std::vector<int16_t>& samples)
{
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    const uint32_t sample_rate = 8000;
    const uint16_t channels    = 1;
    const uint16_t bits        = 16;
    const uint16_t block_align = (uint16_t)(channels * bits / 8);
    const uint32_t byte_rate   = sample_rate * block_align;
    const uint16_t fmt_code    = 1;        // PCM
    const uint32_t fmt_size    = 16;
    const uint32_t data_size   = (uint32_t)(samples.size() * sizeof(int16_t));
    const uint32_t riff_size   = 36 + data_size;

    std::fwrite("RIFF", 1, 4, f);
    std::fwrite(&riff_size, 4, 1, f);
    std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f);
    std::fwrite(&fmt_size,    4, 1, f);
    std::fwrite(&fmt_code,    2, 1, f);
    std::fwrite(&channels,    2, 1, f);
    std::fwrite(&sample_rate, 4, 1, f);
    std::fwrite(&byte_rate,   4, 1, f);
    std::fwrite(&block_align, 2, 1, f);
    std::fwrite(&bits,        2, 1, f);
    std::fwrite("data", 1, 4, f);
    std::fwrite(&data_size, 4, 1, f);
    if (!samples.empty()) {
        std::fwrite(samples.data(), 2, samples.size(), f);
    }
    std::fclose(f);
}

// -----------------------------------------------------------------------------
// Apply a key=value sweep dimension onto a VocoderParams.
static bool apply_knob(VocoderParams& p, const std::string& name, const json& v)
{
    if      (name == "fmt_alpha")              p.fmt_alpha = v.get<float>();
    else if (name == "fmt_w")                  p.fmt_w = v.get<int>();
    else if (name == "phase_c_env")            p.phase_c_env = v.get<float>();
    else if (name == "phase_w_rand")           p.phase_w_rand = v.get<float>();
    else if (name == "phase_low_blend")        p.phase_low_blend = v.get<float>();
    else if (name == "phase_kernel_d")         p.phase_kernel_d = v.get<int>();
    else if (name == "phase_kernel_gamma")     p.phase_kernel_gamma = v.get<float>();
    else if (name == "voicing_smooth_taps")    p.voicing_smooth_taps = v.get<int>();
    else if (name == "uv_to_v_reset")          p.uv_to_v_reset = v.get<bool>();
    else if (name == "interp_max_l")           p.interp_max_l = v.get<int>();
    else if (name == "interp_pitch_tol")       p.interp_pitch_tol = v.get<float>();
    else if (name == "repeat_amplitude_decay") p.repeat_amplitude_decay = v.get<float>();
    else {
        std::cerr << "warning: unknown knob '" << name << "' in sweep spec — ignoring\n";
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
struct SweepDim {
    std::string  name;
    std::vector<json> values;
};

static void load_sweep_spec(const std::string& path, std::vector<SweepDim>& dims)
{
    std::ifstream f(path);
    if (!f) { std::cerr << "error: cannot open " << path << "\n"; std::exit(1); }
    json spec = json::parse(f);
    for (auto it = spec.begin(); it != spec.end(); ++it) {
        if (!it.key().empty() && it.key()[0] == '_') continue;   // skip comments
        if (!it.value().is_array()) {
            std::cerr << "error: '" << it.key() << "' must be an array of values\n";
            std::exit(1);
        }
        SweepDim d;
        d.name = it.key();
        for (auto& val : it.value()) d.values.push_back(val);
        if (!d.values.empty()) dims.push_back(d);
    }
}

// -----------------------------------------------------------------------------
// Pretty-print a value as it would appear in a CSV cell.
static std::string json_csv(const json& v)
{
    if (v.is_boolean()) return v.get<bool>() ? "true" : "false";
    if (v.is_number_integer()) return std::to_string(v.get<long long>());
    if (v.is_number_float()) {
        std::ostringstream os;
        os << v.get<double>();
        return os.str();
    }
    return v.dump();
}

// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    std::string input_path, sweep_path, output_dir;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if      (a == "--input"      && i + 1 < argc) input_path  = argv[++i];
        else if (a == "--sweep"      && i + 1 < argc) sweep_path  = argv[++i];
        else if (a == "--output-dir" && i + 1 < argc) output_dir  = argv[++i];
        else if (a == "-h" || a == "--help") {
            std::cout << "imbe_tune --input call.imbe --sweep sweep.json --output-dir DIR\n";
            return 0;
        }
        else {
            std::cerr << "unknown arg: " << a << "\n";
            return 1;
        }
    }
    if (input_path.empty() || sweep_path.empty() || output_dir.empty()) {
        std::cerr << "usage: imbe_tune --input <file.imbe> --sweep <file.json> --output-dir <dir>\n";
        return 1;
    }

    std::vector<ImbeFrame> frames;
    if (!read_imbe_file(input_path, frames) || frames.empty()) {
        std::cerr << "no frames read\n";
        return 1;
    }
    std::cerr << "read " << frames.size() << " IMBE frames ("
              << (frames.size() * 0.020) << " s of audio) from " << input_path << "\n";

    std::vector<SweepDim> dims;
    load_sweep_spec(sweep_path, dims);
    if (dims.empty()) {
        std::cerr << "sweep spec is empty — nothing to do\n";
        return 1;
    }

    size_t total = 1;
    for (auto& d : dims) total *= d.values.size();
    std::cerr << "sweep: " << dims.size() << " dim(s), " << total << " combinations\n";
    for (auto& d : dims) std::cerr << "  " << d.name << ": " << d.values.size() << " values\n";

    // Make output dir
    ::mkdir(output_dir.c_str(), 0755);

    // CSV header
    std::ofstream csv(output_dir + "/summary.csv");
    csv << "combo,wav,frames,duration_s,crest,rms,peak";
    for (auto& d : dims) csv << "," << d.name;
    csv << "\n";

    std::vector<size_t> idx(dims.size(), 0);
    for (size_t combo = 0; combo < total; combo++) {
        // Build VocoderParams from defaults + sweep overrides
        VocoderParams params;
        for (size_t i = 0; i < dims.size(); i++) {
            apply_knob(params, dims[i].name, dims[i].values[idx[i]]);
        }

        // Decode
        software_imbe_decoder dec;
        dec.set_params(params);
        dec.clear();

        std::vector<int16_t> samples;
        samples.reserve(frames.size() * 160);
        int16_t buf[160];
        for (const auto& fr : frames) {
            dec.decode_fullrate(buf,
                fr.u[0], fr.u[1], fr.u[2], fr.u[3],
                fr.u[4], fr.u[5], fr.u[6], fr.u[7],
                fr.E0, fr.ET);
            for (int i = 0; i < 160; i++) samples.push_back(buf[i]);
        }

        // Quick in-tool metrics (full set via analyze_vocoder.py on the .wav)
        double sumsq = 0.0;
        int peak = 0;
        for (auto s : samples) {
            int v = (s < 0) ? -s : s;
            if (v > peak) peak = v;
            sumsq += (double)s * (double)s;
        }
        double rms = samples.empty() ? 0.0 : std::sqrt(sumsq / (double)samples.size());
        double crest = (rms > 0.0) ? (double)peak / rms : 0.0;

        // Write WAV + JSON sidecar
        char base[64];
        std::snprintf(base, sizeof(base), "combo_%04zu", combo);

        std::string wav_path = output_dir + "/" + base + ".wav";
        write_wav(wav_path, samples);

        json sidecar;
        for (size_t i = 0; i < dims.size(); i++) {
            sidecar[dims[i].name] = dims[i].values[idx[i]];
        }
        sidecar["_metrics"] = {
            {"frames", frames.size()},
            {"duration_s", frames.size() * 0.020},
            {"crest", crest},
            {"rms", rms},
            {"peak", peak},
        };
        std::ofstream js(output_dir + "/" + base + ".json");
        js << sidecar.dump(2);

        // CSV row
        csv << combo << "," << (std::string(base) + ".wav") << ","
            << frames.size() << "," << (frames.size() * 0.020) << ","
            << crest << "," << rms << "," << peak;
        for (size_t i = 0; i < dims.size(); i++) {
            csv << "," << json_csv(dims[i].values[idx[i]]);
        }
        csv << "\n";

        // Progress line
        std::cerr << "[" << (combo + 1) << "/" << total << "] " << base
                  << "  crest=" << std::fixed << crest;
        for (size_t i = 0; i < dims.size(); i++) {
            std::cerr << "  " << dims[i].name << "=" << json_csv(dims[i].values[idx[i]]);
        }
        std::cerr << "\n";

        // Advance n-dim index (cartesian product, little-endian)
        for (size_t i = 0; i < dims.size(); i++) {
            if (++idx[i] < dims[i].values.size()) break;
            idx[i] = 0;
        }
    }

    csv.close();

    std::cerr << "\ndone. " << total << " combos in " << output_dir << "\n";
    std::cerr << "next steps:\n";
    std::cerr << "  python3 utils/analyze_vocoder.py " << output_dir << "/*.wav\n";
    std::cerr << "  python3 utils/blind_compare.py "  << output_dir << "    # (Phase 4)\n";
    return 0;
}
