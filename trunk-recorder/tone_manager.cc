#include "tone_manager.h"

#include <cctype>
#include <cstdlib>

bool tones_match(const Tone_Config &a, const Tone_Config &b) {
  if (a.mode != b.mode) return false;
  switch (a.mode) {
    case TONE_CTCSS: {
      const double d = a.ctcss_hz - b.ctcss_hz;
      return (d > -0.5 && d < 0.5);
    }
    case TONE_DCS:
      return a.dcs_code == b.dcs_code && a.dcs_inverted == b.dcs_inverted;
    case TONE_OFF:
    case TONE_SEARCH:
      return true;
  }
  return false;
}

Tone_Config parse_tone_spec(const std::string &raw) {
  Tone_Config tc; // default OFF
  if (raw.empty()) return tc;

  // Trim whitespace
  size_t a = 0;
  size_t b = raw.size();
  while (a < b && std::isspace(static_cast<unsigned char>(raw[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(raw[b - 1]))) --b;
  if (a == b) return tc;
  const std::string s = raw.substr(a, b - a);

  // Search mode
  if (s.size() == 1 && (s[0] == 'S' || s[0] == 's')) {
    tc.mode = TONE_SEARCH;
    return tc;
  }

  // DCS: strict "D" + exactly 3 octal digits + 'N'/'n'/'I'/'i'
  if ((s[0] == 'D' || s[0] == 'd') && s.size() == 5) {
    bool ok = true;
    for (int i = 1; i <= 3; ++i) {
      char c = s[i];
      if (c < '0' || c > '7') { ok = false; break; }
    }
    char pol = s[4];
    if (ok && (pol == 'N' || pol == 'n' || pol == 'I' || pol == 'i')) {
      tc.mode = TONE_DCS;
      tc.dcs_code = ((s[1] - '0') * 100) + ((s[2] - '0') * 10) + (s[3] - '0');
      tc.dcs_inverted = (pol == 'I' || pol == 'i');
      return tc;
    }
    // Falls through to OFF below if malformed.
    return tc;
  }

  // CTCSS: any positive number. strtod tolerates ints and floats.
  char *endp = nullptr;
  const double v = std::strtod(s.c_str(), &endp);
  if (endp != s.c_str() && *endp == '\0' && v > 0.0) {
    tc.mode = TONE_CTCSS;
    tc.ctcss_hz = v;
    return tc;
  }

  // Unrecognized — leave as OFF (caller can log if surprising).
  return tc;
}
