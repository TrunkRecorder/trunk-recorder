#ifndef TONE_MANAGER_H
#define TONE_MANAGER_H

#include "global_structs.h"
#include <string>

// Returns true iff two Tone_Configs match for routing purposes (same mode +
// same frequency/code/polarity within tolerance). Used when a single recorder
// serves multiple logical channels and we need to map a detected tone to the
// right CSV row.
bool tones_match(const Tone_Config &a, const Tone_Config &b);

// Parses a Tone column value into a Tone_Config. Accepted forms:
//   ""  or  "0"  / "0.0"               -> TONE_OFF
//   "S" / "s"                          -> TONE_SEARCH
//   strict /D\d{3}[NI]/i               -> TONE_DCS (3-digit octal, polarity required)
//   any positive numeric (CTCSS Hz)    -> TONE_CTCSS
// Anything else returns TONE_OFF — the caller is expected to detect that and log.
Tone_Config parse_tone_spec(const std::string &raw);

#endif
