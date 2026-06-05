# Tone Squelch (CTCSS / DCS)

Analog conventional channels often use a sub-audible tone or digital code to share
one RF frequency between multiple talkgroups. Only receivers configured for that
specific tone open audio; everything else stays muted even though they're all
listening to the same carrier. Trunk-recorder supports the same idea via the
**Tone** column in the `channelFile` CSV.

Three tone families are supported:

| Family | What it is | Tone column value |
|--------|-----------|-------------------|
| **None** | No tone; record any signal that breaks squelch | empty, `0`, or `0.0` |
| **CTCSS** | Continuous Tone-Coded Squelch System — a sub-audible sine tone (67–254 Hz) | The frequency in Hz, e.g. `118.8`, `151.4` |
| **DCS** | Digital Coded Squelch (a.k.a. DPL/Motorola, CDCSS) — a 134.4 baud Golay codeword | `D` + 3-digit octal code + polarity, e.g. `D023N`, `D131I` |
| **Search** | Detect the tone but don't gate on it — report whatever's on air | `S` or `s` |

## Modes in detail

### `0` / empty — no tone

The recorder records everything the squelch lets through, like a stock
conventional channel before this feature existed. No tone detection runs.

### CTCSS (e.g. `118.8`)

Enter the tone frequency in Hz as a number, e.g. `118.8`. trunk-recorder
recognizes the full standard PL set from 67.0 Hz up through 254.1 Hz.
Only transmissions carrying that tone will be recorded.

### DCS (e.g. `D023N`, `D131I`)

Enter the 3-digit code (with the leading zero — `D023N`, not `D23N`)
followed by `N` or `I` to match what your code list shows. You'll usually
see `N`; some older radios use `I`. If you're not sure, try `N` first.

trunk-recorder is forgiving on the `N` vs `I` suffix and will record the
channel regardless of which one the transmitter actually uses.

One thing worth knowing: DCS has a small number of codes that look
identical over the air — for example, a `D023N` channel will also pick up
`D047I` transmissions, because the two signals are indistinguishable to
any receiver. This is a quirk of the DCS protocol itself, not specific to
trunk-recorder. The pairs are rare and almost never both used in the same
geographic area, but if you ever see unexpected recordings on a DCS
channel, this might be why.

### Search (`S`)

The recorder records everything the squelch lets through (no gate), but both
the CTCSS and DCS detectors run side-chain. At end of call, the
identified tone is written to the JSON sidecar and the call log — useful for
discovering what tones are actually in use on a channel before configuring
specific channels for them.

## End-of-call JSON fields

Every call's `*.json` sidecar gains three flat fields:

```json
{
  "tone_mode":       "ctcss",     // "off" | "ctcss" | "dcs" | "search"
  "tone_detected":   "118.8",     // "" | "<Hz>" | "D<code><N|I>" | "D<a>/D<b>"
  "tone_confidence": 0.957        // 0.0 – 1.0
}
```

- `tone_mode` — what the recorder was *configured* to do for this channel.
- `tone_detected` — what was actually heard. Empty when nothing was
  identified. In search mode, when a DCS transmission is one of the
  ambiguous pairs (see the DCS section above), both possibilities are
  reported joined with `/`, e.g. `D023N/D047I`.
- `tone_confidence` — for CTCSS, this is the spectral concentration; for DCS
  it's phase diversity × match rate. Use it to threshold consumers downstream.

For non-analog (P25/DMR/digital) recorders the fields are still emitted but
will always be `"off"` / `""` / `0.0`.

## Multi-row freq groups

A common operator scenario: one frequency carries three or more separate
talkgroup labels, each gated by a different tone. Express this directly by
writing one CSV row per (TG, tone) pair, *sharing the same Frequency*:

```csv
TG Number,Frequency,Tone,Squelch,Alpha Tag,Description,Tag,Category,Enable
203,154325000,D223N,-48,County Fairfield,County Fairfield,County Fire,Fire,true
204,154325000,151.4,-48,County Eaton,County Eaton,County Fire,Fire,true
205,154325000,118.8,-48,County North,County North,County Fire,Fire,true
```

Trunk-recorder collapses all rows at the same frequency into a **single
analog recorder running in search mode**, then routes each captured call to
the matching row by detected tone. This is much cheaper than three separate
recorders sharing an antenna, and it correctly produces three independent
"call streams" tagged with the right talkgroup, alpha tag, and category.

Setup behavior:

- The first row at a freq is the **primary** — its `Squelch` and
  `Signal Detector` settings drive the shared recorder.
- All other rows are **alternates** — only their TG/Alpha/Description/Tag/
  Category metadata is used (their per-row Squelch values are ignored).
- A row with `Tone=0`/empty or `Tone=S` in a group acts as a **catch-all**
  for that frequency: any tone (or no tone) that doesn't match a specific
  alternate ends up routed to it.

### What happens when no row matches: SKIPPED

If the detected tone doesn't match any row's `Tone` value **and** there's no
catch-all row in the group, the call is concluded as **SKIPPED**:

- No WAV files are kept.
- No plugins fire (no upload, no stream, no log emission to external systems).
- A single log line records what was dropped and why:

  ```
  [system]  17C   TG: <primary>  Freq: 154.325 MHz  SKIPPED — detected tone '127.3' did not match any allowed row
  ```

This makes the multi-row config a **strict allow-list**, not a catch-all.
Use it when you specifically want to ignore traffic on a frequency that
isn't from one of your configured groups.

If you want the more permissive "record everything, label what we can"
behavior, add a single `Tone=S` row to the group — it acts as a fallback
bucket that catches non-matching transmissions.

## Examples

### Single channel with a CTCSS tone

```csv
TG Number,Frequency,Tone,Alpha Tag,Description,Tag,Category
100,154265000,151.4,FD Disp,Fire Dispatch,Fire,County
```

### Single channel with a DCS code

```csv
TG Number,Frequency,Tone,Alpha Tag,Description,Tag,Category
106,155857500,D162N,FD South,Fire South Ops,Fire,County
```

### Search-only channel (discover tones)

```csv
TG Number,Frequency,Tone,Alpha Tag,Description,Tag,Category
109,155250000,S,FD Paging,Fire Paging Out,Fire,Paging
```

Every call on this channel concludes with `tone_detected` populated; no calls
are dropped.

### Multi-row freq group, strict allow-list

```csv
TG Number,Frequency,Tone,Alpha Tag,Description,Tag,Category
203,154325000,D223N,County A,County A Fire,Fire,County
204,154325000,151.4,County B,County B Fire,Fire,County
205,154325000,118.8,County C,County C Fire,Fire,County
```

Three TGs share 154.325 MHz; only D223N, 151.4 Hz CTCSS, and 118.8 Hz CTCSS
make it to disk. Anything else gets SKIPPED.

### Multi-row freq group, with catch-all

```csv
TG Number,Frequency,Tone,Alpha Tag,Description,Tag,Category
203,154325000,D223N,County A,County A Fire,Fire,County
204,154325000,151.4,County B,County B Fire,Fire,County
299,154325000,S,County Misc,County Other Traffic,Fire,County
```

D223N and 151.4 Hz are labelled specifically; everything else lands on
TG 299 with the detected tone in the sidecar.

## Tips

- The CTCSS detector typically locks within 200–400 ms of carrier; DCS
  needs at least one full codeword (~170 ms) plus a confirming repeat,
  so very short keyups (< 250 ms) may still record but with empty
  `tone_detected`.
- For DCS, don't sweat the `N`/`I` suffix — pick whichever your code list
  shows (usually `N`). trunk-recorder will record either polarity.
- The end-of-call diagnostic verdict from each detector is emitted at
  `debug` log level; turn it on (`"logLevel": "debug"` in config) when
  tuning a new channel.
- The recorder still runs all of trunk-recorder's normal squelch and
  signal-detection logic. Tone gating is *additional* — it's evaluated
  on audio that has already passed the squelch.
