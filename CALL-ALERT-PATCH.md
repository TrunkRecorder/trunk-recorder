# P25 Call Alert (TSBK 0x1F) Decoding

## Summary

Adds proper decoding of P25 TSBK opcode 0x1F (Call Alert) to trunk-recorder.
Previously this was a stub that only logged a debug message and discarded the message.
Now it extracts source/destination unit IDs, sets the message type, and flows through
the plugin system like all other unit events.

## Changes

### `trunk-recorder/systems/parser.h`
Added `CALL_ALERT = 18` to the `MessageType` enum.

### `trunk-recorder/systems/p25_parser.cc`
Replaced the stub with real decoding:
- Source Unit ID extracted from bits 16–39: `bitset_shift_mask(tsbk, 16, 0xffffff)`
- Target Unit ID extracted from bits 40–63: `bitset_shift_mask(tsbk, 40, 0xffffff)`
- Sets `message.message_type = CALL_ALERT`, `message.source`, and `message.talkgroup`
- Logs at **info** level: `tsbk1f\tCall Alert\tSource Unit: <src>\tTarget Unit: <dst>`

Bit layout reference (from OP25):
```
args = dst << 24 | src   →   bits[16:39] = src, bits[40:63] = dst
```

### `trunk-recorder/plugin_manager/plugin_api.h`
Added virtual method to the plugin API with a default no-op — fully backward compatible
with existing plugins that don't implement it:
```cpp
virtual int unit_call_alert(System *sys, long source_id, long talkgroup) { return 0; };
```

### `trunk-recorder/plugin_manager/plugin_manager.h`
Declared `plugman_unit_call_alert(System *system, long source_id, long talkgroup)`.

### `trunk-recorder/plugin_manager/plugin_manager.cc`
Implemented `plugman_unit_call_alert()` — iterates running plugins and calls
`plugin->api->unit_call_alert()` on each, matching the pattern of all other unit events.

### `trunk-recorder/monitor_systems.cc`
- Added `unit_call_alert()` function that calls `plugman_unit_call_alert()`
- Added `case CALL_ALERT:` to the message dispatch switch

## Built Image

```
trunk-recorder:call-alert
```

Built from `ghcr.io/trunk-reporter/trunk-recorder:latest` as runtime base,
with the patched `trunk-recorder` binary replacing `/usr/local/bin/trunk-recorder`.
Dockerfile: `Dockerfile.call-alert`
