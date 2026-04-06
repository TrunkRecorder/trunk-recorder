---
sidebar_label: 'simplestream'
---

# simplestream Plugin

**Plugin name:** `simplestream`  
**Library:** `libsimplestream.so`

The `simplestream` plugin streams live raw PCM audio over UDP or TCP while Trunk Recorder is actively recording a call. It can also prepend metadata to the stream and optionally emit JSON `call_start` and `call_end` events. 

## Build Requirement

This plugin is **not built by default**.

You must enable it in `trunk-recorder/CMakeLists.txt` before building Trunk Recorder.

### Example

```bash
sed -i 's/```[[:space:]]*#\s*add_subdirectory(plugins\/simplestream)/add_subdirectory(plugins\/simplestream)/' \
    trunk-recorder/CMakeLists.txt && \
mkdir -p trunk-recorder/build
```

After enabling it, rebuild Trunk Recorder as usual.

## Runtime Requirement

`simplestream` receives live audio through the plugin audio callback path. For that to happen, the global `audioStreaming` setting must be enabled in `config.json`.

If `audioStreaming` is not enabled, the plugin can still load, but it will not receive live audio sample callbacks. 

### Minimal Runtime Example

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "Live Audio",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 58914,
          "address": "127.0.0.1",
          "port": 9123
        }
      ]
    }
  ]
}
```

## Plugin Object

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| name | ✓ | `simplestream` | string | Friendly name for this plugin instance. This is especially useful when running multiple differently configured instances. |
| library | ✓ |  | string | Must be `libsimplestream.so`. |
| enabled |  | `true` | **true** / **false** | Whether this plugin instance should be loaded. |
| streams | ✓ |  | array | Array of stream objects. Each stream defines where audio or metadata should be sent and what traffic it should match.  |

## Stream Object

Each object in `streams` defines one output target.

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| TGID | ✓ |  | number | Talkgroup to match. Set to `0` to stream all talkgroups. Patched talkgroups are also considered when matching. |
| address | ✓ |  | string | Destination IP address. Used for both UDP and TCP. |
| port | ✓ |  | number | Destination port. Used for both UDP and TCP. |
| shortName |  | `""` | string | Optional system filter. If empty, all systems match. If set, it must match the call's `shortName`. |
| sendJSON |  | `false` | **true** / **false** | When `true`, prepends JSON metadata to each live audio packet. JSON audio metadata includes `src`, `src_tag`, `talkgroup`, `patched_talkgroups`, `freq`, `short_name`, `audio_sample_rate`, and `event`. If this is enabled, it should be preferred over `sendTGID`.  |
| sendCallStart |  | `false` | **true** / **false** | Only meaningful when `sendJSON` is enabled. Sends a JSON `call_start` event at the beginning of matching calls.  |
| sendCallEnd |  | `false` | **true** / **false** | Only meaningful when `sendJSON` is enabled. Sends a JSON `call_end` event at the end of matching calls.  |
| sendTGID |  | `false` | **true** / **false** | Prepends the matched TGID as 4 bytes before each audio payload. This is effectively a legacy metadata mode. If `sendJSON` is enabled, downstream consumers should use the JSON metadata instead.  |
| useTCP |  | `false` | **true** / **false** | When `true`, uses TCP instead of UDP for this stream. TCP sockets are connected during plugin startup and closed during plugin shutdown. |

## Audio Format

The plugin sends raw signed 16-bit mono PCM samples from Trunk Recorder's audio callback. Audio is transmitted exactly as provided to the plugin.

Sample rate depends on the recorder providing the audio:

- digital audio is typically `8000`
- analog audio is typically `16000`

When `sendJSON` is enabled, the plugin includes `audio_sample_rate` in the JSON metadata for each audio packet. 

## Packet Format

### Raw Audio Only

When both `sendJSON` and `sendTGID` are `false`, the packet contains only raw PCM audio.

### TGID + Audio

When `sendTGID` is `true`, the packet contains:

1. a 4-byte TGID
2. the raw PCM audio

### JSON + Audio

When `sendJSON` is `true`, the packet contains:

1. a 4-byte JSON length
2. the JSON payload
3. the raw PCM audio 

## JSON Event Payloads

### Audio Packet Metadata

When `sendJSON` is enabled, each live audio packet includes a JSON object with:

- `src`
- `src_tag`
- `talkgroup`
- `patched_talkgroups`
- `freq`
- `short_name`
- `audio_sample_rate`
- `event` = `"audio"` 

### call_start Metadata

When both `sendJSON` and `sendCallStart` are enabled, the plugin sends a `call_start` JSON message containing:

- `src`
- `src_tag`
- `talkgroup`
- `talkgroup_tag`
- `patched_talkgroups`
- `patched_talkgroup_tags`
- `freq`
- `short_name`
- `event` = `"call_start"` 

### call_end Metadata

When both `sendJSON` and `sendCallEnd` are enabled, the plugin sends a `call_end` JSON message containing:

- `talkgroup`
- `patched_talkgroups`
- `freq`
- `short_name`
- `event` = `"call_end"` 

## Matching Behavior

A stream is used when all applicable conditions match:

- `shortName` matches the call's system short name, or `shortName` is empty
- `TGID` matches the call's talkgroup or one of its patched talkgroups
- or `TGID` is `0`, which means match everything

If a call source is not immediately available during audio or `call_start` processing, the plugin attempts to use the source from the most recent transmission for that call.

## Transport Behavior

- UDP streams use a shared UDP socket.
- TCP streams create and connect a socket for each configured stream during plugin startup.
- TCP sockets are shut down and closed during plugin stop.

## Examples

### Minimal UDP Example

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County Live Audio",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 58914,
          "address": "127.0.0.1",
          "port": 9123,
          "shortName": "CountyTrunked"
        }
      ]
    }
  ]
}
```

### Example with JSON Audio Metadata

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County JSON Audio",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 58914,
          "address": "127.0.0.1",
          "port": 9123,
          "shortName": "CountyTrunked",
          "sendJSON": true
        }
      ]
    }
  ]
}
```

### Example with call_start and call_end Events

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County Event Stream",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 0,
          "address": "127.0.0.1",
          "port": 9123,
          "shortName": "CountyTrunked",
          "sendJSON": true,
          "sendCallStart": true,
          "sendCallEnd": true
        }
      ]
    }
  ]
}
```

### Example Using TCP

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County TCP Stream",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 58918,
          "address": "127.0.0.1",
          "port": 9125,
          "shortName": "CountyTrunked",
          "useTCP": true
        }
      ]
    }
  ]
}
```

## PulseAudio Examples

PulseAudio can receive raw PCM audio from `simplestream` over TCP using `module-simple-protocol-tcp`.

This works well when you want Trunk Recorder to send live audio directly into a local or remote Linux audio stack. 

### Important Notes

- These examples require `useTCP: true` in the `simplestream` stream object.
- These examples also require `audioStreaming: true` in the main Trunk Recorder config.
- When sending directly to PulseAudio, `sendJSON` and `sendTGID` should both be `false`, because PulseAudio expects raw PCM audio only.
- The PulseAudio sample rate must match the audio being sent:
  - digital audio: `8000`
  - analog audio: `16000` 

### Example 1: Digital Audio to PulseAudio over TCP

This example sets up PulseAudio to receive 8 kHz mono signed 16-bit PCM on TCP port `9125`.

```bash
pacmd load-module module-simple-protocol-tcp sink=1 playback=true port=9125 format=s16le rate=8000 channels=1
```

Matching `simplestream` config:

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County Digital PulseAudio",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 58918,
          "address": "127.0.0.1",
          "port": 9125,
          "shortName": "CountyTrunked",
          "useTCP": true,
          "sendJSON": false,
          "sendTGID": false
        }
      ]
    }
  ]
}
```

### Example 2: Analog Audio to PulseAudio over TCP

This example is the same idea, but for analog audio at 16 kHz.

```bash
pacmd load-module module-simple-protocol-tcp sink=1 playback=true port=9126 format=s16le rate=16000 channels=1
```

Matching `simplestream` config:

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County Analog PulseAudio",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 1541,
          "address": "127.0.0.1",
          "port": 9126,
          "shortName": "CountyAnalog",
          "useTCP": true,
          "sendJSON": false,
          "sendTGID": false
        }
      ]
    }
  ]
}
```

### Example 3: Separate PulseAudio Streams for Different Talkgroups

If you want separate PulseAudio applications for different talkgroups, use a different TCP port for each one and load PulseAudio once per port.

PulseAudio:

```bash
pacmd load-module module-simple-protocol-tcp sink=1 playback=true port=9127 format=s16le rate=8000 channels=1
pacmd load-module module-simple-protocol-tcp sink=1 playback=true port=9128 format=s16le rate=8000 channels=1
```

Matching `simplestream` config:

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County Multi PulseAudio",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 58914,
          "address": "127.0.0.1",
          "port": 9127,
          "shortName": "CountyTrunked",
          "useTCP": true,
          "sendJSON": false,
          "sendTGID": false
        },
        {
          "TGID": 58916,
          "address": "127.0.0.1",
          "port": 9128,
          "shortName": "CountyTrunked",
          "useTCP": true,
          "sendJSON": false,
          "sendTGID": false
        }
      ]
    }
  ]
}
```

Each TCP connection should appear as a separate application in PulseAudio.

### Example 4: Stream All Talkgroups from One System to PulseAudio

You can stream all recorded talkgroups from one system by setting `TGID` to `0`.

```bash
pacmd load-module module-simple-protocol-tcp sink=1 playback=true port=9129 format=s16le rate=8000 channels=1
```

Matching `simplestream` config:

```json
{
  "audioStreaming": true,
  "plugins": [
    {
      "name": "County All TGIDs PulseAudio",
      "library": "libsimplestream.so",
      "streams": [
        {
          "TGID": 0,
          "address": "127.0.0.1",
          "port": 9129,
          "shortName": "CountyTrunked",
          "useTCP": true,
          "sendJSON": false,
          "sendTGID": false
        }
      ]
    }
  ]
}
```

This is simple, but audio from different calls will all land in the same PulseAudio stream.

## When to Use JSON Instead

If you want downstream software to know the talkgroup, source, patched talkgroups, or call event type, use `sendJSON: true`.

JSON-enabled streams are **not** suitable for direct input into PulseAudio, because PulseAudio expects raw PCM only. JSON mode is better for custom receivers or other software that can parse the metadata before handling the audio. 

## Example: Sending Audio to FFmpeg

FFmpeg can also receive raw PCM audio from `simplestream` and transcode it. This only works when `sendJSON` and `sendTGID` are both `false`, because FFmpeg expects raw PCM audio only on the input stream.

```bash
ffmpeg -loglevel warning -f s16le -ar 16000 -ac 1 -i udp://localhost:9125 -af:a adeclick -f:a ogg -c:a libopus -frame_duration:a 20 -vbr:a on -b:a 48000 -application:a voip pipe:1
```