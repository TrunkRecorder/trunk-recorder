---
sidebar_label: 'Rdio Scanner'
---

# Rdio Scanner Plugin

**Plugin name:** `rdioscanner_uploader`  
**Library:** `librdioscanner_uploader.so`

The Rdio Scanner plugin uploads completed calls to an Rdio Scanner server using that server's `/api/call-upload` endpoint. It runs on `call_end`, skips encrypted calls, and supports per-system API credentials and optional talkgroup allow/deny filters.

## Plugin Object

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| name | ✓ | `rdioscanner_uploader` | string | Friendly name for this plugin instance. This is used in logging. If set to exactly `rdioscanner_uploader`, log messages display `Rdio Scanner`. Otherwise the configured name is used. |
| library | ✓ |  | string | Must be `librdioscanner_uploader.so`. |
| enabled |  | `true` | **true** / **false** | Whether this plugin instance should be loaded. General plugin behavior is handled by the plugin manager. |
| server | ✓ |  | string | Base URL for the Rdio Scanner server. The plugin validates that this is a parseable HTTP or HTTPS URL. Uploads are sent to `/api/call-upload` on this server. |
| systems | ✓ |  | array | Array of Rdio Scanner system objects. At least one valid system with an `apiKey` must be configured or the plugin returns an error during configuration parsing. |

## Rdio Scanner System Object

Each object in `systems` describes one Trunk Recorder system that should be uploaded to Rdio Scanner.

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| shortName | ✓ |  | string | Must match the `shortName` of a system defined in the main Trunk Recorder config. The plugin uses this to match a completed call to the correct Rdio Scanner upload settings.|
| apiKey | ✓ |  | string | API key used for uploads for this system. If a system has no API key configured, uploads for that system are skipped. The key is partially redacted in logs. |
| systemId | ✓ |  | number | Rdio Scanner system ID to send with uploads.|
| talkgroupAllow |  | `[]` | array of string/number | Optional allow-list of talkgroups. If this list is non-empty, the talkgroup must match at least one pattern or the upload is skipped. Patterns use glob-style matching with `*` and `?`. Numeric values are accepted and converted to strings. |
| talkgroupDeny |  | `[]` | array of string/number | Optional deny-list of talkgroups. If a talkgroup matches any deny pattern, the upload is skipped. Patterns use glob-style matching with `*` and `?`. Numeric values are accepted and converted to strings. |

## Talkgroup Filter Rules

Talkgroup filters are evaluated per configured Rdio Scanner system.

- Talkgroups are compared as strings, for example `50712`.
- `*` matches any number of characters.
- `?` matches a single character.
- If `talkgroupAllow` is non-empty, the talkgroup must match at least one allow pattern.
- If `talkgroupDeny` is non-empty, the talkgroup must not match any deny pattern.
- If both are present, allow is checked first, then deny.

## What the Plugin Uploads

On successful `call_end`, the plugin uploads the call audio plus metadata fields including:

- audio file
- audio filename
- audio type
- date/time
- frequencies
- primary frequency
- API key
- patches
- talkgroup
- talkgroup group / label / tag / name
- sources
- system ID
- system label (`shortName`)

The plugin uses the converted file when `compress_wav` is enabled for the call, otherwise it uploads the WAV file. It sets the logical audio type to `audio/mp4` for converted audio and `audio/wav` for WAV uploads.

## Behavior Notes

- Encrypted calls are skipped and treated as a non-error.
- If the matched system is missing or has no API key, the upload is skipped and treated as a non-error. 
- If talkgroup filters reject the call, the upload is skipped and an informational log message is written.
- HTTP 2xx responses are considered successful. HTTP `202` is logged as an accepted upload.
- The plugin uses shared cURL DNS caching with a 300 second TTL to reduce DNS lookups.
## Example

```json
{
  "plugins": [
    {
      "name": "Rdio Main",
      "library": "librdioscanner_uploader.so",
      "server": "http://127.0.0.1",
      "systems": [
        {
          "shortName": "county",
          "apiKey": "fakekey",
          "systemId": 411,
          "talkgroupAllow": ["507*", "12???"],
          "talkgroupDeny": ["507?9", "12345"]
        }
      ]
    }
  ]
}
```

## Minimal Example

```json
{
  "plugins": [
    {
      "name": "Rdio Main",
      "library": "librdioscanner_uploader.so",
      "server": "http://127.0.0.1",
      "systems": [
        {
          "shortName": "county",
          "apiKey": "fakekey",
          "systemId": 411
        }
      ]
    }
  ]
}
```