---
sidebar_label: 'Broadcastify Calls'
---

# Broadcastify Calls Plugin

**Plugin name:** `broadcastify_uploader`  
**Library:** `libbroadcastify_uploader.so`

The Broadcastify Calls plugin uploads completed calls to a Broadcastify Calls server. It runs on `call_end`, uploads call metadata first, and then uploads the converted audio file if the metadata step succeeds. It skips encrypted calls and supports per-system API credentials plus optional talkgroup allow/deny filters.

## Requirements

Broadcastify Calls requires audio uploads in `.m4a` format with AAC audio.

This plugin uploads the converted call audio file, not the raw WAV file. Because of that, `compressWav` must be enabled on the system so Trunk Recorder creates the converted `.m4a` file before the plugin runs.

## Plugin Object

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| name | ✓ | `broadcastify_uploader` | string | Friendly name for this plugin instance. This is used in logging. If set to exactly `broadcastify_uploader`, log messages display `Broadcastify`. Otherwise the configured name is used. |
| library | ✓ |  | string | Must be `libbroadcastify_uploader.so`. |
| enabled |  | `true` | **true** / **false** | Whether this plugin instance should be loaded. General plugin loading behavior is handled by the plugin manager. |
| broadcastifyCallsServer | ✓ |  | string | Broadcastify Calls server URL. The plugin validates that this is a parseable HTTP or HTTPS URL. Metadata uploads are sent to this URL directly. |
| broadcastifySslVerifyDisable |  | `false` | **true** / **false** | When true, disables SSL certificate and host verification for Broadcastify uploads. |
| broadcastifyOTA |  | `true` | **true** / **false** | Enables upload of the OTA alias as `srcId_alias` when available from the first entry in `transmission_source_list`. |
| systems | ✓ |  | array | Array of system objects. The plugin scans these for systems with a `broadcastifyApiKey` and uses them to match completed calls by `shortName`. If no usable systems are configured, the plugin returns an error during configuration parsing. |

## Broadcastify System Object

Each object in `systems` describes one Trunk Recorder system that should be uploaded to Broadcastify Calls.

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| shortName | ✓ |  | string | Must match the `shortName` of a system defined in the main Trunk Recorder config. The plugin uses this to match a completed call to the correct Broadcastify upload settings. |
| broadcastifyApiKey | ✓ |  | string | API key used for uploads for this system. Systems without an API key are ignored by the plugin. The key is partially redacted in logs. |
| broadcastifySystemId | ✓ |  | number | Broadcastify system ID to send with uploads. If this is `0`, uploads for the system are skipped. |
| broadcastifyAllow |  | `[]` | array of string/number | Optional allow-list of talkgroups. If this list is non-empty, the talkgroup must match at least one pattern or the upload is skipped. Patterns use glob-style matching with `*` and `?`. Numeric values are accepted and converted to strings. |
| broadcastifyDeny |  | `[]` | array of string/number | Optional deny-list of talkgroups. If a talkgroup matches any deny pattern, the upload is skipped. Patterns use glob-style matching with `*` and `?`. Numeric values are accepted and converted to strings. |


## Talkgroup Filter Rules

Talkgroup filters are evaluated per configured Broadcastify system.

- Talkgroups are compared as strings, for example `50712`.
- `*` matches any number of characters.
- `?` matches a single character.
- If `broadcastifyAllow` is non-empty, the talkgroup must match at least one allow pattern.
- If `broadcastifyDeny` is non-empty, the talkgroup must not match any deny pattern.
- If both are present, allow is checked first, then deny.

## What the Plugin Uploads

The plugin uploads in two steps.

### 1. Metadata Upload

It sends a multipart metadata request containing:

- `metadata` as `call_meta.json` from `call_info.call_json.dump()`
- `callDuration`
- `systemId`
- `apiKey`
- optionally `srcId_alias` when `broadcastifyOTA` is enabled and an OTA alias is present in the first transmission source entry

### 2. Audio Upload

If the metadata upload succeeds and the server returns a success response with an audio upload URL, the plugin then uploads the converted m4a as audio with content type `audio/aac`.

The plugin always uploads m4a files as required by Broadcastify Calls, it requires `compressWav` enabled.

## Behavior Notes

- If a completed call's `shortName` does not match a configured Broadcastify system, the upload is skipped and treated as a non-error.
- Encrypted calls are skipped and treated as a non-error.
- If talkgroup filters reject the call, the upload is skipped and an informational log message is written.
- If the matched system has no API key or system ID, the upload is skipped and treated as a non-error.
- Metadata upload must return HTTP `200` before audio upload is attempted.
- If the metadata response starts with:
  - `1 SKIPPED...` the upload is logged as skipped and treated as a non-error.
  - `1 REJECTED...` the upload is logged as rejected and treated as a non-error.
  - any other non-zero code is treated as a retryable error.
- Audio upload errors are treated as retryable failures.
- The plugin uses shared cURL DNS caching with a 300 second TTL to reduce DNS lookups.

## Example

```json
{
  "plugins": [
    {
      "name": "Broadcastify Main",
      "library": "libbroadcastify_uploader.so",
      "broadcastifyCallsServer": "https://api.broadcastify.com/call-upload",
      "broadcastifySslVerifyDisable": false,
      "broadcastifyOTA": true,
      "systems": [
        {
          "shortName": "county",
          "broadcastifyApiKey": "fakekey",
          "broadcastifySystemId": 411,
          "broadcastifyAllow": ["507*", "12???"],
          "broadcastifyDeny": ["507?9", "12345"]
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
      "name": "Broadcastify Main",
      "library": "libbroadcastify_uploader.so",
      "broadcastifyCallsServer": "https://api.broadcastify.com/call-upload",
      "systems": [
        {
          "shortName": "county",
          "broadcastifyApiKey": "fakekey",
          "broadcastifySystemId": 411
        }
      ]
    }
  ]
}
```