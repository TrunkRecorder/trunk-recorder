---
sidebar_label: 'OpenMHz'
---

# OpenMHz Plugin

**Plugin name:** `openmhz_uploader`  
**Library:** `libopenmhz_uploader.so`

The OpenMHz plugin uploads completed calls to an OpenMHz server. It runs on `call_end` and uploads the converted call audio file plus call metadata to the configured OpenMHz system upload endpoint.

## Plugin Object

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| name | ✓ | `openmhz_uploader` | string | Friendly name for this plugin instance. This is used in logging. If set to exactly `openmhz_uploader`, log messages display `OpenMHz`. Otherwise the configured name is used. |
| library | ✓ |  | string | Must be `libopenmhz_uploader.so`. |
| enabled |  | `true` | **true** / **false** | Whether this plugin instance should be loaded. General plugin loading behavior is handled by the plugin manager. |
| uploadServer | ✓ |  | string | Base OpenMHz server URL. The plugin validates that this is a parseable HTTP or HTTPS URL. Uploads are sent to `/<openmhzSystemId>/upload` on this server. |
| systems | ✓ |  | array | Array of system objects. The plugin scans these for systems with an `apiKey` and uses them to match completed calls by `shortName`. If no usable systems are configured, the plugin returns an error during configuration parsing. |

## OpenMHz System Object

Each object in `systems` describes one Trunk Recorder system that should be uploaded to OpenMHz.

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| shortName | ✓ |  | string | Must match the `shortName` of a system defined in the main Trunk Recorder config. The plugin uses this to match a completed call to the correct OpenMHz upload settings. |
| apiKey | ✓ |  | string | API key used for uploads for this system. Systems without an API key are ignored by the plugin. The key is partially redacted in logs.  |
| openmhzSystemId |  | `shortName` | string | OpenMHz system ID to upload into. If omitted, the plugin uses the system `shortName`.|

## What the Plugin Uploads

On successful `call_end`, the plugin uploads the converted call audio file and the following metadata fields:

- `freq`
- `error_count`
- `spike_count`
- `start_time`
- `stop_time`
- `call_length`
- `talkgroup_num`
- `emergency`
- `api_key`
- `patch_list`
- `source_list`

The plugin always uploads `call_info.converted`, so this plugin expects the converted/compressed call artifact to exist.

## Behavior Notes

- If a completed call's `shortName` does not match a configured OpenMHz system with an API key, the upload is skipped and treated as a non-error.
- If the server returns HTTP `200`, the upload is logged as a success.
- Certain server-side responses are treated as configuration or policy issues and do **not** go to the retry queue:
    - `API Keys do not match` → logged as `Invalid API Key`
    - `ShortName does not exist` → logged as `Invalid System Name`
    - `Error, invalid filename` → logged as `Invalid Filename`
    - `Talkgroup does not exist` → logged as `Skipped: System Ignoring Unknown Talkgroups`
- Other failures are logged as upload errors and returned as retryable failures.
- The plugin uses shared cURL DNS caching with a 300 second TTL to reduce DNS lookups.

## Example

```json
{
  "plugins": [
    {
      "name": "OpenMHz Main", 
      "library": "libopenmhz_uploader.so", 
      "uploadServer": "https://api.openmhz.com", 
      "systems": [
        {
          "shortName": "county", 
          "apiKey": "fakekey", 
          "openmhzSystemId": "county-main"
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
      "name": "OpenMHz Main", 
      "library": "libopenmhz_uploader.so", 
      "uploadServer": "https://api.openmhz.com", 
      "systems": [
        {
          "shortName": "county", 
          "apiKey": "fakekey"
        }
      ]
    }
  ]
}
```