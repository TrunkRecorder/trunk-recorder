---
sidebar_label: 'unit_script'
---

# unit_script Plugin

**Plugin name:** `unit_script`  
**Library:** `libunit_script.so`

The `unit_script` plugin runs an external script when certain radio and call events occur. Scripts are configured per system using that system's `unitScript` value, and the plugin looks up the script to run by matching the call or event against the system's `shortName`.

## Plugin Object

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| name | ✓ | `unit_script` | string | Friendly name for this plugin instance. |
| library | ✓ |  | string | Must be `libunit_script.so`. |
| enabled |  | `true` | **true** / **false** | Whether this plugin instance should be loaded. |
| systems | ✓ |  | array | Array of system objects. The plugin reads `shortName` and `unitScript` from each system object and builds its per-system script mapping from them. |

## System Settings Used

This plugin does not define its own standalone per-plugin system object. Instead, it reads the following values from each object in the plugin's `systems` array:

| Key | Required | Type | Description |
| --- | :---: | --- | --- |
| shortName | ✓ | string | Must match the Trunk Recorder system short name for the events you want this script to handle. |
| unitScript | ✓ | string | Path to the script or command that should be run for that system. If this is empty, the plugin does nothing for that system. |

## How It Works

During configuration parsing, the plugin reads each system entry, extracts `shortName` and `unitScript`, and stores only the systems that have a non-empty `unitScript`.

When an event occurs, the plugin:

1. finds the configured script for the matching system `shortName`
2. builds a shell command with event-specific arguments
3. runs it asynchronously using `system("... &")`

If no script is configured for the matching system, or the source radio ID is `0`, the event is ignored.

## Events and Arguments

The plugin runs the script with positional arguments. The first three are always:

1. system short name
2. source radio ID
3. action

Additional arguments depend on the event type.

### Radio Registration

Action: `on`

```text
<script> <shortName> <radioID> on
```

Triggered by `unit_registration()`. The plugin also stores the unit's affiliation as `0`.

### Radio Deregistration

Action: `off`

```text
<script> <shortName> <radioID> off
```

Triggered by `unit_deregistration()`. The plugin stores the unit's affiliation as `-1`.

### Acknowledge Response

Action: `ackresp`

```text
<script> <shortName> <radioID> ackresp
```

Triggered by `unit_acknowledge_response()`.

### Group Affiliation

Action: `join`

```text
<script> <shortName> <radioID> join <talkgroup> <patchedTalkgroups>
```

Triggered by `unit_group_affiliation()`. The plugin stores the unit's current affiliation as the talkgroup and includes a comma-separated patch list as the final argument.

### Data Grant

Action: `data`

```text
<script> <shortName> <radioID> data
```

Triggered by `unit_data_grant()`.

### Answer Request

Action: `ans_req`

```text
<script> <shortName> <radioID> ans_req <talkgroup>
```

Triggered by `unit_answer_request()`.

### Location

Action: `location`

```text
<script> <shortName> <radioID> location <talkgroup> <patchedTalkgroups>
```

Triggered by `unit_location()`. The plugin stores the unit's current affiliation as the talkgroup and includes a comma-separated patch list as the final argument.

### Call Start

Action: `call`

```text
<script> <shortName> <radioID> call <talkgroup> <patchedTalkgroups>
```

Triggered by `call_start()`. The plugin uses the current call source ID and includes a comma-separated patch list as the final argument.

## Patch List Behavior

For `join`, `location`, and `call`, the final argument is a comma-separated list of patched talkgroups. If there are no patches, the final argument will be an empty string.

## Important Notes

- The script is launched through the shell using `system(...)`, not `execve`, so script paths and arguments should be chosen carefully.
- The command is backgrounded with `&`, so the plugin does not wait for the script to finish.
- The implementation uses a fixed 200-character command buffer, so extremely long script paths or argument expansions may be truncated.
- This plugin reacts to unit and call events only. It does not process `call_end()`.

## Example Plugin Configuration

```json
{
  "plugins": [
    {
      "name": "Unit Scripts",
      "library": "libunit_script.so",
      "systems": [
        {
          "shortName": "county",
          "unitScript": "/opt/trunk-recorder/scripts/unit-script.sh"
        },
        {
          "shortName": "city",
          "unitScript": "/opt/trunk-recorder/scripts/city-unit-script.sh"
        }
      ]
    }
  ]
}
```

## Example Script Workflow

A common use case is to maintain a per-system radio list and daily radio activity log.

The older example script does the following:

- maintains `radiolist.csv` in the system shortName directory
- writes daily activity to `radiolog.csv` under the day's recording directory
- records rows in the form:

```text
radioID,timestamp,action,talkgroup
```

That older script also notes:

- `CAPTUREDIR` must match the `captureDir` path from `config.json`, without a trailing slash
- `radiolist.csv` should exist beforehand because the script relies on `sed`
- a midnight cron task can be used to create the next day's directory before the first recorded call of the day

## Example Script Notes

From the older example documentation:

```text
Creates a list of radio IDs in a file named "radiolist.csv" located in the
shortName directory along side the recordings, and logs radio activity to
"radiolog.csv" files located in each day's recordings directory

Make sure to fill in the CAPTUREDIR="" line with the path used in config.json, no ending /

file format: radioID,timestamp,action,talkgroup,
for radiolist.csv, acknowledgment response timestamps are added at the end
when they are seen after a different action

Feel free to customize the script; to use for multiple systems, include in
each system's config.json section

NOTE: You need to run "echo > radiolist.csv" where the file(s) is going to be
beforehand as sed doesn't work on empty files, and to capture actions before
trunk-recorder makes the daily directory upon recording the first call, set
up a cron task of: 0 0 * * * mkdir -p <capturedir>/$(date +\%Y/\%-m/\%-d/)
```