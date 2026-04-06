---
sidebar_label: 'stat_socket'
---

# stat_socket Plugin

**Plugin name:** `stat_socket`  
**Library:** `libstat_socket.so`

The `stat_socket` plugin connects to a WebSocket server and streams live Trunk Recorder status and telemetry data. It sends configuration information, system status, recorder status, active calls, system rate updates, and optionally decoded signaling events.

## Plugin Object

| Key | Required | Default Value | Type | Description |
| --- | :---: | --- | --- | --- |
| name | ✓ | `stat_socket` | string | Friendly name for this plugin instance. |
| library | ✓ |  | string | Must be `libstat_socket.so`. |
| enabled |  | `true` | **true** / **false** | Whether this plugin instance should be loaded. |

## Plugin-Specific Settings

This plugin currently does **not** define any plugin-specific settings in its `parse_config()` method. Its behavior depends on the main Trunk Recorder configuration instead.

## Main Config Settings Used

The plugin uses the following main configuration values:

| Key | Required | Description |
| --- | :---: | --- |
| `statusServer` | ✓ | WebSocket server URL to connect to. If this is empty, the plugin does nothing.  |
| `instanceId` |  | Included in outbound status messages.  |
| `instanceKey` |  | Included in outbound status messages.  |
| `broadcastSignals` |  | When enabled, decoded signaling events are sent over the WebSocket connection.  |

## What the Plugin Sends

When connected, the plugin can send several message types over the WebSocket connection.

### Config Message

When the WebSocket connection opens, the plugin sends a `config` message containing:

- source configuration and source recorder counts
- system configuration and channel/control-channel lists
- instance-level settings such as `captureDir`, `uploadServer`, `callTimeout`, `instanceId`, and `instanceKey`
- optional `broadcast_signals` when enabled

### System Messages

The plugin sends:

- `systems` messages containing current system stats
- `system` messages when an individual system is set up or updated
- `rates` messages during `system_rates()` updates

### Recorder Messages

The plugin sends:

- `recorders` messages containing recorder stats
- `recorder` messages when an individual recorder is set up or updated

### Call Messages

The plugin sends:

- `calls_active` messages with active call stats
- `call_start` messages when a call begins

The current `call_end()` implementation does not send a `call_end` message. It returns immediately when called.

### Signaling Messages

When `broadcastSignals` is enabled, decoded signaling events can be sent as `signaling` messages. These may include:

- `unit_id`
- attached `call` stats
- attached `recorder` stats
- attached `system` stats

## Connection Behavior

- The plugin connects to the configured `statusServer` when it starts.
- It uses WebSocket++ with the non-TLS ASIO client configuration.
- If the connection closes or fails, it schedules a reconnect attempt after a delay based on the retry count plus a small random offset.
- Reconnect attempts are handled during `poll_one()`.
- The plugin tracks whether config has already been sent for the current connection and resends it after reconnect.

## Requirements

This plugin requires a valid WebSocket server URL in the main `statusServer` setting.

### Example

```json
{
  "statusServer": "ws://127.0.0.1:8080",
  "instanceId": "my-instance",
  "instanceKey": "my-secret",
  "broadcastSignals": true,
  "plugins": [
    {
      "name": "Status Socket",
      "library": "libstat_socket.so"
    }
  ]
}
```

If `statusServer` is empty, the plugin loads but does not open a connection. 

## Minimal Example

```json
{
  "statusServer": "ws://127.0.0.1:8080",
  "plugins": [
    {
      "name": "Status Socket",
      "library": "libstat_socket.so"
    }
  ]
}
```