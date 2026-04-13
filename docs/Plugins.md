---
sidebar_label: 'Plugins'
sidebar_position: 4
---

# Plugins

Plugins make it easy to customize Trunk Recorder and better fit it to your workflow. Some plugins are included with Trunk Recorder, and others are developed separately by the community.

To load a plugin, add it to the top-level `plugins` array in your `config.json` file.

```json
{
  "plugins": [
    {
      "name": "simplestream-main", 
      "library": "libsimplestream.so"
    }
  ]
}
```

For general configuration, see [Configure](./Configure.md).

## Plugin Object

Each entry in the top-level `plugins` array describes one plugin instance to load and the settings that should be passed to it.

Plugins are only loaded if they are explicitly listed in the `plugins` array. If multiple plugins are configured, they are initialized in the order they appear in the config, and their callbacks are run in that same order.

### Standard Plugin Object Keys

| Key     | Required | Default Value                 | Type                 | Description |
| ------- | :------: | ----------------------------- | -------------------- | ----------- |
| library |    ✓     |                               | string               | The shared library filename for the plugin, such as `libopenmhz_uploader.so`. |
| name    |    ✓     | derived from library filename | string               | Friendly name for this plugin instance. This name is used in logging and helps distinguish multiple instances of the same plugin. If omitted, Trunk Recorder currently derives the name from the library filename, but setting it explicitly is recommended. |
| enabled |          | true                          | **true** / **false** | Whether this configured plugin instance should be loaded. Disabled plugins are skipped. |

### Additional Plugin Settings

Plugins may define additional configuration keys beyond `library`, `name`, and `enabled`.

Any additional keys included in a plugin object are passed to that plugin during configuration parsing. The exact supported settings depend on the plugin you are using.

### Notes

- Each object in the `plugins` array creates a separate plugin instance.
- The same plugin library may be listed more than once if you need multiple differently configured instances.
- When using multiple instances of the same plugin, giving each one a unique `name` is strongly recommended.
- Only plugins listed in the `plugins` array are loaded.

## Included Trunk Recorder Plugins

The following plugins are included with Trunk Recorder. This page provides only a brief summary for each one. For configuration details, supported settings, and examples, open the linked plugin page.

### Rdio Scanner

**Plugin name:** `rdioscanner_uploader`  
**Library:** `librdioscanner_uploader.so`

Uploads recordings and call information to an Rdio Scanner server. Supports per-system configuration and optional talkgroup allow/deny filters.

More details: [Rdio Scanner Plugin](./plugins/rdio-scanner.md)

### OpenMHz

**Plugin name:** `openmhz_uploader`  
**Library:** `libopenmhz_uploader.so`

Uploads calls to OpenMHz.

More details: [OpenMHz Plugin](./plugins/openmhz.md)

### Broadcastify Calls

**Plugin name:** `broadcastify_uploader`  
**Library:** `libbroadcastify_uploader.so`

Uploads calls to Broadcastify Calls.

More details: [Broadcastify Calls Plugin](./plugins/broadcastify-calls.md)

### simplestream

**Plugin name:** `simplestream`  
**Library:** `libsimplestream.so`

Streams uncompressed PCM audio over UDP or TCP in real time while Trunk Recorder is recording.

More details: [simplestream Plugin](./plugins/simplestream.md)

### stat_socket

**Plugin name:** `stat_socket`  
**Library:** `libstat_socket.so`

Provides live status-style output and access to internal Trunk Recorder information useful for live updates or offline analysis.

More details: [stat_socket Plugin](./plugins/stat-socket.md)

### unit_script

**Plugin name:** `unit_script`  
**Library:** `libunit_script.so`

Handles unit event scripting. This plugin is associated with the `unitScript` system setting, but that setting alone does not load the plugin.

More details: [unit_script Plugin](./plugins/unit-script.md)

## Community Plugins

Community plugins can extend the features of Trunk Recorder and allow customized workflows or analysis.

> As new plugins are developed, authors are encouraged to add to the tables below by submitting a PR to this document.

Plugins that are built out-of-tree and installed separately from Trunk Recorder:

| Plugin Name / Link | Description |
| --- | --- |
| [MQTT Status](https://github.com/TrunkRecorder/trunk-recorder-mqtt-status) | Publishes the current status of a Trunk Recorder instance over MQTT |
| [MQTT Statistics](https://github.com/TrunkRecorder/trunk-recorder-mqtt-statistics) | Publishes statistics about a Trunk Recorder instance over MQTT |
| [Decode rates logger](https://github.com/rosecitytransit/trunk-recorder-decode-rate) | Logs trunking control channel decode rates to a CSV file and includes a PHP file that outputs an SVG graph |
| [Daily call log and live Web page](https://github.com/rosecitytransit/trunk-recorder-daily-log) | Creates a daily log of calls and includes an updating PHP Web page with audio player |
| [Prometheus exporter](https://github.com/USA-RedDragon/trunk-recorder-prometheus) | Publishes statistics to a metrics endpoint via HTTP |

### user_plugins

Plugins that are ready to clone into `/user_plugins` for automatic building and installation:

| Plugin Name / Link | Description |
| --- | --- |
| [Placeholder](https://github.com/tr_plugin_developer/my-tr-plugin.git) | Not a real plugin, but it could be. |

## Automatic Building and Development

As an alternative to developing out-of-tree, or within `/plugins`, user plugins may be staged into a subdirectory of `/user_plugins` to automatically compile as if they were a built-in plugin. Minor changes may be required for existing user plugins to benefit from automatic building, but this can simplify development and installation and help keep plugins up to date with changes to Trunk Recorder.

### Example

1. Clone the plugin repository:

```bash
cd /user_plugins
git clone https://github.com/tr_plugin_developer/my-tr-plugin.git
```

Or add it as a submodule:

```bash
cd /user_plugins
git submodule add https://github.com/tr_plugin_developer/my-tr-plugin.git
```

2. Review plugin requirements, and ensure all dependencies have been met.

3. Return to your build directory and resume from the `cmake` step:

```bash
cd trunk-build
cmake ../trunk-recorder
```

Near the end of the `cmake` output, the plugin should be listed:

```text
-- Added user plugin: my-tr-plugin
```

4. Continue to build and install Trunk Recorder with included plugins:

```bash
make
sudo make install
```

Return to the `cmake` step as you add or remove user plugins.

### Development Quick-Start

Any of the built-in plugins in `/plugins` can be directly copied to `/user_plugins` as a template for development. The `rdio_scanner` plugin is a good example of a curl-based uploader, and `stat_socket` shows how many internal Trunk Recorder methods can be accessed for live updates or offline analysis. Ensure that instances of the previous plugin name are changed in `CMakeFile.txt` to avoid conflicts with built-in plugins.