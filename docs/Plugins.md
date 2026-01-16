---
sidebar_label: 'Plugins'
sidebar_position: 4
---

# Plugins
Plugins make it easy to customize Trunk Recorder and have it better fit you workflow. There are some built-in Plugins that are included in Trunk Recorder and also ones that the Community has developed.

The [Built-in Plugins](#built-in-plugins) are compiled and installed when you setup Trunk Recorder. Follow the instructions for the [Community Plugins](#community-plugins) to install them. In order to load a plugin when you start Trunk Recorder, you need to add a **plugins** section to your **config.json** file. A [Plugin JSON Object](#plugin-object) is add for each of the Plugins you wish to load. The Plugin Object tells Trunk Recorder how to load it and its configuration.

```json
{
  Global Configs

  "sources": [{ Source Object }, { Source Object }],
  "systems": [{ System Object }, { System Object }],
  "plugins": [{ Plugin Object }]
}
```


#### Plugin Object

| Key     | Required | Default Value | Type                 | Description                                                                                                                 |
| ------- | :------: | ------------- | -------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| library |    ✓     |               | string               | the name of the library that contains the plugin.                                                                           |
| name    |    ✓     |               | string               | the name of the plugin. This name is used to find the `<name>_plugin_new` method that creates a new instance of the plugin. |
| enabled |          | true          | **true** / **false** | control whether a configured plugin is enabled or disabled                                                                  |
|         |          |               |                      | *Additional elements can be added, they will be passed into the `parse_config` method of the plugin.*                       |


## Built-in Plugins

##### Rdio Scanner Plugin

**Name:** rdioscanner_uploader  
**Library:** librdioscanner_uploader.so

This plugin makes it easy to connect Trunk Recorder with [Rdio Scanner](https://github.com/chuot/rdio-scanner). It uploads recordings and the information about them. The following additional settings are required:

| Key     | Required | Default Value | Type   | Description                                                                                                                                                                |
| ------- | :------: | ------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| server  |    ✓     |               | string | The URL for uploading to Rdio Scanner. The default is an empty string. It should be the same URL as the one you are using to access Rdio Scanner.                          |
| systems |    ✓     |               | array  | This is an array of objects, where each is a system that should be passed to Rdio Scanner. More information about what should be in each object is in the following table. |

*Rdio Scanner System Object:*

| Key              | Required | Default Value | Type   | Description                                                                                                                                                                                                 |
| ---------------- | :------: | ------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| systemId         |    ✓     |               | number | System ID for Rdio Scanner.                                                                                                                                                                                 |
| apiKey           |    ✓     |               | string | System-specific API key for uploading calls to Rdio Scanner. See the ApiKey section in the Rdio Scanner administrative dashboard for the value it should be.                                                |
| shortName        |    ✓     |               | string | This should match the shortName of a system that is defined in the main section of the config file.                                                                                                         |
| talkgroupWhitelist |        | []            | array  | Optional allow-list of talkgroups to upload for this system. If set (non-empty), the talkgroup **must** match at least one pattern to be uploaded. Patterns are glob-style (supports `*` and `?`).        |
| talkgroupBlacklist |        | []            | array  | Optional deny-list of talkgroups to block upload for this system. If set (non-empty), any matching talkgroup will be skipped. Patterns are glob-style (supports `*` and `?`).                             |

**Talkgroup filter rules (per-system):**
- Talkgroup comparisons are done against the numeric talkgroup ID as a string (e.g. `50712`).
- Patterns are glob-style:
    - `*` matches any number of characters
    - `?` matches a single character
- If `talkgroupWhitelist` is provided and non-empty, the talkgroup must match at least one whitelist pattern (otherwise it is skipped).
- If `talkgroupBlacklist` is provided and non-empty, the talkgroup must **not** match any blacklist pattern (otherwise it is skipped).
- If both are provided, the whitelist check is applied first, then the blacklist check.

**Examples:**
- Allow only talkgroups starting with `507`:
    - `talkgroupWhitelist: ["507*"]`
- Block a specific talkgroup:
    - `talkgroupBlacklist: ["12345"]`
- Block a range/prefix while allowing others:
    - `talkgroupBlacklist: ["99*"]`

##### Example Plugin Object:

```json
{
  "name": "rdioscanner_uploader", 
  "library": "librdioscanner_uploader.so", 
  "server": "http://127.0.0.1", 
  "systems": [
    {
      "shortName": "test", 
      "apiKey": "fakekey", 
      "systemId": 411,

      "talkgroupWhitelist": ["507*"],
      "talkgroupBlacklist": ["507999", "12345"]
    }
  ]
}
```


##### simplestream Plugin

**Name:** simplestream
**Library:** libsimplestream.so

This plugin streams uncompressed audio (16 bit Int, 8 kHz, mono) to UDP or TCP ports in real time as it is being recorded by trunk-recorder.  It can be configured to stream audio from all talkgroups and systems being recorded or only specified talkgroups and systems.  TGID information can be prepended to the audio data to allow the receiving program to take action based on the TGID.  Audio from different Systems should be streamed to different UDP/TCP ports to prevent crosstalk and interleaved audio from talkgroups with the same TGID on different systems.

This plugin does not, by itself, stream audio to any online services.  Because it sends uncompressed PCM audio, it is not bandwidth efficient and is intended mostly to send audio to other programs running on the same computer as trunk-recorder or to other computers on the LAN.  The programs receiving PCM audio from this plugin may play it on speakers, compress it and stream it to an online service, etc.

**NOTE 1: In order for this plugin to work, the audioStreaming option in the Global Configs section (see above) must be set to true.**

**NOTE 2: trunk-recorder passes analog audio to this plugin at 16 kHz sample rate and digital audio at 8 kHz sample rate.  Since the audio data being streamed doesn't contain the sample rate, analog and digital audio should be configured to be sent to different ports to receivers that are matched to the same sample rate.**

| Key     | Required | Default Value | Type  | Description                                                                                                                                                                                         |
| ------- | :------: | ------------- | ----- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| streams |    ✓     |               | array | This is an array of objects, where each is an audio stream that will be sent to a specific IP address and UDP port. More information about what should be in each object is in the following table. |

*Audio Stream Object:*

| Key       | Required | Default Value | Type                 | Description                                                                                                                                                                                                                                                                                                                                                                                                         |
| --------- | :------: | ------------- | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| address   |    ✓     |               | string               | IP address to send this audio stream to.  Use "127.0.0.1" to send to the same computer that trunk-recorder is running on.                                                                                                                                                                                                                                                                                           |
| port      |    ✓     |               | number               | UDP or TCP port that this stream will send audio to.                                                                                                                                                                                                                                                                                                                                                                |
| TGID      |    ✓     |               | number               | Audio from this Talkgroup ID will be sent on this stream.  Set to 0 to stream all recorded talkgroups.                                                                                                                                                                                                                                                                                                              |
| sendTGID  |          | false         | **true** / **false** | When set to true, the TGID will be prepended in long integer format (4 bytes, little endian) to the audio data each time a packet is sent.                                                                                                                                                                                                                                                                          |
| shortName |          |               | string               | shortName of the System that audio should be streamed for.  This should match the shortName of a system that is defined in the main section of the config file.  When omitted, all Systems will be streamed to the address and port configured.  If TGIDs from Systems overlap, each system must be sent to a different port to prevent interleaved audio for talkgroups from different Systems with the same TGID. |
| useTCP    |          | false         | **true** / **false** | When set to true, TCP will be used instead of UDP.                                                                                                                                                                                                                                                                                                                                                                  |

###### Plugin Object Example #1:
This example will stream audio from talkgroup 58914 on system "CountyTrunked" to the local machine on UDP port 9123.
```yaml
        {
          "name":"simplestream",
          "library":"libsimplestream.so",
          "streams":[{
            "TGID":58914,
            "address":"127.0.0.1",
            "port":9123,
            "sendTGID":false,
            "shortName":"CountyTrunked"}
        }
```

###### Plugin Object Example #2:
This example will stream audio from talkgroup 58914 from System CountyTrunked to the local machine on UDP port 9123 and stream audio from talkgroup 58916 from System "StateTrunked" to the local machine on UDP port 9124.
```yaml
        {
          "name":"simplestream",
          "library":"libsimplestream.so",
          "streams":[{
            "TGID":58914,
            "address":"127.0.0.1",
            "port":9123,
            "sendTGID":false,
            "shortName":"CountyTrunked"},
           {"TGID":58916,
            "address":"127.0.0.1",
            "port":9124,
            "sendTGID":false,
            "shortName":"StateTrunked"}
          ]}
        }
```

###### Plugin Object Example #3:
This example will stream audio from talkgroups 58914 and 58916 from all Systems to the local machine on the same UDP port 9123.  It will prepend the TGID to the audio data in each UDP packet so that the receiving program can differentiate the two audio streams (the receiver may decide to only play one depending on priority, mix the two streams, play one left and one right, etc.)
```yaml
        {
          "name":"simplestream",
          "library":"libsimplestream.so",
          "streams":[{
            "TGID":58914,
            "address":"127.0.0.1",
            "port":9123,
            "sendTGID":true},
           {"TGID":58916,
            "address":"127.0.0.1",
            "port":9123,
            "sendTGID":true}
          ]}
        }
```
###### Plugin Object Example #4:
This example will stream audio from all talkgroups being recorded on System CountyTrunked to the local machine on UDP port 9123.  It will prepend the TGID to the audio data in each UDP packet so that the receiving program can decide which ones to play or otherwise handle)
```yaml
        {
          "name":"simplestream",
          "library":"libsimplestream.so",
          "streams":[{
            "TGID":0,
            "address":"127.0.0.1",
            "port":9123,
            "sendTGID":true,
            "shortName":"CountyTrunked"}
        }
```
##### Example - Sending Audio to pulseaudio
pulseaudio is the default sound system on many Linux computers, including the Raspberry Pi.  If configured to do so, pulseaudio can accept raw audio via TCP connection using the module-simple-protocol-tcp module.  Each TCP connection will show up as a different "application" in the pavucontrol volume mixer.

An example command to set up pulseaudio to receive 8 kHz digital audio from simplestream on TCP port 9125 (for 16 kHz analog audio, use `rate=16000`):
```
pacmd load-module module-simple-protocol-tcp sink=1 playback=true port=9125 format=s16le rate=8000 channels=1
```
The matching simplestream config to send audio from talkgroup 58918 to TCP port 9125 would then be something like this:
```yaml
        {
          "name":"simplestream",
          "library":"libsimplestream.so",
          "streams":[{
            "TGID":58918,
            "address":"127.0.0.1",
            "port":9125,
            "sendTGID":false,
            "shortName":"CountyTrunked",
            "useTCP":true}
        }
```

## Community Plugins
Community plugins can extend the features of Trunk Recorder and allow customized workflows or analysis.  
> As new plugins are developed, authors are encouraged to add to the below tables by submitting a PR to this document.

#### External Plugins
Plugins that are built out-of-tree and installed separately from Trunk Recorder:
| Plugin Name / Link                                                                              | Description                                                                                                               |
| ----------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| [MQTT Status](https://github.com/TrunkRecorder/trunk-recorder-mqtt-status)                         | Publishes the current status of a Trunk Recorder instance over MQTT                                                       |
| [MQTT Statistics](https://github.com/TrunkRecorder/trunk-recorder-mqtt-statistics)                 | Publishes statistics about a Trunk Recorder instance over MQTT                                                            |
| [Decode rates logger](https://github.com/rosecitytransit/trunk-recorder-decode-rate)            | Logs trunking control channel decode rates to a CSV file, and includes a PHP file that outputs an SVG graph               |
| [Daily call log and live Web page](https://github.com/rosecitytransit/trunk-recorder-daily-log) | Creates a daily log of calls (instead of just individual JSON files) and includes an updating PHP Web page w/audio player |
| [Prometheus exporter](https://github.com/USA-RedDragon/trunk-recorder-prometheus)               | Publishes statistics to a metrics endpoint via HTTP                                                                       |

#### user_plugins
Plugins that are ready to clone into `/user_plugins` for automatic building and installation:
| Plugin Name / Link                                                     | Description                         |
| ---------------------------------------------------------------------- | ----------------------------------- |
| [Placeholder](https://github.com/tr_plugin_developer/my-tr-plugin.git) | Not a real plugin, but it could be. |

## Automatic Building and Development
As an alternative to developing out-of-tree, or within `/plugins`, user plugins may be staged into a subdirectory of `/user_plugins` to automatically compile as if they were a built-in plugin.  Minor changes may be required for exsting user plugins to benefit from automatic building, but this should simplify the process of development and installation, and ensure plugins remain up-to-date with any changes to Trunk Recorder.

#### Example
1. Clone the plugin repository
```bash
cd /user_plugins
git clone https://github.com/tr_plugin_developer/my-tr-plugin.git
```
&emsp; or add it as a submodule.  
```bash
cd /user_plugins
git submodule add https://github.com/tr_plugin_developer/my-tr-plugin.git
```

2. Review plugin requirements, and ensure all dependencies have been met.

3. Return to your 'build' directory, and resume from the `cmake` step: e.g.
```bash
cd trunk-build
cmake ../trunk-recorder
```
&emsp; Note that the plugin is displayed near the end of the cmake output:
```
-- Added user plugin: my-tr-plugin
```

4. Continue to build and install Trunk Recorder with included plugins:
```bash
make
sudo make install
```
&emsp; Return to the `cmake` step as you add or remove user plugins.

### Development Quick-Start
Any of the built-in plugins in `/plugins` can be directly copied to `/user_plugins` as a template for development.  The `rdio_scanner` plugin is a good example of a curl-based uploader, and `stat_socket` shows how many of the internal Trunk Recorder methods can be accessed for live updates or offline analysis.  Ensure that instances of the previous plugin name are changed in `CMakeFile.txt` to avoid any conflicts with built-in plugins.
