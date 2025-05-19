![Trunk Recorder](./docs/media/trunk-recorder-header.png)
---
[![Discord](./docs/media/discord.jpg)](https://discord.gg/btJAhESnks) &nbsp;&nbsp;


## Sponsors
**Do you find Trunk Recorder and OpenMHz useful?** 
Become a [Sponsor](https://github.com/sponsors/robotastic) to help support continued development and operation!
Thank you to everyone who has contributed!

## 🎉 V5.0 Our Best Release Yet!!
Thanks to everyone who contributed, tested and helped collect cored dumps! 

## Overview
Need help? Got something working? Share it!


- [Discord Server](https://discord.gg/btJAhESnks) 

- [Documentation](https://trunkrecorder.com/docs/intro)

- ... and don't forget the [Wiki](https://github.com/robotastic/trunk-recorder/wiki)

![screenshot](./docs/media/screenshot.jpg)

Trunk Recorder is able to record calls on trunked and conventional radio systems. It uses 1 or more Software Defined Radios (SDRs) to do this. The SDRs capture large swathes of RF and then use software to process what was received. [GNU Radio](https://gnuradio.org/) is used to do this processing because it provides lots of convenient RF blocks that can be pieced together to allow for complex RF processing. The libraries from the amazing [OP25](https://osmocom.org/projects/op25/wiki) project are used for a lot of the P25 functionality. Multiple radio systems can be recorded at the same time.


Trunk Recorder currently supports the following:

 - Trunked P25 & SmartNet Systems
 - Conventional P25, DMR & analog systems, where each talkgroup has a dedicated RF channel
 - P25 Phase 1, P25 Phase 2 & Analog voice channels

### Supported platforms

- **Ubuntu**
- **Raspberry Pi** (Raspberry OS/Raspbian & Ubuntu) 
- **Arch Linux** 
- **Debian** 
- **macOS**

GNU Radio 3.7 - 3.10

### SDRs

RTL-SDR dongles; HackRF; Ettus USRP B200, B210, B205; BladeRF; Airspy; SDRplay


## Install

### Linux
- [Docker](docs/Install/INSTALL-DOCKER.md) 
- [From Source](docs/Install/INSTALL-LINUX.md)

### Raspberry Pi
- [Docker](docs/Install/INSTALL-DOCKER.md) 
- [From Source](docs/Install/INSTALL-PI.md) - [Video Walkthrough](https://youtu.be/DizBtDZ6kE8)

### MacOS
- [From Source](docs/Install/INSTALL-MAC.md#using-homebrew)



## Setup
* [Configuring a system](docs/CONFIGURE.md)
* [Uploading to OpenMHz](./docs/OpenMHz.md)
* [FAQ](docs/FAQ.md)


### Playback & Sharing
By default, Trunk Recorder just dumps a lot of recorded files into a directory. Here are a couple of options to make it easier to browse through recordings and share them on the Internet.
* [OpenMHz](https://github.com/robotastic/trunk-recorder/wiki/Uploading-to-OpenMHz): This is my free hosted platform for sharing recordings
* [Trunk Player](https://github.com/ScanOC/trunk-player): A great Python based server, if you want to you want to run your own
* [Rdio Scanner](https://github.com/chuot/rdio-scanner): Provide a good looking, scanner style interface for listening to Trunk Recorder
* Broadcastify Calls (API): see Radio Reference [forum thread](https://forums.radioreference.com/threads/405236/) and [wiki page](https://wiki.radioreference.com/index.php/Broadcastify-Calls-Trunk-Recorder)
* [Broadcastify via Liquidsoap](https://github.com/robotastic/trunk-recorder/wiki/Streaming-online-to-Broadcastify-with-Liquid-Soap)
* [audioplayer.php](https://github.com/robotastic/trunk-recorder/wiki/Using-audioplayer.php)
* [rosecitytransit's Live Web page](https://github.com/rosecitytransit/trunk-recorder-daily-log)

### Plugins

* [MQTT Status](https://github.com/taclane/trunk-recorder-mqtt-status): Publishes the current status of a Trunk Recorder instance over MQTT
* [MQTT Statistics](https://github.com/robotastic/trunk-recorder-mqtt-statistics): Publishes statistics about a Trunk Recorder instance over MQTT
* [Decode rates logger](https://github.com/rosecitytransit/trunk-recorder-decode-rate): Logs trunking control channel decode rates to a CSV file, and includes a PHP file that outputs an SVG graph
* [Daily call log and live Web page](https://github.com/rosecitytransit/trunk-recorder-daily-log): Creates a daily log of calls (instead of just individual JSON files) and includes an updating PHP Web page w/audio player
* [Prometheus exporter](https://github.com/USA-RedDragon/trunk-recorder-prometheus): Publishes statistics to a metrics endpoint via HTTP

### Troubleshooting

If are having trouble, check out the [FAQ](docs/FAQ.md) and/or ask a question on the [Discord Server](https://discord.gg/btJAhESnks) 


## How Trunking Works
For those not familiar, trunking systems allow a large number of user groups to share a limited number of radio frequencies by temporarily, dynamically assigning radio frequencies to talkgroups (channels) on-demand. It is understood that most user groups actually use the radio very sporadically and don't need a dedicated frequency. 

Most trunking system types (such as SmartNet and P25) set aside one of the radio frequencies as a "control channel" that manages and broadcasts radio frequency assignments. When someone presses the Push to Talk button on their radio, the radio sends a message to the system which then assigns a voice frequency and broadcasts a Channel Grant message about it on the control channel. This lets the radio know what frequency to transmit on and tells other radios set to the same talkgroup to listen.

In order to follow all of the transmissions, Trunk Recorder constantly listens to and decodes the control channel. When a frequency is granted to a talkgroup, Trunk Recorder creates a monitoring process which decodes the portion of the radio spectrum for that frequency from the SDR that is already pulling it in.

No message is transmitted on the control channel when a conversation on a talkgroup is over. The monitoring process keeps track of transmissions and if there has been no activity for a specified period, it ends the recording.
