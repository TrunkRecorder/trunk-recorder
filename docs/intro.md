---
sidebar_label: 'Introduction'
sidebar_position: 1
---
# All About Trunk Recorder

## Sponsors
**Do you find Trunk Recorder and OpenMHz useful?** 
Become a [Sponsor](https://github.com/sponsors/robotastic) to help support continued development and operation!
Thank you to everyone who has contributed!

## Overview
Need help? Got something working? Share it!

[Discord Server](https://discord.gg/btJAhESnks) - and don't forget the [Wiki](https://github.com/TrunkRecorder/trunk-recorder/wiki)

Trunk Recorder is able to record the calls on trunked and conventional radio systems. It uses 1 or more Software Defined Radios (SDRs) to do this. The SDRs capture large swathes of RF and then use software to process what was received. [GNU Radio](https://gnuradio.org/) is used to do this processing because it provides lots of convenient RF blocks that can be pieced together to allow for complex RF processing. The libraries from the amazing [OP25](http://op25.osmocom.org/trac/wiki) project are used for a lot of the P25 functionality. Multiple radio systems can be recorded at the same time.


Trunk Recorder currently supports the following:

 - Trunked P25 & SmartNet Systems
 - Conventional P25 & analog systems, where each group has a dedicated RF channel
 - P25 Phase 1, P25 Phase 2 & Analog voice channels

Supported platforms:

**Ubuntu** (18.04,  20.04, 21.04, 22.04, 23.04); **Raspberry Pi** (Raspberry OS/Raspbian & Ubuntu 21.04, 22.04); **Arch Linux** (2021.09.20); **Debian** (9.x); **macOS**

GNU Radio 3.7 - 3.10

...and SDRs:

RTL-SDR dongles; HackRF; Ettus USRP B200, B210, B205; BladeRF; Airspy


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
* [Adjusting Squelch for Conventional Systems](docs/SQUELCH.md)
* [CTCSS / DCS Tone Squelch (analog conventional)](docs/TONE-SQUELCH.md)
* [FAQ](docs/FAQ.md)

### Troubleshooting

If are having trouble, check out the [FAQ](./FAQ.md) and/or ask a question on the [Discord Server](https://discord.gg/btJAhESnks) 

## How Trunking Works
For those not familiar, trunking systems allow a large number of user groups to share a limited number of radio frequencies by temporarily, dynamically assigning radio frequencies to talkgroups (channels) on-demand. It is understood that most user groups actually use the radio very sporadically and don't need a dedicated frequency. 

Most trunking system types (such as SmartNet and P25) set aside one of the radio frequencies as a "control channel" that manages and broadcasts radio frequency assignments. When someone presses the Push to Talk button on their radio, the radio sends a message to the system which then assigns a voice frequency and broadcasts a Channel Grant message about it on the control channel. This lets the radio know what frequency to transmit on and tells other radios set to the same talkgroup to listen.

In order to follow all of the transmissions, Trunk Recorder constantly listens to and decodes the control channel. When a frequency is granted to a talkgroup, Trunk Recorder creates a monitoring process which decodes the portion of the radio spectrum for that frequency from the SDR that is already pulling it in.

No message is transmitted on the control channel when a conversation on a talkgroup is over. The monitoring process keeps track of transmissions and if there has been no activity for a specified period, it ends the recording.
