Trunk Recorder ChangeLog
========================
### Unreleased
* Update simpstream.cc by @aaknitt in #1024
* Multisite should be boolean, not string by @ancker010 #1026
* Some documentation updates by @SimonSheehan in #1019 and #1023
* When logging Control Channel decode rate warnings, include the Control Channel's frequency by @gofaster in #1030
* Removed boost 'system' required component by @taclane in #1037

### Version 5.0.2 - Quality of Life Improvements
* Hostname resolution cache for upload plugins by @taclane in #973
* fmt 11 compile fix by @taclane in #977
* Update INSTALL-PI.md by @Dygear in #979
* Configurable de-emphasis time constant for analog recorders by @algertc in #975
* Update unit to unit recording config documentation. by @tadscottsmith in #987
* Fixed Raspberry Pi Docker Install Typo by @avery1227 in #983
* Docker Build: Only build librtlsdr 1x, install 2x by @mkmer in #986
* Correct spelling of transmissions. by @tadscottsmith in #982
* auto-restart.sh - Enable Email Alerts by @decoy4fun in #984
* For testing / future - Ubuntu 24.04 Dockerfile by @jquagga in #953
* don't remove call when upload failed by @rosecitytransit in #939
* Dev/backlog by @robotastic in #991
* Motorola opcode 0x05 0x09 fix by @taclane in #992
* fix(add openssh-client and ffmpeg) by @jared-bloomer in #1003
* Min Duration to Info instead of error by @decoy4fun in #1001
* Initialize noise value by @taclane in #1006
*  Update main.yml by @robotastic in #1012
* static_cast start_time by @taclane in #1013
* Quit if SDR drops by @robotastic in #1015
* Updated json.hpp library by @robotastic in #1016

### Version 5.0.1
* Addition of P25 custom bandplans by @tadscottsmith in https://github.com/TrunkRecorder/trunk-recorder/pull/968

### Version 5.0.0 - The best release yet! 
* Version 5 of Trunk Recorder involved many commits and changes. A full list can be found here, but some will be summarized below. https://github.com/TrunkRecorder/trunk-recorder/pull/963/
* Channelizer block improvements
* Significant upgrades to analog recording, squelch, and recorders.
* Updates to Rdio Scanner, OpenMhz, and Broadcastify uploader plugins.
* Signal detector improvements.
* Significant improvements to overall performance and many bugs fixed. 
* Improved CSV/JSON handling 

### Version 4.6.0
* Cleaned up Rdio Scanner talkgroup display logs. by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/750
* adjust Source sample rate to match SDR sample rate by @aaknitt in https://github.com/robotastic/trunk-recorder/pull/753
* Create log file earlier by @aaknitt in https://github.com/robotastic/trunk-recorder/pull/754
* Enable/Disable config sections by @taclane in https://github.com/robotastic/trunk-recorder/pull/756
* broadcastify: add option to disable ssl certificate verification by @galenguyer in https://github.com/robotastic/trunk-recorder/pull/757
* Update signaling decoders to print logs via boost info log instead of plain stdout by @EricTendian in https://github.com/robotastic/trunk-recorder/pull/759
* help screen fix by @taclane in https://github.com/robotastic/trunk-recorder/pull/767
* Adding support for regex in unitTagsFile by @EricTendian in https://github.com/robotastic/trunk-recorder/pull/764
* Enable Multi-site mode features. by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/758
* Use more compatible cmake command by @dotsam in https://github.com/robotastic/trunk-recorder/pull/768
* Multi-site bug fixes and improvements. by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/770
* Clean up a few debug warning messages. by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/771
* Update Homebrew instructions by @taclane in https://github.com/robotastic/trunk-recorder/pull/776
* use iterators for std::fill, fix out of bounds access by @Jannik2099 in https://github.com/robotastic/trunk-recorder/pull/782
* Console log cleanup by @taclane in https://github.com/robotastic/trunk-recorder/pull/784
* OpenMHz Uploader update by @robotastic in https://github.com/robotastic/trunk-recorder/pull/794
* Spelling fixes and additional call stat reporting by @taclane in https://github.com/robotastic/trunk-recorder/pull/798
* Ignore missing files by @devicenull in https://github.com/robotastic/trunk-recorder/pull/796
* UnitID alpha improvements and decode error display by @taclane in https://github.com/robotastic/trunk-recorder/pull/785
* Fix freq offset variable declarations in trunking messages by @taclane in https://github.com/robotastic/trunk-recorder/pull/802
* Update call_concluder.h by @taclane in https://github.com/robotastic/trunk-recorder/pull/801
* updates to the latest version of OP25 by @robotastic in https://github.com/robotastic/trunk-recorder/pull/803
* actual crash fix this time by @gisforgirard in https://github.com/robotastic/trunk-recorder/pull/800
* Remove sys num by @robotastic in https://github.com/robotastic/trunk-recorder/pull/805
* Fix p25 parse by @robotastic in https://github.com/robotastic/trunk-recorder/pull/806
* update boost log line by @taclane in https://github.com/robotastic/trunk-recorder/pull/804
* Fixes retuning the CC when it has to move to a new source by @robotastic in https://github.com/robotastic/trunk-recorder/pull/810
* Feature/message hook by @robotastic in https://github.com/robotastic/trunk-recorder/pull/814
* update/edit README.md and docs/CONFIGURE.md, 3 minor fixes by @rosecitytransit in https://github.com/robotastic/trunk-recorder/pull/816
* Release/new call mgmt by @robotastic in https://github.com/robotastic/trunk-recorder/pull/817
* P25 Trunking Message Improvements by @robotastic in https://github.com/robotastic/trunk-recorder/pull/820
* add option to disable output to console, docs/CONFIGURE.md formatting by @rosecitytransit in https://github.com/robotastic/trunk-recorder/pull/819
* Feature/temp dir by @robotastic in https://github.com/robotastic/trunk-recorder/pull/823
* correct callLog option in docs/CONFIGURE.md to true to match code by @rosecitytransit in https://github.com/robotastic/trunk-recorder/pull/824
* Per-day call log plugin, log P25 priority/mode/duplex by @rosecitytransit in https://github.com/robotastic/trunk-recorder/pull/482

### Version 4.5.0
* Set d_call to null to prevent potential use after free by @galenguyer in https://github.com/robotastic/trunk-recorder/pull/739
* Documenting maxDuration config param by @EricTendian in https://github.com/robotastic/trunk-recorder/pull/741
* Conventional recorder info for MQTT and status-server plugins by @taclane in https://github.com/robotastic/trunk-recorder/pull/744
* Cleanup dropped samples logging. by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/745
* Multiple CMake fixes and version updates to resolve compiler warnings by @galenguyer in https://github.com/robotastic/trunk-recorder/pull/738
* Log cleanup by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/749

### Version 4.4.2
* Fix logging when all transmissions are removed. by @tadscottsmith in #735
* Fixes issue mentioned in #691 by @Dygear in #733
* Improve CSV parsing to allow quoted and escaped fields by @EricTendian in #734
* Updated Broadcastify Uploader to make it more robust by @robotastic in #736

### Version 4.4.1
* Fix for #606 by @JoeGilkey in #721
* Improved Talkgroup Priority Logging by @tadscottsmith in #700
* Build gr-osmosdr in Docker with multiple Airspy source patch by @EricTendian in #727

### Version 4.4.0
* Add support for using a .csv file for adding for conventional channels
* Removed alphaTags support from config.json, use the channel.csv instead
* M/A-COM patch following enhancement by @aaknitt in https://github.com/robotastic/trunk-recorder/pull/643
* Conventional Channel Information by @robotastic in https://github.com/robotastic/trunk-recorder/pull/652
* Note on column headings for priority detection by @hayden-t in https://github.com/robotastic/trunk-recorder/pull/650
* Add TCP support to simplestream plugin by @aaknitt in https://github.com/robotastic/trunk-recorder/pull/649
* Add per-system configuration option for minimum transmission length. by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/654
* GNU Radio typo by @777arc in https://github.com/robotastic/trunk-recorder/pull/655
* Fix op25 errors by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/659
* Report Errors/Spikes to Rdio Scanner by @taclane in https://github.com/robotastic/trunk-recorder/pull/660
* Set alpha tag for conventional talkgroup and restore startup console message by @taclane in https://github.com/robotastic/trunk-recorder/pull/662
* Log data channel grants, unit answer requests, Location Reg Responses by @rosecitytransit in https://github.com/robotastic/trunk-recorder/pull/661
* minor fix to unit answer reqest logging by @rosecitytransit in https://github.com/robotastic/trunk-recorder/pull/663
* a new approach for tracking call timeouts  by @robotastic in https://github.com/robotastic/trunk-recorder/pull/664
* Simplestream fixes by @aaknitt in https://github.com/robotastic/trunk-recorder/pull/671
* Fix missing audio on rdio-scanner web and mobile devices by @kb2ear in https://github.com/robotastic/trunk-recorder/pull/668
* Update macOS Build Instructions by @dechilders in https://github.com/robotastic/trunk-recorder/pull/680
* Add config option to not record Unit-to-Unit voice calls by @tadscottsmith in https://github.com/robotastic/trunk-recorder/pull/667
* Public Plugin API by @robotastic in https://github.com/robotastic/trunk-recorder/pull/681
* Remove extra characters by @dechilders in https://github.com/robotastic/trunk-recorder/pull/684
* Raspberry Pi Bullseye & Ubuntu 22.04 Updates. by @Dygear in https://github.com/robotastic/trunk-recorder/pull/685
* added --version switch to pull both github and CMake version information as well as log the Version info into the log file by @rabarar in https://github.com/robotastic/trunk-recorder/pull/688
* Remove nested BOOST_LOG_TRIVIAL to fix erroneous log level display by @taclane in https://github.com/robotastic/trunk-recorder/pull/692
* Add initial support for gnuradio 3.10. by @geezer85 in https://github.com/robotastic/trunk-recorder/pull/682
* Added debian notes to PI install. by @Dygear in https://github.com/robotastic/trunk-recorder/pull/695
* Update INSTALL-LINUX.md by @joe00dev in https://github.com/robotastic/trunk-recorder/pull/696
* Include boost/filesystem header by @geezer85 in https://github.com/robotastic/trunk-recorder/pull/698
* P25p2 uid by @robotastic in https://github.com/robotastic/trunk-recorder/pull/708
* Op25 june 22 by @robotastic in https://github.com/robotastic/trunk-recorder/pull/709
* mark one function visible to allow shared build by @ZeroChaos- in https://github.com/robotastic/trunk-recorder/pull/720
* modified version switch to build without .git repo directory and adde… by @rabarar in https://github.com/robotastic/trunk-recorder/pull/715

### Version 4.3.0
* Add support for DMR / MotoTRBO


### Version 4.2.0
* Added support for tracking System Patches
* Simple Streaming Plugin
* Switch from Floats to int16 for decoder->wav
* Resets the garnder costas clock in convetional mode after each transmission

### Version 4.1.1
* Got the Wrong docker package by @MaxwellDPS in #555
* Plugin rdioscanner by @chuot in #557
* Plugin Audio Streaming CPU/Memory Usage Reduction by @JoeGilkey in #559
* Significant improvements to audioplayer by @devicenull in #549
* Read support for reading in direct CSV exports from Radio Reference by @devicenull in #547
* Fix compile errors on Fedora 35 by @chuot in #560

### Version 4.1.0
* pulls in the latest version of OP25

### Version 4.0.3
* Fixed support for P25 Conventional recorders. 

### Version 4.0
* The executable generated has changed from `recorder` to `trunk-recorder` to help prevent differentiate it from other applications that maybe instaslled.
* A new method is used to detect the end of call. Instead of waiting fora timeout after the last trunking message is recieved, the voice channel is monitored for messages announcing the end of a transmission. Each transmission is stored in a separate file and then merged together after a talkgroup stop using a frequency.
* The format for audio filenames has changed slightly. It is now: [ Talkgroup ID ]\_[ Unix Timestamp ]-[ Frequency ]-call\_[ Call Counter ].wav


### Version 3.3
* Changes the format of the config.json file. Modulation type, Squelch and audio levels are now set in each System instead of under a Source. See sample config files in the /example folder. 
* Config files are also now versioned, to help catch misconfigurations. After you have updated your config file, add "ver": 2, to the top. The processing of SmartNet talkgroup numbers as also been fixed. 
* The decimal talkgroup numbers will now match what is in Radio Reference. Please update your talkgroup.csv, if needed.

### Version 3.2
* Added support for GnuRadio 3.8

### Version 3.1.3
* Add support for uploading to Broadcastify
* Switch to using LibCurl for networking

### Version 3.1.2
* Added support for Motorola TPS Signaling
* Made Debug Recorder work over a nextwork connection

### Version 3.1.1
* Switched to use `fdkaac` & `sox` instead of `ffmpeg` to compress audio files because it is a lot easier to install
* changed to buffer sizes for blocks to a minimum to help prevent audio from getting left-over in a recorder
* prevented a recording from moving from one source to another. This likely causes audio to get jammed in a recorders buffers because a recording is stopped and things don't have a chance to flush.
* Pulled in a change request from Joe Gilkey that adds Unit Names and support for a number of analog data overlays.

### Version 3.1.0
* Updated to the latest version of OP25.
* Updated P25 Recorder, P25 Trunking and SmartNet Trunking to use the double decimation technique from OP25. It should handle SDRs with a high sample rate better now.
* Updated to the latest version of the websocketpp library.

### Version 3.0.1
* Updated to the latest version of OP25. Supposed performance improvements.

### Version 3.0
*I really have to do a better job of tracking changes... I think I got most items.*
* Bug fixes on Phase 2 decoding, catching some additional messages.
* Sending status over a WebSocket

### Version 2.2.1
* Just capturing random bug fixes

### Version 2.2
Lot's of fun things added!
* P25 Phase 2 support was added. It should just work and the P25 recorder should automatically switch between Phase 1 and Phase 2 depending on what it sees on the trunking channel.
* SmartNet BandPlan support - lets you set configs in the config.json file to specify which type of bandplan to use or define a custom one. If anyone is using VHF systems, let me know... work it probably still needed.
* Error logging - P25 recorders now keep tracking of the number of errors during decoded for each call recording. This is spit out in the Freq List in the JSON file and uploaded to openMHz.
* audioArchive - config lets you select if you don't want to save the audio recordings after they are uploaded.
* callLog - config lets you turn off JSON file creation.
* logs - I changed the formatting of the logs that are printed to the screen.
* Specify the config file from the command line using the `--config=x.json` argument.

### Version 2.1.2
* Added support for Conventional Analog and P25 systems
* Improved FSK4 performance

### Version 2.0
* Adds support for recording multiple systems and switches to using FFT for recorders.
