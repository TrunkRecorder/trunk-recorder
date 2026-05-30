# Trunk Recorder — macOS Auto-Start Setup Guide

This document covers how to set up a Trunk Recorder instance on macOS so that it starts automatically on boot, stays running after crashes, and writes logs you can monitor.

---

## Overview

Two components work together to keep Trunk Recorder running:

1. **A launcher script** — sets up the environment and starts `trunk-recorder`. launchd restarts it automatically if it exits.
2. **A LaunchAgent plist** — tells macOS to run the script at login, keep it alive, and capture its output to a log file.

Because Trunk Recorder talks directly to USB SDR hardware it must run inside a user session — not as a root system daemon — so that macOS grants it access to USB devices. Enabling automatic login (see section 3) means the session starts immediately at boot with no one at the keyboard.

---

## Directory Layout

All files live under a single working directory. Trunk Recorder writes recordings into subdirectories named after each system's `shortName` from the config file.

```
trunk-build/
├── trunk-recorder          # compiled binary
├── config-<system>.json    # system config
├── start-<system>.sh       # launcher script
└── <shortName>/            # recordings written here automatically
```

Trunk Recorder must be started with its **working directory set to `trunk-build/`** so that relative paths in the config and recording output directories resolve correctly. The LaunchAgent plist handles this via `WorkingDirectory`.

---

## 1. Launcher Script

The script sets up the environment and hands off to `trunk-recorder` using `exec`. The `exec` replaces the bash process with trunk-recorder directly, so launchd tracks the correct PID and restarts it via `KeepAlive` when it exits.

**`start-<system>.sh`**
```bash
#!/bin/bash
ulimit -c unlimited
eval "$(/opt/homebrew/bin/brew shellenv)"

# Only needed for USRP/UHD-based SDRs — remove if using RTL-SDR or other drivers
export UHD_IMAGES_DIR=/usr/share/uhd/images/

exec ./trunk-recorder --config=config-<system>.json
```

Make the script executable:
```bash
chmod +x start-<system>.sh
```

> **Multiple configs or alternating sources:** If you need to run trunk-recorder against more than one config sequentially (e.g. alternating between SDR sources), replace `exec` with an `until` loop. `exec` is correct for the common single-config case.

---

## 2. LaunchAgent Plist

LaunchAgents live in `~/Library/LaunchAgents/`. Each Trunk Recorder instance gets its own plist. The `Label` value must be unique and match the filename (without `.plist`).

**File location:** `~/Library/LaunchAgents/local.trunk-recorder-<system>.plist`

See `example-launchagent.plist` in this directory for a fully annotated template.

Key fields:

| Key | Purpose |
|---|---|
| `Label` | Unique identifier — must match the filename |
| `ProgramArguments` | `/bin/bash` followed by the absolute path to the launcher script |
| `WorkingDirectory` | Absolute path to `trunk-build/` — recordings land here |
| `RunAtLoad` | Start the job as soon as the plist is loaded (at login) |
| `KeepAlive` | launchd automatically restarts the process when it exits |
| `StandardOutPath` | Log file path for stdout |
| `StandardErrorPath` | Set to the same path as `StandardOutPath` — trunk-recorder writes everything to stderr, so combining them keeps all output in one file |

> **Note:** `UserName` and `GroupName` are only valid in LaunchDaemons (system-level). LaunchAgents always run as the logged-in user — do not include those keys.

---

## 3. Automatic Login

LaunchAgents require an active user session, so the machine must be logged in for them to run. Enabling automatic login means Trunk Recorder starts immediately at boot with no interaction required.

**System Settings → Users & Groups → Automatic Login → select your user**

> **FileVault:** If FileVault full-disk encryption is enabled, macOS disables automatic login — the disk must be decrypted with a password before the OS can boot. Check status with:
> ```bash
> fdesetup status
> ```
> If FileVault is on, someone must enter the password once after each reboot before the recorders start. Disabling FileVault allows fully unattended boots.

---

## 4. Installing and Loading

Create the log directory, place the plist in `~/Library/LaunchAgents/`, then load it:

```bash
# Create the log directory (one time)
mkdir -p ~/Library/Logs/TrunkRecorder

# Load the job and start it immediately
launchctl load ~/Library/LaunchAgents/local.trunk-recorder-<system>.plist

# Stop a running job
launchctl unload ~/Library/LaunchAgents/local.trunk-recorder-<system>.plist

# Check status of all trunk-recorder jobs
launchctl list | grep trunk-recorder
```

Once loaded, the job starts automatically at every login — no need to re-run `launchctl load` after reboots.

---

## 5. Log Files

Each instance writes all output to a single log file:

```
~/Library/Logs/TrunkRecorder/<system>.log
```

Trunk Recorder (a GnuRadio/C++ application) writes all output — status messages, call activity, errors — to stderr. Both `StandardOutPath` and `StandardErrorPath` in the plist point to the same `.log` file so nothing is lost.

### Viewing Logs

**Launch Control (recommended)**
Open Launch Control → click the job → log output is shown inline. This is the easiest way to monitor job status and output together.

**Console.app**
Open Console.app → in the left sidebar under **Files**, navigate to `~/Library/Logs/TrunkRecorder/` and select a log file. Console.app live-tails it automatically.

> Note: the logs do *not* appear in Console.app's main unified log stream — only in the Files section.

**Terminal**
```bash
# Follow a single instance
tail -f ~/Library/Logs/TrunkRecorder/<system>.log

# Follow all instances at once
tail -f ~/Library/Logs/TrunkRecorder/*.log
```

---

## 6. Log Rotation

macOS includes `newsyslog`, a built-in log rotation daemon that runs automatically every hour. Adding a config file to `/etc/newsyslog.d/` tells it to rotate and compress Trunk Recorder logs before they grow too large.

**Create the config (requires sudo, add one line per instance):**
```bash
sudo tee /etc/newsyslog.d/trunk-recorder.conf > /dev/null << 'EOF'
# logfilename                                                [owner:group]   mode  count  size  when  flags
/Users/<username>/Library/Logs/TrunkRecorder/<system>.log   <username>:staff  640    5    10240   *    NJ
EOF
```

Column reference:

| Column | Value | Meaning |
|---|---|---|
| `mode` | `640` | Permissions on the fresh log file after rotation |
| `count` | `5` | Compressed old logs to keep before the oldest is deleted |
| `size` | `10240` | Rotate when the file reaches 10 MB |
| `when` | `*` | Size-based only — no scheduled time rotation |
| `N` flag | — | Don't signal the process after rotation |
| `J` flag | — | Compress archived logs with bzip2 |

With 5 copies at 10 MB each, disk usage per instance is capped at ~50 MB.

**Test that the config is valid:**
```bash
sudo newsyslog -v
```

> **File descriptor note:** launchd holds the log file open, so after rotation new output continues to the old file until the process is restarted. In practice this is a non-issue — when trunk-recorder exits and launchd restarts it via `KeepAlive`, it opens a fresh handle to the new log file.

---

## 7. Quick Reference

```bash
# Load (start) a job
launchctl load ~/Library/LaunchAgents/local.trunk-recorder-<system>.plist

# Unload (stop) a job
launchctl unload ~/Library/LaunchAgents/local.trunk-recorder-<system>.plist

# Check running jobs
launchctl list | grep trunk-recorder

# Follow all logs live
tail -f ~/Library/Logs/TrunkRecorder/*.log

# Test log rotation
sudo newsyslog -v

# Check FileVault status
fdesetup status
```
