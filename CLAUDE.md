# CLAUDE.md

Notes for Claude Code when working in this repository. Keep changes minimal and idiomatic to what is already here — this is a long-lived C++ project with a lot of historical conventions baked into the structure.

## What this project is

Trunk Recorder is a C++ application that uses GNU Radio + OP25 to record voice calls from trunked (P25, SmartNet) and conventional (P25, DMR, analog) radio systems via SDRs. It is a single binary (`trunk-recorder`) plus a set of loadable plugin shared libraries.

The high-level pipeline is in [trunk-recorder/main.cc](trunk-recorder/main.cc):

```
load_config -> start_plugins -> setup_systems -> monitor_messages
```

The `tb` (`gr::top_block_sptr`) is the GNU Radio flow graph that runs while `monitor_messages()` polls queues, manages calls, and drives plugins.

## Build and run

There is no in-tree test suite or linter target. The standard build is:

```bash
mkdir -p build && cd build
cmake ../
make -j$(nproc)
```

For debugging crashes, use `cmake -DCMAKE_BUILD_TYPE=Debug ../` — see [docs/DEBUG.md](docs/DEBUG.md).

The binary expects a config file at `./config.json` by default; override with `-c`:

```bash
./build/trunk-recorder -c /path/to/config.json
```

Plugins are built as `MODULE` shared libraries from [plugins/](plugins/) and end up in `build/` (e.g. `librdioscanner_uploader.so`). They are loaded by name via the `plugins` array in `config.json` — see [docs/Plugins.md](docs/Plugins.md).

CI ([.github/workflows/main.yml](.github/workflows/main.yml)) only does a Docker multi-arch buildx. There is no GitHub-side test runner. Treat a clean local `make` as the bar.

## Source layout

| Path | What lives there |
|---|---|
| [trunk-recorder/](trunk-recorder/) | Core app (entry point, config, call/recorder/system orchestration) |
| [trunk-recorder/systems/](trunk-recorder/systems/) | `System` interface + P25/SmartNet trunking and parsers |
| [trunk-recorder/recorders/](trunk-recorder/recorders/) | Per-recorder GNU Radio hier blocks (P25, DMR, analog, SigMF, debug) |
| [trunk-recorder/gr_blocks/](trunk-recorder/gr_blocks/) | Custom GNU Radio blocks (transmission sink, channelizer, squelch, signal detector, decoders) |
| [trunk-recorder/sources/](trunk-recorder/sources/) | Non-osmosdr sources (currently IQ file source) |
| [trunk-recorder/plugin_manager/](trunk-recorder/plugin_manager/) | `Plugin_Api` interface + dynamic plugin loader |
| [trunk-recorder/call_concluder/](trunk-recorder/call_concluder/) | Post-call file finalization, ffmpeg, upload queueing |
| [plugins/](plugins/) | Built-in plugins (rdioscanner, openmhz, broadcastify, simplestream, stat_socket, unit_script) |
| [user_plugins/](user_plugins/) | Drop-in directory for out-of-tree plugins |
| [lib/](lib/) | Vendored deps (OP25 repeater, nlohmann json, lfsr) — **do not modify; upstream lives elsewhere** |
| [docs/](docs/) | User-facing docs published to trunkrecorder.com (frontmatter with `sidebar_label`) |
| [docs/Notes/](docs/Notes/) | Developer notes — call handling, states, plugin internals, status JSON |
| [examples/](examples/) | Sample `config-*.json`, talkgroup CSVs, systemd unit, helper scripts |
| [tests/mqtt/](tests/mqtt/) | The only thing in `tests/` — a small MQTT test plugin, not a unit-test framework |

`ARCHITECTURE_REVIEW.md` (root) is a candid review of the core's accumulated complexity and is worth reading before any non-trivial refactor — it identifies the string-based system dispatch, the `Source` recorder-vector zoo, the god `System` interface, and the `monitor_messages()` mega-loop as the main pain points.

## Code style

- **C++17**, `-pthread -Wno-narrowing -fvisibility=hidden -fPIC`. Set in [CMakeLists.txt](CMakeLists.txt) around line 225.
- **clang-format** is configured with only two overrides ([.clang-format](.clang-format)): `ColumnLimit: 0` and `BreakStringLiterals: false`. Everything else is LLVM defaults. Don't reflow lines just to satisfy a column limit — there isn't one.
- **No `-Wall -Werror`** in release builds. Debug adds `-Wall -Wno-deprecated-declarations -g3`. Don't gratuitously add warnings flags.
- **Fast-math is forcibly stripped** ([CMakeLists.txt#L232-L250](CMakeLists.txt#L232-L250)) because OP25's LFSR/Eigen code requires IEEE NaN/Inf semantics. Do not add `-ffast-math`, `-Ofast`, or `-ffinite-math-only` anywhere.
- **GNU Radio versions 3.7 - 3.10 are all supported.** The repo uses `#if GNURADIO_VERSION < 0x030800` / `< 0x030900` blocks (see [recorder.h](trunk-recorder/recorders/recorder.h)) to pick the right include and block type. Whenever you touch GNU Radio includes or block constructors, check whether the existing file already branches on `GNURADIO_VERSION` and follow the same pattern.
- **Logging is Boost.Log.** Use `BOOST_LOG_TRIVIAL(info|warning|error|debug) << ...`. Don't introduce `std::cout` / `printf` for log output.
- **JSON is nlohmann/json** (vendored in `lib/`, included as `<json.hpp>`). The plugin API hands `json` objects to `parse_config()`.
- **Naming**: snake_case for functions, variables, file names. PascalCase for class names. The pattern `Foo` (interface header) + `Foo_impl` (implementation header/cc) is used heavily — see `System`/`System_impl`, `Call`/`Call_impl`, `*_recorder`/`*_recorder_impl`. Preserve this when adding new types.
- **Headers** use `#ifndef NAME_H` / `#define NAME_H` guards, not `#pragma once`.
- **Boost is pervasive** (`boost::log`, `boost::program_options`, `boost::filesystem`, `boost::algorithm::string`, `boost::property_tree`). Prefer the Boost variant already used in the file over adding a new dependency.

## When writing or modifying a plugin

- Inherit from `Plugin_Api` in [plugin_manager/plugin_api.h](trunk-recorder/plugin_manager/plugin_api.h) and override only the hooks you need (each one has a no-op default returning 0).
- Export a `<name>_plugin_new` factory function — the loader finds it by symbol name based on the `name` field in the plugin config block ([plugin_manager.cc#L21-L34](trunk-recorder/plugin_manager/plugin_manager.cc)).
- Add a new directory under [plugins/](plugins/) with a `CMakeLists.txt` that builds a `MODULE` library and links `trunk_recorder_library`. Use [plugins/simplestream/CMakeLists.txt](plugins/simplestream/CMakeLists.txt) as a template.
- Add the subdirectory to the top-level [CMakeLists.txt](CMakeLists.txt) so it gets built. Plugins that need optional deps should be gated behind a CMake option.
- Return `0` on success, `-1` on failure from every hook.

## Things to NOT do

- Don't add a unit-test framework, lint step, or pre-commit hook unless asked — there isn't one today, and the contribution flow is built around manual + Docker testing.
- Don't bump or modify anything under [lib/op25_repeater/](lib/op25_repeater/) without flagging it — that's a vendored upstream.
- Don't change config file schema or CLI flags without updating [docs/CONFIGURE.md](docs/CONFIGURE.md) and an example in [examples/](examples/).
- Don't add `-ffast-math` / `-Ofast` (see code-style section).
- Don't switch include guards to `#pragma once` or reformat existing files purely for style.

## Documentation conventions

User-facing docs in [docs/](docs/) are published via the trunkrecorder.com Docusaurus site. They use YAML frontmatter:

```yaml
---
sidebar_label: 'Foo'
sidebar_position: 4
---
```

`docs/Notes/` is dev-internal — Mermaid diagrams are common there ([docs/Notes/CALL-HANDLING.md](docs/Notes/CALL-HANDLING.md)). [CHANGELOG.md](CHANGELOG.md) is updated per release with a bullet per merged PR in the form `* <description> by @<author> in #<PR>`.
