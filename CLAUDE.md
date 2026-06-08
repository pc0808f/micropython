# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Official MicroPython — a Python 3.x implementation for microcontrollers. This checkout's active work is porting the **01studio pyDrone ESP32-S3 board** (from the `01studio-lab` fork at `C:\github\mp-old`) to current official MicroPython, building inside a Docker container with ESP-IDF 5.3.2.

## Current porting project (PYDRONE board)

Build environment — host is Windows 11; all ESP32 builds run in Docker:

```bash
docker run --rm -it -v "C:/github/mp-new:/micropython" -w /micropython/ports/esp32 espressif/idf:v5.3.2 bash
# inside the container:
rm -rf build-PYDRONE && make BOARD=PYDRONE -j$(nproc)
```

Key facts about this port:

- `ports/esp32/boards/PYDRONE/` was copied from the old fork, plus `boards/sdkconfig.usb` and `boards/sdkconfig.cam` (all currently untracked).
- `boards/PYDRONE/mpconfigboard.cmake` is the core of the port: it `include()`s `boards/mpconfigboard_esp32s3_common.cmake` to inherit the official S3 base config (which brings in `sdkconfig.base` with required settings like `CONFIG_MBEDTLS_PLATFORM_TIME_ALT=y`), then `list(APPEND SDKCONFIG_DEFAULTS ...)` to layer PYDRONE-specific sdkconfig files on top. Do not replace `SDKCONFIG_DEFAULTS` wholesale — always append.
- ULP is disabled for this board (`set(MICROPY_PY_ESP32_ULP 0)`); IDF 5.x moved ULP headers, so extra `-I/opt/esp/idf/components/ulp/...` include paths are appended in the board cmake. In IDF 5.x the include in `esp32_ulp.c` is `esp32s3/ulp.h`.
- Old-fork `sdkconfig.board` files may use renamed IDF config keys (e.g. `CONFIG_ESP32S3_ULP_COPROC_ENABLED` → `CONFIG_ULP_COPROC_ENABLED`). Renames usually produce warnings, not errors; fix them by comparing against `boards/sdkconfig.base`.
- Old-fork sdkconfig options can also reference hooks the fork's C code provided but official MicroPython removed: `CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP=y` caused `undefined reference to vPortCleanUpTCB` at link (the fork defined it in `mpthreadport.c`; official sets this option to `n` in `sdkconfig.base`). Fixed by removing the line from PYDRONE's `sdkconfig.board`.
- The build currently succeeds, but the app partition is nearly full (0x7a0 bytes free of 0x190000) **before** any of the fork's drone/cam C modules are added. Enlarging the app partition in `boards/PYDRONE/partitions-8MiB.csv` will be required when porting those modules.
- The fork's feature flags (`MICROPY_PORT_DRONE`, `MICROPY_PORT_PICLIB`, `MICROPY_PORT_CAMLIB`, `MICROPY_PORT_WEB_STREAM`) are set in the board cmake; corresponding C modules must exist/build for them to have effect.

## Build commands

mpy-cross (cross-compiler) must be built first for any port:

```bash
make -C mpy-cross
```

ESP32 port (CMake/idf.py-based, run inside the IDF container):

```bash
cd ports/esp32
make submodules               # init required git submodules (once per port)
make BOARD=PYDRONE            # or BOARD=ESP32_GENERIC_S3, etc.
make BOARD=<board> deploy     # flash over serial
make BOARD=<board> erase      # erase flash first on a fresh device
```

Output lands in `ports/esp32/build-<BOARD>/firmware.bin`. The Makefile wraps `idf.py`, which can also be used directly. A full clean is `rm -rf build-<BOARD>` — necessary after sdkconfig changes, since sdkconfig is cached in the build dir.

Unix port (fastest way to test core/extmod changes, no hardware needed):

```bash
cd ports/unix
make submodules
make -j
```

## Tests

```bash
cd tests
./run-tests.py                          # run against the unix port build
./run-tests.py basics/int_big1.py       # run a single test
./run-tests.py -t /dev/ttyUSB0          # run against an attached board
./run-tests.py -t a0 -d basics          # board on serial port, one directory
```

Tests work by comparing MicroPython output against CPython output (or a `.exp` file).

## Code style and commits

- C code: format with `tools/codeformat.py` (uses uncrustify v0.71/v0.72 only; newer versions produce wrong output — `pip install micropython-uncrustify`). Pass changed files as arguments to avoid reformatting everything.
- Python code: `ruff format` / ruff lint.
- Commit messages: first line is `path/prefix: Capitalized sentence with full stop.` within 72 chars (e.g. `esp32/boards: Add PYDRONE board definition.`), and commits must be signed off (`git commit -s`).
- `pre-commit install` enables format/codespell/commit-message hooks.

## Architecture

- `py/` — core compiler, VM, runtime, and built-in types. Port-independent.
- `extmod/` — extra C modules shared across ports (e.g. `machine_*`, `network_*`, `vfs`). Port-independent code that wraps port-provided hooks.
- `ports/<name>/` — platform code. Each port supplies `mpconfigport.h` (port-wide config), `mphalport.c/h` (HAL glue), and `main.c`. ESP32 runs MicroPython as a FreeRTOS task on top of ESP-IDF.
- `lib/` — third-party code as git submodules (vendor HALs, tinyusb, etc.); init via `make submodules` in the port directory.
- `mpy-cross/` — cross-compiler that turns `.py` into `.mpy` bytecode for frozen modules.
- `tools/` — `pyboard.py`, `codeformat.py`, `mpremote` lives on PyPI but `pyboard.py` is here.

Configuration is layered: `py/mpconfig.h` defaults ← `ports/<port>/mpconfigport.h` ← `boards/<BOARD>/mpconfigboard.h`. On esp32 there's a parallel cmake layer: `boards/<BOARD>/mpconfigboard.cmake` selects the IDF target and accumulates `SDKCONFIG_DEFAULTS` (a list of sdkconfig fragment files; later files override earlier ones). Shared fragments live in `ports/esp32/boards/` (`sdkconfig.base`, `sdkconfig.ble`, `sdkconfig.usb`, ...); board-specific ones in the board directory (`sdkconfig.board`).

A board definition directory contains: `mpconfigboard.h` (MicroPython feature flags, board name), `mpconfigboard.cmake` (IDF target + sdkconfig list + frozen manifest), `sdkconfig.board` (IDF Kconfig overrides), optional `partitions-*.csv` (flash partition table), and `board.json` (metadata for the downloads website — optional for local-only boards).

Build-time code generation: the build scans sources for `MP_QSTR_xxx` identifiers and generates interned string tables (`qstrdefs.generated.h`), plus compressed ROM data for module/method tables. This is why adding a new `MP_QSTR_` or a new module sometimes requires a clean build when the build system fails to pick it up.
