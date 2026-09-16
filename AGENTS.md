# AGENTS.md

Working notes for agents in this repository.

## What this is

The GS130 stereo camera and IMU SDK: one native library, three layers on top of it.

| Layer | Directory | Built with |
| --- | --- | --- |
| Core | `core/` | C11 and C++17, a GNU Makefile |
| Python wrapper | `python/` | Python 3.10+, `ctypes` over `libgs130` |
| ROS 2 wrapper | `ros2/` | C++17, `ament_cmake` |

`VERSION` at the root is shared by Core and the Python wrapper; the ROS 2 package declares its own version in `package.xml`.

## Ground rules

- **Keep the layers apart.** `core/` must not assume the shape of either wrapper, and the wrappers must not re-implement what `libgs130` already does. A behaviour belongs in Core unless it is genuinely language- or middleware-specific.
- **When the task is comments or documentation, behaviour must not move.** Strip the comments and whitespace and compare the remaining tokens against the previous revision; that check is what makes "comment-only" a claim rather than a hope.
- **Never claim support the code does not have.** `core/src/devices/pipeline/` holds the implemented backends, and `GS130_CONFIG_PLATFORM` prints to stderr and exits for anything else. Today that means RDK X5; RDK S100 and RDK S600 are described as in development.
- **This is a developer preview (alpha).** Say so in user-facing documents instead of presenting the interfaces as settled.
- **Leave other people's work alone.** Check `git status` first: uncommitted changes you did not make are not yours to edit, commit, reformat, or `chown`.
- **Fetch before you push.** `develop` is shared and has diverged before. Compare against `origin/develop` rather than assuming a fast-forward.

## Build and verify

Core links the Horizon multimedia libraries (`libvpf`, `libhbmem`, `libcam` in `/usr/hobot/lib`), OpenCV 4 and `libtbb.so.2`, so it builds on the board, not on a plain host.

```bash
cd core
make -j$(nproc)          # library, samples, tools and the .deb, for the default platform (RDKX5)
make lib                 # library only
make RDKX5 <target>      # a platform name is only honoured as the first goal
```

`make` asks once for confirmation before compiling; a stdin that is not a terminal skips the question.

```bash
cd python && ./build-wheel.sh                            # wheel into python/dist/
cd ros2 && colcon build --packages-select gs130_camera   # needs libgs130 installed
```

The wheel does not bundle the native library. `python/setup.py` uses `python/README.md` as the long description whenever that file exists, and the binding resolves `libgs130` through `GS130_LIB`, then `ldconfig`, then `libgs130.so`.

```bash
dpkg -s gs130-camera     # is the SDK installed? (the .deb is gs130-camera)
```

## Conventions

**Sources** open with a header naming the project URL, `Copyright (c) 2026 D-Robotics.`, `SPDX-License-Identifier: MIT` and a pointer to `LICENSE`. The API is documented in Doxygen (`@file`, `@brief`, `@param`, `@retval`, `@note`, `@warning`), and the comments carry the contracts that the signatures cannot: units, ownership, thread safety and coordinate frames.

**Documents** are bilingual and cross-linked at the top: `README.md` in English, `README.zh-CN.md` in Chinese. Neither has badges, both use one emoji heading per section, and the two are expected to stay in step — a change to one belongs in the other.

**Commit messages** are English and imperative, with a short body that says what changed and why.

## Checks worth running

Cheap and mechanical; these catch the mistakes this repository actually produces.

- **Comment-only work**: strip `//`, `/* */` and string literals with a small lexer, collapse whitespace, compare the token streams.
- **`CMakeLists.txt`**: compare the non-`#` lines.
- **`launch/*.launch.py`**: `ast.parse` both revisions and compare the dumps with docstrings removed.
- **`package.xml`**: compare the parsed element structure — only `<description>` should differ for a documentation change.
- **README tables that mirror code**: pull the defaults out of `declare_parameter(...)` (ROS 2) or `_FIELDS` (Python) and check them against the tables.
- **Relative links**: confirm every path a document points at exists from that document's directory.

## What lives where

```
core/include/gs130.h        the public C API; the reference the wrappers mirror
core/include/gs130_define.h platform presets (GS130_CONFIG*), GNU C only
core/samples/               the gs130 front end and the sample programs
core/src/tools/             the three detect tools
python/gs130_camera/        the wrapper, with __init__.pyi and py.typed
ros2/src/gs130_camera/      the ROS 2 node, launch files and package metadata
```

Build output is ignored by `.gitignore` (`core/build`, `core/out`, `python/build`, `python/dist`, `python/*.egg-info`, `__pycache__`, `ros2/build|install|log`) and must never be committed.
