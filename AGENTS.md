# AGENTS.md

Operational notes. Terse by design. Everything here is verified against the tree.

## Map

| Path | Role | Build |
| --- | --- | --- |
| `core/` | `libgs130` (C11+C++17), CLI, samples, tools, `.deb` | GNU Make |
| `python/` | `gs130_camera` wrapper (ctypes) | setuptools, `build-wheel.sh` |
| `ros2/src/gs130_camera/` | ROS 2 node `gs130_node` | ament_cmake / colcon |
| `core/include/gs130.h` | public C API — source of truth for every layer | — |
| `core/include/gs130_define.h` | `GS130_CONFIG*` presets, GNU C only | — |
| `VERSION` | version shared by core + python | — |

Dependency order: `core` → `python`, `ros2`. Install core first.

## Build

Core needs `/usr/hobot/lib/lib{vpf,hbmem,cam}.so`, OpenCV 4 headers, `libtbb.so.2`, GCC with C11/C++17. Board only, no host build.

```bash
cd core
make -j$(nproc)          # all: lib + samples + tools + deb (PLATFORM=RDKX5 default)
make lib                 # lib only -> out/<PLATFORM>/libgs130.{a,so,so.<major>,so.<ver>}
make samples tools deb   # individually
make fifo|i2c|rectify|tracker|eeprom|imu|pipeline   # partial object rebuilds
make <PLATFORM> <target> # platform name valid ONLY as the first goal
make clean
```

`make` prompts `Build? [Y/n]` once, tty only; non-tty skips it; `GS130_NO_CONFIRM=1` skips it. Output: `build/<PLATFORM>/`, `out/<PLATFORM>/`, deb `out/gs130-camera_<ver>+<PLATFORM>_<arch>.deb`.

```bash
dpkg -s gs130-camera                             # installed?
sudo dpkg -i core/out/gs130-camera_*.deb         # -> /usr/include/{gs130.h,gs130_define.h}
                                                 #    /usr/lib/<multiarch>/libgs130.{a,so*}
                                                 #    /usr/bin/gs130*
```

```bash
cd python && ./build-wheel.sh                    # -> python/dist/gs130_camera-<ver>-py3-none-any.whl
python3 -m pip install dist/gs130_camera-*.whl
./build-wheel.sh clean
```

Wheel excludes the native library. Runtime lookup: `$GS130_LIB` → `ldconfig` → `libgs130.so`. Package/library version mismatch: older library refused, newer warns.

```bash
cd ros2
source /opt/ros/humble/setup.bash                # or jazzy
colcon build --packages-select gs130_camera
source install/setup.bash
```

ROS link: `find_library(gs130)`; header: `core/include` via `../../../`, else `$GS130_ROOT/include`. Override: `--cmake-args -DGS130_LIBRARY= -DGS130_INCLUDE_DIR=`.

## Secondary development

| Change | Edit |
| --- | --- |
| New platform backend | add `core/src/devices/pipeline/<PLATFORM>/`; Makefile discovers it (`PLATFORMS := wildcard src/devices/pipeline/*/`); add `GS130_CONFIG_<PLATFORM>_<DEVICE>` to `core/include/gs130_define.h` |
| Pipeline behaviour | `core/src/devices/pipeline/pipeline.{hpp,cpp}`, `pipeline/<PLATFORM>/` |
| Calibration / rectification | `core/src/base/rectify/`, `core/src/base/tracker/` |
| I2C, FIFO primitives | `core/src/base/i2c/`, `core/src/base/fifo/` |
| EEPROM models | `core/src/devices/eeprom/` (+ `GS130_EEPROM_REGISTER_MODEL`) |
| IMU sensor | `core/src/devices/imu/` |
| Public C API | `core/include/gs130.h` + `core/src/gs130.cpp` |
| CLI subcommand | `core/samples/gs130` (bash dispatcher) + program in `core/samples/` or `core/src/tools/` |
| Python surface | `python/gs130_camera/{_device,_types,_config,_preset,_abi}.py` + `__init__.pyi` |
| ROS 2 topics/params | `ros2/src/gs130_camera/src/gs130_node.cpp`, `launch/*.launch.py` |

C skeleton: `GS130_CONFIG(device, mode, w, h, fps, odr)` → `gs130_create` → `gs130_init` → `gs130_start` → `gs130_get_nv12_frame` | `gs130_get_stereo_nv12_frame` | `gs130_get_imu_packet` → `gs130_stop` → `gs130_deinit` → `gs130_destroy`.
Frame buffers are `malloc`-owned by the caller: `free(image.data)`. Reference implementations: `core/samples/gs130-run.c`, `gs130-rec.c`.

Python skeleton: `preset()` | `config()` → `Device(cfg)` as context manager → `start()` → `read_image()` / `read_imu()` (both `None` when empty) → `close()`. Config is a nested dict; `preset()` fills it, edit before constructing `Device`.

## Test

```bash
python3 test/test_gs130.py GS130WI rect 544 448 30 200 [--output-dir D] [--overwrite]
    # run from python/; hardware test; needs cv2; prints calibration/queues, writes NV12->PNG
    # skips absent hardware instead of failing

core/out/<PLATFORM>/gs130-run GS130WI rect 544 448 30 200    # stream camera + IMU
core/out/<PLATFORM>/gs130 detect imu -b 4 6                  # I2C probe; verified GS130WI + RDK X5 buses, not a program default
core/out/<PLATFORM>/gs130 shell -d GS130WI                   # interactive: imu-info|eeprom-info|calib-export|run|rec
```

TROS launches (need `hobot_codec`, `websocket`, `hobot_stereonet`): `source /opt/tros/humble/setup.bash` then `ros2 launch gs130_camera gs130{,_websocket,_stereonet}.launch.py`; page at `http://<board>:8000`.

## Checks before claiming done

- comment/doc-only change ⇒ code token stream unchanged: strip `//`, `/* */`, string and char literals, collapse whitespace, diff against the previous revision.
- `CMakeLists.txt` ⇒ non-`#` lines identical.
- `launch/*.launch.py` ⇒ `ast.parse` both revisions, compare dumps with docstrings removed.
- `package.xml` ⇒ parsed element structure identical; only `<description>` may differ for doc work.
- README tables that mirror code ⇒ compare against `declare_parameter(...)` (ROS 2) or `python/gs130_camera/_config.py::_FIELDS`.
- Relative links ⇒ every path resolves from that document's own directory.

## Rules

- Comment/doc work: zero token changes in code. Verify, do not assume.
- Never document unsupported capability. Implemented backends = directories under `core/src/devices/pipeline/`; anything else makes `GS130_CONFIG_PLATFORM` print and `exit(1)`. Today: RDKX5 only; S100/S600 are "in development".
- Project is a developer preview (Alpha). Label it as such in user-facing documents. The
  stage shows up as a suffix on release tags (`0.1.0-Alpha`) and not inside `VERSION`, which
  holds three plain numbers; see the release rule below.
- Do not edit, commit, reformat or `chown` files you did not change — check `git status` first.
- `git fetch` and compare `origin/develop` before committing or pushing; the branch is shared and has diverged before.
- **Never commit or push directly to `develop` or `main`.** All work goes on a feature branch
  named `feature/<topic>`, and reaches `develop` only through a reviewed pull request:
  1. Branch off `develop` as `feature/<topic>`.
  2. Commit on that branch. One Jira maps to one branch and one pull request; several commits
     on the branch are fine, but keep them scoped to that Jira and tidy before requesting review.
  3. Push the feature branch and open a pull request **against `develop`**, describing the
     scope, how it was verified, the results, and anything not verified.
  4. **Do not merge the pull request.** Merging is the maintainer's action. This holds even
     when the author has write access, which would otherwise let a self-authored pull request
     be merged without anyone reviewing it.
  Contributors with write access branch inside this repository and do not fork; contributors
  without write access fork and open the pull request against `develop` upstream. The
  maintainer opens the pull request for agent work unless a credential for it has been provided.
- A release is announced by the maintainer as "release x.y.z". The version number is the
  maintainer's decision; do not propose one. Then, in order:
  1. **While the project is Alpha or Beta, ask the maintainer which pre-release suffix the
     version carries** (`-Alpha`, `-Beta`, or none) before touching anything. The suffix
     belongs to the tag, not to `VERSION`. Skip the question once the project is at a
     stable release.
  2. Edit exactly two files: `VERSION` (drives core, the Python wheel and the Debian package
     name) and `ros2/src/gs130_camera/package.xml`. The ROS 2 version is not derived from
     `VERSION`; update both files in the same commit. No other tracked file carries a
     release version, and nothing enforces this automatically.
  3. Rebuild and confirm the new version is what the artifacts actually report.
  4. Commit those two edits as one commit on a `feature/<topic>` branch, and get them into
     `develop` through a reviewed pull request like any other change. A release is decided by
     the maintainer, so this pull request is expected to be quick, but it is still a pull
     request: do not push the version bump straight to `develop`.
  5. Merge `develop` into `main` with `--no-ff`, so the release point stays identifiable on
     `main`. This repository's history is linear to date and this introduces the first merge
     commits; do not fast-forward a release merge.
  6. Tag that commit **on `main`**, using `x.y.z-Alpha` / `x.y.z-Beta` / `x.y.z`. Tags are
     annotated, matching the existing `0.1.0-Alpha`. Never move or delete an existing tag.
  7. Push the release commit, `main` and the tag.
  Record the commit, the tag and the post-push check in the Jira. Three of these steps change
  code or history elsewhere, so they need the maintainer's explicit go-ahead first: opening
  the version-bump pull request, the merge into `main`, and creating the tag.
- `python/setup.py` uses `python/README.md` as `long_description` when that file exists.
- Never commit build output. `.gitignore` covers: `core/{build,out}`, `python/{build,dist,*.egg-info}`, `__pycache__`, `ros2/{build,install,log}`.
- READMEs are bilingual and cross-linked: `README.md` (English) + `README.zh-CN.md` (Chinese). One change belongs in both.
