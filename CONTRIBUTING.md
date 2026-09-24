# Contributing

[简体中文](CONTRIBUTING.zh-CN.md) | **English**

How a change gets into the project: where to send it, how to name the branch,
what a commit message looks like, and what a reviewer will look at.

For what the code is, how to build it and how to run it, see the
[README](README.md).

## Ways to contribute

- **Bug report or feature request** — open an [issue](https://github.com/D-Robotics/gs130_camera_sdk/issues)
  describing what you observed or need. Include the board, the camera model, the
  commands you ran and their output.
- **Code or documentation change** — send a pull request. You do not need to open an
  issue first, and you do not need to ask for permission before starting.

## Where a change goes

Changes reach `develop`, never `main`. `main` carries released versions only.

1. Fork the repository. If you already have write access, branch inside the
   repository instead of forking.
2. Branch off `develop`, named `feature/<topic>` — for example
   `feature/eeprom-new-vendor` or `feature/fix-imu-timestamp`.
3. Commit on that branch. Several commits are fine while you work; tidy them before
   requesting review.
4. Push the branch and open a pull request **against `develop`**.

Do not push directly to `develop` or `main`, and do not merge your own pull request.

## Commit messages

Write for the person reading the history in a year, not for the reviewer.

- **Title**: one imperative line saying what the change does, about 72 characters,
  no trailing period. `Add ...`, `Fix ...`, `Clarify ...` — not `Added`, not `Fixes bug`.
- **Body**: what changed and why. Three to eight lines is usually right. Skip it only
  when the title already says everything.
- **State facts, do not argue.** Leave out defensive phrasing such as "no missing
  term is invented" or "rather than returning a partial result".
- **Keep details where they belong.** Offsets, field layouts, checksum formulas,
  command lines and expected values go in code comments and documentation, not in
  the commit message.
- **Do not restate the diff.** Which files changed and how many lines is what
  `git show --stat` is for.
- **Bilingual is welcome, not required.** If you write both, put English first, and
  give each language once — do not repeat the same paragraph in both.
- If you amend or rewrite a branch you have already pushed, use
  `git push --force-with-lease`, never a bare `--force`.

## What a reviewer looks for

- The change does one thing, and the pull request description says what that is.
- No unrelated edits — reformatting or renaming files the change does not need makes
  review harder and will be asked to be split out.
- No build output. `.gitignore` covers `core/{build,out}`,
  `python/{build,dist,*.egg-info}`, `__pycache__` and `ros2/{build,install,log}`.
- Claims match evidence. "Verified on hardware" means someone ran it on hardware and
  the pull request says how; a host-side test is described as a host-side test.
  Anything not verified is listed as not verified rather than implied to work.
- Documentation and comments that mirror code were updated together with the code.

## Checks to run before requesting review

Run whatever applies to your change. Build from a clean tree first, so the result is
not carried over from an earlier build.

```bash
cd core
make -j$(nproc)            # library, samples, tools and the Debian package
make lib                   # library only

# run what your change touches, for example
out/RDKX5/gs130 detect eeprom -b 4 -a 0x50
out/RDKX5/gs130 shell -d GS130WI
```

For the Python wrapper, the hardware test runs from `python/`:

```bash
python3 test/test_gs130.py GS130WI rect 544 448 30 200
```

It skips absent hardware instead of failing, so a pass without a board attached
proves less than it looks. Say which of the two you ran.

For the ROS 2 wrapper:

```bash
cd ros2 && colcon build --packages-select gs130_camera
```

## Rules any change has to follow

- Match the surrounding code. There is no formatter to run; follow the style already
  in the file you are editing.
- Update documentation and comments in the same change as the code they describe.
  Calibration field layouts, checksum formulas and similar details belong in code
  comments and documentation, not in commit messages or pull request descriptions.
- Do not commit build output.
- Do not rename, reformat or `chown` files your change does not need. Check
  `git status` before committing.
- A change that only alters comments, formatting or documentation must leave the
  code itself identical.
