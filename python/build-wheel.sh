#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

PYTHON="${PYTHON:-python3}"

clean() {
    rm -rf build dist gs130_camera.egg-info
}

case "${1:-wheel}" in
    clean)
        clean
        printf 'Cleaned Python build artifacts.\n'
        exit 0
        ;;
    wheel)
        clean
        mkdir -p dist
        "$PYTHON" -m pip wheel \
            --no-deps \
            --no-build-isolation \
            . \
            --wheel-dir dist
        printf '\nBuilt wheel:\n'
        ls -lh dist/gs130_camera-*.whl
        ;;
    *)
        printf 'Usage: %s [clean|wheel]\n' "$0" >&2
        exit 2
        ;;
esac
