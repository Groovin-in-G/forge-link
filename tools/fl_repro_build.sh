#!/usr/bin/env bash
# fl_repro_build.sh - rebuild a Forge Link RELEASE from its archived source, byte-identical (SHIP3).
#
# The archive (made by tools/fl_source_archive.sh) has no .git, so the four values the original build
# read from git and the clock come from the archive's REPRO.json:
#   git_rev, build_number, build_timestamp  -> FL_REPRO_* (src/inject_build_info.py honours them)
#   cc_date, cc_time                        -> __DATE__ / __TIME__ via PLATFORMIO_BUILD_FLAGS
# Nothing else is overridden. The script ends by comparing the UF2's SHA-256 with REPRO.json's, and
# exits 0 only on a match.
#
#   tools/fl_repro_build.sh [<unpacked-archive-dir>]      (default: the current directory)
#
# Needs PlatformIO Core with network access for the pinned platform/framework the first time
# (versions: docs/gpl-notice.md, "Build tools").
set -euo pipefail

dir="${1:-.}"
cd "$dir"
[ -f REPRO.json ] || { echo "fl_repro_build: no REPRO.json in $(pwd)" >&2; exit 2; }

get() { python3 -c 'import json,sys; print(json.load(open("REPRO.json"))[sys.argv[1]])' "$1"; }
env_name=$(get env)
want=$(get uf2_sha256)

export FL_REPRO_GIT_REV="$(get git_rev)"
export FL_REPRO_BUILD_NUMBER="$(get build_number)"
export FL_REPRO_TIMESTAMP="$(get build_timestamp)"
# Each define is quoted whole: the date holds a space, and PlatformIO splits build flags shell-style.
export PLATFORMIO_BUILD_FLAGS="-Wno-builtin-macro-redefined '-D__DATE__=\"$(get cc_date)\"' '-D__TIME__=\"$(get cc_time)\"'"

echo "fl_repro_build: env $env_name, git $FL_REPRO_GIT_REV, build $FL_REPRO_BUILD_NUMBER, stamp $FL_REPRO_TIMESTAMP"
pio run -e "$env_name"

uf2=".pio/build/$env_name/firmware.uf2"
got=$(shasum -a 256 "$uf2" | cut -d' ' -f1)
echo "fl_repro_build: built $got"
echo "fl_repro_build: want  $want"
if [ "$got" = "$want" ]; then
    echo "fl_repro_build: MATCH - byte-identical to the released UF2"
    exit 0
fi
echo "fl_repro_build: MISMATCH" >&2
exit 3
