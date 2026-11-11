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
# Needs PlatformIO Core with network access for the pinned platform/framework/toolchain the first time (they
# install into ~/.platformio, not the archive). The libraries need none: they are in the archive (libdeps/).
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
# C19c (2026-10-30): before anything is seeded, check every file of libdeps/ against REPRO.json "lib_manifest"
# (written by tools/fl_source_archive.py: per library its version, resolved commit and path -> sha256). A changed,
# missing or extra file is refused here, naming the library and the file, instead of showing only as a firmware
# SHA MISMATCH after a full build. An archive made before C19c has no manifest: one line says the check was
# skipped and the rebuild goes on as before. Stdlib only, offline; -I so nothing in the archive is imported.
# C19c verify (review, 2026-10-30): only an ABSENT key skips; a null, empty or malformed manifest
# refuses by name. This names a wrong copy; it does not prove the archive genuine (the manifest travels in the
# same archive): the published archive sha256 is the trust anchor, checked before this script is run.
python3 -I - <<'PY'
import hashlib, json, os, sys
from pathlib import Path
repro = json.load(open("REPRO.json"))
if "lib_manifest" not in repro:
    print("fl_repro_build: REPRO.json has no lib_manifest (an archive made before C19c); per-file library check skipped")
    sys.exit(0)
man = repro["lib_manifest"]
if (not isinstance(man, dict) or not man
        or not all(isinstance(e, dict) and isinstance(e.get("files"), dict) for e in man.values())):
    sys.exit("fl_repro_build: REFUSED, REPRO.json lib_manifest is present but null, empty or malformed")
root = Path("libdeps")
bad = []
have_libs = sorted(p.name for p in root.iterdir()) if root.is_dir() else []
for name in have_libs:
    if name not in man:
        bad.append(f"libdeps/{name}: library not in the manifest")
for name, ent in sorted(man.items()):
    d = root / name
    if not d.is_dir():
        bad.append(f"libdeps/{name}: library missing from the archive")
        continue
    got = {}
    for base, dirs, files in os.walk(d):
        for n in list(dirs):
            if (Path(base) / n).is_symlink():
                files.append(n)
        for n in files:
            p = Path(base) / n
            got[p.relative_to(d).as_posix()] = ("symlink:" + os.readlink(p) if p.is_symlink()
                                                else hashlib.sha256(p.read_bytes()).hexdigest())
    want = ent["files"]
    for rel in sorted(set(want) | set(got)):
        if rel not in got:
            bad.append(f"libdeps/{name}/{rel}: missing (the manifest lists it)")
        elif rel not in want:
            bad.append(f"libdeps/{name}/{rel}: extra file, not in the manifest")
        elif got[rel] != want[rel]:
            bad.append(f"libdeps/{name}/{rel}: sha256 {got[rel]} but the manifest says {want[rel]}")
if bad:
    sys.exit("fl_repro_build: REFUSED, the archive's libraries are not the ones it was made with:\n  " + "\n  ".join(bad))
nf = sum(len(e["files"]) for e in man.values())
print(f"fl_repro_build: checked {nf} files in {len(man)} libraries against lib_manifest, all match")
PY
# C19: the archive carries the libraries the release built with (libdeps/, versions in REPRO.json
# "libs"). Seed them where PlatformIO looks, so a newer upstream release inside a caret range (measured
# 2026-10-27: Adafruit GFX ^1.12.1 now resolves to 1.13.0 and ship1 rebuilt to a different SHA) cannot change
# the build. An archive without libdeps/ fetches as before.
if [ -d libdeps ]; then
    mkdir -p ".pio/libdeps/$env_name"
    cp -R libdeps/. ".pio/libdeps/$env_name/"
    echo "fl_repro_build: seeded $(ls libdeps | wc -l | tr -d ' ') libraries from the archive's libdeps/"
    # C19b (2026-10-29): seeding alone did not stop the network. PlatformIO keeps a git library's record of
    # where it came from (.piopm) INSIDE its .git folder, and the archive leaves .git out, so the five git
    # libraries arrived with no record, PlatformIO matched none of them to its `name=https://...` lib_dep and
    # cloned all five from GitHub again (measured offline: exit 1 at the SdFat clone). Write that record back,
    # at the library's root (where PlatformIO also looks), from PlatformIO's own reading of this env's lib_deps
    # and the version REPRO.json lists. The library files and their path are untouched, so the build is too.
    pio project config --json-output > .pio/fl_repro_config.json
    python3 - "$env_name" ".pio/libdeps/$env_name" <<'PY'
import json, sys
from pathlib import Path
from urllib.parse import urlparse
env, root = sys.argv[1], Path(sys.argv[2])
libs = json.load(open("REPRO.json")).get("libs", {})
deps = [v for s, opts in json.load(open(".pio/fl_repro_config.json")) if s == "env:" + env
        for k, v in opts if k == "lib_deps"]
deps = deps[0] if deps else []
n = 0
# every git lib_dep of this env must end up with a record, or PlatformIO clones it again with no error
# (review 2026-10-29: a dep with no "name=", or a folder named differently, was skipped silently)
git_deps = [d for d in deps if "github.com" in d or d.split("#")[0].endswith(".git") or "git+" in d]
for dep in git_deps:
    if "=" not in dep:
        sys.exit(f"fl_repro_build: git lib_dep {dep!r} has no name=, so its archived copy cannot be matched")
for dep in deps:
    if "=" not in dep:
        continue
    name, url = (x.strip() for x in dep.split("=", 1))
    d = root / name
    if not d.is_dir() or (d / ".piopm").exists() or (d / ".git").exists():
        continue
    if name not in libs:
        sys.exit(f"fl_repro_build: libdeps/{name} is in the archive but REPRO.json lists no version for it")
    # the spec URI exactly as PlatformIO 6.2 writes it (package/meta.py PackageSpec._parse_uri): a GitHub or
    # *.git URL without a vcs+ scheme gets "git+" in front
    p = urlparse(url)
    if not ("+" in p.scheme or p.scheme in ("file", "symlink")) and (
            p.path.endswith(".git") or p.netloc in ("github.com", "gitlab.com", "bitbucket.com")):
        url = "git+" + url
    (d / ".piopm").write_text(json.dumps({"type": "library", "name": name, "version": libs[name],
        "spec": {"owner": None, "id": None, "name": name, "requirements": None, "uri": url}}))
    n += 1
missing = [d.split("=", 1)[0].strip() for d in git_deps
           if not ((root / d.split("=", 1)[0].strip() / ".piopm").exists()
                   or (root / d.split("=", 1)[0].strip() / ".git").exists())]
if missing:
    sys.exit(f"fl_repro_build: no archived copy with a source record for {missing}; PlatformIO would fetch them")
print(f"fl_repro_build: recorded the source of {n} git libraries, so PlatformIO uses the archive's copies")
PY
fi
# tools/tests/test_fl_repro_offline.py stops here and asks PlatformIO itself whether it would fetch anything
[ "${FL_REPRO_SEED_ONLY:-}" = 1 ] && { echo "fl_repro_build: FL_REPRO_SEED_ONLY, stopping before the build"; exit 0; }
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
