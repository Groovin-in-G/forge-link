# ZuluSCSI™ - Copyright (c) 2022-2025 Rabbit Hole Computing™
#
# ZuluSCSI™ firmware is licensed under the GPL version 3 or any later version.
#
# https://www.gnu.org/licenses/gpl-3.0.html
# ----
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Bakes the exact git commit and a real build timestamp into the firmware,
# independent of the compiler's __DATE__/__TIME__ macros. Those macros only
# refresh when the specific translation unit that uses them actually gets
# recompiled -- if a build cache serves a stale object for that file, the
# reported build time silently stays frozen even though "clean"
# dependency-checking (touch, mtime) says it should be current.
#
# Writes a generated header (src/ZuluSCSI_build_info_generated.h) rather than
# appending to env["CPPDEFINES"] -- an earlier version did that, but this
# script is listed in extra_scripts without a "pre:" prefix, so it runs after
# PlatformIO has already scheduled the normal compile commands, and the
# appended defines never reached the actual compiler invocation (confirmed:
# the script's own diagnostic prints showed the right values, but `strings`
# on the resulting firmware.bin showed only the __DATE__/__TIME__ fallback).
# Writing the header sidesteps that timing question entirely: as long as the
# file is on disk with new content before the compiler reads it -- true for
# any extra_scripts, pre or post, since they all run during SCons's Python
# setup phase, strictly before any compiler subprocess is invoked -- SCons's
# ordinary #include dependency tracking takes care of the rest.

import datetime
import os
import sys
import subprocess

Import("env")

_project_dir = env.subst("$PROJECT_DIR")
_header_path = os.path.join(_project_dir, "src", "ZuluSCSI_build_info_generated.h")
print("inject_build_info.py: PROJECT_DIR = %r" % _project_dir)


def _run(cmd):
    try:
        out = subprocess.check_output(
            cmd, cwd=_project_dir, stderr=subprocess.STDOUT
        )
        return out.decode("utf-8", "replace").strip()
    except Exception as e:
        print("inject_build_info.py: %r failed: %r" % (cmd, e))
        return ""


# SHIP3 (2026-10-13): a RELEASE REBUILD from the archived source tarball has no .git, so the values the
# original build read from git and the clock are handed in instead (tools/fl_repro_build.sh reads them
# from the archive's REPRO.json). FL_REPRO_GIT_REV unset = the normal build, exactly as before.
_repro_rev = os.environ.get("FL_REPRO_GIT_REV", "")
if _repro_rev:
    git_rev, git_status = _repro_rev, ""
    print("inject_build_info.py: REPRO build, git rev %s from FL_REPRO_GIT_REV" % git_rev)
else:
    git_rev = _run(["git", "rev-parse", "--short", "HEAD"])
    git_status = _run(["git", "status", "--porcelain"])
# ⭐ L4 (2026-10-06): a MONOTONIC build number for the HELLO ident's `build:<N>` token. A hash
# cannot say "or later"; the commit count on HEAD's ancestry can. It is a fact of the commit, not of
# the moment: the same HEAD gives the same N however often it is rebuilt, and a dirty tree keeps
# HEAD's N (the "-dirty" suffix on git: is what says the bytes differ). 0 = git could not say, or
# the clone is SHALLOW (its count is the fetch depth, not the history) - see src/fl_build_number.py;
# the release gate refuses build:0.
sys.path.insert(0, os.path.join(_project_dir, "src"))
from fl_build_number import build_number as _build_number      # noqa: E402
build_number = int(os.environ["FL_REPRO_BUILD_NUMBER"]) if _repro_rev else _build_number(_run)
git_ident = (git_rev or "unknown") + ("-dirty" if git_rev and git_status else "")

# ⛔ "-dirty" ON ITS OWN IS A FACT NOBODY CAN ACT ON. The METHOD RULE says verify the build that is
# actually ON THE BOARD by its banner hash — but a banner reading `9d8b1b7-dirty` does not say
# whether the difference was a doc, a capture log or the firmware itself, so the check it exists for
# cannot be made. Measured on 2026-09-22: a bench UF2 was built from a tree carrying canon edits and
# untracked capture logs, none of which changed a byte of firmware, and the only honest thing anyone
# could say about it was "something was different".
#
# So the header now records WHICH paths were dirty, capped, with the count stated rather than the
# list silently truncated. The firmware's banner prints it (slice A) and the bench-UF2 filing
# refuses a dirty build outright (slice B); this is the half that produces the fact.
# ⛔ NEVER DELETE THIS GENERATED HEADER TO FORCE A FRESH STAMP — REWRITE IT (which is what this
# script does on every build). MEASURED 2026-09-22 by a council seat that read PlatformIO's own
# source: the project checksum is a SHA1 of the core version, the config, and the sorted PATHS of
# the source files — contents and mtimes are NOT hashed. So deleting this file drops a path, the
# checksum disagrees, and `clean_build_dir` wipes EVERY environment's ELF, not just this one's.
# That is exactly what produced the N6 step-2 build's red `check-irq-ram` (rc 4, "no ELF at
# .pio/build/ZuluSCSI_Blaster/firmware.elf") on a tree whose own build had just reported rc 0.
# Rewriting the same path costs a recompile of the three translation units that include it —
# ZuluSCSI_log.cpp, ForgeLink_probe.cpp, ForgeLink_inject.cpp — and a relink of that env alone.
#
# The parsing lives in its own module so it can be tested without dragging SCons in; the bug it
# had (a fixed-width slice eating one character of the FIRST path) is pinned there by a test.
sys.path.insert(0, os.path.join(_project_dir, "src"))
from fl_dirty_paths import dirty_paths_from_porcelain          # noqa: E402

_dirty_paths, _dirty_list = dirty_paths_from_porcelain(git_status)

# No spaces: a raw space in a #define value is an easy way to end up with an
# unterminated string literal if any step along the way re-tokenizes it.
build_timestamp = os.environ["FL_REPRO_TIMESTAMP"] if _repro_rev else datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

print("inject_build_info.py: git_ident=%s build=%d build_timestamp=%s -> %s" % (git_ident, build_number, build_timestamp, _header_path))

with open(_header_path, "w") as f:
    f.write("// Auto-generated by inject_build_info.py on every build. Do not edit.\n")
    f.write("#pragma once\n")
    f.write('#define ZULU_BUILD_GIT_REV "%s"\n' % git_ident)
    f.write('#define ZULU_BUILD_TIMESTAMP "%s"\n' % build_timestamp)
    f.write('#define ZULU_BUILD_NUMBER "%d"\n' % build_number)
    f.write('#define ZULU_BUILD_DIRTY_COUNT %d\n' % len(_dirty_paths))
    f.write('#define ZULU_BUILD_DIRTY_FILES "%s"\n' % _dirty_list)
