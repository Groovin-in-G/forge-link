# Forge Link - the monotonic build number baked into the HELLO ident's `build:<N>` token (L4).
# Copyright (c) 2026 the Forge Link contributors. GPLv3 or later (see LICENSE).
#
# Its own module, like fl_dirty_paths, so it can be tested without SCons (inject_build_info.py
# calls it). N = `git rev-list --count HEAD`: a fact of the commit, the same for every rebuild.
#
# ⛔ 0 = "git could not say", and the release gate refuses build:0 (tools/fl_release_gate.py). That
# covers a failed git call AND A SHALLOW CLONE: there rev-list counts only the commits fetched, so a
# `--depth 1` checkout would silently bake build:1 (or whatever depth it has), a number that looks
# real and orders nothing (L4 review, 2026-10-06).


def build_number(run) -> int:
    """`run(cmd_list) -> str` returns stripped stdout, or "" when the command failed."""
    if run(["git", "rev-parse", "--is-shallow-repository"]).strip() != "false":
        return 0                     # shallow ("true"), or git could not even answer
    count = run(["git", "rev-list", "--count", "HEAD"]).strip()
    return int(count) if count.isdigit() and int(count) > 0 else 0
