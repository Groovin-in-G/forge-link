"""The dirty-file list that goes into the build banner — a pure function, so it can be tested.

WHY IT IS ITS OWN MODULE. `inject_build_info.py` runs under PlatformIO with a live SCons `env`, so
importing it from a test drags the whole build in. The parsing is the part that had a bug, so the
parsing is the part that has to be testable on its own.

WHAT IT IS FOR. The METHOD RULE of this lane says: verify the build actually on the board by its
banner hash. A banner reading `4b6c898-dirty` cannot discharge that — it does not say whether the
difference was a document, a capture log or the firmware itself. The list does.
"""


def dirty_paths_from_porcelain(status: str, cap: int = 6) -> tuple[list[str], str]:
    """`git status --porcelain` text -> (every dirty path, a capped one-line summary).

    ⛔ NEVER SLICE A FIXED-WIDTH PREFIX OFF A PORCELAIN LINE. The caller strips the whole output,
    which removes the leading space of the FIRST line only, so a `line[3:]` slice silently eats a
    character of that one path and leaves every other path correct — measured 2026-09-22, the
    generated header read `ocs/PHASES.md`. A defect that damages exactly one item in a list is the
    kind that survives review, because the list looks right.
    """
    paths: list[str] = []
    for line in (status or "").splitlines():
        parts = line.strip().split(None, 1)
        if not parts:
            continue
        path = (parts[1] if len(parts) > 1 else parts[0]).strip()
        if " -> " in path:          # a rename: the TARGET is the file that now exists
            path = path.split(" -> ", 1)[1].strip()
        path = path.strip('"')      # git quotes paths containing spaces or unicode
        if path:
            paths.append(path)

    shown = paths[:cap]
    summary = " ".join(shown)
    if len(paths) > cap:
        summary += " +%d-more" % (len(paths) - cap)
    # A quote or a backslash would break the #define this ends up inside, and a filename is not
    # trusted input: it is whatever happens to be on disk.
    summary = summary.replace("\\", "/").replace('"', "'")
    return paths, summary
