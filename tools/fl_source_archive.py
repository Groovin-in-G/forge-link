#!/usr/bin/env python3
"""fl_source_archive - the complete corresponding source of one Forge Link RELEASE, as one archive (SHIP3).

    tools/fl_source_archive.py docs/releases/<release>.repro.json [--out DIR]

FL-109 (2026-11-09, "Only what builds it, scrubbed"): the archive carries ONLY what builds the firmware,
an ALLOWLIST cut from the release's source revision with `git archive`:
  platformio.ini, src/, lib/, boards/   the PlatformIO project of the release's env (BUILD_PATHS)
  LICENSE, README-ZuluSCSI-upstream.md  upstream's own licence and README, when the tree has them
Every file platformio.ini names for the env (extra_scripts, linker_script_template, ldscript_bootloader,
board_build.ldscript, the board JSON) is checked to sit inside that allowlist, so a new build input
outside it REFUSES (exit 5) instead of shipping an archive that cannot rebuild. Our notes (docs/ bar the
notice), scripts/, tools/ bar the overlay, tests and bench recordings are left out: none of them builds the
UF2, which the byte-for-byte rebuild proves. Then a small overlay that a git-less rebuild needs:
  src/inject_build_info.py      honours the FL_REPRO_* values (unset = the normal build)
  tools/fl_repro_build.sh       the rebuild recipe; exits 0 only when the UF2 SHA matches
  tools/fl_source_archive.py    this file
  COPYING, docs/gpl-notice.md   the licence and the notice that ship beside the firmware
  REPRO.json                    the release's own values (the .repro.json given here)
None of the overlay is compiled into the firmware: the rebuild is checked byte for byte, so an overlay
that changed the output would show as a MISMATCH.

SOURCE REV (FL-109): the release's "git_rev" is the firmware's identity (its HELLO says git:<rev>) and
never changes. An optional "source_rev" names the commit the archive is cut from instead: a child of
git_rev whose tree differs from it only in comments (names and pointers to internal notes scrubbed). The
tool refuses (exit 5) a source_rev that does not descend from git_rev. The notice's {{source_line}} and
REPRO.json's "source_note" say so in plain words; the rebuild still has to match the UF2 byte for byte.

THE SCAN (FL-109): before the tar is written, every archived file (libdeps/ included) is searched,
case-insensitively, for SCAN_TERMS: personal names, home paths, the competitor's names and the names of
our tools. Any hit REFUSES (exit 6) and is listed by file and line. ALLOW holds the only exceptions, each
a third-party library's own text, matched by path and line, never edited here.

C18: the overlay is read from the release's `overlay_rev` (a commit, in its .repro.json) with `git show`,
so archiving a release twice, a month apart, gives the same bytes. Without one it falls back to the
current tree and says so on stderr, and REPRO.json records "overlay_rev": "worktree". docs/gpl-notice.md
is a TEMPLATE: {{env}} {{label}} {{git_rev}} {{build_number}} {{uf2_sha256}} {{source_line}} are filled
from this release (label = the .repro.json name before the git rev), so a release's notice names that
release, not ship1. --notice-out writes the filled notice to ship beside it.

C19: the libraries PlatformIO fetched are complete corresponding source too. The release's .repro.json
lists them under "libs" ({name: version as pio printed it, e.g. "2.2.3+sha.ea911f7"}); each is copied
from .pio/libdeps/<env>/<name> (less .git) into libdeps/<name>/, and the archive REFUSES (exit 4) if an
installed library's version differs from the list or one is missing. Without a "libs" list it says so
on stderr and archives none (the old releases).

C19c: REPRO.json also carries "lib_manifest", one entry per archived library: its version, its resolved
commit where one is known (the git HEAD of the installed copy, read BEFORE .git is dropped, else the hex
"+sha." part of its version, else null: never invented), and every archived file's relative path ->
sha256 (a symlink records "symlink:<target>", never followed). tools/fl_repro_build.sh checks every file
of libdeps/ against it before seeding, so a wrong or damaged library copy is refused by library and file
instead of surfacing only as a firmware SHA MISMATCH at the end of a full build. It NAMES a wrong copy;
it does not prove the archive genuine, since the manifest travels in the same archive: the published
archive sha256 is the trust anchor. Only an absent key skips; null/empty/malformed refuses. It lives in
REPRO.json, not libdeps/MANIFEST.json, so the seeding's `cp -R libdeps/.` puts nothing but libraries where
PlatformIO looks; REPRO.json is written with sort_keys, so archiving twice stays byte-identical.

The tar is deterministic (sorted names, fixed mtime, uid/gid 0, gzip mtime 0), so making it twice gives
the same SHA-256, which the release notice names. Output: <out>/forgelink-source-<git_rev>.tar.gz + .sha256
(named by git_rev, the firmware's identity, whatever source_rev it was cut from).
"""
import argparse, codecs, configparser, gzip, hashlib, io, json, os, re, shutil, subprocess, sys, tarfile, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OVERLAY = ["src/inject_build_info.py", "tools/fl_repro_build.sh", "tools/fl_source_archive.py",
           "COPYING", "docs/gpl-notice.md"]
FIXED_MTIME = 1760313600          # 2025-10-13T00:00:00Z: any constant would do; it must never change
NOTICE = "docs/gpl-notice.md"
# FL-109: the allowlist. What the PlatformIO build of a release env reads from the project (read off
# platformio.ini, and checked against it on every run by check_build_inputs), plus upstream's own files.
BUILD_PATHS = ["platformio.ini", "src", "lib", "boards"]
UPSTREAM_FILES = ["LICENSE", "README-ZuluSCSI-upstream.md"]
# platformio.ini options whose values are project files the build reads
PIO_FILE_OPTS = ("extra_scripts", "linker_script_template", "ldscript_bootloader", "board_build.ldscript")

# FL-109: the scan. Stored rot13 so this file, which ships in every archive, does not trip its own scan.
SCAN_TERMS = [codecs.decode(w, "rot13") for w in
              ("trbetr", "tqnfujbbq", "jrfgjlpbzor", "/Hfref/", "f-pbaireg", "moevqtr", "pynhqr", "snoyr", "tebx")]
SCAN_RE = re.compile("|".join(re.escape(w) for w in SCAN_TERMS), re.I)
# The only exceptions: third-party library text, matched by archive path (regex) AND a substring of the
# hit line (rot13, as above). Each is the library's own words, shipped unedited; nothing of ours.
ALLOW = [
    (r"libdeps/SdFat/doc/Doxyfile", "P:/Hfref/ovyy/Qbphzragf/"),          # SdFat author's Windows path
    (r"libdeps/RP2350_Logic_Sniffer/README\.md", "fvtebx.bet/jvxv/ChyfrIvrj"),  # the PulseView project
    (r"libdeps/Adafruit BusIO/.*", "Jevggra jvgu uryc ol Pynhqr!"),     # Adafruit's own credit line
    (r"libdeps/Adafruit BusIO/.*", "uggcf://pynhqr.nv/pung/335s50o1-3qq8-435r-9139-57rp7pn26n3p"),
]
ALLOW = [(re.compile(pat), codecs.decode(sub, "rot13")) for pat, sub in ALLOW]


def release_label(repro_path: Path, rev: str) -> str:
    """ship3170-62087f65.repro.json -> ship3170 (the name before the git rev)"""
    stem = repro_path.name[:-len(".repro.json")] if repro_path.name.endswith(".repro.json") else repro_path.stem
    return stem[:-len(rev) - 1] if stem.endswith("-" + rev) else stem


def source_note(repro: dict) -> str:
    """FL-109: one plain sentence naming the tree the archive was cut from"""
    rev, src = repro["git_rev"], repro.get("source_rev")
    if src:
        return (f"source tree {src}, comments scrubbed from {rev}; builds byte for byte the firmware "
                f"identified as {rev} build {repro['build_number']}")
    return f"source tree {rev}, the revision the firmware reports (build {repro['build_number']})"


def fill_notice(text: str, repro: dict, label: str) -> str:
    vals = {"env": repro["env"], "label": label, "git_rev": repro["git_rev"],
            "build_number": str(repro["build_number"]), "uf2_sha256": repro["uf2_sha256"],
            "source_line": source_note(repro)}
    out = re.sub(r"\{\{(\w+)\}\}", lambda m: vals[m.group(1)], text)
    if "{{" in out:
        raise SystemExit(f"fl_source_archive: unfilled placeholder in {NOTICE}")
    return out


def overlay_bytes(rel: str, overlay_rev):
    """(bytes, mode) of one overlay file, from overlay_rev when given, else the current tree"""
    if overlay_rev is None:
        return (ROOT / rel).read_bytes(), (ROOT / rel).stat().st_mode & 0o777
    data = subprocess.run(["git", "-C", str(ROOT), "show", f"{overlay_rev}:{rel}"], check=True,
                          capture_output=True).stdout
    mode = subprocess.run(["git", "-C", str(ROOT), "ls-tree", overlay_rev, "--", rel], check=True,
                          capture_output=True, text=True).stdout.split()[0]
    return data, (0o755 if mode == "100755" else 0o644)


def installed_lib_version(d: Path) -> str:
    """the version PlatformIO prints for an installed library: the registry version, or <ver>+sha.<7> for git"""
    pm = d / ".piopm"
    if pm.is_file() and pm.read_text().strip():
        return json.loads(pm.read_text())["version"]
    lj = json.loads((d / "library.json").read_text()) if (d / "library.json").is_file() else {}
    ver = lj.get("version")
    if ver is None and (d / "library.properties").is_file():
        m = re.search(r"^version=(.*)$", (d / "library.properties").read_text(), re.M)
        ver = m.group(1).strip() if m else None
    sha = ""
    if (d / ".git").exists():     # never ask a parent repo (git -C walks up)
        sha = subprocess.run(["git", "-C", str(d), "rev-parse", "--short=7", "HEAD"], capture_output=True,
                             text=True).stdout.strip()
    return f"{ver}+sha.{sha}" if sha else str(ver)


def glob_escape(name: str) -> str:
    return re.sub(r"([*?\[])", r"[\1]", name)


def _pkg_name(d: Path) -> str:
    """The package's own name (.piopm), else the folder name before any '@<ver>'."""
    pm = d / ".piopm"
    try:
        if pm.is_file() and pm.read_text().strip():
            return json.loads(pm.read_text())["name"]
    except (ValueError, KeyError):
        pass
    return d.name.split("@", 1)[0]


def resolved_commit(d: Path, version: str):
    """C19c: (commit, how) for an installed library: the full git HEAD of its own .git, else the "+sha." part
    of its version, else (None, None). --git-dir, never -C: -C walks up to a parent repo past a broken .git."""
    short = re.search(r"\+sha\.([0-9a-f]{7,40})$", version)
    head = ""
    if (d / ".git").exists():
        head = subprocess.run(["git", f"--git-dir={d / '.git'}", "rev-parse", "HEAD"], capture_output=True,
                              text=True).stdout.strip()
    if re.fullmatch(r"[0-9a-f]{40}", head):
        if short and not head.startswith(short.group(1)):
            print(f"fl_source_archive: REFUSED, {d.name}: version {version} but its .git HEAD is {head}",
                  file=sys.stderr)
            raise SystemExit(4)
        return head, "git HEAD"
    if short:
        return short.group(1), "version"
    return None, None


def file_hashes(d: Path) -> dict:
    """C19c: sorted relative path -> sha256 of every file under d; a symlink records its target, unfollowed"""
    out = {}
    for base, dirs, files in os.walk(d):
        for n in list(dirs):
            if (Path(base) / n).is_symlink():      # os.walk does not descend it; record it as the link it is
                files.append(n)
        for n in files:
            p = Path(base) / n
            rel = p.relative_to(d).as_posix()
            out[rel] = "symlink:" + os.readlink(p) if p.is_symlink() else hashlib.sha256(p.read_bytes()).hexdigest()
    return dict(sorted(out.items()))


def copy_libs(repro: dict, tree: Path, libdeps_root: Path):
    """copy the release's libraries into tree/libdeps; returns the C19c manifest (None with no 'libs' list)"""
    libs = repro.get("libs")
    if not libs:
        print("fl_source_archive: WARNING this release lists no 'libs'; the fetched libraries are not archived",
              file=sys.stderr)
        return None
    src = libdeps_root / repro["env"]
    bad, pick = [], {}
    for name, want in sorted(libs.items()):
        # a library can sit in several folders ('<name>' and '<name>@<ver>', measured 10-27: SSD1306 pulls GFX
        # unpinned beside the pin); vendor the one at the version the release built with
        cands = [d for d in sorted(src.glob(glob_escape(name) + "*")) if d.is_dir() and _pkg_name(d) == name]
        got = [installed_lib_version(d) for d in cands]
        if want in got:
            pick[name] = cands[got.index(want)]
        else:
            bad.append(f"{name}: release built with {want}, installed {', '.join(got) or 'nothing'}")
    if bad:
        print("fl_source_archive: REFUSED, the installed libraries are not the release's:\n  " + "\n  ".join(bad),
              file=sys.stderr)
        raise SystemExit(4)
    manifest = {}
    for name in sorted(libs):
        commit, how = resolved_commit(pick[name], libs[name])       # before the copy leaves .git behind
        shutil.copytree(pick[name], tree / "libdeps" / name, ignore=shutil.ignore_patterns(".git"), symlinks=True)
        if commit is None:
            print(f"fl_source_archive: note {name} {libs[name]}: no resolved commit (no .git HEAD, no +sha.); "
                  "recorded null", file=sys.stderr)
        manifest[name] = {"version": libs[name], "commit": commit, "commit_from": how,
                          "files": file_hashes(tree / "libdeps" / name)}
    return manifest


def git(*args, text=True):
    return subprocess.run(["git", "-C", str(ROOT), *args], check=True, capture_output=True, text=text).stdout


def check_source_rev(repro: dict) -> str:
    """the commit to cut the archive from: source_rev when given (it must descend from git_rev), else git_rev"""
    rev, src = repro["git_rev"], repro.get("source_rev")
    if not src:
        return rev
    ok = subprocess.run(["git", "-C", str(ROOT), "merge-base", "--is-ancestor", rev, src],
                        capture_output=True).returncode == 0
    if not ok:
        print(f"fl_source_archive: REFUSED, source_rev {src} does not descend from git_rev {rev}", file=sys.stderr)
        raise SystemExit(5)
    return src


def check_build_inputs(tree: Path, env: str):
    """every project file platformio.ini names for the archived env must be in the archive (FL-109)"""
    cp = configparser.ConfigParser(inline_comment_prefixes=(";",), interpolation=None, strict=False)
    cp.read(tree / "platformio.ini", encoding="utf-8")
    chain, sec = [], f"env:{env}"
    while sec and sec in cp and sec not in chain:
        chain.append(sec)
        sec = cp[sec].get("extends", "").strip() or None
    if f"env:{env}" not in cp:
        print(f"fl_source_archive: REFUSED, platformio.ini has no [env:{env}]", file=sys.stderr)
        raise SystemExit(5)
    missing = []
    for sec in chain:
        for opt in PIO_FILE_OPTS:
            for v in cp[sec].get(opt, "").split():
                if "$" in v:            # ${BUILD_DIR}/... is made by the build, not read from the project
                    continue
                if not (tree / v).is_file():
                    missing.append(f"[{sec}] {opt} = {v}")
        board = cp[sec].get("board", "").strip()
        if board and not (tree / "boards" / f"{board}.json").is_file():
            missing.append(f"[{sec}] board = {board} (boards/{board}.json)")
    if missing:
        print("fl_source_archive: REFUSED, platformio.ini names build inputs the allowlist left out:\n  "
              + "\n  ".join(missing), file=sys.stderr)
        raise SystemExit(5)


def scan(tree: Path) -> list:
    """FL-109: every line of every archived file that names a SCAN_TERM, less the ALLOW exceptions"""
    hits = []
    for p in sorted(tree.rglob("*")):
        if not p.is_file() or p.is_symlink():
            continue
        rel = p.relative_to(tree).as_posix()
        for n, line in enumerate(p.read_bytes().decode("latin-1").splitlines(), 1):
            if SCAN_RE.search(line) and not any(pat.fullmatch(rel) and sub in line for pat, sub in ALLOW):
                hits.append(f"{rel}:{n}: {line.strip()[:160]}")
    return hits


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("repro", type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "firmware-bench")
    ap.add_argument("--notice-out", type=Path, help="also write this release's filled notice here")
    ap.add_argument("--libdeps", type=Path, default=ROOT / ".pio" / "libdeps", help=argparse.SUPPRESS)
    # tests only: archive a pre-FL-109 release (its comments were never scrubbed) to test the other rules;
    # the hits are still listed and the result is marked unfit to ship
    ap.add_argument("--allow-scan-hits", action="store_true", help=argparse.SUPPRESS)
    a = ap.parse_args()
    repro = json.loads(a.repro.read_text())
    rev = repro["git_rev"]
    label = release_label(a.repro, rev)
    overlay_rev = repro.get("overlay_rev")
    if overlay_rev is None:
        print("fl_source_archive: WARNING no overlay_rev in this release; the overlay comes from the current tree, "
              "so a later archive of it may differ", file=sys.stderr)
    prefix = f"forgelink-source-{rev}"

    with tempfile.TemporaryDirectory() as td:
        tree = Path(td) / prefix
        tree.mkdir()
        src_rev = check_source_rev(repro)
        present = set(git("ls-tree", "--name-only", src_rev).split())
        paths = BUILD_PATHS + [f for f in UPSTREAM_FILES if f in present]
        absent = [p for p in BUILD_PATHS if p not in present]
        if absent:
            print(f"fl_source_archive: REFUSED, {src_rev} has no {', '.join(absent)}", file=sys.stderr)
            raise SystemExit(5)
        tar = subprocess.run(["git", "-C", str(ROOT), "archive", src_rev, "--", *paths], check=True,
                             capture_output=True).stdout
        with tarfile.open(fileobj=io.BytesIO(tar)) as t:
            t.extractall(tree, filter="data")
        check_build_inputs(tree, repro["env"])
        notice = None
        for rel in OVERLAY:
            data, mode = overlay_bytes(rel, overlay_rev)
            if rel == NOTICE:
                data = notice = fill_notice(data.decode(), repro, label).encode()
            (tree / rel).parent.mkdir(parents=True, exist_ok=True)
            (tree / rel).write_bytes(data)
            os.chmod(tree / rel, mode)
        manifest = copy_libs(repro, tree, a.libdeps)
        out_repro = dict(repro, overlay_rev=overlay_rev or "worktree", source_note=source_note(repro))
        if manifest is not None:
            out_repro["lib_manifest"] = manifest       # C19c: checked by fl_repro_build.sh before seeding
        (tree / "REPRO.json").write_text(json.dumps(out_repro, indent=1, sort_keys=True) + "\n")
        hits = scan(tree)
        if hits:
            print(f"fl_source_archive: {'WARNING' if a.allow_scan_hits else 'REFUSED'}, {len(hits)} archived "
                  "line(s) name a scanned term:\n  " + "\n  ".join(hits), file=sys.stderr)
            if not a.allow_scan_hits:
                raise SystemExit(6)
            print("fl_source_archive: --allow-scan-hits: this archive is NOT fit to ship", file=sys.stderr)

        nfiles = sum(1 for p in tree.rglob("*") if p.is_file())
        buf = io.BytesIO()
        with tarfile.open(fileobj=buf, mode="w", format=tarfile.PAX_FORMAT) as t:
            for p in sorted(tree.rglob("*")):
                info = t.gettarinfo(str(p), arcname=f"{prefix}/{p.relative_to(tree)}")
                info.mtime, info.uid, info.gid, info.uname, info.gname = FIXED_MTIME, 0, 0, "", ""
                info.mode = 0o755 if (p.is_dir() or info.mode & 0o100) else 0o644
                if p.is_file():
                    with open(p, "rb") as f:
                        t.addfile(info, f)
                else:
                    t.addfile(info)
        a.out.mkdir(parents=True, exist_ok=True)
        out = a.out / f"{prefix}.tar.gz"
        with open(out, "wb") as f:
            with gzip.GzipFile(filename="", mode="wb", fileobj=f, mtime=0) as g:
                g.write(buf.getvalue())
    sha = hashlib.sha256(out.read_bytes()).hexdigest()
    (out.parent / (out.name + ".sha256")).write_text(f"{sha}  {out.name}\n")
    print(f"fl_source_archive: {out}  sha256 {sha}  ({nfiles} files, from {src_rev})")
    if a.notice_out:
        a.notice_out.write_bytes(notice)
        print(f"fl_source_archive: notice {a.notice_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
