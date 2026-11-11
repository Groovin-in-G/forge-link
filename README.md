# Forge Link firmware: source code

This is the complete source code of the **Forge Link** firmware that ships with the Sampler Forge
app: **build 3541**, the image the app installs on a ZuluSCSI Blaster (UF2 SHA-256
`917f3141ace7ab340d23bb0b883f9f21511d8de2478cb00fa58fd89753643b88`). It is published under the
GNU General Public License, version 3 or later (see `LICENSE` and `COPYING`).

Forge Link is a modified version of the ZuluSCSI firmware by Rabbit Hole Computing
(github.com/ZuluSCSI/ZuluSCSI-firmware). ZuluSCSI is a trademark of Rabbit Hole Computing; this
firmware is not made or endorsed by them.

## Which build this is

| | |
|---|---|
| Build number | 3541 |
| Source revision | `cb78047e` (in the developer's own repository; this repository has fresh history) |
| PlatformIO environment | `ForgeLink_Blaster_unified_nonoteat` |
| Board ident | `Forge Link unified 2026.09.10-release git:cb78047e caps:safe-reset,data-crc trace:off build:3541` |

`REPRO.json` holds the values the original build read from git and the clock.

## Building it as shipped

You need PlatformIO (`pip install platformio`). From this folder:

```
bash tools/fl_repro_build.sh
```

That script reads `REPRO.json` and passes the original revision, build number, timestamp and
compiler date and time to the build, then runs `pio run -e ForgeLink_Blaster_unified_nonoteat`.
The libraries the release was built with are included in `libdeps/`; the script checks every one of
their files against `REPRO.json` (`lib_manifest`) before using them, so the build needs no library
downloads. The image lands in `.pio/build/ForgeLink_Blaster_unified_nonoteat/`. The script ends by
comparing the image's SHA-256 with `REPRO.json` and reports MATCH only when they are identical
(checked 2026-11-11: MATCH).

A plain `pio run -e ForgeLink_Blaster_unified_nonoteat` also builds, but it stamps its own
revision, date and build number, so its bytes differ from the shipped image.
