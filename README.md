# Forge Link firmware: source code

This is the complete source code of the **Forge Link** firmware that ships with the Sampler Forge
app: **build 3191**, the image the app installs on a ZuluSCSI Blaster (UF2 SHA-256
`8295e38039af0c4e7aa479b2b125e29afe0a68e53a55442950bc5500513617f3`). It is published under the
GNU General Public License, version 3 or later (see `LICENSE` and `COPYING`).

Forge Link is a modified version of the ZuluSCSI firmware by Rabbit Hole Computing
(github.com/ZuluSCSI/ZuluSCSI-firmware). ZuluSCSI is a trademark of Rabbit Hole Computing; this
firmware is not made or endorsed by them.

## Which build this is

| | |
|---|---|
| Build number | 3191 |
| Source revision | `62087f65` (in the developer's own repository; this repository has fresh history) |
| PlatformIO environment | `ForgeLink_Blaster_unified_nonoteat` |
| Board ident | `Forge Link unified 2026.09.10-release git:62087f65 caps:safe-reset,data-crc trace:off build:3191` |

`REPRO.json` holds the values the original build read from git and the clock.

## Building it as shipped

You need PlatformIO (`pip install platformio`). From this folder:

```
bash tools/fl_repro_build.sh
```

That script reads `REPRO.json` and passes the original revision, build number, timestamp and
compiler date and time to the build, then runs
`pio run -e ForgeLink_Blaster_unified_nonoteat`. The image lands in
`.pio/build/ForgeLink_Blaster_unified_nonoteat/`. PlatformIO downloads the libraries named in
`platformio.ini` on the first build. The three Adafruit display libraries are pinned to the exact
versions build 3191 was built with (SSD1306 2.5.17, GFX Library 1.12.6, BusIO 1.17.4); the shipped
file allowed newer ones, and GFX Library 1.13.0 builds different bytes. The script ends by
comparing the image's SHA-256 with `REPRO.json` and reports MATCH only when they are identical
(checked 2026-11-11: MATCH).

A plain `pio run -e ForgeLink_Blaster_unified_nonoteat` also builds, but it stamps its own
revision, date and build number, so its bytes differ from the shipped image.
