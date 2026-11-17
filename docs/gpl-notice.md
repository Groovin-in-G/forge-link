# Forge Link firmware: licence, changes and source offer

Forge Link firmware is free software. You can redistribute it and/or modify it under the terms of
the GNU General Public License, version 3 or (at your option) any later version, as published by the
Free Software Foundation. It is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
The full licence text is in the file `COPYING` that comes with this notice and with the source, and
at <https://www.gnu.org/licenses/gpl-3.0.html>.

## Where it comes from

Forge Link firmware is a modified version of ZuluSCSI-firmware by Rabbit Hole Computing, which is
itself based on SCSI2SD V6 and BlueSCSI.

    ZuluSCSI(TM) - Copyright (c) 2022-2025 Rabbit Hole Computing(TM)
    SCSI2SD code - Copyright (C) 2016-2022 Michael McMaster
    BlueSCSI code - Copyright (C) 2021 Eric Helgeson

Those notices, the licence headers in every source file (some of our own files say "the Forge Link
contributors"; the holder of our changes is the company named below) and the attribution the original project
asks every copy to keep are preserved in the source and in the boot log.

## What we changed (GPLv3 section 5a)

    Modifications Copyright (c) 2026 Blue Dawn Records Ltd, trading as Blue Dawn Software.

Forge Link was modified from ZuluSCSI-firmware revision e7bd5c5 starting 2026-09-18. The changes, in
outline: a USB link that lets a computer read and write a sampler's memory over the SCSI bus while
the sampler runs; editing the disk images the board serves while the sampler is on; a firmware
update path over USB; the product name and file names changed to Forge Link; and a release check
that refuses debug builds. Every change is recorded, dated, in the source's revision history, and
each release names the exact revision it was built from (the `git:` value the board reports).

## Getting the source (GPLv3 section 6(d))

The complete corresponding source code of this firmware comes with it, in the same download, at no
extra charge: the archive `forgelink-source-cb78047e.tar.gz` sits beside the firmware file and this
notice. It holds every source file and build script used to build this exact firmware, and a recipe
(`tools/fl_repro_build.sh`, with the release's values in `REPRO.json`) that rebuilds it and checks
the result is byte-for-byte the shipped file. You may copy, change and share it under the GPL.

    release:          ForgeLink_Blaster_unified_nonoteat-ship3541-cb78047e.uf2 (git:cb78047e, build 3541)
    firmware SHA-256: 917f3141ace7ab340d23bb0b883f9f21511d8de2478cb00fa58fd89753643b88
    source archive:   forgelink-source-cb78047e.tar.gz, its SHA-256 in the .sha256 file beside it
                      (a file cannot carry its own hash, so it is not repeated here)
    source:           source tree ebe85b7a, comments scrubbed from cb78047e; builds byte for byte the firmware identified as cb78047e build 3541

The archive holds what builds the firmware and nothing else: `platformio.ini`, `src/`, `lib/` and
`boards/` (with the build scripts `platformio.ini` names), upstream's own licence and README, this
notice and `COPYING`, the rebuild recipe, `REPRO.json` and the libraries under `libdeps/`. Our working
notes, test tools and bench recordings are not part of the source: nothing in them is used to build the
firmware, as the byte-for-byte rebuild shows. Where the source line above says comments were
scrubbed, the only change from the revision the firmware reports is in comments (names and pointers to
internal notes taken out), which is why the rebuild still matches byte for byte.

## Build tools (pinned, so the source rebuilds the same file)

The libraries the build fetched are in the archive too, under `libdeps/`, at the exact versions the
release built with (listed in `REPRO.json` under `libs`). `tools/fl_repro_build.sh` builds from those
copies, so the libraries need no network. The build tools below are not in the archive: PlatformIO
downloads them when it first builds this project, so a machine that has not built it before may need the
network once, for them.

    PlatformIO Core 6.2.0
    platform  maxgerhardt/platform-raspberrypi @ 6164ed92d83c1241a1d84ad848158d7b0fb486e3
    framework rabbitholecomputing/arduino-pico @ v5.4.2-DaynaPORT
    toolchain toolchain-rp2040-earlephilhower 5.140200.250530

## Shipped beside other software

This firmware is a separate program. Sampler Forge, and the plugin that installs this firmware on a
board, are separate works under their own terms: they talk to the board over USB, and they carry
the firmware file and this notice unchanged. Nothing in this notice changes their terms, and their
terms do not limit your rights to this firmware under the GPL.

## About the name

ZuluSCSI is a trademark of Rabbit Hole Computing. It is used here only to name the original project
this firmware is derived from. Forge Link is not made, endorsed or supported by Rabbit Hole Computing,
and support questions about Forge Link should not go to them.
