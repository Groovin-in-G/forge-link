/**
 * Forge Link - a fork of ZuluSCSI-firmware by Rabbit Hole Computing.
 * Modifications Copyright (c) 2026 the Forge Link contributors.
 *
 * ZuluSCSI(TM) - Copyright (c) 2022-2025 Rabbit Hole Computing(TM)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

// BLOCK INJECTION - the board half.
//
// The Mac names a run of 512-byte sectors by (image id, LBA, count) and the board applies it
// through the image's OWN write path, from the main loop, at bus-free, then invalidates the
// read prefetch. The SD card stays the only store; the sampler never loses its disks.
//
// ⛔ THE WALL: this file deals in SECTORS and GENERATIONS. It contains no notion of a volume,
// a directory entry, a file or a sample, and it must never acquire one - both because that
// knowledge belongs to the closed app and because a board that understands the filesystem is
// a board that can corrupt it in a new way.
//
// The hooks below are called from the disk role and cost it nothing when this build is not
// selected: every one compiles to an empty inline in that case.

#pragma once

#include <stdint.h>
#include <stdbool.h>

// FL_ACT_READ / FL_ACT_WRITE and the block frame numbers are protocol, so they live with the
// rest of the wire constants.
#include "ForgeLink_probe.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef FORGE_LINK_INJECT

// Called from the main loop, in the slot between scsiPoll() and scsiDiskPoll().
void forgeLinkInjectPoll(void);

// Card-reader mode brackets. While this is true the frame parser stands DOWN and the
// stock console key menu is live, so `x` then `y` leaves card-reader mode as it always
// did. Without this a frames-only build would strand the user there until a power cycle
// (the frame's B6/R8). It costs no line in the platform layer: forgeLinkSerialPoll()
// simply returns false, and the caller falls through to the key menu.
void forgeLinkInjectCardMode(bool entering);

// N6 step 3: what the vendor-bulk driver needs to know about this side without reaching into it.
// "Busy" is a WRITE run or a hash job in flight - the two things that make it unsafe to hand the
// RX slots to another owner.
bool     forgeLinkInjectBusy(void);
uint32_t forgeLinkInjectWriteGen(void);

// The sampler's own disk traffic, recorded for WATCH and for the write generation.
// op is FL_ACT_READ or FL_ACT_WRITE. Called from scsiDiskStartRead/scsiDiskStartWrite.
void forgeLinkInjectNoteSampler(uint8_t scsi_id, uint8_t op, uint32_t lba, uint32_t count);

#else

static inline void forgeLinkInjectPoll(void) {}
static inline void forgeLinkInjectCardMode(bool entering) { (void)entering; }
static inline bool forgeLinkInjectBusy(void) { return false; }
static inline uint32_t forgeLinkInjectWriteGen(void) { return 0; }
static inline void forgeLinkInjectNoteSampler(uint8_t id, uint8_t op, uint32_t lba, uint32_t n)
{ (void)id; (void)op; (void)lba; (void)n; }

#endif // FORGE_LINK_INJECT

#ifdef __cplusplus
}
#endif
