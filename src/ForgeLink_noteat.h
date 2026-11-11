/**
 * Forge Link - a fork of ZuluSCSI-firmware by Rabbit Hole Computing.
 * Modifications Copyright (c) 2026 the Forge Link contributors.
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

// TQ2 · NOTE_AT's hooks into the bridge (docs/note-at.md §15). Every caller is #ifdef
// FORGE_LINK_NOTE_AT, so a build without the flag compiles none of this - and its ELF does not move.
// The rules: ForgeLink_noteat_policy.{h,c}. The glue (flash): ForgeLink_noteat.cc. All thread mode,
// main loop only; nothing here is reachable from an interrupt.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef FORGE_LINK_NOTE_AT

#define FL_T_NOTE_AT    0x10
#define FL_T_TQ_SYNC    0x11
#define FL_T_TQ_STATUS  0x12
#define FL_T_RELEASE    0x13
#define FL_O_HELD_FOR_TIMED 0x16   // EXEC outcome (§8): nothing went on the bus; + u64 retry_at_us

#ifdef __cplusplus
extern "C" {
#endif
// fl_dispatch: a frame of type 0x10-0x13. Builds and sends the reply.
void forgeLinkNoteAtFrame(uint8_t type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                          const uint8_t *p, uint32_t len);
// forgeLinkProbeMainLoop: before the next frame is picked up (safety, clean-up, due fires) ...
void forgeLinkNoteAtPoll(void);
// ... and after a frame was served (the lease).
void forgeLinkNoteAtServed(void);
// fl_handle_hello, before the reply: flush + disarm.
void forgeLinkNoteAtHello(void);
// fl_handle_exec: before the bus (true = held; retry_at written LE at retry_le[0..7]) and after it.
bool forgeLinkNoteAtExecHold(const uint8_t *p, uint32_t len, uint8_t *retry_le);
void forgeLinkNoteAtExecDone(const uint8_t *p, uint32_t len, bool success);
// The link guard's own 09 OFF: before and after.
void forgeLinkNoteAtLinkGuardBefore(void);
void forgeLinkNoteAtLinkGuardAfter(bool gave_up);
// ZuluSCSI.cpp's bridge-role save_logfile: false = defer it (a due is within 10 ms).
bool forgeLinkNoteAtLogSaveOk(void);
#ifdef __cplusplus
}
#endif

#define FL_NOTE_AT_LOG_SAVE_OK() forgeLinkNoteAtLogSaveOk()
#else
// Without the flag the bridge saves its log exactly as it always has: `if (1) save_logfile();`.
#define FL_NOTE_AT_LOG_SAVE_OK() 1
#endif
