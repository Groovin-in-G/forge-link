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

// THE LINK GUARD'S RULES, IN PLAIN C WITH NO SDK IN IT (the owner 2026-10-02: "yes").
//
// WHY. While the sampler's "MIDI over SCSI" link is ON (Akai 09 00 01 01 00 00 to ID 6) it IGNORES
// its DIN MIDI input. Only the Mac switches it off again. If the Mac app crashes,
// is killed, or the USB cable is pulled while the link is ON, nothing switches it off and the
// sampler stays deaf to MIDI until a power cycle - a customer would think it broken. So the BRIDGE
// remembers the last 09 it carried and, when the host goes quiet for FL_LINK_GUARD_MS or USB goes
// away, sends 09 00 00 00 00 00 itself.
//
// Every decision lives here; src/ForgeLink_linkguard.cc is only the glue that reads the clock and
// USB, puts the command on the bus and logs. tools/tests/test_fl_linkguard.py compiles THIS file
// with the host compiler and drives it against the Python mirror (tools/fl_link_guard.py).
//
// ⛔ THE STATE IS A HANDFUL OF BYTES OF .bss AND THE CODE IS IN FLASH (a .c and a .cc: the linker
// template sends only *.cpp.o to RAM). The unified build's free heap had 96 B over the owner's 
// floor when this was written; see the header of ForgeLink_linkguard.cc.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- the numbers -----------------------------------------------------------------------------
#define FL_LINK_GUARD_MS        10000u  // host silence that makes the board switch the link off
#define FL_LINK_GUARD_TRIES     5u      // off attempts per sequence (fl_rear's link_off: tries=5)
#define FL_LINK_GUARD_GAP_MS    2000u   // between attempts (fl_rear's link_off: gap_s=2.0; W28)
#define FL_LINK_GUARD_TARGET    6u      // the sampler's SCSI ID, the only one 09 may go to
#define FL_LINK_GUARD_EXEC_MS   5000u   // one attempt's phase-loop timeout = the Mac's default EXEC

// The two CDBs that move the remembered state, and nothing else does (docs/block-interface.md §9b;
// tools/fl_rear.py LINK_ON / LINK_OFF). Byte-exact: any other 09 leaves the state alone.
#define FL_LINK_GUARD_CDB_ON    { 0x09, 0x00, 0x01, 0x01, 0x00, 0x00 }
#define FL_LINK_GUARD_CDB_OFF   { 0x09, 0x00, 0x00, 0x00, 0x00, 0x00 }

// ---- the state (16 bytes) --------------------------------------------------------------------
typedef struct {
    uint32_t last_host_ms;  // when the last host frame (either link) finished being served
    uint32_t next_try_ms;   // earliest next off attempt, after a failed one
    uint16_t fired;         // off sequences the guard has started this boot
    uint8_t  on;            // 1 = the last 09 this board carried successfully was LINK ON
    uint8_t  tries;         // off attempts made in the running sequence
    uint8_t  gone;          // USB went away while on: FL_LG_GONE_* bits (0 = not gone)
    uint8_t  dtr;           // DTR as last seen, for the falling edge
    uint8_t  gave_up;       // sequences that ended without a successful off (saturating)
    // U1: the USB MOUNT generation's low byte as last seen; a change while on is a
    // re-enumeration (a replug). Was `pad`, so the state stays 16 bytes.
    uint8_t  mgen;
} fl_link_guard_t;

// The two kinds of `gone` (U1). LOST: unmounted, DTR fell, or the bus seen SUSPENDED - cleared by a
// host frame served while the bus is NOT suspended (the host is back). REENUM: the mount generation
// moved (a bus reset + a new configuration: a replug) - a new host session, so no host frame clears
// it; only a 09 the host carries, or the guard's own off, does (§9b: after a reconnect the host
// assumes the link OFF and sends LINK ON again).
#define FL_LG_GONE_LOST   1u
#define FL_LG_GONE_REENUM 2u

// What flLinkGuardDecide asks the glue to do.
#define FL_LG_IDLE        0u
#define FL_LG_OFF_SILENT  1u  // send 09 off: the host has been silent FL_LINK_GUARD_MS
#define FL_LG_OFF_USB     2u  // send 09 off: USB went away while the link was on

// What flLinkGuardAfterSend made of one attempt.
#define FL_LG_R_OFF       0u  // the sampler took it: the link is off, state cleared
#define FL_LG_R_RETRY     1u  // failed; the next attempt is due FL_LINK_GUARD_GAP_MS from now
#define FL_LG_R_GAVE_UP   2u  // failed FL_LINK_GUARD_TRIES times: state cleared, nothing more sent

// A host frame (CDC or vendor bulk) was served at now_ms: the host is alive. Clears the USB-gone
// latch and the attempt count of any running sequence (the host is back and owns the link) - U1: but
// only while the bus is NOT suspended (a frame buffered before a pull and served after it is not the
// host coming back), and never a REENUM latch.
void flLinkGuardNoteHostFrame(fl_link_guard_t *g, uint32_t now_ms, bool suspended);

// An EXEC finished. payload/len = the EXEC request as received (target, flags, timeout, cdbLen,
// cdb, ...); success = flExecChainSuccess's judgement of its reply. Only a SUCCESSFUL EXEC to ID 6
// whose 6-byte CDB is exactly LINK ON or LINK OFF moves the state. Returns the state after.
// U1: mount_gen = the mount generation now; a successful LINK ON adopts it (the host is on this configuration).
uint8_t flLinkGuardNoteExec(fl_link_guard_t *g, const uint8_t *payload, uint32_t len, bool success,
                            uint8_t mount_gen);

// USB as seen from the main loop, EVERY pass (fed or not, before the host frame is noted). Unmounted,
// or DTR falling 1 -> 0, while the link is on latches `gone` (LOST). DTR low that was never high (a
// host on the vendor link without the CDC port) is not a drop. U1: so does the bus being
// SUSPENDED when sampled (LOST) - a cable pulled from a bus-powered board never unmounts, it only
// suspends - and a CHANGE in the mount generation (REENUM). The suspend is judged by its LEVEL, not
// its generation: a suspend that already resumed when the main loop looks (a blip) is not a loss and
// sends no 09 off. The mount generation is stored on every call, link on or off, so a mount while off
// never latches later.
void flLinkGuardNoteUsb(fl_link_guard_t *g, bool mounted, bool dtr, bool suspended, uint8_t mount_gen);

// The decision. busy = a host frame is queued or running (the guard never interleaves with one).
uint8_t flLinkGuardDecide(const fl_link_guard_t *g, uint32_t now_ms, bool busy);

// The result of one off attempt (ok = flExecChainSuccess of the guard's own EXEC).
uint8_t flLinkGuardAfterSend(fl_link_guard_t *g, uint32_t now_ms, bool ok);

#ifdef __cplusplus
}
#endif
