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

// ⛔ DSP2 · PER-EXEC MICROSECOND STAMPS - A BENCH-ONLY INSTRUMENT (-DFORGE_LINK_STAMPS).
//
// WHY: a link note sounds as its EXEC completes, and its
// send->sound p99 - median is 4.55 ms on the release build against 0.38 ms over DIN. Whether that
// spread lives on the USB/host side (frame arrival, the main loop's pick-up) or inside the SCSI
// exchange (arbitration/selection, the XL's REQ/ACK pacing, message/status) decides whether a
// firmware timed release could touch it. The bench trace stamps in whole ms; the host sees
// only the round trip. This build stamps every EXEC in board microseconds into a .bss ring that
// the host reads AFTER the run (FL_T_STAMPS, tools/fl_stamps.py).
//
// ⛔ NEVER SHIPS. tools/fl_release_gate.py refuses a UF2 whose env carries FORGE_LINK_STAMPS, whose
// HELLO ident says stamps:on, or whose binary holds the FL_STAMPS_MARKER string below.
//
// COST ON THE HOT PATH: each stamp is one inline read of the TIMER0 low word (time_us_32, the same
// counter micros() truncates) and one store to .bss - a handful of instructions in the RAM-resident
// caller, no call, no print. Packing a record into the ring (flash code, ForgeLink_stamps.cc) runs
// in the main loop AFTER the reply has gone out, so it is outside every measured interval.
//
// ⛔ MAIN LOOP ONLY. Every stamp is taken in thread mode on core 0 (the dispatcher, the EXEC phase
// loop it calls); nothing here is reachable from an interrupt, and the ring is never touched there.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#if defined(FORGE_LINK_STAMPS) && defined(FORGE_LINK_PROBE)
#define FL_STAMPS_ON 1

#include "hardware/timer.h"

// Frame type, host -> board. Payload: empty, or {flags u8 [, filter u8]}: flags bit 0 = clear the
// ring after this read; bit 1 = set the record FILTER to byte 1 for every later EXEC (0 = every EXEC,
// 1 = only note EXECs (0Ch whose first data byte is 8n/9n), 2 = only note-ons (9n)). The ring holds
// FL_STAMPS_RING records, so a run that interleaves POLLs fills it ~4x faster than its notes do;
// the filter keeps the ring for the notes. Boot default: 0 (every EXEC). Reply: FL_STAMPS_HDR_LEN bytes of header, then `count` records of FL_STAMPS_REC_LEN bytes,
// oldest first. All little-endian. tools/fl_stamps.py decodes both and is the second copy of this.
//
// REPLY HEADER (20 B):
//   0 u8  version (1)          1 u8  record length (36)      2 u16 ring capacity (records)
//   4 u16 count returned       6 u16 flags (bit 0: the ring was cleared after this read;
//                                           bits 8-15: the filter in force after this request)
//   8 u32 total records committed since boot or the last clear (total - count = overwritten)
//  12 u64 the board's time_us_64() as the reply is built (the host aligns clocks on it)
// RECORD (36 B), one per EXEC frame the dispatcher handed to fl_handle_exec:
//   0 u32 t0 - the absolute board us (low 32 bits) of the first PRESENT slot (normally RX0)
//   4 u16 d[8] - slots 1..8 (FL_ST_USB..FL_ST_FREE): us since the previous PRESENT slot; 0 when the
//              slot is absent; 0xFFFF = saturated (mask bit 15 set)
//  20 u16 mask - bit k = slot k present; bit 15 = at least one delta saturated
//  22 u8 frame type   23 u8 seq   24 u8 EXEC flags   25 u8 outcome   26 u8 status (0xFF none)
//  27 u8 cdb[0]       28 u8 first data-out byte (0 if none)
//  29 u8 control handshakes: REQ/ACK bytes that were NOT data (message out, CDB, status, message
//        in), saturating at 255. Total handshakes = this + sent + received.
//  30 u16 frame payload length  32 u16 bytes sent (data out)  34 u16 bytes received (data in)
//        (all three saturate at 0xFFFF)
#define FL_T_STAMPS 0x0D

#ifndef FL_STAMPS_RING
#define FL_STAMPS_RING 128          // records; a power of two
#endif
#define FL_STAMPS_REC_LEN 36
#define FL_STAMPS_HDR_LEN 20
#define FL_STAMPS_VERSION 1
#define FL_STAMPS_CLEAR   0x01      // request flag: clear the ring after this read
#define FL_STAMPS_SET_FILTER 0x02   // request flag: byte 1 is the new filter
#define FL_STAMPS_F_ALL      0
#define FL_STAMPS_F_NOTES    1
#define FL_STAMPS_F_NOTE_ON  2

// TQ4b (docs/note-at.md §14 cell 7) · A NOTE_AT FIRE RECORD. Every 0C exchange the board makes on its
// OWN (flTqHookExchange: a timed fire, a RELEASE, a flush's clean-up) is not an EXEC frame, so the
// commit above never sees it. It is written straight into the same ring, in the same 36-byte layout,
// marked by frame type FL_STAMPS_TYPE_FIRE (never a real frame type: 0xF1 has the reply bit set):
//   slots ARB (the anchor, t0 = r->arb_us low 32), SEL (arb + select_us, absent on NO_SELECT), FREE
//   (r->end_us); seq 0; byte 24 (EXEC flags in an EXEC record) = the first message's data byte 1
//   (so Bn 40 00 is told from Bn 7B 00); outcome = FL_O_COMPLETED when the exchange was ok, else
//   FL_O_NO_SELECT / the phase loop's outcome; status 0xFF; cdb0 0x0C; out0 = the first MIDI byte;
//   len = sent = the bytes carried when ok (sent 0 when not); received 0.
// The ring's filter applies to it as to an EXEC (a fire whose first byte is 9n is a note-on record).
#define FL_STAMPS_TYPE_FIRE  0xF1

// The stamp slots, in order along one EXEC. The record stores each as a u16 delta from the
// previous PRESENT slot (index 0 is the absolute anchor).
enum {
    FL_ST_RX0 = 0,   // the frame's first sync byte parsed (flFramesHeadStartUs; bulk: its command block)
    FL_ST_USB,       // the frame complete and CRC-checked (flFramesHeadDoneUs)       = t_usb
    FL_ST_PICK,      // the main loop dispatches it                                   = t_pick
    FL_ST_ARB,       // arbitration/selection starts (scsiHostPhySelect called)       = t_arb
    FL_ST_SEL,       // the target answered selection (BSY), select returned          = t_sel
    FL_ST_CMD,       // the CDB is sent                                               = t_cmd_done
    FL_ST_DATA,      // the last data phase transfer returned                         = t_data_done
    FL_ST_STATUS,    // the status byte is in                                         = t_status
    FL_ST_FREE,      // the bus is free                                               = t_free
    FL_ST_N
};

// The record being built for the frame in hand. Main loop only.
typedef struct {
    uint32_t t[FL_ST_N];
    uint16_t mask;          // bit k = slot k stamped
    uint8_t  open;          // an EXEC handler took this frame: commit it after the dispatch
    uint8_t  type, seq, flags, outcome, status, cdb0, out0;
    uint32_t len, sent, received, handshakes;   // handshakes: every REQ/ACK byte, data included
} fl_stamp_work_t;

// The marker string the release gate looks for in the ELF and the UF2 payload. Kept in the binary
// because forgeLinkStampsHelloLine logs it on every HELLO.
#define FL_STAMPS_MARKER "Forge Link STAMPS: per-EXEC us ring, BENCH ONLY - this build never ships"

#ifdef __cplusplus
extern "C" {
#endif
extern fl_stamp_work_t g_fl_st;
// Flash code (ForgeLink_stamps.cc), main loop only, never between two stamps of one EXEC.
void forgeLinkStampsCommit(void);
void forgeLinkStampsFrame(uint8_t seq, uint32_t req_crc, uint32_t req_len, const uint8_t *p, uint32_t len);
void forgeLinkStampsHelloLine(void);
// TQ4b: one fire record (above). Flash code, main loop only (flTqHookExchange's caller is the main loop
// or the dispatcher, never an interrupt). Never touches g_fl_st, so an EXEC record open around a
// board-made exchange (a LINK ON's owed releases) is committed intact after it.
void forgeLinkStampsFire(uint64_t arb_us, uint32_t select_us, uint64_t end_us, const uint8_t *midi, uint32_t n,
                         uint8_t outcome);
#ifdef __cplusplus
}
#endif

static inline __attribute__((always_inline)) void flStamp(int slot)
{
    g_fl_st.t[slot] = time_us_32();
    g_fl_st.mask |= (uint16_t)(1u << slot);
}
static inline __attribute__((always_inline)) void flStampAt(int slot, uint32_t us)
{
    g_fl_st.t[slot] = us;
    g_fl_st.mask |= (uint16_t)(1u << slot);
}
static inline __attribute__((always_inline)) void flStampHandshakes(uint32_t n)
{
    g_fl_st.handshakes += n;
}
#define FL_STAMP(slot)            flStamp(slot)
#define FL_STAMP_HS(n)            flStampHandshakes(n)
#define FL_STAMP_FIRE(arb, sel, end, midi, n, oc) forgeLinkStampsFire((arb), (sel), (end), (midi), (n), (oc))

#else
#define FL_STAMP(slot)            do {} while (0)
// ⛔ The ship build: nothing - no call, no argument evaluated (every argument is a plain field read anyway).
#define FL_STAMP_FIRE(arb, sel, end, midi, n, oc) do {} while (0)
// FL_STAMP_HS evaluates its argument in every build, so a call written as its argument could never
// be compiled out by accident (the call sites pass plain counts, never the transfer itself).
#define FL_STAMP_HS(n)            ((void)(n))
#endif
