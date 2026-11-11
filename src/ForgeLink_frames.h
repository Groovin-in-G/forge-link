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

// THE FRAMED USB TRANSPORT, AND IT IS ONE COPY ON PURPOSE.
//
// The wire format below is also implemented, byte for byte, in tools/forge_link_proto.py.
// Two consumers now ride it - the PROBE (bridge/initiator role) and the INJECTOR (disk role) -
// and before this file existed the parser, the CRC, the reply cache and the replay-by-sequence
// rule lived inside ForgeLink_probe.cpp where only the probe could reach them.
//
// ⛔ A SECOND COPY OF A WIRE PARSER IS A SECOND TRUTH, and this one already carries four
// hard-won behaviours that a fresh copy would not: the 2000 ms GAP timer (not a frame-duration
// timer - see the comment on the drain), the oversize/crc_errors split that told a delivery bug
// from a corruption bug, the one-frame-in-flight rule that makes it safe to drain from inside a
// SCSI transaction, and the replay cache that stops a SEND being applied twice. All four were
// paid for on the bench in September 2026.
//
// Wire format (little-endian):
//   sync[2] = F5 A7 | type[1] | seq[1] | len[4] | payload[len] | crc32[4]
//   crc32 = IEEE CRC-32 (zlib) over type, seq, len and payload.

#pragma once

#include "ForgeLink_probe.h"

#if defined(FORGE_LINK_PROBE) || defined(FORGE_LINK_INJECT)

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Frame header is type(1) seq(1) len(4); total overhead = sync 2 + hdr 6 + crc 4
#define FL_HDR_LEN 6
#define FL_EXEC_FIXED_LEN (1 + 1 + 4 + 1 + 4 + 4)  // without CDB and data-out
#define FL_MAX_PAYLOAD (FL_EXEC_FIXED_LEN + 12 + FORGE_LINK_MAX_DATA)
#define FL_REPLY_FIXED_LEN (1 + 1 + 4 + 4)

// ⭐ STAGE P · HOW MANY COMPLETE FRAMES THE PARSER MAY HOLD, IN ARRIVAL ORDER.
// The probe keeps ONE: its executor calls platform_poll() mid-EXEC and its host is strictly one
// command in flight, so nothing about it changes. The injection build keeps TWO - measured from
// the ELF with arm-none-eabi-size/nm: the slots are 65,632 B where one was 32,828, and the whole
// of stage P moved RAM 446,572 -> 481,080 B (91.8%) on the inject ELF. ~~heap 77,216 -> 42,900 B~~
// (that was the .heap SECTION from arm-none-eabi-size, not the free heap). The free heap is
// __HeapLimit - __end__ from nm on the inject_test ELF: ~~0x20080000 - 0x2007776c = 34,964 B~~ (stage P;
// superseded by the 34,428 B at 4b6c898 below, and due a re-measure after N6 probe-bulk). The
// stacks are separate: core 0 has 4 KiB (SCRATCH_Y), core 1 has 2.5 KiB (SCRATCH_X). The second
// slot exists so that while the board writes run N to
// the card the NEXT run is already streaming into the second slot - USB transfer overlapping SD
// time, which is the whole of pipelining on the board side. A third slot buys nothing: the link
// (~188 ms per 32 KiB) is slower than the board's work per run, so one frame of look-ahead
// already keeps the link busy.
//
// ⭐ N6 PROBE-BULK SLICE 2 · THE PROBE+BULK BUILD HAS TWO SLOTS, PAID FOR BY A HALVED LOG RING
// (frame item 10; the review of slice 2 found the RAM). Free heap = __HeapLimit - __end__ by
// nm on ZuluSCSI_Blaster, every number MEASURED 2026-09-28 and tied to the tree it came from:
//   a8bb1da, one slot ........................................ 64,392 B
//   a8bb1da + two slots only (nothing else changed) ............ 31,320 B  ⛔ under 34,000 by 2,680
//   ca360ca, one slot (slice 2's RAM-placed code: +784 B .data) . 63,608 B
//   this change: two slots + LOGBUFSIZE 8192 (ZuluSCSI_config.h) 38,472 B  ✓ over 34,000 by 4,472
// The two one-slot numbers are BOTH right: 64,392 is the base, 63,608 is after ca360ca's code,
// which is RAM-placed like the rest of the frames path. The second slot costs 33,072 B; the
// frame's inferred ~34,428 B (and the review's ≈39,500 B) are struck by the 38,472 measured here.
// ~~The probe+bulk build stays on one slot~~ (ca360ca's fallback, superseded by this change).
// The inject builds and target7 unflag FORGE_LINK_PROBE, so neither the slot count nor the log
// ring moves for them: 55,600 / 55,600 / 55,400 B, the same as at ca360ca.
#ifndef FL_RX_SLOTS
#  if defined(FORGE_LINK_INJECT) || (defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK))
#    define FL_RX_SLOTS 2
#  else
#    define FL_RX_SLOTS 1
#  endif
#endif

// ⭐ N6 STEP 3 · THE REPLY CACHE IS SHRUNK IN INJECTION BUILDS, AND THAT IS WHAT PAYS FOR N6.
// The cache is one flat array that doubles as the data buffer for a reply. The PROBE keeps the
// full 32 KiB span, because its ECHO/SOURCE sweeps measure throughput with 32 KiB payloads and
// forge_link_proto.py drives them. The INJECTION build cannot: its free heap at 4b6c898 is
// 34,428 B (inject_test, __HeapLimit - __end__ by nm) against a 34,000 B floor, and N6's two
// vendor A/B buffers are the stage-P RX slots reused (+0 B). The 24,512 B this releases is the
// whole of the headroom N6 spends.
//
// ⛔ WHY 8,192 + 64 AND NOT LESS. The largest reply an injection build can ever send is a
// READ_SECTORS: FL_INJECT_READ_MAX_SECTORS (16) x 512 = 8,192 data bytes behind an 11-byte
// header, so the frame is 2 + 6 + 8,203 + 4 = 8,215 bytes. The trailing +64 slack below is
// SEPARATE from the span's own +64 and both are kept deliberately: sizing this array at
// 2 + 6 + 10 + 8,192 + 4 = 8,214 is ONE BYTE short of that read, and the overrun guard in
// flFramesSendReply would not overrun - it would silently send an EMPTY reply instead. A
// 16-sector read is the common path (the Mac reads the partition header before every push),
// so that byte is the difference between a working link and a mute one.
//
// The number 8,342 is a WIRE CONSTANT shared with the Mac side, which refuses to produce an
// injection-role reply above it. Changing it here without changing it there is a protocol change.
#if defined(FORGE_LINK_INJECT) && !defined(FORGE_LINK_PROBE)   // the DIP-ROLE UNION keeps the probe's span
#  define FL_REPLY_DATA_SPAN (8192 + 64)
#else
#  define FL_REPLY_DATA_SPAN FORGE_LINK_MAX_DATA
#endif
#define FL_REPLY_CACHE_BYTES (2 + FL_HDR_LEN + FL_REPLY_FIXED_LEN + FL_REPLY_DATA_SPAN + 64 + 4)

#ifdef __cplusplus
extern "C" {
#endif

uint32_t flCrc32(uint32_t crc, const uint8_t *data, size_t len);
uint32_t flGet32(const uint8_t *p);
void     flPut32(uint8_t *p, uint32_t v);

// True while a reply frame is being written; the USB log pump must not interleave.
bool flFramesTxBusy(void);

// Drain USB serial input into the parser. Stops as soon as every slot holds a complete frame,
// leaving the rest in the FIFO. With FL_RX_SLOTS == 1 that is exactly the old "one command in
// flight" rule, which is what makes it safe to call from platform_poll() inside a SCSI
// transaction. ⛔ IT NEVER DROPS A BYTE PAST A FRAME BOUNDARY (stage P): bytes of the next frame
// that arrived in the same USB read are CARRIED to the next drain. The old drain discarded them
// and relied on the host resending, which is harmless for a one-in-flight host and fatal to a
// pipelined one - every second frame would have been lost.
void flFramesRxDrain(void);

// The OLDEST complete frame is the head. ClearPending RELEASES the head slot: after it the
// parser may reuse that memory. A caller whose work outlives the dispatch (a preemptible WRITE
// run points straight into the body) must not release until the work is done.
bool           flFramesPending(void);
void           flFramesClearPending(void);
// THE DISPATCHERS' DOOR (C2a). flFramesPending, after first RELEASING every queued frame a BULK
// RESET has abandoned - unexecuted, unanswered, logged once - and marking the surviving head as
// the request the next reply answers. Without the bulk driver it is exactly flFramesPending.
// Main loop only, and only when nothing (a WRITE run) holds the head.
bool           flFramesNextHead(void);
const uint8_t *flFramesHdr(void);    // type, seq, len(4)       - of the head frame
const uint8_t *flFramesBody(void);   // payload + its 4 CRC bytes - of the head frame
uint32_t       flFramesCrcErrors(void);
uint32_t       flFramesOversize(void);
uint32_t       flFramesReplyDroppedDisconnected(void);   // V-F1: replies dropped, CDC port not open
#ifdef FL_SILENTX_DIAG
void           flFramesSilentxCounters(uint32_t out[6]); // SILENT-X v2 (BENCH ONLY): see ForgeLink_frames.cpp
#endif
uint32_t       flFramesQueued(void);         // complete frames held, 0..FL_RX_SLOTS

// Stage P timing for the head frame, in board micros(): when its first sync byte was parsed and
// when its CRC checked out. The difference is the USB transfer as the board saw it (including
// any stretch in which the board was too busy to drain); "now minus done" is its queue wait.
uint32_t       flFramesHeadStartUs(void);
uint32_t       flFramesHeadDoneUs(void);

// Same sequence AND identical request bytes as the last one: the host did not get our
// reply. Resends the cached frame and returns true; the caller must not execute again.
bool flFramesReplayedDuplicate(uint8_t seq, uint32_t req_crc, uint32_t req_len);
void flFramesInvalidateCache(void);
#ifdef FORGE_LINK_NOTE_AT
// TQ2 (docs/note-at.md §4): the frame type of the reply the cache holds (type | 0x80), 0 = none or a
// header-only entry. NOTE_AT drops a cached NOTE_AT reply at every generation bump, and only that one.
uint8_t flFramesCachedReplyType(void);
#endif

// Build a reply frame in the cache buffer and send it. p2 may point at
// flFramesReplyDataBuffer(), in which case it is not copied over itself.
void flFramesSendReply(uint8_t req_type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                       const uint8_t *p1, uint32_t n1, const uint8_t *p2, uint32_t n2);

// Payload lands directly where the reply frame will carry it, so the reply cache
// doubles as the data buffer (RAM on the Blaster build is tight).
// ⛔ C39: this getter and flFramesReplyPayloadAt WAIT (flFramesWaitBulkTxIdle: thread mode, <= 2 s)
// for the previous reply to finish leaving these bytes on the vendor IN endpoint, so ask for the
// pointer immediately before writing, from the main loop, never with USB.mutex held.
uint8_t *flFramesReplyDataBuffer(void);
uint32_t flFramesReplyDataCapacity(void);

#ifdef FORGE_LINK_BULK
// ---- N6 STEP 3 · SLOT OWNERSHIP ---------------------------------------------------------------
// The vendor-bulk driver and the CDC parser feed the SAME two RX slots, so exactly one of them
// owns them at a time. This is code, not convention (frame C6), and the rule it protects is the
// oldest one in this lane: while bulk owns the slots the CDC drain READS AND DISCARDS, COUNTED -
// it never parses and never forwards. A stalled CDC FIFO is where the console keys wait, and the
// `u`-then-`y` bootloader rule depends on no byte reaching the key handler by accident.
#define FL_OWNER_CDC  0
#define FL_OWNER_BULK 1

// Take the slots. Refused unless the CDC parser is at a standing start (sync-0, nothing queued)
// and the caller reports no WRITE run or hash job in flight. Returns true on success.
bool     flFramesClaimForBulk(bool work_active);
// Give them back. Refused while anything is still queued.
bool     flFramesReleaseFromBulk(void);
uint8_t  flFramesOwner(void);
uint32_t flFramesCdcDiscarded(void);   // CDC bytes read and thrown away while bulk owned

// Where a bulk data phase is armed so the slot ends up INDISTINGUISHABLE from a CDC-delivered
// frame: the command block's header bytes sit at the body's start and the data follows them,
// exactly as the CDC parser would have laid it out. The driver writes those header bytes itself
// from the command block. The whole WRITE run machinery is then untouched.
// ⭐ N6 PROBE-BULK: the header length is the CALLER'S, because it is no longer one number. A
// WRITE_BULK's is the 13-byte stage-P header; an EXEC's is its own 15 + cdb_len bytes. `hdr_len`
// 0 is the body's start (where the driver copies the header to). The capacity is what fits after
// it, keeping the 4 CRC bytes the commit writes; a header past the body gives 0.
uint8_t *flFramesBulkArmTarget(uint32_t hdr_len);
uint32_t flFramesBulkArmCapacity(uint32_t hdr_len);
bool     flFramesBulkSlotFree(void);
// Publish the tail slot as a complete frame, in order, as if the parser had built it.
void     flFramesBulkCommit(uint8_t type, uint8_t seq, uint32_t payload_len,
                            uint32_t start_us);
// Installed by the bulk driver so a reply goes out on the vendor IN endpoint while bulk owns
// the slots. Null everywhere else, and flFramesSendReply falls back to CDC.
void     flFramesSetBulkTx(bool (*tx)(const uint8_t *data, uint32_t len));
// ⭐ N6 PROBE-BULK · THE REPLY BUFFER IS ALSO A BULK IN TRANSFER'S SOURCE. A reply sent on the
// vendor IN endpoint is transmitted straight out of the reply cache, and that transfer can still
// be draining when the main loop builds the NEXT reply (or an EXEC's DATA_IN lands) in the same
// bytes. The driver installs `idle` (true when its IN endpoint has no transfer in flight) beside
// the tx hook, and flFramesWaitBulkTxIdle() waits for it: thread mode only, feeding the
// watchdog, bounded at 2000 ms. In handler mode it returns at once - nothing here ever waits on
// the IRQ path, where the busy flag could not clear anyway.
void     flFramesSetBulkTxIdle(bool (*idle)(void));
void     flFramesWaitBulkTxIdle(void);
// ⭐ N6 PROBE-BULK SLICE 2 · A CHAIN BREAK NOTED AT ARRIVAL (frame item 11, the driver's half).
// The chain latch (ForgeLink_probe.cpp) is judged when a frame is DISPATCHED, but the driver
// refuses or loses a block when it ARRIVES - and with two slots those are not the same order: a
// block refused at arrival can sit between an EXEC already queued (which may yet succeed and
// clear the latch) and a CHAINED one behind it. So the driver notes the break against the
// publish counter at that moment, and every frame published after it sees the chain broken.
// Called from the driver in tud_task context; the earliest un-applied break is kept.
void     flFramesNoteChainBreak(void);
// Forget an un-applied break. The INJECTION dispatcher never asks flFramesHeadAfterChainBreak (it
// has no EXEC), so without this the first refusal there would leave the marker set for good; the
// driver clears it when the host abandons the link (USB reset, and the §3.4 bulk reset's epoch).
void     flFramesClearChainBreak(void);

// ⭐ C2a · A BULK RESET DROPS WHAT IS QUEUED (candidate D; the rule is
// flBulkFrameStale in ForgeLink_bulk_policy.c). Two plain globals, no functions, because the
// writer is the EP0 handler in tud_task's IRQ and the unified env has ~80 B of heap to spare:
//   g_fl_bulk_epoch  - the driver's CURRENT epoch. Stored by the driver at init and by every
//                      BULK RESET (IRQ); flFramesBulkCommit stamps it into the slot it publishes.
//   g_fl_reply_epoch - the stamp of the request the main loop is answering. Set by
//                      flFramesNextHead at dispatch and kept until the next one, so a WRITE run or a
//                      hash job that replies later is still judged by its own request. Main loop only;
//                      forgeLinkBulkPoll zeroes it around the driver's own refusals (never stale).
// A reply is suppressed (not sent, not cached) when flBulkFrameStale(g_fl_reply_epoch,
// g_fl_bulk_epoch): the EXEC that was running when the host reset FINISHES on the bus, but its
// answer belongs to the epoch the host abandoned. NOT CACHED either: the reset invalidated the
// cache so a repeat of the request in the new epoch runs again, and caching would undo that.
extern volatile uint16_t g_fl_bulk_epoch;
extern uint16_t          g_fl_reply_epoch;
// ⭐ ARMRETRY r8 · THE RULE (fl_bulk_end_session in ForgeLink_bulk.cpp): the first epoch ended by a session end
// the host was still waiting through (a bare OUT CLEAR_FEATURE), 0 = none. A queued frame of an epoch from
// here to the current one is dropped WITH a one-byte 0x18 (never silently), and a request of such an epoch
// that was already running keeps its real reply (flBulkFrameOwed). Written by the driver (tud_task).
extern volatile uint16_t g_fl_owed_from;

#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
// ⭐ FETCH FROM THE SLOT (framed 2026-09-29). PROBE
// + BULK ONLY: the inject and target7 ELFs compile none of this.
//
// A fetch READ's 32 KiB DATA_IN used to land in the reply cache, so the NEXT READ had to wait
// for the previous reply's IN transfer to leave those bytes before it could select (117-122 ms a
// chunk against ~85 ms of bus). Now it lands in the EXEC's OWN rx slot, behind room for the
// reply's sync/header/fixed part, and the reply frame is sent from the slot. READ k+1 selects
// while READ k's reply is still leaving from a different buffer.
//
// Where the DATA_IN lands: &head.body[2 + FL_HDR_LEN + FL_REPLY_FIXED_LEN] (offset 18, the same
// offset as in the reply cache). NULL unless bulk owns the slots and its IN carrier is installed,
// in which case the caller keeps today's reply-cache path. ⛔ THE REQUEST'S CDB IS body[7..7+cdb)
// AND A 12-BYTE ONE OVERLAPS body[18..]: the caller copies it to a local before the bus, and
// never uses this with a data-out phase (which sits at body[15+cdb..]).
uint8_t *flFramesHeadReplyArea(void);
// Build the reply frame in the head slot around the data already there and send it on bulk IN,
// holding the slot until the transfer completes. Small (< 4096) or ZLP-prone ((22 + received) %
// 64 == 0) replies, or a carrier that is not ready, are COPIED into the reply cache and sent by
// flFramesSendReply exactly as before. The replay cache keeps a HEADER-ONLY entry (frame_len 0):
// a duplicate is answered FL_O_REPLAY_UNAVAILABLE, never re-executed. Main loop only.
void     flFramesSendReplyFromHead(uint8_t req_type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                                   const uint8_t *fixed, uint32_t n_fixed, uint32_t received);
// The IN transfer the slot is held for is over (or was aborted): release the hold. A plain
// volatile store of the armed count, so every caller writes the same value - the IN completion
// (tud_task, the 1 ms alarm IRQ), the bulk reset, the USB reset, CLEAR_FEATURE on the IN
// endpoint, and the main loop when the carrier refused. No flash-only calls, no waits.
void     flFramesNoteBulkTxDone(void);
#endif
#else
static inline void flFramesWaitBulkTxIdle(void) {}
#endif // FORGE_LINK_BULK
// True ONCE, for the head frame, when a break was noted before it was published (always false
// without the bulk driver). The dispatcher asks before every frame; main loop only.
bool     flFramesHeadAfterChainBreak(void);

// Where a reply's SECOND part must be built so flFramesSendReply does not have to move it.
// ⛔ NOT A MICRO-OPTIMISATION ON THIS BOARD: a READ_SECTORS reply is up to 32 KiB, and the
// alternative is a 32 KiB memmove inside the main-loop slot, on every read, while the sampler
// is waiting for the bus. `fixed_len` is the length of the reply's first part.
uint8_t *flFramesReplyPayloadAt(uint32_t fixed_len);

#ifdef __cplusplus
}
#endif

#endif // FORGE_LINK_PROBE || FORGE_LINK_INJECT
