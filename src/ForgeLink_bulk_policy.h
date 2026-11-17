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

// N6 STEP 3 · THE VENDOR-BULK POLICY - the half of the class driver that can be PROVEN ON A DESK.
//
// Step 2's ForgeLink_hwdog_policy.c is the precedent and the reason: the board this firmware runs
// on lives inside a powered S1000 and cannot be single-stepped, so anything that CAN be
// decided by pure logic is decided here, compiled with the HOST compiler by
// tools/tests/test_fl_bulk_fw.py, and driven through its real transitions. What is left in
// ForgeLink_bulk.cpp is TinyUSB glue: arming endpoints and handing bytes to these functions.
//
// ⛔ NOTHING HERE INCLUDES TINYUSB, ARDUINO OR ANY BOARD HEADER. That is what makes it testable,
// and a single #include of tusb.h would quietly end that.

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- the interface tuple -------------------------------------------------------------------
// ⛔ ALL THREE FIELDS ARE MATCHED IN open(), AND THAT IS NOT PEDANTRY. Application class drivers
// are offered every interface BEFORE the built-in ones, so a driver that matches too broadly
// steals CDC or MSC and the console dies. The core's own picotool reset interface is
// (0xFF, 0x00, 0x01) - vendor class, like ours - and it is the near miss this tuple must reject.
// (RESET_INTERFACE_SUBCLASS/PROTOCOL, pico-sdk usb_reset_interface.h:17,19.)
#define FL_BULK_ITF_CLASS     0xFF
#define FL_BULK_ITF_SUBCLASS  0x46   /* 'F' */
#define FL_BULK_ITF_PROTOCOL  0x4C   /* 'L' */
#define FL_BULK_ITF_STRING    "Forge Link bulk"

#define FL_BULK_EP_SIZE       64
#define FL_BULK_CB_BYTES      64     /* a command block is EXACTLY this, zero padded */
#define FL_BULK_DESC_BYTES    23     /* 9 interface + 7 + 7 endpoints */

// ---- the wire (C7 of the N6 step 3 frame) ----------------------------------------------------
#define FL_T_WRITE_BULK       0x27
#define FL_T_SINK_BULK        0x28

// ⛔ 17, NOT 19 - AND THE FRAME SAID 19. Re-derived from the real firmware: fl_do_write's own
// FIXED is `1 + 4 + 2 + 1 + 1 + 4` = 13 (src/ForgeLink_inject.cpp, "{id u8, lba u32, count u16,
// mode u8, flags u8, expected_write_gen u32}"), and WRITE_BULK adds ONE u32 data_crc32 on top.
// 13 + 4 = 17. The same 13 is the offset the data phase is armed at, and the frame states that
// number correctly elsewhere, so 19 is an arithmetic slip on a SHARED wire constant. The Mac
// side must encode 17 or the two halves do not meet.
#define FL_BULK_WRITE_HDR     13     /* the stage-P WRITE_SECTORS header, unchanged */
#define FL_BULK_WRITE_CB_LEN  (FL_BULK_WRITE_HDR + 4)   /* + data_crc32 = 17 */
#define FL_BULK_SINK_CB_LEN   4      /* {n u32} */

// ⭐ N6 PROBE-BULK SLICE 1 · EXEC CARRIES A DATA PHASE (frame
// item 5). The command block carries the EXEC's own fixed header and CDB, exactly the bytes
// fl_handle_exec reads (ForgeLink_probe.cpp, and build_exec in tools/forge_link_proto.py):
//   target u8 | flags u8 | timeout_ms u32 | cdb_len u8 | cdb[cdb_len] | in_alloc u32 | out_len u32
// = 15 + cdb_len bytes. The data-out, when the flags say there is one, is the separate data phase
// and lands straight after those bytes in the slot, so the slot ends up byte-for-byte a
// CDC-delivered EXEC frame. The three numbers below are the probe's own (ForgeLink_probe.h),
// spelled out here because this file includes no board header; each is #ifndef-guarded so the
// identical definition in ForgeLink_probe.h is not a redefinition.
#ifndef FL_T_EXEC
#define FL_T_EXEC             0x02
#endif
#ifndef FL_F_DATA_OUT
#define FL_F_DATA_OUT         0x01
#endif
#define FL_BULK_EXEC_FIXED    15     /* 1 + 1 + 4 + 1 + 4 + 4, without the CDB */
#define FL_BULK_EXEC_CDB_OFF  7      /* cdb_len sits at payload[6], the CDB at payload[7] */
// ⭐ TQ2 · NOTE_AT (0x10) AND RELEASE (0x13) ARE DATA-PHASE VERBS (docs/note-at.md §2, §15 item 1),
// exactly as EXEC with data out: the command block carries the fixed head (24 / 16 bytes, no other
// length), the head's length field sizes the data phase (entries_len at +12, keys_len at +12), and a
// length above the cap (384 = 6 x 64 / 128 = 2 x 64) is a bad command block. Only with the flag (or in
// the host test): without it neither type has a data phase, as before.
#if defined(FORGE_LINK_NOTE_AT) || defined(FL_BULK_NOTEAT_HOST)
#define FL_BULK_NOTEAT        1
#define FL_BULK_T_NOTE_AT     0x10
#define FL_BULK_T_RELEASE     0x13
#define FL_BULK_NOTE_AT_HEAD  24
#define FL_BULK_RELEASE_HEAD  16
#define FL_BULK_NOTE_AT_MAX   384u
#define FL_BULK_RELEASE_MAX   128u
#endif

// The largest data-out an EXEC may declare: FORGE_LINK_MAX_DATA (32768). ForgeLink_bulk.cpp
// static_asserts the two are equal, so they cannot drift.
#define FL_BULK_EXEC_MAX_DATA 32768u

// EP0, class-type to the interface. Anything else is STALLed.
#define FL_REQ_BULK_RESET     0x01
#define FL_REQ_STATUS         0x02
#define FL_REQ_REBOOT         0x03
#define FL_REQ_BOOTSEL        0x04
#define FL_REQ_MAGIC          0x464C
// 20 since 2026-09-28: busy_parks (u32 LE) appended at 16, additive - a host that asks for 16
// gets the first 16 unchanged (a control IN is cut to the host's wLength), and a host that asks
// for 20 of an older board gets 16 back and reads no busy_parks.
// ARMRETRY r7 · DATACRC: 24 - the session (u32 LE) appended at 20, additive the same way.
#ifdef FL_SILENTX_DIAG
// ⛔ BENCH ONLY (-DFL_SILENTX_DIAG; never the ship env). Bytes 0..23 stay r7's, FROZEN (the shipped app asks 24
// and reads the session at 20). SILENT-X (2026-10-16) + LAND-R7: +60 bytes AFTER the session, additive: v1
// @24..47, v2 @48..75, idle_race_seen @76..79, last_fail_ep @80, last_fail_result @81, pad @82..83.
#define FL_REQ_STATUS_BYTES   84
#else
#define FL_REQ_STATUS_BYTES   24
#endif
#define FL_BULK_EPOCH_MAX     0x7FFF
#define FL_BULK_NOSTALL_BIT   0x8000 /* wValue bit 15: flush with a ZLP instead of a stall */

// How long the slots may be held with no command block before they go back to CDC.
#define FL_BULK_IDLE_RELEASE_MS 2000

// HUB1 (2026-10-13): how long a DATA / SINKING phase may sit with no
// command block behind it before the board treats the epoch as abandoned. A 32 KiB data phase
// crosses the bus in ~60 ms (~575 KiB/s), so 3 s is ~50x the honest worst case.
#define FL_BULK_DATA_STALL_MS 3000

// ---- CB parse ---------------------------------------------------------------------------------

typedef enum {
    FL_CB_OK = 0,
    FL_CB_BAD_FRAME,      /* bad sync, bad crc, non-zero padding, or len past the block */
    FL_CB_NOT_A_CB        /* not exactly FL_BULK_CB_BYTES long: never a command block */
} fl_cb_result_t;

typedef struct {
    uint8_t  type;
    uint8_t  seq;
    uint32_t len;          /* payload length inside the block */
    uint32_t payload_off;  /* offset of the payload within the block */
    uint32_t data_bytes;   /* bytes the separate data phase will carry (0 if none) */
    /* ⭐ ARMRETRY r7 · DATACRC: which trailer the block carried, told apart by EXACT length
       (FL_GUARD_*), the payload bytes in front of it (what goes into the slot), and its two fields. */
    uint8_t  guard;
    uint32_t head_len;     /* len minus the trailer; == len for an unguarded block */
    uint32_t session;      /* the trailer's session (0 when unguarded) */
    uint32_t data_crc;     /* the trailer's data_crc32 (FL_GUARD_FULL only; else 0) */
} fl_bulk_cb_t;

// ---- ⭐ ARMRETRY r7 · DATACRC: a session and a data fingerprint on every state-changing write ----
// 2026-10-19: fingerprint, a 4-byte data check on every write. Spec, in our
// own words: docs/block-interface.md §1d "Session and data check". Only WRITE_BULK used to bind its data
// to its command (data_crc32); EXEC / NOTE_AT / RELEASE bound nothing, so a data phase left over from an
// earlier request could be committed under a new command block, and a command block queued before a
// reset is a well-formed frame. Two fields close both, whatever the USB timing:
//   session u32   minted by the BOARD (never 0) at every USB open, bus reset, BULK RESET and every
//                 abort the ARMRETRY path takes; reported in HELLO and STATUS; a block carrying any
//                 other value is refused STALE SESSION before anything runs.
//   data_crc32    zlib CRC-32 of the raw data-phase bytes (0 = CRC of nothing, for a block with no
//                 phase); a mismatch is refused BAD DATA before the commit (flFramesBulkCommit).
// The guarded shapes are EXACT LENGTHS, so the old exact lengths stay accepted, unguarded:
//   FULL (session + data_crc32, 8 B after the head): EXEC 15 + cdb_len + 8, NOTE_AT 32, RELEASE 24.
//   SESSION (u32 after the payload): WRITE_BULK 21 (its data_crc32 is already at +13), and, checked by
//   the DISPATCHERS for either carrier, WRITE_SECTORS 13 + count x 512 + 4, SYNC 5, CARD_MODE 8,
//   TQ_SYNC 16, TQ_STATUS 5 (flSessionTrailerAt). Never guarded: DIAGLOG, REBOOT_UF2, EP0, PICOBOOT.
// The two refusals are ONE-BYTE replies with the same value in every namespace they reach (the §3 block
// outcomes, the EXEC outcomes of §9b, note-at §11), unused in all three: neither is ever a success.
#define FL_GUARD_NONE         0
#define FL_GUARD_FULL         1     /* session u32 + data_crc32 u32 after the head */
#define FL_GUARD_SESSION      2     /* session u32 after the payload */
#define FL_DC_TRAILER_FULL    8
#define FL_DC_TRAILER_SESSION 4
#define FL_DC_BAD_DATA        0x17  /* the data phase's CRC-32 is not the block's: nothing ran; resend once */
#define FL_DC_STALE_SESSION   0x18  /* the block's session is not the board's: nothing ran; re-read, resend */
#define FL_DC_NO_TRAILER      0xFFFFFFFFu
#ifndef FL_FEAT_DATA_CRC
#define FL_FEAT_DATA_CRC      0x80  /* HELLO features bit (both builds): the trailers above are understood */
#endif
#ifndef FL_T_WRITE_SECTORS
#define FL_T_WRITE_SECTORS    0x22
#endif
#ifndef FL_T_SYNC
#define FL_T_SYNC             0x24
#endif
#ifndef FL_T_CARD_MODE
#define FL_T_CARD_MODE        0x25
#endif
#define FL_DC_T_TQ_SYNC       0x11
#define FL_DC_T_TQ_STATUS     0x12

// The next session (ARMRETRY r8, the r7 reviews' SHOULD a): a mint COUNTER xor a per-boot SALT (the board
// draws it once from the RP2350's TRNG). Distinct counters give distinct sessions, so no session repeats
// within 2^32 mints of one boot; a counter value whose session would be 0 is skipped. ~~prev + entropy
// through an invertible finaliser~~ (r7: a crafted or repeated clock could reproduce an earlier session).
uint32_t flBulkSessionNext(uint32_t *counter, uint32_t salt);

// ⭐ ARMRETRY r8 · THE RULE: a new session ENDS every piece of work stamped with the old one. Every end
// bumps the epoch (C2a: queued frames of an older epoch are released unexecuted). When the end is one the
// host is still waiting through (a bare OUT CLEAR_FEATURE), the replies it is owed are NOT silently lost:
// `owed_from` is the first epoch so ended since the last end the host itself asked to abandon (BULK RESET,
// bus reset, reopen, link halt), and a frame stamped with an epoch from owed_from up to (not including)
// `current` is owed a one-byte 0x18 when dropped, and keeps its real reply if it was already running.
// 0 = nothing owed. Epochs run 1..FL_BULK_EPOCH_MAX and wrap.
bool flBulkFrameOwed(uint16_t frame_epoch, uint16_t owed_from, uint16_t current);

// Before the data phase is armed (or, with no phase, before the commit): 0 = go on, else the one-byte
// refusal - FL_DC_STALE_SESSION when a guarded block's session is not `current`, FL_DC_BAD_DATA when a
// FULL block with NO data phase carries a data_crc32 other than 0. Unguarded blocks always pass.
uint8_t flBulkGuardCheckCb(const fl_bulk_cb_t *cb, uint32_t current_session);

// After a FULL block's data phase has landed: 0 = the CRC matches, else FL_DC_BAD_DATA.
uint8_t flBulkGuardCheckData(const fl_bulk_cb_t *cb, const uint8_t *data, uint32_t n);

// The session-only verbs inside one frame (either carrier). The offset of the session u32 - which is
// also the old exact payload length the handler then reads - when `len` is the guarded shape of `type`;
// FL_DC_NO_TRAILER otherwise (an unguarded frame, or a type that is never guarded).
uint32_t flSessionTrailerAt(uint8_t type, const uint8_t *p, uint32_t len);

// Parse a 64-byte command block. `len` is what the transfer actually delivered, so a short or
// long transfer is rejected as NOT a command block rather than parsed as a damaged one.
fl_cb_result_t flBulkParseCb(const uint8_t *buf, uint32_t len, fl_bulk_cb_t *out);

// The 23 descriptor bytes. Returns the length written, or 0 if `cap` is too small.
uint32_t flBulkDescriptor(uint8_t *dst, uint32_t cap,
                          uint8_t itf, uint8_t ep_out, uint8_t ep_in, uint8_t str_index);

// open() matches all three fields and nothing else.
bool flBulkOpenMatches(uint8_t cls, uint8_t sub, uint8_t proto);

// ---- the state machine -------------------------------------------------------------------------

typedef enum {
    FL_BULK_IDLE = 0,     /* not configured / not owning the slots */
    FL_BULK_WAIT_CB,      /* a 64-byte block is armed on OUT */
    FL_BULK_DATA,         /* a data phase is armed into a slot */
    FL_BULK_SINKING,      /* a refused request's data phase is going to the discard sink */
    FL_BULK_HALTED        /* both endpoints stalled after a reset, waiting for clear-halt */
} fl_bulk_state_t;

typedef struct {
    fl_bulk_state_t state;
    uint16_t epoch;
    uint32_t data_remaining;   /* bytes still expected in the current data phase */
    /* counters - every refusal in this design is counted, never silent */
    uint32_t cb_count;
    uint32_t bad_frames;
    uint32_t stale_completions;  /* completions from a previous epoch, ignored */
    uint32_t refused_arms;       /* an OUT arm met a busy endpoint holding another phase's transfer:
                                    never retried; the link halts for the host's reset (~~retried from the next xfer_cb~~) */
    uint32_t sunk_bytes;
} fl_bulk_rt_t;

void flBulkRtInit(fl_bulk_rt_t *rt);

// A completion carrying an epoch that is not the current one is ignored AND counted.
// Returns true when the completion should be acted on.
bool flBulkRtAcceptCompletion(fl_bulk_rt_t *rt, uint16_t completion_epoch);

// Bump the epoch for a reset. Wraps at FL_BULK_EPOCH_MAX and never returns 0, so 0 stays
// available as "no epoch".
uint16_t flBulkRtNewEpoch(fl_bulk_rt_t *rt);

// A refused arm is counted, not fatal (C14: the failure it leaves is a host timeout, which the
// reset recovers). Returns the running count.
uint32_t flBulkRtNoteRefusedArm(fl_bulk_rt_t *rt);

// How many bytes the data phase for this command block carries, read out of its payload.
// 0 = no data phase.
uint32_t flBulkDataBytesFor(uint8_t type, const uint8_t *payload, uint32_t len);

// May a data phase of this length be accepted for this type at all? ⛔ AN EXEC DECLARING MORE
// THAN FL_BULK_EXEC_MAX_DATA IS REFUSED - and, like every refusal here, still consumes its data
// phase so the pipe stays in step. The other types' limits are the slot capacity the driver
// already checks.
bool flBulkDataPhaseAllowed(uint8_t type, uint32_t data_bytes);

// Is this EP0 request one we answer? (A wrong magic on REBOOT/BOOTSEL is a STALL.)
bool flBulkControlAllowed(uint8_t b_request, uint16_t w_value);

// ---- the slot-ownership rule (frame C6), as logic rather than as convention -------------------
// The bulk driver may take the two RX slots ONLY from a standing start: the CDC parser at
// sync-0 (not mid-frame), nothing queued, and no WRITE run or hash job in flight. Anything else
// and a half-parsed CDC frame or a running job would have its memory taken out from under it.
bool flBulkClaimAllowed(bool cdc_at_sync0, uint32_t queued, bool work_active);

// And it goes back only when the queue has drained.
bool flBulkReleaseAllowed(uint32_t queued);

// ---- C2a · A BULK RESET ABANDONS WHAT IS QUEUED, NOT ONLY WHAT IS PARKED (cand. D) ----
// Every frame the bulk driver commits is stamped with the epoch it arrived in; a CDC frame carries
// 0. The main loop discards a queued frame whose stamp is stale WITHOUT executing it and without a
// reply, and suppresses a reply owed to a request whose stamp went stale while it ran. Without
// this an abandoned session's queued EXECs (each up to timeout + 10 s) ran after the host had
// reset, and kept the board deaf for minutes. 0 is never stale; every other stamp is stale once
// it is not the current epoch. (Main loop only - this file lives in flash.)
bool flBulkFrameStale(uint16_t frame_epoch, uint16_t current_epoch);

// ---- refusals that still consume their data phase ---------------------------------------------
// The outcome an injection build returns for an ATOMIC run that arrives while the queue holds
// other work. ⛔ DISTINCT FROM 0x01 (no quiet window) ON PURPOSE: the Mac RETRIES 0x01 and STOPS
// on 0x08, so collapsing them would turn a permanent condition into an infinite retry.
#ifndef FL_W_QUEUE_NOT_EMPTY
#define FL_W_QUEUE_NOT_EMPTY  0x08
#endif

bool flBulkAtomicMustWait(uint8_t mode, uint32_t queued_other);

// The data CRC is checked BEFORE the run starts, so a corrupted payload applies nothing.
bool flBulkDataCrcOk(uint32_t expected, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif
