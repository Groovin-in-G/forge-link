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

// ⭐ TQ2 · NOTE_AT - THE BOARD-CLOCK TIMED RELEASE OF LIVE MIDI, ITS WHOLE STATE MACHINE IN PLAIN C
// (docs/note-at.md DRAFT r3; the owner approved building it 2026-10-08).
//
// The queue, the admission rules, the session generation, the lease, the firing (deadline first,
// <= 16 messages per exchange), RELEASE, the sounding list keyed by (channel, key) with the first
// note-off ending it (r3), the flushes and their causes, TQ_STATUS and the
// EXEC admission check all live HERE, with no SDK, no Arduino and no board header, so
// tools/tests/test_fl_noteat.py compiles this file with the host compiler and drives every rule on a
// desk. src/ForgeLink_noteat.cc is only the glue: the clock, the one 0C exchange to ID 6, USB, the
// link guard's memory, the reply cache and the watchdog kick (the hooks below).
//
// ⛔ COMPILED ONLY WITH -DFORGE_LINK_NOTE_AT (or the host test's FL_NOTEAT_HOST). An unreferenced
// object still reorders veneers (ForgeLink_bulk_policy.c, gate A6), so without the flag it is not
// there at all, and every other env's ELF is byte-for-byte what it was.
//
// ⛔ A .c, SO ITS CODE AND .rodata LIVE IN FLASH (rp23xx-template.ld sends only *.cpp.o to RAM). It
// runs from the main loop ONLY - the dispatcher, the main-loop pass and the EXEC handler, all thread
// mode on core 0 - and is reachable from no interrupt. Its .bss is the RAM it costs (§12).

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- the wire numbers (docs/note-at.md §2, §11) ----------------------------------------------
#define FL_TQ_T_NOTE_AT      0x10
#define FL_TQ_T_SYNC         0x11
#define FL_TQ_T_STATUS       0x12
#define FL_TQ_T_RELEASE      0x13

#define FL_TQ_O_OK           0x00
#define FL_TQ_O_DUPLICATE    0x01
#define FL_TQ_O_QUEUE_FULL   0x02
#define FL_TQ_O_NOT_ARMED    0x03
#define FL_TQ_O_LINK_OFF     0x04
#define FL_TQ_O_BAD_MESSAGE  0x05
#define FL_TQ_O_BAD_REQUEST  0x06
#define FL_TQ_O_TOO_FAR      0x07
#define FL_TQ_O_STALE        0x08
#define FL_TQ_O_NOT_INIT     0x0A
#define FL_TQ_O_BOOT_MISMATCH 0x0B
#define FL_TQ_O_CANCELLED    0x0C
// The EXEC outcome admission adds (§8): nothing went on the bus; the reply carries retry_at_us.
#define FL_TQ_O_HELD_FOR_TIMED 0x16

#define FL_TQ_SYNC_PING      0x00
#define FL_TQ_SYNC_ARM       0x01
#define FL_TQ_SYNC_DISARM    0x02
#define FL_TQ_SYNC_FLUSH     0x03

// Flush causes (TQ_STATUS's last-flush record, §7)
#define FL_TQ_C_LEASE        1
#define FL_TQ_C_USB          2
#define FL_TQ_C_HELLO        3
#define FL_TQ_C_LINK_OFF     4
#define FL_TQ_C_BULK_RESET   5
#define FL_TQ_C_HOST         6
#define FL_TQ_C_FIRE_FAIL    7
#define FL_TQ_C_REARM        8

// Late-log causes (§7)
#define FL_TQ_L_BUSY         0
#define FL_TQ_L_PAST_DUE     1
#define FL_TQ_L_RETRY        2
#define FL_TQ_L_FAILED       3
#define FL_TQ_L_SLOW         4
#define FL_TQ_L_DROPPED      5
#define FL_TQ_L_SPLIT        6
#define FL_TQ_L_HOUSEKEEPING 7

// state_flags (TQ_SYNC byte 14, TQ_STATUS byte 18, RELEASE byte 15)
#define FL_TQ_SF_SOUNDING_EMPTY  0x01
#define FL_TQ_SF_LIST_OVERFLOW   0x02
#define FL_TQ_SF_CLEANUP_PENDING 0x04
#define FL_TQ_SF_UNCERTAIN       0x08
// TQ2 review 1 (not yet in docs/note-at.md): a LINK OFF was carried with releases still owed - they
// wait for the next LINK ON, and go out first then. TQ_STATUS byte 143 counts them (saturating).
#define FL_TQ_SF_STRANDED        0x10

// ---- the design numbers (§6, §12; every one a DESIGN value TQ4 may move) ----------------------
#define FL_TQ_VERSION            1
#define FL_TQ_QCAP               256    // message slots, the owner's 64-a-frame case with headroom (§6)
#define FL_TQ_RESERVE            64     // slots only releases may use
#define FL_TQ_FRAMES             64     // frame table
#define FL_TQ_FRAMES_REL_ONLY    8      // of which reserved for frames holding only releases
#define FL_TQ_LIST               64     // sounding list entries, keyed (channel, key)
#define FL_TQ_LATE_LOG           16
#define FL_TQ_FLUSH_REC          16
#define FL_TQ_MAX_MSGS           64     // per NOTE_AT frame: THE OWNER 2026-10-08, "64 notes would be good as a minimum"
#define FL_TQ_MAX_KEYS           64     // per RELEASE frame
#define FL_TQ_PER_EXCHANGE       16     // messages in one fire exchange (16 MEASURED clean)
#define FL_TQ_MAX_AHEAD_MS       1000
#define FL_TQ_FLOOR_US           500
#define FL_TQ_LATE_US            200
#define FL_TQ_SLOW_US            1000
#define FL_TQ_COALESCE_DEFAULT   300
#define FL_TQ_COALESCE_MAX       2000
#define FL_TQ_LEASE_DEFAULT      3000
#define FL_TQ_LEASE_MIN          500
#define FL_TQ_LEASE_MAX          9000
#define FL_TQ_FAIL_RUN           3      // consecutive failed fire exchanges = a link drop (fire_fail)
#define FL_TQ_RELEASE_RETRIES    3
#define FL_TQ_BUSYWAIT_US        150
// TQ2 review 1 item 3: one pass fires at most this many exchange groups (4 x 16 = 64 messages, one
// full frame), then returns so a host frame - a RELEASE above all - is served between them.
#define FL_TQ_GROUPS_PER_PASS    4
#define FL_TQ_LOGSAVE_DEFER_US   10000
#define FL_TQ_CLEANUP_RETRY_US   1000000
#define FL_TQ_EXEC_BASE_US       3500   // the board's estimate of one small EXEC (median 3.02 ms)
#define FL_TQ_EXEC_GUARD_US      300
// (KA1, midi 2026-10-09, option b): a whole-record write (a non-bulk 0C carrying >= 128 B of data out) keeps
// the XL busy applying it after the EXEC ends (~60 ms, the host's E_KDATA_BUSY_AFTER is 78 ms), and a
// note fired inside that apply can sound at a half-applied pitch. The estimate adds this tail. Model-blind: a size
// rule, no SysEx parsing; the SECOND line - the host keeps its own dues clear (note-at.md §9 rule 10).
#define FL_TQ_APPLY_TAIL_US      78000
#define FL_TQ_APPLY_TAIL_MIN_OUT 128
#define FL_TQ_GROUP_BASE_US      1500   // a queued group's own fire exchange, for retry_at (§8)
#define FL_TQ_PAST_CLAMP_US      1800000000ull   // a due older than 30 min is stored as 30 min old (build notes)

#define FL_TQ_NOTE_AT_HEAD       24
#define FL_TQ_RELEASE_HEAD       16
#define FL_TQ_SYNC_LEN           12
#define FL_TQ_NOTE_AT_REPLY      32
#define FL_TQ_SYNC_REPLY         76
#define FL_TQ_RELEASE_REPLY      24
#define FL_TQ_STATUS_FIXED       144
// The largest reply this file ever builds (TQ_STATUS with every record): the glue's buffer.
// + 2: the clear-read count trailer (B10-FATE, 2026-10-12), a u16 after the late records - see tq_status.
#define FL_TQ_REPLY_MAX          (FL_TQ_STATUS_FIXED + 4 * FL_TQ_FLUSH_REC + 12 * FL_TQ_LATE_LOG + 2)

// ---- one exchange, as the glue reports it ---------------------------------------------------------
typedef struct {
    uint64_t arb_us;      // the board began arbitration (before the select call)
    uint64_t end_us;      // the bus was free again
    uint32_t select_us;   // arbitration start -> the target answered selection (0 if it never did)
    bool     ok;          // flExecChainSuccess's judgement, exactly as a host EXEC is judged
    bool     no_select;   // the target never answered selection (the ~250 ms select wait ran out)
    // TQ2 follow-up: the target SELECTED, then the 5 s phase loop ran out (FL_O_TIMEOUT). Treated
    // exactly like no_select: the sampler is unreachable, nothing more is tried this call.
    bool     timed_out;
} fl_tq_xres_t;

// ---- the hooks: the glue (ForgeLink_noteat.cc) or the host test defines them -----------------------
uint64_t flTqHookNowUs(void);                  // the 64-bit board microsecond clock (time_us_64)
uint32_t flTqHookRandom(void);                 // boot_id source
// ONE 0C exchange to the sampler (ID 6): CDB 0C 00 nn nn nn 00, data out = the n bytes, quiet.
void     flTqHookExchange(const uint8_t *midi, uint32_t n, fl_tq_xres_t *r);
bool     flTqHookLinkOn(void);                 // the link guard's memory: the last 09 carried was ON
void     flTqHookKick(void);                   // feed the watchdog: a busy-wait, and before EVERY exchange try
void     flTqHookGenerationBumped(void);       // drop a cached NOTE_AT reply (§4)
// USB as the link guard reads it (mounted, the CDC DTR) and g_fl_bulk_epoch (0 without the fast link).
// Read at every safety check: each main-loop pass AND between the exchange groups of one fire run.
// U1: suspended = tud_suspended (the LEVEL); loss_gen = the USB loss
// generation (suspends + mounts, counted by tud_suspend_cb / tud_mount_cb; ForgeLink_linkguard.h). A
// CHANGE in loss_gen, or suspended while armed, is a USB loss, because a pulled cable on a bus-powered
// board never unmounts. Also read by an ARM, which is refused NOT_ARMED while suspended or unmounted.
void     flTqHookUsb(bool *mounted, bool *dtr, bool *suspended, uint16_t *bulk_epoch, uint16_t *loss_gen);

// ---- the entry points -----------------------------------------------------------------------------
// A NOTE_AT / TQ_SYNC / TQ_STATUS / RELEASE frame (type 0x10-0x13). rx_us = board_rx_us of the
// request; bridge = the board is in the initiator (bridge) role. Writes the reply into `out` (at least
// FL_TQ_REPLY_MAX bytes) and returns its length. SAFE MODE is the dispatcher's, before this is called.
uint32_t flTqFrame(uint8_t type, const uint8_t *p, uint32_t len, uint64_t rx_us, bool bridge, uint8_t *out);

// The main-loop pass, BEFORE the next host frame is picked up (§6): the safety checks (USB loss, the
// lease, a bulk reset - read through flTqHookUsb), the pending clean-up, then any due fire.
void     flTqPoll(void);
// A host frame was served (any frame, either link, whatever its outcome, a replay included): renews
// the lease (§4).
void     flTqServed(void);
// The EXEC admission check (§8), as an EXEC reaches the front, after its own validation and before
// the bus. Also the LINK OFF attempt's flush (§10): an EXEC that is exactly the 09 OFF to ID 6 flushes
// first (and is then never held). Returns true = HELD: answer 0x16 with *retry_at_us.
bool     flTqExecHold(const uint8_t *p, uint32_t len, uint64_t *retry_at_us);
// An EXEC that went on the bus finished: the shared accounting (§10, armed only) and exec_overran.
void     flTqExecDone(const uint8_t *p, uint32_t len, bool success);
// A HELLO arrived: flush + disarm (§10).
void     flTqHello(void);
// The link guard is about to send its own 09 OFF (host silent / USB gone): flush first (§10).
void     flTqLinkGuardBefore(void);
// ... and it has: gave_up = it failed its last try and forgot the link (pending releases go with it).
void     flTqLinkGuardAfter(bool gave_up);
// May the bridge save the SD log this pass? false while armed with a due within 10 ms (§6 item 5).
bool     flTqLogSaveOk(void);

// Read-only views, for the glue and the host test.
bool     flTqArmed(void);
uint32_t flTqBootId(void);
uint16_t flTqGeneration(void);
uint16_t flTqQueueUsed(void);

// Host test only: forget everything, as a board restart would.
void     flTqResetForTest(void);

#ifdef __cplusplus
}
#endif
