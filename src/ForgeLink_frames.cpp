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

// The framed USB transport, lifted out of ForgeLink_probe.cpp unchanged so the injector
// can ride it too. See ForgeLink_frames.h for why it is one copy.

#include "ForgeLink_frames.h"

#if defined(FORGE_LINK_PROBE) || defined(FORGE_LINK_INJECT)

#include <Arduino.h>
#include <string.h>
#include "ZuluSCSI_config.h"
#include "ZuluSCSI_log.h"
#include <ZuluSCSI_platform.h>
#include "ForgeLink_digest.h"
// V-F1 · the reply guard reads the CDC line state LIVE from TinyUSB (fl_cdc_line_up below). The desk
// build of this file (tools/tests/test_fl_pipeline.py) has no TinyUSB; it brings its own stub tusb.h,
// or falls back to the platform call. A firmware build must never take the fallback silently.
#if defined(__has_include)
#  if __has_include(<tusb.h>)
#    include <tusb.h>
#    define FL_HAVE_TUSB 1
#  endif
#endif
#if defined(ARDUINO) && !defined(FL_HAVE_TUSB)
#  error "Forge Link frames: <tusb.h> not found - the reply guard needs the live CDC line state"
#endif
#ifdef FORGE_LINK_BULK
#include "ForgeLink_bulk_policy.h"
#include "ForgeLink_context.h"
#ifdef FL_RXSTAMP   // RX-SLOW: included ONLY in the bench build, so the ship build's link is unchanged
#include "ForgeLink_rxstamp.h"
#endif
#endif

#ifdef FL_SILENTX_DIAG
// ⛔ BENCH ONLY (-DFL_SILENTX_DIAG; never the ship env - LAND-R7 review, the quiet-ship rule).
// SILENT-X v2 (2026-10-16): every exit where a RECEIVED request can end with no reply on bulk IN.
// Read only by bulk STATUS (ForgeLink_bulk.cpp FL_REQ_STATUS @48.., after the r7 session; LAND-R7); counting only, no behaviour change.
// Bumped from BOTH the main loop and tud_task (a 1 ms timer IRQ: flFramesBulkCommit), so every increment is
// masked (FL_SX_INC); only the ++ sits inside, never a wait or a USB call.
#include "hardware/sync.h"
#define FL_SX_INC(c) do { uint32_t fl_sx_irq_ = save_and_disable_interrupts(); (c)++; restore_interrupts(fl_sx_irq_); } while (0)
static uint32_t g_fl_sx_tx_wait_cap;      // flFramesWaitBulkTxIdle gave up after 2 s with IN still busy
static uint32_t g_fl_sx_reply_to_cdc;     // fl_carry_frame: vendor IN refused, frame went to CDC
static uint32_t g_fl_sx_write_giveup;     // fl_write_all: host stopped reading CDC, frame abandoned
static uint32_t g_fl_sx_commit_refused;   // flFramesBulkCommit: slots full, the bulk request dropped
static uint32_t g_fl_sx_dedup_hits;       // flFramesReplayedDuplicate: answered from the cache, not run
static uint32_t g_fl_sx_reply_stale;      // flFramesSendReply / replay: epoch reset while it ran, not sent
#endif


/*************************
 * CRC-32 (IEEE, zlib)   *
 *************************/

// ⭐ STAGE P: TABLE-DRIVEN, SAME NUMBERS. The bit-at-a-time loop cost ~40 cycles a byte, so every
// 32 KiB frame's CRC was a ~9 ms lump in the main loop (and every WRITE's read-back CRC another).
// The table does ~5. The function is ForgeLink_digest.c's, which the host test checks against zlib.
extern "C" uint32_t flCrc32(uint32_t crc, const uint8_t *data, size_t len)
{
    return flDigestCrc32(crc, data, len);
}

extern "C" uint32_t flGet32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

extern "C" void flPut32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

/*************************
 * USB frame I/O         *
 *************************/

enum fl_rx_state_t { FL_RX_SYNC0, FL_RX_SYNC1, FL_RX_HDR, FL_RX_BODY };

// One complete (or in-progress) frame. The parser always writes into the TAIL slot, which is
// free by construction: it is only fed while fewer than FL_RX_SLOTS frames are held.
struct fl_rx_slot_t {
    uint8_t  hdr[FL_HDR_LEN];
#ifdef FORGE_LINK_BULK
    // C2a: the bulk epoch this frame arrived in, 0 for CDC (flBulkFrameStale). The slot does not
    // grow: 6 + 2 + 32,799 = 32,807 -> 32,808 before start_us, exactly as 6 + 32,799 padded to.
    // In FRONT of body, not behind it, so every access is a short immediate offset (6) rather than
    // a movw'd 32,806 - RAM code is what the unified env has none of.
    uint16_t epoch;
#endif
    uint8_t  body[FL_MAX_PAYLOAD + 4];
    uint32_t start_us;   // first sync byte parsed
    uint32_t done_us;    // CRC checked out
};

// ⛔ N6 PROBE-BULK SLICE 2 · THE QUEUE IS TWO FREE-RUNNING COUNTERS, ONE PER SIDE (frame item 9).
// It used to be {head, count}, with `count++` on the producer side and `count--` on the consumer
// side. The bulk driver commits a frame from tud_task, which runs in the 1 ms alarm IRQ
// (ForgeLink_bulk.h), while the main loop releases the head in thread mode - and on this core
// `count++`/`count--` are each a load, an add and a store. An IRQ landing between the main
// loop's load and store lost the commit (a frame delivered but never dispatched: the host times
// out into a reset) or lost the release (a slot that stays full for good). It is the inject
// build's defect too, where WRITE_BULK commits from the same IRQ.
//
// Now each counter has exactly ONE writer: `produced` is advanced only by whoever fills the tail
// (the CDC parser or the bulk driver - never both, slot ownership sees to that), `consumed` only
// by flFramesClearPending on the main loop. Held = produced - consumed, which unsigned wrap-around
// keeps right, and the slot index is the counter mod FL_RX_SLOTS (1 and 2 both divide 2^32). A
// reader can see the other side's counter one step stale, and every such staleness is the SAFE
// direction: the producer may think a slot is still full, the consumer may not yet see a frame.
// The fences order the slot's bytes against the counter that publishes or releases them.
static struct {
    fl_rx_state_t state;
    uint32_t pos;      // bytes collected in current state
    uint32_t need;     // payload + crc bytes to collect in FL_RX_BODY
    uint32_t start_us; // when the frame being parsed began
    volatile uint32_t produced;   // frames ever published, IN ORDER - the producer's alone
    volatile uint32_t consumed;   // frames ever released - flFramesClearPending's alone
    uint32_t crc_errors;
    uint32_t oversize;
} g_fl_rx;

static fl_rx_slot_t g_fl_slot[FL_RX_SLOTS];

// Complete, CRC-valid frames waiting for the main loop.
static inline uint32_t fl_held(void)
{
    return g_fl_rx.produced - g_fl_rx.consumed;
}

static inline fl_rx_slot_t &fl_tail(void)
{
    return g_fl_slot[g_fl_rx.produced % FL_RX_SLOTS];
}

static inline fl_rx_slot_t &fl_head(void)
{
    return g_fl_slot[g_fl_rx.consumed % FL_RX_SLOTS];
}

// The tail slot is full: publish it. Its bytes are written before the count that says so.
static inline void fl_publish_tail(void)
{
    __sync_synchronize();
    g_fl_rx.produced = g_fl_rx.produced + 1;
}

static volatile bool g_fl_tx_busy = false;

static void fl_rx_reset();   // defined below; the release path resyncs the CDC parser

// ---- N6 STEP 3 · slot ownership ----------------------------------------------------------------
// ⛔ THE WHOLE FEATURE IS BEHIND -DFORGE_LINK_BULK. Without the flag not one line below is
// compiled, which is what lets the non-injection builds stay byte-identical (gate A6).
#ifdef FORGE_LINK_BULK
static uint8_t  g_fl_owner = FL_OWNER_CDC;
static uint32_t g_fl_cdc_discarded = 0;
// True while flFramesRxDrain() holds CDC bytes it has READ but not yet PARSED (the stage-P carry:
// the queue filled mid-chunk). Those bytes are the next CDC frame, so the parser can sit at sync-0
// with count 0 and still be mid-frame in every sense that matters. Set by the drain, read by the
// claim; both on the main loop.
static bool     g_fl_chunk_carried = false;
// Installed by the bulk driver at init. Null on a build without it, and on the probe.
static bool (*g_fl_bulk_tx)(const uint8_t *data, uint32_t len) = nullptr;

#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
// ⭐ THE SLOT HOLD, TWO SINGLE-WRITER COUNTERS LIKE produced/consumed.
// A reply sent from an rx slot is read by the IN transfer AFTER the main loop has released that
// slot, so the slot must not be refilled until the transfer is over. `armed` is advanced by the
// main loop alone, when it arms a from-slot transfer (and only with the IN endpoint idle, so the
// next IN completion is necessarily that transfer's). `done` is only ever STORED AS A COPY OF
// `armed` - by the IN completion (tud_task), the resets, and the main loop's own fallback - so
// every writer writes the same value and no read-modify-write can be torn. Held = armed != done.
// 9 B of state; no buffer.
static volatile uint32_t g_fl_tx_armed = 0;
static volatile uint32_t g_fl_tx_done = 0;
static volatile uint8_t  g_fl_tx_slot = 0;    // index of the held slot, valid while held

static inline bool fl_tx_hold_live(void)
{
    return g_fl_tx_armed != g_fl_tx_done;
}

extern "C" void flFramesNoteBulkTxDone(void)
{
    g_fl_tx_done = g_fl_tx_armed;
}
#endif

extern "C" void flFramesSetBulkTx(bool (*tx)(const uint8_t *, uint32_t))
{
    g_fl_bulk_tx = tx;
}

// Installed beside the tx hook (N6 probe-bulk, frame item 8). Null until the driver opens.
static bool (*g_fl_bulk_tx_idle)(void) = nullptr;

extern "C" void flFramesSetBulkTxIdle(bool (*idle)(void))
{
    g_fl_bulk_tx_idle = idle;
}

extern "C" void flFramesWaitBulkTxIdle(void)
{
    // ⛔ THREAD MODE ONLY. The IN endpoint's busy flag is cleared inside tud_task's own event
    // loop, so a wait from handler mode could never end early - it would only burn the SCSI
    // service and the watchdog feed. From handler mode this returns at once.
    if (!g_fl_bulk_tx_idle || fl_handler_context()) return;
    uint32_t start = millis();
    while (!g_fl_bulk_tx_idle() && (uint32_t)(millis() - start) < 2000)
    {
        platform_reset_watchdog();
        delay(1);
    }
#ifdef FL_SILENTX_DIAG
    if (!g_fl_bulk_tx_idle()) FL_SX_INC(g_fl_sx_tx_wait_cap);
#endif
}

extern "C" uint8_t  flFramesOwner(void)         { return g_fl_owner; }
extern "C" uint32_t flFramesCdcDiscarded(void)  { return g_fl_cdc_discarded; }

extern "C" bool flFramesClaimForBulk(bool work_active)
{
    // The predicate itself lives in the policy file so it can be driven on a desk.
    if (!flBulkClaimAllowed(g_fl_rx.state == FL_RX_SYNC0, fl_held(), work_active))
        return false;
    // ⛔ MID-FRAME CDC IS NEVER CUT OFF (2026-09-24, with the lazy claim). Sync-0 and an empty
    // queue are not quite a standing start: bytes the drain has read but not parsed, or bytes
    // already waiting in the CDC FIFO, are a frame on its way, and taking the slots now would
    // throw it into the discard counter. The bulk block waits a pass instead. Called only from
    // the main loop (forgeLinkBulkPoll), never with USB.mutex held, so available() is truthful.
    if (g_fl_chunk_carried || Serial.available() > 0)
        return false;
    g_fl_owner = FL_OWNER_BULK;
    return true;
}

extern "C" bool flFramesReleaseFromBulk(void)
{
    if (!flBulkReleaseAllowed(fl_held())) return false;
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    // a slot is still the source of a bulk IN transfer. Handing the slots to the CDC parser
    // now would let it write into bytes still on the wire; the idle release retries next pass.
    if (fl_tx_hold_live()) return false;
#endif
    g_fl_owner = FL_OWNER_CDC;
    fl_rx_reset();
    return true;
}

extern "C" uint8_t *flFramesBulkArmTarget(uint32_t hdr_len)
{
    // Offset hdr_len: for a WRITE_BULK, 13 - the sector data of a WRITE_SECTORS payload,
    // immediately after {id u8, lba u32, count u16, mode u8, flags u8, expected_gen u32}. For an
    // EXEC, 15 + cdb_len - the data-out, immediately after the EXEC's own fixed header and CDB.
    if (hdr_len > sizeof(fl_tail().body)) hdr_len = sizeof(fl_tail().body);
    return &fl_tail().body[hdr_len];
}

extern "C" uint32_t flFramesBulkArmCapacity(uint32_t hdr_len)
{
    if (hdr_len + 4 >= sizeof(fl_tail().body)) return 0;
    return (uint32_t)sizeof(fl_tail().body) - hdr_len - 4;
}

extern "C" bool flFramesBulkSlotFree(void)
{
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    // the tail is the slot a from-slot reply is still leaving from. Not free: the block
    // parks (fl_bulk_serve_cb) and the poll serves it once the IN transfer completes.
    if (fl_tx_hold_live() && (g_fl_rx.produced % FL_RX_SLOTS) == g_fl_tx_slot) return false;
#endif
    return fl_held() < FL_RX_SLOTS;
}

extern "C" void flFramesBulkCommit(uint8_t type, uint8_t seq, uint32_t payload_len,
                                   uint32_t start_us)
{
    if (fl_held() >= FL_RX_SLOTS)
    {
#ifdef FL_SILENTX_DIAG
        FL_SX_INC(g_fl_sx_commit_refused);
#endif
        return;
    }
    fl_rx_slot_t &s = fl_tail();
    s.hdr[0] = type;
    s.hdr[1] = seq;
    flPut32(&s.hdr[2], payload_len);
    // The CRC bytes the dispatcher reads for the replay cache sit just past the payload, exactly
    // where the CDC parser would have left them. The data phase was already CRC-checked as a
    // whole by the driver, against the command block's data_crc32.
    flPut32(&s.body[payload_len], flCrc32(flCrc32(0, s.hdr, FL_HDR_LEN), s.body, payload_len));
    s.start_us = start_us;    // when the COMMAND BLOCK arrived, so rx_us spans CB -> phase done
    s.done_us = micros();
    // C2a: stamped in the same context as the BULK RESET that would bump it (tud_task, or the poll
    // with USB.mutex held), so a frame is never stamped with an epoch it did not arrive in.
    s.epoch = g_fl_bulk_epoch;
    fl_publish_tail();
#ifdef FL_RXSTAMP_ON
    FL_RX(FL_RXS_COMMIT, type, seq);   // RX-SLOW (bench only)
#endif
}

// The earliest chain break the dispatcher has not applied yet: frames published at or after
// `serial` see it. Written by the driver (tud_task), applied and cleared by the main loop.
static volatile bool     g_fl_break_pending = false;
static volatile uint32_t g_fl_break_serial = 0;

extern "C" void flFramesNoteChainBreak(void)
{
    // An earlier break already covers every frame after it, so a second one adds nothing
    // until the first has been applied.
    if (g_fl_break_pending) return;
    g_fl_break_serial = g_fl_rx.produced;
    __sync_synchronize();
    g_fl_break_pending = true;
}

extern "C" void flFramesClearChainBreak(void)
{
    g_fl_break_pending = false;
}

// C2a · see ForgeLink_frames.h. Both zero-initialised (.bss): no .data on an env with none to spare.
volatile uint16_t g_fl_bulk_epoch;
uint16_t          g_fl_reply_epoch;
volatile uint16_t g_fl_owed_from;   // ARMRETRY r8 · THE RULE, see ForgeLink_frames.h

// ⛔ THE LOG TEXT LIVES IN FLASH, not .rodata: a .cpp.o's .rodata is copied to RAM on this
// template (rp23xx-template.ld excludes *.cpp.o from the flash .rodata), and the unified env's
// .data ends 4 B short of a 256 B .bss alignment step. Main loop only, so XIP is fine.
static const char k_fl_drop_a[] __attribute__((section(".flashdata.fl_c2a_a"))) =
    "Forge Link: BULK RESET dropped ";
static const char k_fl_drop_b[] __attribute__((section(".flashdata.fl_c2a_b"))) =
    " queued frame(s) of epoch ";

// The reply about to be built answers a request a BULK RESET has abandoned. Main loop only.
// noinline: two call sites, and inlined it cost 32 B of RAM code at each (measured, unified ELF).
static bool __attribute__((noinline)) fl_reply_stale(void)
{
    // ARMRETRY r8 · THE RULE: a request that was already RUNNING when a bare OUT CLEAR_FEATURE ended its
    // session finishes, and the host - still waiting - gets its real answer (an owed epoch), never silence.
    return flBulkFrameStale(g_fl_reply_epoch, g_fl_bulk_epoch) &&
           !flBulkFrameOwed(g_fl_reply_epoch, g_fl_owed_from, g_fl_bulk_epoch);
}
#endif // FORGE_LINK_BULK

extern "C" bool flFramesHeadAfterChainBreak(void)
{
#ifdef FORGE_LINK_BULK
    // ⛔ INTERRUPTS OFF FOR THE CHECK-AND-CLEAR, because the driver sets the pair from tud_task's
    // IRQ: a break noted between our compare and our clear would otherwise be wiped with it.
    // A handful of instructions, main loop only.
    bool hit = false;
    noInterrupts();
    if (g_fl_break_pending && (int32_t)(g_fl_rx.consumed - g_fl_break_serial) >= 0)
    {
        g_fl_break_pending = false;
        hit = true;
    }
    interrupts();
    return hit;
#else
    return false;
#endif
}

// Last reply, kept so a repeated request (same seq, same bytes) is answered
// from cache instead of re-executed (a SEND must never be applied twice).
static struct {
    bool valid;
    uint8_t seq;
    uint32_t request_crc;
    uint32_t request_len;
    uint32_t frame_len;
    uint8_t frame[FL_REPLY_CACHE_BYTES];   // 32,854 on the probe; 8,342 on injection builds (N6 step 3)
} g_fl_last;

// The sizes the Mac side is built against. A silent change here is a protocol change, so it is
// a compile error instead.
#if defined(FORGE_LINK_INJECT) && !defined(FORGE_LINK_PROBE)   // the union: the probe's 32,854
static_assert(FL_REPLY_CACHE_BYTES == 8342, "the injection reply cache is a wire constant (8342)");
#else
static_assert(FL_REPLY_CACHE_BYTES == 32854, "the probe reply cache must not shrink");
#endif
// And the read that must still fit: 2 sync + 6 hdr + (11 header + 16*512 data) + 4 crc.
static_assert(FL_REPLY_CACHE_BYTES >= 2 + FL_HDR_LEN + (11 + 16 * 512) + 4,
              "a full READ_SECTORS reply no longer fits the cache");

#define FL_DATA_IN_OFFSET (2 + FL_HDR_LEN + FL_REPLY_FIXED_LEN)

// ⛔ C39 (deep review, 2026-10-27) · THE CACHE IS NOT HANDED OUT WHILE IT IS ON THE WIRE.
// A reply sent on the vendor IN endpoint is read by the transfer AFTER flFramesSendReply returns, out
// of these bytes. READ_SECTORS, HASH_SECTORS, DISK_INFO and WATCH write their data into the cache
// BEFORE they call flFramesSendReply, whose own idle wait came too late: a pipelined request
// dispatched while the previous reply was still draining overwrote it mid-transfer, and the host
// received the old header and CRC around the new bytes. So the getters below are the ONE choke point:
// every writer of the cache asks for it here first, and here waits for the IN endpoint to go idle.
//  - The common case costs one call to the driver's idle test: the host reads a reply before it
//    sends the next request, so the endpoint is already idle and nothing waits.
//  - It is flFramesWaitBulkTxIdle itself: thread mode only (from handler mode it returns at once, as
//    the busy flag clears in tud_task's 1 ms alarm IRQ, which a thread-mode wait lets run), feeding
//    the watchdog every 1 ms, bounded at 2000 ms. At the bound it gives up and the caller writes
//    anyway, exactly as flFramesSendReply always did: the stuck reply then leaves with bytes its CRC
//    does not cover, the host's CRC check rejects it and it resends (answered fresh or from cache).
// - No wait while an slot hold is live: that transfer is reading an rx SLOT, not the cache, and
//    it could only have been armed with the endpoint idle (flFramesSendReplyFromHead waits first), so
//    no cache transfer is in flight behind it. The probe's EXEC asks for its landing pointer before
//    choosing slot or cache, and keeps its overlap with a from-slot reply.
//  - Without the bulk driver (CDC only) flFramesWaitBulkTxIdle is a no-op: CDC replies are written
//    synchronously by fl_write_all, so the cache is never on the wire after it returns.
static inline void fl_cache_writable(void)
{
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    if (fl_tx_hold_live()) return;
#endif
    flFramesWaitBulkTxIdle();
}

extern "C" uint8_t *flFramesReplyDataBuffer(void)
{
    fl_cache_writable();
    return &g_fl_last.frame[FL_DATA_IN_OFFSET];
}

extern "C" uint8_t *flFramesReplyPayloadAt(uint32_t fixed_len)
{
    fl_cache_writable();
    return &g_fl_last.frame[2 + FL_HDR_LEN + fixed_len];
}

extern "C" uint32_t flFramesReplyDataCapacity(void)
{
    return (uint32_t)sizeof(g_fl_last.frame) - FL_DATA_IN_OFFSET - 4;
}

extern "C" bool flFramesTxBusy(void)
{
    return g_fl_tx_busy;
}

extern "C" bool flFramesPending(void)
{
    if (fl_held() == 0) return false;
    __sync_synchronize();    // the head's bytes are read after the count that published them
    return true;
}
extern "C" void flFramesClearPending(void)
{
    // Release the HEAD. With one slot this is exactly the old `pending = false`. The consumer's
    // counter is advanced here and nowhere else (see g_fl_rx), after every read of the slot.
    if (fl_held() == 0) return;
    __sync_synchronize();
    g_fl_rx.consumed = g_fl_rx.consumed + 1;
}

extern "C" bool flFramesNextHead(void)
{
#ifdef FORGE_LINK_BULK
    // ⛔ C2a · AN ABANDONED SESSION'S QUEUED FRAMES ARE RELEASED HERE, NEVER RUN (D).
    // The BULK RESET itself only bumps the epoch: it runs in tud_task's IRQ, possibly while this
    // loop is inside the executor, so it must not touch slot memory or the consumer's counter. The
    // release is the one a completed frame gets (flFramesClearPending), so the queue, the 
    // hold and the chain-break serial all see an ordinary consumed frame.
    // C2c (L1, C2a review): the log names the DROPPED frames' epoch, taken from the first dropped
    // head. `e` ends on the surviving head's epoch when a live frame follows the dropped ones, so
    // logging `e` named the new session instead of the abandoned one.
    uint32_t n = 0;
    uint16_t e = 0;
    uint16_t dropped_epoch = 0;
    bool live;
    while ((live = flFramesPending()) && flBulkFrameStale(e = fl_head().epoch, g_fl_bulk_epoch))
    {
        if (n == 0) dropped_epoch = e;
        // ⭐ ARMRETRY r8 · THE RULE: a frame ended by a session end the host is still waiting through is
        // owed an answer - the one-byte 0x18 STALE SESSION, never silence (it did NOT run: nothing reached
        // SCSI or the card). Sent as a driver refusal is (request crc and length 0, so no replay matches it,
        // and judged by no request's epoch). A frame of an end the host asked for stays silent (C2a).
        if (flBulkFrameOwed(e, g_fl_owed_from, g_fl_bulk_epoch))
        {
            const uint8_t r = FL_DC_STALE_SESSION;
            const uint16_t keep = g_fl_reply_epoch;
            g_fl_reply_epoch = 0;
            flFramesSendReply(fl_head().hdr[0], fl_head().hdr[1], 0, 0, &r, 1, nullptr, 0);
            g_fl_reply_epoch = keep;
        }
        flFramesClearPending();
        n++;
    }
    if (n) logmsg(k_fl_drop_a, (int)n, k_fl_drop_b, (int)dropped_epoch);
    if (live) g_fl_reply_epoch = e;   // the request every reply until the next dispatch answers
    return live;
#else
    return flFramesPending();
#endif
}
extern "C" const uint8_t *flFramesHdr(void)  { return fl_head().hdr; }
extern "C" const uint8_t *flFramesBody(void) { return fl_head().body; }
extern "C" uint32_t flFramesCrcErrors(void)  { return g_fl_rx.crc_errors; }
extern "C" uint32_t flFramesOversize(void)   { return g_fl_rx.oversize; }
extern "C" uint32_t flFramesQueued(void)     { return fl_held(); }
extern "C" uint32_t flFramesHeadStartUs(void) { return fl_head().start_us; }
extern "C" uint32_t flFramesHeadDoneUs(void)  { return fl_head().done_us; }

static void fl_rx_reset()
{
    g_fl_rx.state = FL_RX_SYNC0;
    g_fl_rx.pos = 0;
    g_fl_rx.need = 0;
}

// ⛔ R7 · A DESYNCED PARSER DISCARDS TO THE NEXT SYNC MARKER AND NEVER FORWARDS A BYTE.
// This is the whole defence against a payload byte reaching the console key handler, where
// `u` then `y` drops the board into its UF2 bootloader. Note what the SYNC0/SYNC1 arms do:
// a byte that is not the expected sync is CONSUMED HERE and goes nowhere. There is no path
// out of this function that hands a byte to anyone.
static void fl_rx_byte(uint8_t b)
{
    switch (g_fl_rx.state)
    {
        case FL_RX_SYNC0:
            if (b == FORGE_LINK_SYNC0) g_fl_rx.state = FL_RX_SYNC1;
            break;

        case FL_RX_SYNC1:
            if (b == FORGE_LINK_SYNC1)
            {
                g_fl_rx.state = FL_RX_HDR;
                g_fl_rx.pos = 0;
                g_fl_rx.start_us = micros();
            }
            else if (b != FORGE_LINK_SYNC0) g_fl_rx.state = FL_RX_SYNC0;
            break;

        case FL_RX_HDR:
            fl_tail().hdr[g_fl_rx.pos++] = b;
            if (g_fl_rx.pos == FL_HDR_LEN)
            {
                uint32_t len = flGet32(&fl_tail().hdr[2]);
                if (len > FL_MAX_PAYLOAD)
                {
                    g_fl_rx.oversize++;
                    fl_rx_reset();
                }
                else
                {
                    g_fl_rx.need = len + 4;
                    g_fl_rx.pos = 0;
                    g_fl_rx.state = FL_RX_BODY;
                }
            }
            break;

        case FL_RX_BODY:
            fl_tail().body[g_fl_rx.pos++] = b;
            if (g_fl_rx.pos == g_fl_rx.need)
            {
                fl_rx_slot_t &s = fl_tail();
                uint32_t len = g_fl_rx.need - 4;
                uint32_t crc = flCrc32(0, s.hdr, FL_HDR_LEN);
                crc = flCrc32(crc, s.body, len);
                if (crc == flGet32(&s.body[len]))
                {
                    s.start_us = g_fl_rx.start_us;
                    s.done_us = micros();
#ifdef FORGE_LINK_BULK
                    s.epoch = 0;                 // C2a: CDC frames are never stale
#endif
                    fl_publish_tail();           // the tail becomes a queued frame, in order
                }
                else
                {
                    // Dropped silently: the host resends the same sequence on timeout.
                    g_fl_rx.crc_errors++;
                }
                fl_rx_reset();
            }
            break;
    }
}

static void fl_report_reply_drops(void);   // V-F1, defined beside its counter below

extern "C" void flFramesRxDrain(void)
{
    fl_report_reply_drops();
    // Stall timeout: a frame that stops arriving mid-way (host gave up, cable glitch) must
    // not wedge the parser waiting for bytes that will never come.
    //
    // WHY 2000 AND NOT 200 (2026-09-19). This is a GAP timer, not a frame-duration timer:
    // last_byte_ms is re-stamped on every chunk below, so a frame that takes 190 ms to arrive
    // steadily never trips it. What trips it is a >200 ms HOLE in the byte stream. When one
    // occurred the parser reset MID-FRAME, resynced on random payload bytes, read garbage as a
    // length and rejected a good frame as oversize - which the board's own counters showed
    // exactly: oversize climbing while crc_errors stayed at 0. Raising it to 2000 ms drove
    // oversize to 0. The holes turned out to be our own fault injection.
    //
    // ⚠ STAGE P: THE GAP IS JUDGED ONLY WHEN THE FIFO IS EMPTY. With a queue, a frame can sit
    // half-received while the board is busy (a 64-sector write, a hash slice) and not draining;
    // the host kept sending and the bytes are waiting in the FIFO. Judging the gap before looking
    // at the FIFO would throw that frame away for the board's own busyness. So the reset fires
    // only when there is genuinely nothing more to read - a frame that STOPPED arriving.
    static uint32_t last_byte_ms = 0;

    // Read in blocks: one Serial.read() per main-loop pass capped throughput at ~40 KiB/s.
    //
    // ⛔ STAGE P: THE CHUNK IS CARRIED, NEVER DROPPED. When the queue fills mid-chunk, the rest of
    // the chunk is the NEXT frame's first bytes. The old drain threw them away ("with one command
    // in flight this does not happen in practice") - true for a one-in-flight host, and exactly
    // what a pipelined host does on every frame. They wait here until a slot frees.
    static uint8_t chunk[512];
    static uint32_t chunk_len = 0, chunk_pos = 0;

#ifdef FORGE_LINK_BULK
    // ⛔ WHILE BULK OWNS THE SLOTS THE CDC DRAIN READS AND DISCARDS, COUNTED - IT NEVER PARSES
    // AND NEVER FORWARDS. The slots are the bulk driver's buffers for the duration, so parsing
    // into them would corrupt a transfer in flight. Discarding rather than simply not reading
    // matters just as much: a stalled CDC FIFO is where the console keys wait, and the
    // `u`-then-`y` bootloader rule depends on no byte reaching the key handler by accident.
    if (g_fl_owner == FL_OWNER_BULK)
    {
        chunk_len = chunk_pos = 0;    // anything half-read belonged to the previous owner
        for (;;)
        {
            int avail = Serial.available();
            if (avail <= 0) break;
            int n = Serial.readBytes((char *)chunk, avail < (int)sizeof(chunk) ? avail : (int)sizeof(chunk));
            if (n <= 0) break;
            g_fl_cdc_discarded += (uint32_t)n;
        }
        return;
    }
#endif

    while (fl_held() < FL_RX_SLOTS)
    {
        if (chunk_pos >= chunk_len)
        {
            int avail = Serial.available();
            if (avail <= 0)
            {
                if (g_fl_rx.state != FL_RX_SYNC0 && (uint32_t)(millis() - last_byte_ms) > 2000)
                    fl_rx_reset();
                break;
            }
            int n = Serial.readBytes((char *)chunk, avail < (int)sizeof(chunk) ? avail : (int)sizeof(chunk));
            if (n <= 0) break;
            chunk_len = (uint32_t)n;
            chunk_pos = 0;
            last_byte_ms = millis();
        }
        while (chunk_pos < chunk_len && fl_held() < FL_RX_SLOTS)
            fl_rx_byte(chunk[chunk_pos++]);
    }
#ifdef FORGE_LINK_BULK
    g_fl_chunk_carried = chunk_pos < chunk_len;
#endif
}

// ⛔ V-F1 (2026-10-14) · THE REPLY GUARD READS THE LINE STATE LIVE. It used platform_serial_connected(),
// upstream's debug-log check, which caches bool(Serial) and re-reads it at most every 50 ms (and
// never while the MSC lock is held). Just after the host opens the port and raises DTR that cache
// can still say "closed", and a reply written then was dropped here without a trace: the host's
// first HELLO went unanswered, and only a resend (answered from the reply cache) got through.
// This is the same test SerialUSB::write makes itself (tud_cdc_connected: ready AND DTR), and the
// same DTR bit ForgeLink_linkguard.cc reads. Main loop only, like every caller of fl_write_all.
static inline bool fl_cdc_line_up(void)
{
#ifdef FL_HAVE_TUSB
    return tud_ready() && (tud_cdc_n_get_line_state(0) & 0x01) != 0;
#else
    return platform_serial_connected();
#endif
}

// Replies (fresh or replayed) thrown away by that guard because the port was not open, counted so
// the bench can see whether a lost reply died here. Reported by fl_report_reply_drops() below and
// on the bridge role's periodic "FL RX:" line. .bss, main loop only.
static uint32_t g_fl_reply_dropped_disconnected;
#ifdef FL_SILENTX_DIAG
// SILENT-X v2 (BENCH ONLY): the counters are declared at the top of this file.
extern "C" void flFramesSilentxCounters(uint32_t out[6])
{
    out[0] = g_fl_sx_tx_wait_cap; out[1] = g_fl_sx_reply_to_cdc; out[2] = g_fl_sx_write_giveup;
    out[3] = g_fl_sx_commit_refused; out[4] = g_fl_sx_dedup_hits; out[5] = g_fl_sx_reply_stale;
}
#endif
extern "C" uint32_t flFramesReplyDroppedDisconnected(void) { return g_fl_reply_dropped_disconnected; }

// One log line each time the count has moved, written once the port is open again, so it reaches
// the host that just connected - in BOTH roles of the union and whatever DIAGLOG says (the periodic
// line is bridge-role only). Called at the top of every drain: a compare and nothing else when the
// count is still. The text lives in flash (see k_fl_drop_a above for why not .rodata).
#ifdef ARDUINO
static const char k_fl_rdrop_a[] __attribute__((section(".flashdata.fl_vf1_a"))) =
    "Forge Link: replies dropped with the port closed: ";
static const char k_fl_rdrop_b[] __attribute__((section(".flashdata.fl_vf1_b"))) = " (+";
static const char k_fl_rdrop_c[] __attribute__((section(".flashdata.fl_vf1_c"))) = ")";
#endif
static void __attribute__((noinline)) fl_report_reply_drops(void)
{
    static uint32_t reported;
    const uint32_t n = g_fl_reply_dropped_disconnected;
    if (n == reported || !fl_cdc_line_up()) return;
#ifdef ARDUINO
    logmsg(k_fl_rdrop_a, (int)n, k_fl_rdrop_b, (int)(n - reported), k_fl_rdrop_c);
#endif
    reported = n;
}

static void fl_write_all(const uint8_t *data, uint32_t len)
{
    uint32_t start = millis();
    while (len > 0)
    {
        if (!fl_cdc_line_up())
        {
            g_fl_reply_dropped_disconnected++;
            return;
        }
        int avail = Serial.availableForWrite();
        if (avail > 0)
        {
            uint32_t n = len < (uint32_t)avail ? len : (uint32_t)avail;
            size_t w = Serial.write(data, n);
            data += w;
            len -= w;
            if (w > 0) start = millis();
        }
        platform_reset_watchdog();
        if ((uint32_t)(millis() - start) > 2000)
        {
            // Host stopped reading; give up rather than wedge the board.
#ifdef FL_SILENTX_DIAG
            FL_SX_INC(g_fl_sx_write_giveup);
#endif
            return;
        }
    }
    Serial.flush();
}

// ⛔ ONE CARRIER RULE FOR EVERY FRAME THAT LEAVES THE CACHE - a fresh reply AND a replay. While
// bulk owns the slots the frame goes out on the vendor IN endpoint (CDC only if that refuses);
// otherwise CDC. The bytes are identical either way - one frame format, two carriers.
// FIXED 2026-09-28 (the N6 probe-bulk review, "replay over bulk"): the replay used to be
// written with fl_write_all straight onto CDC whatever the owner. A block committed by the bulk
// driver IS dispatched through the same dedup check as a CDC frame (flFramesBulkCommit leaves the
// request CRC where the dispatcher reads it), so a repeated bulk request was never re-executed -
// but its cached answer went to the CDC console, the host waiting on bulk IN never saw it, and
// every retry of a lost reply ended in a timeout and a reset instead of the replay it was owed.
static void fl_carry_frame(const uint8_t *frame, uint32_t len)
{
#ifdef FORGE_LINK_BULK
    if (g_fl_owner == FL_OWNER_BULK && g_fl_bulk_tx)
    {
        if (!g_fl_bulk_tx(frame, len))
        {
#ifdef FL_SILENTX_DIAG
            FL_SX_INC(g_fl_sx_reply_to_cdc);
#endif
            fl_write_all(frame, len);   // vendor IN refused: CDC is the fallback
        }
        return;
    }
#endif
    fl_write_all(frame, len);
}

extern "C" bool flFramesReplayedDuplicate(uint8_t seq, uint32_t req_crc, uint32_t req_len)
{
    if (!g_fl_last.valid || g_fl_last.seq != seq ||
        g_fl_last.request_crc != req_crc || g_fl_last.request_len != req_len)
    {
        return false;
    }
#ifdef FL_SILENTX_DIAG
    FL_SX_INC(g_fl_sx_dedup_hits);
#endif
    // A replay touches nothing but the carrier: no target is selected, the chain latch is not
    // read or written, and the queue moves on exactly as for any other frame (the caller releases
    // the head). It never writes the cache, so it needs no idle wait before it - fl_bulk_tx waits
    // for the IN endpoint itself, in thread mode, before it arms the transfer.
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    if (g_fl_last.frame_len == 0)
    {
        // ⛔ A HEADER-ONLY ENTRY: that reply was sent from its rx slot, and the slot has been
        // reused, so the data cannot be replayed. Answer a 10-byte EXEC reply saying so -
        // {REPLAY_UNAVAILABLE, status 0xFF, received 0, sent 0} - and NEVER re-execute (a READ run
        // twice is harmless, but the rule is the rule, and the chain latch is not touched). Built
        // in the reply cache, never on the stack: a bulk IN transfer reads it after we return.
        flFramesWaitBulkTxIdle();
        uint8_t *f = g_fl_last.frame;
        f[0] = FORGE_LINK_SYNC0;
        f[1] = FORGE_LINK_SYNC1;
        f[2] = FL_T_EXEC | FL_T_REPLY_BIT;
        f[3] = seq;
        flPut32(&f[4], FL_REPLY_FIXED_LEN);
        f[8] = FL_O_REPLAY_UNAVAILABLE;
        f[9] = 0xFF;
        flPut32(&f[10], 0);
        flPut32(&f[14], 0);
        flPut32(&f[8 + FL_REPLY_FIXED_LEN], flCrc32(0, &f[2], FL_HDR_LEN + FL_REPLY_FIXED_LEN));
        platform_flush_usb_log();
        g_fl_tx_busy = true;
        fl_carry_frame(f, 2 + FL_HDR_LEN + FL_REPLY_FIXED_LEN + 4);
        g_fl_tx_busy = false;
        return true;
    }
#endif
    platform_flush_usb_log();
    g_fl_tx_busy = true;
    fl_carry_frame(g_fl_last.frame, g_fl_last.frame_len);
    g_fl_tx_busy = false;
    return true;
}

extern "C" void flFramesInvalidateCache(void)
{
    g_fl_last.valid = false;
}

#ifdef FORGE_LINK_NOTE_AT
extern "C" uint8_t flFramesCachedReplyType(void)
{
    return (g_fl_last.valid && g_fl_last.frame_len) ? g_fl_last.frame[2] : 0;
}
#endif

extern "C" void flFramesSendReply(uint8_t req_type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                                  const uint8_t *p1, uint32_t n1, const uint8_t *p2, uint32_t n2)
{
#ifdef FL_RXSTAMP_ON
    FL_RX(FL_RXS_REPLY, req_type, seq);   // RX-SLOW (bench only)
#endif
#ifdef FORGE_LINK_BULK
    // ⛔ THE PREVIOUS REPLY MAY STILL BE LEAVING FROM THESE BYTES on the vendor IN endpoint (N6
    // probe-bulk, frame item 8). Wait for it before one byte of the new frame is written.
    flFramesWaitBulkTxIdle();
    // ⛔ C2a: its request's epoch was reset while it ran - not sent, NOT cached (see frames.h).
    if (fl_reply_stale())
    {
#ifdef FL_SILENTX_DIAG
        FL_SX_INC(g_fl_sx_reply_stale);
#endif
        return;
    }
#endif
    uint8_t *f = g_fl_last.frame;
    uint32_t len = n1 + n2;
    if (2 + FL_HDR_LEN + len + 4 > sizeof(g_fl_last.frame))
    {
        // Cannot happen with the sizes above; refuse to overrun.
        len = 0; n1 = 0; n2 = 0;
    }
    f[0] = FORGE_LINK_SYNC0;
    f[1] = FORGE_LINK_SYNC1;
    f[2] = req_type | FL_T_REPLY_BIT;
    f[3] = seq;
    flPut32(&f[4], len);
    if (n1) memcpy(&f[8], p1, n1);
    if (n2 && p2 != &f[8 + n1]) memmove(&f[8 + n1], p2, n2);
    uint32_t crc = flCrc32(0, &f[2], FL_HDR_LEN + len);
    flPut32(&f[8 + len], crc);

    g_fl_last.valid = true;
    g_fl_last.seq = seq;
    g_fl_last.request_crc = req_crc;
    g_fl_last.request_len = req_len;
    g_fl_last.frame_len = 2 + FL_HDR_LEN + len + 4;

    // Flush any queued log text first, then hold the log pump off while the
    // frame goes out so text never lands inside a frame.
#ifdef FL_RXSTAMP_ON
    {   // RX-SLOW (bench only): this flush loops up to 250 ms while log text is queued - stamp it when it costs.
        const uint32_t fl_rx_t0 = FL_RX_NOW();
        platform_flush_usb_log();
        const uint32_t fl_rx_us = FL_RX_NOW() - fl_rx_t0;
        if (fl_rx_us > 50u) FL_RX_AT(fl_rx_t0, FL_RXS_LOG_FLUSH, 0, fl_rx_us);
    }
#else
    platform_flush_usb_log();
#endif
    g_fl_tx_busy = true;
    // While bulk owns the slots the reply goes out on the vendor IN endpoint, so nothing above
    // this call knows which link answered (fl_carry_frame, the one rule a replay shares).
    fl_carry_frame(g_fl_last.frame, g_fl_last.frame_len);
    g_fl_tx_busy = false;
}

#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
// ⭐ FETCH FROM THE SLOT. See ForgeLink_frames.h for the why; the pieces are here.

// A full EXEC reply frame (2 + 6 + 10 + 32,768 + 4 = 32,790 B) must fit in a slot body (32,799 B).
static_assert(FL_DATA_IN_OFFSET + FORGE_LINK_MAX_DATA + 4 <= FL_MAX_PAYLOAD + 4,
              "a full EXEC reply no longer fits its own rx slot");

// Below this a reply is copied into the cache and sent as before: small replies (control
// commands, INQUIRY) gain nothing, and the fetch READs this exists for are 32 KiB.
#define FL_FROM_SLOT_MIN 4096u

extern "C" uint8_t *flFramesHeadReplyArea(void)
{
    if (g_fl_owner != FL_OWNER_BULK || !g_fl_bulk_tx || !g_fl_bulk_tx_idle) return nullptr;
    return &fl_head().body[FL_DATA_IN_OFFSET];
}

extern "C" void flFramesSendReplyFromHead(uint8_t req_type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                                          const uint8_t *fixed, uint32_t n_fixed, uint32_t received)
{
    // C2a: an abandoned request's answer is dropped here too - and the slot is never held for it.
    if (fl_reply_stale())
    {
#ifdef FL_SILENTX_DIAG
        FL_SX_INC(g_fl_sx_reply_stale);
#endif
        return;
    }
    uint8_t *body = fl_head().body;
    const uint32_t cap = (uint32_t)sizeof(fl_head().body) - FL_DATA_IN_OFFSET - 4;
    if (received > cap) received = cap;
    const uint32_t frame_len = FL_DATA_IN_OFFSET + received + 4;   // 22 + received

    // The carrier may not have changed (release is refused while the head is queued), but this
    // is the one place the decision is re-checked, so a refusal here costs only a copy.
    bool from_slot = n_fixed == FL_REPLY_FIXED_LEN && received >= FL_FROM_SLOT_MIN &&
                     (frame_len % FL_BULK_EP_SIZE) != 0 &&            // no device ZLP behind it
                     g_fl_owner == FL_OWNER_BULK && g_fl_bulk_tx && g_fl_bulk_tx_idle;

    // The previous reply (from the cache, or from the OTHER slot) must be gone first: the hold is
    // one deep, and `armed` may only move while the IN endpoint is idle.
    flFramesWaitBulkTxIdle();
    platform_flush_usb_log();
    if (from_slot && !g_fl_bulk_tx_idle()) from_slot = false;

    if (!from_slot)
    {
        // ⛔ TODAY'S PATH, BYTE FOR BYTE: the data moves into the reply cache and flFramesSendReply
        // (unchanged) builds, caches and sends it. The slot is not held.
        uint8_t *dst = flFramesReplyDataBuffer();
        uint32_t n = received <= flFramesReplyDataCapacity() ? received : flFramesReplyDataCapacity();
        if (n) memcpy(dst, &body[FL_DATA_IN_OFFSET], n);
        flFramesSendReply(req_type, seq, req_crc, req_len, fixed, n_fixed, dst, n);
        return;
    }

    // The request's sync/header/EXEC fields in body[0..18) are dead (the handler copied what it
    // needed); the reply's sync, header and fixed part go there, the CRC after the data.
    const uint32_t len = n_fixed + received;
    body[0] = FORGE_LINK_SYNC0;
    body[1] = FORGE_LINK_SYNC1;
    body[2] = req_type | FL_T_REPLY_BIT;
    body[3] = seq;
    flPut32(&body[4], len);
    memcpy(&body[8], fixed, n_fixed);
    flPut32(&body[8 + len], flCrc32(0, &body[2], FL_HDR_LEN + len));

    // A HEADER-ONLY cache entry: the dedup still recognises this request, but its bytes are in a
    // slot that will be reused, so a duplicate is answered REPLAY_UNAVAILABLE. frame_len 0 is the
    // marker - flFramesSendReply always writes >= 12 - so it needs no flag it would have to clear.
    g_fl_last.valid = true;
    g_fl_last.seq = seq;
    g_fl_last.request_crc = req_crc;
    g_fl_last.request_len = req_len;
    g_fl_last.frame_len = 0;

    g_fl_tx_busy = true;
    g_fl_tx_slot = (uint8_t)(g_fl_rx.consumed % FL_RX_SLOTS);
    __sync_synchronize();
    // Known edge (review, optional 2): a CLEAR_FEATURE(IN) landing between this increment and the
    // xfer below clears the hold for a transfer about to start. Only reachable mid-recovery, after the
    // host has abandoned the reply; the host's next reset heals it.
    g_fl_tx_armed = g_fl_tx_armed + 1;     // the main loop's counter; the slot is held from here
    __sync_synchronize();
    if (!g_fl_bulk_tx(body, frame_len))
    {
        // Vendor IN refused: CDC is the fallback (the one carrier rule), written synchronously,
        // so nothing reads the slot after it returns and the hold ends here.
        fl_write_all(body, frame_len);
        flFramesNoteBulkTxDone();
    }
    g_fl_tx_busy = false;
}
#endif // FORGE_LINK_PROBE && FORGE_LINK_BULK

#endif // FORGE_LINK_PROBE || FORGE_LINK_INJECT
