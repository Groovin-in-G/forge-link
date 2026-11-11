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

#include "ForgeLink_bulk.h"

#ifdef FORGE_LINK_BULK

#include <Arduino.h>
#include <string.h>
#include <tusb.h>
#include <device/usbd_pvt.h>
#include <USB.h>
#include <CoreMutex.h>

#include "ForgeLink_bulk_policy.h"
#include "ForgeLink_context.h"
#include "ForgeLink_frames.h"
#include "ForgeLink_probe.h"
#include "ForgeLink_inject.h"
#include "ForgeLink_hwdog.h"
#include "ZuluSCSI_log.h"
#include <ZuluSCSI_platform.h>
#include "pico/bootrom.h"
#include "hardware/sync.h"   // HUB1: save_and_disable_interrupts around the abandoned-epoch reset
#include "pico/rand.h"       // ARMRETRY r8: get_rand_32, the per-boot session salt (RP2350 TRNG-seeded)
extern "C" bool fl_ep_abort_xfer(uint8_t ep_addr);   // HUB1: src/tinyusb_override/rp2040_usb.c
                                                     // ARMRETRY r3: true = cancelled, false = EP_ABORT_DONE timed out
// ARMRETRY r6: clear one endpoint's latched BUFF_STATUS bit (rp2040_usb.c),
// and the override's counters, read only by the poll's log line (STATUS stays 20 bytes).
extern "C" void fl_ep_clear_pending(uint8_t ep_addr);
extern "C" volatile uint32_t fl_ep_abort_timeouts, fl_ep_abort_landed, fl_ep_rearm_abort_timeouts,
                             fl_ep_stale_status_drops, fl_ep_pending_cleared;

// FL_REQ_BOOTSEL's deferred entry into the USB bootloader (see forgeLinkBulkPoll).
#define FL_BOOTSEL_DELAY_MS 50u
static volatile bool g_fl_bootsel_pending = false;
static volatile uint32_t g_fl_bootsel_at_ms = 0;

// The EXEC data phase's ceiling is the probe's FORGE_LINK_MAX_DATA, spelled out in the policy
// header (which includes no board header). Equal, or this does not compile.
static_assert(FL_BULK_EXEC_MAX_DATA == FORGE_LINK_MAX_DATA,
              "the EXEC data-phase ceiling must be FORGE_LINK_MAX_DATA");

// ⛔ NOTHING IN THIS FILE IS MARKED FOR RAM, AND NOTHING NEEDS TO BE. Every function here runs in
// tud_task() context or the main loop, never in the USB hardware IRQ. See the header.

static struct {
    fl_bulk_rt_t rt;
    uint8_t  itf;
    uint8_t  ep_out;
    uint8_t  ep_in;
    uint8_t  usb_id;
    bool     registered;
    bool     opened;
    bool     card_mode;
    // ~~bool arm_pending; // an arm was refused; retry from the next callback or poll~~ REMOVED by
    // (ARMRETRY): the retry re-armed a DATA phase as a command block. See THE OUT TICKET.
    // ⭐ THE OUT TICKET: what the ONE transfer TinyUSB holds on OUT was armed FOR - its
    // kind (FL_ARM_*) and the phase generation it belongs to. Written with interrupts masked at the
    // arm, read and cleared by the OUT completion.
    uint8_t  out_kind;
    uint16_t out_gen;
    uint16_t phase_gen;        // bumped at every new DATA / SINKING phase (never at a CB arm)
    volatile uint32_t link_faults;   // links halted because OUT held a transfer not this phase's
    uint32_t link_faults_seen;       // ... of which the poll has logged and acted on
    // ARMRETRY r3: fl_ep_abort_xfer timeouts. A transfer whose abort timed out is LEFT
    // ALIVE in the DCD (a late completion on an inactive endpoint is upstream's panic), so the
    // endpoint is marked uncancelled: nothing re-arms OUT over it (the link stays HALTED) and the
    // slots it may point at are not handed to CDC, until a later abort succeeds or a bus reset
    // ends every transfer. Counted for the poll's log only; STATUS is unchanged (20 bytes).
    volatile uint32_t abort_timeouts;
    uint32_t abort_timeouts_seen;
    volatile bool out_uncancelled;
    volatile bool in_uncancelled;
    // ⭐ ARMRETRY r4: THE RE-ARM AFTER AN ABORT IS DEFERRED. The kind
    // (FL_ARM_CB / FL_ARM_SINK) an aborted endpoint is to be re-armed with, or FL_ARM_NONE, and the
    // token of the FUNC_CALL posted for it. See fl_bulk_defer_arm_locked.
    volatile uint8_t  rearm_kind;
    volatile uint16_t rearm_token;
    // A parsed command block waiting for the RX slots (see "THE SLOTS ARE TAKEN LAZILY" below).
    // Written in tud_task context, read and cleared by forgeLinkBulkPoll on the main loop.
    volatile bool cb_parked;
    uint32_t cb_start_us;      // when the command block landed
    volatile uint32_t last_cb_ms;   // for the idle release; written in tud_task's IRQ (read it before millis)
    uint32_t phase_ms;         // HUB1: when the current DATA / SINKING phase began (the stall watchdog)
    uint32_t data_remaining;   // bytes still expected in the current data phase
    uint32_t busy_parks;       // blocks parked because every slot was full (N6 probe-bulk); STATUS[16..19]
    fl_bulk_cb_t cb;           // the block being served
    uint8_t  desc[FL_BULK_DESC_BYTES];
    // ⭐ ARMRETRY r7 · DATACRC: the board-minted session (never 0 after init), how many times it
    // was minted (mixed into the next), and the blocks / frames the two checks refused (poll log only;
    // STATUS reports the session itself at 20..23).
    volatile uint32_t session;
    uint32_t session_mints;          // ARMRETRY r8: the mint COUNTER (session = counter ^ salt)
    uint32_t salt;                   // ARMRETRY r8: drawn once at init from the TRNG
    volatile uint32_t dc_stale, dc_bad_data;
    uint32_t dc_seen;
    // ARMRETRY r8 (the r7 reviews' SHOULD e): how long the data-CRC check of a completed phase took, in
    // tud_task - the last one and the worst since boot. The DATACRC log line reports both for the soak.
    volatile uint32_t crc_us_last, crc_us_max;
    uint32_t crc_us_max_seen;
    // ARMRETRY r9 (SHOULD a): the block in `cb` has been answered (a reply stashed) or handed on (committed),
    // or abandoned by an end the host asked for. A bare CLEAR owes it a 0x18 only while this is false.
    volatile bool cb_answered;
} g_bulk;

// The 64-byte command-block landing pad, and the discard sink a refused request's data phase
// goes into. The sink is deliberately ONE packet: a refused phase is drained 64 bytes at a time
// rather than buffered, because the whole point of refusing is that we do not want the bytes.
static uint8_t g_cb_buf[FL_BULK_CB_BYTES];
static uint8_t g_sink_buf[FL_BULK_EP_SIZE];

// The reply frame is built by flFramesSendReply into its own cache; we only carry it.
static bool fl_bulk_tx(const uint8_t *data, uint32_t len);
static bool fl_bulk_tx_idle(void);

// How many command-block bytes sit at the head of the slot, in front of the data phase: the
// 13-byte stage-P WRITE header for WRITE_BULK (and SINK_BULK, whose phase is never committed),
// the EXEC's own 15 + cdb_len bytes for an EXEC. The slot then reads exactly as the CDC parser
// would have left the same request.
static uint32_t fl_bulk_phase_hdr(const fl_bulk_cb_t &cb)
{
    // ARMRETRY r7 · DATACRC: head_len, never len - a guarded block's 8-byte trailer stays in g_cb_buf
    // (its data_crc32 is read back from there at the phase's end) and never reaches the slot, so the
    // committed frame is the old exact shape and nothing above the seam can tell the two apart.
#ifdef FL_BULK_NOTEAT
    // TQ2: NOTE_AT and RELEASE lay out like an EXEC - the whole head, then the data phase.
    if (cb.type == FL_BULK_T_NOTE_AT || cb.type == FL_BULK_T_RELEASE) return cb.head_len;
#endif
    return cb.type == FL_T_EXEC ? cb.head_len : (uint32_t)FL_BULK_WRITE_HDR;
}

// The DATACRC log line's text, in FLASH: a .cpp.o's .rodata is copied to RAM on this template (the
// same reason as ForgeLink_frames.cpp's k_fl_drop_*). Main loop only, so XIP is fine. The desk build
// (FL_BULK_HOST, tools/tests/bulk_glue) has no such section, and Mach-O refuses the name.
#ifdef FL_BULK_HOST
#define FL_DC_FLASH(name)
#else
#define FL_DC_FLASH(name) __attribute__((section(".flashdata." name)))
#endif
static const char k_fl_dc_a[] FL_DC_FLASH("fl_dc_a") = "Forge Link: DATACRC (ARMRETRY r7) refused: stale session ";
static const char k_fl_dc_b[] FL_DC_FLASH("fl_dc_b") = ", bad data ";
static const char k_fl_dc_c[] FL_DC_FLASH("fl_dc_c") = "; sessions minted ";
static const char k_fl_dc_d[] FL_DC_FLASH("fl_dc_d") = "; data CRC us last ";
static const char k_fl_dc_e[] FL_DC_FLASH("fl_dc_e") = " max ";

// ⭐ ARMRETRY r8 · THE RULE (the r7 reviews' MUST-A / MUST-B, verify S3 / S4): A NEW SESSION ENDS EVERY
// PIECE OF WORK STAMPED WITH THE OLD ONE. This is the ONLY mint site after init, and it does both halves:
//   1. a fresh session (counter ^ salt, never 0, no repeat within a boot), so a command block the host
//      still holds - or that is still on the wire - is refused 0x18 when it lands;
//   2. a new EPOCH (C2a), so a guarded block that was already PUBLISHED (its trailer stripped, nothing left
//      to recheck) is released unexecuted by flFramesNextHead and never reaches SCSI, the card or the queue.
//      ~~r7 minted on CLEAR_FEATURE, bus reset and reopen without moving the epoch~~ - such a frame ran.
// `owed`: the host is still waiting through this end (a bare OUT CLEAR_FEATURE; every other caller is an
// end the host asked for or cannot hear: BULK RESET, bus reset, reopen, link halt). Then nothing is lost
// silently: frames dropped for this end get a one-byte 0x18 (flFramesNextHead, via g_fl_owed_from), a frame
// already running keeps its real reply, and the chain is broken for whatever follows. Callers run in
// tud_task (a control or class callback) or masked. Flash, never on the USB hardware IRQ path.
static void fl_bulk_end_session(bool owed)
{
    const uint16_t old = g_fl_bulk_epoch;
    g_fl_bulk_epoch = flBulkRtNewEpoch(&g_bulk.rt);
    if (!owed)                    g_fl_owed_from = 0;
    else if (g_fl_owed_from == 0) g_fl_owed_from = old;
    if (owed) flFramesNoteChainBreak();
    else      g_bulk.cb_answered = true;   // r9: an end the host asked for owes the abandoned block nothing
    flFramesInvalidateCache();    // a same-bytes request after the end runs again, never a replay
    g_bulk.session = flBulkSessionNext(&g_bulk.session_mints, g_bulk.salt);
}

extern "C" uint32_t forgeLinkBulkSession(void)
{
    return g_bulk.session;
}

extern "C" uint8_t forgeLinkBulkSessionGate(uint8_t type, const uint8_t *p, uint32_t *len)
{
    const uint32_t at = flSessionTrailerAt(type, p, *len);
    if (at == FL_DC_NO_TRAILER) return 0;             // the old exact shape: unguarded, as before
    if (flGet32(&p[at]) != g_bulk.session)
    {
        g_bulk.dc_stale++;
        return FL_DC_STALE_SESSION;
    }
    *len = at;
    return 0;
}

// ⛔ HANDLER MODE: fl_handler_context() from ForgeLink_context.h, the ONE definition the hardware
// watchdog uses too (it was a pinned duplicate until 2026-09-24).

// ⭐ THE ONE-DEEP PENDING REPLY, AND IT IS WHY NO REPLY IS BUILT IN CALLBACK CONTEXT.
//
// ⛔ THE BUG THIS EXISTS TO KILL. flFramesSendReply -> fl_bulk_tx used to spin on
// usbd_edpt_busy(IN) with delay(1) x 2000. That busy flag is cleared ONLY inside tud_task's own
// event loop (device/usbd.c:648) - the very loop our callback is nested inside - so when IN was
// busy the wait COULD NOT END EARLY. Add platform_flush_usb_log's 250 ms (Serial returns 0
// because usbIRQ holds USB.mutex and CoreMutex returns unacquired on same-core re-entry) and
// fl_write_all's 2 s fallback and it is ~4.25 s inside the USB task IRQ. For those seconds the
// main loop, the S1000's SCSI service, CDC and the host's own reset SETUP are all dead - AND THE
// HARDWARE WATCHDOG IS UNFED: usbIRQ and the 1 s watchdog tick sit at the same
// PICO_DEFAULT_IRQ_PRIORITY (0x80), so the tick cannot preempt us, and the main loop's direct feed
// refuses itself outside thread mode. delay() in handler mode is tight_loop_contents().
// It fires on a bad-CRC or refused data-phase block pipelined behind an unread ack, and on ANY
// pipelined SINK_BULK - which is exactly the parked bench's own B2b/B5b drill at depth 2.
//
// So: the five reply sites STASH here and return; forgeLinkBulkPoll(), on the main loop, sends it.
// The command-block slot is NOT re-armed until that reply has gone, which is what keeps this one
// deep by construction rather than by hope.
static struct {
    bool     valid;
    uint8_t  type;
    uint8_t  seq;
    uint8_t  len;
    uint8_t  body[8];
} g_pending_reply;

static void fl_bulk_queue_reply(uint8_t type, uint8_t seq, const uint8_t *body, uint8_t len)
{
    if (len > sizeof(g_pending_reply.body)) len = sizeof(g_pending_reply.body);
    if (g_pending_reply.valid)
    {
        // Cannot happen while the CB slot is only re-armed after a reply goes out. Counted rather
        // than overwritten, because losing a reply silently is how a host learns to distrust a link.
        g_bulk.rt.bad_frames++;
        return;
    }
    g_pending_reply.type = type;
    g_pending_reply.seq  = seq;
    g_pending_reply.len  = len;
    if (len && body) memcpy(g_pending_reply.body, body, len);
    g_pending_reply.valid = true;
    g_bulk.cb_answered = true;   // ARMRETRY r9: every stashed reply answers the block in g_bulk.cb
}

/*****************************************************************************
 * arming - every one checked, every refusal counted, never fatal (frame C14) *
 *****************************************************************************/

// ~~static bool fl_bulk_arm(uint8_t ep, uint8_t *buf, uint32_t len)~~ REMOVED: it
// checked busy, armed, and counted a refusal - and every caller then treated a refusal as "retry
// later", which is the defect (see THE OUT TICKET). Its one rule lives on in fl_bulk_arm_out:
// ⛔ ONE OWNER PER ENDPOINT, AND BUSY IS CHECKED BEFORE EVERY ARM. A double-arm is a refused
// TU_ASSERT inside TinyUSB, not a wedge - but only if we look. EVERY OUT arm goes through
// fl_bulk_arm_out below, because an OUT arm has a meaning (command block, data phase, sink) and
// the completion must be able to prove which one it is finishing. (fl_bulk_tx arms IN itself.)

static void fl_bulk_drop_parked(void);

/*****************************************************************************
 * ⭐ THE OUT TICKET - (ARMRETRY), the owner's "Retry safety fix" 2026-10-18 *
 *****************************************************************************
 *
 * ⛔ WHAT A REFUSED OUT ARM MEANS - READ FROM THE STACK, NOT GUESSED. pico-sdk TinyUSB's
 * usbd_edpt_xfer refuses in exactly one way: TU_ASSERT(busy == 0) (device/usbd.c:1337).
 * dcd_edpt_xfer (dcd_rp2040.c:515-518) always returns true. `busy` is set by an arm and by
 * usbd_edpt_stall, and cleared ONLY by tud_task immediately BEFORE it calls our xfer_cb
 * (usbd.c:648), by usbd_edpt_clear_stall, and by a bus reset. So, outside HALTED (the one state
 * that stalls OUT), a refusal means one thing: OUT IS STILL ARMED INTO SOME OTHER BUFFER. That
 * transfer is not cancelled by the refusal; it goes on swallowing the host's bytes, and its
 * completion WILL be dispatched to fl_bulkd_xfer_cb - always before any retry could see busy == 0.
 *
 * ⛔ THE OLD RETRY TURNED THAT INTO SILENCE, AND c7fc531c TURNED IT INTO WRONG DATA (sent back,
 * handback 2026-10-16-c329-verify). The old retry re-armed a refused DATA arm as a COMMAND BLOCK
 * (fl_bulk_arm_cb_slot set WAIT_CB first), so the data phase was parsed as a short CB: silence. The
 * "retry the recorded arm" fix let the stale transfer complete under DATA, and fl_bulk_on_data_done
 * then committed an EXEC whose data region was NEVER WRITTEN this phase: the previous request's
 * bytes, on the S1000's or the XL's SCSI bus. A silent request is recoverable; that is not.
 *
 * ⭐ SO: every OUT arm records a TICKET (kind + phase generation) with interrupts masked, and
 *   (1) an OUT completion is acted on ONLY when its ticket is the current phase's - the kind the
 *       state expects AND the current generation. Anything else is STALE: counted, never parsed,
 *       never committed, and the link is halted (below). It cannot become this phase's data,
 *       because this phase's data is only ever accepted from a transfer armed for this phase.
 *   (2) a refused arm whose ticket is EXACTLY the wanted one (same kind, same generation - e.g. the
 *       poll's re-arm after a reply meeting the CB slot CLEAR_FEATURE already armed) is a no-op:
 *       the transfer we wanted is the one in flight.
 *   (3) any OTHER refused arm is a LINK FAULT, handled exactly like the HUB1 abandoned-phase
 * watchdog (proven on metal): new epoch (queued frames dropped unexecuted), the OUT
 *       and IN transfers aborted in the controller so no late byte lands in a slot, both
 *       endpoints stalled, state HALTED. The host's request times out and its existing §3.4
 *       BULK_RESET + clear-halt recovers the link (and check_at_open resets a HALTED board).
 *       No retry exists any more: there is no correct buffer to retry INTO once the stream's
 *       byte position is unknown - the stale transfer has eaten an unknown share of the phase.
 * Never seen on metal: refused_arms / fail_in / fail_out stayed 0 in every diag run. This
 * is robustness, and the happy path is unchanged: there the ticket always matches. */
enum { FL_ARM_NONE = 0, FL_ARM_CB = 1, FL_ARM_DATA = 2, FL_ARM_SINK = 3 };

static uint8_t fl_bulk_kind_for(fl_bulk_state_t st)
{
    switch (st)
    {
        case FL_BULK_WAIT_CB: return FL_ARM_CB;
        case FL_BULK_DATA:    return FL_ARM_DATA;
        case FL_BULK_SINKING: return FL_ARM_SINK;
        default:              return FL_ARM_NONE;   // IDLE / HALTED arm nothing on OUT
    }
}

// ARMRETRY r3: cancel one endpoint's transfer in the controller; the CALLER holds the mask. True when
// it is really gone (or there was none). False when EP_ABORT_DONE never came: the transfer is left
// alive, counted, and the endpoint marked uncancelled (see abort_timeouts above).
static bool fl_bulk_cancel_locked(uint8_t ep)
{
    const bool ok = fl_ep_abort_xfer(ep);
    if (ep == g_bulk.ep_out) g_bulk.out_uncancelled = !ok;
    else                     g_bulk.in_uncancelled  = !ok;
    if (!ok) g_bulk.abort_timeouts++;
    return ok;
}

// ⛔ ARMRETRY r4: A STALL IS WRITTEN ONLY TO AN ENDPOINT WHOSE ABORT
// COMPLETED. ~~r3: "on a timeout the transfer is left alive, untouched"~~ - FALSE in r3: every caller
// stalled both endpoints straight after the cancel, and dcd_edpt_stall rewrites the endpoint's buffer
// control (_hw_endpoint_buffer_control_set_value32(ep, USB_BUF_CTRL_STALL), dcd_rp2040.c:548), which the
// RP2350 allows only once EP_ABORT_DONE is set (or the controller does not own the buffer). So an
// endpoint whose abort timed out is NOT stalled: its buffer control is not written by us at all, the
// transfer finishes or sits as the hardware has it, and the link is held HALTED by THIS driver
// (fl_bulk_arm_out refuses OUT over HALTED, fl_bulk_tx refuses IN while in_uncancelled) rather than by
// a hardware stall. The host still sees its request time out and resets; the reset retries the cancel.
// What we give up: the host is not shown a STALL on that endpoint (its OUT packets meet the live
// transfer or a NAK, its IN reads the live reply or a NAK) - never a re-arm, never a slot handed to CDC.
static void fl_bulk_stall_cancelled(bool out_gone, bool in_gone)
{
    if (out_gone) usbd_edpt_stall(0, g_bulk.ep_out);
    if (in_gone)  usbd_edpt_stall(0, g_bulk.ep_in);
}

// ⛔ ARMRETRY r6: A LATCHED BUFF_STATUS BIT
// OUTLIVES ITS TRANSFER. dcd_edpt_close_all (SET_CONFIGURATION to a new value) and the bus reset's
// reset_non_control_endpoints clear ep_ctrl and hw_endpoints but NOT usb_hw->buf_status. A packet the SIE
// took just before them leaves its bit set, and the USBCTRL IRQ shares usbIRQ's priority, so it cannot
// service that bit until tud_task returns - by which time the deferred CB arm (open, a FUNC_CALL drained
// in the same tud_task) has made the endpoint active again: sync_ep_buffer then copies the OLD packet
// from DPRAM (same address: buffers are re-allocated in the same order) under the fresh {CB, gen} ticket
// = an abandoned command block served again. With no arm in between (a lost FUNC_CALL) it is the
// "continue xfer on inactive ep" panic instead (rp2040_usb.c, now a counted drop).
// So, masked, the bits of OUR two endpoints are cleared at the reset and at open, before any arm.
// Why no packet of ours is lost: at open, usbd_edpt_open -> hw_endpoint_init has just zeroed both buffer
// controls (dcd_rp2040.c:116), so the controller owns no buffer of ours and can latch nothing new; nothing
// of ours is armed (the arm is deferred). At the bus reset every transfer is already gone. Any set bit
// therefore belongs to a transfer that no longer exists. BUFF_STATUS is write-1-to-clear ("WC",
// hardware/regs/usb.h USB_BUFF_STATUS_*_ACCESS), cleared through usb_hw_clear exactly as upstream's
// hw_handle_buff_status does. Never EP0 (an endpoint address of 0 means "not opened yet").
static void fl_bulk_clear_pending_locked(void)
{
    if (g_bulk.ep_out & 0x7F) fl_ep_clear_pending(g_bulk.ep_out);
    if (g_bulk.ep_in & 0x7F)  fl_ep_clear_pending(g_bulk.ep_in);
}

// The HUB1 halt, shared by the abandoned-phase watchdog and the study- link fault. The CALLER
// holds interrupts masked (save_and_disable_interrupts), so no completion runs half-way through.
static void fl_bulk_halt_link_locked(void)
{
    fl_bulk_end_session(false);   // ARMRETRY r8 · THE RULE: new session + new epoch (the host resets)
    g_bulk.data_remaining = 0;
    fl_bulk_drop_parked();
    flFramesClearChainBreak();
    flFramesInvalidateCache();
    g_bulk.rt.state = FL_BULK_HALTED;
    g_bulk.rt.bad_frames++;
    // The transfers themselves go first (an outside review): abort them in the controller so no
    // late packet is copied into the slot that CDC is about to own, THEN halt.
    const bool out_gone = fl_bulk_cancel_locked(g_bulk.ep_out);
    const bool in_gone = fl_bulk_cancel_locked(g_bulk.ep_in);
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    // ARMRETRY r3: the from-slot hold ends only when the IN transfer REALLY ended (was: before the abort).
    if (in_gone) flFramesNoteBulkTxDone();
#endif
    // ARMRETRY r4 (F3): only an endpoint whose abort COMPLETED is stalled (fl_bulk_stall_cancelled).
    fl_bulk_stall_cancelled(out_gone, in_gone);
    g_bulk.out_kind = FL_ARM_NONE;   // the aborted transfer carries no ticket; a queued completion is stale
    g_bulk.rearm_kind = FL_ARM_NONE; // ARMRETRY r4: a deferred re-arm dies with the link; its FUNC_CALL is a no-op
}

// Study: OUT held a transfer that is not the current phase's. Halt (above), count it; the
// poll logs it and hands the slots back to CDC (the same context as HUB1's release).
static void fl_bulk_link_fault_locked(void)
{
    fl_bulk_halt_link_locked();
    g_bulk.link_faults++;
}

// ⭐ THE ONLY OUT ARM. Sets `st` and records the ticket BEFORE the arm, all with
// interrupts masked: called from thread mode (the poll) as well as from tud_task, and tud_task must
// never see a transfer without its ticket, nor run between the busy test and the arm.
static bool fl_bulk_arm_out(uint8_t kind, fl_bulk_state_t st, uint8_t *buf, uint32_t len)
{
    if (!g_bulk.opened) return false;
    bool ok = false;
    uint32_t irq = save_and_disable_interrupts();
    if (g_bulk.rt.state == FL_BULK_HALTED)
    {
        // ⛔ NEVER ARM OUT WHILE HALTED: the stall must last until the host clears it. Only
        // CLEAR_FEATURE and the no-stall BULK_RESET leave HALTED, and they change the state first.
        // (Before the poll's post-reply re-arm reached here, set WAIT_CB over HALTED and was
        // refused by the stall's busy flag.)
    }
    else if (g_bulk.rearm_kind != FL_ARM_NONE)
    {
        // ⛔ ARMRETRY r4: A RE-ARM AFTER AN ABORT IS STILL DEFERRED (fl_bulk_defer_arm_locked). Nothing
        // arms OUT ahead of it: completions of the aborted transfer may still sit in the event FIFO,
        // and only the deferred arm is ordered behind them. A CB or SINK arm asked for meanwhile is
        // folded into it - a reset's SINK outranks a CB (the sink's ZLP arms the CB slot itself), so
        // a CB asked for over a deferred SINK changes nothing. A DATA arm cannot be asked for here (it
        // needs a command block, and every OUT completion is dropped while the deferral stands): if
        // one is, the link halts rather than arm over the order.
        if (kind == FL_ARM_SINK || kind == FL_ARM_CB)
        {
            if (kind == FL_ARM_SINK) g_bulk.rearm_kind = FL_ARM_SINK;
            ok = true;
        }
        else
        {
            flBulkRtNoteRefusedArm(&g_bulk.rt);
            fl_bulk_link_fault_locked();
        }
    }
    else if (usbd_edpt_busy(0, g_bulk.ep_out))
    {
        if (g_bulk.out_kind == kind && g_bulk.out_gen == g_bulk.phase_gen)
        {
            g_bulk.rt.state = st;   // (2) the transfer in flight IS the one wanted
            ok = true;
        }
        else
        {
            flBulkRtNoteRefusedArm(&g_bulk.rt);
            fl_bulk_link_fault_locked();   // (3) state is now HALTED
        }
    }
    else
    {
        g_bulk.rt.state = st;
        g_bulk.out_kind = kind;
        g_bulk.out_gen  = g_bulk.phase_gen;
        if (usbd_edpt_xfer(0, g_bulk.ep_out, buf, (uint16_t)len)) ok = true;
        else
        {
            // Unreachable while masked with busy == 0 (usbd.c:1337, dcd_rp2040.c:518). Kept honest.
            g_bulk.out_kind = FL_ARM_NONE;
            flBulkRtNoteRefusedArm(&g_bulk.rt);
            fl_bulk_link_fault_locked();
        }
    }
    restore_interrupts(irq);
    return ok;
}

// ⭐ ARMRETRY r10 (the r9 review's MUST): THE ONE-DEEP STASH HOLDS BY CONSTRUCTION AT EVERY CB ARM. Every
// command-slot arm, direct or deferred, is decided MASKED against usbIRQ and is SKIPPED while a reply still
// sits in the stash: the next block may need that stash for its own answer (a 0x17 / 0x18 / SINK / busy
// refusal), and fl_bulk_queue_reply could only drop it. The poll arms the slot once the reply has gone
// (fl_bulk_post_reply_arm). A skipped arm returns false; no caller retries on false.
static bool fl_bulk_arm_cb_slot(void)
{
    uint32_t irq = save_and_disable_interrupts();
    bool ok = false;
    if (!g_pending_reply.valid)
        ok = fl_bulk_arm_out(FL_ARM_CB, FL_BULK_WAIT_CB, g_cb_buf, FL_BULK_CB_BYTES);
    restore_interrupts(irq);
    return ok;
}

// A sink arm continues the CURRENT sink phase; whoever starts a new one bumps phase_gen first.
static bool fl_bulk_arm_sink(void)
{
    g_bulk.phase_ms = millis();
    uint32_t n = g_bulk.data_remaining < FL_BULK_EP_SIZE ? g_bulk.data_remaining : FL_BULK_EP_SIZE;
    return fl_bulk_arm_out(FL_ARM_SINK, FL_BULK_SINKING, g_sink_buf, n);
}

/*****************************************************************************
 * ⭐ THE DEFERRED RE-ARM - ARMRETRY r4 *
 *****************************************************************************
 *
 * ⛔ THE HOLE r3 LEFT. An OUT completion carries no ticket of its own: fl_bulkd_xfer_cb reads the
 * CURRENT {out_kind, out_gen} when the event is dispatched. TinyUSB has ONE event FIFO (usbd.c,
 * _usbd_q), and the USBCTRL IRQ posts an XFER_COMPLETE the moment the last packet's buffer is handled.
 * So when the host's last DATA packet is ACKed just AFTER it sent a bare CLEAR_FEATURE(OUT), the
 * completion sits in the FIFO BEHIND that SETUP. r3's handler then cancelled an endpoint that had
 * already finished (a no-op), replaced the ticket and armed the CB slot - and the queued completion,
 * dispatched next, read the NEW {CB, gen} ticket, passed, and fl_bulk_on_cb re-parsed g_cb_buf (whose
 * only check is the length, 64): the abandoned EXEC re-served, the host's next block taken as its
 * data-out. Glue scenario clear_behind.
 *
 * ⭐ THE FIX IS AN ORDER, NOT A GUESS. After a successful cancel the re-arm is NOT made in the handler:
 * the ticket is cleared, rearm_kind records what is owed, and usbd_defer_func posts a FUNC_CALL into
 * THE SAME FIFO (usbd.c:1273, usbd_pvt.h:128). The proof that no completion of the aborted transfer can
 * match the arm that follows:
 *   1. fl_ep_abort_xfer, with interrupts masked, makes the endpoint inactive and clears its buf_status
 *      bit, so the USBCTRL IRQ can post NO completion for that transfer after the cancel returns.
 *   2. Every completion it posted before then is already in the FIFO; the FUNC_CALL is posted AFTER
 *      the cancel, so it is queued behind every one of them (one FIFO, first in first out).
 *   3. While rearm_kind is set, no OUT transfer exists at all (fl_bulk_arm_out folds every arm into the
 *      deferral), so EVERY OUT completion dispatched meanwhile is one of those - fl_bulkd_xfer_cb drops
 *      it as stale, before reading any ticket.
 *   4. The FUNC_CALL then makes the arm with a fresh ticket. Nothing older than it is left to match it.
 * If the FIFO was full the FUNC_CALL is lost (queue_event's TU_ASSERT returns false and usbd_defer_func
 * returns void): the poll then finds rearm_kind still set and, masked, arms only once the FIFO is
 * EMPTY (tud_task_event_ready() false), which is the same proof - nothing queued is left to match.
 * A link halt, a bus reset or a newer deferral (a new token) turns an old FUNC_CALL into a no-op. */
static void fl_bulk_run_deferred_locked(void)
{
    const uint8_t kind = g_bulk.rearm_kind;
    g_bulk.rearm_kind = FL_ARM_NONE;
    // ⛔ ARMRETRY r9 (the r8 review's MUST 2, adjudicated REAL): NEVER ARM THE COMMAND SLOT WHILE A REPLY IS
    // STILL IN THE ONE-DEEP STASH. r8's CLEAR mid-DATA queued A's 0x18 and deferred the CB arm; this FUNC_CALL
    // ran in tud_task before the poll had sent the 0x18, armed the slot, and the next block B - answered
    // through the same stash (a 0x18, 0x17, QUEUE_NOT_EMPTY or SINK reply) - was dropped by
    // fl_bulk_queue_reply: B got NO reply. Now the deferral ends here and the poll arms the slot once the
    // reply has gone (its post-reply arm); this FUNC_CALL has already passed every queued completion of the
    // cancelled transfer, so that later direct arm keeps the deferral's ordering proof.
    if (kind == FL_ARM_CB && g_pending_reply.valid) return;
    if (kind == FL_ARM_CB)        fl_bulk_arm_cb_slot();
    else if (kind == FL_ARM_SINK) fl_bulk_arm_sink();
}

static void fl_bulk_deferred_arm(void *param)
{
    // tud_task context (USBD_EVENT_FUNC_CALL). Masked: the poll arms from thread mode too.
    uint32_t irq = save_and_disable_interrupts();
    if (g_bulk.rearm_kind != FL_ARM_NONE && (uint16_t)(uintptr_t)param == g_bulk.rearm_token)
        fl_bulk_run_deferred_locked();
    restore_interrupts(irq);
}

// The CALLER holds the mask, has just CANCELLED OUT successfully and taken the link out of HALTED.
static void fl_bulk_defer_arm_locked(uint8_t kind)
{
    g_bulk.out_kind = FL_ARM_NONE;
    g_bulk.rt.state = FL_BULK_IDLE;          // nothing is armed on OUT until the deferred arm runs
    g_bulk.rearm_kind = kind;
    g_bulk.rearm_token++;
    usbd_defer_func(fl_bulk_deferred_arm, (void *)(uintptr_t)g_bulk.rearm_token, false);
}

/*****************************************************************************
 * the reply carrier                                                          *
 *****************************************************************************/

// True when nothing is in flight on the vendor IN endpoint - i.e. the reply cache is no longer
// being read by a transfer. Installed beside fl_bulk_tx (N6 probe-bulk, frame item 8); asked only
// from thread mode, by flFramesWaitBulkTxIdle.
static bool fl_bulk_tx_idle(void)
{
    // ARMRETRY r5 (the r4 review's F2 follow-up): an IN transfer whose abort timed out may still be
    // reading the reply cache or a held slot, whatever TinyUSB's busy says (a CLEAR_FEATURE(IN) zeroes
    // it). Not idle, so flFramesWaitBulkTxIdle waits for it and a from-slot reply is never chosen over
    // it. ⚠ The wait is bounded (2 s, ForgeLink_frames.cpp); after it the cache IS rewritten - the abandoned
    // reply is then garbled on the wire (its CRC fails at the host). Only while in_uncancelled, i.e.
    // after an EP_ABORT_DONE timeout, which the link already holds HALTED.
    return !g_bulk.opened || (!g_bulk.in_uncancelled && !usbd_edpt_busy(0, g_bulk.ep_in));
}

static bool fl_bulk_tx(const uint8_t *data, uint32_t len)
{
    if (!g_bulk.opened || g_bulk.card_mode) return false;

    // ⛔ ARMRETRY r4: NEVER ARM IN OVER A TRANSFER WHOSE ABORT TIMED
    // OUT. A host CLEAR_FEATURE(IN) zeroes TinyUSB's busy (usbd.c:1409-1420) whatever the DCD holds, so
    // busy alone would let this arm reach hw_endpoint_xfer_start on the still-live transfer, whose
    // fl_ep_abort_before_rearm ignores its own abort timeout (rp2040_usb.c ~252-269) and re-arms over
    // it. Refused here, counted; the link is HALTED whenever in_uncancelled is set, so the host's
    // timeout drives the reset, which retries the cancel. (Both arms below - the reply and its ZLP.)
    if (g_bulk.in_uncancelled) { flBulkRtNoteRefusedArm(&g_bulk.rt); return false; }

    // One transfer from the reply cache. The cache is static and outlives the transfer, so
    // there is nothing to copy.
    // ⛔ NEVER WAIT IN HANDLER MODE. The busy flag below is cleared only by tud_task's own event
    // loop, so a spin here inside a class callback cannot end early - it just burns the main loop,
    // the SCSI service and the watchdog feed for up to 2 s. Refuse and let the main loop retry.
    if (fl_handler_context())
    {
        if (usbd_edpt_busy(0, g_bulk.ep_in)) { flBulkRtNoteRefusedArm(&g_bulk.rt); return false; }
    }

    uint32_t waited = 0;
    while (!fl_handler_context() && usbd_edpt_busy(0, g_bulk.ep_in) && waited < 2000)
    {
        // Thread mode only: this is the main loop waiting for the previous reply to drain.
        platform_reset_watchdog();
        delay(1);
        waited++;
    }
    if (usbd_edpt_busy(0, g_bulk.ep_in)) return false;
    if (!usbd_edpt_xfer(0, g_bulk.ep_in, (uint8_t *)data, (uint16_t)len))
    {
        flBulkRtNoteRefusedArm(&g_bulk.rt);
        return false;
    }

    // ⛔ A DEVICE ZLP WHEN THE REPLY IS A WHOLE NUMBER OF PACKETS. Without it the host's read
    // blocks waiting for a short packet that never comes, and every reply whose length happens
    // to be a multiple of 64 would look like a hung link.
    if ((len % FL_BULK_EP_SIZE) == 0 && len > 0)
    {
        waited = 0;
        while (!fl_handler_context() && usbd_edpt_busy(0, g_bulk.ep_in) && waited < 2000)
        {
            platform_reset_watchdog();
            delay(1);
            waited++;
        }
        if (!usbd_edpt_busy(0, g_bulk.ep_in) && !g_bulk.in_uncancelled)
            usbd_edpt_xfer(0, g_bulk.ep_in, nullptr, 0);
    }
    return true;
}

/*****************************************************************************
 * the command block                                                          *
 *****************************************************************************/

static void fl_bulk_refuse(uint8_t type, uint8_t seq, uint8_t outcome, uint32_t data_bytes)
{
    // ⛔ A REFUSED REQUEST STILL CONSUMES ITS DATA PHASE. If it did not, the host's next command
    // block would be read out of the middle of this request's payload and every command after it
    // would be garbage. Refuse, swallow, THEN reply.
    (void)type; (void)seq;
    // ⛔ AND A REFUSED BLOCK BREAKS THE EXEC CHAIN (N6 probe-bulk slice 2, frame item 11). A
    // CHAINED EXEC behind it must not run as if the refused one had: a bulk 0C fill landing one
    // chunk early in the S1000's armed residual is exactly the fault the latch exists to stop.
    flFramesNoteChainBreak();
    g_bulk.cb.type = type;
    g_bulk.cb.seq = seq;
    g_bulk.data_remaining = data_bytes;
    if (data_bytes > 0)
    {
        // the outcome is stashed and sent once the phase is drained
        g_bulk.cb.len = outcome;
        g_bulk.phase_gen++;          // a new sink phase; its arms carry the new ticket
        fl_bulk_arm_sink();          // sets SINKING and phase_ms (or HALTS the link: THE OUT TICKET)
    }
    else
    {
        uint8_t rep = outcome;
        fl_bulk_queue_reply(type, seq, &rep, 1);   // the main loop sends it, then re-arms
    }
}

static void fl_bulk_serve_cb(void);

static void fl_bulk_drop_parked(void)
{
    // A parked block belongs to the epoch a reset has just abandoned. It is not served.
    g_bulk.cb_parked = false;
}

static void fl_bulk_on_cb(uint32_t xferred)
{
    fl_bulk_cb_t cb;
    fl_cb_result_t r = flBulkParseCb(g_cb_buf, xferred, &cb);

    if (r != FL_CB_OK)
    {
        g_bulk.rt.bad_frames++;
        flFramesNoteChainBreak();    // whatever it was, a chained EXEC behind it must not run
        // We cannot know the data phase's length from an unparseable block, so there is nothing
        // to swallow: re-arm and let the host's timeout drive the reset. Counted, never silent.
        fl_bulk_arm_cb_slot();
        return;
    }

    g_bulk.rt.cb_count++;
    g_bulk.cb_answered = false;  // ARMRETRY r9: a new block, unanswered until committed or replied to
    g_bulk.last_cb_ms = millis();
    g_bulk.cb = cb;
    g_bulk.data_remaining = cb.data_bytes;

    // ⭐ THE SLOTS ARE TAKEN LAZILY - BY A COMMAND BLOCK, AND ONLY BY ONE.
    // ⛔ FIXED 2026-09-24, FOUND ON THE BOARD (inject_test-bulk-3232492): the slots used to be
    // claimed the moment the host opened this interface, and taken straight back by the poll
    // whenever CDC went quiet. So bulk owned them essentially always, the CDC drain discarded
    // every frame, and `fl_inject`'s CDC HELLO - the SAFETY handshake it deliberately sends
    // BEFORE opening this pipe (role check, SAFE MODE by name) - was never answered. Our own tool
    // could not reach the fast link. Now nothing claims but a command block, the idle release
    // hands the slots back to CDC, and they stay there until the next command block.
    //
    // ⛔ AND THE CLAIM IS NEVER MADE HERE. This callback runs in tud_task's IRQ, which can land
    // between the main loop's CDC drain reading a chunk and parsing it - the parser can be at
    // sync-0 with half a frame still in hand. The claim (flFramesClaimForBulk) is therefore made only
    // from the main-loop poll (forgeLinkBulkPoll), the same context as the drain it must not cut
    // off. The block is PARKED - g_cb_buf is not re-armed, so it cannot be overwritten - and
    // served from the poll once the slots are ours. Card mode never claims (the stand-down): its
    // refusal needs no slot, so it is served at once, as before.
    //
    // ⛔ A BLOCK THAT ARRIVES WHILE CDC IS MID-FRAME (or has a frame queued, or a run or hash in
    // flight) WAITS - IT IS NOT REFUSED. The design (N6-DESIGN §1.3, frame C6) lets the claim
    // succeed only from a standing start and names no refusal for a block that meets a busy CDC
    // side; a refusal would also have nowhere correct to go, because a reply is carried by
    // whichever side OWNS the slots, and that is CDC. CDC's own 2 s gap timer returns its parser
    // to sync-0, so the wait is bounded by CDC's activity. If the host gives up first, its
    // timeout drives the §3.4 reset, which drops the parked block (fl_bulk_drop_parked).
    if (!g_bulk.card_mode && flFramesOwner() != FL_OWNER_BULK)
    {
        g_bulk.rt.state = FL_BULK_IDLE;   // "not owning the slots" - and nothing is armed on OUT
        g_bulk.cb_parked = true;
        return;
    }
    fl_bulk_serve_cb();
}

// The rest of a command block's handling, once the slots are ours (or card mode has stood down).
// Runs from the xfer callback when bulk already owns the slots, or from the poll for a parked one.
static void fl_bulk_serve_cb(void)
{
    const fl_bulk_cb_t cb = g_bulk.cb;

    // SAFE MODE refuses everything but HELLO/HWDOG/EXIT/CARD_MODE, and the bulk driver refuses
    // the same way the CDC dispatcher does - discarding the data phase first.
    // ⛔ FIXED 2026-09-24 (M7): this gate used to refuse EVERY block, HELLO included, while this
    // very comment said otherwise - so the settling HELLO of the host's reset got the byte 0x09 and
    // the Mac could not name SAFE MODE. The four exempt types carry no data phase and go on to the
    // dispatcher, whose own gate (ForgeLink_inject.cpp) answers them exactly as over CDC.
    if (forgeLinkHwdogSafeMode() && cb.type != FL_T_HELLO && cb.type != FL_T_HWDOG && cb.type != FL_T_EXIT && cb.type != FL_T_CARD_MODE
#ifdef FL_UNIFIED
        // the union's BRIDGE role can be in SAFE MODE too, and REBOOT_UF2 is its only exit
        // (forge_link_proto sends it over bulk when bulk owns the slots). It disarms first.
        && cb.type != FL_T_REBOOT_UF2
#endif
        )
    {
        fl_bulk_refuse(cb.type, cb.seq, FL_W_SAFE_MODE, cb.data_bytes);
        return;
    }

    if (g_bulk.card_mode)
    {
        fl_bulk_refuse(cb.type, cb.seq, FL_W_BAD_REQUEST, cb.data_bytes);
        return;
    }

    // ⭐ ARMRETRY r7 · DATACRC: A GUARDED BLOCK FROM ANOTHER SESSION IS REFUSED HERE, before its
    // data phase is armed into a slot and before anything is committed: its phase goes to the discard
    // sink (fl_bulk_refuse swallows exactly data_bytes) and the host gets the one-byte 0x18. A FULL block
    // with no data phase and a data_crc32 other than 0 is refused 0x17 the same way. Judged at SERVE
    // time, so a block that parked across a reset meets the session that reset minted.
    {
        const uint8_t g = flBulkGuardCheckCb(&cb, g_bulk.session);
        if (g)
        {
            if (g == FL_DC_STALE_SESSION) g_bulk.dc_stale++;
            else                          g_bulk.dc_bad_data++;
            fl_bulk_refuse(cb.type, cb.seq, g, cb.data_bytes);
            return;
        }
    }

    if (cb.data_bytes == 0)
    {
        // No data phase: hand the block straight to the dispatcher as a frame.
        if (!flFramesBulkSlotFree())
        {
            // ⛔ PARKED, NEVER DROPPED AND NEVER REFUSED (N6 probe-bulk, frame item 7). This used
            // to fall through to the re-arm with no commit and no reply - the block vanished and
            // the host timed out into a reset (G4, the inject build's defect too). It now waits
            // exactly as a block that met a busy CDC side waits: g_cb_buf is not re-armed, so it
            // cannot be overwritten, and forgeLinkBulkPoll serves it once a slot has freed. A
            // refusal would be wrong too: the request is fine, the board is only busy.
            g_bulk.busy_parks++;
            g_bulk.rt.state = FL_BULK_IDLE;   // nothing is armed on OUT while a block is parked
            g_bulk.cb_parked = true;
            return;
        }
        // ARMRETRY r7 · DATACRC: head_len - a guarded block's trailer (checked above) is not part of
        // the frame the dispatcher reads; for every other block head_len == len.
        memcpy(flFramesBulkArmTarget(0), &g_cb_buf[cb.payload_off], cb.head_len);
        flFramesBulkCommit(cb.type, cb.seq, cb.head_len, g_bulk.cb_start_us);
        g_bulk.cb_answered = true;   // ARMRETRY r9: handed to the dispatcher, which answers it
        fl_bulk_arm_cb_slot();
        return;
    }

    // A data phase. It needs a free slot, the policy's leave (an EXEC may not declare more than
    // FORGE_LINK_MAX_DATA), and it must fit behind its header.
    const uint32_t hdr = fl_bulk_phase_hdr(cb);

    // ⭐ N6 PROBE-BULK SLICE 2 · AN EXEC WITH DATA-OUT THAT MEETS FULL SLOTS IS PARKED, like a no-data
    // block (above), rather than refused QUEUE_NOT_EMPTY. A pipelining host keeps at most rx_slots
    // EXECs in flight and tops up the moment the oldest reply is judged, which can land inside
    // the few microseconds between this board sending that reply and releasing its slot. A
    // refusal there would cost the fill a round trip AND break the chain; parking costs a NAKed
    // data phase for one main-loop pass. g_cb_buf is not re-armed, so the block cannot be
    // overwritten, and forgeLinkBulkPoll serves it once the slot frees. WRITE_BULK keeps its
    // QUEUE_NOT_EMPTY refusal: the injector's Mac side retries it after a drain, and is tested so.
#ifdef FL_BULK_NOTEAT
    // TQ2: a NOTE_AT / RELEASE meeting full slots is parked like an EXEC: a one-byte 0x08 refusal
    // would read as NOTE_AT's STALE.
    if ((cb.type == FL_T_EXEC || cb.type == FL_BULK_T_NOTE_AT || cb.type == FL_BULK_T_RELEASE) &&
        !flFramesBulkSlotFree())
#else
    if (cb.type == FL_T_EXEC && !flFramesBulkSlotFree())
#endif
    {
        g_bulk.busy_parks++;
        g_bulk.rt.state = FL_BULK_IDLE;
        g_bulk.cb_parked = true;
        return;
    }
    if (!flFramesBulkSlotFree() || !flBulkDataPhaseAllowed(cb.type, cb.data_bytes) ||
        cb.data_bytes > flFramesBulkArmCapacity(hdr))
    {
        fl_bulk_refuse(cb.type, cb.seq,
                       flFramesBulkSlotFree() ? FL_W_BAD_REQUEST : FL_W_QUEUE_NOT_EMPTY,
                       cb.data_bytes);
        return;
    }

    // ⛔ ARMED DIRECTLY INTO THE SLOT, BEHIND THE HEADER. The driver writes the header bytes
    // itself from the command block (13 for a WRITE_BULK, 15 + cdb_len for an EXEC); the data
    // lands straight after them. The slot is then byte-for-byte what the CDC parser would have
    // produced, and nothing above the seam can tell which carrier delivered it.
    memcpy(flFramesBulkArmTarget(0), &g_cb_buf[cb.payload_off], hdr);
    g_bulk.phase_ms = millis();
    // ⛔ STUDY (ARMRETRY): a NEW generation for this data phase, so only a transfer armed for
    // THIS request can ever complete it. A refused arm halts the link (THE OUT TICKET); it is never
    // retried as anything, and the slot's data region is never committed unwritten.
    g_bulk.phase_gen++;
    fl_bulk_arm_out(FL_ARM_DATA, FL_BULK_DATA, flFramesBulkArmTarget(hdr), cb.data_bytes);
}

// ARMRETRY r8 (SHOULD e): the data-CRC scan runs in tud_task; keep its last and worst duration for the soak.
static void fl_bulk_note_crc_us(uint32_t us)
{
    g_bulk.crc_us_last = us;
    if (us > g_bulk.crc_us_max) g_bulk.crc_us_max = us;
}

static void fl_bulk_on_data_done(void)
{
    const uint8_t *payload = flFramesBulkArmTarget(0);

    if (g_bulk.cb.type == FL_T_WRITE_BULK)
    {
        // ⛔ THE DATA CRC IS CHECKED BEFORE THE RUN STARTS, so a corrupted payload applies
        // NOTHING. This board is writing into the disks of a sampler that is powered on and
        // serving them; a half-applied corrupt run is the failure this check exists to prevent.
        uint32_t expected = flGet32(&g_cb_buf[g_bulk.cb.payload_off + FL_BULK_WRITE_HDR]);
        const uint32_t t0 = micros();
        const bool crc_ok = flBulkDataCrcOk(expected, flFramesBulkArmTarget(FL_BULK_WRITE_HDR), g_bulk.cb.data_bytes);
        fl_bulk_note_crc_us(micros() - t0);
        if (!crc_ok)
        {
            g_bulk.rt.bad_frames++;
            // ARMRETRY r7 · DATACRC: a GUARDED (21-byte) WRITE_BULK says BAD DATA (0x17), which its host
            // resends once; the old 17-byte block keeps BAD_REQUEST, which an older host stops on.
            if (g_bulk.cb.guard != FL_GUARD_NONE) g_bulk.dc_bad_data++;
            uint8_t rep = g_bulk.cb.guard != FL_GUARD_NONE ? (uint8_t)FL_DC_BAD_DATA : (uint8_t)FL_W_BAD_REQUEST;
            fl_bulk_queue_reply(g_bulk.cb.type, g_bulk.cb.seq, &rep, 1);
            return;
        }

        // An ATOMIC run may not start behind other queued work: it publishes without yielding.
        uint8_t mode = payload[7];
        if (flBulkAtomicMustWait(mode, flFramesQueued()))
        {
            uint8_t rep = FL_W_QUEUE_NOT_EMPTY;
            fl_bulk_queue_reply(g_bulk.cb.type, g_bulk.cb.seq, &rep, 1);
            return;
        }

        // Publish the slot as a WRITE_SECTORS frame. 13 header bytes + the sector data.
        g_bulk.cb_answered = true;   // ARMRETRY r9: handed to the run machinery, which answers it
        flFramesBulkCommit(FL_T_WRITE_SECTORS, g_bulk.cb.seq,
                           FL_BULK_WRITE_HDR + g_bulk.cb.data_bytes, g_bulk.cb_start_us);
    }
    else if (g_bulk.cb.type == FL_T_SINK_BULK)
    {
        g_bulk.rt.sunk_bytes += g_bulk.cb.data_bytes;
        uint8_t rep[5];
        rep[0] = FL_W_OK;
        flPut32(&rep[1], g_bulk.cb.data_bytes);
        fl_bulk_queue_reply(g_bulk.cb.type, g_bulk.cb.seq, rep, sizeof(rep));
    }
    else if (g_bulk.cb.type == FL_T_EXEC
#ifdef FL_BULK_NOTEAT
             || g_bulk.cb.type == FL_BULK_T_NOTE_AT || g_bulk.cb.type == FL_BULK_T_RELEASE
#endif
             )
    {
        // ⭐ ARMRETRY r7 · DATACRC: A GUARDED BLOCK'S DATA IS CHECKED HERE, BEFORE THE COMMIT. The
        // phase landed behind the head (head_len, never the trailer); its CRC-32 must be the block's
        // data_crc32 or nothing is published: a one-byte 0x17, the chain broken (a chained EXEC behind
        // it must not run as if this one had), and the slot left uncommitted for the next phase. This is
        // the guard that closes "old data under a new command block" whatever the USB timing.
        const uint32_t t0 = micros();
        const uint8_t g = flBulkGuardCheckData(&g_bulk.cb, flFramesBulkArmTarget(g_bulk.cb.head_len),
                                               g_bulk.cb.data_bytes);
        if (g_bulk.cb.guard == FL_GUARD_FULL && g_bulk.cb.data_bytes) fl_bulk_note_crc_us(micros() - t0);
        if (g)
        {
            g_bulk.dc_bad_data++;
            flFramesNoteChainBreak();
            uint8_t rep = g;
            fl_bulk_queue_reply(g_bulk.cb.type, g_bulk.cb.seq, &rep, 1);
            return;
        }
        // ⭐ N6 PROBE-BULK · Publish the slot as the EXEC frame the CDC parser would have built:
        // its 15 + cdb_len header bytes, then the data-out. The EXEC handler re-checks every
        // field (length, flags, allow-list) before the bus, exactly as it does over CDC.
        // TQ2: NOTE_AT / RELEASE the same - the head, then the entries - stamped in tud_task
        // (board_rx_us, note-at §16 N4). The handler re-checks every field.
        g_bulk.cb_answered = true;   // ARMRETRY r9: handed to the dispatcher, which answers it
        flFramesBulkCommit(g_bulk.cb.type, g_bulk.cb.seq, g_bulk.cb.head_len + g_bulk.cb.data_bytes,
                           g_bulk.cb_start_us);
    }
    else
    {
        // ⛔ ANY OTHER DATA PHASE IS REFUSED, NOT SWALLOWED. Only the three types above can carry
        // one (flBulkDataBytesFor); a fourth reaching here means the policy and this switch have
        // drifted apart, and the host is owed an answer rather than a silence and a timeout.
        g_bulk.rt.bad_frames++;
        uint8_t rep = FL_W_BAD_REQUEST;
        fl_bulk_queue_reply(g_bulk.cb.type, g_bulk.cb.seq, &rep, 1);
    }

    // A WRITE_BULK that reached the run machinery produces no reply here - the main loop's run
    // does that - so it re-arms straight away (and so does a committed EXEC: its handler replies). Anything that queued a reply waits for the poll.
    if (!g_pending_reply.valid) fl_bulk_arm_cb_slot();
}

/*****************************************************************************
 * the TinyUSB class driver                                                   *
 *****************************************************************************/

static void fl_bulkd_init(void)
{
    flBulkRtInit(&g_bulk.rt);
    // ARMRETRY r8: the first session - never 0 from here on. Init is not an END (nothing is stamped yet).
    g_bulk.salt = get_rand_32();
    g_bulk.session = flBulkSessionNext(&g_bulk.session_mints, g_bulk.salt);
    g_fl_bulk_epoch = g_bulk.rt.epoch;   // C2a: commits are stamped with it
    g_bulk.opened = false;
}

static bool fl_bulkd_deinit(void)
{
    return true;
}

static void fl_bulkd_reset(uint8_t rhport)
{
    (void)rhport;
    g_bulk.opened = false;
    g_bulk.rt.state = FL_BULK_IDLE;
    g_bulk.data_remaining = 0;
    g_bulk.out_kind = FL_ARM_NONE;   // a bus reset ends every transfer (TinyUSB clears busy)
    g_bulk.out_uncancelled = false;  // ARMRETRY r3: ... and the DCD's (reset_non_control_endpoints)
    g_bulk.in_uncancelled = false;
    g_bulk.rearm_kind = FL_ARM_NONE; // ARMRETRY r4: a deferred re-arm belongs to the session just ended
    {
        uint32_t irq = save_and_disable_interrupts();
        fl_bulk_clear_pending_locked();   // ARMRETRY r6 (H1): no stale BUFF_STATUS survives the reset
        fl_bulk_end_session(false);       // ARMRETRY r8 · THE RULE: a bus reset ends the session AND its queue
        restore_interrupts(irq);
    }
    fl_bulk_drop_parked();
    flFramesClearChainBreak();   // a break belongs to the session the reset just ended
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    // a USB reset ends every transfer; a from-slot reply's hold ends with it (and must, or
    // the release below would be refused for a transfer that will never complete).
    flFramesNoteBulkTxDone();
#endif
    if (flFramesOwner() == FL_OWNER_BULK) flFramesReleaseFromBulk();
}

static uint16_t fl_bulkd_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    // ⛔ ALL THREE FIELDS. Application drivers are offered every interface BEFORE the built-in
    // ones, so a loose match here steals CDC (the console, the log and the fallback) or MSC. The
    // core's own picotool reset interface is vendor-class too - (0xFF, 0x00, 0x01) - and is the
    // near miss this must reject.
    if (!flBulkOpenMatches(itf_desc->bInterfaceClass,
                           itf_desc->bInterfaceSubClass,
                           itf_desc->bInterfaceProtocol))
    {
        return 0;
    }

    uint16_t const drv_len = (uint16_t)(sizeof(tusb_desc_interface_t) +
                                        2 * sizeof(tusb_desc_endpoint_t));
    TU_VERIFY(max_len >= drv_len, 0);

    g_bulk.itf = itf_desc->bInterfaceNumber;

    uint8_t const *p = (uint8_t const *)itf_desc + sizeof(tusb_desc_interface_t);
    for (int i = 0; i < 2; i++)
    {
        tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
        TU_VERIFY(TUSB_DESC_ENDPOINT == ep->bDescriptorType, 0);
        TU_VERIFY(usbd_edpt_open(rhport, ep), 0);
        if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) g_bulk.ep_in = ep->bEndpointAddress;
        else                                                  g_bulk.ep_out = ep->bEndpointAddress;
        p += sizeof(tusb_desc_endpoint_t);
    }

    g_bulk.opened = true;
    g_bulk.last_cb_ms = millis();

    g_bulk.cb_parked = false;

    // ⛔ NO CLAIM AT OPEN (fixed 2026-09-24). Opening the interface is not a request; the slots
    // stay with CDC, where the Mac's safety HELLO is about to arrive, until a command block asks.
    flFramesSetBulkTx(fl_bulk_tx);
    flFramesSetBulkTxIdle(fl_bulk_tx_idle);
    // Study: the endpoints were opened just above (busy clear, nothing in flight), so start the
    // ticket clean and leave any HALTED behind - fl_bulk_arm_out never arms over HALTED.
    // ⛔ ARMRETRY r5: ~~fl_bulk_arm_cb_slot here, at once~~. A host
    // may send SET_CONFIGURATION with a NEW value while configured (2 on this one-config device is
    // accepted: the core's descriptor callback ignores the index, USB.cpp:322; usbd.c:780-803 then closes
    // every endpoint, resets every driver and re-opens us, inside that SETUP). A DATA completion the
    // USBCTRL IRQ posted before the close is still in the FIFO BEHIND that SETUP, and usbd re-maps the
    // endpoint to this driver here (ep2drv), so it would be dispatched under this fresh {CB, gen} ticket
    // and re-parse the abandoned EXEC from g_cb_buf (glue scenario reopen_behind). So the first CB arm is
    // DEFERRED too (THE DEFERRED RE-ARM): dcd_edpt_close_all disabled the endpoints first, so nothing is
    // posted after this point, and the FUNC_CALL lands behind every completion already queued. On the
    // ordinary enumeration (a bus reset, then SET_CONFIGURATION 1) the FUNC_CALL simply runs next.
    // NOT DONE, and why: refusing a configuration value other than 1. usbd.c handles SET_CONFIGURATION
    // itself and the class driver is never told the value; the descriptor callback is the core's
    // (cores/rp2040/USB.cpp), outside this tree. The deferral makes the value harmless instead.
    {
        uint32_t irq = save_and_disable_interrupts();
        g_bulk.rearm_kind = FL_ARM_NONE; // ARMRETRY r4: an older FUNC_CALL finds no matching token
        fl_bulk_clear_pending_locked();  // ARMRETRY r6 (H1): before the arm below can exist (see the helper)
        fl_bulk_end_session(false);      // ARMRETRY r8 · THE RULE: SET_CONFIGURATION (a reopen) ends both
        fl_bulk_defer_arm_locked(FL_ARM_CB);   // ticket NONE, state IDLE, CB arm behind the FIFO
        restore_interrupts(irq);
    }

    return drv_len;
}

static bool fl_bulkd_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                     tusb_control_request_t const *request)
{
    // ⛔ CLEAR_FEATURE(ENDPOINT_HALT) IS A *STANDARD* REQUEST, AND IT IS THE OTHER HALF OF THE
    // RESET. usbd.c clears the stall itself and then forwards the request to the owning class
    // driver (device/usbd.c:933-950, `invoke_class_control`), which is the ONLY hook this stack
    // gives a driver to learn its endpoint was un-halted - there is no "endpoint unhalted"
    // callback and the core does NOT re-arm on our behalf. Refusing it because it is not a CLASS
    // request, as this function first did, meant the CB slot was never re-armed after a reset
    // and the link stayed mute until the board rebooted.
    // usbd_edpt_clear_stall() has already cleared both `stalled` and `busy` by the time we are
    // called (usbd.c:1409-1420), so the arm below is accepted.
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT &&
        request->bRequest == TUSB_REQ_CLEAR_FEATURE &&
        request->wValue == TUSB_REQ_FEATURE_EDPT_HALT)
    {
        if (stage == CONTROL_STAGE_SETUP &&
            (uint8_t)(request->wIndex & 0xFF) == g_bulk.ep_out)
        {
            // ⭐ ARMRETRY r8 · THE RULE, MUST-A: EVERY OUT CLEAR_FEATURE ENDS THE SESSION - ~~not from WAIT_CB
            // or IDLE~~ (r7), where a guarded block the host still held arrived later with the old session
            // and passed. The host re-reads STATUS after a clear anyway (midi M2). The one exception is a
            // clear while HALTED: whatever halted the link (a BULK RESET, a link fault, the HUB1 watchdog)
            // already ended the session, and nothing can be published while OUT is stalled; a second epoch
            // bump there would also break the host's §3.4 check that STATUS shows exactly the reset's epoch.
            // Owed replies (verify S4: a parked block was LOST silently): the request whose data phase this
            // clear abandons, or whose block was parked, gets a one-byte 0x18 from the poll.
            const bool dc_end = g_bulk.rt.state != FL_BULK_HALTED;
            // ARMRETRY r9 (SHOULD a; an outside review 3, verify S2): owed ONLY while the request has no answer yet -
            // ~~state alone~~ also owed a request already answered (a refused one still marked SINKING, or the
            // stale cb of a no-stall reset's flush), which then got two replies.
            const bool dc_owe = dc_end && !g_bulk.cb_answered &&
                                (g_bulk.rt.state == FL_BULK_DATA || g_bulk.rt.state == FL_BULK_SINKING ||
                                 g_bulk.cb_parked);
            g_bulk.data_remaining = 0;
            fl_bulk_drop_parked();
            // Study: CLEAR_FEATURE is one of the two ways out of HALTED (fl_bulk_arm_out refuses
            // to arm over a stall). ~~Not halted: the CB arm below is a no-op if the CB slot is
            // already armed, and HALTS the link if OUT is mid-phase.~~ WRONG for this TinyUSB (ARMRETRY
            // r2, the review): usbd_edpt_clear_stall has just zeroed `busy` UNCONDITIONALLY
            // (usbd.c:1409-1420, its "only if stalled" comment is not code), so the arm below always
            // reaches the DCD: it is never a no-op and never halts here. If OUT still holds a live
            // transfer (a host clearing a halt it never had), hw_endpoint_xfer_start aborts it and
            // arms the CB slot, or - a packet already landed - keeps it, and its completion arrives
            // under this {CB, gen} ticket. The stall branch of FL_REQ_BULK_RESET and the HUB1 halt
            // abort OUT first, so after a real halt the arm here is always a clean one.
            // ⛔ ARMRETRY r3 (the r2 review's remaining door): a host clearing a halt it never had,
            // mid-DATA with a packet landed, still reached that landed-skip - the DATA transfer kept,
            // completed under the {CB, gen} ticket, and the stale g_cb_buf re-served (glue scenario
            // bare_clear). So OUT is CANCELLED here first, masked, before the ticket is replaced: the
            // arm below is then always a fresh transfer, whatever the state. If the cancel times out
            // the old transfer is still alive: no arm; the link stays (or goes) HALTED and OUT is
            // stalled again, for the host's next reset.
            // ⛔ ARMRETRY r4 (the r3 review's F1): ~~the arm below is then always a fresh transfer~~ -
            // fresh in the DCD, but a completion of the cancelled transfer QUEUED BEHIND THIS SETUP was
            // then dispatched under the new {CB, gen} ticket and passed (glue scenario clear_behind). So
            // the CB arm is DEFERRED behind every such completion (THE DEFERRED RE-ARM, above).
            // ARMRETRY r4 (F3): on a cancel timeout OUT is NOT stalled again (~~r3 re-stalled it~~): its
            // buffer control may not be written before ABORT_DONE; the link is held HALTED instead.
            // ARMRETRY r4 (F2): leaving HALTED needs IN cancelled too - an in_uncancelled IN is retried
            // here, and if it still times out the link stays HALTED (OUT, cancelled, is stalled again).
            // ⚠ ARMRETRY r5 (the r4 review's G2) - A HAZARD WE DO NOT CLOSE. Before this callback runs,
            // usbd.c:937 has already called the UPSTREAM dcd_edpt_clear_stall (dcd_rp2040.c:537-546, not
            // overridden here): next_pid = DATA0 and a read-modify-write of the endpoint's buffer control,
            // with no abort - also on an endpoint whose abort timed out, where the RP2350 does not promise
            // that write is safe. What holds: no arm, no commit and no slot handover follow (HALTED, the
            // in_uncancelled gate, the deferral). What does not: that endpoint's data toggle and buffer
            // state may be garbled until a later cancel or bus reset - a wedge or a garbled abandoned reply,
            // never a wrong write. Closing it needs an override of dcd_rp2040.c or a link --wrap of
            // dcd_edpt_clear_stall, neither of them bounded enough for this round.
            uint32_t irq = save_and_disable_interrupts();
            if (dc_end) fl_bulk_end_session(true);   // ARMRETRY r8 · THE RULE (owed: the host is still there)
            if (fl_bulk_cancel_locked(g_bulk.ep_out))
            {
                g_bulk.out_kind = FL_ARM_NONE;   // the cancelled transfer's ticket dies with it
                bool in_ok = true;
                if (g_bulk.in_uncancelled)
                {
                    in_ok = fl_bulk_cancel_locked(g_bulk.ep_in);
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
                    if (in_ok) flFramesNoteBulkTxDone();   // the live from-slot reply really ended now
#endif
                }
                if (in_ok)
                {
                    fl_bulk_defer_arm_locked(FL_ARM_CB);   // leaves HALTED (state IDLE until the arm)
                }
                else
                {
                    // in_uncancelled implies HALTED (every failed cancel holds the link HALTED).
                    g_bulk.rt.state = FL_BULK_HALTED;
                    g_bulk.rearm_kind = FL_ARM_NONE;
                    usbd_edpt_stall(rhport, g_bulk.ep_out);   // safe: OUT's abort completed
                }
            }
            else
            {
                g_bulk.rearm_kind = FL_ARM_NONE;          // never arm over the live transfer
                if (g_bulk.rt.state != FL_BULK_HALTED) fl_bulk_link_fault_locked();
            }
            restore_interrupts(irq);
            if (dc_owe)
            {
                g_bulk.dc_stale++;
                uint8_t rep = FL_DC_STALE_SESSION;
                fl_bulk_queue_reply(g_bulk.cb.type, g_bulk.cb.seq, &rep, 1);   // sent by the poll
            }
        }
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
        // the host cleared a halt on the IN endpoint - any transfer on it is gone, so the
        // slot a from-slot reply was leaving from is no longer being read.
        // ARMRETRY r3: ~~any transfer on it is gone~~ only once we have CANCELLED it: a clear-halt
        // the host sends with no halt in place leaves a multi-packet from-slot reply still reading
        // the slot. The hold ends only if the abort really completed.
        if (stage == CONTROL_STAGE_SETUP &&
            (uint8_t)(request->wIndex & 0xFF) == g_bulk.ep_in)
        {
            // ARMRETRY r4 (the r3 review's F2): a cancel that times out HOLDS THE LINK HALTED (was: the
            // flag set, the state left as it was, and the next reply re-armed IN over the live transfer
            // because this clear had zeroed busy). the reply carrier now refuses IN while in_uncancelled.
            uint32_t irq = save_and_disable_interrupts();
            const bool in_gone = fl_bulk_cancel_locked(g_bulk.ep_in);
            if (!in_gone && g_bulk.rt.state != FL_BULK_HALTED) fl_bulk_link_fault_locked();
            restore_interrupts(irq);
            if (in_gone) flFramesNoteBulkTxDone();
        }
#endif
        return true;   // the core ACKs a standard request whatever we return
    }

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return false;
    if (request->wIndex != g_bulk.itf) return false;
    if (stage != CONTROL_STAGE_SETUP) return true;

    if (!flBulkControlAllowed(request->bRequest, request->wValue)) return false;  // STALL

    switch (request->bRequest)
    {
        case FL_REQ_BULK_RESET:
        {
            bool no_stall = (request->wValue & FL_BULK_NOSTALL_BIT) != 0;
            // HUB1 review (A): the board may have halted both endpoints ITSELF (the abandoned-phase
            // watchdog in forgeLinkBulkPoll). A host that resets with the no-stall flush never sends
            // CLEAR_FEATURE, so un-halt them here first, or its ZLP meets a stall and the sink's arm
            // is refused. On the stall branch below they are halted again on purpose.
            // ARMRETRY r4 (F3): only an endpoint whose transfer is CANCELLED is un-stalled here -
            // dcd_edpt_clear_stall rewrites buffer control too, and an uncancelled endpoint was never
            // stalled by us (fl_bulk_stall_cancelled). Its cancel is retried below, masked.
            if (g_bulk.rt.state == FL_BULK_HALTED)
            {
                if (!g_bulk.out_uncancelled) usbd_edpt_clear_stall(rhport, g_bulk.ep_out);
                if (!g_bulk.in_uncancelled)  usbd_edpt_clear_stall(rhport, g_bulk.ep_in);
            }
            // ⛔ C2a · THE NEW EPOCH IS ALSO WHAT DROPS THE QUEUE. Frames
            // already committed carry the old stamp; the main loop releases them unexecuted and
            // unanswered (flFramesNextHead), and suppresses the late reply of an EXEC that is
            // running right now (it is NOT interrupted - no SCSI-side change). One volatile store:
            // nothing here touches slot memory, and nothing here can block.
            fl_bulk_end_session(false);  // ARMRETRY r8 · THE RULE: new epoch (drops the queue) + new session
            g_bulk.data_remaining = 0;
            fl_bulk_drop_parked();
            flFramesClearChainBreak();   // the abandoned epoch's break; the settling HELLO clears the latch
            // A run already writing FINISHES; its reply is suppressed because the host has
            // abandoned the epoch it belonged to.
            flFramesInvalidateCache();
            if (no_stall)
            {
                // The fallback path for a host whose libusb cannot clear a halt correctly: arm
                // into the discard sink and wait for a zero-length packet as the flush marker.
                g_bulk.data_remaining = FL_BULK_EP_SIZE;
                g_bulk.phase_ms = millis();
                g_bulk.phase_gen++;      // the flush is a new sink phase
                // ⛔ STUDY: OUT IS USUALLY STILL ARMED HERE - the idle CB slot. A CB or sink
                // transfer is a <= 64-byte transfer into a driver-private buffer that SINKING never
                // reads, i.e. it already IS a sink: re-label its ticket instead of arming (which
                // TinyUSB would refuse). Before that refusal was counted and the CB transfer
                // caught the ZLP anyway. A DATA transfer (into a slot) is not harmless: the arm below
                // is refused and HALTS the link, and this host's next reset (which un-halts a
                // HALTED board first, above) arms the sink cleanly.
                uint32_t irq = save_and_disable_interrupts();
                const bool relabel = g_bulk.rt.state != FL_BULK_HALTED &&
                                     usbd_edpt_busy(rhport, g_bulk.ep_out) &&
                                     (g_bulk.out_kind == FL_ARM_CB || g_bulk.out_kind == FL_ARM_SINK);
                bool arm_now = !relabel;
                if (relabel)
                {
                    g_bulk.out_kind = FL_ARM_SINK;
                    g_bulk.out_gen  = g_bulk.phase_gen;
                    g_bulk.rt.state = FL_BULK_SINKING;
                }
                else if (g_bulk.rt.state == FL_BULK_HALTED)
                {
                    // ARMRETRY r3: an OUT transfer whose abort timed out is still alive: cancel it
                    // again before leaving HALTED. ARMRETRY r4: IN too (F2: leaving HALTED needs BOTH
                    // cancelled); if either still times out, stay HALTED with nothing armed and NO stall
                    // written (F3; and this host cannot clear a halt anyway). Its next reset retries.
                    // ARMRETRY r4 (F1): leaving HALTED here follows a cancel, so the sink arm is DEFERRED
                    // behind any completion of the cancelled transfer (THE DEFERRED RE-ARM).
                    bool out_ok = true, in_ok = true;
                    if (g_bulk.out_uncancelled)
                    {
                        out_ok = fl_bulk_cancel_locked(g_bulk.ep_out);
                        if (out_ok) usbd_edpt_clear_stall(rhport, g_bulk.ep_out);
                    }
                    if (g_bulk.in_uncancelled)
                    {
                        in_ok = fl_bulk_cancel_locked(g_bulk.ep_in);
                        if (in_ok) usbd_edpt_clear_stall(rhport, g_bulk.ep_in);
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
                        if (in_ok) flFramesNoteBulkTxDone();
#endif
                    }
                    if (out_ok && in_ok) fl_bulk_defer_arm_locked(FL_ARM_SINK);
                    arm_now = false;
                }
                else if (g_bulk.rearm_kind != FL_ARM_NONE)
                {
                    // ARMRETRY r4: a CLEAR_FEATURE's CB re-arm is still deferred; it becomes the sink
                    // (fl_bulk_arm_out folds it), still ordered behind the cancelled transfer's completions.
                    g_bulk.rearm_kind = FL_ARM_SINK;
                    arm_now = false;
                }
                restore_interrupts(irq);
                if (arm_now) fl_bulk_arm_sink();
            }
            else
            {
                // the epoch's IN transfer is abandoned (IN is stalled on this branch). NOT on the
                // no_stall branch above: there IN stays armed, and the hold must last until that stale
                // transfer completes or fails (a review, optional 1). ARMRETRY r3: the hold now
                // ends inside the masked block below, after IN's abort, and only if it completed.
                // ⛔ ARMRETRY r2: ABORT BOTH TRANSFERS BEFORE THE STALL (the HUB1 recipe).
                // dcd_edpt_stall only writes STALL into the buffer control: the endpoint stays
                // `active` with user_buf aimed at the abandoned transfer's buffer (a frame slot, if
                // a DATA phase was armed). At the host's CLEAR_FEATURE our CB arm then reaches
                // hw_endpoint_xfer_start on an ACTIVE endpoint, and if a packet has landed there
                // (rp2040_usb.c fl_ep_abort_before_rearm) the DCD keeps the OLD transfer and skips
                // the new one, while usbd_edpt_xfer still returns true: the old transfer's
                // completion arrives under the new {CB, gen} ticket, passes the ticket check, and
                // fl_bulk_on_cb re-parses the stale g_cb_buf - an abandoned EXEC re-served, its
                // data-out taken from the host's NEXT request (glue scenario landed_skip). Aborted,
                // the endpoint is inactive, a landed packet is dropped, and CLEAR_FEATURE's arm is
                // always a clean one. A completion tud_task had already queued finds HALTED and no
                // ticket, and is dropped as stale.
                // ARMRETRY r4: a deferred re-arm dies here (HALTED until the host's CLEAR_FEATURE, which
                // defers its own), and (F3) only an endpoint whose abort completed is stalled.
                uint32_t irq = save_and_disable_interrupts();
                g_bulk.rt.state = FL_BULK_HALTED;
                g_bulk.out_kind = FL_ARM_NONE;
                g_bulk.rearm_kind = FL_ARM_NONE;
                const bool out_gone = fl_bulk_cancel_locked(g_bulk.ep_out);
                const bool in_gone = fl_bulk_cancel_locked(g_bulk.ep_in);
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
                if (in_gone) flFramesNoteBulkTxDone();
#endif
                fl_bulk_stall_cancelled(out_gone, in_gone);
                restore_interrupts(irq);
            }
            return tud_control_status(rhport, request);
        }

        case FL_REQ_STATUS:
        {
            static uint8_t st[FL_REQ_STATUS_BYTES];
            memset(st, 0, sizeof(st));
            st[0] = (uint8_t)(g_bulk.rt.epoch & 0xFF);
            st[1] = (uint8_t)(g_bulk.rt.epoch >> 8);
            st[2] = (uint8_t)g_bulk.rt.state;
            st[3] = forgeLinkInjectBusy() ? 1 : 0;
            st[4] = (uint8_t)flFramesQueued();
            st[5] = flFramesOwner();
            flPut32(&st[8], forgeLinkInjectWriteGen());
            st[12] = g_bulk.cb.seq;
            // ⭐ ADDITIVE, AT THE END (the N6 probe-bulk review): blocks parked because every
            // slot was full. It was counted and never reported, so a bench could not tell a
            // pipelined fill that met full slots from one that never did.
            flPut32(&st[16], g_bulk.busy_parks);
            // ⭐ ARMRETRY r7 · DATACRC, ADDITIVE, AT THE END: the current session at 20..23. A host
            // asking 20 is cut to its wLength and never sees it; a host asking 24 of an older board gets
            // 20 back and reads no session (and that board shows no caps:data-crc either).
            flPut32(&st[20], g_bulk.session);
            return tud_control_xfer(rhport, request, st, sizeof(st));
        }

        case FL_REQ_REBOOT:
            // ⛔ DISARM FIRST, deliberate=true. A deliberate reboot must not count toward the
            // three-consecutive-watchdog-boots rule that drops the board into SAFE MODE.
            forgeLinkHwdogDisarm(true);
            tud_control_status(rhport, request);
            platform_reset_mcu(10);   // ms, as every other deliberate reboot in this tree
            return true;

        case FL_REQ_BOOTSEL:
            // ⛔ DISARM FIRST, deliberate=true. Same rule.
            // ~~platform_boot_to_main_firmware()~~ FIXED 2026-10-12 (P_Q2 first metal run, the VST's
            // install door): that call is a SYSRESETREQ straight back into the MAIN APPLICATION, so
            // "reboot into the USB bootloader" (block-interface §1d) rebooted the board as itself and
            // no INFO_UF2.TXT drive ever appeared. Now: ack, then enter the ROM's USB bootloader from
            // thread mode (forgeLinkBulkPoll) once the status stage has had time to leave - a class
            // callback runs inside tud_task's IRQ, the wrong place to tear USB down.
            forgeLinkHwdogDisarm(true);
            tud_control_status(rhport, request);
            g_fl_bootsel_at_ms = millis() + FL_BOOTSEL_DELAY_MS;
            g_fl_bootsel_pending = true;
            return true;

        default:
            return false;
    }
}

static bool fl_bulkd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                             uint32_t xferred_bytes)
{
    (void)rhport;

    if (ep_addr == g_bulk.ep_in)
    {
        // ⛔ STUDY (ARMRETRY): AN IN COMPLETION NEVER TOUCHES OUT. A failed IN transfer used to
        // fall into the shared failure branch and re-arm the CB slot - over a live DATA phase that
        // meant WAIT_CB over a data transfer (refused), the data then parsed out of g_cb_buf as the
        // PREVIOUS command block, served again, and the next block swallowed as its data-out.
        // OUT is armed (or parked, or waiting on a pending reply) independently of IN, always.
        if (result != XFER_RESULT_SUCCESS)
        {
            // Not fatal and not silent: a reply was lost, the host's timeout drives the reset.
            g_bulk.rt.bad_frames++;
            flFramesNoteChainBreak();
        }
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
        // a reply went out (or failed - either way it no longer reads the held slot). If it
        // left from an rx slot, the slot is free again: a plain volatile store (tud_task, the 1 ms
        // alarm IRQ - no wait, no flash-only call).
        flFramesNoteBulkTxDone();
#endif
        return true;
    }

    // ⛔ ARMRETRY r4 (THE DEFERRED RE-ARM): while a re-arm after an abort is deferred, no OUT transfer
    // exists, so this completion is one the cancelled transfer posted before the cancel - queued ahead
    // of the FUNC_CALL that will arm. Dropped before any ticket is read: it can never meet the new one.
    if (g_bulk.rearm_kind != FL_ARM_NONE)
    {
        g_bulk.rt.stale_completions++;
        return true;
    }

    // ⭐ STUDY · THE TICKET OF THE OUT TRANSFER THAT JUST ENDED. TinyUSB holds one transfer per
    // endpoint, so this completion finishes the last arm that succeeded - whose ticket is this.
    const uint8_t  done_kind = g_bulk.out_kind;
    const uint16_t done_gen  = g_bulk.out_gen;
    g_bulk.out_kind = FL_ARM_NONE;

    if (g_bulk.rt.state == FL_BULK_HALTED)
    {
        // ⛔ STALLED AND WAITING FOR CLEAR-HALT. A completion arriving here was already in the
        // event FIFO when the stall landed; acting on it would clear `busy` and let the
        // default arm below UN-STALL the OUT endpoint before the host has cleared it. Count
        // it and do nothing - the re-arm belongs to CLEAR_FEATURE. This state IS the epoch
        // marker: TinyUSB's xfer_cb carries no epoch, so it is the only thing that can tell a
        // pre-reset completion from a fresh one, and it is what finally moves the counter
        // flBulkRtAcceptCompletion promised. (Study: a FAILED one too - the old failure
        // branch re-armed here, after tud_task had cleared busy, un-stalling the endpoint.)
        g_bulk.rt.stale_completions++;
        return true;
    }

    if (done_kind == FL_ARM_NONE || done_kind != fl_bulk_kind_for(g_bulk.rt.state) ||
        done_gen != g_bulk.phase_gen)
    {
        // ⛔ STUDY: NOT THIS PHASE'S TRANSFER. Its bytes went into another phase's buffer (or
        // into this phase's slot from an older request), so they are neither this phase's data nor
        // its command block, and the stream's byte position is now unknown. Never parsed, never
        // committed: count it and halt the link (THE OUT TICKET, case 1). Unreachable while every
        // arm goes through fl_bulk_arm_out, which halts first; this is the proof at the point of use.
        g_bulk.rt.stale_completions++;
        uint32_t irq = save_and_disable_interrupts();
        fl_bulk_link_fault_locked();
        restore_interrupts(irq);
        return true;
    }

    if (result != XFER_RESULT_SUCCESS)
    {
        // THIS phase's own OUT transfer failed. Not fatal and not silent; as before, re-arm the CB
        // slot and let the host's timeout drive the reset. The failed transfer has ended (its
        // ticket was just cleared), so the arm is a fresh one, not a retry over a live transfer.
        g_bulk.rt.bad_frames++;
        flFramesNoteChainBreak();
        // A parked block still lives in g_cb_buf; re-arming would let the next one overwrite it.
        if (!g_bulk.cb_parked) fl_bulk_arm_cb_slot();
        return true;
    }

    switch (g_bulk.rt.state)
    {
        case FL_BULK_WAIT_CB:
            g_bulk.cb_start_us = micros();
            fl_bulk_on_cb(xferred_bytes);
            break;

        case FL_BULK_DATA:
            if (xferred_bytes < g_bulk.data_remaining)
            {
                // A short packet mid-phase: the host gave up. Resync rather than guess.
                g_bulk.rt.bad_frames++;
                flFramesNoteChainBreak();
                fl_bulk_arm_cb_slot();
                break;
            }
            fl_bulk_on_data_done();
            break;

        case FL_BULK_SINKING:
            if (g_bulk.data_remaining > xferred_bytes) g_bulk.data_remaining -= xferred_bytes;
            else                                       g_bulk.data_remaining = 0;

            if (xferred_bytes == 0)
            {
                // The ZLP flush marker of a no-stall reset.
                g_bulk.data_remaining = 0;
                fl_bulk_arm_cb_slot();
                break;
            }
            if (g_bulk.data_remaining > 0) { fl_bulk_arm_sink(); break; }

            // Phase drained: now the refusal can be answered.
            {
                uint8_t rep = (uint8_t)g_bulk.cb.len;
                fl_bulk_queue_reply(g_bulk.cb.type, g_bulk.cb.seq, &rep, 1);
            }
            break;

        // FL_BULK_HALTED is handled above, before the ticket test.
        default:
            // Defence only (a review of 2b973e2): no path is known to reach here while a
            // command block is parked, but re-arming over a parked block would overwrite it.
            if (!g_bulk.cb_parked) fl_bulk_arm_cb_slot();
            break;
    }
    return true;
}

static usbd_class_driver_t const fl_bulk_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "FLBULK",
#endif
    .init            = fl_bulkd_init,
    .deinit          = fl_bulkd_deinit,
    .reset           = fl_bulkd_reset,
    .open            = fl_bulkd_open,
    .control_xfer_cb = fl_bulkd_control_xfer_cb,
    .xfer_cb         = fl_bulkd_xfer_cb,
    .sof             = NULL
};

// ⛔ THE STRONG DEFINITION OF TINYUSB'S WEAK HOOK. Verified unclaimed in THIS build: libpico.a
// carries it as `w` (a weak UNDEFINED reference, nm), and the core's only definition
// (cores/rp2040/USB.cpp:742, the picotool reset interface) sits inside #ifdef ENABLE_PICOTOOL_USB,
// which is an Arduino-IDE upload-method option this project does not set - the symbol is absent
// from the base ELF while USBClass::registerInterface is present, which is the proof. If anyone
// ever does define ENABLE_PICOTOOL_USB, this becomes a DUPLICATE SYMBOL and the link fails
// loudly, which is the correct outcome.
extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &fl_bulk_driver;
}

/*****************************************************************************
 * registration and the main loop                                             *
 *****************************************************************************/

void forgeLinkBulkInit(void)
{
    if (g_bulk.registered) return;

    flBulkRtInit(&g_bulk.rt);

    // ⛔ THE DISCONNECT IS NOT OPTIONAL AND IT IS NOT A 500 ms TAX - WITHOUT IT THIS INTERFACE
    // NEVER ENUMERATES AT ALL. USBClass::setupUSBDescriptor() early-returns while a descriptor
    // blob already exists (cores/rp2040/USB.cpp:333-335), and by the time we run there always is
    // one: main() calls USB.begin() (which builds it) and then Serial.begin() BEFORE setup(),
    // and platform_late_init() - our caller - runs inside setup(). Registering without forcing a
    // rebuild only mutates the in-memory interface list; the bytes the host is served never
    // change, so the interface is invisible for the whole life of the boot. disconnect() frees
    // the blob (USB.cpp:466-468) and connect() rebuilds it, which is exactly why both
    // SerialUSB::begin() and platform_start_msc() wrap their own registration this way.
    //
    // ⚠ THIS CORRECTS THE FRAME. C14 inferred we could register "early, before enumeration, so
    // USB.disconnect() skips its 500 ms sleep". There is no such moment available to a platform
    // hook on this core.
    USB.disconnect();

    uint8_t ep_in  = USB.registerEndpointIn();
    uint8_t ep_out = USB.registerEndpointOut();
    uint8_t str    = USB.registerString(FL_BULK_ITF_STRING);

    uint32_t n = flBulkDescriptor(g_bulk.desc, sizeof(g_bulk.desc),
                                  0 /* patched by simpleInterface */, ep_out, ep_in, str);
    if (n != FL_BULK_DESC_BYTES)
    {
        logmsg("ForgeLink bulk: descriptor build failed, interface NOT registered");
        USB.connect();      // never leave the device detached on an error path
        return;
    }

    // ⛔ HAND-WRITTEN, NEVER TUD_VENDOR_DESCRIPTOR: that macro hard-codes subclass and protocol
    // to 0, which is exactly the tuple open() must NOT match.
    // Ordering 3 puts us after CDC and MSC so neither is renumbered.
    g_bulk.usb_id = USB.registerInterface(1, USBClass::simpleInterface,
                                          g_bulk.desc, sizeof(g_bulk.desc), 3, 0);
    USB.connect();
    g_bulk.registered = true;
    logmsg("ForgeLink bulk: vendor interface registered (FF/46/4C), EP OUT ", (int)ep_out,
           " IN ", (int)ep_in);
}

void forgeLinkBulkPoll(void)
{
    if (!g_bulk.registered) return;

    // FL_REQ_BOOTSEL, deferred from its control callback: into the ROM's USB bootloader (the same
    // call as the REBOOT_UF2 frame, src/ForgeLink_probe.cpp). Disarmed again here: the guard
    // (test_fl_hwdog) wants the disarm next to every reboot call, and a second disarm is harmless.
    if (g_fl_bootsel_pending && (int32_t)(millis() - g_fl_bootsel_at_ms) >= 0)
    {
        logmsg("Forge Link: rebooting into UF2 bootloader on a vendor BOOTSEL request");
        platform_flush_usb_log();
        forgeLinkHwdogDisarm(true);
        reset_usb_boot(0, 0);
    }

    // ⭐ THE REPLY GOES OUT HERE, IN THREAD MODE, AND NOWHERE ELSE. Every refusal and every ack
    // built inside a class callback is stashed rather than sent, because flFramesSendReply can
    // block for seconds and a class callback runs inside tud_task's IRQ. Re-arming the command
    // block slot is deferred until the reply has gone, which is what keeps the stash one deep.
    if (g_pending_reply.valid)
    {
        // C2a: the driver's own refusal answers no queued request, so it is never judged stale by
        // whatever request the dispatcher last took (a WRITE run may still be holding that one).
        const uint16_t owed = g_fl_reply_epoch;
        g_fl_reply_epoch = 0;
        flFramesSendReply(g_pending_reply.type, g_pending_reply.seq, 0, 0,
                          g_pending_reply.body, g_pending_reply.len, nullptr, 0);
        g_fl_reply_epoch = owed;
        // ⛔ ARMRETRY r10 (the r9 review's MUST): ~~g_pending_reply.valid = false; if (g_bulk.opened)
        // fl_bulk_arm_cb_slot();~~ with interrupts ON. usbIRQ could run between the two lines: an OUT CLEAR's
        // deferred arm saw the stash empty and armed the slot, block B landed and stashed its refusal, and
        // this poll then armed the slot AGAIN - over B's stashed reply - so block C's answer was dropped
        // (glue r10_irq_window). Now the stash is emptied and the arm decided in ONE masked section, and the
        // arm is made only if nothing is stashed (fl_bulk_arm_cb_slot), no block is parked in g_cb_buf, and
        // nothing is already armed on OUT. An OUT transfer that is already live (a CB the deferred arm made, a
        // DATA phase, or a no-stall reset's SINK - the r9 SHOULD rev_x, which used to halt the link here)
        // arms the slot itself when it ends. flFramesSendReply stays outside the mask: it can block.
        uint32_t irq = save_and_disable_interrupts();
        g_pending_reply.valid = false;
        if (g_bulk.opened && !g_bulk.cb_parked && !usbd_edpt_busy(0, g_bulk.ep_out)) fl_bulk_arm_cb_slot();
        restore_interrupts(irq);
    }

    // ARMRETRY r4 (THE DEFERRED RE-ARM): a deferred arm whose FUNC_CALL never ran - the FIFO was full
    // when usbd_defer_func posted it. Armed here only once the FIFO is EMPTY, decided masked: then no
    // completion of the cancelled transfer is still queued to meet the new ticket (the same proof).
    if (g_bulk.rearm_kind != FL_ARM_NONE)
    {
        uint32_t irq = save_and_disable_interrupts();
        if (g_bulk.rearm_kind != FL_ARM_NONE && !tud_task_event_ready()) fl_bulk_run_deferred_locked();
        restore_interrupts(irq);
    }

    // ~~Retry an arm that was refused. This is the retry the design promises instead of a wedge.~~
    // ⛔ REMOVED BY STUDY (ARMRETRY): it picked its arm from the state (SINKING -> sink,
    // anything else -> CB slot), so a refused DATA arm came back as a COMMAND-BLOCK arm and the data
    // phase was lost. A refused OUT arm now either IS the transfer wanted, or halts the link
    // (THE OUT TICKET); the host's §3.4 reset is the recovery, as for HUB1. Here the poll only logs
    // a halt and hands the slots back to CDC, in thread mode, as the HUB1 branch below does.
    if (g_bulk.link_faults != g_bulk.link_faults_seen)
    {
        g_bulk.link_faults_seen = g_bulk.link_faults;
        // ARMRETRY r2 (the review's point 3): release only if the link is STILL halted, decided with
        // interrupts masked. If the main loop was blocked long enough for the host to reset and begin
        // a new phase (CLEAR_FEATURE leaves HALTED from tud_task), the slots now belong to that phase.
        // ARMRETRY r3: never while an OUT transfer that may point at a slot is still alive, and the
        // log says what really happened (flFramesReleaseFromBulk can refuse, e.g. a live from-slot hold).
        bool released = false;
        uint32_t irq = save_and_disable_interrupts();
        const bool owned = flFramesOwner() == FL_OWNER_BULK;
        if (g_bulk.rt.state == FL_BULK_HALTED && owned && !g_bulk.out_uncancelled)
            released = flFramesReleaseFromBulk();
        restore_interrupts(irq);
        logmsg("Forge Link: bulk OUT held a transfer that was not this phase's (study C349, faults ",
               (int)g_bulk.link_faults_seen, "); link halted for the host's reset, console ",
               released ? "released" : (owned ? "NOT released (the idle release retries)" : "already CDC's"));
    }
    // ARMRETRY r6 (the r5 review's SHOULD 2): the same line now fires when ANY abort counter moves, the
    // override's included (they were read nowhere, so a soak could not see them). Log only; STATUS unchanged.
    {
        static uint32_t seen_hw = 0;
        const uint32_t hw = fl_ep_abort_timeouts + fl_ep_abort_landed + fl_ep_rearm_abort_timeouts +
                            fl_ep_stale_status_drops + fl_ep_pending_cleared;
        if (g_bulk.abort_timeouts != g_bulk.abort_timeouts_seen || hw != seen_hw)
        {
            g_bulk.abort_timeouts_seen = g_bulk.abort_timeouts;
            seen_hw = hw;
            logmsg("Forge Link: bulk aborts (ARMRETRY r6): bulk timeouts ", (int)g_bulk.abort_timeouts_seen,
                   ", ep abort timeouts ", (int)fl_ep_abort_timeouts, ", landed dropped ", (int)fl_ep_abort_landed,
                   ", re-arms skipped ", (int)fl_ep_rearm_abort_timeouts, ", stale status dropped ",
                   (int)fl_ep_stale_status_drops, ", pending cleared ", (int)fl_ep_pending_cleared,
                   "; OUT ", g_bulk.out_uncancelled ? "uncancelled" : "ok",
                   ", IN ", g_bulk.in_uncancelled ? "uncancelled" : "ok");
        }
    }
    // ⭐ ARMRETRY r7 · DATACRC: a line whenever either check has refused something since the last
    // one, so a soak can count them (grep "DATACRC (ARMRETRY r7)"). Log only; STATUS carries the session.
    {
        // ARMRETRY r8: also when the worst CRC time grows, with the last and the worst (SHOULD e).
        const uint32_t dc = g_bulk.dc_stale + g_bulk.dc_bad_data;
        if (dc != g_bulk.dc_seen || g_bulk.crc_us_max != g_bulk.crc_us_max_seen)
        {
            g_bulk.dc_seen = dc;
            g_bulk.crc_us_max_seen = g_bulk.crc_us_max;
            logmsg(k_fl_dc_a, (int)g_bulk.dc_stale, k_fl_dc_b, (int)g_bulk.dc_bad_data, k_fl_dc_c,
                   (int)g_bulk.session_mints, k_fl_dc_d, (int)g_bulk.crc_us_last, k_fl_dc_e, (int)g_bulk.crc_us_max);
        }
    }

    // ⛔ HUB1 · AN ABANDONED DATA PHASE (2026-10-13). Through two USB2
    // hubs the Mac's bulk OUT died with EIO part-way through a WRITE's data phase. The transfer we
    // armed for the rest never completed, nothing timed it out, and the idle release below skips
    // DATA / SINKING, so bulk kept the slots and the CDC drain discarded every frame: the board
    // answered nothing, not even the CDC HELLO, until a replug's bus reset ran fl_bulkd_reset.
    // Now a DATA / SINKING phase that has not finished FL_BULK_DATA_STALL_MS after it BEGAN (phase_ms,
    // stamped at every entry; not last_cb_ms, which a host's no-stall reset does not move) is treated
    // exactly like a host's FL_REQ_BULK_RESET (stall branch): a new epoch, the queue dropped,
    // both endpoints halted so the host's next open clears them (check_at_open), and the slots
    // handed back to CDC at once. Masked, because the data callback runs in tud_task's IRQ and
    // may complete the phase between our test and our reset.
    if ((g_bulk.rt.state == FL_BULK_DATA || g_bulk.rt.state == FL_BULK_SINKING) &&
        (uint32_t)(millis() - g_bulk.phase_ms) > FL_BULK_DATA_STALL_MS)
    {
        bool reset = false;
        uint32_t irq = save_and_disable_interrupts();
        // Both re-tested under the mask (HUB1 review C): the phase may have finished, or a NEW one
        // begun, between the test above and here.
        if ((g_bulk.rt.state == FL_BULK_DATA || g_bulk.rt.state == FL_BULK_SINKING) &&
            (uint32_t)(millis() - g_bulk.phase_ms) > FL_BULK_DATA_STALL_MS)
        {
            // New epoch, queue dropped, both transfers aborted, both endpoints halted, HALTED.
            // (Study moved the body into fl_bulk_halt_link_locked, unchanged, to share it.)
            fl_bulk_halt_link_locked();
            reset = true;
        }
        restore_interrupts(irq);
        if (reset)
        {
            if (flFramesOwner() == FL_OWNER_BULK && !g_bulk.out_uncancelled) flFramesReleaseFromBulk();   // ARMRETRY r3
            logmsg("Forge Link: bulk data phase abandoned by the host (no progress for ",
                   (int)FL_BULK_DATA_STALL_MS, " ms); link reset, console released");
        }
    }

    // Idle release: the slots go back to CDC after a stretch with no command block, so the
    // console is never left mute because a host walked away mid-session.
    // ⛔ SILENT-X (2026-10-17): THE SILENT EXCHANGE WAS THIS TEST FIRING MID-TRAFFIC.
    // It used to read `millis() - g_bulk.last_cb_ms`, compiled as: call millis(), THEN load
    // last_cb_ms. A command block arriving in tud_task's IRQ while millis() was still running
    // stamped last_cb_ms a millisecond boundary AFTER the time millis() was about to return, so the
    // difference came out 0xFFFFFFFF, "idle for 49 days", and the slots went to CDC with a request
    // in flight; its reply never came back on bulk (the host waited 22 s). Before 2^32 us of uptime
    // (71.6 min) millis() is one UDIV and the window is a few cycles; after it, the high word is set
    // and millis is __aeabi_uldivmod, hundreds of cycles - hence "only after 72 minutes" (-).
    // Now: last_cb_ms is read BEFORE millis() (so now >= last, whatever lands in between), and the
    // whole decision is re-taken with interrupts masked, as the HUB1 stall branch above does, so a
    // block that arrives after a genuine 2 s gap cannot be released out from under its own data
    // phase either. The masked re-test runs only when the cheap one already says "idle".
    if (flFramesOwner() == FL_OWNER_BULK &&
        g_bulk.rt.state != FL_BULK_DATA && g_bulk.rt.state != FL_BULK_SINKING)
    {
        const uint32_t last = g_bulk.last_cb_ms;    // volatile: loaded before the clock is read
        if ((uint32_t)(millis() - last) > FL_BULK_IDLE_RELEASE_MS)
        {
            uint32_t irq = save_and_disable_interrupts();
            if (flFramesOwner() == FL_OWNER_BULK &&
                g_bulk.rt.state != FL_BULK_DATA && g_bulk.rt.state != FL_BULK_SINKING &&
                !g_bulk.cb_parked && !g_bulk.out_uncancelled &&   /* ARMRETRY r3 */
                (uint32_t)(millis() - g_bulk.last_cb_ms) > FL_BULK_IDLE_RELEASE_MS)
            {
                flFramesReleaseFromBulk();
            }
            restore_interrupts(irq);
        }
    }

    // ⛔ NO AUTO-RECLAIM (fixed 2026-09-24). This used to take the slots straight back "when the
    // CDC side goes quiet", which made the release above meaningless and CDC permanently mute.
    // The ONLY claim is here, and only for a command block that is waiting for one.
    if (g_bulk.cb_parked)
    {
        if (!g_bulk.opened)
        {
            fl_bulk_drop_parked();
        }
        else if (g_bulk.card_mode || flFramesOwner() == FL_OWNER_BULK ||
                 flFramesClaimForBulk(forgeLinkInjectBusy()))
        {
            // Serve it with tud_task held off: usbIRQ skips its tick while USB.mutex is taken
            // (cores/rp2040/USB.cpp:431-437), so a reset cannot land between the re-check of
            // `cb_parked` and the arm. Thread mode, never handler mode, so the mutex is ours.
            CoreMutex m(&USB.mutex);
            if (m && g_bulk.cb_parked)
            {
                g_bulk.cb_parked = false;
                g_bulk.last_cb_ms = millis();   // the idle clock starts at the claim
                fl_bulk_serve_cb();
            }
        }
        // else: CDC is mid-frame, has a frame queued, or a run or hash is in flight. The block
        // waits and the claim is retried on the next pass. See fl_bulk_on_cb.
    }
}

void forgeLinkBulkCardMode(bool entering)
{
    g_bulk.card_mode = entering;
    if (entering && flFramesOwner() == FL_OWNER_BULK) flFramesReleaseFromBulk();
}

bool forgeLinkBulkActive(void)
{
    return g_bulk.opened && flFramesOwner() == FL_OWNER_BULK;
}

#endif // FORGE_LINK_BULK
