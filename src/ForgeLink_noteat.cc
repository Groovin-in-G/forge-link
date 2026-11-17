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

// TQ2 · NOTE_AT, THE GLUE - in FLASH (a .cc: rp23xx-template.ld sends only *.cpp.o to RAM; the same
// reason ForgeLink_linkguard.cc is one). Every rule is in ForgeLink_noteat_policy.c; this file only
// reads the clock and USB, runs the one 0C exchange to ID 6, and carries replies.
//
// ⛔ THREAD MODE, MAIN LOOP ONLY. Called from forgeLinkProbeMainLoop, the dispatcher and the EXEC
// handler; never from platform_poll, never from an interrupt (tools/check-irq-ram.sh).
//
// ⛔ THE FIRE PATH LOGS NOTHING (§6), on any build: no logmsg here, and the debug log is cleared for
// the exchange exactly as fl_execute clears it for a quiet EXEC.

#include "ForgeLink_noteat.h"

#if defined(FORGE_LINK_NOTE_AT) && defined(FORGE_LINK_PROBE)

#include <Arduino.h>
#include <string.h>
#include <tusb.h>
#include <hardware/timer.h>
#include <pico/rand.h>
#include "ForgeLink_noteat_policy.h"
#include "ForgeLink_probe.h"
#include "ForgeLink_frames.h"
#include "ForgeLink_linkguard.h"
#include "ForgeLink_stamps.h"    // TQ4b: FL_STAMP_FIRE - nothing at all unless -DFORGE_LINK_STAMPS
#include "ZuluSCSI_log.h"
#include "ZuluSCSI_initiator.h"
#include <ZuluSCSI_platform.h>

#ifdef PLATFORM_HAS_INITIATOR_MODE
#include <scsiHostPhy.h>
#include <scsi2sd.h>
extern "C" {
#include <scsi.h>
}
#endif

#define FL_TQ_TARGET      6u
#define FL_TQ_XCHG_MS     5000u   // one fire exchange's phase-loop timeout = the Mac's default EXEC

/*************************
 * the hooks             *
 *************************/

extern "C" uint64_t flTqHookNowUs(void) { return time_us_64(); }
extern "C" uint32_t flTqHookRandom(void) { return get_rand_32(); }
extern "C" bool     flTqHookLinkOn(void) { return forgeLinkLinkGuardIsOn(); }
extern "C" void     flTqHookKick(void)   { platform_reset_watchdog(); }

extern "C" void flTqHookUsb(bool *mounted, bool *dtr, bool *suspended, uint16_t *bulk_epoch, uint16_t *loss_gen)
{
    *mounted = tud_mounted();
    *dtr = (tud_cdc_n_get_line_state(0) & 0x01) != 0;
    // U1: the level, and the loss generation (suspends + mounts; ForgeLink_probe.cpp's callbacks)
    *suspended = tud_suspended();
    *loss_gen = (uint16_t)(g_fl_usb_suspend_gen + g_fl_usb_mount_gen);
#ifdef FORGE_LINK_BULK
    *bulk_epoch = g_fl_bulk_epoch;
#else
    *bulk_epoch = 0;
#endif
}

extern "C" void flTqHookGenerationBumped(void)
{
    // §4: a same-seq resend of a NOTE_AT after a flush must run again (and be answered CANCELLED), so a
    // cached NOTE_AT reply is dropped. Any other cached reply stays: an EXEC must never run twice.
    if (flFramesCachedReplyType() == (FL_T_NOTE_AT | FL_T_REPLY_BIT)) flFramesInvalidateCache();
}

// ONE 0C exchange to ID 6, cloned from the link guard's own 09 OFF (fl_lg_send_off) and generalised
// to a data-out: the same select (upstream's, with the initiator's own ID, ATN off, no IDENTIFY - the
// flags the Mac uses for every Akai command), the same phase loop, judged by flExecChainSuccess.
extern "C" void flTqHookExchange(const uint8_t *midi, uint32_t n, fl_tq_xres_t *r)
{
    memset(r, 0, sizeof(*r));
#ifdef PLATFORM_HAS_INITIATOR_MODE
    const uint8_t cdb[6] = { 0x0C, 0x00, (uint8_t)(n >> 16), (uint8_t)(n >> 8), (uint8_t)n, 0x00 };
    const bool dbg = g_log_debug;
    g_log_debug = false;                       // the fire path logs nothing (§6)
    uint8_t status = 0xFF, outcome = FL_O_PHASE_ERROR;
    uint32_t sent = 0;
    bool complete = false, got_status = false, sent_msg = false;
    platform_reset_watchdog();                 // TQ2 review 1: fed before every try, whatever follows
    scsiHostSetBusWidth(0);
    r->arb_us = time_us_64();
    if (!scsiHostPhySelect(FL_TQ_TARGET, (uint8_t)scsiInitiatorGetOwnID()))
    {
        scsiHostPhySetATN(false);
        scsiHostPhyRelease();
        platform_reset_watchdog();             // and after the select wait (~250 ms) ran out
        r->end_us = time_us_64();
        r->no_select = true;
        g_log_debug = dbg;
        FL_STAMP_FIRE(r->arb_us, 0, r->end_us, midi, n, FL_O_NO_SELECT);   // TQ4b, stamps builds only
        return;                                // NO_SELECT: not ok, and not retried
    }
    r->select_us = (uint32_t)(time_us_64() - r->arb_us);
    const uint32_t start = millis();
    SCSI_PHASE phase;
    while ((phase = (SCSI_PHASE)scsiHostPhyGetPhase()) != BUS_FREE)
    {
        platform_reset_watchdog();
        platform_poll();
        if (g_scsiHostPhyReset) { outcome = FL_O_BUS_RESET; break; }
        if ((uint32_t)(millis() - start) > FL_TQ_XCHG_MS) { outcome = FL_O_TIMEOUT; break; }
        if (phase == MESSAGE_IN)
        {
            uint8_t msg = 0xFF;
            scsiHostRead(&msg, 1);
            if (msg == MSG_COMMAND_COMPLETE) { complete = true; break; }
        }
        else if (phase == MESSAGE_OUT)
        {
            scsiHostPhySetATN(false);
            uint8_t msg = sent_msg ? 0x08 : 0x80;
            sent_msg = true;
            scsiHostWrite(&msg, 1);
        }
        else if (phase == COMMAND)
        {
            scsiHostWrite(cdb, sizeof(cdb));
        }
        else if (phase == DATA_OUT)
        {
            if (sent >= n) break;              // the target wants more than we have: PHASE_ERROR
            uint32_t w = scsiHostWrite(midi + sent, n - sent);
            sent += w;
            if (w == 0) { outcome = g_scsiHostPhyReset ? FL_O_BUS_RESET : FL_O_PHASE_ERROR; break; }
        }
        else if (phase == STATUS)
        {
            uint8_t st = 0xFF;
            scsiHostRead(&st, 1);
            status = st;
            got_status = true;
        }
        else if (phase == DATA_IN) { outcome = FL_O_DATA_OVERFLOW; break; }
    }
    scsiHostPhySetATN(false);
    scsiHostWaitBusFree();
    r->end_us = time_us_64();
    if (complete && got_status) outcome = FL_O_COMPLETED;
    r->ok = flExecChainSuccess(outcome, status, sent, n, 0, 0);
    r->timed_out = outcome == FL_O_TIMEOUT;    // selected, then hung: unreachable, like NO_SELECT
    g_log_debug = dbg;
    // TQ4b (§14 cell 7): the fire on the board's clock, for the DSP2 join. Stamps builds only (a bare
    // do {} while (0) in every other env); after the exchange, so outside what it records.
    FL_STAMP_FIRE(r->arb_us, r->select_us, r->end_us, midi, n, r->ok ? (uint8_t)FL_O_COMPLETED : outcome);
#else
    (void)midi; (void)n;
    r->arb_us = r->end_us = time_us_64();
#endif
}

/*************************
 * the bridge's calls    *
 *************************/

static inline bool fl_tq_bridge(void)
{
#ifdef PLATFORM_HAS_INITIATOR_MODE
    return platform_is_initiator_mode_enabled();
#else
    return false;
#endif
}

extern "C" void forgeLinkNoteAtFrame(uint8_t type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                                     const uint8_t *p, uint32_t len)
{
    // board_rx_us: the head frame's completion stamp, extended to 64 bits. On the fast link that is
    // flFramesBulkCommit in tud_task (the USB receive path, §16 N4); on CDC the parser's CRC check in
    // the drain (main loop or platform_poll). A 32-bit µs stamp is always < 71 min old, so the
    // extension is exact.
    const uint64_t now = time_us_64();
    const uint64_t rx = now - (uint32_t)((uint32_t)now - flFramesHeadDoneUs());
    // The reply is built where flFramesSendReply carries it (p2 == &frame[8] is not moved), after the
    // previous reply has left those bytes on the vendor IN endpoint.
    flFramesWaitBulkTxIdle();
    uint8_t *out = flFramesReplyPayloadAt(0);
    const uint32_t n = flTqFrame(type, p, len, rx, fl_tq_bridge(), out);
    flFramesSendReply(type, seq, req_crc, req_len, nullptr, 0, out, n);
}

extern "C" void forgeLinkNoteAtPoll(void)
{
    if (!fl_tq_bridge()) return;
    flTqPoll();
}

extern "C" void forgeLinkNoteAtServed(void) { flTqServed(); }
extern "C" void forgeLinkNoteAtHello(void)  { flTqHello(); }

extern "C" bool forgeLinkNoteAtExecHold(const uint8_t *p, uint32_t len, uint8_t *retry_le)
{
    uint64_t at = 0;
    if (!flTqExecHold(p, len, &at)) return false;
    for (int i = 0; i < 8; i++) retry_le[i] = (uint8_t)(at >> (8 * i));
    return true;
}

extern "C" void forgeLinkNoteAtExecDone(const uint8_t *p, uint32_t len, bool success)
{
    flTqExecDone(p, len, success);
}

extern "C" void forgeLinkNoteAtLinkGuardBefore(void) { flTqLinkGuardBefore(); }
extern "C" void forgeLinkNoteAtLinkGuardAfter(bool gave_up) { flTqLinkGuardAfter(gave_up); }
extern "C" bool forgeLinkNoteAtLogSaveOk(void) { return flTqLogSaveOk(); }

#endif // FORGE_LINK_NOTE_AT && FORGE_LINK_PROBE
