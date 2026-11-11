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

// THE LINK GUARD - the glue, in FLASH (the owner 2026-10-02: "yes").
//
// The bridge remembers the last Akai 09 it carried to ID 6. If it was LINK ON and the host then
// goes silent for 10 s, or USB goes away (unmounted, or the CDC port's DTR falls), the board sends
// 09 00 00 00 00 00 to the sampler itself, so a crashed / killed / unplugged Mac app cannot leave
// the sampler deaf to its DIN MIDI input until a power cycle. Rules: ForgeLink_linkguard_policy.c.
//
// ⛔ WHY A .cc: the linker template sends the .text AND .rodata of every *.cpp.o to RAM and leaves
// everything else in flash (rp23xx-template.ld; the same reason ForgeLink_hwdog_cold.cc is a .cc).
// The unified build's free heap was 29,596 B against the owner's 29,500 B floor when this was
// written, and .bss starts on a 256-byte boundary after .data, so even ~40 B more RAM code would
// have cost 256 B of heap. Nothing here is on the IRQ path or the hot SCSI path: it runs from the
// main loop, between host frames, when the host has gone away. ForgeLink_probe.cpp (RAM) holds
// only the two calls. tools/tests/test_fl_linkguard.py pins the extension and the ELF placement.
//
// ⛔ THREAD MODE ONLY. Both entry points are called from forgeLinkProbeMainLoop, never from
// platform_poll and never from an interrupt. USB is POLLED here (tud_mounted, the CDC line state)
// rather than hooked from a TinyUSB callback, because those callbacks run on the USB IRQ path,
// which must never touch flash (tools/check-irq-ram.sh) - and the core already owns
// tud_cdc_line_state_cb (SerialUSB.cpp). U1: the exceptions are tud_suspend_cb and
// tud_mount_cb, and they are not here: they live in RAM in ForgeLink_probe.cpp and only bump the
// loss generations this poll reads (ForgeLink_linkguard.h).

#include "ForgeLink_linkguard.h"
#include "ForgeLink_linkguard_policy.h"

#if defined(FORGE_LINK_PROBE)

#include <Arduino.h>
#include <tusb.h>
#include "ForgeLink_probe.h"
#include "ForgeLink_frames.h"
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

// 16 bytes of .bss: the whole of the guard's RAM.
static fl_link_guard_t g_fl_lg;

extern "C" void forgeLinkLinkGuardNoteExec(const uint8_t *payload, uint32_t len, bool success)
{
    const uint8_t before = g_fl_lg.on;
    const uint8_t after = flLinkGuardNoteExec(&g_fl_lg, payload, len, success, (uint8_t)g_fl_usb_mount_gen);
    if (after != before)
        logmsg("Forge Link: link guard: the host switched the sampler's link ", after ? "ON" : "OFF",
               after ? "; the board will switch it off after 10 s of host silence or a USB disconnect" : "");
}

#ifdef PLATFORM_HAS_INITIATOR_MODE
// ONE 09 OFF TO ID 6, exactly as the bridge carries the host's own (fl_execute_on_bus with the
// flags the Mac sends for every Akai command: no IDENTIFY, no NO_INIT_ID, no data, no BLIND):
// upstream's scsiHostPhySelect with the initiator's own ID, ATN off, the same phase handling and
// the same outcome codes, so flExecChainSuccess judges it exactly as it judges the host's.
static uint8_t fl_lg_send_off(uint8_t *status_out)
{
    static const uint8_t cdb[6] = FL_LINK_GUARD_CDB_OFF;
    *status_out = 0xFF;
    scsiHostSetBusWidth(0);
    if (!scsiHostPhySelect(FL_LINK_GUARD_TARGET, (uint8_t)scsiInitiatorGetOwnID()))
    {
        scsiHostPhySetATN(false);
        scsiHostPhyRelease();
        return FL_O_NO_SELECT;
    }
    bool complete = false, got_status = false, sent_msg = false;
    uint8_t outcome = FL_O_PHASE_ERROR;
    const uint32_t start = millis();
    SCSI_PHASE phase;
    while ((phase = (SCSI_PHASE)scsiHostPhyGetPhase()) != BUS_FREE)
    {
        platform_reset_watchdog();
        platform_poll();
        if (g_scsiHostPhyReset) { outcome = FL_O_BUS_RESET; break; }
        if ((uint32_t)(millis() - start) > FL_LINK_GUARD_EXEC_MS) { outcome = FL_O_TIMEOUT; break; }
        if (phase == MESSAGE_IN)
        {
            uint8_t msg = 0xFF;
            scsiHostRead(&msg, 1);
            if (msg == MSG_COMMAND_COMPLETE) { complete = true; break; }
        }
        else if (phase == MESSAGE_OUT)
        {
            // As the bridge: IDENTIFY (LUN 0, no disconnect) once, NO OPERATION after.
            scsiHostPhySetATN(false);
            uint8_t msg = sent_msg ? 0x08 : 0x80;
            sent_msg = true;
            scsiHostWrite(&msg, 1);
        }
        else if (phase == COMMAND)
        {
            scsiHostWrite(cdb, sizeof(cdb));
        }
        else if (phase == STATUS)
        {
            uint8_t st = 0xFF;
            scsiHostRead(&st, 1);
            *status_out = st;
            got_status = true;
        }
        else if (phase == DATA_IN) { outcome = FL_O_DATA_OVERFLOW; break; }   // 09 has no data
        else if (phase == DATA_OUT) { break; }                                // PHASE_ERROR
    }
    scsiHostPhySetATN(false);
    scsiHostWaitBusFree();
    if (complete && got_status) outcome = FL_O_COMPLETED;
    return outcome;
}
#endif

#ifdef FORGE_LINK_NOTE_AT
#include "ForgeLink_noteat.h"
extern "C" bool forgeLinkLinkGuardIsOn(void) { return g_fl_lg.on != 0; }
#endif

extern "C" void forgeLinkLinkGuardPoll(bool fed)
{
    const uint32_t now = millis();
    // U1: USB is read on EVERY pass, fed or not (it used to be skipped on a fed pass). The frame
    // is noted FIRST, with the suspend level: a frame served while the bus is suspended (buffered before
    // a pull) clears nothing. Then USB, so a loss seen this pass stays latched whatever the frame was -
    // and a DTR fall right after a host's last frame (a port closed with the link left ON) latches as it
    // always did. The suspend LEVEL and the mount generation: ForgeLink_linkguard.h.
    const bool suspended = tud_suspended();
    if (fed) flLinkGuardNoteHostFrame(&g_fl_lg, now, suspended);
    // Read every pass, link on or off, so the DTR edge is an edge and not a level.
    flLinkGuardNoteUsb(&g_fl_lg, tud_mounted(), (tud_cdc_n_get_line_state(0) & 0x01) != 0,
                       suspended, (uint8_t)g_fl_usb_mount_gen);
    if (fed) return;
#ifdef PLATFORM_HAS_INITIATOR_MODE
    if (!platform_is_initiator_mode_enabled()) return;
    const uint8_t why = flLinkGuardDecide(&g_fl_lg, now, flFramesPending());
    if (why == FL_LG_IDLE) return;
    uint8_t status = 0xFF;
#ifdef FORGE_LINK_NOTE_AT
    // TQ2 (§10): the timed queue is flushed and its releases sent while the link is still ON.
    forgeLinkNoteAtLinkGuardBefore();
#endif
    const uint8_t outcome = fl_lg_send_off(&status);
    const bool ok = flExecChainSuccess(outcome, status, 0, 0, 0, 0);
    const uint8_t r = flLinkGuardAfterSend(&g_fl_lg, millis(), ok);
#ifdef FORGE_LINK_NOTE_AT
    forgeLinkNoteAtLinkGuardAfter(r == FL_LG_R_GAVE_UP);
#endif
    logmsg("Forge Link: link guard: host silent ", (int)(now - g_fl_lg.last_host_ms), " ms",
           why == FL_LG_OFF_USB ? " (USB gone)" : "", ", sent 09 off -> outcome ", (int)outcome,
           " status ", (int)status,
           r == FL_LG_R_OFF ? ": the sampler's link is OFF"
               : (r == FL_LG_R_RETRY ? ": failed, again in 2 s"
                  : ": failed 5 times, GAVE UP and forgot the link - the sampler may stay deaf to MIDI"
                    " until a power cycle"),
           " (sequences ", (int)g_fl_lg.fired, ", given up ", (int)g_fl_lg.gave_up, ")");
#endif
}

#endif // FORGE_LINK_PROBE
