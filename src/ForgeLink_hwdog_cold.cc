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

// THE RP2350 HARDWARE WATCHDOG - the glue's COLD half, in FLASH (a .cc on purpose: see
// ForgeLink_hwdog_internal.h). Boot classification, arming, the healthy-run clear, the two frames
// and the banner. Nothing here is on the SCSI path or the crash path.

#ifdef FORGE_LINK_INJECT

#include <Arduino.h>
#include <string.h>

#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>
#include <hardware/timer.h>

#include "ForgeLink_hwdog.h"
#include "ForgeLink_hwdog_internal.h"
#include "ForgeLink_frames.h"
#include "ForgeLink_digest.h"
#include "ForgeLink_probe.h"
#include "ZuluSCSI_log.h"
#include <ZuluSCSI_platform.h>

#if __has_include("ZuluSCSI_build_info_generated.h")
#include "ZuluSCSI_build_info_generated.h"
#endif
#ifndef ZULU_BUILD_GIT_REV
#define ZULU_BUILD_GIT_REV "unknown"
#endif

// The build's identity: its git rev AND its build time, so two builds from one commit (inject and
// inject_test, or a rebuild) are different builds and a record left by one is not held against the
// other.
static const char g_build_marker[] = ZULU_BUILD_GIT_REV " " __DATE__ " " __TIME__;
static uint32_t g_build_id = 0;
// The registers exactly as the reset left them, before this code wrote anything - for the banner
// and the status frame, so the bench can see whether REASON survives the bootloader's hop.
static uint32_t g_at_boot_reason, g_at_boot_s2, g_at_boot_s3, g_at_boot_s4;
static bool     g_at_boot_sdk_says_wdog;
// ⭐ FIX 3 · WATCHDOG.CTRL AS THE RESET LEFT IT, read BEFORE EarlyBoot's watchdog_disable() and
// before anything else touches the peripheral. It is the only place the question "was the watchdog
// still ENABLED and counting when firmware started?" can be answered, and the SDK does not settle
// it: WATCHDOG_CTRL_ENABLE's documented RESET value is 0, but the same header gives the scratch
// registers a reset value of 0 and then says they survive a soft reset, so a reset value is not a
// post-fire state. Re-derived 2026-09-22 in the RP2350 register header; UNMEASURED on this board
// until the bench reads this number. Bit 30 is ENABLE, bits 23..0 are the live countdown (TIME).
static uint32_t g_at_boot_ctrl;
// ⭐ FIX 3 · THE HEAP FLOOR, measured on the running board rather than argued from the linker.
// N6's RAM floor is a static 34,436 B of free heap on inject_test; what the bench needs is the
// LOWEST the heap actually reached with the watchdog, the injector and a real transfer in place.
// Sampled from the main-loop poll (cold, never the IRQ path). 0xFFFFFFFF = never sampled.
static uint32_t g_heap_low_water = 0xFFFFFFFFu;
// ⭐ THE MAIN-LOOP GAP (bench R8b): the longest time between two successive passes, sampled at the
// top of this poll. Zero-initialised = not primed, window from boot. Read and reset only by the
// HWDOG status frame, which is dispatched from this same main loop - one context, no race.
static fl_hwdog_gap_t g_loop_gap;

extern "C" void forgeLinkHwdogEarlyBoot(void)
{
    if (g_fl_hwdog_booted) return;

    // ⛔ CTRL FIRST, AND BEFORE watchdog_disable() BELOW. Once that call has run the answer is
    // gone: the whole value of this number is that nothing of ours has touched the peripheral yet.
    g_at_boot_ctrl = watchdog_hw->ctrl;
    g_at_boot_reason = watchdog_hw->reason;
    g_at_boot_s2 = watchdog_hw->scratch[2];
    g_at_boot_s3 = watchdog_hw->scratch[3];
    g_at_boot_s4 = watchdog_hw->scratch[4];
    g_at_boot_sdk_says_wdog = watchdog_enable_caused_reboot();

    // ⛔ A KNOWN STATE BEFORE ANYTHING SLOW. Whether the hardware watchdog stays enabled across the
    // resets this board goes through is not measured; this makes it not matter. Nothing in setup -
    // SD init, the ini, a card-reader session that can last an hour - can be reset by it.
    watchdog_disable();

    g_build_id = flDigestCrc32(0, (const uint8_t *)g_build_marker, strlen(g_build_marker));
    flHwdogClassifyBoot(g_at_boot_s2, g_at_boot_s3, g_build_id, &g_fl_hwdog_boot);
    flHwdogRtInit(&g_fl_hwdog_rt, &g_fl_hwdog_boot);

    // Written NOW: ARMED is cleared, so a reset during setup (never armed) is not a watchdog boot,
    // and the incremented count is already on record if this boot dies too.
    watchdog_hw->scratch[2] = g_fl_hwdog_boot.scratch2_out;
    watchdog_hw->scratch[3] = g_fl_hwdog_boot.scratch3_out;
    g_fl_hwdog_booted = true;
}

extern "C" void forgeLinkHwdogPoll(bool card_mode)
{
    if (!g_fl_hwdog_booted) return;   // EarlyBoot always runs first; if it did not, stay unarmed

    // R8b: FIRST, so the timestamp marks the pass's entry and nothing below is charged to it
    // twice. One timer read, a subtract and a compare - identical in every state, SAFE included.
    // Card-reader mode is not the disk path and ends in a reboot; it only un-primes.
    if (card_mode) g_loop_gap.primed = false;
    else           flHwdogGapSample(&g_loop_gap, time_us_32());

    // Fix 3: the heap floor, sampled from the MAIN LOOP only. Two reads and a compare; nothing
    // here is on the SCSI path or the IRQ path, and the poll already runs at main-loop rate.
    uint32_t heap_now = (uint32_t)rp2040.getFreeHeap();
    if (heap_now < g_heap_low_water) g_heap_low_water = heap_now;

    uint32_t now = millis();
    if (flHwdogRtArm(&g_fl_hwdog_rt, now, card_mode))
    {
        // Record first, then enable: if it fired on the very next instruction, the record says so.
        watchdog_hw->scratch[2] = flHwdogScratchOnArm(watchdog_hw->scratch[2]);
        watchdog_enable(FL_HWDOG_TIMEOUT_MS, true);
        logmsg("Forge Link HW watchdog: ARMED, ", (int)(FL_HWDOG_TIMEOUT_MS / 1000),
               " s, consecutive watchdog boots ", (int)g_fl_hwdog_rt.count, "/", (int)FL_HWDOG_K);
        return;
    }

    if (flHwdogRtHealthy(&g_fl_hwdog_rt, now))
    {
        watchdog_hw->scratch[2] = flHwdogScratchOnClear(watchdog_hw->scratch[2]);
        logmsg("Forge Link HW watchdog: ", (int)(FL_HWDOG_HEALTHY_MS / 1000),
               " s healthy - consecutive watchdog boot count cleared");
    }
}

extern "C" void forgeLinkHwdogCardMode(bool entering)
{
    if (entering && flHwdogRtSuspend(&g_fl_hwdog_rt))
    {
        watchdog_disable();
        watchdog_hw->scratch[2] = flHwdogScratchOnDisarm(watchdog_hw->scratch[2], true);
        logmsg("Forge Link HW watchdog: suspended for card-reader mode");
    }
}

uint8_t flHwdogHelloFlags(void)
{
    uint8_t f = 0;
    if (g_fl_hwdog_rt.state == FL_HWDOG_ARMED) f |= FL_HWDOG_HF_ARMED;
    if (g_fl_hwdog_boot.watchdog_boot)         f |= FL_HWDOG_HF_WDBOOT;
    if (g_fl_hwdog_rt.state == FL_HWDOG_SAFE)  f |= FL_HWDOG_HF_SAFE;
    if (g_fl_hwdog_boot.fresh)                 f |= FL_HWDOG_HF_FRESH;
    return f;
}

extern "C" void forgeLinkHwdogHelloBytes(uint8_t out[4])
{
    out[0] = flHwdogHelloFlags();
    out[1] = g_fl_hwdog_rt.count;
    out[2] = (uint8_t)FL_HWDOG_K;
    out[3] = (uint8_t)(FL_HWDOG_TIMEOUT_MS / 1000);
}

#ifdef FORGE_LINK_INJECT_TEST
// ⛔ TEST BUILD ONLY. Halts the main loop ON PURPOSE so the bench can watch the hardware watchdog
// bring the board back. Refused unless the watchdog is ARMED, because a halt with nothing to
// recover it is a physical unplug from inside the S1000 - the very cost this exists to remove.
static void fl_wdog_test(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                         const uint8_t *p, uint32_t len)
{
    uint8_t mode = (len == 5) ? p[4] : 0xFF;
    bool ok = len == 5 && memcmp(p, "HALT", 4) == 0 && mode <= FL_WDOG_HALT_IRQ_ON;
    if (!ok || g_fl_hwdog_rt.state != FL_HWDOG_ARMED)
    {
        uint8_t rep[2] = { FL_W_BAD_REQUEST, (uint8_t)g_fl_hwdog_rt.state };
        flFramesSendReply(FL_T_WDOG_TEST, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
        return;
    }

    uint8_t rep[2] = { FL_W_OK, mode };
    flFramesSendReply(FL_T_WDOG_TEST, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
    // ⚠ WHICH TIMER WINS DEPENDS ON THE MODE, since the "truly frozen only" ruling gave the 1 s
    // alarm tick a feed. IRQ_ON leaves interrupts alive, so the hardware counter is fed for ever
    // and upstream's software reboot at ~30 s is what recovers the board. The other two modes kill
    // the tick (masked, or the crash blink loop), so the hardware watchdog fires at ~8 s.
    if (mode == FL_WDOG_HALT_IRQ_ON)
        logmsg("Forge Link WDOG_TEST (TEST BUILD): halting, mode ", (int)mode,
               " (interrupts ON) - the 1 s alarm tick keeps feeding the hardware watchdog, so"
               " UPSTREAM'S SOFTWARE REBOOT at ~30 s must recover the board, not the 8 s hardware"
               " watchdog. Bus reset logged at ~15 s. It disarms first, so it COUNTS.");
    else
        logmsg("Forge Link WDOG_TEST (TEST BUILD): halting, mode ", (int)mode,
               " (interrupts dead) - the hardware watchdog must reboot the board within ",
               (int)(FL_HWDOG_TIMEOUT_MS / 1000), " s");
    platform_flush_usb_log();
    delay(200);   // let the reply and the log line leave; this does not feed

    if (mode == FL_WDOG_HALT_BKPT)
    {
        // No debugger: the BKPT escalates to HardFault -> show_hardfault -> zuluerr.txt, the
        // exact W49 chain. The crash feed gives the save a full timeout; then it expires.
        __asm volatile ("bkpt #0x46");
    }
    if (mode == FL_WDOG_HALT_IRQ_OFF)
        __asm volatile ("cpsid i" ::: "memory");
    for (;;) __asm volatile ("nop");
}
#endif

static void fl_hwdog_status(uint8_t seq, uint32_t req_crc, uint32_t req_len, bool reset_gap)
{
    const fl_hwdog_rt_t &rt = g_fl_hwdog_rt;
    // ⭐ FIX 3 GREW THIS BY THREE WORDS, ADDITIVELY AT THE TAIL: WATCHDOG.CTRL as the reset left
    // it, the heap's low-water mark, and the heap right now. Appending means the Mac tool can read
    // a board on the older 45-byte build and simply report the new fields as absent, rather than
    // refusing a status it could mostly understand.
    uint8_t rep[73];
    memset(rep, 0, sizeof(rep));
    rep[0] = FL_W_OK;
    rep[1] = (uint8_t)rt.state;
    rep[2] = rt.count;
    rep[3] = (uint8_t)FL_HWDOG_K;
    rep[4] = flHwdogHelloFlags();
    flPut32(&rep[5],  FL_HWDOG_TIMEOUT_MS);
    flPut32(&rep[9],  g_build_id);
    flPut32(&rep[13], g_at_boot_s2);
    flPut32(&rep[17], g_at_boot_s3);
    flPut32(&rep[21], g_at_boot_reason);
    flPut32(&rep[25], g_at_boot_s4);
    flPut32(&rep[29], rt.feeds);
    flPut32(&rep[33], rt.refused_feeds);
    flPut32(&rep[37], rt.state == FL_HWDOG_ARMED ? watchdog_get_time_remaining_ms() : 0);
    flPut32(&rep[41], rt.state == FL_HWDOG_ARMED ? (uint32_t)(millis() - rt.armed_at_ms) : 0);
    flPut32(&rep[45], g_at_boot_ctrl);            // fix 3: CTRL as the reset left it
    flPut32(&rep[49], g_heap_low_water);          // fix 3: the lowest free heap seen this run
    flPut32(&rep[53], (uint32_t)rp2040.getFreeHeap());
    // ⛔ THE (iii) DISCRIMINATOR, appended at the tail for the same reason as the three above: a
    // board on an older build reports it ABSENT, never 0. `feeds` next to it counts BOTH doors and
    // rises at main-loop rate on any live board, so this is the only field that says the 1 s tick
    // is feeding.
    flPut32(&rep[57], rt.alarm_feeds);
    // ⭐ R8b, appended at the tail the same way (61 -> 73 bytes): the longest main-loop gap in us,
    // the passes it was measured over, and the window's length in ms - so a mean pass time comes
    // out as window * 1000 / passes. passes == 0 means NOTHING WAS MEASURED, never "no gap".
    flPut32(&rep[61], g_loop_gap.max_gap_us);
    flPut32(&rep[65], g_loop_gap.passes);
    flPut32(&rep[69], (uint32_t)(millis() - g_loop_gap.window_start_ms));
    flFramesSendReply(FL_T_HWDOG, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
    // 'S' 'R': read, THEN start a new window. The reply above carries the old window whole; the
    // pass answering it is never counted in the new one (the reset leaves the gap un-primed).
    // An older build ignores the second byte and simply answers the status - additive both ways.
    if (reset_gap) flHwdogGapReset(&g_loop_gap, millis());
}

extern "C" void forgeLinkHwdogFrame(uint8_t type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                                    const uint8_t *p, uint32_t len)
{
    if (type == FL_T_HWDOG)
    {
        if (len >= 1 && p[0] == 'S')
        {
            fl_hwdog_status(seq, req_crc, req_len, len >= 2 && p[1] == 'R');
            return;
        }
        if (len == 5 && p[0] == 'C' && memcmp(&p[1], "CLR!", 4) == 0)
        {
            uint8_t had = flHwdogRtClear(&g_fl_hwdog_rt) ? 1 : 0;
            watchdog_hw->scratch[2] = flHwdogScratchOnClear(watchdog_hw->scratch[2]);
            logmsg("Forge Link HW watchdog: counter CLEARED by the host (it was ",
                   had ? "non-zero" : "already zero", ")",
                   g_fl_hwdog_rt.state == FL_HWDOG_SAFE ? "; this run stays in safe mode" : "");
            uint8_t rep[2] = { FL_W_OK, had };
            flFramesSendReply(FL_T_HWDOG, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
            return;
        }
        uint8_t rep = FL_W_BAD_REQUEST;
        flFramesSendReply(FL_T_HWDOG, seq, req_crc, req_len, &rep, 1, nullptr, 0);
        return;
    }
#ifdef FORGE_LINK_INJECT_TEST
    if (type == FL_T_WDOG_TEST)
    {
        fl_wdog_test(seq, req_crc, req_len, p, len);
        return;
    }
#endif
    uint8_t err = 0xFF;   // unknown frame type, exactly as the dispatcher's default
    flFramesSendReply(type, seq, req_crc, req_len, &err, 1, nullptr, 0);
}

extern "C" void forgeLinkHwdogBanner(void)
{
    const fl_hwdog_rt_t &rt = g_fl_hwdog_rt;
    logmsg(" HW watchdog: ", (int)(FL_HWDOG_TIMEOUT_MS / 1000), " s, fed from",
           " platform_reset_watchdog on core 0 in thread mode AND from upstream's 1 s alarm tick",
           " (TRULY FROZEN ONLY: a wedged main loop with live interrupts is left to upstream's",
           " 15 s bus reset and 30 s reboot); arms on the first main-loop pass;",
           " K=", (int)FL_HWDOG_K, " consecutive watchdog boots -> plain disk.");
    if (g_fl_hwdog_boot.watchdog_boot)
    {
        // ⛔ FIX 4 · QUALIFIED, BECAUSE THE RECORD CANNOT TELL THESE APART YET. All this line
        // KNOWS is that the previous run ended with the ARMED bit still set. A hardware-watchdog
        // fire does that - and so does any reset the firmware could not mark, which includes a
        // SAME-BUILD 1200-baud reboot (reboot-uf2, restore-stock.sh) and a BOOTSEL entry without
        // a power cycle: neither can be hooked from firmware, so ARMED stays set and this counts.
        // Three of those inside the 60 s windows reach SAFE MODE from no fault at all. What would
        // discriminate them - REASON and scratch 4, printed below - is UNMEASURED across the SD
        // bootloader's SYSRESETREQ hop until the bench runs the same-build 1200-baud step.
        logmsg(" ⚠ HW WATCHDOG RESET #", (int)rt.count, " (consecutive) - the previous run ended",
               " WHILE ARMED, without a deliberate reboot. build id ", g_build_id);
        logmsg("   ⚠ NOT PROOF OF A FREEZE. A same-build 1200-baud reboot or a BOOTSEL entry with",
               " no power cycle cannot be marked from firmware and lands here identically.",
               " Compare REASON and scratch 4 below; that discriminator is UNMEASURED.");
    }
    else if (g_fl_hwdog_boot.fresh)
        logmsg(" HW watchdog: fresh record (power-on, or a new build). build id ", g_build_id);
    else
        logmsg(" HW watchdog: previous run ended deliberately; consecutive count ",
               (int)rt.count, ". build id ", g_build_id);
    logmsg(" HW watchdog at boot: REASON ", g_at_boot_reason, " scratch2 ", g_at_boot_s2,
           " scratch3 ", g_at_boot_s3, " scratch4 ", g_at_boot_s4, "; SDK says watchdog-enable",
           " reboot: ", g_at_boot_sdk_says_wdog ? "yes" : "no");
    // Fix 3: CTRL exactly as the reset left it, read before EarlyBoot disabled anything. Bit 30 is
    // ENABLE, bits 23..0 the live countdown. Whether a watchdog fire leaves ENABLE set is NOT
    // settled by the SDK headers, so this is the bench's measurement and nothing else's.
    logmsg(" HW watchdog at boot: WATCHDOG.CTRL ", g_at_boot_ctrl, " (bit30 ENABLE = ",
           (int)((g_at_boot_ctrl >> 30) & 1u), ", TIME ", (uint32_t)(g_at_boot_ctrl & 0x00FFFFFFu),
           " us) - read BEFORE our own watchdog_disable()");
    logmsg(" HW watchdog: heap free now ", rp2040.getFreeHeap(), " of ", rp2040.getTotalHeap(),
           " bytes; low-water this run ",
           g_heap_low_water == 0xFFFFFFFFu ? 0 : g_heap_low_water,
           g_heap_low_water == 0xFFFFFFFFu ? " (not sampled yet: the main loop has not polled)"
                                           : " (sampled from the main-loop poll)");
    if (rt.state == FL_HWDOG_SAFE)
    {
        logmsg(" ⛔ HW WATCHDOG SAFE MODE: ", (int)rt.count, " consecutive watchdog resets.");
        logmsg("   PLAIN DISK ONLY: the injector refuses its verbs and the watchdog is NOT armed.");
        logmsg("   To leave it: power-cycle, or `fl_hwdog.py clear` and then a deliberate reboot.");
    }
#ifdef FORGE_LINK_INJECT_TEST
    logmsg(" ⛔⛔ TEST BUILD: the WDOG_TEST (halt) verb IS COMPILED IN. It stops the board ON");
    logmsg("   PURPOSE. Refused unless the watchdog is armed. DO NOT KEEP THIS BUILD ON THE BOARD.");
#else
    logmsg(" WDOG_TEST (halt) verb: COMPILED OUT (this is not a test build).");
#endif
}

#endif // FORGE_LINK_INJECT
