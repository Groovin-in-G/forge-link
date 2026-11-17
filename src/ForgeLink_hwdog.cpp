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

// THE RP2350 HARDWARE WATCHDOG - the glue's HOT half, in RAM. Every decision is in
// ForgeLink_hwdog_policy.c; the cold half (boot, arm, frames, banner) is ForgeLink_hwdog_cold.cc,
// which lives in flash - see ForgeLink_hwdog_internal.h for why.
//
// WHERE IT IS TOUCHED, and nowhere else (tools/tests/test_fl_hwdog.py greps this list):
//   known    forgeLinkHwdogEarlyBoot()   <- top of zuluscsi_setup(): disabled until armed   [cold]
//   arm      forgeLinkHwdogPoll()        <- forgeLinkInjectPoll(), the main loop, first pass [cold]
//   feed     forgeLinkHwdogKick()        <- platform_reset_watchdog(), ZuluSCSI_platform.cpp
//            (fl_feed(), the one write to WATCHDOG.LOAD, has exactly THREE callers: Kick,
//             AlarmFeed and CrashFeed - and no fourth)
//   feed     forgeLinkHwdogAlarmFeed()   <- watchdog_callback(), ZuluSCSI_platform.cpp: upstream's
//                                           1 s software-watchdog alarm tick. THE ONE PERMITTED
//                                           IRQ-CONTEXT FEED (the "truly frozen only" ruling,
//                                           2026-09-21). Handler-mode- and ARMED-gated.
//   feed x1  forgeLinkHwdogCrashFeed()   <- show_hardfault() entry, ZuluSCSI_platform.cpp
//   disarm   forgeLinkHwdogDisarm(true)  <- platform_reset_mcu(), ZuluSCSI_platform.cpp: every
//                                           watchdog_reboot / reset_usb_boot the firmware asks for
//                                           (CARD_MODE, the console r/s/i/u+y keys, spin_for_reboot)
//            forgeLinkHwdogDisarm(false) <- the software watchdog's own AIRCR reboot (counts)
//   suspend  forgeLinkHwdogCardMode()    <- forgeLinkInjectCardMode()                    [cold]

#ifdef FORGE_LINK_INJECT

#include <Arduino.h>

#include <hardware/watchdog.h>
#include <hardware/structs/watchdog.h>
#include <pico/platform.h>

#include "ForgeLink_context.h"
#include "ForgeLink_hwdog.h"
#include "ForgeLink_hwdog_internal.h"

fl_hwdog_boot_t g_fl_hwdog_boot;
fl_hwdog_rt_t   g_fl_hwdog_rt;     // zero = FL_HWDOG_OFF: nothing feeds before EarlyBoot
bool            g_fl_hwdog_booted = false;

// Thread mode (no exception active) on core 0 - the main loop. Core 1 runs the platform's
// function-pointer dispatcher (ZuluSCSI_platform.cpp:339), and anything there, or in any interrupt,
// could go on feeding while the main loop is dead.
static inline bool fl_main_context(void)
{
    return (fl_ipsr() & 0x1FFu) == 0 && get_core_num() == 0;
}

// Handler mode: an exception is active, so IPSR carries its number. Upstream's 1 s tick is a real
// TIMER IRQ - hardware_alarm_set_callback() installs hardware_alarm_irq_handler as the exclusive
// TIMER handler and that handler calls the callback (pico-sdk hardware_timer/timer.c), so this is
// true there and false anywhere in the main loop. The CORE is deliberately NOT checked: the alarm
// is claimed and serviced on whichever core first reached platform_reset_watchdog, and an IRQ feed
// is what this door is FOR. What it must exclude is a thread-mode caller, which is exactly a
// second main-loop feed by another name.
// fl_handler_context() itself lives in ForgeLink_context.h, shared with the N6 bulk driver.

// The feed itself: what the SDK's watchdog_update() does (watchdog.c:26-28), written here so the
// SCSI hot path that calls platform_reset_watchdog() does not take a call into flash, and so the
// crash feed does not depend on flash either. RP2350 counts 1 per us (WATCHDOG_XFACTOR 1).
static_assert(FL_HWDOG_TIMEOUT_MS * 1000u <= WATCHDOG_LOAD_BITS, "timeout past the 24-bit counter");
static inline void fl_feed(void)
{
    watchdog_hw->load = FL_HWDOG_TIMEOUT_MS * 1000u;
}

extern "C" void forgeLinkHwdogKick(void)
{
    if (flHwdogRtKick(&g_fl_hwdog_rt, fl_main_context()))
        fl_feed();
}

extern "C" void forgeLinkHwdogAlarmFeed(void)
{
    if (flHwdogRtAlarmFeed(&g_fl_hwdog_rt, fl_handler_context()))
        fl_feed();
}

extern "C" void forgeLinkHwdogCrashFeed(void)
{
    if (flHwdogRtCrash(&g_fl_hwdog_rt))
        fl_feed();
}

extern "C" void forgeLinkHwdogDisarm(bool deliberate)
{
    flHwdogRtDisarm(&g_fl_hwdog_rt);
    // Unconditional: disabling a watchdog that was never enabled is harmless, and a SYSRESETREQ is
    // ASSUMED not to stop it (R-k 2), so it must be off before any reboot we ask for.
    watchdog_disable();
    watchdog_hw->scratch[2] = flHwdogScratchOnDisarm(watchdog_hw->scratch[2], deliberate);
}

extern "C" bool forgeLinkHwdogSafeMode(void)
{
    return g_fl_hwdog_rt.state == FL_HWDOG_SAFE;
}

#endif // FORGE_LINK_INJECT
