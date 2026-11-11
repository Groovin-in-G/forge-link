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

// THE RP2350 HARDWARE WATCHDOG, INJECTION BUILDS ONLY (N6 step 2; review row R-k).
//
// Why it exists: the 09-19/20 wedges were a panic -> BKPT -> HardFault -> show_hardfault, which
// saves zuluerr.txt and then blinks FOREVER WITH INTERRUPTS MASKED. The upstream 30 s
// watchdog is an alarm interrupt, so it never fired, and each recovery was a physical unplug from
// inside the owner's S1000. Only a hardware watchdog turns that halt into a reboot.
//
// The rules are in ForgeLink_hwdog_policy.c (host-tested). This header is the glue's surface:
// hooks the platform layer calls, and the wire constants for the two frames it answers.
//
// Every hook is an empty inline in any build without FORGE_LINK_INJECT, so the probe, target7 and
// stock builds do not change.

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ---- wire constants (additive; the N6 design reserves 0x27/0x28, FEAT 0x08 and W 0x08) --------
#define FL_T_HWDOG          0x29  // {op u8 ['S' status ['R' = then reset the loop gap] | 'C' + "CLR!" clear the counter]}
#define FL_T_WDOG_TEST      0x2E  // ⛔ TEST BUILDS ONLY: {"HALT", mode u8} - halt the board on purpose
#define FL_FEAT_HWDOG       0x10  // HELLO bytes 9..12 carry the watchdog state (below)
#define FL_W_SAFE_MODE      0x09  // refused: K consecutive watchdog boots, the injector is disabled

// HELLO bytes 9..12, present when FL_FEAT_HWDOG is set:
//   [9]  flags: bit0 armed now, bit1 THIS boot followed a watchdog reset, bit2 SAFE MODE,
//               bit3 the record was fresh (power-on or a new build)
//   [10] consecutive watchdog boots
//   [11] K (the count that means safe mode)
//   [12] the timeout, in seconds
#define FL_HWDOG_HF_ARMED   0x01
#define FL_HWDOG_HF_WDBOOT  0x02
#define FL_HWDOG_HF_SAFE    0x04
#define FL_HWDOG_HF_FRESH   0x08

// WDOG_TEST modes. Each is a different road to the same dead main loop.
#define FL_WDOG_HALT_IRQ_OFF  0x00  // interrupts masked, spin: the W49 signature without a crash log
#define FL_WDOG_HALT_BKPT     0x01  // BKPT -> HardFault -> show_hardfault -> zuluerr.txt: the W49 chain
// ⚠ IRQ_ON NO LONGER REACHES THE HARDWARE WATCHDOG, and that is the POINT of the "truly frozen
// only" ruling. The 1 s alarm tick is alive in this mode and feeds the hardware counter, so the 8 s
// timer never expires. What resets the board is upstream's own ladder: the bus reset at ~15 s (which
// a spinning main loop never acts on) and the software reboot at ~30 s, which disarms first and
// therefore COUNTS. Come-back window at the bench is ~35 s, not 20 s. (Was documented as "the 8 s
// hardware beats the 15 s software", which was true before the alarm feed and is now false.)
#define FL_WDOG_HALT_IRQ_ON   0x02  // spin with interrupts ON: upstream's 30 s software reboot wins

#ifdef __cplusplus
extern "C" {
#endif

#ifdef FORGE_LINK_INJECT

// Top of zuluscsi_setup(): read and classify the record, then put the hardware in a KNOWN state
// (disabled) before anything slow runs - SD init, the ini, card-reader mode.
void forgeLinkHwdogEarlyBoot(void);
// From the injection poll (the main loop): arm once, and clear the counter after a healthy run.
void forgeLinkHwdogPoll(bool card_mode);
// platform_reset_watchdog(): the main loop's feed. Refused unless thread mode on core 0.
void forgeLinkHwdogKick(void);
// watchdog_callback(): upstream's 1 s software-watchdog alarm tick. THE ONE PERMITTED IRQ-CONTEXT
// FEED - the owner's "truly frozen only" ruling, 2026-09-21. Refused unless in handler mode AND
// ARMED. Its consequence: a wedged main loop with live interrupts is handled by upstream's own
// ladder (15 s bus reset, 30 s reboot), and the hardware watchdog fires only on a real freeze.
void forgeLinkHwdogAlarmFeed(void);
// show_hardfault() entry: one feed so the emergency log is saved, then never again.
void forgeLinkHwdogCrashFeed(void);
// Before every reboot. deliberate=true: it does not count. false (the software watchdog's own
// crash reboot): the hardware is disabled so the bootloader is not cut short, but it counts.
void forgeLinkHwdogDisarm(bool deliberate);
// Card-reader mode brackets.
void forgeLinkHwdogCardMode(bool entering);
// True for the whole run once K consecutive watchdog boots have been seen.
bool forgeLinkHwdogSafeMode(void);
// HELLO bytes 9..12.
void forgeLinkHwdogHelloBytes(uint8_t out[4]);
// The FL_T_HWDOG and FL_T_WDOG_TEST frames.
void forgeLinkHwdogFrame(uint8_t type, uint8_t seq, uint32_t req_crc, uint32_t req_len,
                         const uint8_t *p, uint32_t len);
void forgeLinkHwdogBanner(void);

#else

static inline void forgeLinkHwdogEarlyBoot(void) {}
static inline void forgeLinkHwdogKick(void) {}
static inline void forgeLinkHwdogAlarmFeed(void) {}
static inline void forgeLinkHwdogCrashFeed(void) {}
static inline void forgeLinkHwdogDisarm(bool deliberate) { (void)deliberate; }
// N6 probe-bulk slice 1: the vendor-bulk driver (ForgeLink_bulk.cpp) asks this on every command
// block, and the PROBE build now compiles that driver. A build without the hardware watchdog
// has no safe mode, so the answer is a constant false - nothing about the watchdog changes.
static inline bool forgeLinkHwdogSafeMode(void) { return false; }

#endif // FORGE_LINK_INJECT

#ifdef __cplusplus
}
#endif
