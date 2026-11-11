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

// THE HARDWARE WATCHDOG'S RULES, IN PLAIN C WITH NO SDK IN IT - ON PURPOSE (N6 step 2, R-k).
//
// Every decision the watchdog makes lives here: how a boot is classified from the scratch
// register, when the counter moves, when the board may arm, when a feed is allowed, what a
// deliberate reboot does. src/ForgeLink_hwdog.cpp is only the glue that reads the registers and
// performs the answers. tools/tests/test_fl_hwdog.py compiles THIS file with the host compiler
// and drives it, so "a deliberate reboot does not count" and "K boots is a plain disk" are tests
// that run, not beliefs.
//
// ⛔ THE SAFETY DIRECTION. When in doubt, the board stays a working disk for the S1000:
//   - it never goes to BOOTSEL by itself (that takes the sampler's disks away until a human
//     flashes it) - rev 2 said BOOTSEL after two resets; the brief overrode that, and so does this;
//   - after K consecutive watchdog boots it stays a PLAIN DISK: the injector refuses its verbs and
//     the hardware watchdog is NOT armed, so it can never loop;
//   - anything it cannot account for (an unmarked reset while armed) counts TOWARD safe mode.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- the numbers -----------------------------------------------------------------------------
// 8 s: far above any legitimate feed gap (the upstream software watchdog tolerates 15 s before it
// acts, ZuluSCSI_config.h:99) and inside the RP2350 counter's ~16.7 s limit (24-bit us count,
// pico-sdk watchdog.c:49 + hardware/regs/watchdog.h WATCHDOG_LOAD_BITS 0x00ffffff).
#define FL_HWDOG_TIMEOUT_MS   8000u
// Consecutive watchdog boots that put the board in safe mode (the brief's K).
#define FL_HWDOG_K            3u
// A run this long, armed and feeding, clears the counter: two unrelated resets an hour apart do
// not add up to safe mode.
#define FL_HWDOG_HEALTHY_MS   60000u

// ---- scratch register 2 ----------------------------------------------------------------------
// Scratch 0 and 1 are upstream's (card-mode magic + COW, ZuluSCSI_platform.cpp:279-288, :847-851);
// 4-7 are the SDK's and the bootrom's (watchdog.c:84, :102-108). 2 and 3 are free, and this code is
// their only user. Layout of scratch 2:
//   bits 31..16  'FL' magic 0x464C - anything else (power-on zero, a stranger's value) is "no record"
//   bit  8       ARMED - set when the hardware watchdog is armed, cleared by every DELIBERATE
//                reboot and at every boot. Still set at boot means the previous run ENDED WHILE
//                ARMED WITHOUT SAYING SO: the watchdog fired, or something reset us we could not mark.
//   bits 7..0    consecutive watchdog boots, saturating at 255
// Scratch 3 holds the build id (a CRC-32 of the build's git rev and build time), so a record left
// by a DIFFERENT build is not held against this one: flashing a fix starts from zero.
#define FL_HWDOG_MAGIC        0x464Cu
#define FL_HWDOG_F_ARMED      0x00000100u
#define FL_HWDOG_COUNT_MASK   0x000000FFu

uint32_t flHwdogScratchPack(uint8_t count, bool armed);
// How scratch 2 changes at each event. A DELIBERATE disarm clears ARMED, so the next boot is not
// a watchdog boot; a CRASH reboot (the software watchdog's own AIRCR reset) disables the hardware
// so the bootloader is not cut short, but KEEPS ARMED, so it still counts.
uint32_t flHwdogScratchOnArm(uint32_t scratch2);
uint32_t flHwdogScratchOnDisarm(uint32_t scratch2, bool deliberate);
uint32_t flHwdogScratchOnClear(uint32_t scratch2);

// What a boot IS, decided from the two scratch words as they were at reset.
typedef struct {
    uint8_t  prev_count;     // the counter as the previous run left it (0 when there is no record)
    uint8_t  count;          // after this boot: prev + 1 on a watchdog boot, else unchanged
    bool     watchdog_boot;  // the previous run ended while armed, without a deliberate disarm
    bool     fresh;          // no usable record: power-on, first boot of a new build, foreign value
    bool     safe_mode;      // count >= K: stay a plain disk, never arm
    uint32_t scratch2_out;   // what to write back NOW (ARMED cleared, count updated)
    uint32_t scratch3_out;   // this build's id
} fl_hwdog_boot_t;

void flHwdogClassifyBoot(uint32_t scratch2, uint32_t scratch3, uint32_t build_id,
                         fl_hwdog_boot_t *out);

// ---- the run-time state machine --------------------------------------------------------------
//   OFF       not armed yet (setup, card-reader mode, or suspended). Kick does nothing.
//   ARMED     running; a kick from the main loop feeds it, and so does upstream's 1 s alarm tick
//             (the "truly frozen only" ruling - see flHwdogRtAlarmFeed).
//   CRASHED   show_hardfault fed it ONCE; nothing feeds it again. It expires with the log saved.
//   REBOOTING a deliberate reboot is under way. Nothing feeds it and nothing re-arms it: the
//             SDK's own watchdog_reboot() must be allowed to expire.
//   SAFE      K consecutive watchdog boots. Never armed this run.
typedef enum {
    FL_HWDOG_OFF = 0,
    FL_HWDOG_ARMED,
    FL_HWDOG_CRASHED,
    FL_HWDOG_REBOOTING,
    FL_HWDOG_SAFE,
} fl_hwdog_state_t;

typedef struct {
    fl_hwdog_state_t state;
    uint8_t  count;          // the live counter (mirrors scratch 2's low byte)
    uint32_t armed_at_ms;
    uint32_t feeds;          // diagnostics: feeds performed, FROM EITHER DOOR
    uint32_t refused_feeds;  // diagnostics: feeds refused (wrong context, or not armed)
    // ⛔ THE ONE NUMBER THAT DISCRIMINATES OPTION (iii), AND `feeds` IS NOT IT.
    // `feeds` counts BOTH doors, and platform_reset_watchdog() is called once per iteration of
    // zuluscsi_main_loop() (src/ZuluSCSI.cpp:2072, plus six more sites), so on ANY healthy board -
    // with the alarm feed or without it - `feeds` rises at loop rate. Watching it climb proves the
    // board is alive and says NOTHING about whether the 1 s tick is feeding. This counter is
    // incremented ONLY by flHwdogRtAlarmFeed, so it is the only evidence at the bench that the
    // ruling actually landed, short of halting the board.
    // ⚠ The tempting alternative was "read `feeds` while the main loop is idle". There is no such
    // moment: the loop is never idle on a working board. That is precisely why a shared counter
    // cannot separate the two, and it is the reasoning this field exists to replace.
    uint32_t alarm_feeds;    // diagnostics: feeds from the 1 s alarm tick ALONE
} fl_hwdog_rt_t;

void flHwdogRtInit(fl_hwdog_rt_t *rt, const fl_hwdog_boot_t *boot);

// Each returns what the glue must DO. None of them touch hardware.
bool flHwdogRtArm(fl_hwdog_rt_t *rt, uint32_t now_ms, bool card_mode);   // true: enable it now
bool flHwdogRtDisarm(fl_hwdog_rt_t *rt);                                // true: disable it now
bool flHwdogRtSuspend(fl_hwdog_rt_t *rt);                               // true: disable it now
bool flHwdogRtHealthy(fl_hwdog_rt_t *rt, uint32_t now_ms);              // true: clear the counter
bool flHwdogRtClear(fl_hwdog_rt_t *rt);                                 // host CLEAR; true: cleared

const char *flHwdogStateName(fl_hwdog_state_t s);

// ---- the main-loop gap: the instrument for "SAFE MODE loads slower" (bench R8b) ---------------
// the owner, 2026-09-24, SAFE MODE: the S1000 shows BUSY and loads "not as quick". The suspect was
// SAFE MODE's refused-feed path costing time per pass. ⛔ THE CODE SAYS OTHERWISE: per pass, SAFE
// does LESS than ARMED - flHwdogRtKick's refusal is a compare and one increment, where the armed
// path is a compare, one increment AND the WATCHDOG.LOAD store; flHwdogRtArm and flHwdogRtHealthy
// both return on their first compare in SAFE; nothing logs, locks or writes flash per pass in
// either state. The millions of refused feeds are simply the kick count - in ARMED the same kicks
// land in `feeds` instead. So this measures rather than argues: the longest gap between two
// successive main-loop passes (one forgeLinkHwdogPoll entry to the next), how many passes it
// covered and over how long, so one disk load ARMED can be set against one in SAFE MODE.
//
// A gap includes everything one pass does - a whole SCSI command inside scsiDiskPoll, a log save
// in STATUS phase, a frame dispatch - which is the point: it is what the sampler waits on.
typedef struct {
    uint32_t last_us;        // entry time of the previous pass; meaningful only when `primed`
    uint32_t max_gap_us;     // the high-water mark since the last reset
    uint32_t passes;         // gaps measured since the last reset (0 = nothing measured yet)
    uint32_t window_start_ms;// when the window was reset, for a mean pass time: window / passes
    bool     primed;         // false until the first pass after a reset: nothing to measure from
} fl_hwdog_gap_t;

// A new window. The first pass after it only primes: the pass that reset it (a status frame being
// answered) is never counted against the load being measured.
void flHwdogGapReset(fl_hwdog_gap_t *g, uint32_t now_ms);
// One main-loop pass began at now_us (a wrapping 1 MHz counter). Returns the gap it measured, or 0
// for a priming pass.
uint32_t flHwdogGapSample(fl_hwdog_gap_t *g, uint32_t now_us);

// The two feed decisions are INLINE. platform_reset_watchdog() is on the SCSI hot path and this
// project keeps that path in RAM - a .c file links to flash (rp23xx-template.ld excludes only
// *.cpp.o from flash), so an out-of-line call would be a flash call per feed. And the crash feed
// runs inside the HardFault handler, which should not depend on flash at all.
static inline bool flHwdogRtKick(fl_hwdog_rt_t *rt, bool main_context)
{
    // ⛔ THE HOOK'S feed is thread-mode-on-core-0 ONLY. A kick arriving from interrupt context or
    // from core 1 is refused, not trusted, because platform_reset_watchdog is reached from both
    // and a feed from either would be a feed with the main loop dead. The one WANTED IRQ feed has
    // its own door and its own gate: flHwdogRtAlarmFeed below. This one stays shut.
    if (rt->state != FL_HWDOG_ARMED || !main_context)
    {
        rt->refused_feeds++;
        return false;
    }
    rt->feeds++;
    return true;
}

// ⛔ THE ONE PERMITTED IRQ-CONTEXT FEED, AND IT IS A DELIBERATE EXCEPTION TO THE RULE ABOVE.
// the owner's ruling, 2026-09-21 ("truly frozen only", option iii): upstream's 1 s software-watchdog
// alarm tick ALSO feeds the hardware watchdog while it is counting. So a main loop that is merely
// WEDGED with live interrupts is left to upstream's own ladder - the 15 s bus reset and the 30 s
// reboot - and ONLY a dead-interrupt freeze is reset by the hardware. N6-DESIGN §5.3 says "feed
// from the hook and nowhere else. Never from an IRQ or an alarm"; this is the single exception the
// ruling creates, and every OTHER IRQ or alarm feed stays forbidden (the caller-set test in
// tools/tests/test_fl_hwdog.py pins it at exactly three named callers).
//
// Gated TWICE, and neither gate is the other's spare:
//   handler_mode  the alarm tick runs as the TIMER IRQ, so IPSR != 0. A thread-mode caller that
//                 wandered in here would be a second main-loop feed wearing this function's name.
//   ARMED         never while EarlyBoot, SAFE, disarmed/REBOOTING, CRASHED or card mode. In
//                 particular the crash path: show_hardfault feeds ONCE and moves to CRASHED, so
//                 this cannot extend a crash - and it blinks with interrupts masked anyway.
//
// ⚠ `feeds` / `refused_feeds` are now written from two contexts, so an increment can be LOST to a
// preempting tick. All three counters here are DIAGNOSTIC (the status frame is their only reader);
// nothing branches on any of them. `alarm_feeds` is written ONLY from here, i.e. only from the
// TIMER IRQ, so it has no cross-context writer at all and cannot lose an increment. `state`,
// `count` and `armed_at_ms` are read here and never written, so this feed cannot corrupt a
// decision. Re-derived 2026-09-22 against every reader in the tree.
static inline bool flHwdogRtAlarmFeed(fl_hwdog_rt_t *rt, bool handler_mode)
{
    if (rt->state != FL_HWDOG_ARMED || !handler_mode)
    {
        rt->refused_feeds++;
        return false;
    }
    rt->feeds++;
    rt->alarm_feeds++;
    return true;
}

// True: feed it ONCE now, so show_hardfault's emergency log (zuluerr.txt) gets a full timeout to
// reach the card - then never again, so the board resets with the evidence already saved.
static inline bool flHwdogRtCrash(fl_hwdog_rt_t *rt)
{
    if (rt->state != FL_HWDOG_ARMED) return false;
    rt->state = FL_HWDOG_CRASHED;
    return true;
}

#ifdef __cplusplus
}
#endif
