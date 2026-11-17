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

// The hardware watchdog's rules. See ForgeLink_hwdog_policy.h for why this file has no SDK in it.
// Compiled into the INJECTION builds only; the host test defines FL_HWDOG_HOST.

#if defined(FORGE_LINK_INJECT) || defined(FL_HWDOG_HOST)

#include "ForgeLink_hwdog_policy.h"

uint32_t flHwdogScratchPack(uint8_t count, bool armed)
{
    return ((uint32_t)FL_HWDOG_MAGIC << 16) | (armed ? FL_HWDOG_F_ARMED : 0u) | (uint32_t)count;
}

static bool fl_valid(uint32_t s2)
{
    return (s2 >> 16) == FL_HWDOG_MAGIC;
}

uint32_t flHwdogScratchOnArm(uint32_t s2)
{
    uint8_t count = fl_valid(s2) ? (uint8_t)(s2 & FL_HWDOG_COUNT_MASK) : 0;
    return flHwdogScratchPack(count, true);
}

uint32_t flHwdogScratchOnDisarm(uint32_t s2, bool deliberate)
{
    uint8_t count = fl_valid(s2) ? (uint8_t)(s2 & FL_HWDOG_COUNT_MASK) : 0;
    bool armed = fl_valid(s2) && (s2 & FL_HWDOG_F_ARMED);
    return flHwdogScratchPack(count, deliberate ? false : armed);
}

uint32_t flHwdogScratchOnClear(uint32_t s2)
{
    bool armed = fl_valid(s2) && (s2 & FL_HWDOG_F_ARMED);
    return flHwdogScratchPack(0, armed);
}

void flHwdogClassifyBoot(uint32_t s2, uint32_t s3, uint32_t build_id, fl_hwdog_boot_t *out)
{
    // ⛔ THE RECORD, NOT THE REASON REGISTER, DECIDES. The SDK's watchdog_enable_caused_reboot()
    // reads WATCHDOG.REASON, and every boot of this board passes through the SD bootloader, which
    // leaves by a SYSRESETREQ (program_flash.cpp:126-133). Whether REASON survives that second
    // reset is UNMEASURED. Our own ARMED bit is set and cleared only by us, and scratch registers
    // are MEASURED to survive the hop (upstream's card-mode magic in scratch 0 does exactly that).
    // The glue reports REASON beside this verdict so the bench can compare the two.
    bool valid = fl_valid(s2) && s3 == build_id;

    out->fresh = !valid;
    out->prev_count = valid ? (uint8_t)(s2 & FL_HWDOG_COUNT_MASK) : 0;
    out->watchdog_boot = valid && (s2 & FL_HWDOG_F_ARMED) != 0;

    uint8_t count = out->prev_count;
    if (out->watchdog_boot && count < 255) count++;
    out->count = count;
    out->safe_mode = count >= FL_HWDOG_K;
    out->scratch2_out = flHwdogScratchPack(count, false);
    out->scratch3_out = build_id;
}

void flHwdogRtInit(fl_hwdog_rt_t *rt, const fl_hwdog_boot_t *boot)
{
    rt->state = boot->safe_mode ? FL_HWDOG_SAFE : FL_HWDOG_OFF;
    rt->count = boot->count;
    rt->armed_at_ms = 0;
    rt->feeds = 0;
    rt->refused_feeds = 0;
    rt->alarm_feeds = 0;
}

bool flHwdogRtArm(fl_hwdog_rt_t *rt, uint32_t now_ms, bool card_mode)
{
    // Only from OFF. SAFE never arms (that is what stops a loop); REBOOTING never re-arms (the
    // SDK's reboot timer must expire); CRASHED is terminal; ARMED is already armed.
    if (rt->state != FL_HWDOG_OFF) return false;
    // ⛔ Never in card-reader mode: a reset there could cut the Mac off mid-write to the FAT.
    if (card_mode) return false;
    rt->state = FL_HWDOG_ARMED;
    rt->armed_at_ms = now_ms;
    return true;
}

bool flHwdogRtDisarm(fl_hwdog_rt_t *rt)
{
    // Every deliberate reboot. From ANY state: the glue disables the hardware unconditionally,
    // and the answer only says whether it had been running.
    bool was_running = rt->state == FL_HWDOG_ARMED || rt->state == FL_HWDOG_CRASHED;
    rt->state = FL_HWDOG_REBOOTING;
    return was_running;
}

bool flHwdogRtSuspend(fl_hwdog_rt_t *rt)
{
    // Card-reader mode entered while armed: back to OFF, so it re-arms after, never during.
    if (rt->state != FL_HWDOG_ARMED) return false;
    rt->state = FL_HWDOG_OFF;
    return true;
}

bool flHwdogRtHealthy(fl_hwdog_rt_t *rt, uint32_t now_ms)
{
    if (rt->state != FL_HWDOG_ARMED || rt->count == 0) return false;
    if ((uint32_t)(now_ms - rt->armed_at_ms) < FL_HWDOG_HEALTHY_MS) return false;
    rt->count = 0;
    return true;
}

bool flHwdogRtClear(fl_hwdog_rt_t *rt)
{
    // The host's CLEAR. It zeroes the counter for the NEXT boot; a board already in SAFE stays safe
    // for the rest of this run, because the reason it is safe has not been fixed by a frame.
    bool had = rt->count != 0;
    rt->count = 0;
    return had;
}

const char *flHwdogStateName(fl_hwdog_state_t s)
{
    switch (s)
    {
        case FL_HWDOG_OFF:       return "off";
        case FL_HWDOG_ARMED:     return "armed";
        case FL_HWDOG_CRASHED:   return "crashed";
        case FL_HWDOG_REBOOTING: return "rebooting";
        case FL_HWDOG_SAFE:      return "safe";
    }
    return "?";
}

void flHwdogGapReset(fl_hwdog_gap_t *g, uint32_t now_ms)
{
    g->last_us = 0;
    g->max_gap_us = 0;
    g->passes = 0;
    g->window_start_ms = now_ms;
    g->primed = false;
}

uint32_t flHwdogGapSample(fl_hwdog_gap_t *g, uint32_t now_us)
{
    if (!g->primed)
    {
        g->last_us = now_us;
        g->primed = true;
        return 0;
    }
    uint32_t gap = (uint32_t)(now_us - g->last_us);   // wraps correctly across the 71 min roll
    g->last_us = now_us;
    if (g->passes != 0xFFFFFFFFu) g->passes++;
    if (gap > g->max_gap_us) g->max_gap_us = gap;
    return gap;
}

#endif // FORGE_LINK_INJECT || FL_HWDOG_HOST
