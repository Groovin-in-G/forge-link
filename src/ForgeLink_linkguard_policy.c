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

// THE LINK GUARD'S RULES. Plain C, no SDK; the header says why each rule exists.
// Mirrored line for line by tools/fl_link_guard.py; tools/tests/test_fl_linkguard.py drives both.

#include "ForgeLink_linkguard_policy.h"

// ⛔ COMPILED ONLY INTO BRIDGE BUILDS - AND THE HOST TEST. An unreferenced object in another env
// still reorders that env's veneers (ForgeLink_bulk_policy.c, gate A6), so it is not there at all.
#if defined(FORGE_LINK_PROBE) || defined(FL_LINKGUARD_HOST)

#include <string.h>

// The EXEC request layout (ForgeLink_frames.h FL_EXEC_FIXED_LEN and fl_handle_exec), spelled out
// so this file stays free of board headers: target, flags, timeout u32, cdbLen, cdb, in u32, out u32.
#define FL_LG_EXEC_FIXED 15u

void flLinkGuardNoteHostFrame(fl_link_guard_t *g, uint32_t now_ms, bool suspended)
{
    g->last_host_ms = now_ms;
    if (suspended || (g->gone & FL_LG_GONE_REENUM)) return;   // U1: not the host coming back
    g->gone = 0;
    g->tries = 0;
}

uint8_t flLinkGuardNoteExec(fl_link_guard_t *g, const uint8_t *payload, uint32_t len, bool success,
                            uint8_t mount_gen)
{
    static const uint8_t on[6] = FL_LINK_GUARD_CDB_ON;
    static const uint8_t off[6] = FL_LINK_GUARD_CDB_OFF;
    if (!success || len < FL_LG_EXEC_FIXED + 6u) return g->on;
    if (payload[0] != FL_LINK_GUARD_TARGET || payload[6] != 6u) return g->on;
    const uint8_t *cdb = &payload[7];
    if (memcmp(cdb, on, 6) == 0)
    {
        g->on = 1;
        g->tries = 0;
        g->gone = 0;   // U1: the host carried LINK ON itself - it is here and owns the link
        // U1 final review: and it did so on THIS configuration. Adopt the mount generation, or a
        // tud_mount_cb that landed since the last look would latch REENUM on this very pass and
        // the guard would send 09 OFF straight after the host's own ON.
        g->mgen = mount_gen;
    }
    else if (memcmp(cdb, off, 6) == 0)
    {
        g->on = 0;
        g->tries = 0;
        g->gone = 0;
    }
    return g->on;
}

void flLinkGuardNoteUsb(fl_link_guard_t *g, bool mounted, bool dtr, bool suspended, uint8_t mount_gen)
{
    if (g->on && (!mounted || (g->dtr && !dtr) || suspended)) g->gone |= FL_LG_GONE_LOST;
    if (g->on && mount_gen != g->mgen) g->gone |= FL_LG_GONE_REENUM;   // U1: a replug
    g->dtr = dtr ? 1 : 0;
    g->mgen = mount_gen;
}

uint8_t flLinkGuardDecide(const fl_link_guard_t *g, uint32_t now_ms, bool busy)
{
    if (!g->on || busy) return FL_LG_IDLE;
    uint8_t why;
    if (g->gone) why = FL_LG_OFF_USB;
    else if ((uint32_t)(now_ms - g->last_host_ms) >= FL_LINK_GUARD_MS) why = FL_LG_OFF_SILENT;
    else return FL_LG_IDLE;
    if (g->tries != 0 && (int32_t)(now_ms - g->next_try_ms) < 0) return FL_LG_IDLE;
    return why;
}

uint8_t flLinkGuardAfterSend(fl_link_guard_t *g, uint32_t now_ms, bool ok)
{
    if (g->tries == 0 && g->fired != 0xFFFFu) g->fired++;
    if (ok)
    {
        g->on = 0;
        g->tries = 0;
        g->gone = 0;
        return FL_LG_R_OFF;
    }
    g->tries++;
    if (g->tries >= FL_LINK_GUARD_TRIES)
    {
        // Given up: the state is CLEARED, so the guard never sends again for this link. The sampler
        // may still be deaf (a power cycle clears it, W29); the log says so.
        g->on = 0;
        g->tries = 0;
        g->gone = 0;
        if (g->gave_up != 0xFFu) g->gave_up++;
        return FL_LG_R_GAVE_UP;
    }
    g->next_try_ms = now_ms + FL_LINK_GUARD_GAP_MS;
    return FL_LG_R_RETRY;
}

#endif // FORGE_LINK_PROBE || FL_LINKGUARD_HOST
