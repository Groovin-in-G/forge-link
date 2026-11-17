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

uint8_t flLinkGuardNoteExec(fl_link_guard_t *g, const uint8_t *payload, uint32_t len, bool success, uint8_t outcome,
                            uint8_t mount_gen)
{
    static const uint8_t on[6] = FL_LINK_GUARD_CDB_ON;
    static const uint8_t off[6] = FL_LINK_GUARD_CDB_OFF;
    if (len < FL_LG_EXEC_FIXED + 6u) return g->on;
    if (payload[0] != FL_LINK_GUARD_TARGET || payload[6] != 6u) return g->on;
    const uint8_t *cdb = &payload[7];
    if (!success)
    {
        // C45 follow-up (review): only an exchange that got PAST SELECTION can have put the
        // CDB in the sampler. NO_SELECT, REJECTED, BAD_REQUEST, NOT_INITIATOR, CHAIN_BROKEN,
        // REPLAY_UNAVAILABLE and note-at's HELD put nothing on the bus and leave the state alone:
        // a MAYBE there would fire 5 OFFs at an unreachable sampler, give up, and drop note-at's
        // kept note-offs - a stuck note. Codes = ForgeLink_probe.h FL_O_* (the test checks them).
        const bool may_have_landed = outcome == 0x00u    /* COMPLETED, bad status */
                                  || outcome == 0x03u    /* PHASE_ERROR */
                                  || outcome == 0x04u    /* TIMEOUT */
                                  || outcome == 0x05u    /* BUS_RESET */
                                  || outcome == 0x13u;   /* DATA_OVERFLOW: a data phase after the CDB */
        if (!may_have_landed) return g->on;
        // C45 (deep review): a LINK ON that did not succeed may still have LANDED (a
        // timeout, a reset, a phase error or a bad status after the CDB went out), and the sampler
        // may be deaf to its DIN now. Not known-off, so the guard keeps the duty to send 09 OFF:
        // possibly-on, never downgrading a confirmed ON. A failed OFF changes nothing (still on).
        if (memcmp(cdb, on, 6) == 0 && g->on == FL_LG_ON_NO)
        {
            g->on = FL_LG_ON_MAYBE;
            g->tries = 0;
            g->gone = 0;
            g->mgen = mount_gen;   // as a good ON: the host is on THIS configuration
        }
        return g->on;
    }
    if (memcmp(cdb, on, 6) == 0)
    {
        g->on = FL_LG_ON_YES;
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

/*************************
 * C48 · the notes       *
 *************************/

static uint32_t lg_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int lg_find(const fl_lg_notes_t *t, uint16_t k)
{
    for (int i = 0; i < t->n; i++) if (t->key[i] == k) return i;
    return -1;
}

static void lg_carried(fl_lg_notes_t *t, uint8_t st, uint8_t d1, uint8_t d2, bool ok)
{
    const uint8_t ch = st & 0x0F, hi = st & 0xF0;
    const uint16_t k = (uint16_t)((ch << 7) | d1);
    if (hi == 0x90 && d2 != 0)
    {
        t->used |= (uint16_t)(1u << ch);
        if (lg_find(t, k) >= 0) return;
        if (t->n < FL_LG_NOTES) t->key[t->n++] = k;
        else if (t->overflow != 0xFFu) t->overflow++;   // Bn 7B 00 on `used` is its release
    }
    else if (hi == 0x80 || hi == 0x90)
    {
        if (!ok) return;                                 // a failed off: it may still sound
        const int i = lg_find(t, k);
        if (i >= 0) t->key[i] = t->key[--t->n];          // the first note-off ends it
    }
    else if (hi == 0xB0 && d1 == 0x40)
    {
        if (d2 >= 64) t->sustain |= (uint16_t)(1u << ch);
        else if (ok) t->sustain &= (uint16_t)~(1u << ch);
    }
}

void flLgNotesNoteExec(fl_lg_notes_t *t, const uint8_t *payload, uint32_t len, bool success, uint8_t outcome)
{
    if (len < FL_LG_EXEC_FIXED + 6u || payload[0] != FL_LINK_GUARD_TARGET || payload[6] != 6u) return;
    const uint8_t *cdb = &payload[7];
    if (cdb[0] != 0x0Cu || (cdb[5] & 0x80u)) return;     // a non-bulk 0C only: the MIDI one
    // As the 09 rule (C45): a failed exchange counts only if its data may have reached the sampler.
    if (!success && !(outcome == 0x00u || outcome == 0x03u || outcome == 0x04u || outcome == 0x05u
                      || outcome == 0x13u)) return;
    const uint32_t out_len = lg_get32(&payload[7 + 6 + 4]);
    if (len != FL_LG_EXEC_FIXED + 6u + out_len) return;
    const uint8_t *m = &payload[FL_LG_EXEC_FIXED + 6u];
    uint8_t run = 0, d[2] = { 0, 0 }, have = 0;
    bool sysex = false;
    for (uint32_t i = 0; i < out_len; i++)
    {
        const uint8_t b = m[i];
        if (b >= 0xF8) continue;                         // real-time: no effect on running status
        if (sysex) { if (b & 0x80) sysex = false; else continue; }   // F7 or any status ends it
        if (b == 0xF0) { sysex = true; run = 0; continue; }
        if (b >= 0xF0) { run = 0; have = 0; continue; }  // system common cancels running status
        if (b & 0x80) { run = b; have = 0; continue; }
        if (!run) continue;
        d[have++] = b;
        const uint8_t need = ((run & 0xF0) == 0xC0 || (run & 0xF0) == 0xD0) ? 1u : 2u;
        if (have == need)
        {
            have = 0;
            if (need == 2u) lg_carried(t, run, d[0], d[1], success);
        }
    }
}

static uint16_t lg_put(uint8_t *out, uint16_t cap, uint16_t n, uint8_t a, uint8_t b, uint8_t c)
{
    if ((uint32_t)(n + 1u) * 3u > cap) return n;
    out[n * 3u] = a; out[n * 3u + 1u] = b; out[n * 3u + 2u] = c;
    return (uint16_t)(n + 1u);
}

uint16_t flLgNotesBuild(const fl_lg_notes_t *t, uint8_t *out, uint16_t cap)
{
    uint16_t n = 0;
    for (uint8_t c = 0; c < 16; c++)
        if (t->sustain & (1u << c)) n = lg_put(out, cap, n, (uint8_t)(0xB0 | c), 0x40, 0x00);
    for (int i = 0; i < t->n; i++)
        n = lg_put(out, cap, n, (uint8_t)(0x80 | ((t->key[i] >> 7) & 0x0F)), (uint8_t)(t->key[i] & 0x7F), 0x00);
    for (uint8_t c = 0; c < 16; c++)
        if (t->used & (1u << c)) n = lg_put(out, cap, n, (uint8_t)(0xB0 | c), 0x7B, 0x00);
    return n;
}

void flLgNotesClear(fl_lg_notes_t *t)
{
    memset(t, 0, sizeof(*t));
}

uint8_t flLinkGuardBeforeReset(const fl_link_guard_t *g, const fl_lg_notes_t *t)
{
    if (!g->on) return FL_LG_RESET_NOTHING;
    return (uint8_t)(FL_LG_RESET_OFF | ((t->n || t->used || t->sustain) ? FL_LG_RESET_RELEASE : 0u));
}

#endif // FORGE_LINK_PROBE || FL_LINKGUARD_HOST
