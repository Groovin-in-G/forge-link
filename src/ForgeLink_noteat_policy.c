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

// TQ2 · NOTE_AT's rules (docs/note-at.md r3). The header says why it is plain C and where it lives.
// Deviations from the spec, each with its reason: docs/reviews/note-at-tq2-build-notes.md.

#include "ForgeLink_noteat_policy.h"

#if defined(FORGE_LINK_NOTE_AT) || defined(FL_NOTEAT_HOST)

#include <string.h>

/*************************
 * byte helpers (LE)     *
 *************************/

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
static void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)v); put32(p + 4, (uint32_t)(v >> 32)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16); }
static uint64_t get64(const uint8_t *p) { return (uint64_t)get32(p) | ((uint64_t)get32(p + 4) << 32); }

/*************************
 * the state (.bss)      *
 *************************/

// One queued message. The due is kept apart as its low 32 bits (q_due): a queued due is always within
// ±30 min of now (FL_TQ_PAST_CLAMP_US, max_ahead 1 s), so the low word is unambiguous. `idx` (the
// message's index in its frame) is carried because a RELEASE can remove messages from the middle of a
// frame, after which the index cannot be derived (build notes: 10 B per slot, not §12's 8).
typedef struct { uint8_t st, d1, d2, frame, idx, flags; } tq_slot_t;
#define SF_PAST_DUE   0x01   // its due was already past when its frame arrived
#define SF_RELEASE    0x02   // a note-off, Bn 7B 00 or a pedal-up (§6)
#define SF_DROPPABLE  0x04   // a controller of a CTL_MAY_DROP frame (§5)

typedef struct { uint16_t batch; uint8_t n, left, flags, drop_ms; } tq_frame_t;   // left 0 = free

typedef struct { uint16_t batch; uint8_t idx, cause; uint32_t late_us, select_us; } tq_late_t;

// Sounding-list entry: channel (4 bits) << 7 | key (7 bits), plus two flags.
#define LE_KEYMASK    0x07FFu
#define LE_UNCERTAIN  0x0800u
#define LE_PENDING    0x1000u

typedef struct {
    uint32_t frames_admitted, msgs_admitted, msgs_fired, fires, msgs_late, msgs_slow, late_log_overflow;
    uint32_t max_late_us, max_early_us, max_select_us;
    uint32_t frames_refused_full, frames_refused_other, frames_duplicate, frames_cancelled;
    uint32_t frames_dropped, msgs_flushed, msgs_dropped_late;
    uint16_t drops[8];              // by cause - 1: lease usb hello link_off bulk_reset host fire_fail rearm
    uint32_t releases_sent, fire_failures, exec_held;
    uint16_t exec_overran, queue_high_water, max_receipt_to_queued_us, releases_served;
    uint32_t log_saves_deferred;
} tq_counters_t;

static struct {
    uint8_t  inited, armed, fail_run, dtr, epoch_seen, deferring, stranded, split_full;
    // TQ2 review 1 item 5: the board owes a release from an earlier session. With `armed`, the ONE
    // cheap test every hook makes before any clock read or state write. Set when an obligation is
    // marked; a stale 1 only costs one cleanup_pending() count, which clears it.
    uint8_t  pend_any;
    uint16_t clear_reads;           // B10-FATE: TQ_STATUS bit0 clears since boot (never itself cleared; wraps)
    uint16_t generation, last_admitted, lease_ms, coalesce_us, q_used, bulk_epoch;
    uint16_t usb_loss_gen;          // U1: the USB loss generation as last seen (seeded at ARM)
    uint32_t boot_id, frames_dropped_boot;
    uint64_t split_due, last_served_us, paused_us, bus_free_at_us, last_cleanup_us, last_logsave_us, exec_next_due;
    uint8_t  frames_used, list_n, late_n, fr_n;
    uint16_t list[FL_TQ_LIST];
    uint16_t sustain, overflow, used, pend_pedal, pend_7b;
    tq_counters_t c;
    uint8_t  fr_cause, fr_more, fr_released;
    uint16_t fr_gen, fr_frames;
    uint16_t fr_batch[FL_TQ_FLUSH_REC];
    uint8_t  fr_unfired[FL_TQ_FLUSH_REC];
    tq_late_t late[FL_TQ_LATE_LOG];
} g;

static uint32_t   q_due[FL_TQ_QCAP];
static tq_slot_t  q[FL_TQ_QCAP];
static tq_frame_t fr[FL_TQ_FRAMES];

// Release messages being sent by a flush, a RELEASE or the pending clean-up (3 bytes each). The most
// any one can need: 16 pedal-ups + 64 named keys + 64 listed keys + 16 all-notes-off.
#define REL_MAX (16 + FL_TQ_MAX_KEYS + FL_TQ_LIST + 16)
static uint8_t  rel_buf[REL_MAX * 3];
static uint16_t rel_n;

static void tq_ensure(void)
{
    if (g.inited) return;
    uint32_t id = flTqHookRandom();
    memset(&g, 0, sizeof(g));
    g.inited = 1;
    g.boot_id = id ? id : 1u;
    g.coalesce_us = FL_TQ_COALESCE_DEFAULT;
}

void flTqResetForTest(void)
{
    memset(&g, 0, sizeof(g));
    g.q_used = 0;
    rel_n = 0;
}

bool     flTqArmed(void)      { tq_ensure(); return g.armed != 0; }
uint32_t flTqBootId(void)     { tq_ensure(); return g.boot_id; }
uint16_t flTqGeneration(void) { tq_ensure(); return g.generation; }
uint16_t flTqQueueUsed(void)  { tq_ensure(); return g.q_used; }

/*************************
 * messages (§3)         *
 *************************/

static uint8_t msg_need(uint8_t st) { return ((st & 0xF0) == 0xC0 || (st & 0xF0) == 0xD0) ? 2 : 3; }

static bool msg_admissible(uint8_t st, uint8_t d1, uint8_t d2, uint8_t len)
{
    if (st < 0x80 || st >= 0xF0) return false;
    const uint8_t hi = st & 0xF0;
    if (hi == 0xC0) return false;                       // Program Change: always (the P5 rule)
    if (len != msg_need(st)) return false;
    if (d1 & 0x80) return false;
    if (len == 3 && (d2 & 0x80)) return false;
    if (hi == 0xB0)
    {
        if (d1 == 0x00 || d1 == 0x20 || d1 == 0x06 || d1 == 0x26) return false;
        if (d1 >= 0x60 && d1 <= 0x65) return false;
        if (d1 >= 0x78) return d1 == 0x7B && d2 == 0;   // channel mode: only all-notes-off
    }
    return true;
}

static bool msg_is_on(uint8_t st, uint8_t d2)  { return (st & 0xF0) == 0x90 && d2 != 0; }
static bool msg_is_off(uint8_t st, uint8_t d2) { return (st & 0xF0) == 0x80 || ((st & 0xF0) == 0x90 && d2 == 0); }

static bool msg_is_release(uint8_t st, uint8_t d1, uint8_t d2)
{
    if (msg_is_off(st, d2)) return true;
    if ((st & 0xF0) == 0xB0) return (d1 == 0x7B && d2 == 0) || (d1 == 0x40 && d2 < 64);
    return false;
}

static bool msg_is_droppable_ctl(uint8_t st, uint8_t d1)
{
    const uint8_t hi = st & 0xF0;
    if (hi == 0xB0) return d1 != 0x7B && d1 != 0x40;
    return hi == 0xA0 || hi == 0xD0 || hi == 0xE0;
}

/*************************
 * time                  *
 *************************/

static uint64_t due64(uint32_t lo, uint64_t now)
{
    return now + (uint64_t)(int64_t)(int32_t)(lo - (uint32_t)now);
}

static uint64_t q_due64(uint16_t i, uint64_t now) { return due64(q_due[i], now); }

static uint64_t next_due(uint64_t now) { return g.q_used ? q_due64(0, now) : 0; }

static void bump_generation(void)
{
    g.generation = (uint16_t)(g.generation + 1);
    if (g.generation == 0) g.generation = 1;
    g.last_admitted = 0;
    flTqHookGenerationBumped();
}

/*************************
 * the sounding list     *
 *************************/

static int list_find(uint8_t ch, uint8_t key)
{
    const uint16_t k = (uint16_t)((ch << 7) | key);
    for (int i = 0; i < g.list_n; i++)
        if ((g.list[i] & LE_KEYMASK) == k) return i;
    return -1;
}

static void list_add(uint8_t ch, uint8_t key, uint16_t flags)
{
    int i = list_find(ch, key);
    if (i >= 0)
    {
        // A certain delivery makes a possibly-sounding entry certain; a failed one never makes a
        // certain entry uncertain (it sounds either way). A pending flag is only ever added.
        if (!(flags & LE_UNCERTAIN)) g.list[i] &= (uint16_t)~LE_UNCERTAIN;
        g.list[i] |= (uint16_t)(flags & LE_PENDING);
        return;
    }
    if (g.list_n >= FL_TQ_LIST)
    {
        g.overflow |= (uint16_t)(1u << ch);   // the channel falls back to Bn 7B 00 at clean-up
        if (flags & LE_PENDING) g.pend_7b |= (uint16_t)(1u << ch);
        return;
    }
    g.list[g.list_n++] = (uint16_t)((ch << 7) | key | flags);
}

static void list_remove_at(int i)
{
    g.list[i] = g.list[--g.list_n];
}

// r3 (MEASURED): keyed by (channel, key), and the FIRST note-off removes the entry.
static void list_remove(uint8_t ch, uint8_t key)
{
    int i = list_find(ch, key);
    if (i >= 0) list_remove_at(i);
}

// What a message the sampler took (ok) or may have taken (!ok) does to the board's memory. Shared by
// the fire path and the EXEC path (§10). Releases that FAILED are the caller's (retry / pending).
static void apply_carried(uint8_t st, uint8_t d1, uint8_t d2, bool ok)
{
    const uint8_t ch = st & 0x0F, hi = st & 0xF0;
    if (msg_is_on(st, d2))
    {
        g.used |= (uint16_t)(1u << ch);
        list_add(ch, d1, ok ? 0 : LE_UNCERTAIN);
    }
    else if (msg_is_off(st, d2))
    {
        if (ok) list_remove(ch, d1);
    }
    else if (hi == 0xB0 && d1 == 0x40)
    {
        if (d2 >= 64) g.sustain |= (uint16_t)(1u << ch);            // down, or possibly down
        else if (ok) { g.sustain &= (uint16_t)~(1u << ch); g.pend_pedal &= (uint16_t)~(1u << ch); }
    }
    else if (hi == 0xB0 && d1 == 0x7B && ok)
    {
        // All notes off went through: the channel's fallback and its "used" duty are discharged. Its
        // listed keys are KEPT (a held pedal may keep them; an extra note-off is harmless).
        g.overflow &= (uint16_t)~(1u << ch);
        g.used &= (uint16_t)~(1u << ch);
        g.pend_7b &= (uint16_t)~(1u << ch);
    }
}

// A release that could not be delivered after its retries: the obligation is PENDING, never cleared.
static void mark_pending(uint8_t st, uint8_t d1, uint8_t d2)
{
    g.pend_any = 1;
    const uint8_t ch = st & 0x0F, hi = st & 0xF0;
    if (msg_is_off(st, d2)) list_add(ch, d1, LE_UNCERTAIN | LE_PENDING);
    else if (hi == 0xB0 && d1 == 0x40) g.pend_pedal |= (uint16_t)(1u << ch);
    else if (hi == 0xB0 && d1 == 0x7B) g.pend_7b |= (uint16_t)(1u << ch);
}

static uint16_t popcount16(uint16_t v) { uint16_t n = 0; while (v) { n += v & 1u; v >>= 1; } return n; }

static uint16_t cleanup_pending(void)
{
    uint16_t n = (uint16_t)(popcount16(g.pend_pedal) + popcount16(g.pend_7b));
    for (int i = 0; i < g.list_n; i++) if (g.list[i] & LE_PENDING) n++;
    return n;
}

static uint8_t state_flags(void)
{
    uint8_t f = 0;
    if (g.list_n == 0) f |= FL_TQ_SF_SOUNDING_EMPTY;
    if (g.overflow) f |= FL_TQ_SF_LIST_OVERFLOW;
    if (cleanup_pending()) f |= FL_TQ_SF_CLEANUP_PENDING;
    for (int i = 0; i < g.list_n; i++) if (g.list[i] & LE_UNCERTAIN) { f |= FL_TQ_SF_UNCERTAIN; break; }
    if (g.stranded) f |= FL_TQ_SF_STRANDED;
    return f;
}

/*************************
 * the late log (§7)     *
 *************************/

static void late_log(uint16_t batch, uint8_t idx, uint8_t cause, uint32_t late_us, uint32_t select_us)
{
    if (g.late_n >= FL_TQ_LATE_LOG) { g.c.late_log_overflow++; return; }
    tq_late_t *r = &g.late[g.late_n++];
    r->batch = batch;
    r->idx = idx;
    r->cause = (uint8_t)((cause & 0x0F) | ((g.generation & 0x0F) << 4));
    r->late_us = late_us;
    r->select_us = select_us;
}

/*************************
 * the queue             *
 *************************/

// Remove the first n slots, keeping each frame's count of what is left.
static void q_drop_prefix(uint16_t n)
{
    for (uint16_t i = 0; i < n; i++)
    {
        tq_frame_t *f = &fr[q[i].frame];
        if (f->left && --f->left == 0) g.frames_used--;
    }
    g.q_used = (uint16_t)(g.q_used - n);
    memmove(&q_due[0], &q_due[n], g.q_used * sizeof(q_due[0]));
    memmove(&q[0], &q[n], g.q_used * sizeof(q[0]));
}

static void q_remove_at(uint16_t i)
{
    tq_frame_t *f = &fr[q[i].frame];
    if (f->left && --f->left == 0) g.frames_used--;
    g.q_used--;
    memmove(&q_due[i], &q_due[i + 1], (g.q_used - i) * sizeof(q_due[0]));
    memmove(&q[i], &q[i + 1], (g.q_used - i) * sizeof(q[0]));
}

// Insert after every slot due at or before `due`: due order, ties in admission order (§6).
static void q_insert(uint64_t due, uint64_t now, const tq_slot_t *s)
{
    uint16_t lo = 0, hi = g.q_used;
    while (lo < hi)
    {
        uint16_t mid = (uint16_t)((lo + hi) / 2);
        if (q_due64(mid, now) <= due) lo = (uint16_t)(mid + 1); else hi = mid;
    }
    memmove(&q_due[lo + 1], &q_due[lo], (g.q_used - lo) * sizeof(q_due[0]));
    memmove(&q[lo + 1], &q[lo], (g.q_used - lo) * sizeof(q[0]));
    q_due[lo] = (uint32_t)due;
    q[lo] = *s;
    g.q_used++;
}

/*************************
 * sending releases      *
 *************************/

static void rel_add(uint8_t st, uint8_t d1, uint8_t d2)
{
    if (rel_n >= REL_MAX) return;
    rel_buf[rel_n * 3] = st;
    rel_buf[rel_n * 3 + 1] = d1;
    rel_buf[rel_n * 3 + 2] = d2;
    rel_n++;
}

static bool rel_has(uint8_t st, uint8_t d1, uint8_t d2)
{
    for (uint16_t i = 0; i < rel_n; i++)
        if (rel_buf[i * 3] == st && rel_buf[i * 3 + 1] == d1 && rel_buf[i * 3 + 2] == d2) return true;
    return false;
}

static void rel_add_once(uint8_t st, uint8_t d1, uint8_t d2)
{
    if (!rel_has(st, d1, d2)) rel_add(st, d1, d2);
}

static void rel_add_pedal_ups(uint16_t mask)
{
    for (uint8_t c = 0; c < 16; c++) if (mask & (1u << c)) rel_add_once((uint8_t)(0xB0 | c), 0x40, 0x00);
}

static void rel_add_all_offs(uint16_t mask)
{
    for (uint8_t c = 0; c < 16; c++) if (mask & (1u << c)) rel_add_once((uint8_t)(0xB0 | c), 0x7B, 0x00);
}

// One exchange of up to FL_TQ_PER_EXCHANGE messages, retried (§6: a repeated release is harmless).
// Returns true when the sampler took it.
// ⛔ TQ2 review 1 · AN ABSENT SAMPLER MUST NOT STARVE THE MAIN LOOP. Every try is preceded by a
// watchdog kick, and a try the target did not even SELECT (the ~250 ms select wait ran out) ends the
// retries: retrying an absent sampler only multiplies that wait.
// ⛔ TQ2 follow-up · A SAMPLER THAT SELECTS AND THEN HANGS is the same: a try whose 5 s phase loop ran
// out (timed_out) ends the retries too. Either one sets g_tq_unreach, a LATCH that holds for the rest of
// the entry call (flTqPoll, flTqFrame, flTqExecHold/Done, flTqHello, flTqLinkGuardBefore - each clears it
// first): rel_send tries nothing more (all PENDING), a fire run fires no further group, a failed fire's
// releases are not retried. So ONE entry call puts at most ONE unanswered try on the bus (build notes R3).
static bool g_tq_unreach;
static void tq_unreach_clear(void) { g_tq_unreach = false; }
static bool tq_exchange(const uint8_t *msgs3, uint8_t n, uint8_t tries, fl_tq_xres_t *first, fl_tq_xres_t *last)
{
    uint8_t buf[FL_TQ_PER_EXCHANGE * 3];
    uint32_t len = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        const uint8_t st = msgs3[i * 3];
        buf[len++] = st;
        buf[len++] = msgs3[i * 3 + 1];
        if (msg_need(st) == 3) buf[len++] = msgs3[i * 3 + 2];
    }
    for (uint8_t t = 0; t < tries; t++)
    {
        fl_tq_xres_t x;
        memset(&x, 0, sizeof(x));
        flTqHookKick();
        flTqHookExchange(buf, len, &x);
        g.c.fires++;
        g.bus_free_at_us = x.end_us;
        if (first && t == 0) *first = x;
        if (last) *last = x;
        if (x.ok) return true;
        g.c.fire_failures++;
        if (x.no_select || x.timed_out) { g_tq_unreach = true; break; }
    }
    return false;
}

// Send rel_buf (all releases) in exchanges, each tried 1 + FL_TQ_RELEASE_RETRIES times. What is
// delivered updates the memory; what still fails is PENDING. With the link not ON nothing can be sent
// and everything is PENDING - and so it is once this entry call already found the sampler unreachable.
// Returns 0 delivered, 1 some failed, 2 nothing to send.
static uint8_t rel_send(uint8_t *sent, uint64_t *t0, uint64_t *t1)
{
    if (rel_n == 0) return 2;
    uint8_t result = 0;
    bool link = flTqHookLinkOn() && !g_tq_unreach;
    for (uint16_t at = 0; at < rel_n; at += FL_TQ_PER_EXCHANGE)
    {
        const uint8_t n = (uint8_t)((rel_n - at) < FL_TQ_PER_EXCHANGE ? (rel_n - at) : FL_TQ_PER_EXCHANGE);
        const uint8_t *m = &rel_buf[at * 3];
        bool ok = false;
        if (link)
        {
            fl_tq_xres_t a, b;
            ok = tq_exchange(m, n, 1 + FL_TQ_RELEASE_RETRIES, &a, &b);
            if (t0 && *t0 == 0) *t0 = a.arb_us;
            if (t1) *t1 = b.end_us;
            // An unanswered selection or a phase timeout: the sampler is not answering. The rest of
            // this send is not tried (each try would wait it out again); it is all PENDING, retried
            // <= once a second.
            if (!ok && g_tq_unreach) link = false;
        }
        for (uint8_t i = 0; i < n; i++)
        {
            if (ok) apply_carried(m[i * 3], m[i * 3 + 1], m[i * 3 + 2], true);
            else mark_pending(m[i * 3], m[i * 3 + 1], m[i * 3 + 2]);
        }
        if (ok) { g.c.releases_sent += n; if (sent) *sent = (uint8_t)((*sent + n) > 255 ? 255 : (*sent + n)); }
        else result = 1;
    }
    rel_n = 0;
    if (g.stranded && !cleanup_pending()) g.stranded = 0;
    return result;
}

// Everything the board owes and has not delivered, in one send: pedal-ups, the pending note-offs, the
// pending all-notes-off. The clean-up's retry, and (review 1 item 2) the first thing after a LINK ON.
static uint8_t tq_send_pending(void)
{
    rel_n = 0;
    rel_add_pedal_ups(g.pend_pedal);
    for (int i = 0; i < g.list_n; i++)
        if (g.list[i] & LE_PENDING)
            rel_add_once((uint8_t)(0x80 | ((g.list[i] >> 7) & 0x0F)), (uint8_t)(g.list[i] & 0x7F), 0);
    rel_add_all_offs(g.pend_7b);
    return rel_send(0, 0, 0);
}

// Every obligation the board holds: pedal-ups first, then a note-off per listed key, then Bn 7B 00 on
// every overflowed channel AND (r3) every channel a note was carried on. A flush sends all of it.
static void rel_build_everything(void)
{
    rel_n = 0;
    rel_add_pedal_ups((uint16_t)(g.sustain | g.pend_pedal));
    for (int i = 0; i < g.list_n; i++)
    {
        const uint16_t e = g.list[i];
        rel_add_once((uint8_t)(0x80 | ((e >> 7) & 0x0F)), (uint8_t)(e & 0x7F), 0x00);
    }
    rel_add_all_offs((uint16_t)(g.overflow | g.used | g.pend_7b));
}

/*************************
 * flush (§10)           *
 *************************/

static void tq_flush(uint8_t cause, bool disarm)
{
    // 1 · remove every queued, unfired message; one record per cut frame, oldest batch first.
    g.fr_cause = cause;
    g.fr_more = 0;
    g.fr_gen = g.generation;
    g.fr_frames = 0;
    g.fr_n = 0;
    for (;;)
    {
        // the oldest remaining cut frame: the largest distance back from the last admitted batch
        int best = -1;
        uint16_t best_age = 0;
        for (int i = 0; i < FL_TQ_FRAMES; i++)
        {
            if (!fr[i].left) continue;
            uint16_t age = (uint16_t)(g.last_admitted - fr[i].batch);
            if (best < 0 || age > best_age) { best = i; best_age = age; }
        }
        if (best < 0) break;
        g.fr_frames++;
        g.c.frames_dropped++;
        g.frames_dropped_boot++;
        if (g.fr_n < FL_TQ_FLUSH_REC)
        {
            g.fr_batch[g.fr_n] = fr[best].batch;
            g.fr_unfired[g.fr_n] = fr[best].left;
            g.fr_n++;
        }
        else g.fr_more = 1;
        fr[best].left = 0;
    }
    g.c.msgs_flushed += g.q_used;
    g.q_used = 0;
    g.frames_used = 0;
    memset(fr, 0, sizeof(fr));
    bump_generation();
    if (cause >= 1 && cause <= 8 && g.c.drops[cause - 1] != 0xFFFF) g.c.drops[cause - 1]++;

    // 2 · release, removal having come first.
    rel_build_everything();
    const uint8_t r = rel_send(0, 0, 0);
    g.fr_released = r != 1;

    // 3 · the cause's own last step.
    if (disarm)
    {
        g.armed = 0;
        g.fail_run = 0;
    }
}

/*************************
 * firing (§6)           *
 *************************/

// ⛔ TQ2 review 1 item 3 · THE SAFETY CHECKS, ONE COPY: each main-loop pass (step 1) AND between the
// exchange groups of a fire run, so a long run cannot carry on into a dead USB link, an expired lease
// or a reset epoch. Returns true when it flushed.
static bool tq_safety(void)
{
    bool mounted = true, dtr = false, suspended = false;
    uint16_t epoch = 0, loss = 0;
    flTqHookUsb(&mounted, &dtr, &suspended, &epoch, &loss);
    const bool dtr_fell = g.dtr && !dtr;
    g.dtr = dtr ? 1 : 0;
    const bool epoch_moved = g.epoch_seen && epoch != g.bulk_epoch;
    g.bulk_epoch = epoch;
    // U1: a USB SUSPEND or a RE-ENUMERATION since the last look is a USB
    // loss. A cable pulled from a bus-powered board never unmounts (pico-sdk forces VBUS detect) and DTR
    // never falls; the controller only reports a suspend, so without this the lease was the only thing
    // that flushed (9 s, every queued note fired after the pull). The loss GENERATION catches an event
    // that ended between two looks (several coalesce into one change); the suspended LEVEL catches a
    // loss still in force, whatever the generation did. The generation is consumed on EVERY read, armed
    // or not: one change is one flush, and a change while disarmed is never held against a later session.
    const bool lost = g.epoch_seen && loss != g.usb_loss_gen;
    g.usb_loss_gen = loss;
    g.epoch_seen = 1;
    if (!g.armed) return false;
    const uint64_t now = flTqHookNowUs();
    if (!mounted || dtr_fell || lost || suspended) { tq_flush(FL_TQ_C_USB, true); return true; }
    if (now - g.last_served_us >= g.paused_us + (uint64_t)g.lease_ms * 1000u) { tq_flush(FL_TQ_C_LEASE, true); return true; }
    if (epoch_moved) { tq_flush(FL_TQ_C_BULK_RESET, false); return true; }
    return false;
}

static void tq_fire_run(void)
{
    uint64_t seg = flTqHookNowUs();
    uint8_t groups = 0;
    while (g.armed && g.q_used)
    {
        // The board's own fires PAUSE the lease (§4), accrued per group so the check below is exact.
        uint64_t now = flTqHookNowUs();
        g.paused_us += now - seg;
        seg = now;
        // U1 review: before EVERY group, the first included - flTqPoll's busy-wait up to the head's due
        // runs between step 1's check and here, and a suspend in that gap must not let a group fire.
        if (tq_safety()) return;
        if (groups >= FL_TQ_GROUPS_PER_PASS) return;          // the rest fires next pass, SPLIT kept
        if (g_tq_unreach) break;                              // unreachable this call: the rest waits a pass
        now = flTqHookNowUs();
        if (q_due64(0, now) > now) { g.split_full = 0; break; }
        groups++;

        // Every message due at or before now + coalesce, in queue order, at most 16. A controller of a
        // CTL_MAY_DROP frame that would start more than ctl_drop_ms late is dropped, never a note.
        uint8_t msgs[FL_TQ_PER_EXCHANGE * 3];
        tq_slot_t meta[FL_TQ_PER_EXCHANGE];
        uint64_t dues[FL_TQ_PER_EXCHANGE];
        uint8_t n = 0;
        uint16_t i = 0;
        while (i < g.q_used && n < FL_TQ_PER_EXCHANGE)
        {
            const uint64_t due = q_due64(i, now);
            if (due > now + g.coalesce_us) break;
            const tq_slot_t s = q[i];
            const tq_frame_t *f = &fr[s.frame];
            if ((s.flags & SF_DROPPABLE) && now > due && (now - due) > (uint64_t)f->drop_ms * 1000u)
            {
                g.c.msgs_dropped_late++;
                late_log(f->batch, s.idx, FL_TQ_L_DROPPED, (uint32_t)(now - due), 0);
                i++;
                continue;
            }
            msgs[n * 3] = s.st; msgs[n * 3 + 1] = s.d1; msgs[n * 3 + 2] = s.d2;
            meta[n] = s;
            dues[n] = due;
            n++;
            i++;
        }
        uint16_t batches[FL_TQ_PER_EXCHANGE];
        for (uint8_t k = 0; k < n; k++) batches[k] = fr[meta[k].frame].batch;
        q_drop_prefix(i);
        if (n == 0) continue;

        fl_tq_xres_t x;
        const bool ok = tq_exchange(msgs, n, 1, &x, 0);
        g.c.msgs_fired += n;
        if (x.select_us > g.c.max_select_us) g.c.max_select_us = x.select_us;
        const bool slow = x.select_us > FL_TQ_SLOW_US;

        for (uint8_t k = 0; k < n; k++)
        {
            const uint64_t due = dues[k];
            uint8_t cause = 0xFF;
            uint32_t late_us = 0;
            if (x.arb_us > due + FL_TQ_LATE_US)
            {
                late_us = (uint32_t)(x.arb_us - due);
                g.c.msgs_late++;
                if (late_us > g.c.max_late_us) g.c.max_late_us = late_us;
                if (meta[k].flags & SF_PAST_DUE) cause = FL_TQ_L_PAST_DUE;
                else if (g.split_full && due <= g.split_due) cause = FL_TQ_L_SPLIT;
                else if (g.last_logsave_us > due && g.last_logsave_us < x.arb_us) cause = FL_TQ_L_HOUSEKEEPING;
                else cause = FL_TQ_L_BUSY;
            }
            else if (due > x.arb_us && (uint32_t)(due - x.arb_us) > g.c.max_early_us)
                g.c.max_early_us = (uint32_t)(due - x.arb_us);
            if (slow) { g.c.msgs_slow++; if (cause == 0xFF) cause = FL_TQ_L_SLOW; }
            if (!ok) cause = FL_TQ_L_FAILED;
            if (cause != 0xFF) late_log(batches[k], meta[k].idx, cause, late_us, x.select_us);
            if (ok) apply_carried(meta[k].st, meta[k].d1, meta[k].d2, true);
            else if (!(meta[k].flags & SF_RELEASE)) apply_carried(meta[k].st, meta[k].d1, meta[k].d2, false);
        }

        if (ok) g.fail_run = 0;
        else
        {
            g.fail_run++;
            // Releases are retried (a repeated release is harmless); note-ons and controllers are not.
            uint8_t rm[FL_TQ_PER_EXCHANGE * 3];
            uint8_t rn = 0, rk[FL_TQ_PER_EXCHANGE];
            for (uint8_t k = 0; k < n; k++)
                if (meta[k].flags & SF_RELEASE)
                {
                    memcpy(&rm[rn * 3], &msgs[k * 3], 3);
                    rk[rn++] = k;
                }
            if (rn && g_tq_unreach)
            {
                // The sampler did not select, or selected and hung: no retry now, the releases are
                // owed (PENDING).
                for (uint8_t j = 0; j < rn; j++)
                    mark_pending(meta[rk[j]].st, meta[rk[j]].d1, meta[rk[j]].d2);
            }
            else if (rn)
            {
                fl_tq_xres_t a;
                const bool rok = tq_exchange(rm, rn, FL_TQ_RELEASE_RETRIES, &a, 0);
                for (uint8_t j = 0; j < rn; j++)
                {
                    const uint8_t k = rk[j];
                    const uint32_t lt = a.arb_us > dues[k] ? (uint32_t)(a.arb_us - dues[k]) : 0;
                    late_log(batches[k], meta[k].idx, FL_TQ_L_RETRY, lt, a.select_us);
                    if (rok) apply_carried(meta[k].st, meta[k].d1, meta[k].d2, true);
                    else mark_pending(meta[k].st, meta[k].d1, meta[k].d2);
                }
            }
            if (g.fail_run >= FL_TQ_FAIL_RUN)
            {
                tq_flush(FL_TQ_C_FIRE_FAIL, true);
                break;
            }
        }
        g.split_full = n == FL_TQ_PER_EXCHANGE;
        g.split_due = dues[n - 1];
    }
    if (!g.q_used) g.split_full = 0;
    // The board's own fires PAUSE the lease (§4): a dead host's queue cannot keep it alive.
    g.paused_us += flTqHookNowUs() - seg;
}

/*************************
 * the main-loop pass    *
 *************************/

void flTqPoll(void)
{
    if (!g.armed && !g.pend_any) return;          // unarmed, nothing owed: one branch (review 1 item 5)
    tq_unreach_clear();
    // 1 · safety first: USB loss, the lease, a bulk reset - before any fire and any host frame.
    (void)tq_safety();

    // 2 · a release the board owes and could not deliver: at most once a second, link ON.
    if (g.pend_any && !cleanup_pending()) g.pend_any = 0;
    if (g.pend_any && flTqHookLinkOn())
    {
        const uint64_t now = flTqHookNowUs();
        if (g.last_cleanup_us == 0 || now - g.last_cleanup_us >= FL_TQ_CLEANUP_RETRY_US)
        {
            (void)tq_send_pending();
            // Stamped AFTER the send: the once-a-second limit is a second of main loop between
            // clean-ups, not a second from one clean-up's start (which an absent sampler could fill).
            g.last_cleanup_us = flTqHookNowUs();
        }
    }

    // 3 · due fires. Within FL_TQ_BUSYWAIT_US of the head's due, wait for it here (the watchdog fed)
    // rather than return to a loop whose pass length would become the jitter. (A sampler found
    // unreachable in steps 1-2 of this pass fires nothing: tq_fire_run checks the latch first.)
    if (!g.armed || !g.q_used) return;
    uint64_t now = flTqHookNowUs();
    uint64_t due = q_due64(0, now);
    if (due > now)
    {
        if (due - now > FL_TQ_BUSYWAIT_US) return;
        while ((now = flTqHookNowUs()) < due) flTqHookKick();
    }
    tq_fire_run();
}

void flTqServed(void)
{
    if (!g.armed) return;
    g.last_served_us = flTqHookNowUs();
    g.paused_us = 0;
}

bool flTqLogSaveOk(void)
{
    if (!g.armed) return true;
    bool defer = false;
    if (g.q_used)
    {
        const uint64_t now = flTqHookNowUs();
        const uint64_t due = q_due64(0, now);
        defer = due <= now + FL_TQ_LOGSAVE_DEFER_US;
    }
    if (defer)
    {
        if (!g.deferring) g.c.log_saves_deferred++;   // one per deferral episode (build notes)
        g.deferring = 1;
        return false;
    }
    g.deferring = 0;
    g.last_logsave_us = flTqHookNowUs();
    return true;
}

void flTqHello(void)
{
    if (!g.armed) return;
    tq_unreach_clear();
    tq_flush(FL_TQ_C_HELLO, true);
}

// ⛔ TQ2 review 1 item 2 · A LINK OFF MUST NOT STRAND A RELEASE. Pending releases are retried only
// while the link is ON, so a 09 OFF carried while the board still owes note-offs would leave them
// owed until the next LINK ON. So: flush (armed); if anything is still PENDING, ONE more full release
// attempt while the link is still ON; whatever survives that is STRANDED - reported (state flag 0x10,
// TQ_STATUS byte 143) and sent FIRST after the next LINK ON (flTqExecDone).
static void tq_before_link_off(void)
{
    if (g.armed) tq_flush(FL_TQ_C_LINK_OFF, false);
    if (cleanup_pending()) (void)tq_send_pending();
    if (cleanup_pending()) g.stranded = 1;
}

void flTqLinkGuardBefore(void)
{
    if (!g.armed && !g.pend_any) return;
    tq_unreach_clear();
    tq_before_link_off();
}

void flTqLinkGuardAfter(bool gave_up)
{
    if (!g.armed && !g.pend_any) return;
    g.bus_free_at_us = flTqHookNowUs();
    if (gave_up)
    {
        // §10: pending releases are kept "until the link guard gives up and forgets the link".
        g.pend_pedal = g.pend_7b = 0;
        g.pend_any = 0;
        g.stranded = 0;
        for (int i = g.list_n - 1; i >= 0; i--) if (g.list[i] & LE_PENDING) list_remove_at(i);
    }
}

/*************************
 * EXEC (§8, §10)        *
 *************************/

// EXEC request: target u8, flags u8, timeout u32, cdb_len u8, cdb, in_alloc u32, out_len u32, data out.
#define EX_FIXED 15u

static bool exec_is_link_off(const uint8_t *p, uint32_t len)
{
    static const uint8_t off[6] = { 0x09, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return len >= EX_FIXED + 6u && p[0] == 6 && p[6] == 6 && memcmp(&p[7], off, 6) == 0;
}

static bool exec_is_link_on(const uint8_t *p, uint32_t len)
{
    static const uint8_t on[6] = { 0x09, 0x00, 0x01, 0x01, 0x00, 0x00 };
    return len >= EX_FIXED + 6u && p[0] == 6 && p[6] == 6 && memcmp(&p[7], on, 6) == 0;
}

static uint64_t exec_estimate(const uint8_t *p, uint32_t len)
{
    uint64_t est = FL_TQ_EXEC_BASE_US;
    if (len < EX_FIXED) return est;
    const uint8_t cdb_len = p[6];
    if (len < EX_FIXED + cdb_len) return est;
    const uint8_t *cdb = &p[7];
    const uint32_t in_alloc = get32(&p[7 + cdb_len]);
    const uint32_t out_len = get32(&p[11 + cdb_len]);
    const bool bulk = cdb_len == 6 && cdb[0] == 0x0C && (cdb[5] & 0x80);
    est += (uint64_t)out_len * (bulk ? 2u : 25u);   // any non-bulk data out costs 25 µs a byte (conservative)
    est += (uint64_t)in_alloc * 4u;
    if (cdb_len == 6 && cdb[0] == 0x0C && !bulk && out_len >= FL_TQ_APPLY_TAIL_MIN_OUT)
        est += FL_TQ_APPLY_TAIL_US;                 // the XL's apply after a whole-record write 
    return est;
}

// The earliest T >= now whose window [T, T + est + guard] meets no queued group's fire.
static uint64_t retry_at(uint64_t now, uint64_t est)
{
    if (est > (uint64_t)FL_TQ_MAX_AHEAD_MS * 1000u) return UINT64_MAX;
    uint64_t t = now;
    uint16_t i = 0;
    while (i < g.q_used)
    {
        const uint64_t gdue = q_due64(i, now);
        uint32_t bytes = 0;
        while (i < g.q_used && q_due64(i, now) == gdue) { bytes += msg_need(q[i].st); i++; }
        const uint64_t gend = gdue + FL_TQ_GROUP_BASE_US + 25u * (uint64_t)bytes;
        if (gend <= t) continue;
        if (t + est + FL_TQ_EXEC_GUARD_US <= gdue) break;
        t = gend;
    }
    return t;
}

bool flTqExecHold(const uint8_t *p, uint32_t len, uint64_t *retry)
{
    if (!g.armed && !g.pend_any) return false;    // an unarmed EXEC: one branch (review 1 item 5)
    tq_unreach_clear();
    g.exec_next_due = 0;
    // The LINK OFF attempt (§10): flush and release while the link is still ON, then carry the OFF.
    // Its queue was just emptied, so it is never held. Owed releases are tried once more (review 1).
    if (exec_is_link_off(p, len)) { tq_before_link_off(); return false; }
    if (!g.armed) return false;
    if (!g.q_used) return false;
    const uint64_t now = flTqHookNowUs();
    const uint64_t nd = next_due(now);
    const uint64_t est = exec_estimate(p, len);
    if (now + est + FL_TQ_EXEC_GUARD_US > nd)
    {
        g.c.exec_held++;
        *retry = retry_at(now, est);
        return true;
    }
    g.exec_next_due = nd;
    return false;
}

void flTqExecDone(const uint8_t *p, uint32_t len, bool success)
{
    if (!g.armed && !g.pend_any) return;          // an unarmed EXEC: one branch (review 1 item 5)
    tq_unreach_clear();
    // TQ2 review 1 item 2: a successful LINK ON sends whatever the board still owes FIRST - before
    // this EXEC's reply, before any fire, before the next host frame.
    if (success && g.pend_any && cleanup_pending() && exec_is_link_on(p, len)) (void)tq_send_pending();
    if (!g.armed) return;
    const uint64_t now = flTqHookNowUs();
    g.bus_free_at_us = now;
    if (g.exec_next_due && now > g.exec_next_due + FL_TQ_LATE_US && g.c.exec_overran != 0xFFFF)
        g.c.exec_overran++;
    g.exec_next_due = 0;
    // The shared accounting: a non-bulk 0C to ID 6 carries MIDI the board must remember (§10).
    if (len < EX_FIXED || p[0] != 6) return;
    const uint8_t cdb_len = p[6];
    if (cdb_len != 6 || len < EX_FIXED + cdb_len) return;
    const uint8_t *cdb = &p[7];
    if (cdb[0] != 0x0C || (cdb[5] & 0x80)) return;
    const uint32_t out_len = get32(&p[11 + cdb_len]);
    if (len != EX_FIXED + cdb_len + out_len) return;
    const uint8_t *m = &p[EX_FIXED + cdb_len];
    uint8_t run = 0, d[2], have = 0;
    bool sysex = false;
    for (uint32_t i = 0; i < out_len; i++)
    {
        const uint8_t b = m[i];
        if (b >= 0xF8) continue;                        // real-time: no effect on running status
        if (sysex) { if (b == 0xF7) sysex = false; else if (b & 0x80) { sysex = false; } else continue; }
        if (b == 0xF0) { sysex = true; run = 0; continue; }
        if (b >= 0xF0) { run = 0; have = 0; continue; }  // system common: cancels running status
        if (b & 0x80) { run = b; have = 0; continue; }
        if (!run) continue;
        d[have++] = b;
        if (have == msg_need(run) - 1)
        {
            have = 0;
            if ((run & 0xF0) != 0xC0 && (run & 0xF0) != 0xD0)
                apply_carried(run, d[0], d[1], success);
        }
    }
}

/*************************
 * NOTE_AT (§5)          *
 *************************/

static uint32_t note_at_reply(uint8_t *o, uint8_t outcome, uint8_t detail, uint16_t batch, uint64_t rx,
                              int64_t lead)
{
    memset(o, 0, FL_TQ_NOTE_AT_REPLY);
    o[0] = outcome;
    o[1] = detail;
    put16(&o[2], batch);
    put32(&o[4], g.boot_id);
    put16(&o[8], g.generation);
    put16(&o[10], g.q_used);
    put16(&o[12], FL_TQ_QCAP);
    put16(&o[14], FL_TQ_FLOOR_US);
    put64(&o[16], rx);
    if (lead > INT32_MAX) lead = INT32_MAX;
    if (lead < INT32_MIN) lead = INT32_MIN;
    put32(&o[24], (uint32_t)(int32_t)lead);
    put16(&o[28], g.last_admitted);
    if (outcome == FL_TQ_O_DUPLICATE) g.c.frames_duplicate++;
    else if (outcome == FL_TQ_O_QUEUE_FULL) g.c.frames_refused_full++;
    else if (outcome == FL_TQ_O_CANCELLED) g.c.frames_cancelled++;
    else if (outcome != FL_TQ_O_OK) g.c.frames_refused_other++;
    return FL_TQ_NOTE_AT_REPLY;
}

// lead_us = due of message 0 - rx, in unsigned arithmetic only (no signed overflow for any base), and
// saturated to the i32 the reply carries. An overflowing due is "infinitely far": INT32_MAX.
static int64_t lead_of(uint64_t base, uint16_t d0, uint64_t rx)
{
    if (d0 > UINT64_MAX - base) return INT32_MAX;
    const uint64_t due = base + d0;
    if (due >= rx) return (due - rx) > (uint64_t)INT32_MAX ? INT32_MAX : (int64_t)(due - rx);
    return (rx - due) > (uint64_t)INT32_MAX + 1u ? INT32_MIN : -(int64_t)(rx - due);
}

static uint32_t tq_note_at(const uint8_t *p, uint32_t len, uint64_t rx, bool bridge, uint8_t *o)
{
    const uint16_t batch = len >= 4 ? get16(&p[2]) : 0;
    if (!bridge) return note_at_reply(o, FL_TQ_O_NOT_INIT, 0, batch, rx, 0);
    if (len < FL_TQ_NOTE_AT_HEAD) return note_at_reply(o, FL_TQ_O_BAD_REQUEST, 0, batch, rx, 0);
    if (get32(&p[4]) != g.boot_id) return note_at_reply(o, FL_TQ_O_BOOT_MISMATCH, 0, batch, rx, 0);
    const uint8_t n = p[1];
    const uint8_t flags = p[10], drop_ms = p[11];
    const uint32_t entries_len = get32(&p[12]);
    const uint64_t base = get64(&p[16]);
    const uint8_t *e = &p[FL_TQ_NOTE_AT_HEAD];
    bool bad = p[0] != FL_TQ_VERSION || n < 1 || n > FL_TQ_MAX_MSGS || entries_len != 6u * n ||
               len != FL_TQ_NOTE_AT_HEAD + entries_len || (flags & 0xFE) ||
               ((flags & 1) ? drop_ms == 0 : drop_ms != 0);
    if (!bad)
        for (uint8_t i = 0; i < n; i++)
        {
            if (i && get16(&e[i * 6]) < get16(&e[(i - 1) * 6])) { bad = true; break; }   // decreasing delta
            if (e[i * 6 + 2] == 2 && e[i * 6 + 5] != 0) { bad = true; break; }          // the unused data2 byte
        }
    const int64_t lead = bad ? 0 : lead_of(base, get16(&e[0]), rx);
    if (bad) return note_at_reply(o, FL_TQ_O_BAD_REQUEST, 0, batch, rx, 0);
    if (!g.armed) return note_at_reply(o, FL_TQ_O_NOT_ARMED, 0, batch, rx, lead);
    if (get16(&p[8]) != g.generation) return note_at_reply(o, FL_TQ_O_CANCELLED, 0, batch, rx, lead);
    if (!flTqHookLinkOn()) return note_at_reply(o, FL_TQ_O_LINK_OFF, 0, batch, rx, lead);
    uint16_t nr = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        const uint8_t *m = &e[i * 6];
        if (!msg_admissible(m[3], m[4], m[5], m[2])) return note_at_reply(o, FL_TQ_O_BAD_MESSAGE, i, batch, rx, lead);
        if (!msg_is_release(m[3], m[4], m[5])) nr++;
    }
    // The batch rule (§4): the last admitted again = DUPLICATE, the next = new, anything else STALE.
    uint16_t next = (uint16_t)(g.last_admitted + 1);
    if (next == 0) next = 1;
    if (g.last_admitted && batch == g.last_admitted) return note_at_reply(o, FL_TQ_O_DUPLICATE, 0, batch, rx, lead);
    if (batch != next) return note_at_reply(o, FL_TQ_O_STALE, 0, batch, rx, lead);
    const uint64_t horizon = rx + (uint64_t)FL_TQ_MAX_AHEAD_MS * 1000u;
    for (uint8_t i = 0; i < n; i++)
    {
        // TQ2 review 1 item 4: base + delta is checked BEFORE it is added. A due past 2^64 µs would wrap
        // to the past and fire at once; it is refused TOO_FAR with the first offender's index.
        if (get16(&e[i * 6]) > UINT64_MAX - base) return note_at_reply(o, FL_TQ_O_TOO_FAR, i, batch, rx, lead);
        if (base + get16(&e[i * 6]) > horizon) return note_at_reply(o, FL_TQ_O_TOO_FAR, i, batch, rx, lead);
    }
    // Room (§6): everything fits; non-release messages leave the 64-slot release reserve free; a frame
    // with any non-release message leaves the 8 release-only frame entries free.
    if (g.q_used + n > FL_TQ_QCAP || (nr && g.q_used + nr > FL_TQ_QCAP - FL_TQ_RESERVE))
        return note_at_reply(o, FL_TQ_O_QUEUE_FULL, 0, batch, rx, lead);
    if (g.frames_used >= FL_TQ_FRAMES || (nr && g.frames_used >= FL_TQ_FRAMES - FL_TQ_FRAMES_REL_ONLY))
        return note_at_reply(o, FL_TQ_O_QUEUE_FULL, 1, batch, rx, lead);

    uint8_t fi = 0;
    while (fr[fi].left) fi++;
    fr[fi].batch = batch;
    fr[fi].n = n;
    fr[fi].left = n;
    fr[fi].flags = flags;
    fr[fi].drop_ms = drop_ms;
    g.frames_used++;
    const uint64_t now = flTqHookNowUs();
    for (uint8_t i = 0; i < n; i++)
    {
        const uint8_t *m = &e[i * 6];
        uint64_t due = base + get16(&m[0]);
        tq_slot_t s;
        s.st = m[3]; s.d1 = m[4]; s.d2 = m[2] == 3 ? m[5] : 0;
        s.frame = fi;
        s.idx = i;
        s.flags = 0;
        if (due < rx) s.flags |= SF_PAST_DUE;
        if (msg_is_release(s.st, s.d1, s.d2)) s.flags |= SF_RELEASE;
        if ((flags & 1) && msg_is_droppable_ctl(s.st, s.d1)) s.flags |= SF_DROPPABLE;
        if (now > FL_TQ_PAST_CLAMP_US && due < now - FL_TQ_PAST_CLAMP_US) due = now - FL_TQ_PAST_CLAMP_US;
        q_insert(due, now, &s);
    }
    g.last_admitted = batch;
    g.c.frames_admitted++;
    g.c.msgs_admitted += n;
    if (g.q_used > g.c.queue_high_water) g.c.queue_high_water = g.q_used;
    const uint64_t r2q = now > rx ? now - rx : 0;
    const uint16_t r2q16 = r2q > 0xFFFF ? 0xFFFF : (uint16_t)r2q;
    if (r2q16 > g.c.max_receipt_to_queued_us) g.c.max_receipt_to_queued_us = r2q16;
    return note_at_reply(o, FL_TQ_O_OK, 0, batch, rx, lead);
}

/*************************
 * TQ_SYNC (§4)          *
 *************************/

static uint32_t tq_sync(const uint8_t *p, uint32_t len, uint64_t rx, bool bridge, uint8_t *o)
{
    uint8_t outcome = FL_TQ_O_OK;
    const uint8_t op = len >= 1 ? p[0] : 0xFF;
    const uint32_t host_tag = len >= FL_TQ_SYNC_LEN ? get32(&p[8]) : 0;
    if (len != FL_TQ_SYNC_LEN || op > FL_TQ_SYNC_FLUSH || p[1] != 0 || get16(&p[6]) != 0)
        outcome = FL_TQ_O_BAD_REQUEST;
    else if (op == FL_TQ_SYNC_ARM && !bridge)
        outcome = FL_TQ_O_NOT_INIT;   // never arm a board that cannot reach the sampler (build notes 22)
    else if (op == FL_TQ_SYNC_ARM)
    {
        uint16_t lease = get16(&p[2]), co = get16(&p[4]);
        if (lease == 0) lease = FL_TQ_LEASE_DEFAULT;
        if (co == 0xFFFF) co = FL_TQ_COALESCE_DEFAULT;
        // U1: the USB as it is NOW. An ARM buffered before a pull and served after it would
        // re-seed the loss generation and swallow the loss; NOTE_ATs behind it would then fire until the
        // lease. So an ARM is refused NOT_ARMED while the bus is suspended or the device unmounted (no
        // state changes; an armed board is flushed by the next pass's safety check on the level).
        bool um = true, ud = false, us = false;
        uint16_t ue = 0, ul = 0;
        flTqHookUsb(&um, &ud, &us, &ue, &ul);
        if (lease < FL_TQ_LEASE_MIN || lease > FL_TQ_LEASE_MAX || co > FL_TQ_COALESCE_MAX)
            outcome = FL_TQ_O_BAD_REQUEST;
        else if (!um || us)
            outcome = FL_TQ_O_NOT_ARMED;
        else
        {
            if (g.armed) tq_flush(FL_TQ_C_REARM, false);   // a new generation
            else bump_generation();
            g.armed = 1;
            g.fail_run = 0;
            g.deferring = 0;
            // Unarmed, the hooks read nothing (review 1 item 5): seed the DTR edge and the bulk epoch
            // here, so the first armed pass judges a change from THIS moment.
            g.dtr = ud ? 1 : 0;
            g.bulk_epoch = ue;
            g.usb_loss_gen = ul;               // U1: a loss before this ARM (and over) is not this session's
            g.epoch_seen = 1;
            g.lease_ms = lease;
            g.coalesce_us = co;
            g.last_served_us = flTqHookNowUs();
            g.paused_us = 0;
        }
    }
    else if (get16(&p[2]) != 0 || get16(&p[4]) != 0)
        outcome = FL_TQ_O_BAD_REQUEST;
    else if (op == FL_TQ_SYNC_DISARM)
    {
        if (g.armed) tq_flush(FL_TQ_C_HOST, true);
    }
    else if (op == FL_TQ_SYNC_FLUSH)
    {
        if (g.armed) tq_flush(FL_TQ_C_HOST, false);   // disarmed: nothing is queued, nothing to do
    }

    memset(o, 0, FL_TQ_SYNC_REPLY);
    const uint64_t now = flTqHookNowUs();
    o[0] = outcome;
    o[1] = FL_TQ_VERSION;
    o[2] = g.armed;
    o[3] = flTqHookLinkOn() ? 1 : 0;
    put32(&o[4], g.boot_id);
    put32(&o[8], host_tag);
    put16(&o[12], g.generation);
    o[14] = state_flags();
    o[15] = FL_TQ_PER_EXCHANGE;
    put64(&o[16], rx);
    put16(&o[32], g.q_used);
    put16(&o[34], FL_TQ_QCAP);
    put16(&o[36], FL_TQ_RESERVE);
    put16(&o[38], FL_TQ_FLOOR_US);
    put64(&o[40], next_due(now));
    put64(&o[48], g.bus_free_at_us);
    uint32_t window = 0xFFFFFFFFu;
    if (g.armed && g.q_used)
    {
        const uint64_t nd = next_due(now);
        window = nd > now + FL_TQ_EXEC_GUARD_US ? (uint32_t)(nd - now - FL_TQ_EXEC_GUARD_US) : 0;
    }
    put32(&o[56], window);
    put16(&o[60], g.armed ? g.lease_ms : 0);
    put16(&o[62], g.coalesce_us);
    put16(&o[64], FL_TQ_MAX_MSGS);
    put16(&o[66], FL_TQ_MAX_AHEAD_MS);
    put32(&o[68], g.frames_dropped_boot);
    put16(&o[72], g.list_n);
    put64(&o[24], flTqHookNowUs());   // board_tx_us: last, immediately before the reply is handed to USB
    return FL_TQ_SYNC_REPLY;
}

/*************************
 * RELEASE (§5b)         *
 *************************/

static uint32_t tq_release(const uint8_t *p, uint32_t len, bool bridge, uint8_t *o)
{
    uint8_t outcome = FL_TQ_O_OK, fire = 2, ons = 0, offs = 0, sent = 0;
    uint64_t t0 = 0, t1 = 0;
    const uint32_t host_tag = len >= FL_TQ_RELEASE_HEAD ? get32(&p[8]) : 0;
    if (!bridge) outcome = FL_TQ_O_NOT_INIT;
    else
    {
        const uint8_t k = len >= 2 ? p[1] : 0;
        bool bad = len < FL_TQ_RELEASE_HEAD || p[0] != FL_TQ_VERSION || k > FL_TQ_MAX_KEYS ||
                   (p[4] & 0xFC) || p[5] != 0 || get16(&p[6]) != 0 ||
                   get32(&p[12]) != 2u * k || len != FL_TQ_RELEASE_HEAD + 2u * k;
        const uint8_t *keys = &p[FL_TQ_RELEASE_HEAD];
        for (uint8_t i = 0; !bad && i < k; i++)
            if (keys[i * 2] > 15 || keys[i * 2 + 1] > 127) bad = true;
        if (bad) outcome = FL_TQ_O_BAD_REQUEST;
        else if (!flTqHookLinkOn()) outcome = FL_TQ_O_LINK_OFF;
        else
        {
            const uint16_t mask = get16(&p[2]);
            const uint8_t flags = p[4];
            // 1 · remove the named keys' queued note-ons and note-offs (controllers stay).
            for (int i = (int)g.q_used - 1; i >= 0; i--)
            {
                const tq_slot_t s = q[i];
                if (!msg_is_on(s.st, s.d2) && !msg_is_off(s.st, s.d2)) continue;
                const uint8_t ch = s.st & 0x0F;
                bool hit = (mask >> ch) & 1u;
                for (uint8_t j = 0; !hit && j < k; j++) hit = keys[j * 2] == ch && keys[j * 2 + 1] == s.d1;
                if (!hit) continue;
                if (msg_is_on(s.st, s.d2)) { if (ons < 255) ons++; } else if (offs < 255) offs++;
                q_remove_at((uint16_t)i);
            }
            // 2 · fire: pedal-ups for every named channel, the named keys' note-offs, then each masked
            // channel's listed keys and its all-notes-off (ALL_OFF, or the list overflowed there), then
            // whatever the board still owes (§10: pending releases are retried at every RELEASE).
            uint16_t named = mask;
            for (uint8_t j = 0; j < k; j++) named |= (uint16_t)(1u << keys[j * 2]);
            rel_n = 0;
            if (flags & 1) rel_add_pedal_ups(named);
            rel_add_pedal_ups(g.pend_pedal);
            for (uint8_t j = 0; j < k; j++) rel_add_once((uint8_t)(0x80 | keys[j * 2]), keys[j * 2 + 1], 0);
            for (int i = 0; i < g.list_n; i++)
            {
                const uint16_t e = g.list[i];
                const uint8_t ch = (uint8_t)((e >> 7) & 0x0F);
                if (((mask >> ch) & 1u) || (e & LE_PENDING)) rel_add_once((uint8_t)(0x80 | ch), (uint8_t)(e & 0x7F), 0);
            }
            rel_add_all_offs((uint16_t)(((flags & 2) ? mask : 0) | (mask & g.overflow) | g.pend_7b));
            fire = rel_send(&sent, &t0, &t1);
            if (g.c.releases_served != 0xFFFF) g.c.releases_served++;
        }
    }
    memset(o, 0, FL_TQ_RELEASE_REPLY);
    o[0] = outcome;
    o[1] = outcome == FL_TQ_O_OK ? fire : 2;
    put16(&o[2], g.generation);
    put32(&o[4], g.boot_id);
    put32(&o[8], host_tag);
    o[12] = ons;
    o[13] = offs;
    o[14] = sent;
    o[15] = state_flags();
    put16(&o[16], g.q_used);
    put32(&o[20], t1 > t0 ? (uint32_t)(t1 - t0) : 0);
    return FL_TQ_RELEASE_REPLY;
}

/*************************
 * TQ_STATUS (§7)        *
 *************************/

static uint32_t tq_status(const uint8_t *p, uint32_t len, uint8_t *o)
{
    const uint8_t flags = len >= 1 ? p[0] : 0;
    const uint8_t outcome = (len != 1 || (flags & 0xFE)) ? FL_TQ_O_BAD_REQUEST : FL_TQ_O_OK;
    const tq_counters_t *c = &g.c;
    memset(o, 0, FL_TQ_STATUS_FIXED);
    o[0] = outcome;
    o[1] = FL_TQ_VERSION;
    o[2] = g.late_n;
    o[3] = flags;
    put32(&o[4], g.boot_id);
    put64(&o[8], flTqHookNowUs());
    put16(&o[16], g.generation);
    o[18] = state_flags();
    o[19] = g.fr_n;
    put32(&o[20], c->frames_admitted);  put32(&o[24], c->msgs_admitted);
    put32(&o[28], c->msgs_fired);       put32(&o[32], c->fires);
    put32(&o[36], c->msgs_late);        put32(&o[40], c->msgs_slow);
    put32(&o[44], c->late_log_overflow);
    put32(&o[48], c->max_late_us);      put32(&o[52], c->max_early_us);
    put32(&o[56], c->max_select_us);
    put32(&o[60], c->frames_refused_full); put32(&o[64], c->frames_refused_other);
    put32(&o[68], c->frames_duplicate); put32(&o[72], c->frames_cancelled);
    put32(&o[76], c->frames_dropped);   put32(&o[80], c->msgs_flushed);
    put32(&o[84], c->msgs_dropped_late);
    for (int i = 0; i < 8; i++) put16(&o[88 + 2 * i], c->drops[i]);
    put32(&o[104], c->releases_sent);
    put32(&o[108], c->fire_failures);   put32(&o[112], c->exec_held);
    put16(&o[116], c->exec_overran);    put16(&o[118], c->queue_high_water);
    put16(&o[120], c->max_receipt_to_queued_us);
    put16(&o[122], g.list_n);
    put16(&o[124], cleanup_pending());
    put16(&o[126], g.overflow);
    put16(&o[128], g.sustain);
    put16(&o[130], c->releases_served);
    put32(&o[132], c->log_saves_deferred);
    o[136] = g.fr_cause;
    o[137] = g.fr_more;
    put16(&o[138], g.fr_gen);
    put16(&o[140], g.fr_frames);
    o[142] = g.fr_released;
    {
        const uint16_t sn = g.stranded ? cleanup_pending() : 0;
        o[143] = (uint8_t)(sn > 255 ? 255 : sn);     // review 1: stranded obligations (spec: reserved)
    }
    uint32_t n = FL_TQ_STATUS_FIXED;
    for (uint8_t i = 0; i < g.fr_n; i++, n += 4)
    {
        put16(&o[n], g.fr_batch[i]);
        o[n + 2] = g.fr_unfired[i];
        o[n + 3] = 0;
    }
    for (uint8_t i = 0; i < g.late_n; i++, n += 12)
    {
        put16(&o[n], g.late[i].batch);
        o[n + 2] = g.late[i].idx;
        o[n + 3] = g.late[i].cause;
        put32(&o[n + 4], g.late[i].late_us);
        put32(&o[n + 8], g.late[i].select_us);
    }
    // ⭐ B10-FATE (midi, 2026-10-12): the CLEAR-READ COUNT, a u16 TRAILER after the late records, so a host can
    // see a counter clear made by ANY reader between its baseline and its question (generation does not move on
    // a clear). It already counts THIS read when this read clears. Not in g.c, so a clear never resets it; a
    // reboot does (with a new boot_id). Additive: a reader that stops after the late records never sees it.
    const bool clearing = outcome == FL_TQ_O_OK && (flags & 1);
    put16(&o[n], (uint16_t)(g.clear_reads + (clearing ? 1 : 0)));
    n += 2;
    if (clearing)
    {
        g.clear_reads++;
        // Atomic with the reply: the board is single-threaded. The list and the state flags stay.
        memset(&g.c, 0, sizeof(g.c));
        g.c.queue_high_water = g.q_used;
        g.late_n = 0;
        g.fr_n = 0; g.fr_cause = 0; g.fr_more = 0; g.fr_gen = 0; g.fr_frames = 0; g.fr_released = 0;
    }
    return n;
}

uint32_t flTqFrame(uint8_t type, const uint8_t *p, uint32_t len, uint64_t rx_us, bool bridge, uint8_t *out)
{
    tq_ensure();
    tq_unreach_clear();
    switch (type)
    {
        case FL_TQ_T_NOTE_AT: return tq_note_at(p, len, rx_us, bridge, out);
        case FL_TQ_T_SYNC:    return tq_sync(p, len, rx_us, bridge, out);
        case FL_TQ_T_STATUS:  return tq_status(p, len, out);
        case FL_TQ_T_RELEASE: return tq_release(p, len, bridge, out);
        default: out[0] = 0xFF; return 1;
    }
}

#endif // FORGE_LINK_NOTE_AT || FL_NOTEAT_HOST
