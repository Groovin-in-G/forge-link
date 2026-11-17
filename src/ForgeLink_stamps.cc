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

// DSP2 · THE STAMP RING - packing and the host's read, in FLASH (bench only, -DFORGE_LINK_STAMPS).
//
// ⛔ WHY A .cc: the linker template sends the .text and .rodata of every *.cpp.o to RAM and leaves
// everything else in flash (rp23xx-template.ld; ForgeLink_linkguard.cc and ForgeLink_hwdog_cold.cc
// are .cc for the same reason). The unified build's free heap is a few hundred bytes over the
// 29,500 B floor, and .bss starts on a 256-byte boundary after .data. Nothing here is on the
// IRQ path or inside a measured interval: forgeLinkStampsCommit runs in the main loop AFTER the
// dispatch (the reply has gone out), the read handler is a host frame of its own. The stamps
// themselves are inline stores in the RAM-resident callers (ForgeLink_stamps.h).
//
// The ring's RAM (.bss) is what this build pays for: FL_STAMPS_RING x 36 B + the work record.
// The env pays it back by halving the log ring (platformio.ini, ForgeLink_Blaster_unified_stamps).

#include "ForgeLink_stamps.h"

#ifdef FL_STAMPS_ON

#include <Arduino.h>
#include <string.h>
#include "ForgeLink_probe.h"
#include "ForgeLink_frames.h"
#include "ZuluSCSI_log.h"

static_assert((FL_STAMPS_RING & (FL_STAMPS_RING - 1)) == 0, "FL_STAMPS_RING must be a power of two");
static_assert(FL_STAMPS_HDR_LEN + FL_STAMPS_RING * FL_STAMPS_REC_LEN <= FORGE_LINK_MAX_DATA,
              "the whole ring must fit one reply");

fl_stamp_work_t g_fl_st;

static uint8_t  g_fl_st_ring[FL_STAMPS_RING][FL_STAMPS_REC_LEN];
static uint32_t g_fl_st_total;     // records committed since boot or the last clear
static uint8_t  g_fl_st_filter;    // FL_STAMPS_F_*, set by the host's read request

static const char k_fl_st_marker[] = FL_STAMPS_MARKER;

static inline void put16(uint8_t *p, uint32_t v)
{
    if (v > 0xFFFFu) v = 0xFFFFu;
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

static inline void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// Pack the frame just dispatched into the ring, if an EXEC handler opened it. Always closes the
// work record, so the next frame starts clean. Main loop, after fl_dispatch.
extern "C" void forgeLinkStampsCommit(void)
{
    if (!g_fl_st.open)
    {
        g_fl_st.mask = 0;
        return;
    }
    const uint8_t hi = (uint8_t)(g_fl_st.out0 & 0xF0);
    const bool note = g_fl_st.cdb0 == 0x0C && g_fl_st.sent > 0 && (hi == 0x80 || hi == 0x90);
    if ((g_fl_st_filter == FL_STAMPS_F_NOTES && !note) ||
        (g_fl_st_filter == FL_STAMPS_F_NOTE_ON && !(note && hi == 0x90)))
    {
        g_fl_st.open = 0;
        g_fl_st.mask = 0;
        return;
    }
    uint8_t *r = g_fl_st_ring[g_fl_st_total & (FL_STAMPS_RING - 1)];
    memset(r, 0, FL_STAMPS_REC_LEN);
    uint16_t mask = 0;
    bool have_prev = false, sat = false;
    uint32_t prev = 0;
    for (int k = 0; k < FL_ST_N; k++)
    {
        if (!(g_fl_st.mask & (1u << k))) continue;
        const uint32_t t = g_fl_st.t[k];
        mask |= (uint16_t)(1u << k);
        if (!have_prev)
        {
            // The anchor. Slot 0 when present; otherwise the first present slot, whose own delta
            // (if it has one, k >= 1) is written as 0: it IS the anchor.
            put32(&r[0], t);
            have_prev = true;
        }
        else
        {
            const uint32_t d = t - prev;   // wraps correctly across the 71-minute 32-bit roll
            if (d >= 0xFFFFu) sat = true;
            put16(&r[4 + 2 * (k - 1)], d);
        }
        prev = t;
    }
    if (sat) mask |= 0x8000u;
    put16(&r[20], mask);
    r[22] = g_fl_st.type;
    r[23] = g_fl_st.seq;
    r[24] = g_fl_st.flags;
    r[25] = g_fl_st.outcome;
    r[26] = g_fl_st.status;
    r[27] = g_fl_st.cdb0;
    r[28] = g_fl_st.out0;
    const uint32_t data = g_fl_st.sent + g_fl_st.received;
    const uint32_t ctrl = g_fl_st.handshakes > data ? g_fl_st.handshakes - data : 0;
    r[29] = (uint8_t)(ctrl > 255 ? 255 : ctrl);
    put16(&r[30], g_fl_st.len);
    put16(&r[32], g_fl_st.sent);
    put16(&r[34], g_fl_st.received);
    g_fl_st_total++;
    g_fl_st.open = 0;
    g_fl_st.mask = 0;
}

// TQ4b · a board-made 0C exchange (a NOTE_AT fire, a RELEASE, a flush's clean-up) as one ring record,
// layout in ForgeLink_stamps.h (FL_STAMPS_TYPE_FIRE). Called by flTqHookExchange after the exchange
// has ended, so nothing here is inside the exchange it records. g_fl_st is never touched.
extern "C" void forgeLinkStampsFire(uint64_t arb_us, uint32_t select_us, uint64_t end_us, const uint8_t *midi,
                                    uint32_t n, uint8_t outcome)
{
    const uint8_t out0 = n ? midi[0] : 0;
    const uint8_t hi = (uint8_t)(out0 & 0xF0);
    const bool note = n > 0 && (hi == 0x80 || hi == 0x90);
    if ((g_fl_st_filter == FL_STAMPS_F_NOTES && !note) ||
        (g_fl_st_filter == FL_STAMPS_F_NOTE_ON && !(note && hi == 0x90)))
        return;
    uint8_t *r = g_fl_st_ring[g_fl_st_total & (FL_STAMPS_RING - 1)];
    memset(r, 0, FL_STAMPS_REC_LEN);
    const uint32_t t0 = (uint32_t)arb_us;
    uint16_t mask = (uint16_t)(1u << FL_ST_ARB);
    uint32_t prev = t0;
    bool sat = false;
    put32(&r[0], t0);                                   // the anchor: ARB, the first present slot
    if (outcome != FL_O_NO_SELECT)
    {
        const uint32_t d = select_us;
        if (d >= 0xFFFFu) sat = true;
        put16(&r[4 + 2 * (FL_ST_SEL - 1)], d);
        mask |= (uint16_t)(1u << FL_ST_SEL);
        prev = t0 + select_us;
    }
    const uint32_t d = (uint32_t)end_us - prev;         // wraps correctly across the 32-bit roll
    if (d >= 0xFFFFu) sat = true;
    put16(&r[4 + 2 * (FL_ST_FREE - 1)], d);
    mask |= (uint16_t)(1u << FL_ST_FREE);
    if (sat) mask |= 0x8000u;
    put16(&r[20], mask);
    r[22] = FL_STAMPS_TYPE_FIRE;
    r[23] = 0;
    r[24] = n > 1 ? midi[1] : 0;                        // the first message's data byte 1 (40 vs 7B)
    r[25] = outcome;
    r[26] = 0xFF;
    r[27] = 0x0C;
    r[28] = out0;
    r[29] = 0;
    put16(&r[30], n);
    put16(&r[32], outcome == FL_O_COMPLETED ? n : 0);
    put16(&r[34], 0);
    g_fl_st_total++;
}

// FL_T_STAMPS. Answers the whole ring, oldest record first, behind the 20-byte header described in
// ForgeLink_stamps.h. Reads nothing from the bus and changes nothing but (on request) the ring.
// ⛔ SAFE MODE: refused by the bridge's gate in fl_dispatch like every verb that is not HELLO, HWDOG,
// EXIT or REBOOT_UF2 - no exception was added for it (and in safe mode no EXEC ran to stamp).
extern "C" void forgeLinkStampsFrame(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                                     const uint8_t *p, uint32_t len)
{
    const uint8_t want = (len >= 1) ? p[0] : 0;
    if ((want & FL_STAMPS_SET_FILTER) && len >= 2 && p[1] <= FL_STAMPS_F_NOTE_ON) g_fl_st_filter = p[1];
    const uint32_t count = g_fl_st_total < FL_STAMPS_RING ? g_fl_st_total : FL_STAMPS_RING;
    const uint32_t first = g_fl_st_total - count;
#ifdef FORGE_LINK_BULK
    // The previous reply may still be leaving from these bytes on the vendor IN endpoint.
    flFramesWaitBulkTxIdle();
#endif
    uint8_t *b = flFramesReplyPayloadAt(0);
    b[0] = FL_STAMPS_VERSION;
    b[1] = FL_STAMPS_REC_LEN;
    put16(&b[2], FL_STAMPS_RING);
    put16(&b[4], count);
    put16(&b[6], ((want & FL_STAMPS_CLEAR) ? 1u : 0u) | ((uint32_t)g_fl_st_filter << 8));
    put32(&b[8], g_fl_st_total);
    const uint64_t now = time_us_64();
    put32(&b[12], (uint32_t)now);
    put32(&b[16], (uint32_t)(now >> 32));
    for (uint32_t i = 0; i < count; i++)
        memcpy(&b[FL_STAMPS_HDR_LEN + i * FL_STAMPS_REC_LEN],
               g_fl_st_ring[(first + i) & (FL_STAMPS_RING - 1)], FL_STAMPS_REC_LEN);
    const uint32_t n = FL_STAMPS_HDR_LEN + count * FL_STAMPS_REC_LEN;
    if (want & FL_STAMPS_CLEAR) g_fl_st_total = 0;
    flFramesSendReply(FL_T_STAMPS, seq, req_crc, req_len, nullptr, 0, b, n);
}

extern "C" void forgeLinkStampsHelloLine(void)
{
    logmsg(k_fl_st_marker, "; ", (int)g_fl_st_total, " record(s) since boot/clear, ring ",
           (int)FL_STAMPS_RING);
}

#endif // FL_STAMPS_ON

// ============================================================================================================
// RX-SLOW · THE RECEIVE-PATH EVENT RING - storage and the host's read, in FLASH (bench only, -DFL_RXSTAMP).
// It lives in THIS .cc (not a file of its own) because a new object - even an empty one - changes the ship
// build's link layout; the block below is preprocessed away there. Flash, like the stamps code above: nothing
// here is on an IRQ path. The pushes are always_inline in their callers (ForgeLink_rxstamp.h).

#ifdef FL_RXSTAMP
#include "ForgeLink_rxstamp.h"
#endif

#ifdef FL_RXSTAMP_ON

#include <Arduino.h>
#include <string.h>
#include "ForgeLink_probe.h"
#include "ForgeLink_frames.h"
#include "ZuluSCSI_log.h"

static_assert((FL_RXSTAMP_RING & (FL_RXSTAMP_RING - 1)) == 0, "FL_RXSTAMP_RING must be a power of two");
static_assert(sizeof(fl_rxs_ev_t) == FL_RXSTAMP_EV_LEN, "the event is a wire layout");
static_assert(FL_RXS_N <= 32, "the event mask is a u32");

fl_rxs_ev_t g_fl_rxs_ring[FL_RXSTAMP_RING];
// Boot: recording ON, every code. A run normally arms it first (CLEAR | RESUME, its own mask).
fl_rxs_state_t g_fl_rxs_state = { 0, 0xFFFFFFFEu, 1 };

static const char k_fl_rx_marker[] = FL_RXSTAMP_MARKER;

static inline void rx_put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void rx_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint32_t rx_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// FL_T_RXSTAMP (layout in ForgeLink_rxstamp.h). Reads nothing from the bus. Main loop (the dispatcher).
// ⛔ SAFE MODE: refused by the bridge's gate in fl_dispatch like every verb but HELLO/HWDOG/EXIT/REBOOT_UF2.
extern "C" void forgeLinkRxStampFrame(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                                      const uint8_t *p, uint32_t len)
{
    const uint8_t flags = len >= 1 ? p[0] : 0;
    uint32_t start = len >= 5 ? rx_get32(&p[1]) : 0;
    uint32_t want = len >= 7 ? ((uint32_t)p[5] | ((uint32_t)p[6] << 8)) : 0;

    if (flags & FL_RXSTAMP_PAUSE) g_fl_rxs_state.on = 0;
    if ((flags & FL_RXSTAMP_SET_MASK) && len >= 11)
    {
        const uint32_t irq = save_and_disable_interrupts();
        g_fl_rxs_state.mask = rx_get32(&p[7]);
        restore_interrupts(irq);
    }
    // While recording, the read's own events land too; mark it so the host can drop them.
    FL_RX(FL_RXS_READ, flags, seq);

#ifdef FORGE_LINK_BULK
    // The previous reply may still be leaving from these bytes on the vendor IN endpoint.
    flFramesWaitBulkTxIdle();
#endif
    const uint32_t irq = save_and_disable_interrupts();
    const uint32_t total = g_fl_rxs_state.total;
    restore_interrupts(irq);
    const uint32_t held = total < FL_RXSTAMP_RING ? total : FL_RXSTAMP_RING;
    const uint32_t oldest = total - held;
    if (start < oldest) start = oldest;
    if (start > total) start = total;
    const uint32_t room = (FORGE_LINK_MAX_DATA - FL_RXSTAMP_HDR_LEN) / FL_RXSTAMP_EV_LEN;
    uint32_t count = total - start;
    if (count > want) count = want;
    if (count > room) count = room;

    uint8_t *b = flFramesReplyPayloadAt(0);
    b[0] = FL_RXSTAMP_VERSION;
    b[1] = FL_RXSTAMP_EV_LEN;
    rx_put16(&b[2], FL_RXSTAMP_RING);
    rx_put16(&b[4], count);
    rx_put32(&b[8], total);
    rx_put32(&b[12], start);
    const uint64_t now = time_us_64();
    rx_put32(&b[16], (uint32_t)now);
    rx_put32(&b[20], (uint32_t)(now >> 32));
    // ⛔ While recording is ON an IRQ can overwrite the oldest slots mid-copy; a host that wants a clean
    // read PAUSEs first (scripts/rx-slow.py does). Each event is copied masked so none is torn.
    for (uint32_t i = 0; i < count; i++)
    {
        const uint32_t m = save_and_disable_interrupts();
        memcpy(&b[FL_RXSTAMP_HDR_LEN + i * FL_RXSTAMP_EV_LEN], &g_fl_rxs_ring[(start + i) & (FL_RXSTAMP_RING - 1)],
               FL_RXSTAMP_EV_LEN);
        restore_interrupts(m);
    }
    if (flags & FL_RXSTAMP_CLEAR)
    {
        const uint32_t m = save_and_disable_interrupts();
        g_fl_rxs_state.total = 0;
        restore_interrupts(m);
    }
    if (flags & FL_RXSTAMP_RESUME) g_fl_rxs_state.on = 1;
    rx_put16(&b[6], (g_fl_rxs_state.on ? 1u : 0u) | ((flags & FL_RXSTAMP_CLEAR) ? 2u : 0u));
    flFramesSendReply(FL_T_RXSTAMP, seq, req_crc, req_len, nullptr, 0, b,
                      FL_RXSTAMP_HDR_LEN + count * FL_RXSTAMP_EV_LEN);
}

extern "C" void forgeLinkRxStampHelloLine(void)
{
    logmsg(k_fl_rx_marker, "; ", (int)g_fl_rxs_state.total, " event(s) since boot/clear, ring ",
           (int)FL_RXSTAMP_RING, g_fl_rxs_state.on ? ", recording" : ", paused");
}

#endif // FL_RXSTAMP_ON
