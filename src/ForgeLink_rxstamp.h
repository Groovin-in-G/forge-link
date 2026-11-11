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

// ⛔ RX-SLOW · A BOARD-CLOCK EVENT RING FOR THE VENDOR-BULK RECEIVE PATH - A BENCH-ONLY INSTRUMENT (-DFL_RXSTAMP).
//
// WHY: a host write of a command block over vendor bulk stalls ~5-10 ms
// about once every ~0.6 s WITH an rx slot free. Whether the board held the OUT endpoint un-armed (tud_task late,
// the main loop busy in a log flush / SD work) or the host sat on bytes the board was ready for cannot be told
// from the host. This build stamps, in board microseconds, every point on that path and every periodic board job
// that could block it, into a .bss ring the host reads AFTER the run (FL_T_RXSTAMP, scripts/rx-slow.py).
//
// ⛔ NEVER SHIPS. tools/fl_release_gate.py refuses the FL_RXSTAMP flag, the HELLO ident's rxstamp:on token and
// the FL_RXSTAMP_MARKER string below. Without -DFL_RXSTAMP every FL_RX* macro is an empty statement and this
// header is never included (every include and call site is #ifdef FL_RXSTAMP), so the ship build is byte-identical.
//
// ⛔ IRQ-SAFE AND RAM-ONLY BY CONSTRUCTION. A stamp is pushed from the USB IRQ (src/tinyusb_override/rp2040_usb.c),
// from tud_task (the 1 ms alarm IRQ) and from the main loop, so every push is one masked section of plain loads and
// stores: the TIMER0 raw low word read straight from the register (no time_us_32 call, which a .c file could emit
// out of line in flash), and the SDK's __force_inline interrupt mask. The push is always_inline into its caller,
// so it lives wherever the caller lives (the IRQ path is RAM; tools/check-irq-ram.sh proves it on this ELF).

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef FL_RXSTAMP
#define FL_RXSTAMP_ON 1

#include "hardware/sync.h"
#include "hardware/timer.h"

// Frame type, host -> board (bench only; a build without the flag answers it FF like any unknown type).
// REQUEST (all little-endian; every field optional, a shorter payload leaves the rest at 0):
//   0 u8  flags: bit0 CLEAR the ring (total = 0) AFTER building this reply · bit1 PAUSE recording BEFORE building
//               it · bit2 RESUME recording AFTER it · bit3 SET the event mask to bytes 7-10
//   1 u32 start: the absolute event index (counted from the last clear) of the first event wanted; an index
//               already overwritten is moved up to the oldest one held (the reply's `first` says where it began)
//   5 u16 max:   events wanted (0 = none, header only; capped by the reply room)
//   7 u32 mask:  bit k = record event code k (with flag bit3)
// REPLY: a 24-byte header, then `count` events of FL_RXSTAMP_EV_LEN bytes, oldest first.
//   0 u8  version (1)    1 u8 event length (8)    2 u16 ring capacity (events)    4 u16 count returned
//   6 u16 state: bit0 recording on (after this request) · bit1 the ring was cleared after this read
//   8 u32 total events pushed since boot / the last clear (total - capacity = overwritten, when positive)
//  12 u32 first: the absolute index of the first returned event
//  16 u64 the board's time_us_64() as the reply is built (the host unwraps the u32 stamps on it)
// EVENT (8 B): 0 u32 t (board us, TIMER0 low word) · 4 u8 code · 5 u8 a8 · 6 u16 a16 (saturating)
#define FL_T_RXSTAMP 0x0E

#ifndef FL_RXSTAMP_RING
#define FL_RXSTAMP_RING 2048        // events; a power of two (16 KiB of .bss)
#endif
#define FL_RXSTAMP_EV_LEN   8
#define FL_RXSTAMP_HDR_LEN  24
#define FL_RXSTAMP_VERSION  1
#define FL_RXSTAMP_CLEAR    0x01
#define FL_RXSTAMP_PAUSE    0x02
#define FL_RXSTAMP_RESUME   0x04
#define FL_RXSTAMP_SET_MASK 0x08

// The event codes. Second copy: scripts/rx-slow.py (EVENTS). Never renumber.
enum {
    FL_RXS_OUT_IRQ   = 1,   // USB IRQ: an OUT transfer completed in hardware. a8 = ep address, a16 = bytes
    FL_RXS_IN_IRQ    = 2,   // USB IRQ: an IN transfer completed (the host took it). a8 = ep address, a16 = bytes
    FL_RXS_COMMIT    = 3,   // tud_task: a bulk request handed to the rx slots. a8 = frame type, a16 = seq
    FL_RXS_CB_ARM    = 4,   // the bulk OUT endpoint armed for the next command block. a8 = 1 in an IRQ, 0 thread
    FL_RXS_ARM_SKIP  = 5,   // a command-block arm skipped: a refusal reply still sits in the stash
    FL_RXS_PARK      = 6,   // a command block parked (OUT NOT re-armed). a8 = 1 CDC owns the slots, 2 slots full,
                           //   3 data-out EXEC with slots full. a16 = seq
    FL_RXS_PICK      = 7,   // main loop: a frame dispatched. a8 = type, a16 = seq
    FL_RXS_DONE      = 8,   // main loop: the dispatch returned (the slot is released next). a8 = type, a16 = seq
    FL_RXS_REPLY     = 9,   // flFramesSendReply entered. a8 = request type, a16 = seq
    FL_RXS_LOG_FLUSH = 10,  // the reply's platform_flush_usb_log (t = start). a16 = us it took (only when > 50 us)
    FL_RXS_LOOP_GAP  = 11,  // the bridge poll was not entered for a16 us after its last pass ended (t = that end;
                           //   recorded when > FL_RXSTAMP_GAP_US): the rest of the main loop held it
    FL_RXS_LOG_SD    = 12,  // save_logfile wrote the log to the SD card (t = start). a16 = us
    FL_RXS_PLAT_POLL = 13,  // the main loop's platform_reset_watchdog + platform_poll (the CDC log pump, serial
                            //   input, ADC) took a16 us (t = start; only when > 50 us)
    FL_RXS_SD_POLL   = 14,  // the main loop's SD card hot-plug poll (t = start). a16 = us
    FL_RXS_WDOG      = 15,  // RESERVED, never pushed: the 1 s watchdog alarm lives in ZuluSCSI_platform.cpp, whose
                            //   __LINE__-carrying assert()s would move the ship bytes (an IRQ that long shows as
                            //   OUT_IRQ -> COMMIT latency or a LOOP_GAP anyway)
    FL_RXS_READ      = 16,  // an FL_T_RXSTAMP request was served (the host's own reads, recognisable)
    FL_RXS_N
};

#ifndef FL_RXSTAMP_GAP_US
#define FL_RXSTAMP_GAP_US 300u
#endif

// The marker string the release gate looks for in the ELF and the UF2 payload (logged on every HELLO).
#define FL_RXSTAMP_MARKER "Forge Link RXSTAMP: bulk receive-path us ring, BENCH ONLY - this build never ships"

typedef struct {
    uint32_t t;
    uint8_t  code, a8;
    uint16_t a16;
} fl_rxs_ev_t;

typedef struct {
    volatile uint32_t total;   // events pushed since boot / the last clear
    volatile uint32_t mask;    // bit k = record code k
    volatile uint8_t  on;      // recording
} fl_rxs_state_t;

#ifdef __cplusplus
extern "C" {
#endif
extern fl_rxs_ev_t g_fl_rxs_ring[FL_RXSTAMP_RING];
extern fl_rxs_state_t g_fl_rxs_state;
// Flash code (the RX-SLOW block of ForgeLink_stamps.cc), main loop only.
void forgeLinkRxStampFrame(uint8_t seq, uint32_t req_crc, uint32_t req_len, const uint8_t *p, uint32_t len);
void forgeLinkRxStampHelloLine(void);
#ifdef __cplusplus
}
#endif

static inline __attribute__((always_inline)) uint32_t flRxNow(void)
{
    return timer_hw->timerawl;   // the same count time_us_32 returns; a register read, no call
}

static inline __attribute__((always_inline)) void flRxPushAt(uint32_t t, uint8_t code, uint8_t a8, uint32_t a16)
{
    const uint32_t irq = save_and_disable_interrupts();
    if (g_fl_rxs_state.on && (g_fl_rxs_state.mask & (1u << code)))
    {
        const uint32_t n = g_fl_rxs_state.total;
        fl_rxs_ev_t *e = &g_fl_rxs_ring[n & (FL_RXSTAMP_RING - 1)];
        e->t = t;
        e->code = code;
        e->a8 = a8;
        e->a16 = (uint16_t)(a16 > 0xFFFFu ? 0xFFFFu : a16);
        g_fl_rxs_state.total = n + 1;
    }
    restore_interrupts(irq);
}

static inline __attribute__((always_inline)) uint32_t flRxInIrq(void)
{
    uint32_t ipsr;
    __asm volatile ("mrs %0, ipsr" : "=r" (ipsr));
    return (ipsr & 0x1FFu) ? 1u : 0u;
}

#define FL_RX(code, a8, a16)          flRxPushAt(flRxNow(), (code), (uint8_t)(a8), (uint32_t)(a16))
#define FL_RX_AT(t, code, a8, a16)    flRxPushAt((t), (code), (uint8_t)(a8), (uint32_t)(a16))
#define FL_RX_NOW()                   flRxNow()

#else
// ⛔ Every other build: nothing. No call, no argument evaluated, no symbol.
#define FL_RX(code, a8, a16)          do {} while (0)
#define FL_RX_AT(t, code, a8, a16)    do {} while (0)
#endif
