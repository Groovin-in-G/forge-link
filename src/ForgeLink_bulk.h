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

// N6 STEP 3 · THE VENDOR-BULK CLASS DRIVER.
//
// ⛔ WHY THIS EXISTS AND WHY IT IS SHAPED LIKE THIS. The Mac->board CDC pipe measures 170 KiB/s
// against a 349 KB/s parity bar (the sampler's own disk speed). The obvious fix - double-buffering
// the CDC bulk OUT endpoint (verb 0x09, DBUF) - is BANNED: it wedged the board four times and its
// crash log was read, a panic inside the USB buffer-status interrupt with interrupts masked.
//
// So this driver puts NOTHING on the USB interrupt path. It is registered through TinyUSB's
// unclaimed weak hook usbd_app_driver_get_cb, and every callback below runs inside tud_task(),
// which on this core is a 1 ms alarm-triggered USER IRQ, not the USB hardware IRQ. Upstream's own
// cdcd_xfer_cb and mscd_xfer_cb live in flash for exactly the same reason. src/tinyusb_override/
// gets ZERO lines, and tools/check-irq-ram.sh refuses any fl_bulk_* symbol that reaches the
// derived IRQ-path list. That gate is the safety case; this comment is only its explanation.
//
// The transport is a SWAP UNDER THE STAGE-P SEAM, not a new protocol: the same frame format, the
// same two RX slots, the same WRITE run machinery. A command block arrives as exactly 64 bytes on
// bulk OUT; its data phase is armed DIRECTLY into a slot at offset 13, where a CDC-delivered
// WRITE_SECTORS payload's data would sit, so the slot ends up indistinguishable from a parsed CDC
// frame and nothing above the seam changes.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef FORGE_LINK_BULK

// Registered ONCE, early, before enumeration, so USB.disconnect() does not have to re-enumerate
// later (and so it skips its 500 ms settle). Called from the platform's USB init.
void forgeLinkBulkInit(void);

// Called from the main loop: releases the slots after an idle stretch with no command block,
// and retries an arm that was refused.
void forgeLinkBulkPoll(void);

// Card-reader mode: the interface stays enumerated and the driver STANDS DOWN. ~~One CARD_MODE_ACTIVE
// reply, nothing applied.~~ C41: NO reply at all (block-interface: no HELLO answer in card mode);
// every block, and its data phase, is swallowed and nothing is applied or left to leak onto CDC afterwards.
void forgeLinkBulkCardMode(bool entering);

// True once the host has claimed the interface and the slots are ours.
bool forgeLinkBulkActive(void);

// ⭐ ARMRETRY r7 · DATACRC: the board's current session, never 0 once the driver is up. Minted
// at every USB open, bus reset, BULK RESET and ARMRETRY abort; reported in HELLO and STATUS.
uint32_t forgeLinkBulkSession(void);

// The dispatchers' gate for the session-only verbs inside one frame (WRITE_SECTORS, SYNC, CARD_MODE,
// TQ_SYNC, TQ_STATUS), on either carrier. Returns 0 to go on - with *len cut back to the old exact
// length when the frame carried a session, so the handler reads the frame it always read - or
// FL_DC_STALE_SESSION (0x18), the one-byte refusal, when the session is not the current one. Main loop.
uint8_t forgeLinkBulkSessionGate(uint8_t type, const uint8_t *p, uint32_t *len);

#else

static inline void forgeLinkBulkInit(void) {}
static inline void forgeLinkBulkPoll(void) {}
static inline void forgeLinkBulkCardMode(bool entering) { (void)entering; }
static inline bool forgeLinkBulkActive(void) { return false; }
static inline uint32_t forgeLinkBulkSession(void) { return 0; }
static inline uint8_t forgeLinkBulkSessionGate(uint8_t type, const uint8_t *p, uint32_t *len)
{ (void)type; (void)p; (void)len; return 0; }

#endif // FORGE_LINK_BULK

#ifdef __cplusplus
}
#endif
