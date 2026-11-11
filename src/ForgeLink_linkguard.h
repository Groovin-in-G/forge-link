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

// THE LINK GUARD's two hooks, called from the bridge in ForgeLink_probe.cpp. Both run in
// THREAD MODE from the main loop and nowhere else - never from platform_poll, never from an IRQ.
// Rules: ForgeLink_linkguard_policy.h. Glue (flash): ForgeLink_linkguard.cc.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// fl_handle_exec, after the chain latch: the EXEC request as received and whether it succeeded.
void forgeLinkLinkGuardNoteExec(const uint8_t *payload, uint32_t len, bool success);

// forgeLinkProbeMainLoop, once per pass AFTER the dispatch: fed = this pass served a host frame.
// Stamps the host clock when fed; otherwise reads USB and, if the policy says so, sends 09 off.
void forgeLinkLinkGuardPoll(bool fed);

#if defined(FORGE_LINK_PROBE)
// U1 · THE USB LOSS GENERATIONS, defined in ForgeLink_probe.cpp.
//
// WHY. pico-sdk's dcd_rp2040.c forces VBUS detect on (FORCE_VBUS_DETECT 1), so on a board powered
// from the SCSI bus a pulled USB-C cable NEVER unmounts: tud_mounted() stays true and DTR never
// falls (and the vendor-bulk link never raises DTR at all). The controller only sees the bus go idle,
// which TinyUSB reports as a SUSPEND. measured the consequence: the NOTE_AT queue was flushed by
// its 9 s lease instead of at once, and every queued note fired after the pull. A Mac going to sleep
// suspends the bus the same way. A replug is a bus reset + a new SET_CONFIGURATION, and the unmounted
// interval between them can fall between two main-loop looks.
//
// So the strong tud_suspend_cb / tud_mount_cb each bump a LOSS GENERATION. Only a CHANGE means
// anything; several suspends (or mounts) between two looks coalesce into one change. Each consumer
// keeps the value it last saw, so neither clears the event for the other. The LEVEL, tud_suspended(),
// is read beside them. Written only inside tud_task (single writer, under USB.mutex); read by the
// main loop; a 32-bit aligned load is single-copy atomic on Cortex-M33.
extern volatile uint32_t g_fl_usb_suspend_gen;
extern volatile uint32_t g_fl_usb_mount_gen;
#endif

#ifdef FORGE_LINK_NOTE_AT
// TQ2 (docs/note-at.md §4 link_on): the guard's memory - the last 09 this board carried was LINK ON.
bool forgeLinkLinkGuardIsOn(void);
#endif

#ifdef __cplusplus
}
#endif
