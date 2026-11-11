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

// THE ONE WAY INTO THE UF2 BOOTLOADER THAT THE FIRMWARE ASKS FOR ITSELF (deep review C90).
//
// Two routes reach it: the vendor BOOTSEL request (ForgeLink_bulk.cpp, fl_bulk_bootsel_poll) and the
// REBOOT_UF2 frame (ForgeLink_probe.cpp, the bridge role's only exit). ~~Each called reset_usb_boot(0, 0)
// itself~~: no wait for the sampler's SCSI write, no image close, no card sync. Both now COMMIT here and
// return; the next pass's spin_for_reboot (ZuluSCSI.cpp) drains (bounded for a UF2 reboot, A14.12:
// ForgeLink_reboot_drain.h), closes the images, syncs the card, and platform_reset_mcu (which disarms
// again) enters the ROM bootloader because the UF2 flag is set.
//
// ORDER (C44, then C31/): the link guard's note releases and LINK OFF first, while the hardware
// watchdog is still armed; then the deliberate disarm, only once committed; then the UF2 flag and
// g_rebooting. Thread mode only. The caller logs and flushes its own line first.
#pragma once

#include <ZuluSCSI_platform.h>    // platform_set_uf2_reboot
#include "ForgeLink_hwdog.h"      // forgeLinkHwdogDisarm (a no-op without FORGE_LINK_INJECT)
#ifdef FORGE_LINK_PROBE
#include "ForgeLink_linkguard.h"  // forgeLinkLinkGuardBeforeReset
#endif

extern bool g_rebooting;          // ZuluSCSI.cpp; acted on by spin_for_reboot at the top of zuluscsi_main_loop

static inline void forgeLinkUf2RebootCommit(const char *why)
{
#ifdef FORGE_LINK_PROBE
    forgeLinkLinkGuardBeforeReset(why);   // C44: bounded, best effort, still armed
#else
    (void)why;
#endif
    forgeLinkHwdogDisarm(true);           // deliberate: a requested reboot never counts toward SAFE MODE
    platform_set_uf2_reboot();
    g_rebooting = true;
}
