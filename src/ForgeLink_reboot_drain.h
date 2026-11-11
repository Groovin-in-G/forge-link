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

// THE DRAIN BEFORE A REQUESTED REBOOT, AND ITS BOUND FOR THE UF2 ESCAPE HATCH (A14.12). Plain C, no SDK.
//
// spin_for_reboot (ZuluSCSI.cpp) waits for the sampler's SCSI write / read to finish before it closes the
// images and resets. For a plain REBOOT that wait stays UNBOUNDED, as upstream wrote it. ⛔ For a UF2 reboot
// (vendor BOOTSEL, the REBOOT_UF2 frame, the console UF2 key) it is CAPPED at FL_UF2_DRAIN_CAP_MS: those are
// the escape hatches for a WEDGED board, and Blaster #1 sits inside the S1000 with no reachable BOOTSEL
// button, so a sampler holding the bus must never make the bootloader unreachable. The hardware watchdog is
// already disarmed by then (deliberately), so this cap is the only thing that ends the wait.
// Returns false when cut short: the caller logs it and resets anyway.
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define FL_UF2_DRAIN_CAP_MS 2000u

static inline bool flRebootDrainWait(bool bounded, uint32_t start_ms, bool (*finished)(void),
                                     uint32_t (*now_ms)(void), void (*kick)(void))
{
    while (!finished())
    {
        kick();
        if (bounded && (uint32_t)(now_ms() - start_ms) >= FL_UF2_DRAIN_CAP_MS) return false;
    }
    return true;
}
