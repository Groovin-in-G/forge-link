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

// The hardware watchdog's state, shared by its two glue files and nothing else:
//   ForgeLink_hwdog.cpp     the HOT half, in RAM (every *.cpp.o is, rp23xx-template.ld:89):
//                           the feed, the crash feed, the disarm, the safe-mode test.
//   ForgeLink_hwdog_cold.cc the COLD half, in FLASH: boot, arm, card mode, HELLO bytes, the
//                           frames, the banner.
// ⛔ WHY THE .cc: the linker template sends the .text AND .rodata of every *.cpp.o to RAM and
// leaves everything else in flash (rp23xx-template.ld:89 and :308 exclude only `*.cpp.o`). The
// cold half is ~2.9 KB of code and banner strings that run once, and N6's RAM floor is a measured
// 34,964 B of free heap (N6-DESIGN.md rev 2: the stage-P RAM table and review row R-d). A .cc file
// is C++ like any other, compiled with the same flags - only its object's name keeps it out of
// RAM. Renaming it to .cpp moves ~2.9 KB back into RAM; tools/tests/test_fl_hwdog.py pins it.

#pragma once

#ifdef FORGE_LINK_INJECT

#include "ForgeLink_hwdog_policy.h"

extern fl_hwdog_boot_t g_fl_hwdog_boot;
extern fl_hwdog_rt_t   g_fl_hwdog_rt;
extern bool            g_fl_hwdog_booted;

uint8_t flHwdogHelloFlags(void);

#endif
