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

// ⭐ SHIP-1 (2026-10-12): the config file's name, picked once per card mount.
//
// forgelink.ini is read when the card carries one; otherwise the legacy zuluscsi.ini is, so cards
// written by the app today (and the bench cards) keep working unchanged. CONFIGFILE expands to
// g_fl_configfile (src/ZuluSCSI_config.h), so every ini_* call, every log line that names the file,
// and the settings cache (minIni_cache.cpp matches the cached file by name) all see the SAME file.
//
// A .cpp on purpose: *.cpp.o .text stays in flash (rp23xx-template.ld), nothing here is hot.

#include "ZuluSCSI_config.h"
#include "ZuluSCSI_log.h"
#include <SdFat.h>

extern SdFs SD;

extern "C" {

const char *g_fl_configfile = CONFIGFILE_NEW;

// Called from mountSDCard() (src/ZuluSCSI.cpp) right after the card mounts, before the settings
// cache loads. Returns the name it picked.
const char *forgelink_select_configfile(void)
{
    const bool have_new = SD.exists(CONFIGFILE_NEW);
    const bool have_legacy = !have_new && SD.exists(CONFIGFILE_LEGACY);
    g_fl_configfile = have_new ? CONFIGFILE_NEW : have_legacy ? CONFIGFILE_LEGACY : CONFIGFILE_NEW;
    // INI1 (2026-10-13): say which file this mount reads. SHIP-1 picked silently, so a card with a
    // stale or partial pair could boot good firmware on the wrong settings with nothing in the log
    // to show it (the VST's SHIP-1 pre-flight review).
    if (have_new)
        logmsg("Config file: ", CONFIGFILE_NEW);
    else if (have_legacy)
        logmsg("Config file: ", CONFIGFILE_LEGACY, " (legacy name; ", CONFIGFILE_NEW, " not on the card)");
    else
        logmsg("Config file: none on the card (", CONFIGFILE_NEW, " or ", CONFIGFILE_LEGACY, "), defaults in use");
    return g_fl_configfile;
}

}
