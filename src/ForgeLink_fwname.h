/* ForgeLink_fwname.h - the ONE rule for which SD-card .bin is this board's firmware (GPLv3, part of Forge Link).
 * Shared by the bootloader that flashes it (find_firmware_image, ZuluSCSI_bootloader.cpp) and the main firmware's
 * "power cycle to update" notice (check_for_unused_update_files, ZuluSCSI.cpp), so the two can never disagree
 * (A14.10(b), 2026-10-27: after C28 the notice still matched by prefix). Header-only, static inline: no SdFat, no
 * logmsg, no platform calls; tools/tests/test_sd_updater_c28_c33.py compiles it on the Mac.
 */
#ifndef FORGELINK_FWNAME_H
#define FORGELINK_FWNAME_H
#include <stddef.h>
#include <strings.h>

// C28 (deep review): the file name the bootloader will flash. It used to accept any .bin
// whose name merely STARTED with this build's env name, so a longer-named bench env's image
// (ForgeLink_Blaster_inject_test_... on a ForgeLink_Blaster_inject board, or
// ForgeLink_Blaster_unified_nonoteat_... on a ForgeLink_Blaster_unified board) flashed onto it.
// Accepted now, case-insensitive, and nothing else:
//   <env>.bin                          what firmware_update() extracts from a ForgeLink-FW*.zip
//   <env>_YYYY-MM-DD_<version>.bin     a release .bin as utils/rename_binaries.sh names it
//                                      (<env>_$DATE_$(git describe --always)), <version> non-empty
// The date's digits are what separate the env name from the rest: no Forge Link env name has an
// underscore followed by a date, so a longer env name can never pass for this one.
static inline bool fl_is_digit(char c) { return c >= '0' && c <= '9'; }

static inline bool fl_fw_name_matches_env(const char *name, size_t namelen, const char *env, size_t envlen)
{
    if (namelen < envlen + 4) return false;
    if (strncasecmp(name, env, envlen) != 0) return false;
    if (strncasecmp(name + namelen - 4, ".bin", 4) != 0) return false;

    const char *rest = name + envlen;
    size_t restlen = namelen - envlen;   // everything after the env name, ".bin" included
    if (restlen == 4) return true;       // "<env>.bin"

    // "_YYYY-MM-DD_" + at least one version character + ".bin"
    if (restlen < 12 + 1 + 4) return false;
    if (rest[0] != '_' || rest[11] != '_') return false;
    for (int k = 1; k <= 10; k++)
    {
        if (k == 5 || k == 8)
        {
            if (rest[k] != '-') return false;
        }
        else if (!fl_is_digit(rest[k]))
        {
            return false;
        }
    }
    return true;
}

#endif
