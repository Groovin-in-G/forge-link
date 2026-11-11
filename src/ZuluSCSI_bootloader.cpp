/**
 * ZuluSCSI™ - Copyright (c) 2022-2025 Rabbit Hole Computing™
 * Copyright (c) 2025-2026 Eric Helgeson <eric@bluescsi.com>
 * 
 * ZuluSCSI™ firmware is licensed under the GPL version 3 or any later version. 
 * 
 * https://www.gnu.org/licenses/gpl-3.0.html
 * ----
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/

// Simple bootloader that loads new firmware from SD card.

#include <ZuluSCSI_platform.h>
#include "ZuluSCSI_config.h"
#include "ZuluSCSI_log.h"
#include <string.h>
#include <SdFat.h>
#ifdef FORGE_LINK_INJECT
#include <hardware/watchdog.h>   // N6 step 2: the one watchdog_disable() at bootloader_main entry
#endif

#ifdef PLATFORM_BOOTLOADER_SIZE

extern SdFs SD;
extern FsFile g_logfile;

bool find_firmware_image(FsFile &file, char name[MAX_FILE_PATH + 1])
{
    FsFile root;
    root.open("/");

    while (file.openNext(&root, O_READ))
    {
        if (file.isDir()) continue;

        int namelen = file.getName(name, MAX_FILE_PATH);

        // SHIP-1: a Forge Link file family name (FIRMWARE_FAMILY_MATCH, ZuluSCSI_config.h - the same
        // macro the main firmware's check_for_unused_update_files() uses) AND this build env's name.
        if (namelen >= sizeof(FIRMWARE_NAME_PREFIX) + 3 &&
            FIRMWARE_FAMILY_MATCH(name) &&
            strncasecmp(name, FIRMWARE_NAME_PREFIX, sizeof(FIRMWARE_NAME_PREFIX) - 1) == 0 &&
            strncasecmp(name + namelen - 3, "bin", 3) == 0)
        {
            root.close();
            logmsg("Found firmware file: ", name);
            return true;
        }

        file.close();
    }

    root.close();
    return false;
}

#ifndef PLATFORM_FLASH_SECTOR_ERASE
bool program_firmware(FsFile &file)
{
    uint32_t filesize = file.size();

    if (filesize <= PLATFORM_BOOTLOADER_SIZE)
    {
        logmsg("Firmware file too small: ", (int)filesize, " bytes, minimum ",
               (int)PLATFORM_BOOTLOADER_SIZE);
        return false;
    }

    uint32_t fwsize = filesize - PLATFORM_BOOTLOADER_SIZE;
    uint32_t num_pages = (fwsize + PLATFORM_FLASH_PAGE_SIZE - 1) / PLATFORM_FLASH_PAGE_SIZE;

    // Make sure the buffer is aligned to word boundary
    static uint32_t buffer32[PLATFORM_FLASH_PAGE_SIZE / 4];
    uint8_t *buffer = (uint8_t*)buffer32;

    if (filesize > PLATFORM_FLASH_TOTAL_SIZE)
    {
        logmsg("Firmware too large: ", (int)filesize, " flash size ", (int)PLATFORM_FLASH_TOTAL_SIZE);
        return false;
    }

    if (!file.seek(PLATFORM_BOOTLOADER_SIZE))
    {
        logmsg("Seek failed");
        return false;
    }

    for (int i = 0; i < num_pages; i++)
    {
        if (i % 2)
            LED_ON();
        else
            LED_OFF();
        
        int bytes_read = file.read(buffer, PLATFORM_FLASH_PAGE_SIZE);
        if (bytes_read <= 0)
        {
            logmsg("Firmware file read failed on page ", i);
            return false;
        }

        if (bytes_read < (int)PLATFORM_FLASH_PAGE_SIZE)
        {
            // Only the final page may be short, elsewhere the tail is stale
            if (i != (int)num_pages - 1)
            {
                logmsg("Short read on page ", i, ", got bytes: ", bytes_read);
                return false;
            }
            memset(buffer + bytes_read, 0xFF, PLATFORM_FLASH_PAGE_SIZE - bytes_read);
        }

        if (!platform_rewrite_flash_page(PLATFORM_BOOTLOADER_SIZE + i * PLATFORM_FLASH_PAGE_SIZE, buffer))
        {
            logmsg("Flash programming failed on page ", i);
            return false;
        }
    }

    return true;
}
#else // PLATFORM_FLASH_SECTOR_ERASE
bool program_firmware(FsFile &file)
{
    if (!platform_firmware_erase(file))
    {
        return false;
    }
    if (!platform_firmware_program(file))
    {
        return false;
    }
    return true;
    
}

#endif // PLATFORM_FLASH_SECTOR_ERASE

// Keep a rejected image, but out of the way so the next boot does not retry it
static void move_bad_firmware_aside(const char *name)
{
    char badname[MAX_FILE_PATH + 5];
    size_t namelen = strlen(name);
    if (namelen > MAX_FILE_PATH) return;

    memcpy(badname, name, namelen);
    memcpy(badname + namelen, ".bad", sizeof(".bad"));

    SD.remove(badname); // rename() will not overwrite an existing file
    if (SD.rename(name, badname))
    {
        logmsg("Renamed rejected firmware to ", badname);
    }
    else if (SD.remove(name))
    {
        logmsg("Removed rejected firmware ", name);
    }
    logmsg("Copy the firmware .zip to the SD card again to retry the update");
}

static bool mountSDCard()
{
  // Check for the common case, FAT filesystem as first partition
  if (SD.begin(SD_CONFIG))
    return true;

  // Do we have any kind of card?
  if (!SD.card() || SD.sdErrorCode() != 0)
    return false;

  // Try to mount the whole card as FAT (without partition table)
  if (static_cast<FsVolume*>(&SD)->begin(SD.card(), true, 0))
    return true;

  // Bootloader cannot do anything without FAT
  return false;
}

extern "C"
int bootloader_main(void)
{
#ifdef FORGE_LINK_INJECT
    // ⛔ N6 STEP 2 · FIRST STATEMENT, BEFORE platform_init(). The injection builds arm an 8 s
    // hardware watchdog, and this bootloader can run for far longer than that: it mounts the card
    // (twice, if the first go fails), finds a firmware image and writes it to flash. If the
    // watchdog were still enabled and counting when we got here it would reset the board MID
    // FLASH WRITE, which is the one operation in this whole system that must not be interrupted.
    //
    // WHY IT IS HERE RATHER THAN ARGUED AWAY: whether a watchdog FIRE leaves WATCHDOG.CTRL's
    // ENABLE bit set on RP2350 is NOT settled by the SDK. The field's documented reset value is 0,
    // but the same header gives the scratch registers a reset value of 0 and then says they
    // survive a soft reset, so a reset value is not a post-fire state - and nothing in the SDK
    // reads CTRL back after a reset. Re-derived 2026-09-22, and the council agreed the source does
    // not answer it. So this is a one-instruction insurance against an unmeasured hardware
    // question, on a path where being wrong destroys the firmware image.
    //
    // It is the SDK call and not forgeLinkHwdogDisarm(): there is no policy state here (this is a
    // separate image with its own RAM, linked before the app ever runs), nothing to record, and a
    // disarm that touched scratch 2 would rewrite the very record the app boots up and reads.
    // Disabling a watchdog that was never enabled is harmless.
    // The app's own EarlyBoot does the same thing on its side; this covers the stage in between.
    watchdog_disable();
#endif

    platform_init();
    g_log_debug = true;

    logmsg("Bootloader version: " __DATE__ " " __TIME__ " " PLATFORM_NAME);

    if (mountSDCard() || mountSDCard())
    {
        FsFile fwfile;
        char name[MAX_FILE_PATH + 1];
        if (find_firmware_image(fwfile, name))
        {
            if (program_firmware(fwfile))
            {
                logmsg("Firmware update successful!");
                fwfile.close();
                if (!SD.remove(name))
                {
                    logmsg("Failed to remove firmware file");
                }
            }
            else
            {
                logmsg("Firmware update failed!");
                fwfile.close();
                move_bad_firmware_aside(name);
                platform_emergency_log_save();
            }
            
        }
    }
    else
    {
        logmsg("Bootloader SD card init failed");
    }

    logmsg("Bootloader continuing to main firmware");
    platform_boot_to_main_firmware();

    return 0;
}

#endif
