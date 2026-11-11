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

// BLOCK INJECTION - the board half. Spec in our own words: docs/block-interface.md.
//
// One sentence: a run of 512-byte sectors the Mac names by (image id, LBA, count) is written
// through the served image's OWN backing store, from the main loop, at bus-free, and the read
// prefetch is invalidated before the reply goes back.
//
// ⛔ THE WALL. SECTORS AND GENERATIONS ONLY. Nothing here knows what a volume, a directory
// entry, a file or a sample is. The Mac owns all of that.
//
// ⛔ THE FOUR SAFETY PROPERTIES THIS FILE IS RESPONSIBLE FOR, and where each one lives:
//   R5 · never past the end of the image, never truncate  -> fl_range_ok(), and the total
//        absence of any call to img.file.truncate() in this file. A sector write inside an
//        existing file's extent cannot touch the card's FAT, so a torn write costs image
//        content and never the filesystem.
//   R3 · never serve a stale sector from the board's own prefetch -> scsiDiskPrefetchInvalidate
//        after EVERY applied run, before the reply.
//   R4 · never a torn publish -> FL_WM_ATOMIC applies the whole run without yielding, and only
//        after a quiet window.
//   R1 · never delay the sampler's selection past its timeout -> FL_WM_PREEMPTIBLE checks for a
//        pending selection between single sectors and resumes on the next pass.
//
// ⚠ WHAT IS NOT MEASURED, AND THE NUMBERS THAT REST ON IT: the atomic cap (32 sectors) and the
// quiet window (50 ms) are the frame's values, not measurements. M1 (the S1000's tolerance of a
// delayed selection) and M3 (SD write latency on his card) both want the board on the bench.
// Until then, treat every timing constant here as a placeholder wearing a number's clothes.

#include "ForgeLink_inject.h"

#ifdef FORGE_LINK_INJECT

#include <Arduino.h>
#include <string.h>
#include <SdFat.h>

#include "ForgeLink_probe.h"
#include "ForgeLink_frames.h"
#include "ForgeLink_digest.h"
#include "ForgeLink_hwdog.h"
#include "ForgeLink_bulk.h"
#include "ZuluSCSI_config.h"
#include "ZuluSCSI_log.h"
#include "ZuluSCSI_disk.h"
#include <ZuluSCSI_platform.h>
#include <scsiPhy.h>
#include <pico/rand.h>          // FW-BOOTID: get_rand_64, the RP2350's TRNG-seeded generator

extern "C" {
#include <scsi.h>
}

#if __has_include("ZuluSCSI_build_info_generated.h")
#include "ZuluSCSI_build_info_generated.h"
#endif
#ifndef ZULU_BUILD_GIT_REV
#define ZULU_BUILD_GIT_REV "unknown"
#endif
#ifndef ZULU_BUILD_DIRTY_COUNT
#define ZULU_BUILD_DIRTY_COUNT -1          // -1 = the build could not say (no generated header)
#define ZULU_BUILD_DIRTY_FILES ""
#endif

extern SdFs SD;
extern bool g_rebooting;

#define FL_SECTOR_BYTES 512

// ⛔ THE UNION'S DISK ROLE IS WIRE-IDENTICAL TO ForgeLink_Blaster_inject. The union keeps the probe's
// 32,854 B reply cache (the bridge's CDC EXEC needs it), so every cap this file reads from the cache
// is pinned back to inject's own 8,342 B cache's capacity: 8,342 - (2 sync + 6 hdr + 10 fixed) - 4
// crc = 8,320 B. ECHO above it is refused by name exactly as on inject. Other builds: unchanged.
#ifdef FL_UNIFIED
#  define FL_INJECT_REPLY_CAP() ((uint32_t)(8342u - (2 + FL_HDR_LEN + FL_REPLY_FIXED_LEN) - 4))
static_assert(8342u - (2 + FL_HDR_LEN + FL_REPLY_FIXED_LEN) - 4 == 8320, "inject's reply capacity is 8,320 B");
#else
#  define FL_INJECT_REPLY_CAP() flFramesReplyDataCapacity()
#endif

/*****************************************************************************
 * Per-image generations, and the sampler-activity ring (frame B12 / B13)
 *****************************************************************************/

// A GENERATION IS A COUNT OF WRITES, NOT A CHECKSUM AND NOT A VERSION.
// It exists for exactly one question: "has the sampler written this image since the Mac last
// read it?" That is all the Mac needs to decide whether its allocation is still valid, and it
// is all the board is allowed to know. The read generation is there for symmetry and for the
// app's own diagnostics; nothing conditions on it.
static uint32_t g_read_gen[S2S_MAX_TARGETS];
static uint32_t g_write_gen[S2S_MAX_TARGETS];

#define FL_ACT_RING 64
struct fl_act_t {
    uint32_t index;
    uint32_t ms;
    uint32_t lba;
    uint16_t count;
    uint8_t  id;
    uint8_t  op;
};
static fl_act_t g_act[FL_ACT_RING];
static uint32_t g_act_next_index = 0;      // index the NEXT entry will carry
static uint32_t g_last_sampler_ms = 0;
static bool     g_saw_sampler = false;

static bool g_card_mode = false;

// ⭐ FW-BOOTID (2026-10-14): A PER-BOOT IDENTITY, appended to every WATCH reply.
// The save counter g_write_gen is RAM only, so a reboot puts it back to 0 and a push resumed after a
// hub drop could miss a save made before the reboot (block-interface.md, pipelined rule 7(f)). The
// board's clock proves "no reboot" only once the sampler has touched the bus; with an empty ring the
// clock is silent and the resume had to stop. This number is drawn ONCE per boot from the hardware
// random generator (get_rand_64: TRNG-seeded on the RP2350, never a fixed seed), 64 bits, is never 0
// (0 means "this firmware has no boot id"; a 0 draw is thrown away and drawn again, so no value is
// favoured), and lives in RAM, so a reboot always draws a new one (a repeat is a 2^-64 chance). Drawn
// on the first main-loop pass and read only from the main-loop dispatch: no SCSI or USB interrupt
// handler calls it.
static uint64_t g_boot_id = 0;

static uint64_t fl_boot_id(void)
{
    while (g_boot_id == 0) g_boot_id = get_rand_64();
    return g_boot_id;
}

// The largest WATCH reply: the 10-byte head, a full ring of 16-byte entries, the 8-byte boot id.
#define FL_WATCH_MAX_REPLY (10u + 16u * FL_ACT_RING + 8u)
// Checked against the cap the WATCH handler really enforces: the union pins it to inject's 8,320 B even
// though its frame span is the probe's 32 KiB (the sabotage run 2026-10-14 showed a span-only check
// passing a 1,024-entry ring there); other builds use their own span, of which the cap is a part.
#ifdef FL_UNIFIED
static_assert(FL_WATCH_MAX_REPLY <= 8320u, "a full WATCH reply with its boot id must fit inject's reply cap");
#else
static_assert(FL_WATCH_MAX_REPLY <= FL_REPLY_DATA_SPAN, "a full WATCH reply with its boot id must fit the reply span");
#endif

// ⛔ STALL IS TEST-ONLY AND MUST NEVER SHIP IN A BUILD THE OWNER WOULD KEEP.
// A diagnostic that injects a fault defaults OFF and says so in the banner (the 2026-09-19
// rule, paid for by an evening of frame loss that was our own injection). This one goes
// further: it is not merely defaulted off, it is COMPILED OUT unless FORGE_LINK_INJECT_TEST
// is defined, so no runtime flag can reach it in a normal build.
#ifdef FORGE_LINK_INJECT_TEST
static uint16_t g_stall_ms = 0;
#endif

extern "C" void forgeLinkInjectNoteSampler(uint8_t scsi_id, uint8_t op, uint32_t lba, uint32_t count)
{
    uint32_t now = millis();
    g_last_sampler_ms = now;
    g_saw_sampler = true;

    if (op == FL_ACT_WRITE && scsi_id < S2S_MAX_TARGETS) g_write_gen[scsi_id]++;
    if (op == FL_ACT_READ  && scsi_id < S2S_MAX_TARGETS) g_read_gen[scsi_id]++;

    fl_act_t &e = g_act[g_act_next_index % FL_ACT_RING];
    e.index = g_act_next_index;
    e.ms    = now;
    e.lba   = lba;
    e.count = (uint16_t)(count > 0xFFFF ? 0xFFFF : count);
    e.id    = scsi_id;
    e.op    = op;
    g_act_next_index++;
}

extern "C" void forgeLinkInjectCardMode(bool entering)
{
    g_card_mode = entering;
    forgeLinkHwdogCardMode(entering);   // N6 step 2: never armed in card-reader mode
    forgeLinkBulkCardMode(entering);    // N6 step 3: the bulk driver stands down too
    if (entering)
        logmsg("Forge Link inject: card-reader mode - frame parser STANDS DOWN, console keys live");
    else
        logmsg("Forge Link inject: left card-reader mode - frame parser armed again");
}

/*****************************************************************************
 * Image lookup, range checking and the bus-quiet test
 *****************************************************************************/

// Find the served image whose SCSI id matches. Returns nullptr when nothing is served there.
// ⚠ The id is the SCSI id, which is the digit in the HD<n>.img filename - NOT an index into
// the target array, and not a drive letter. Those three have coincided so far on his card and
// nothing guarantees they keep coinciding.
static image_config_t *fl_image(uint8_t scsi_id)
{
    for (int i = 0; i < S2S_MAX_TARGETS; i++)
    {
        image_config_t &img = scsiDiskGetImageConfig(i);
        if (!(img.scsiId & S2S_CFG_TARGET_ENABLED)) continue;
        if ((img.scsiId & S2S_CFG_TARGET_ID_BITS) != (scsi_id & S2S_CFG_TARGET_ID_BITS)) continue;
        if (!img.file.isOpen()) continue;
        // ⛔ 512-BYTE SECTORS ONLY, AND IT IS REFUSED RATHER THAN CONVERTED. Every LBA on this
        // wire is multiplied by 512 to reach a byte offset. On an image whose own sector size
        // is something else, that arithmetic silently addresses the WRONG PART OF THE FILE -
        // the most dangerous kind of wrong, because every range check still passes. His Akai
        // images are 512 and the protocol says so on its face; anything else is not served.
        if (img.bytesPerSector != 0 && img.bytesPerSector != FL_SECTOR_BYTES) continue;
        return &img;
    }
    return nullptr;
}

// ⛔ R5 · THE RANGE CHECK IS THE FAT GUARD. A write that stays inside the existing file's
// extent writes raw sectors or writes inside the extent; neither extends the file, so FAT
// metadata is never touched. A write that runs PAST the end would have to extend it, which is
// the one thing that could corrupt the card rather than one image. Refuse instead.
// Computed in 64-bit throughout: lba + count overflows 32 bits on a large image and a wrapped
// comparison passes a range check it should have failed.
static bool fl_range_ok(image_config_t &img, uint32_t lba, uint32_t count)
{
    uint64_t sectors = img.file.size() / FL_SECTOR_BYTES;
    uint64_t end = (uint64_t)lba + (uint64_t)count;
    return count > 0 && end <= sectors;
}

// A selection is pending, or a transaction is in progress: the sampler wants the bus and the
// board must return to the main loop so scsiPoll() can answer it. The two flags are the same
// pair scsi.c itself tests (lib/SCSI2SD/src/firmware/scsi.c, the BUS_FREE arm).
static inline bool fl_sampler_wants_bus(void)
{
    if (scsiDev.phase != BUS_FREE) return true;
    if (scsiDev.selFlag) return true;
    if (*SCSI_STS_SELECTED) return true;
    if (SCSI_IN(SEL) || SCSI_IN(BSY)) return true;
    return false;
}

static inline bool fl_quiet_window(void)
{
    if (fl_sampler_wants_bus()) return false;
    if (!g_saw_sampler) return true;                 // nothing has ever asked: as quiet as it gets
    return (uint32_t)(millis() - g_last_sampler_ms) >= FL_INJECT_QUIET_MS;
}

/*****************************************************************************
 * The deferred write run
 *****************************************************************************
 * A preemptible run cannot be answered inside one frame dispatch, because yielding to a
 * selection is the whole point of it. So the run becomes main-loop state and the reply is
 * sent when it finishes. While a run is active no further frame is dispatched, which is
 * also what stops a host retry re-applying half of it.
 */
// The WRITE_SECTORS reply: 11 bytes until stage I1, 15 with I1's trailing apply_us u32, and 43
// since stage P, which appends SEVEN more u32s - the split of where a run's time actually went
// (docs/block-interface.md, "the timing split"). Every one is ADDITIVE: the first 15 bytes are
// exactly what they were, and a host that reads 11 or 15 is still correct.
#define FL_WRITE_REPLY_V1_BYTES (1 + 2 + 4 + 4 + 4)
#define FL_WRITE_REPLY_BYTES    (FL_WRITE_REPLY_V1_BYTES + 7 * 4)

// Receive whatever the host has already sent while the board is busy with something else. This
// is the board side of pipelining: the parser fills a FREE slot only, never the head a run is
// using, so it is safe from inside a run or a hash slice. Returns the micros spent, so the reply
// can say how much of the run's wall time was spent taking in the NEXT frame.
static uint32_t fl_drain_timed(void)
{
    if (flFramesQueued() >= FL_RX_SLOTS) return 0;
    uint32_t t = micros();
    flFramesRxDrain();
    return (uint32_t)(micros() - t);
}

static struct {
    bool     active;
    uint8_t  id;
    uint32_t lba;            // next sector to write
    uint32_t remaining;      // sectors still to apply
    uint32_t lba0;           // the run as asked, for the read-back and the prefetch invalidate
    uint32_t count0;
    uint32_t applied;
    uint8_t  mode;
    // M3 · THE BOARD'S OWN SD TIME, in microseconds, reported as the reply's trailing apply_us.
    // PREEMPTIBLE: the WORST single sector (seek + write) - the longest the sampler can be kept
    // waiting by one step of this run, which is R1's number. ATOMIC: the whole run, first seek
    // to last write, because an atomic run never yields and the sampler waits for all of it,
    // which is R4's. The flush and the read-back CRC in fl_run_finish are NOT inside it: this
    // is the time spent in the write calls, and nothing else is claimed.
    uint32_t apply_us;
    // ⭐ STAGE P · THE SPLIT, so a bench run can say where ~520 ms per 32 KiB actually goes
    // instead of inferring it from the Mac's round trip. All micros, all board-measured:
    uint32_t sd_write_us;    // SUM of every sector's seek + write (apply_us is the worst one)
    uint32_t flush_us;       // file flush + the prefetch invalidate, before the reply
    uint32_t readback_us;    // the post-write re-read of the run off the card + its CRC-32
    uint32_t yield_us;       // parked between steps while the sampler had the bus (preemptible)
    uint32_t queue_us;       // this frame sat complete in the RX queue behind earlier work
    uint32_t rx_us;          // this frame's USB transfer as the board saw it, first byte -> CRC ok
    uint32_t drain_us;       // time inside this run spent taking in the NEXT frame (the overlap)
    uint32_t last_step_end;  // micros() when the previous step returned, for yield_us
    bool     stepped;
    // the request coordinates, so the reply lands on the right frame
    uint8_t  seq;
    uint32_t req_crc;
    uint32_t req_len;
    const uint8_t *data;     // into the RX body buffer, which nothing else touches while active
} g_run;

// Read the run back through the same backing store and CRC it, so the Mac can prove what
// landed without a second round trip. ⚠ THIS IS A READ-BACK THROUGH THE BOARD'S OWN WRITE
// PATH, not through the SCSI side. It proves the bytes reached the card; it does NOT prove
// the sampler will see them - that is what invalidating the prefetch is for, and what the
// acceptance test's WATCH witness actually measures.
// `drained_us`, when not null, means "keep taking in the next frame between sectors" and
// receives the time that took, so the caller can keep it out of the read-back's own number.
static uint32_t fl_readback_crc(image_config_t &img, uint32_t lba, uint32_t count,
                                uint32_t *drained_us)
{
    static uint8_t buf[FL_SECTOR_BYTES];
    uint32_t crc = 0;
    if (!img.file.seek((uint64_t)lba * FL_SECTOR_BYTES)) return 0;
    for (uint32_t i = 0; i < count; i++)
    {
        if (img.file.read(buf, FL_SECTOR_BYTES) != (ssize_t)FL_SECTOR_BYTES) return 0;
        crc = flCrc32(crc, buf, FL_SECTOR_BYTES);
        platform_reset_watchdog();
        // The drain moves the file position of nothing: it touches the USB FIFO and the RX
        // slots only, so the next read continues where this one stopped.
        if (drained_us) *drained_us += fl_drain_timed();
    }
    return crc;
}

// The v2 reply's seven trailing u32s, after apply_us. Shared by the applied and refused shapes
// so the two can never disagree about where a field lives.
static void fl_put_split(uint8_t *rep)
{
    flPut32(&rep[15], g_run.sd_write_us);
    flPut32(&rep[19], g_run.flush_us);
    flPut32(&rep[23], g_run.readback_us);
    flPut32(&rep[27], g_run.yield_us);
    flPut32(&rep[31], g_run.queue_us);
    flPut32(&rep[35], g_run.rx_us);
    flPut32(&rep[39], g_run.drain_us);
}

static void fl_run_finish(uint8_t outcome)
{
    image_config_t *img = fl_image(g_run.id);
    uint32_t crc = 0;
    if (outcome == FL_W_OK && img)
    {
        uint32_t t = micros();
        img->file.flush();
        // ⛔ R3 · MANDATORY, AND BEFORE THE REPLY. Without this the S1000 can read the OLD
        // contents of the very sectors just changed, out of the board's own 8 KiB read-ahead,
        // and the Mac would have been told the write succeeded. It did; the sampler just
        // could not see it.
        scsiDiskPrefetchInvalidate(g_run.id);
        g_run.flush_us = (uint32_t)(micros() - t);
        // ⚠ THE READ-BACK READS THROUGH THE SAME img.file PATH AS READ_SECTORS AND HASH_SECTORS.
        // So the Mac's per-run check of this CRC is already a read-back of every run, one frame
        // earlier than any read of its own. What it is not: a check made AFTER the whole push
        // (a later event cannot show in it) and it is CRC-32, not the bytes. Stage P's verify is
        // one SHA-256 over each landed span at the END, which covers both of those.
        // A publish (ATOMIC) takes in nothing new until it has replied, same as its write loop.
        uint32_t drained = 0;
        t = micros();
        crc = fl_readback_crc(*img, g_run.lba0, g_run.count0,
                              g_run.mode == FL_WM_ATOMIC ? nullptr : &drained);
        g_run.readback_us = (uint32_t)(micros() - t) - drained;
        g_run.drain_us += drained;
    }

    // {outcome u8, sectors_applied u16, write_gen_after u32, readback_crc32 u32, apply_us u32,
    //  sd_write_us, flush_us, readback_us, yield_us, queue_us, rx_us, drain_us (u32 each)}
    // Everything after byte 11 is ADDITIVE (I1, then stage P): a host that reads 11 or 15 bytes
    // is still correct.
    uint8_t rep[FL_WRITE_REPLY_BYTES];
    rep[0] = outcome;
    rep[1] = (uint8_t)(g_run.applied);
    rep[2] = (uint8_t)(g_run.applied >> 8);
    flPut32(&rep[3], g_run.id < S2S_MAX_TARGETS ? g_write_gen[g_run.id] : 0);
    flPut32(&rep[7], crc);
    flPut32(&rep[11], g_run.apply_us);
    fl_put_split(rep);

    g_run.active = false;
    flFramesSendReply(FL_T_WRITE_SECTORS, g_run.seq, g_run.req_crc, g_run.req_len,
                      rep, sizeof(rep), nullptr, 0);
}

static void fl_run_step(void)
{
    // yield_us: the gap since the previous step returned is time the run sat parked while the
    // sampler had the bus (plus whatever else the main loop did in between).
    const uint32_t step_start = micros();
    if (g_run.stepped) g_run.yield_us += (uint32_t)(step_start - g_run.last_step_end);
    g_run.stepped = true;

    image_config_t *img = fl_image(g_run.id);
    if (!img) { fl_run_finish(FL_W_NO_SUCH_IMAGE); return; }

    if (g_run.mode == FL_WM_ATOMIC)
    {
        // ⛔ R4 · THE WHOLE RUN OR NOTHING, AND ONLY BETWEEN HIS PRESSES. A publish that is
        // torn across a sampler read lists a volume that will not load. The quiet window puts
        // it between bursts: a page-entry burst is ~48 ms end to end and his presses are
        // seconds apart, so 50 ms of silence means we are between them, not inside one.
        if (!fl_quiet_window()) { fl_run_finish(FL_W_BUSY_NO_QUIET); return; }
    }

    // An atomic run is timed as ONE span; it cannot be split across passes, because it never
    // yields - so t_run is taken once here and closed on every exit from the loop.
    const bool atomic = (g_run.mode == FL_WM_ATOMIC);
    const uint32_t t_run = micros();

    while (g_run.remaining > 0)
    {
        const uint32_t t_sector = micros();
        if (!img->file.seek((uint64_t)g_run.lba * FL_SECTOR_BYTES))
        {
            if (atomic) g_run.apply_us = (uint32_t)(micros() - t_run);
            fl_run_finish(FL_W_SD_ERROR);
            return;
        }
        ssize_t w = img->file.write(g_run.data + (size_t)g_run.applied * FL_SECTOR_BYTES,
                                    FL_SECTOR_BYTES);
        const uint32_t now = micros();
        g_run.sd_write_us += (uint32_t)(now - t_sector);
        if (atomic)
            g_run.apply_us = (uint32_t)(now - t_run);
        else if ((uint32_t)(now - t_sector) > g_run.apply_us)
            g_run.apply_us = (uint32_t)(now - t_sector);
        if (w != (ssize_t)FL_SECTOR_BYTES)
        {
            fl_run_finish(FL_W_SD_ERROR);
            return;
        }
        g_run.lba++;
        g_run.applied++;
        g_run.remaining--;
        platform_reset_watchdog();

        // ⭐ STAGE P · THE OVERLAP. Between sectors of a PREEMPTIBLE run, take in whatever of the
        // next frame the host has already sent, into the free RX slot. Without this the USB
        // endpoint NAKs for the whole run and the next frame's transfer only starts once this
        // one has replied - pipelining would overlap nothing but the Mac's turnaround.
        // ⛔ NOT INSIDE AN ATOMIC RUN. A publish is the one write the sampler must never see
        // half of, and it runs without a single yield; it gets no new work in the middle either.
        if (!atomic) g_run.drain_us += fl_drain_timed();

        // ⛔ R1 · THE YIELD. Between single sectors only, and only in preemptible mode. The
        // worst latency this can add to the sampler's next command is ONE SD sector write
        // (plus, since stage P, one bounded drain: at most the CDC FIFO's 4 KiB of parsing and
        // one table-driven frame CRC).
        if (g_run.mode == FL_WM_PREEMPTIBLE && g_run.remaining > 0 && fl_sampler_wants_bus())
        {
            g_run.last_step_end = micros();
            return;                                   // resume on the next main-loop pass
        }
    }

    fl_run_finish(FL_W_OK);
}

/*****************************************************************************
 * The verbs
 *****************************************************************************/

// ⭐ C13 · THE CARD'S OWN NAME (midi, 2026-10-12: the "ZuluSCSI Blaster · AKAI 2" heading). The name
// a Mac shows for the card is the root directory's volume-label entry, so that is what is read: on
// FAT the 11-byte entry with attribute 0x08, on exFAT the 0x83 entry (UTF-16, up to 11 units). The
// boot sector's copy is NOT read: formatters leave it "NO NAME" and a rename on the Mac changes only
// the root entry. Read per request (one cached directory sector), so a card renamed in CARD_MODE is
// right on the next listing. No label, an unmounted card or a scan past 64 entries = length 0.
static uint8_t fl_card_label(uint8_t *dst)
{
    FsFile root = SD.open("/", O_RDONLY);
    if (!root.isOpen()) return 0;
    const bool exfat = SD.fatType() == FAT_TYPE_EXFAT;
    uint8_t e[32];
    uint8_t n = 0;
    for (int k = 0; k < 64 && root.read(e, sizeof(e)) == (int)sizeof(e); k++)
    {
        if (e[0] == 0x00) break;                               // end of directory, both formats
        if (exfat)
        {
            if (e[0] != 0x83) continue;
            uint8_t units = e[1] > 11 ? 11 : e[1];
            for (uint8_t u = 0; u < units; u++)
            {
                uint16_t c = (uint16_t)(e[2 + 2 * u] | (e[3 + 2 * u] << 8));
                if (c >= 0xD800 && c <= 0xDFFF) c = '?';       // a lone surrogate half: not text
                if (c < 0x80) dst[n++] = (uint8_t)c;
                else if (c < 0x800) { dst[n++] = 0xC0 | (c >> 6); dst[n++] = 0x80 | (c & 0x3F); }
                else { dst[n++] = 0xE0 | (c >> 12); dst[n++] = 0x80 | ((c >> 6) & 0x3F); dst[n++] = 0x80 | (c & 0x3F); }
            }
            break;
        }
        if (e[0] == 0xE5) continue;                             // deleted
        if ((e[11] & 0x3F) == 0x0F) continue;                   // a long-name piece
        if (!(e[11] & 0x08)) continue;
        for (int i = 0; i < 11; i++) dst[n++] = e[i] < 0x80 ? e[i] : '?';  // OEM code page: ASCII only
        while (n && dst[n - 1] == ' ') n--;
        break;
    }
    root.close();
    return n;
}

static void fl_do_disk_info(uint8_t seq, uint32_t req_crc, uint32_t req_len)
{
    uint8_t *out = flFramesReplyDataBuffer();
    uint32_t cap = FL_INJECT_REPLY_CAP();
    uint32_t n = 1;
    uint8_t count = 0;

    for (int i = 0; i < S2S_MAX_TARGETS; i++)
    {
        image_config_t &img = scsiDiskGetImageConfig(i);
        if (!(img.scsiId & S2S_CFG_TARGET_ENABLED)) continue;
        if (!img.file.isOpen()) continue;

        char name[MAX_FILE_PATH];
        size_t namelen = img.file.getFilename(name, sizeof(name));
        if (namelen > 255) namelen = 255;

        if (n + 19 + namelen + 1 + FL_LABEL_MAX > cap) break;   // the label trailer always fits

        uint8_t id = img.scsiId & S2S_CFG_TARGET_ID_BITS;
        uint16_t bps = img.bytesPerSector ? img.bytesPerSector : FL_SECTOR_BYTES;
        uint64_t sectors = img.file.size() / (bps ? bps : FL_SECTOR_BYTES);

        out[n++] = id;
        out[n++] = 0;                                   // LUN: always 0 on this board
        out[n++] = (uint8_t)(bps); out[n++] = (uint8_t)(bps >> 8);
        flPut32(&out[n], (uint32_t)(sectors > 0xFFFFFFFFull ? 0xFFFFFFFFull : sectors)); n += 4;
        out[n++] = img.file.isWritable() ? 1 : 0;
        // Byte 9 - layout (FL_FEAT_LAYOUT): the open-time contiguity bit, 0 bytes of new state.
        // ⛔ isContiguous() only; contiguousRange() walks the FAT on a fragmented file.
        out[n++] = (img.file.isRom() || img.file.isRaw() || img.file.isFolder())
                       ? FL_LAYOUT_NOT_TESTED
                       : (img.file.isContiguous() ? FL_LAYOUT_ONE_PIECE : FL_LAYOUT_FRAGMENTED);
        flPut32(&out[n], g_read_gen[id]);  n += 4;
        flPut32(&out[n], g_write_gen[id]); n += 4;
        out[n++] = (uint8_t)namelen;
        memcpy(&out[n], name, namelen); n += namelen;
        count++;
    }
    out[0] = count;
    // FL_FEAT_LABEL: label_len u8 + label, after the last row. Old parsers stop at `count` rows.
    uint8_t ll = fl_card_label(&out[n + 1]);
    out[n++] = ll; n += ll;
    flFramesSendReply(FL_T_DISK_INFO, seq, req_crc, req_len, nullptr, 0, out, n);
}

static void fl_do_read(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                       const uint8_t *p, uint32_t len)
{
    uint8_t rep[1 + 4 + 4 + 2];
    memset(rep, 0, sizeof(rep));

    if (len < 7) { rep[0] = FL_W_BAD_REQUEST;
                   flFramesSendReply(FL_T_READ_SECTORS, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
                   return; }

    uint8_t  id    = p[0];
    uint32_t lba   = flGet32(&p[1]);
    uint16_t count = (uint16_t)(p[5] | (p[6] << 8));

    image_config_t *img = fl_image(id);
    uint8_t outcome = FL_W_OK;
    if (!img)                                   outcome = FL_W_NO_SUCH_IMAGE;
    else if (count == 0 || count > FL_INJECT_READ_MAX_SECTORS) outcome = FL_W_BAD_REQUEST;
    else if (!fl_range_ok(*img, lba, count))    outcome = FL_W_OUT_OF_RANGE;

    if (outcome != FL_W_OK)
    {
        rep[0] = outcome;
        flFramesSendReply(FL_T_READ_SECTORS, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
        return;
    }

    uint8_t *data = flFramesReplyPayloadAt(sizeof(rep));
    if (!img->file.seek((uint64_t)lba * FL_SECTOR_BYTES) ||
        img->file.read(data, (size_t)count * FL_SECTOR_BYTES) != (ssize_t)count * FL_SECTOR_BYTES)
    {
        rep[0] = FL_W_SD_ERROR;
        flFramesSendReply(FL_T_READ_SECTORS, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
        return;
    }

    rep[0] = FL_W_OK;
    flPut32(&rep[1], g_read_gen[id & S2S_CFG_TARGET_ID_BITS]);
    flPut32(&rep[5], g_write_gen[id & S2S_CFG_TARGET_ID_BITS]);
    rep[9]  = (uint8_t)count;
    rep[10] = (uint8_t)(count >> 8);
    flFramesSendReply(FL_T_READ_SECTORS, seq, req_crc, req_len,
                      rep, sizeof(rep), data, (uint32_t)count * FL_SECTOR_BYTES);
}

static void fl_write_refuse(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                            uint8_t outcome, uint32_t gen)
{
    // Same 43-byte shape as an applied run, apply_us 0: a refusal spent no time in a write call.
    // queue_us and rx_us are still real (g_run carries them from fl_do_write's entry), because a
    // refused frame in a pipeline was still transferred and still waited.
    uint8_t rep[FL_WRITE_REPLY_BYTES];
    memset(rep, 0, sizeof(rep));
    rep[0] = outcome;
    flPut32(&rep[3], gen);
    flPut32(&rep[31], g_run.queue_us);
    flPut32(&rep[35], g_run.rx_us);
    flFramesSendReply(FL_T_WRITE_SECTORS, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
}

static void fl_do_write(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                        const uint8_t *p, uint32_t len)
{
    // {id u8, lba u32, count u16, mode u8, flags u8, expected_write_gen u32, data[]}
    const uint32_t FIXED = 1 + 4 + 2 + 1 + 1 + 4;
    if (len < FIXED) { fl_write_refuse(seq, req_crc, req_len, FL_W_BAD_REQUEST, 0); return; }

    uint8_t  id       = p[0];
    uint32_t lba      = flGet32(&p[1]);
    uint16_t count    = (uint16_t)(p[5] | (p[6] << 8));
    uint8_t  mode     = p[7];
    uint8_t  flags    = p[8];
    uint32_t expected = flGet32(&p[9]);
    const uint8_t *data = p + FIXED;

    // The queue and transfer times of THIS frame, taken before anything can refuse it. g_run is
    // not active here (nothing is dispatched while a run is), so its timing fields are free.
    {
        const uint32_t now = micros();
        g_run.queue_us = (uint32_t)(now - flFramesHeadDoneUs());
        g_run.rx_us    = (uint32_t)(flFramesHeadDoneUs() - flFramesHeadStartUs());
    }

    image_config_t *img = fl_image(id);
    uint32_t gen = (id & S2S_CFG_TARGET_ID_BITS) < S2S_MAX_TARGETS
                   ? g_write_gen[id & S2S_CFG_TARGET_ID_BITS] : 0;

    if (!img)                        { fl_write_refuse(seq, req_crc, req_len, FL_W_NO_SUCH_IMAGE, 0); return; }
    if (count == 0 || count > FL_INJECT_MAX_SECTORS ||
        len != FIXED + (uint32_t)count * FL_SECTOR_BYTES ||
        (mode != FL_WM_PREEMPTIBLE && mode != FL_WM_ATOMIC))
                                     { fl_write_refuse(seq, req_crc, req_len, FL_W_BAD_REQUEST, gen); return; }
    if (mode == FL_WM_ATOMIC && count > FL_INJECT_ATOMIC_MAX_SECTORS)
                                     { fl_write_refuse(seq, req_crc, req_len, FL_W_BAD_REQUEST, gen); return; }
    if (!img->file.isWritable())     { fl_write_refuse(seq, req_crc, req_len, FL_W_READ_ONLY, gen); return; }
    if (!fl_range_ok(*img, lba, count)) { fl_write_refuse(seq, req_crc, req_len, FL_W_OUT_OF_RANGE, gen); return; }

    // ⛔ B12 · THE COMPARE-AND-SWAP. If the sampler wrote this image since the Mac read the
    // ground its allocation stands on, that allocation may now overlap his save. Refuse and
    // let the Mac re-read. The board is not reasoning about the filesystem here; it is
    // counting writes, which is the most it is allowed to know.
    if ((flags & FL_WF_CONDITIONAL) && expected != gen)
    {
        fl_write_refuse(seq, req_crc, req_len, FL_W_GEN_MISMATCH, gen);
        return;
    }

    g_run.active    = true;
    g_run.id        = id;
    g_run.lba       = lba;
    g_run.lba0      = lba;
    g_run.count0    = count;
    g_run.remaining = count;
    g_run.applied   = 0;
    g_run.apply_us  = 0;
    g_run.sd_write_us = 0;
    g_run.flush_us    = 0;
    g_run.readback_us = 0;
    g_run.yield_us    = 0;
    g_run.drain_us    = 0;
    g_run.stepped     = false;
    g_run.mode      = mode;
    g_run.seq       = seq;
    g_run.req_crc   = req_crc;
    g_run.req_len   = req_len;
    g_run.data      = data;

    fl_run_step();
}

/*****************************************************************************
 * HASH_SECTORS - the fingerprint verify (stage P)
 *****************************************************************************
 * One SHA-256 over a run of sectors, read through img.file exactly as READ_SECTORS reads them
 * (and as the WRITE reply's read-back does), so a match means what a byte-for-byte read-back
 * through the board means - without moving the bytes over USB. 1.7 MB of read-back was 14.8 s
 * of W47's 48.4 s; as a fingerprint it is one request and 32 bytes back.
 *
 * ⛔ IT IS DEFERRED AND IT YIELDS, LIKE A PREEMPTIBLE WRITE. A run of thousands of sectors is far
 * longer than anything the main loop may hold, so it reads FL_INJECT_HASH_SLICE_SECTORS (the
 * READ cap, 8 KiB) at a time and returns to the main loop whenever the sampler wants the bus.
 * The worst it adds to a selection is one 8 KiB SD read plus hashing it.
 *
 * The generation is taken at the start and the end and both go back. The Mac treats a moved
 * generation as "the sampler wrote during the verify" - the same refusal a multi-chunk pre-image
 * read makes - because a digest over a run the sampler changed half-way is a digest of neither.
 *
 * The scratch buffer is the reply cache's data area. The cache is INVALIDATED at the start, so
 * no replay can ever send what the slices wrote over it; nothing else is dispatched while the
 * job runs, and the job's own reply is built from a separate small array.
 */
#define FL_HASH_REPLY_BYTES (1 + 1 + 4 + 4 + 4 + 4 + 4 + 4 + FL_SHA256_BYTES)

static struct {
    bool     active;
    uint8_t  id;
    uint8_t  algo;
    uint32_t lba;          // next sector to read
    uint32_t remaining;
    uint32_t count0;
    uint32_t gen0;
    uint32_t t0;           // micros() at dispatch
    uint32_t sd_us;        // time inside the SD reads
    uint32_t queue_us;
    fl_sha256_t sha;
    uint8_t  seq;
    uint32_t req_crc;
    uint32_t req_len;
} g_hash;

// N6 step 3. "Busy" is a WRITE run or a hash job in flight - the two things whose work OUTLIVES
// its dispatch and points straight into an RX slot, so ownership must not change under them
// (frame C6). Declared here rather than at the top because both structs are defined above.
extern "C" bool FL_UNIFIED_RAM_FUNC(forgeLinkInjectBusy)(void)   // IRQ-reachable: bulk STATUS, tud_task
{
    return g_run.active || g_hash.active;
}

extern "C" uint32_t FL_UNIFIED_RAM_FUNC(forgeLinkInjectWriteGen)(void)   // IRQ-reachable: bulk STATUS
{
    return g_run.id < S2S_MAX_TARGETS ? g_write_gen[g_run.id] : 0;
}


static void fl_hash_reply(uint8_t seq, uint32_t req_crc, uint32_t req_len, uint8_t outcome,
                          uint8_t algo, uint32_t count, uint32_t gen0, uint32_t gen1,
                          uint32_t hash_us, uint32_t sd_us, uint32_t queue_us,
                          const uint8_t *digest)
{
    // {outcome u8, algo u8, count u32, write_gen_start u32, write_gen_end u32,
    //  hash_us u32, sd_us u32, queue_us u32, digest[32]}
    uint8_t rep[FL_HASH_REPLY_BYTES];
    memset(rep, 0, sizeof(rep));
    rep[0] = outcome;
    rep[1] = algo;
    flPut32(&rep[2], count);
    flPut32(&rep[6], gen0);
    flPut32(&rep[10], gen1);
    flPut32(&rep[14], hash_us);
    flPut32(&rep[18], sd_us);
    flPut32(&rep[22], queue_us);
    if (digest) memcpy(&rep[26], digest, FL_SHA256_BYTES);
    flFramesSendReply(FL_T_HASH_SECTORS, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
}

static void fl_hash_finish(uint8_t outcome)
{
    uint8_t digest[FL_SHA256_BYTES];
    const uint8_t *d = nullptr;
    if (outcome == FL_W_OK) { flSha256Final(&g_hash.sha, digest); d = digest; }
    uint32_t gen1 = g_hash.id < S2S_MAX_TARGETS ? g_write_gen[g_hash.id] : 0;
    g_hash.active = false;
    fl_hash_reply(g_hash.seq, g_hash.req_crc, g_hash.req_len, outcome, g_hash.algo,
                  g_hash.count0 - g_hash.remaining, g_hash.gen0, gen1,
                  (uint32_t)(micros() - g_hash.t0), g_hash.sd_us, g_hash.queue_us, d);
}

static void fl_hash_step(void)
{
    image_config_t *img = fl_image(g_hash.id);
    if (!img) { fl_hash_finish(FL_W_NO_SUCH_IMAGE); return; }
    uint8_t *buf = flFramesReplyDataBuffer();

    while (g_hash.remaining > 0)
    {
        uint32_t n = g_hash.remaining < FL_INJECT_HASH_SLICE_SECTORS
                     ? g_hash.remaining : FL_INJECT_HASH_SLICE_SECTORS;
        const uint32_t t = micros();
        if (!img->file.seek((uint64_t)g_hash.lba * FL_SECTOR_BYTES) ||
            img->file.read(buf, (size_t)n * FL_SECTOR_BYTES) != (ssize_t)n * FL_SECTOR_BYTES)
        {
            fl_hash_finish(FL_W_SD_ERROR);
            return;
        }
        g_hash.sd_us += (uint32_t)(micros() - t);
        flSha256Update(&g_hash.sha, buf, (size_t)n * FL_SECTOR_BYTES);
        g_hash.lba += n;
        g_hash.remaining -= n;
        platform_reset_watchdog();
        fl_drain_timed();                               // keep taking in the host's next frame

        // ⛔ R1 · THE YIELD, between slices, exactly as a preemptible write yields between sectors.
        if (g_hash.remaining > 0 && fl_sampler_wants_bus()) return;
    }
    fl_hash_finish(FL_W_OK);
}

static void fl_do_hash(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                       const uint8_t *p, uint32_t len)
{
    // {id u8, lba u32, count u32, algo u8}
    const uint32_t queue_us = (uint32_t)(micros() - flFramesHeadDoneUs());
    if (len < 10)
    {
        fl_hash_reply(seq, req_crc, req_len, FL_W_BAD_REQUEST, 0, 0, 0, 0, 0, 0, queue_us, nullptr);
        return;
    }
    uint8_t  id    = p[0];
    uint32_t lba   = flGet32(&p[1]);
    uint32_t count = flGet32(&p[5]);
    uint8_t  algo  = p[9];

    image_config_t *img = fl_image(id);
    uint32_t gen = (id & S2S_CFG_TARGET_ID_BITS) < S2S_MAX_TARGETS
                   ? g_write_gen[id & S2S_CFG_TARGET_ID_BITS] : 0;
    uint8_t outcome = FL_W_OK;
    if (!img)                                                          outcome = FL_W_NO_SUCH_IMAGE;
    else if (algo != FL_HASH_SHA256 || count == 0 ||
             count > FL_INJECT_HASH_MAX_SECTORS)                       outcome = FL_W_BAD_REQUEST;
    else if (!fl_range_ok(*img, lba, count))                           outcome = FL_W_OUT_OF_RANGE;
    if (outcome != FL_W_OK)
    {
        fl_hash_reply(seq, req_crc, req_len, outcome, algo, 0, gen, gen, 0, 0, queue_us, nullptr);
        return;
    }

    // The slices write over the reply cache's data area: a replay of the PREVIOUS reply must not
    // be possible from here on, so the cache is dropped before the first slice.
    flFramesInvalidateCache();

    g_hash.active    = true;
    g_hash.id        = id & S2S_CFG_TARGET_ID_BITS;
    g_hash.algo      = algo;
    g_hash.lba       = lba;
    g_hash.remaining = count;
    g_hash.count0    = count;
    g_hash.gen0      = gen;
    g_hash.t0        = micros();
    g_hash.sd_us     = 0;
    g_hash.queue_us  = queue_us;
    g_hash.seq       = seq;
    g_hash.req_crc   = req_crc;
    g_hash.req_len   = req_len;
    flSha256Init(&g_hash.sha);
    fl_hash_step();
}

static void fl_do_watch(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                        const uint8_t *p, uint32_t len)
{
    uint32_t since = (len >= 4) ? flGet32(p) : 0;

    uint8_t *out = flFramesReplyDataBuffer();
    uint32_t n = 0;
    flPut32(&out[n], g_act_next_index); n += 4;
    out[n++] = fl_sampler_wants_bus() ? 0 : 1;                         // 1 = bus free
    flPut32(&out[n], g_saw_sampler ? (uint32_t)(millis() - g_last_sampler_ms) : 0xFFFFFFFFu);
    n += 4;
    uint32_t count_pos = n;
    out[n++] = 0;

    // Oldest still-held index. A `since` older than that means the host fell behind and
    // entries were overwritten; it is told by the gap between `since` and the first index
    // returned, never by a silent truncation.
    uint32_t oldest = (g_act_next_index > FL_ACT_RING) ? g_act_next_index - FL_ACT_RING : 0;
    uint32_t from = since > oldest ? since : oldest;
    uint8_t emitted = 0;
    for (uint32_t i = from; i < g_act_next_index && emitted < FL_ACT_RING; i++)
    {
        const fl_act_t &e = g_act[i % FL_ACT_RING];
        if (e.index != i) continue;                    // overwritten under us
        flPut32(&out[n], e.index); n += 4;
        flPut32(&out[n], e.ms);    n += 4;
        out[n++] = e.id;
        out[n++] = e.op;
        flPut32(&out[n], e.lba);   n += 4;
        out[n++] = (uint8_t)e.count; out[n++] = (uint8_t)(e.count >> 8);
        emitted++;
    }
    out[count_pos] = emitted;
    // FW-BOOTID: ADDITIVE, after the last entry. A reader that stops after `n` entries reads this reply
    // exactly as before; one that knows the field finds it by the length (10 + 16 n + 8). The span is
    // checked at compile time above; this run-time check is the second wall: no room = no field, and the
    // host treats a WATCH without it as a firmware from before it (the clock rule), never an overflow.
    if (n + 8 <= FL_INJECT_REPLY_CAP())
    {
        uint64_t b = fl_boot_id();
        flPut32(&out[n], (uint32_t)b); flPut32(&out[n + 4], (uint32_t)(b >> 32)); n += 8;
    }
    flFramesSendReply(FL_T_WATCH, seq, req_crc, req_len, nullptr, 0, out, n);
}

static void fl_do_sync(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                       const uint8_t *p, uint32_t len)
{
    if (len >= 1 && p[0] != 0xFF)
    {
        image_config_t *img = fl_image(p[0]);
        if (img) img->file.flush();
    }
    SD.card()->syncDevice();
    uint8_t rep = FL_W_OK;
    flFramesSendReply(FL_T_SYNC, seq, req_crc, req_len, &rep, 1, nullptr, 0);
}

static void fl_do_card_mode(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                            const uint8_t *p, uint32_t len)
{
    // A MAGIC, for the same reason REBOOT_UF2 has one: this verb takes the sampler's disks
    // away for ~60 s, and a mistyped frame type must not be able to do that.
    if (len != 4 || memcmp(p, "CARD", 4) != 0)
    {
        uint8_t rep = FL_W_BAD_REQUEST;
        flFramesSendReply(FL_T_CARD_MODE, seq, req_crc, req_len, &rep, 1, nullptr, 0);
        return;
    }

    uint8_t rep = FL_W_OK;
    flFramesSendReply(FL_T_CARD_MODE, seq, req_crc, req_len, &rep, 1, nullptr, 0);
    platform_flush_usb_log();

    // Exactly what the console `s` + `y` keys do, and nothing more: set the watchdog scratch
    // magic and let the main loop's spin_for_reboot close the images cleanly. NO LINE CHANGES
    // INSIDE THE MSC CODE - the card-reader personality is reached the way it always was.
    logmsg("Forge Link inject: CARD_MODE - rebooting with the SD card as a USB drive");
    volatile uint32_t *scratch0 = (uint32_t *)(WATCHDOG_BASE + WATCHDOG_SCRATCH0_OFFSET);
    *scratch0 = REBOOT_INTO_MASS_STORAGE_MAGIC_NUM;
    g_rebooting = true;
}

#ifdef FORGE_LINK_INJECT_TEST
static void fl_do_stall(uint8_t seq, uint32_t req_crc, uint32_t req_len,
                        const uint8_t *p, uint32_t len)
{
    uint16_t ms = (len >= 2) ? (uint16_t)(p[0] | (p[1] << 8)) : 0;
    if (ms > 2000) ms = 2000;
    g_stall_ms = ms;
    uint8_t rep[3] = { FL_W_OK, (uint8_t)ms, (uint8_t)(ms >> 8) };
    flFramesSendReply(FL_T_STALL, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);
}

// Busy-wait at bus-free for the armed duration, once, so M1 can find the first delay the
// S1000 does not tolerate. One-shot: it disarms itself, so a forgotten STALL cannot keep
// firing. ⚠ THIS DELIBERATELY DELAYS THE SAMPLER. It exists only to find where that breaks.
static void fl_stall_if_armed(void)
{
    if (!g_stall_ms) return;
    if (fl_sampler_wants_bus()) return;
    uint16_t ms = g_stall_ms;
    g_stall_ms = 0;
    logmsg("Forge Link inject: STALL ", (int)ms, " ms at bus-free (TEST BUILD)");
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < ms) platform_reset_watchdog();
}
#endif

/*****************************************************************************
 * Dispatch, poll and the banner
 *****************************************************************************/

static void fl_dispatch(void)
{
    const uint8_t *hdr = flFramesHdr();
    uint8_t  type    = hdr[0];
    uint8_t  seq     = hdr[1];
    uint32_t len     = flGet32(&hdr[2]);
    const uint8_t *p = flFramesBody();
    uint32_t req_crc = flGet32(&p[len]);

    // Same sequence and identical bytes: the host did not get our reply. Never apply twice.
    if (type != FL_T_HELLO && flFramesReplayedDuplicate(seq, req_crc, len)) return;

    // ⛔ N6 STEP 2 · SAFE MODE. After K consecutive hardware-watchdog boots the board is a PLAIN
    // DISK: the frame parser still drains (so no payload byte can reach the console key menu, R7),
    // but only HELLO, the watchdog's own status/clear, EXIT and CARD_MODE (a deliberate reboot, the
    // road back to the card) are answered. Everything that touches the card for the Mac is refused
    // with FL_W_SAFE_MODE, before a byte of it is parsed.
    if (forgeLinkHwdogSafeMode() && type != FL_T_HELLO && type != FL_T_HWDOG &&
        type != FL_T_EXIT && type != FL_T_CARD_MODE)
    {
        uint8_t rep = FL_W_SAFE_MODE;
        flFramesSendReply(type, seq, req_crc, len, &rep, 1, nullptr, 0);
        return;
    }

    // ⭐ ARMRETRY r7 · DATACRC: WRITE_SECTORS (13 + count x 512 + 4), SYNC (5) and CARD_MODE (8) carry
    // the session after their payload, on either carrier. One from another session is refused with the
    // one-byte 0x18 before a byte of the card is touched; the old exact lengths pass unguarded. `plen` is
    // what the handlers read (the frame's own length stays the replay key).
    uint32_t plen = len;
    if (type == FL_T_WRITE_SECTORS || type == FL_T_SYNC || type == FL_T_CARD_MODE)
    {
        const uint8_t g = forgeLinkBulkSessionGate(type, p, &plen);
        if (g)
        {
            flFramesSendReply(type, seq, req_crc, len, &g, 1, nullptr, 0);
            return;
        }
    }

    switch (type)
    {
        case FL_T_HELLO:
        {
            // ⛔ HELLO INVALIDATES THE REPLY CACHE, exactly as the probe's does.
            // A host process starts its sequence at 1 every time it runs, so without this a
            // second `fl_inject read` of the same sectors is answered FROM CACHE rather than
            // from the card. That is not merely stale data: a pre-image read that way would
            // punch a hole straight through "every write records its pre-image first".
            flFramesInvalidateCache();
            // Bytes 5..8 are ADDITIVE (stage P). A host from before them reads five bytes and
            // is still correct; a host after them reads the features and never assumes them.
            // Bytes 9..12 are ADDITIVE too (N6 step 2, FL_FEAT_HWDOG): the hardware watchdog's
            // state - armed, this boot was a watchdog reset, safe mode, the count, K, the timeout.
            // Bytes 13..16 are ADDITIVE too (ARMRETRY r7 · DATACRC): the session u32 LE, present
            // exactly when FL_FEAT_DATA_CRC is set (every vendor-bulk build).
#ifdef FORGE_LINK_BULK
            uint8_t rep[17] = { FORGE_LINK_PROTOCOL_VERSION,
#else
            uint8_t rep[13] = { FORGE_LINK_PROTOCOL_VERSION,
#endif
                               0x02,                                      // role: disk + injector
                               (uint8_t)FL_INJECT_MAX_SECTORS,            // write run cap
                               (uint8_t)FL_INJECT_ATOMIC_MAX_SECTORS,     // atomic publish cap
                               (uint8_t)FL_INJECT_READ_MAX_SECTORS,       // read run cap (R1)
                               (uint8_t)(FL_FEAT_HASH | FL_FEAT_QUEUE | FL_FEAT_WTIME2 |
                                         FL_FEAT_HWDOG | FL_FEAT_LAYOUT | FL_FEAT_LABEL
#ifdef FORGE_LINK_BULK
                                         | FL_FEAT_BULK | FL_FEAT_DATA_CRC
#endif
                                         ),
                               (uint8_t)FL_RX_SLOTS,                      // frames the board queues
                               (uint8_t)(FL_INJECT_HASH_MAX_SECTORS & 0xFF),
                               (uint8_t)(FL_INJECT_HASH_MAX_SECTORS >> 8),
                               0, 0, 0, 0 };
            forgeLinkHwdogHelloBytes(&rep[9]);
#ifdef FORGE_LINK_BULK
            flPut32(&rep[13], forgeLinkBulkSession());
#endif
            flFramesSendReply(type, seq, req_crc, len, rep, sizeof(rep), nullptr, 0);
            break;
        }

        case FL_T_EXEC:
            // ⛔ THE BRIDGE IS NOT AVAILABLE IN THE DISK ROLE AND NEVER WILL BE IN THIS BUILD.
            // No role switching, so none of the initiator/target excursion hazards arise.
            {
                uint8_t rep[FL_REPLY_FIXED_LEN];
                memset(rep, 0, sizeof(rep));
                rep[0] = FL_O_NOT_INITIATOR;
                flFramesSendReply(type, seq, req_crc, len, rep, sizeof(rep), nullptr, 0);
            }
            break;

        case FL_T_ECHO:
            // ⛔ N6 STEP 3 · ECHO IS THE ONE VERB THE SHRUNK REPLY CACHE CAN NO LONGER SERVE.
            // Its reply IS the request payload, and the parser accepts a payload up to
            // FL_MAX_PAYLOAD (32,795) - which is unchanged, because the RX slots did not shrink.
            // On an injection build the cache is 8,342 B, so a larger echo would hit the overrun
            // guard in flFramesSendReply and come back as a SILENT EMPTY REPLY: a host measuring
            // throughput would read that as a working link returning nothing. Refuse it BY NAME
            // instead. The probe keeps the 32 KiB cache, so its ECHO sweeps are unaffected.
#ifdef FL_UNIFIED
            if (len > FL_INJECT_REPLY_CAP())            // the union's disk role: inject's 8,320 B
#else
            if (len > flFramesReplyDataCapacity())
#endif
            {
                uint8_t rep = FL_W_BAD_REQUEST;
                flFramesSendReply(type, seq, req_crc, len, &rep, 1, nullptr, 0);
                break;
            }
            flFramesSendReply(type, seq, req_crc, len, p, len, nullptr, 0);
            break;

        case FL_T_EXIT:
            flFramesInvalidateCache();
            flFramesSendReply(type, seq, req_crc, len, nullptr, 0, nullptr, 0);
            break;

        case FL_T_DISK_INFO:     fl_do_disk_info(seq, req_crc, len); break;
        case FL_T_READ_SECTORS:  fl_do_read(seq, req_crc, len, p, len); break;
        case FL_T_WRITE_SECTORS: fl_do_write(seq, req_crc, len, p, plen); break;
        case FL_T_WATCH:         fl_do_watch(seq, req_crc, len, p, len); break;
        case FL_T_SYNC:          fl_do_sync(seq, req_crc, len, p, plen); break;
        case FL_T_CARD_MODE:     fl_do_card_mode(seq, req_crc, len, p, plen); break;
        case FL_T_HASH_SECTORS:  fl_do_hash(seq, req_crc, len, p, len); break;
        case FL_T_HWDOG:         forgeLinkHwdogFrame(type, seq, req_crc, len, p, len); break;

#ifdef FORGE_LINK_INJECT_TEST
        case FL_T_STALL:         fl_do_stall(seq, req_crc, len, p, len); break;
        case FL_T_WDOG_TEST:     forgeLinkHwdogFrame(type, seq, req_crc, len, p, len); break;
#endif

        default:
        {
            uint8_t err = 0xFF;   // unknown frame type
            flFramesSendReply(type, seq, req_crc, len, &err, 1, nullptr, 0);
            break;
        }
    }
}

#ifndef FORGE_LINK_PROBE
// ⛔ THE DIP-ROLE UNION (FL_UNIFIED) links the PROBE's forgeLinkSerialPoll and forgeLinkTxBusy
// instead of these two (the probe's SerialPoll carries this card-mode return false under
// FL_UNIFIED). This build never defines FORGE_LINK_PROBE, so its bytes do not move.

// Called from the platform console input path. Returning FALSE hands the byte stream back to
// the stock key menu, which is exactly what must happen in card-reader mode (B6/R8).
extern "C" bool forgeLinkSerialPoll(void)
{
    if (g_card_mode) return false;
    // ⛔ A RUN IN FLIGHT OWNS THE HEAD RX SLOT. g_run.data points straight into it - the payload
    // is never copied. ~~Draining here would overwrite the very sectors still being written, so
    // a run in flight returned without draining.~~ STAGE P: the parser now fills only a FREE slot
    // (the tail) and never the head, so draining while a run is active is safe - and it is the
    // whole of pipelining on the board side: the next frame streams in while this one is written.
    // When every slot is full the bytes stay in the CDC FIFO, exactly as before. Returning true
    // either way keeps them away from the console key menu (R7: `u` then `y` is the UF2 bootloader).
    flFramesRxDrain();
    return true;
}

extern "C" bool forgeLinkTxBusy(void)
{
    return flFramesTxBusy();
}
#endif // !FORGE_LINK_PROBE

#ifdef FL_UNIFIED
// The union's two seams into this file (declared in ForgeLink_probe.h under FL_UNIFIED).
extern "C" bool forgeLinkInjectCardModeActive(void)
{
    return g_card_mode;
}

// ⛔ C2c · THIS TEXT LIVES IN FLASH, not .rodata: a .cpp.o's .rodata is copied to RAM on this
// template (rp23xx-template.ld excludes *.cpp.o from the flash .rodata; .flashdata* stays in
// flash), and the union needs the room for its free-heap floor. Same text, byte for byte.
// Safe in flash: read once, from setup() via forgeLinkLogBanner, on the main thread - never from
// an IRQ, never while flash is being written. Union-only, so the injection env does not move.
#define FL_COLD_STR(name, text) \
    static const char name[] __attribute__((section(".flashdata.fl_c2c_" #name))) = text
FL_COLD_STR(k_fli_role, " Role: TARGET (disk) + block injector. The bridge/EXEC path is refused in this role.");
FL_COLD_STR(k_fli_verbs, " Block verbs: DISK_INFO READ_SECTORS WRITE_SECTORS WATCH SYNC CARD_MODE HASH_SECTORS");
FL_COLD_STR(k_fli_rxq, " Stage P: RX queue ");
FL_COLD_STR(k_fli_sha, " frames (pipelined writes), SHA-256 verify up to ");
FL_COLD_STR(k_fli_split, " sectors, WRITE reply carries the timing split.");
FL_COLD_STR(k_fli_run, " Write run ");
FL_COLD_STR(k_fli_atomic, " sectors; atomic cap ");
FL_COLD_STR(k_fli_read, "; READ cap ");
FL_COLD_STR(k_fli_quiet, " (a reply does not yield); quiet window ");
FL_COLD_STR(k_fli_design, " ⚠ The atomic cap and the quiet window are DESIGN VALUES, not measurements:");
FL_COLD_STR(k_fli_m1, "   M1 (the sampler's tolerance of a delayed selection) is unmeasured.");
FL_COLD_STR(k_fli_card, " USB serial input is frames only - EXCEPT in card-reader mode, where the");
FL_COLD_STR(k_fli_keys, "   stock key menu is live so 'x' then 'y' still leaves it.");
FL_COLD_STR(k_fli_stall, " STALL verb: COMPILED OUT (this is not a test build).");
#undef FL_COLD_STR

// The disk role's banner lines, printed by the probe's forgeLinkLogBanner when the DIP is OFF.
extern "C" void forgeLinkInjectBannerLines(void)
{
    logmsg(k_fli_role);
    logmsg(k_fli_verbs);
    logmsg(k_fli_rxq, (int)FL_RX_SLOTS, k_fli_sha,
           (int)FL_INJECT_HASH_MAX_SECTORS, k_fli_split);
    logmsg(k_fli_run, (int)FL_INJECT_MAX_SECTORS, k_fli_atomic,
           (int)FL_INJECT_ATOMIC_MAX_SECTORS, k_fli_read, (int)FL_INJECT_READ_MAX_SECTORS,
           k_fli_quiet, (int)FL_INJECT_QUIET_MS, " ms.");
    logmsg(k_fli_design);
    logmsg(k_fli_m1);
    logmsg(k_fli_card);
    logmsg(k_fli_keys);
    logmsg(k_fli_stall);
    forgeLinkHwdogBanner();
}
#endif // FL_UNIFIED

extern "C" void forgeLinkInjectPoll(void)
{
    forgeLinkBulkPoll();   // N6 step 3: retry a refused arm, release the slots when idle
    (void)fl_boot_id();    // FW-BOOTID: drawn on the first pass, so it exists before any frame is served

    // N6 step 2: arm the hardware watchdog on the first main-loop pass, and clear its counter
    // after a healthy run. Before the card-mode return, which it handles itself.
    forgeLinkHwdogPoll(g_card_mode);
    if (g_card_mode) return;

#ifdef FORGE_LINK_INJECT_TEST
    fl_stall_if_armed();
#endif

    // A run in flight owns the loop slot until it finishes. Nothing else is dispatched,
    // which is also what makes a host retry safe: there is no second application to make.
    // Only a PREEMPTIBLE run ever survives its own dispatch: an atomic run either lands
    // whole inside fl_do_write or is refused there with BUSY_NO_QUIET_WINDOW, because the
    // board does not sit waiting for a window - the Mac retries, which is the frame's rule.
    //
    // ⭐ STAGE P · THE QUEUE. Frames are taken strictly in arrival order, one at a time; only the
    // RECEIVING overlaps. A frame's slot is released when its work is done - at once for
    // everything except a WRITE that became a run, whose slot is held until the run replies,
    // because the run reads its payload straight out of it.
    flFramesRxDrain();
    if (g_run.active)
    {
        if (!fl_sampler_wants_bus()) fl_run_step();
        if (!g_run.active) flFramesClearPending();      // the run replied: its slot is free
        return;
    }
    if (g_hash.active)
    {
        if (!fl_sampler_wants_bus()) fl_hash_step();
        return;                                         // its slot was released at dispatch
    }

    if (flFramesNextHead())   // C2a: drops a BULK RESET's abandoned frames first
    {
        fl_dispatch();
        if (!g_run.active) flFramesClearPending();      // done with the head (a started run keeps it)
    }
}

#ifndef FORGE_LINK_PROBE
// ⛔ THE DIP-ROLE UNION links the PROBE's forgeLinkProbeMainLoop, forgeLinkCdbAllowed and
// forgeLinkLogBanner instead of these three: only the initiator branch calls the first two, and
// in the union that branch is the bridge. The disk role still reaches forgeLinkInjectPoll through
// forgeLinkCallbackPoll (ForgeLink_callback.cpp). This build never defines PROBE: no byte moves.

// The probe's entry point, so the platform layer's existing call sites work unchanged.
extern "C" void forgeLinkProbeMainLoop(void) { forgeLinkInjectPoll(); }

// The disk role never runs the initiator allow-list; nothing calls this in this build.
extern "C" bool forgeLinkCdbAllowed(int, const uint8_t *, size_t, const char **) { return false; }

extern "C" void forgeLinkLogBanner(void)
{
    logmsg("=============================================================");
    logmsg(" Forge Link INJECTION build git:" ZULU_BUILD_GIT_REV);
    // ⭐ WHICH FILES WERE DIRTY AT BUILD TIME, so the METHOD RULE ("verify the build on the board
    // by its banner hash") can actually be made: "-dirty" alone never said whether firmware or a
    // doc differed (2026-09-22). The header half landed then; this is the banner half (2026-09-24).
    if (ZULU_BUILD_DIRTY_COUNT == 0) logmsg(" tree: clean at build time");
    else if (ZULU_BUILD_DIRTY_COUNT < 0) logmsg(" tree: UNKNOWN at build time (no build-info header)");
    else logmsg(" tree: DIRTY at build time, ", (int)ZULU_BUILD_DIRTY_COUNT, " path(s): " ZULU_BUILD_DIRTY_FILES);
    logmsg(" A fork of ZuluSCSI-firmware by Rabbit Hole Computing, GPLv3.");
    logmsg(" Role: TARGET (disk). The bridge/EXEC path is refused here.");
    logmsg(" Block verbs: DISK_INFO READ_SECTORS WRITE_SECTORS WATCH SYNC CARD_MODE HASH_SECTORS");
    logmsg(" Stage P: RX queue ", (int)FL_RX_SLOTS, " frames (pipelined writes), SHA-256 verify up to ",
           (int)FL_INJECT_HASH_MAX_SECTORS, " sectors, WRITE reply carries the timing split.");
    logmsg(" Write run ", (int)FL_INJECT_MAX_SECTORS, " sectors; atomic cap ",
           (int)FL_INJECT_ATOMIC_MAX_SECTORS, "; READ cap ", (int)FL_INJECT_READ_MAX_SECTORS,
           " (a reply does not yield); quiet window ", (int)FL_INJECT_QUIET_MS, " ms.");
    logmsg(" ⚠ The atomic cap and the quiet window are DESIGN VALUES, not measurements:");
    logmsg("   M1 (the sampler's tolerance of a delayed selection) is unmeasured.");
    logmsg(" USB serial input is frames only - EXCEPT in card-reader mode, where the");
    logmsg("   stock key menu is live so 'x' then 'y' still leaves it.");
    // ⛔ A TEST-ONLY FAULT PATH MUST ANNOUNCE ITSELF. The 2026-09-19 rule: an injection left
    // compiled in produced an evening of "frame loss" that was our own doing. STALL is
    // compiled out of any build without FORGE_LINK_INJECT_TEST, and says so either way.
#ifdef FORGE_LINK_INJECT_TEST
    logmsg(" ⛔⛔ TEST BUILD: the STALL verb IS COMPILED IN. It deliberately delays the");
    logmsg("   sampler at bus-free. DO NOT KEEP THIS BUILD ON THE BOARD.");
#else
    logmsg(" STALL verb: COMPILED OUT (this is not a test build).");
#endif
    forgeLinkHwdogBanner();
    logmsg(" build marker: " __DATE__ " " __TIME__);
    logmsg("=============================================================");
}
#endif // !FORGE_LINK_PROBE

#endif // FORGE_LINK_INJECT
