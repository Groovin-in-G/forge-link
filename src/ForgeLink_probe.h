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

// Forge Link probe: a USB-serial bridge that lets a host computer send a small,
// allow-listed set of SCSI commands to a device on the bus, in initiator mode.
//
// Only compiled into builds with FORGE_LINK_PROBE defined. In those builds the
// stock initiator imaging scan and USB mass-storage initiator are compiled out,
// and every USB serial input byte goes to the bridge frame parser instead of the
// key menu (so no stray byte can reboot the board).
//
// Wire format (little-endian), shared with tools/forge-link-probe.py:
//   sync[2] = F5 A7 | type[1] | seq[1] | len[4] | payload[len] | crc32[4]
//   crc32 = IEEE CRC-32 (zlib) over type, seq, len and payload.

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// ⭐ THE DIP-ROLE UNION (env ForgeLink_Blaster_unified, 2026-09-30).
// ONE binary carrying BOTH dispatchers: the probe's bridge and the injector's disk verbs. Which one
// runs is chosen by the INITIATOR DIP at boot, exactly as it already is in every build
// (platform_init reads it; zuluscsi_main_loop branches on it every pass). ONE spelling for "the
// union" in source, derived here from the two defines the env sets - never a third -D from the env.
// A build defining PROBE alone or INJECT alone never sees it, so their bytes do not move.
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_INJECT)
#  define FL_UNIFIED 1
#endif

// platform_late_init's DIP log line (ZuluSCSI_platform.cpp). In the probe build a DIP-OFF boot is a
// disk whose bridge refuses; in the union it is the DISK + injector role. Macros, so that file's
// line count - and with it every __LINE__ its asserts carry - does not move for the other envs.
// ⛔ IRQ-REACHABLE CODE STAYS IN RAM IN THE UNION. Lever L3 sent ForgeLink_probe.cpp.o and
// ForgeLink_inject.cpp.o code to XIP flash in ForgeLink_Blaster_unified only; that was REVERTED
// (2026-09-30, "B: all fast"), so today both files run from RAM anyway and these pins are
// belt-and-braces, kept so a future flash placement cannot strand an IRQ path in XIP.
// Every function in those two files that an interrupt, alarm or fault handler can reach is pinned
// to RAM with this: the SDK's __not_in_flash_func shape (a .time_critical.<name> section, which
// the template's `*(.time_critical*)` places in .data), written out so no pico header is needed here.
// In every other build it expands to the bare name, so their bytes do not move. The list, with the
// caller of each, was reviewed ("Review changes 1-4 applied"); the ELF test
// in tools/tests/test_fl_unified.py asserts each one's address is in SRAM (0x2...).
#ifdef FL_UNIFIED
#  define FL_UNIFIED_RAM_FUNC(name) __attribute__((section(".time_critical." #name))) name
#else
#  define FL_UNIFIED_RAM_FUNC(name) name
#endif

#ifdef FL_UNIFIED
#  define FL_DIP_LOG_NAME "Forge Link unified"
#  define FL_DIP_LOG_OFF  "OFF (DISK + injector)"
#else
#  define FL_DIP_LOG_NAME "Forge Link probe"
#  define FL_DIP_LOG_OFF  "OFF (disk; the bridge will refuse)"
#endif

#define FORGE_LINK_PROTOCOL_VERSION 1

#define FORGE_LINK_SYNC0 0xF5
#define FORGE_LINK_SYNC1 0xA7

// Largest data-in allocation and data-out length one EXEC may carry.
#define FORGE_LINK_MAX_DATA 32768  // Forge Link builds drop networking/audio/RM2 (RAM 97% -> 72%), room for sample-sized transfers

// Frame types, host to board
#define FL_T_HELLO          0x01
#define FL_T_EXEC           0x02
#define FL_T_BUS_RESET      0x03
#define FL_T_EXIT           0x04
#define FL_T_REBOOT_UF2     0x05  // payload must be exactly "UF2!"
#define FL_T_ECHO           0x06  // reply carries the request payload back (USB throughput measurement)
// ⭐ ONE-WAY throughput frames (2026-09-19). EVERY "each way" figure in this lane is a round trip
// halved, and ECHO cannot separate the directions because it is strictly one frame in flight. These
// two measure each direction ALONE, which is the gate that decides whether the double-buffered
// receive build is worth doing at all (C99).
#define FL_T_SINK           0x07  // host streams N bytes at us; we discard them and reply with the count
#define FL_T_SOURCE         0x08  // request carries a 4-byte count; we stream that many bytes back
#define FL_T_DIAGLOG        0x0A  // 1 byte: enable/disable the periodic diagnostic logging
#define FL_T_DBUF           0x09  // arm double-buffered bulk OUT for N seconds (0 = off now)

// ---- BLOCK INJECTION (0x20-0x2F), disk-role builds only -----------------------------------
// The board owns SECTORS and GENERATIONS. It never learns what a volume, a directory entry,
// a file or a sample is - that knowledge lives entirely on the Mac. If a verb below needs one
// of those words to describe it, it is on the wrong side of the wall.
// Spec in our own words: docs/block-interface.md. The Mac implements from THAT, never from here.
#define FL_T_DISK_INFO      0x20  // what images are served, and their generations
#define FL_T_READ_SECTORS   0x21  // {id, lba, count} -> generations + data
#define FL_T_WRITE_SECTORS  0x22  // {id, lba, count, mode, flags, expected_gen, data}
#define FL_T_WATCH          0x23  // the sampler-activity ring after an index
#define FL_T_SYNC           0x24  // flush the SD controller's own cache
#define FL_T_CARD_MODE      0x25  // reboot into card-reader mode (payload must be "CARD")
#define FL_T_HASH_SECTORS   0x26  // {id, lba, count u32, algo} -> SHA-256 of the run as READ_SECTORS reads it
// ---- N6 STEP 3 · the vendor-bulk verbs. Command block on bulk OUT, then a separate data phase.
// ⛔ 0x27/0x28 WERE RESERVED FOR THIS BY STEP 2 (see ForgeLink_hwdog.h's wire-constants note);
// 0x29 HWDOG, 0x2E WDOG_TEST and 0x2F STALL are taken.
#define FL_T_WRITE_BULK     0x27  // CB {id,lba,count,mode,flags,expected_gen,data_crc32} + count*512
#define FL_T_SINK_BULK      0x28  // CB {n u32} + n bytes, discarded and counted
#define FL_T_STALL          0x2F  // ⛔ TEST BUILDS ONLY - busy-wait at bus-free, for M1

// HASH_SECTORS algorithms. One, on purpose: the Mac compares it with hashlib, and a second
// algorithm is a second thing to get wrong on a board nobody can single-step.
#define FL_HASH_SHA256      0x01

// HELLO feature bits (the reply's ADDITIVE byte 5). A host reads them and never assumes them:
// an injection build from before this stage sends a 5-byte HELLO and has none of them.
#define FL_FEAT_HASH        0x01  // HASH_SECTORS exists
#define FL_FEAT_QUEUE       0x02  // the RX parser queues rx_slots complete frames IN ORDER and
                                  // never drops bytes past a frame boundary: pipelining is safe
#define FL_FEAT_WTIME2      0x04  // the WRITE reply carries the stage-P timing split (43 bytes)
#define FL_FEAT_BULK        0x08  // N6: the vendor-bulk interface is present and claimable
                                  // (0x10 is FL_FEAT_HWDOG, in ForgeLink_hwdog.h)
#define FL_FEAT_LAYOUT      0x20  // DISK_INFO row byte 9 is `layout` (below), not reserved-zero.
                                  // Injection builds only. NOT the EXEC flag byte's 0x20
                                  // (forge_link_proto.py): a different field entirely.
#define FL_FEAT_LABEL       0x40  // DISK_INFO ends with the SD card's volume label (label_len u8 +
                                  // UTF-8), after the last image row. Injection builds only.
#define FL_FEAT_DATA_CRC    0x80  // ARMRETRY r7 · DATACRC: the session + data_crc32 trailers
                                  // are understood (block-interface §1d), and the HELLO carries the
                                  // session u32 at its very end. Every FORGE_LINK_BULK build, both roles.
#define FL_LABEL_MAX        33    // 11 characters, up to 3 UTF-8 bytes each (exFAT is UTF-16)

// DISK_INFO row byte 9 when FL_FEAT_LAYOUT is set: is the image in one piece on the card?
// Read from ImageBackingStore::isContiguous() - a bit set at open and cleared LIVE if the board
// ever drops to the slow path (revert_to_noncontiguous), so 0x02 can also mean "fell back since
// boot". Never contiguousRange() at DISK_INFO time: on a fragmented file that walks the FAT.
#define FL_LAYOUT_NOT_TESTED 0x00 // rom / raw / folder: contiguity does not apply (ZuluSCSI_disk.cpp)
#define FL_LAYOUT_ONE_PIECE  0x01
#define FL_LAYOUT_FRAGMENTED 0x02

// Board to host: request type | 0x80
#define FL_T_REPLY_BIT      0x80

// WRITE_SECTORS modes
#define FL_WM_PREEMPTIBLE   0x00  // payload: one sector at a time, yield to a pending selection
#define FL_WM_ATOMIC        0x01  // publish: the whole run, no yield, only in a quiet window

// WRITE_SECTORS flags
#define FL_WF_CONDITIONAL   0x01  // bit 0: refuse unless the image's write generation matches

// Block-verb outcome codes (the full enum; every reply carries exactly one)
#define FL_W_OK                 0x00
#define FL_W_BUSY_NO_QUIET      0x01  // atomic run found no quiet window; retry
#define FL_W_GEN_MISMATCH       0x02  // the sampler wrote this image since the Mac read it
#define FL_W_OUT_OF_RANGE       0x03  // run would pass the end of the image file
#define FL_W_READ_ONLY          0x04  // image not writable
#define FL_W_SD_ERROR           0x05  // seek/read/write failed on the card
#define FL_W_BAD_REQUEST        0x06  // malformed payload, or count out of bounds for the mode
#define FL_W_NO_SUCH_IMAGE      0x07  // no image served at that id
// ⛔ N6: DISTINCT FROM 0x01 ON PURPOSE. 0x01 (no quiet window) is TRANSIENT and the Mac retries
// it; 0x08 means an ATOMIC run arrived while the queue held other work, which retrying cannot
// clear, so the Mac STOPS. Collapsing the two turns a stop into an infinite retry.
#define FL_W_QUEUE_NOT_EMPTY    0x08

// Activity-ring operation codes (WATCH)
#define FL_ACT_READ         0x00
#define FL_ACT_WRITE        0x01

// Largest run one WRITE_SECTORS may carry, in 512-byte sectors. A write YIELDS PER SECTOR in
// preemptible mode, so the run length does not bound the stall and 64 is free.
#define FL_INJECT_MAX_SECTORS 64

// ⛔ READS ARE CAPPED FAR LOWER, AND THIS IS AN R1 CAP, NOT A BUFFER CAP.
// A READ_SECTORS REPLY has no yield in it: flFramesSendReply -> fl_write_all loops until the
// CDC drains, calling only platform_reset_watchdog and never scsiPoll. At the measured
// 560 KiB/s board->Mac that is ~58 ms of dead main loop for a 32 KiB reply - and the Mac reads
// the 16 KiB partition header before EVERY push, so this is the common path, not a corner.
// 16 sectors = 8 KiB bounds it to ~15 ms plus one SD read.
//
// ⚠ THIS IS A PLACEHOLDER WITH A NUMBER'S CONFIDENCE. M1 - what delay the S1000 actually
// tolerates on a selection - is unmeasured, so 16 is chosen to be comfortably small rather
// than to be right. The design target remains §3's sliced reply with a bus-free check between
// slices; that is worth building once M1 says what the budget is, and not before.
#define FL_INJECT_READ_MAX_SECTORS 16
// Atomic runs are capped far lower, because the whole run is applied without yielding and
// the sampler waits for all of it. 32 sectors = 16 KiB = the partition header the S1000
// re-reads in one command. ⚠ THE REAL CAP IS SET BY M1 (the S1000's tolerance of a delayed
// selection), which has not been measured. 32 is the frame's value, not a measured one.
#define FL_INJECT_ATOMIC_MAX_SECTORS 32
// A quiet window: no sampler command for this long before an atomic run may start.
#define FL_INJECT_QUIET_MS 50

// HASH_SECTORS: the largest run one request may fingerprint (8 MiB), and the slice it reads
// between yields. The slice is the READ cap on purpose - 16 sectors is the unyielded SD read
// already judged acceptable for R1 - and the job yields to a pending selection between slices,
// exactly as a preemptible write yields between sectors. The cap bounds how long ONE reply
// takes, not how long the sampler waits.
#define FL_INJECT_HASH_MAX_SECTORS   16384
#define FL_INJECT_HASH_SLICE_SECTORS 16

// EXEC flags
#define FL_F_DATA_OUT       0x01
#define FL_F_DATA_IN        0x02
#define FL_F_IDENTIFY       0x04
#define FL_F_BUS_RESET      0x08
#define FL_F_NO_INIT_ID     0x10  // SCSI-1 single-initiator selection: data = target bit only
#define FL_F_RELEASE_BSY    0x20  // after selection, release OUR BSY for timeout_ms (W34 caveat)
// BLIND CDB (W35): after selection, clock the CDB out on fixed timing WITHOUT reading REQ,
// two blind ACKs for STATUS and MESSAGE IN, then watch BSY for 3 s. Refused (FL_O_BAD_REQUEST,
// nothing on the bus) together with IDENTIFY, RELEASE_BSY, BUS_RESET or a data phase. In a blind
// EXEC, timeout_ms 1..2000 is the settle wait after selection in ms; larger = the 2 ms default.
#define FL_F_BLIND          0x40
// ⭐ N6 PROBE-BULK SLICE 2 · THE CHAIN LATCH (frame item 11). A CHAINED EXEC runs only if the EXEC
// before it succeeded (flExecChainSuccess below); otherwise it is answered FL_O_CHAIN_BROKEN and
// the target is never selected. Any EXEC that fails sets the latch, any that succeeds clears it,
// and so does HELLO. The Mac sets the flag on every EXEC of a pipelined run but its first.
#define FL_F_CHAINED        0x80

// EXEC_REPLY outcome codes
#define FL_O_COMPLETED      0x00  // COMMAND COMPLETE received; status byte is valid
#define FL_O_NO_SELECT      0x01  // target did not answer selection (or bus busy: upstream select can't tell)
#define FL_O_BUS_BUSY       0x02  // reserved: not distinguishable with upstream select
#define FL_O_PHASE_ERROR    0x03  // unexpected phase / bus free without COMMAND COMPLETE
#define FL_O_TIMEOUT        0x04
#define FL_O_BUS_RESET      0x05  // reset seen or requested mid-command
#define FL_O_REJECTED       0x10  // refused by the safety allow-list; nothing was put on the bus
#define FL_O_BAD_REQUEST    0x11  // malformed EXEC payload; nothing was put on the bus
#define FL_O_NOT_INITIATOR  0x12  // board booted in target (disk) role; bridge cannot run
#define FL_O_DATA_OVERFLOW  0x13  // target offered more DATA_IN than the allocation
#define FL_O_CHAIN_BROKEN   0x14  // FL_F_CHAINED and the EXEC before failed; nothing was put on the bus
// (fetch-from-slot): a duplicate of an EXEC whose reply was sent from its rx slot. The data
// is gone (the slot was reused), so the reply is {0x15, status 0xFF, received 0, sent 0} and the
// command is NEVER re-executed; the Mac raises it by name. Only the probe+bulk build sends it.
#define FL_O_REPLAY_UNAVAILABLE 0x15

// The chain latch's SUCCESS: fl_rear's own judgement of a reply (tools/fl_rear.py Session.ok and
// answered_no_data), mirrored by forge_link_proto.chain_success and driven against it on a desk
// by test_n6_pair. COMPLETED with GOOD status and every data-out byte sent, or the S1000's
// no-status-phase shape (PHASE_ERROR: the bus went free after the data) with every declared byte
// moved both ways. Anything else - rejected, no select, timeout, reset, short - is a failure.
// C51 (deep review): the phase-error shape is a success only where NO status came back
// (0xFF, the value fl_execute and the guard's fl_lg_send_off start from) or GOOD did. A STATUS phase
// that said CHECK CONDITION / BUSY is a refusal, whatever the byte counts: before this, a sampler
// REFUSING the link guard's own 09 off (PHASE_ERROR, status 0x02, nothing to move) counted as done.
static inline bool flExecChainSuccess(uint8_t outcome, uint8_t status, uint32_t sent, uint32_t out_len,
                                      uint32_t received, uint32_t in_alloc)
{
    if (outcome == FL_O_COMPLETED) return status == 0x00 && sent >= out_len;
    if (outcome == FL_O_PHASE_ERROR)
        return (status == 0xFF || status == 0x00) && sent >= out_len && received >= in_alloc;
    return false;
}

#ifdef __cplusplus
extern "C" {
#endif

// Safety allow-list. Returns true if this CDB may be sent to target_id.
// Accepts only 6-byte INQUIRY (12h), TEST UNIT READY (00h), and - only when a
// prior standard INQUIRY this session reported peripheral type 03h (processor)
// at LUN 0 - SEND (0Ah) and RECEIVE (08h). Everything else is refused.
// If reason is non-null it is set to a short static explanation on refusal.
bool forgeLinkCdbAllowed(int target_id, const uint8_t *cdb, size_t cdbLen, const char **reason);

// Called from the platform serial menu. Drains USB serial input into the frame
// parser. Returns true when it consumed the input (always, in probe builds).
bool forgeLinkSerialPoll(void);

// True while the bridge is writing a frame; the USB log pump must not interleave.
bool forgeLinkTxBusy(void);

// Called from the main loop (never from platform_poll): executes a pending frame.
void forgeLinkProbeMainLoop(void);

// Logs the identifying banner.
void forgeLinkLogBanner(void);

#ifdef FL_UNIFIED
// The union only (ForgeLink_inject.cpp). The probe's forgeLinkSerialPoll asks the first so that,
// in card-reader mode, the console key menu is live (`u` then `y` - the DIP-OFF flash route) as it
// is in the injection build; the probe's banner calls the second for the disk role's lines.
bool forgeLinkInjectCardModeActive(void);
void forgeLinkInjectBannerLines(void);
#endif

#ifdef __cplusplus
}
#endif

// TQ2 · NOTE_AT's hooks, included only with the flag (host tests copy this header alone). Without it the
// bridge's log save stays `if (1) save_logfile();` (ZuluSCSI.cpp, same line).
#ifdef FORGE_LINK_NOTE_AT
#include "ForgeLink_noteat.h"
#else
#define FL_NOTE_AT_LOG_SAVE_OK() 1
#endif

// ⛔ QX1 · THE RELEASE BUILD'S QUIET EXEC (PART 2 of the 36 h run).
// L1 took the per-phase trace and the debug log off every EXEC but KEPT two costs on the ship build:
//   (a) a small EXEC's non-debug "EXEC id" / "EXEC done" UART lines (~229 B, ~2.3 ms at 1 Mbaud, blocking,
//       before the reply goes out), and
//   (b) with a buyer's DEBUG DIP on, the bridge loop's per-pass SD log save (save_logfile saves on EVERY
//       pass whose log grew while g_log_debug is set, ZuluSCSI.cpp save_logfile + the bridge-role call).
// FL_RELEASE_QUIET_EXEC drops both. It is DEFINED ONLY BY THE SHIP ENV (platformio.ini,
// ForgeLink_Blaster_unified: -DFORGE_LINK_RELEASE_QUIET_EXEC), and it is derived HERE so that every bench
// build that inherits that env's flags stays exactly as it was: a build with the trace
// (-DFORGE_LINK_EXEC_TRACE, the bench-trace UF2) or the stamps (-DFORGE_LINK_STAMPS, both stamps envs) never
// gets it - bench tools (scripts/c154-quiet-ab.py) key on the EXEC id / done lines.
// ⛔ ERRORS ARE NEVER SILENCED: a failed EXEC (outcome not COMPLETED, or a non-zero SCSI status) still logs
// its EXEC done line, "quiet EXEC failed" and "EXEC refused" are untouched, and the log still reaches the SD
// card within LOG_SAVE_INTERVAL_MS (1 s) - the DIP only stops forcing a save on every pass.
#if defined(FORGE_LINK_RELEASE_QUIET_EXEC) && !defined(FORGE_LINK_EXEC_TRACE) && !defined(FORGE_LINK_STAMPS)
#define FL_RELEASE_QUIET_EXEC 1
#endif
// QX1 review: with the EXEC id line gone, the ship build's "EXEC done" line (a failure, in practice) would name
// neither the target nor the command. These extra logmsg arguments add the SCSI ID and the opcode (cdb[0]) on
// the ship build only; empty everywhere else, so every bench build's line - and bytes - stay as they were.
#ifdef FL_RELEASE_QUIET_EXEC
#define FL_QX1_DONE_WHO(target, op) , " id ", (int)(target), " op ", (uint8_t)(op)
#else
#define FL_QX1_DONE_WHO(target, op)
#endif
// (b) as a MACRO, so ZuluSCSI.cpp's line count - and every __LINE__ it carries - does not move: the bridge
// role's save (ZuluSCSI.cpp, after scsiInitiatorMainLoop) runs with g_log_debug cleared on the ship build
// (save, clear, restore - fl_execute's own move), and is the bare save_logfile() in every other build.
// ⭐ C14 · NO SD SAVE WHILE MIDI FLOWS. An SD flush blocks the bridge loop - FL62
// measured link stalls of 173-748 ms whenever lines were logged in play - so on the ship build a log that grew
// mid-play waits for a quiet gap: no host frame for FL_LOGSAVE_QUIET_MS. FL_LOGSAVE_MAX_DEFER_MS caps the wait
// so an error line still reaches the card during a song that never pauses. The CDC/UART log is untouched.
#define FL_LOGSAVE_QUIET_MS      5000u
#define FL_LOGSAVE_MAX_DEFER_MS 60000u
#ifdef __cplusplus
extern "C" {
#endif
bool forgeLinkBridgeLogSaveOk(void);
#ifdef __cplusplus
}
#endif
#ifdef FL_RELEASE_QUIET_EXEC
#define FL_BRIDGE_SAVE_LOGFILE() \
    do { if (!forgeLinkBridgeLogSaveOk()) break; const bool fl_qx1_dbg = g_log_debug; g_log_debug = false; save_logfile(); g_log_debug = fl_qx1_dbg; } while (0)
#else
#define FL_BRIDGE_SAVE_LOGFILE() save_logfile()
#endif
