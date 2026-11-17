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

// Forge Link probe: USB serial bridge + allow-listed SCSI command executor.
// See ForgeLink_probe.h for the wire format and README.md for the safety model.

#include "ForgeLink_probe.h"
#include "ForgeLink_frames.h"
#include "ForgeLink_bulk.h"
#include "ForgeLink_linkguard.h"
#include "ForgeLink_stamps.h"   // DSP2: bench-only us stamps; every FL_STAMP is a no-op without -DFORGE_LINK_STAMPS
#ifdef FL_RXSTAMP   // RX-SLOW: included ONLY in the bench build, so the ship build's link is unchanged
#include "ForgeLink_rxstamp.h"
#endif

#ifdef FORGE_LINK_PROBE

#include <Arduino.h>
#include <string.h>
#include "ZuluSCSI_config.h"
#include "ZuluSCSI_log.h"
#include "ZuluSCSI_initiator.h"
#include <ZuluSCSI_platform.h>
#ifdef FL_UNIFIED
#include "ForgeLink_hwdog.h"   // the hardware watchdog in the bridge role too
#endif
#include "ForgeLink_uf2_reboot.h"   // C90: REBOOT_UF2 commits through the BOOTSEL's path (every probe env)

#if __has_include("ZuluSCSI_build_info_generated.h")
#include "ZuluSCSI_build_info_generated.h"
#endif
#ifndef ZULU_BUILD_NUMBER
#define ZULU_BUILD_NUMBER "0"          // L4: 0 = the build could not say (no generated header)
#endif
#ifndef ZULU_BUILD_GIT_REV
#define ZULU_BUILD_GIT_REV "unknown"
#endif
#ifndef ZULU_BUILD_DIRTY_COUNT
#define ZULU_BUILD_DIRTY_COUNT -1          // -1 = the build could not say (no generated header)
#define ZULU_BUILD_DIRTY_FILES ""
#endif

#ifdef PLATFORM_HAS_INITIATOR_MODE
#include <scsiHostPhy.h>
#include <scsi2sd.h>
extern "C" {
#include <scsi.h>
}
#include "pico/bootrom.h"
#endif

#ifdef FL_UNIFIED
// The HELLO ident's prefix: every Mac gate keys on startswith("Forge Link") and the `git:` token
// only (forge_link_proto.parse_hello; fl_rear.hello_git), so the word after it is free.
#define FORGE_LINK_NAME "Forge Link unified"
#else
#define FORGE_LINK_NAME "Forge Link probe"
#endif

// Not in the SCSI_MESSAGE enum in scsi.h
#define FL_MSG_NO_OPERATION 0x08
#define FL_MSG_IDENTIFY_LUN0_NODISC 0x80

// The frame header, the CRC and the byte helpers now live in ForgeLink_frames.h/.cpp, so the
// injector can ride the SAME parser rather than growing a second one. These two names are kept
// as thin aliases only because they read better in the EXEC code below.
#define fl_get32 flGet32
#define fl_put32 flPut32

/*************************
 * Safety allow-list     *
 *************************/

// ⭐ U1 · THE USB LOSS GENERATIONS (ForgeLink_linkguard.h says why). The
// ONLY TinyUSB callbacks Forge Link defines. Strong definitions override TinyUSB's TU_ATTR_WEAK
// defaults (usbd.c); nothing in the core or upstream defines either. They live HERE because this
// arm of this .cpp is bridge-only (-DFORGE_LINK_PROBE) and RAM-placed (rp23xx-template.ld sends
// *.cpp.o .text to RAM): no other env gets an extra object or a moved veneer.
//
// ⛔ They run inside tud_task, which the core calls from the 1 ms alarm user IRQ (USB.cpp usbIRQ)
// AND in thread mode inside SerialUSB calls (SerialUSB.cpp), both under USB.mutex - so each counter
// has ONE writer either way, and the main loop only loads it. Each callback is one increment: no
// allocation, no logging, no flush, no call of any kind.
volatile uint32_t g_fl_usb_suspend_gen;
volatile uint32_t g_fl_usb_mount_gen;

extern "C" void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    g_fl_usb_suspend_gen = g_fl_usb_suspend_gen + 1u;
}

extern "C" void tud_mount_cb(void)
{
    g_fl_usb_mount_gen = g_fl_usb_mount_gen + 1u;
}

// Peripheral device type per target ID, from the last standard INQUIRY this
// session (LUN 0, qualifier 0). 0xFF = unknown / not yet inquired.
static uint8_t g_fl_periph_type[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static void fl_forget_types()
{
    memset(g_fl_periph_type, 0xFF, sizeof(g_fl_periph_type));
}

extern "C" bool forgeLinkCdbAllowed(int target_id, const uint8_t *cdb, size_t cdbLen, const char **reason)
{
    const char *dummy;
    if (!reason) reason = &dummy;

    if (target_id < 0 || target_id > 7) { *reason = "target id out of range"; return false; }
    if (!cdb || cdbLen != 6) { *reason = "only 6-byte CDBs are allowed"; return false; }

    switch (cdb[0])
    {
        case 0x12: // INQUIRY
        case 0x00: // TEST UNIT READY
            return true;

        // REQUEST SENSE, a READ of why the target last answered CHECK CONDITION (2026-09-29, S3000XL
        // plan step 4: the XL refused an SMDI SEND with CHECK CONDITION and its sense was unreadable).
        // LUN 0 only, allocation <= 32 bytes; it changes nothing on the target.
        case 0x03:
            if ((cdb[1] & 0xE0) != 0) { *reason = "REQUEST SENSE only to LUN 0"; return false; }
            if (cdb[4] > 32) { *reason = "REQUEST SENSE allocation over 32 bytes"; return false; }
            return true;

        case 0x0A: // SEND (processor) == WRITE(6) on a disk
        case 0x08: // RECEIVE (processor) == READ(6) on a disk
            if ((cdb[1] & 0xE0) != 0)
            {
                *reason = "SEND/RECEIVE only to LUN 0";
                return false;
            }
            if (g_fl_periph_type[target_id] != 0x03)
            {
                *reason = "SEND/RECEIVE need a prior INQUIRY this session reporting type 03h (processor)";
                return false;
            }
            return true;

        // Akai "SCSI 2 MIDI" host-driven transport (published for the S3000XL OS 2.00:
        // 09 init, 0C send SysEx, 0D poll 3 bytes, 0E read). Ruled 2026-09-18:
        // allowed tonight to ID 6 ONLY (the S1000's own ID). First use is read-only
        // (0D poll; then a SysEx REQUEST in 0C). Nothing on a disk ID can receive these.
        // C46 (deep review): 09 ONLY as the exact LINK ON (09 00 01 01 00 00) or LINK OFF
        // (09 00 00 00 00 00), FL_LINK_GUARD_CDB_ON / _OFF. The link guard and note-at's pre-OFF flush
        // recognise only those two, and byte 2 is what the S1000's ROM reads as the link setting
        // (W35), so any other 09 could switch the link on without the guard remembering it. Spelled
        // byte by byte: a .cpp's .rodata is RAM.
        case 0x09:
            if (target_id != 6) { *reason = "Akai 09/0C/0D/0E only to ID 6 (ruling 2026-09-18)"; return false; }
            if (cdb[1] != 0 || cdb[4] != 0 || cdb[5] != 0 || cdb[2] != cdb[3] || cdb[2] > 1)
            {
                *reason = "Akai 09 only as exactly 09 00 01 01 00 00 (link on) or 09 00 00 00 00 00 (link off)";
                return false;
            }
            return true;
        case 0x0C:
        case 0x0D:
        case 0x0E:
            if (target_id != 6) { *reason = "Akai 09/0C/0D/0E only to ID 6 (ruling 2026-09-18)"; return false; }
            return true;

        default:
            *reason = "opcode not on the allow-list (12h, 00h, 03h; 0Ah, 08h; 09/0C/0D/0E to ID 6)";
            return false;
    }
}

/*************************
 * USB frame I/O         *
 *************************/

// ⭐ THE PARSER, THE REPLY CACHE AND THE REPLAY RULE MOVED TO ForgeLink_frames.cpp (2026-09-20).
// They were never probe-specific; they were merely probe-resident. The injection build needs the
// identical wire behaviour, and a second copy of a frame parser is a second truth - so there is
// one copy, and this file is now one of its two consumers. Nothing about the bytes changed.

extern "C" bool FL_UNIFIED_RAM_FUNC(forgeLinkTxBusy)(void)   // IRQ-reachable: usb_log_poll
{
    return flFramesTxBusy();
}

// ---- N3 · THE NAK LEDGER (2026-09-19) ----------------------------------------------------
// WHY: the framed USB link plateaus at ~145 KiB/s one way. Eight hypotheses are excluded by
// measurement (C75-C85), macOS is exonerated by a control on a second CDC device doing
// ~1 MB/s through the identical host stack (C86), and the suspected mechanism (C89) is that the
// RP2 device controller SINGLE-BUFFERS every device OUT endpoint - rp2040_usb.c's `force_single`,
// with the reason in its own comment - so the endpoint cannot re-arm between packets, NAKs every
// token, and this host then backs off ~430 us before retrying. 2.3 packets/ms = the measured rate.
//
// This counts the NAKs. INTERRUPT_ON_NAK is bit 16 of each endpoint control word (RP2350.svd,
// "Trigger an interrupt if a NAK is sent. Intended for debug only."). The SIE interrupt enable
// stays OFF, so nothing new fires; we only poll the write-clear status register.
// ⚠ The count is a LOWER BOUND - several NAKs inside one poll collapse into one. The GAP is the
// reading: ~430 us between a NAK and the next completion is the host backing off.
#include "hardware/structs/usb.h"
#include "hardware/structs/usb_dpram.h"
#define FL_EP_CTRL_ENABLE      (1u << 31)
#define FL_EP_CTRL_DOUBLE_BUF  (1u << 30)
#define FL_EP_CTRL_INT_ON_NAK  (1u << 16)

static bool     g_fl_nak_armed = false;
static uint32_t g_fl_nak_count[16][2];     // [endpoint][0=IN, 1=OUT]
static uint32_t g_fl_nak_last_us = 0;
static uint32_t g_fl_gap_sum_us = 0, g_fl_gap_n = 0, g_fl_gap_max_us = 0;

static void fl_nak_arm(void)
{
    // Set the debug NAK bit on every ENABLED device endpoint. The dcd read-modify-writes this
    // word in hw_endpoint_start_next_buffer and never touches bit 16, so it survives.
    for (int ep = 1; ep < 16; ep++)
    {
        io_rw_32 *in  = &usb_dpram->ep_ctrl[ep - 1].in;
        io_rw_32 *out = &usb_dpram->ep_ctrl[ep - 1].out;
        if (*in  & FL_EP_CTRL_ENABLE) *in  |= FL_EP_CTRL_INT_ON_NAK;
        if (*out & FL_EP_CTRL_ENABLE) *out |= FL_EP_CTRL_INT_ON_NAK;
    }
    g_fl_nak_armed = true;
    logmsg("FL NAK: armed. Endpoint map (E=enabled, D=double-buffered):");
    for (int ep = 1; ep < 16; ep++)
    {
        uint32_t i = usb_dpram->ep_ctrl[ep - 1].in, o = usb_dpram->ep_ctrl[ep - 1].out;
        if (!(i & FL_EP_CTRL_ENABLE) && !(o & FL_EP_CTRL_ENABLE)) continue;
        logmsg("FL NAK:   EP", ep,
               (i & FL_EP_CTRL_ENABLE) ? "  IN E" : "  IN -", (i & FL_EP_CTRL_DOUBLE_BUF) ? "D" : "s",
               (o & FL_EP_CTRL_ENABLE) ? "  OUT E" : "  OUT -", (o & FL_EP_CTRL_DOUBLE_BUF) ? "D" : "s");
    }
}

// ⛔ POSITIVE CONTROL for the NAK ledger (2026-09-19). C92 read "EP1 OUT NAKs 2-4/sec while
// receiving" as "the device is innocent". That reading is only worth anything if this counter can
// SEE an OUT NAK at all — and it has never been shown to. So: deliberately stall the OUT endpoint
// for 5 ms once a second while the host is writing. If NAKs on EP1 OUT do not jump, the counter is
// blind and C92 proves nothing. (The estate rule: a guard's test set must contain the shape that
// is NOT the incident, or it only proves it fires on what you already knew.)
// ⛔⛔ DEFAULT OFF SINCE 2026-09-19 LATE, AND THIS IS THE POINT OF THE WHOLE EVENING.
// This is a FAULT INJECTION. It disables EP1 OUT for 5 ms once a second, and the silicon is
// blunter than the comment above assumed: "The device will not reply to any packets for this
// endpoint if this bit is not set" (pico-sdk rp2350 usb_device_dpram.h:150). That is SILENCE,
// not a NAK - so the host sees transaction timeouts, exhausts CErr, halts the endpoint and
// runs recovery, and a packet can be dropped on the way back (data-toggle mismatch).
//
// It was left ON in what everyone then called the stable build, and its ~2-4% frame loss was
// measured for hours and reported as the board's natural behaviour. It also defeated the
// logging A/B: this function's logmsg is NOT gated by g_fl_diag_log, so the fault fired in
// BOTH arms and the "logging makes no difference" result was worthless.
//
// ⇒ A DIAGNOSTIC THAT INJECTS A FAULT MUST DEFAULT OFF AND MUST BE VISIBLE IN THE BANNER.
static volatile bool g_fl_nak_control = false;
// build: the probe personality accepts bridge frames ONLY and ignores console keys,
// so '!K' could never reach it. That trap has now cost a flash and a control run.
static void fl_nak_positive_control(void)
{
    if (!g_fl_nak_control) return;
    static uint32_t next_ms = 0;
    uint32_t now = millis();
    if ((int32_t)(now - next_ms) < 0) return;
    next_ms = now + 1000;
    // Disable the OUT endpoint for 5 ms: the host's tokens must now be refused.
    io_rw_32 *out = &usb_dpram->ep_ctrl[0].out;      // EP1 OUT = CDC data out
    uint32_t save = *out;
    *out = save & ~FL_EP_CTRL_ENABLE;
    busy_wait_us_32(5000);
    *out = save;
    logmsg("FL NAK: positive control fired - EP1 OUT held down 5 ms");
}

static void fl_nak_poll(void)
{
    if (!g_fl_nak_armed) return;
    uint32_t st = usb_hw->ep_nak_stall_status;
    if (st)
    {
        usb_hw->ep_nak_stall_status = st;              // write-clear
        uint32_t now = time_us_32();
        if (g_fl_nak_last_us)
        {
            uint32_t gap = now - g_fl_nak_last_us;
            if (gap < 100000) { g_fl_gap_sum_us += gap; g_fl_gap_n++;
                                if (gap > g_fl_gap_max_us) g_fl_gap_max_us = gap; }
        }
        g_fl_nak_last_us = now;
        for (int ep = 0; ep < 16; ep++)
        {
            if (st & (1u << (2 * ep)))     g_fl_nak_count[ep][0]++;   // IN
            if (st & (1u << (2 * ep + 1))) g_fl_nak_count[ep][1]++;   // OUT
        }
    }
}

static void fl_nak_report(uint32_t window_ms)
{
    uint32_t tot = 0;
    for (int ep = 0; ep < 16; ep++) tot += g_fl_nak_count[ep][0] + g_fl_nak_count[ep][1];
    if (!tot) { logmsg("FL NAK: none in ", (int)window_ms, " ms"); return; }
    for (int ep = 0; ep < 16; ep++)
    {
        if (g_fl_nak_count[ep][0] || g_fl_nak_count[ep][1])
            logmsg("FL NAK: EP", ep, "  IN ", (int)g_fl_nak_count[ep][0],
                   "  OUT ", (int)g_fl_nak_count[ep][1], "  per second: ",
                   (int)((g_fl_nak_count[ep][0] + g_fl_nak_count[ep][1]) * 1000 / (window_ms ? window_ms : 1)));
    }
    if (g_fl_gap_n)
        logmsg("FL NAK: gap between NAK events: mean ", (int)(g_fl_gap_sum_us / g_fl_gap_n),
               " us, max ", (int)g_fl_gap_max_us, " us, samples ", (int)g_fl_gap_n);
    memset((void *)g_fl_nak_count, 0, sizeof(g_fl_nak_count));
    g_fl_gap_sum_us = 0; g_fl_gap_n = 0; g_fl_gap_max_us = 0;
}

// A/B switch for the periodic diagnostic logging. The board's OWN counters point here:
// oversize climbs +1 per 5 s window - exactly this log cadence - while crc_errors stays
// at 0. A log burst shares the CDC pipe with frames and can stall the RX parser past its
// 200 ms reset, after which it resyncs on random payload bytes and discards a good frame.
static volatile bool g_fl_diag_log = true;
// Per-EXEC trace for chained fills: OFF, because every line blocks UART0 (~1 ms) before the reply.
// Compile-time on purpose - a bench that wants it builds with -DFORGE_LINK_EXEC_TRACE.
#ifdef FORGE_LINK_EXEC_TRACE
static const bool g_fl_exec_trace = true;
#else
static const bool g_fl_exec_trace = false;
#endif
// QX1: the SHIP build's small EXECs drop their EXEC id / done lines too (ForgeLink_probe.h,
// FL_RELEASE_QUIET_EXEC - never defined with the trace or the stamps). A failure still logs.
#ifdef FL_RELEASE_QUIET_EXEC
static const bool g_fl_release_quiet_exec = true;
#else
static const bool g_fl_release_quiet_exec = false;
#endif
// ⭐ L4 (2026-10-06) · THE HELLO IDENT SAYS WHICH OF THE TWO THIS IS: ` trace:on` / ` trace:off`, from
// the same #ifdef as g_fl_exec_trace (a const bool cannot be pasted into a string literal, so the
// token cannot drift from it: both sides of ONE preprocessor test). tools/fl_release_gate.py refuses
// a UF2 whose ident says anything but trace:off - 2026-10-06: the trace must be turned off in
// the version that ships.
#ifdef FORGE_LINK_EXEC_TRACE
#define FL_IDENT_TRACE " trace:on"
#else
#define FL_IDENT_TRACE " trace:off"
#endif
// DSP2 · a STAMPS build says so in its ident, AFTER every other token (parsers ignore unknown ones);
// tools/fl_release_gate.py refuses an ident carrying stamps:on. Empty in every other build.
// TQ2 · docs/note-at.md §1: `note-at` joins caps: only when the feature is built in. It rides the
// caps: token, which exists only with the vendor-bulk interface, so NOTE_AT requires that flag too.
#ifdef FORGE_LINK_NOTE_AT
#ifndef FORGE_LINK_BULK
#error "FORGE_LINK_NOTE_AT needs FORGE_LINK_BULK (its caps: token and its data-phase verbs)"
#endif
#define FL_IDENT_NOTE_AT ",note-at"
#else
#define FL_IDENT_NOTE_AT ""
#endif
// ⭐ ARMRETRY r7 · DATACRC: `data-crc` joins caps: in every vendor-bulk build (the trailers live in
// the bulk driver and the dispatchers' session gate). The session itself rides the HELLO tail (below).
#define FL_IDENT_DATA_CRC ",data-crc"
#ifdef FL_STAMPS_ON
#define FL_IDENT_STAMPS " stamps:on"
#else
#define FL_IDENT_STAMPS ""
#endif
// C10 (2026-10-12, test on the release firmware) · WMARK, BENCH ONLY, NEVER SHIPS:
// the quiet release union plus ONE short line per completed MIDI write (0C), "FL WMARK rx <ms> out <n> sum <c>",
// so scripts/wire1-align.py can prove delivery on the build that ships without the per-EXEC trace (~12 ms a
// command). The line costs ~0.25 ms of blocking UART after the bus is free. Says mark:on in the ident;
// tools/fl_release_gate.py refuses the token, the flag and the "FL WMARK" string.
#ifdef FORGE_LINK_WRITE_MARK
#define FL_IDENT_MARK " mark:on"
#else
#define FL_IDENT_MARK ""
#endif
// GATE-DIAG (2026-10-20) · SILENT-X DIAG, BENCH ONLY, NEVER SHIPS: the 84-byte STATUS build (src/ForgeLink_bulk_policy.h)
// says diag:silentx in its ident, so tools/fl_release_gate.py can refuse it from the flashed bytes, not only by env name.
#ifdef FL_SILENTX_DIAG
#define FL_IDENT_DIAG " diag:silentx"
#else
#define FL_IDENT_DIAG ""
#endif
// RX-SLOW (2026-10-22) · BENCH ONLY, NEVER SHIPS: the receive-path event ring (src/ForgeLink_rxstamp.h) says
// rxstamp:on in its ident; tools/fl_release_gate.py refuses the token, the flag and the marker string.
#ifdef FL_RXSTAMP_ON
#define FL_IDENT_RXSTAMP " rxstamp:on"
#else
#define FL_IDENT_RXSTAMP ""
#endif

extern "C" bool forgeLinkSerialPoll(void)
{
#ifdef FL_UNIFIED
    // ⛔ THE UNION'S DISK ROLE, IN CARD-READER MODE: hand the bytes back to the stock key menu,
    // exactly as the injection build does (ForgeLink_inject.cpp, its own forgeLinkSerialPoll).
    // Without this `u` then `y` never reaches the menu and the DIP-OFF flash route dies.
    if (forgeLinkInjectCardModeActive()) return false;
#endif
    // ⏱ LOOP-RATE COUNTER (2026-09-19). The USB link plateaus at ~90-133 KiB/s each way, flat
    // across frame sizes, on a chip whose full-speed USB allows ~1.2 MB/s. The host codec, the
    // flush behaviour and the CDC FIFO SIZE are all excluded by measurement; what is NOT excluded
    // is the SERVICE RATE - throughput = bytes moved per pass x passes per second. Nothing in
    // this firmware counts the passes, so that term has never been measured. One log line every
    // 5 s is far below the noise floor of the link being measured.
#ifdef FL_UNIFIED
    // The union runs this diagnostic block in the BRIDGE role only, where the probe has always run
    // it. The disk role keeps the injection build's plain drain: no 5 s log burst sharing the CDC
    // pipe with block frames, no NAK-ledger arming.
    if (platform_is_initiator_mode_enabled())
#endif
    {
        static uint32_t passes = 0, window_start = 0;
        uint32_t now = millis();
        if (window_start == 0) window_start = now;
        passes++;
        if (g_fl_diag_log && (uint32_t)(now - window_start) >= 5000)
        {
            logmsg("FL LOOP: ", (int)passes, " passes in ", (int)(now - window_start),
                   " ms = ", (int)(passes / ((now - window_start) / 1000)), " per second");
            // ⛔ THE COUNTERS THE BOARD HAS ALWAYS KEPT AND NOBODY HAS EVER READ. A frame the host
            // scores as "lost" either arrived CORRUPTED (crc_errors climbs - a data-integrity bug)
            // or never arrived AT ALL (both flat - a delivery bug). Those are different defects
            // with different fixes, and the board has been counting the difference all along.
            {
                static uint32_t last_crc = 0, last_over = 0;
                logmsg("FL RX: crc_errors ", (int)flFramesCrcErrors(), " (+",
                       (int)(flFramesCrcErrors() - last_crc), ")  oversize ", (int)flFramesOversize(),
                       " (+", (int)(flFramesOversize() - last_over), ")");
                last_crc = flFramesCrcErrors(); last_over = flFramesOversize();
                // V-F1 · a reply the board BUILT but threw away because the CDC port was not open
                // (ForgeLink_frames.cpp fl_write_all). Text in flash: .rodata here is RAM.
                static const char k_fl_rx_rdrop[] __attribute__((section(".flashdata.fl_vf1_rx"))) =
                    "FL TX: replies dropped with the port closed ";
                logmsg(k_fl_rx_rdrop, (int)flFramesReplyDroppedDisconnected());
            }
            fl_nak_report(now - window_start);
            // (The "FL EP OUT/IN arms" re-arm cadence lines were read from counters in the
            // rp2040_usb.c override. Both went in N6 step 1: answered the question.)
            passes = 0; window_start = now;
        }
        if (!g_fl_nak_armed && now > 3000 && Serial) fl_nak_arm();
        fl_nak_poll();
        fl_nak_positive_control();
        if (false) {
        }
    }

    // Every input byte belongs to the bridge in probe builds. The key menu
    // (reboot, UF2, mass storage, debug toggle...) is never reached, so no
    // byte of a binary payload can trigger it (R7).
    //
    // ONE COMMAND IN FLIGHT: while a complete frame waits for the main loop, further bytes
    // stay in the USB FIFO. That is what makes it safe that this runs from platform_poll()
    // inside a SCSI transaction. The 2000 ms gap timer and the oversize/crc split that
    // diagnosed the September frame loss live with the parser, in ForgeLink_frames.cpp.
    flFramesRxDrain();
    return true;
}

/*************************
 * SCSI command executor *
 *************************/

#ifdef PLATFORM_HAS_INITIATOR_MODE

// Modelled on upstream scsiInitiatorRunCommand(), with the Forge Link changes:
//  - a DATA_IN phase that ends early is a normal residual, not an error: the
//    received count is reported and the command continues to STATUS and
//    MESSAGE IN (C2/F2). SMDI RECEIVE depends on this.
//  - DATA_OUT likewise reports how many bytes the target actually took.
//  - IDENTIFY and a prior bus reset are per-command choices from the host.
//  - an explicit outcome code instead of a folded status.
// SCSI-1 single-initiator selection (contact-1 experiment): identical to upstream's
// scsiHostPhySelect (scsiHostPhy.cpp:68-120) except the data bus carries ONLY the
// target's bit, as SCSI-1 allows for a single-initiator bus. Mirrors upstream's
// sequence and its OUT_BSY re-assert (the Blaster reads status lines through U105).
static bool fl_select_scsi1(int target_id)
{
    scsiHostSetBusWidth(0);
    SCSI_OUT(BSY, 1);
    for (int wait = 0; wait < 10; wait++)
    {
        delayMicroseconds(1);
        if (SCSI_IN_DATA() != 0) { SCSI_RELEASE_OUTPUTS(); return false; }
    }
    SCSI_OUT(SEL, 1);
    delayMicroseconds(5);
    SCSI_OUT_DATA((uint8_t)(1 << target_id));
    delayMicroseconds(5);
    SCSI_OUT(BSY, 0);
    for (int wait = 0; wait < 2500; wait++)
    {
        delayMicroseconds(100);
        if (SCSI_IN(BSY)) break;
    }
    if (!SCSI_IN(BSY)) { SCSI_RELEASE_OUTPUTS(); return false; }
    SCSI_RELEASE_DATA_REQ();
    SCSI_OUT(BSY, 1);
    // T4 EDGE TIMING (contact-1, investigation-2): OUT_BSY now gates U105, so the
    // shared pin reads MSG. Sample MSG and REQ with SEL still asserted, then release SEL
    // and time the first MSG and REQ edges in microseconds for 5 ms. MSG high before SEL
    // drops, or <1 us after = stale PCTL (the S1000's firmware never ran); tens of us =
    // its firmware wrote the phase after the Selected interrupt. Read only.
    delayMicroseconds(1);
    int msg_pre = SCSI_IN(MSG), req_pre = SCSI_IN(REQ);
    uint32_t t0 = time_us_32();
    SCSI_OUT(SEL, 0);
    int32_t msg_t = -1, req_t = -1, msg_drop_t = -1;
    int prev_msg = msg_pre;
    uint32_t samples = 0;
    while ((uint32_t)(time_us_32() - t0) < 5000)
    {
        int m = SCSI_IN(MSG), r = SCSI_IN(REQ);
        uint32_t dt = time_us_32() - t0;
        if (m && !prev_msg && msg_t < 0) msg_t = (int32_t)dt;
        if (!m && prev_msg && msg_drop_t < 0) msg_drop_t = (int32_t)dt;
        if (r && req_t < 0) req_t = (int32_t)dt;
        prev_msg = m; samples++;
    }
    logmsg("Forge Link T4: with SEL still on: MSG ", msg_pre, " REQ ", req_pre,
           " | after SEL release: MSG rose at ", (int)msg_t, " us, MSG fell at ", (int)msg_drop_t,
           " us, REQ at ", (int)req_t, " us (-1 = never in 5 ms), ", (int)samples, " samples");
    return true;
}

// ---- BLIND CDB (FL_F_BLIND, W35) ----------------------------------------------------
// WHY: the S1000's ROM, once it services our selection, puts its MB89352 in COMMAND phase
// (file 0x16422: PCTL=2, TC=6, SCMD=0x84) and spins per byte at 0x16512 waiting for our ACK.
// Our board has never read REQ rise - every trace reads the illegal `REQ 0 CD 0 IO 0 MSG 1 BSY 1`,
// which points at our own status-line readback (U105, gated by OUT_BSY; IN_MSG shares GPIO 26 with
// IN_BSY, IN_CD shares GPIO 23 with IN_SEL) rather than at the S1000. So this path completes the
// handshake WITHOUT READING REQ, on fixed timing far slower than any SCSI minimum, and the verdict
// is the eye on the S1000's panel (09 writes the MIDI-via-SCSI setting from CDB byte 2 at ROM
// 0x15ED0). Every REQ/CD/IO/MSG reading logged here is UNVERIFIED for the same reason.
// The data-out mechanics are upstream's own (scsiHostWriteOneByte, scsiHostPhy.cpp): SCSI_OUT_DATA
// puts the byte AND its parity from g_scsi_parity_lookup on the bus, turns the data buffer to
// output, and writes the OUT_REQ bit inactive (REQ is an input in initiator mode, so that write
// has no effect on the bus). Never ATN, never RST.
#define BLIND_SETTLE_US       2000u   // after selection, before the first ACK (default)
#define BLIND_SETTLE_MAX_MS   2000u   // a blind EXEC's timeout_ms 1..this = settle in ms instead
#define BLIND_SETUP_US         200u   // data on the bus -> ACK asserted
#define BLIND_ACK_US           500u   // ACK held asserted
#define BLIND_GAP_US           500u   // ACK released -> next byte
#define BLIND_IN_WAIT_US      2000u   // before each blind ACK of STATUS and MESSAGE IN
#define BLIND_BUSFREE_MS      3000u   // after releasing everything: watch for BSY to drop by itself

static inline uint8_t fl_blind_lines(void)
{
    // bit4 REQ, bit3 CD, bit2 IO, bit1 MSG - all UNVERIFIED readings (U105)
    return (uint8_t)(((int)SCSI_IN(REQ) << 4) | ((int)SCSI_IN(CD) << 3) | ((int)SCSI_IN(IO) << 2)
                     | ((int)SCSI_IN(MSG) << 1));
}

static void fl_blind_wait_us(uint32_t us)
{
    // Long settles (up to 2 s) must not starve the watchdog; short waits stay exact.
    while (us > 1000) { delayMicroseconds(1000); us -= 1000; platform_reset_watchdog(); }
    if (us) delayMicroseconds(us);
}

// Runs right after a successful selection (our OUT_BSY asserted, SEL released, data released).
// Returns FL_O_COMPLETED if BSY dropped by itself within BLIND_BUSFREE_MS, FL_O_BUS_RESET if a
// reset was seen, else FL_O_TIMEOUT. *status_out = the unverified byte read at the STATUS ACK.
static uint8_t fl_blind_cdb(const uint8_t *cdb, uint8_t cdbLen, uint32_t timeout_ms, uint8_t *status_out)
{
    const uint32_t settle_us = (timeout_ms >= 1 && timeout_ms <= BLIND_SETTLE_MAX_MS)
                             ? timeout_ms * 1000u : BLIND_SETTLE_US;
    // Samples are taken during the handshake and LOGGED AFTER IT, so logging cannot stretch the timing.
    struct { uint8_t pre, ack, d_pre, d_ack; } s[12 + 2];
    const uint8_t steps = (uint8_t)(cdbLen + 2);

    uint32_t t_begin = millis();
    logmsg("Forge Link BLIND: BEGIN at t=", (int)t_begin, " ms (board clock), cdb ", bytearray(cdb, cdbLen),
           ", settle ", (int)settle_us, " us, setup ", (int)BLIND_SETUP_US, " / ACK ", (int)BLIND_ACK_US,
           " / gap ", (int)BLIND_GAP_US, " us, in-wait ", (int)BLIND_IN_WAIT_US, " us; REQ is NEVER read to decide"
           " anything - REQ/CD/IO/MSG below are unverified (U105 gated by OUT_BSY)");

    fl_blind_wait_us(settle_us);

    // a+b: the CDB, one byte per blind ACK pulse. The data stays driven across the whole CDB.
    for (uint8_t i = 0; i < cdbLen; i++)
    {
        SCSI_OUT_DATA(cdb[i]);
        delayMicroseconds(BLIND_SETUP_US);
        s[i].pre = fl_blind_lines();
        s[i].d_pre = (uint8_t)(SCSI_IN_DATA() & 0xFF);
        SCSI_OUT(ACK, 1);
        delayMicroseconds(BLIND_ACK_US);
        s[i].ack = fl_blind_lines();
        s[i].d_ack = (uint8_t)(SCSI_IN_DATA() & 0xFF);
        SCSI_OUT(ACK, 0);
        // c: the moment the last byte's ACK is gone, stop driving data - the target drives STATUS next.
        if (i == cdbLen - 1) SCSI_RELEASE_DATA_REQ();
        delayMicroseconds(BLIND_GAP_US);
    }
    uint32_t t_cdb = millis();

    // d: two blind ACKs to accept STATUS and MESSAGE IN (target drives the data bus; we only read it).
    for (uint8_t k = 0; k < 2; k++)
    {
        uint8_t i = (uint8_t)(cdbLen + k);
        fl_blind_wait_us(BLIND_IN_WAIT_US);
        s[i].pre = fl_blind_lines();
        s[i].d_pre = (uint8_t)(SCSI_IN_DATA() & 0xFF);
        SCSI_OUT(ACK, 1);
        delayMicroseconds(BLIND_ACK_US);
        s[i].ack = fl_blind_lines();
        s[i].d_ack = (uint8_t)(SCSI_IN_DATA() & 0xFF);
        SCSI_OUT(ACK, 0);
        delayMicroseconds(BLIND_GAP_US);
    }
    *status_out = s[cdbLen].d_pre;

    // e: let go of everything and WATCH: a target that completed the command goes BUS FREE itself.
    // With OUT_BSY released, GPIO 26 reads BSY (upstream's own online test, scsiHostPhyGetPhase).
    SCSI_RELEASE_OUTPUTS();
    uint32_t t_rel = millis();
    int32_t bsy_drop_ms = -1;
    bool reset_seen = false;
    delayMicroseconds(2);
    while ((uint32_t)(millis() - t_rel) < BLIND_BUSFREE_MS)
    {
        platform_reset_watchdog();
        platform_poll();
        if (g_scsiHostPhyReset) { reset_seen = true; break; }
        if (!SCSI_IN(BSY)) { bsy_drop_ms = (int32_t)(millis() - t_rel); break; }
    }

    for (uint8_t i = 0; i < steps; i++)
    {
        const char *what = (i < cdbLen) ? "CDB byte" : (i == cdbLen ? "STATUS ack" : "MSG-IN ack");
        logmsg("Forge Link BLIND: ", what, " ", (int)(i < cdbLen ? i : i - cdbLen),
               (i < cdbLen) ? " sent " : " read ", (i < cdbLen) ? cdb[i] : s[i].d_pre,
               " | before ACK: REQ ", (s[i].pre >> 4) & 1, " CD ", (s[i].pre >> 3) & 1, " IO ", (s[i].pre >> 2) & 1,
               " MSG ", (s[i].pre >> 1) & 1, " data ", s[i].d_pre,
               " | during ACK: REQ ", (s[i].ack >> 4) & 1, " CD ", (s[i].ack >> 3) & 1, " IO ", (s[i].ack >> 2) & 1,
               " MSG ", (s[i].ack >> 1) & 1, " data ", s[i].d_ack, " (unverified)");
    }
    uint8_t outcome = reset_seen ? FL_O_BUS_RESET : (bsy_drop_ms >= 0 ? FL_O_COMPLETED : FL_O_TIMEOUT);
    uint32_t t_end = millis();
    logmsg("Forge Link BLIND: END at t=", (int)t_end, " ms (board clock); CDB clocked by t=", (int)t_cdb,
           ", outputs released at t=", (int)t_rel, "; status byte (unverified) ", *status_out,
           ", message byte (unverified) ", s[cdbLen + 1].d_pre, "; ",
           reset_seen ? "bus reset seen during the BSY watch"
                      : (bsy_drop_ms >= 0 ? "BSY DROPPED BY ITSELF" : "BSY still held after the watch"),
           " at +", (int)bsy_drop_ms, " ms after release (-1 = not within ", (int)BLIND_BUSFREE_MS, " ms)");
    return outcome;
}

static uint8_t fl_execute_on_bus(int target, uint8_t flags, uint32_t timeout_ms,
                                 const uint8_t *cdb, uint8_t cdbLen,
                                 uint8_t *in, uint32_t inAlloc, uint32_t *received,
                                 const uint8_t *out, uint32_t outLen, uint32_t *sent,
                                 uint8_t *status_out);

// ⛔ THE HOT PATH IS EVERY BULK DATA EXEC, NOT ONLY A CHAINED FILL (2026-09-29, "yes if possible").
// A fetch READ (0E, 32 KiB DATA_IN) is unchained, so it still ran with every TRACE and DBG line and the
// debug log's 32 KiB checksum: ~16 ms of a ~150 ms chunk in the sitting-4 board log. So a chained EXEC
// OR one moving >= FL_QUIET_BYTES either way runs quiet; control commands (INQUIRY, polls, SysEx
// sends) stay small and keep the full trace. -DFORGE_LINK_EXEC_TRACE still turns everything back on.
#define FL_QUIET_BYTES 4096u
static inline bool fl_exec_quiet(uint8_t flags, uint32_t inAlloc, uint32_t outLen)
{
    if (g_fl_exec_trace) return false;
    if (g_fl_release_quiet_exec) return true;   // QX1: every EXEC on the ship build
    return (flags & FL_F_CHAINED) != 0 || inAlloc >= FL_QUIET_BYTES || outLen >= FL_QUIET_BYTES;
}

// ⛔ EVERY EXEC RUNS ITS BUS WORK WITHOUT THE TRACE, NOT ONLY A BULK ONE (2026-10-06, L1,
//). MEASURED on #2 (425dee0, XL, DEBUG DIP ON, CDC link): the same
// read-only INQUIRY(36), 40 unchained (DBG + TRACE on) interleaved with 40 chained (quiet), Mac-clock
// round trip median 15.62 ms -> 3.62 ms (p95 20.46 -> 6.39): the trace was ~77% of a small command,
// 12.0 ms each, and it capped the link near 70 commands/s. A buyer's DEBUG DIP may be on, so the DIP
// no longer decides this: the debug log (g_log_debug) and the per-phase TRACE lines are off for the
// EXEC's bus work whatever the DIP says. ⭐ What stays: fl_exec_quiet still gates the two non-debug
// "EXEC id" / "EXEC done" lines (small EXECs keep them - hosts and bench tools key on them), a failed
// EXEC ~~still says where with its "quiet EXEC failed" line, and every non-EXEC line is untouched~~
// that fails in its phase loop says where with its "quiet EXEC failed" line (a SELECTION failure and a
// BLIND EXEC return before it; their EXEC done line still carries the outcome), and the global debug
// flag is clear for the whole EXEC, so a DBG line from anything else that runs meanwhile is lost too.
// ⚠ THE 3.62 ms IS NOT L1's NUMBER (review 2026-10-06): the chained arm also skipped the EXEC id /
// done lines (229 B ≈ 2.3 ms of UART), which L1 keeps, and the DIP's per-pass SD log save
// (ZuluSCSI.cpp:114/2106) still runs after the reply. ESTIMATE, unmeasured: ~5-6 ms per small EXEC on L1,
// ~10 ms saved. Measure it on metal before quoting a speed. Removing the trace also takes its incidental
// delays out of the phase loop (REQ-to-ACK, the next selection); both Akais have run quiet chained traffic
// since (2026-09-29), but small commands quiet are new to them (review challenge, 2026-10-06).
// -DFORGE_LINK_EXEC_TRACE turns the whole trace back on (and the DIP's debug log with it).
// ⭐ QX1 (2026-10-10): on the SHIP build only (FL_RELEASE_QUIET_EXEC, ForgeLink_probe.h) small EXECs
// drop their EXEC id / done lines too and the DIP's per-pass SD log save goes; failures still log. ESTIMATE,
// unmeasured: ~2-3 ms more per small EXEC (docs/bench/qx1-notes.md). Every bench build keeps the lines.
static inline bool fl_exec_line_trace(void)
{
    return g_fl_exec_trace;
}

// ⛔ A CHAINED FILL RUNS WITH THE DEBUG LOG OFF (2026-09-29, the pre-flash check on the trace
// gate). Blaster #2's DEBUG DIP is ON ("DIP switch settings: debug log 1"; the firmware's own banner
// says it "will severely impact" speed), and the sitting-rear-3 push log is 1,304 DBG lines of
// 2,399: BUS_BUSY, SELECTION, SELECTING, COMMAND, OUT bytes, phase names, checksums - every one a
// blocking UART line inside the EXEC, several while the S1000 holds REQ. Same move as
// forgeLinkServeStream (ForgeLink_callback.cpp): save, clear for the hot phase, restore.
// ~~Unchained EXECs (the first fill, every control command) keep the full debug log.~~ Since L1
// (2026-10-06) EVERY EXEC clears it: fl_exec_line_trace above, -DFORGE_LINK_EXEC_TRACE to keep it.
static uint8_t fl_execute(int target, uint8_t flags, uint32_t timeout_ms,
                          const uint8_t *cdb, uint8_t cdbLen,
                          uint8_t *in, uint32_t inAlloc, uint32_t *received,
                          const uint8_t *out, uint32_t outLen, uint32_t *sent,
                          uint8_t *status_out)
{
    const bool quiet = !fl_exec_line_trace();
    const bool dbg = g_log_debug;
    if (quiet) g_log_debug = false;
    uint8_t outcome = fl_execute_on_bus(target, flags, timeout_ms, cdb, cdbLen, in, inAlloc, received,
                                        out, outLen, sent, status_out);
    g_log_debug = dbg;
    return outcome;
}

static uint8_t fl_execute_on_bus(int target, uint8_t flags, uint32_t timeout_ms,
                                 const uint8_t *cdb, uint8_t cdbLen,
                                 uint8_t *in, uint32_t inAlloc, uint32_t *received,
                                 const uint8_t *out, uint32_t outLen, uint32_t *sent,
                                 uint8_t *status_out)
{
    *received = 0;
    *sent = 0;
    *status_out = 0xFF;

    if (flags & FL_F_BUS_RESET)
    {
        logmsg("Forge Link: bus reset before command");
        scsiHostPhyReset();
        fl_forget_types();
    }

    const bool send_identify = (flags & FL_F_IDENTIFY) != 0;
    bool identify_sent = false;
    bool got_status = false;
    bool complete = false;
    uint8_t outcome = FL_O_PHASE_ERROR;

    scsiHostSetBusWidth(0);
    // ⛔ NOT ON A CHAINED FILL (2026-09-29, the last ~6 ms per chunk). These
    // TRACE lines went out on EVERY EXEC - two fixed lines plus one per phase/REQ change - on the
    // same blocking 1 Mbaud UART that cost ~1.1 ms per line for the EXEC done line (the slice-2
    // review gated THAT one, not these). The pipelined push measured 94.5 ms per 32 KiB chunk
    // against the ~88 ms bus ceiling. Since (2026-09-29) every bulk data EXEC is quiet (chained,
    // or >= 4 KiB either way, fl_exec_quiet); ~~small control commands keep the full trace~~ since L1
    // (2026-10-06,: the trace measured ~12 ms of a 15.62 ms small command) EVERY EXEC runs without them
    // (fl_exec_line_trace); a failed EXEC still logs its EXEC done line and its "quiet EXEC failed"
    // line, and -DFORGE_LINK_EXEC_TRACE turns them all back on.
    const bool line_trace = fl_exec_line_trace();
    // Baseline: the handshake lines BEFORE we touch the bus (read only).
    if (line_trace) {
        logmsg("Forge Link TRACE: idle lines REQ ", (int)SCSI_IN(REQ), " CD ", (int)SCSI_IN(CD),
               " IO ", (int)SCSI_IN(IO), " MSG ", (int)SCSI_IN(MSG), " BSY ", (int)SCSI_IN(BSY),
               " SEL ", (int)SCSI_IN(SEL), " ATN ", (int)SCSI_IN(ATN), " data ", (uint8_t)(SCSI_IN_DATA() & 0xFF));
    }
    if (send_identify) scsiHostPhySetATN(true);

    FL_STAMP(FL_ST_ARB);   // DSP2 t_arb: arbitration + selection start inside the select call
    bool selected = (flags & FL_F_NO_INIT_ID)
        ? fl_select_scsi1(target)
        : scsiHostPhySelect(target, (uint8_t)scsiInitiatorGetOwnID());
    if (selected) FL_STAMP(FL_ST_SEL);   // DSP2 t_sel: the target answered (BSY)
    if (!selected)
    {
        scsiHostPhySetATN(false);
        scsiHostPhyRelease();
        return FL_O_NO_SELECT;
    }

    // RELEASE-BSY KNOCK (W34's caveat; history: the contact-1 experiment of 2026-09-18,
    // concluded C2, ran this same release for a fixed 2 s behind `if (0)`).
    // Upstream's select re-asserts the Blaster's OUT_BSY after the target answers (it gates
    // buffer U105 for reading the status lines), so during a held knock BOTH devices drive BSY.
    // A textbook initiator leaves BSY to the target. With FL_F_RELEASE_BSY the host asks us to
    // release ours for the EXEC's own timeout_ms and log every change on the lines (read only,
    // capped); stop early if REQ appears. REQ seen -> re-assert and carry on into the normal
    // phase loop. Never seen -> TIMEOUT, straight to the bus-free cleanup below.
    // ⚠ With OUT_BSY off, U105 is not enabled, so the status-line readings are UNVERIFIED -
    // and on the Blaster IN_BSY/IN_MSG share GPIO 26 and IN_SEL/IN_CD share GPIO 23, so what
    // those pins show depends on U105. The log says so on every line.
    // Without the flag this whole block is skipped and the path is exactly as before.
    bool release_gave_up = false;
    // BLIND CDB: no phase loop at all. The combination with IDENTIFY/RELEASE_BSY/BUS_RESET/data
    // was refused in fl_handle_exec before the bus, so here ATN is off and our BSY is asserted.
    if (flags & FL_F_BLIND)
    {
        outcome = fl_blind_cdb(cdb, cdbLen, timeout_ms, status_out);
        scsiHostPhySetATN(false);
        scsiHostWaitBusFree();   // the library's 10 s wait + reset remains the safety net
        return outcome;
    }
    if (flags & FL_F_RELEASE_BSY)
    {
        SCSI_OUT(BSY, 0);
        uint32_t t0 = millis();
        logmsg("Forge Link RELBSY: BEGIN own BSY released at t=", (int)t0, " ms (board clock), for up to ",
               (int)timeout_ms, " ms; REQ/CD/IO/MSG readings below are unverified (U105 gated by OUT_BSY);"
               " GPIO 26 = BSY-or-MSG, GPIO 23 = SEL-or-CD");
        int last = -1, n = 0;
        bool req_seen = false, reset_seen = false;
        while ((uint32_t)(millis() - t0) < timeout_ms)
        {
            platform_reset_watchdog();
            platform_poll();
            if (g_scsiHostPhyReset) { reset_seen = true; break; }
            int now = ((int)SCSI_IN(REQ) << 4) | ((int)SCSI_IN(CD) << 3) | ((int)SCSI_IN(IO) << 2)
                    | ((int)SCSI_IN(MSG) << 1) | (int)SCSI_IN(BSY);
            if (now != last && n < 30)
            {
                logmsg("Forge Link RELBSY: +", (int)(millis() - t0), " ms REQ ", (now >> 4) & 1,
                       " CD ", (now >> 3) & 1, " IO ", (now >> 2) & 1, " MSG ", (now >> 1) & 1, " BSY ", now & 1,
                       " (REQ/CD/IO/MSG unverified (U105 gated by OUT_BSY))");
                last = now; n++;
            }
            if (now & 0x10) { req_seen = true; break; }
        }
        uint32_t t1 = millis();
        logmsg("Forge Link RELBSY: END at t=", (int)t1, " ms (board clock), held ", (int)(t1 - t0),
               " ms, ", n, " line changes logged (cap 30), ",
               reset_seen ? "bus reset seen" : (req_seen ? "REQ rose (unverified reading) - re-asserting BSY, carrying on"
                                                         : "REQ never rose - giving up, TIMEOUT"));
        if (reset_seen)
        {
            outcome = FL_O_BUS_RESET;
            release_gave_up = true;
        }
        else if (req_seen)
        {
            SCSI_OUT(BSY, 1);
        }
        else
        {
            outcome = FL_O_TIMEOUT;
            release_gave_up = true;
        }
    }

    uint32_t start = millis();
    SCSI_PHASE phase;
    // LINE TRACE (contact-1): log every change in what the target is asking for,
    // with the raw handshake lines, read only. Capped so a spinning loop cannot
    // flood the USB log.
    int trace_last = -1, trace_n = line_trace ? 0 : 40;
    if (line_trace) {
        logmsg("Forge Link TRACE: selected id ", target, "; lines REQ ", (int)SCSI_IN(REQ),
               " CD ", (int)SCSI_IN(CD), " IO ", (int)SCSI_IN(IO), " MSG ", (int)SCSI_IN(MSG),
               " BSY ", (int)SCSI_IN(BSY), " ATN ", (int)SCSI_IN(ATN));
    }
    while (!release_gave_up && (phase = (SCSI_PHASE)scsiHostPhyGetPhase()) != BUS_FREE)
    {
        platform_reset_watchdog();
        platform_poll();

        int trace_now = ((int)phase << 1) | (int)SCSI_IN(REQ);
        if (trace_now != trace_last)
        {
            if (trace_n < 40)
            {
                logmsg("Forge Link TRACE: +", (int)(millis() - start), " ms phase ", (int)phase,
                       " (CD ", (int)SCSI_IN(CD), " IO ", (int)SCSI_IN(IO), " MSG ", (int)SCSI_IN(MSG),
                       ") REQ ", (int)SCSI_IN(REQ), " BSY ", (int)SCSI_IN(BSY));
                trace_n++;
            }
            trace_last = trace_now;      // tracked even when quiet: the failure line names it
        }

        if (g_scsiHostPhyReset) { outcome = FL_O_BUS_RESET; break; }
        if ((uint32_t)(millis() - start) > timeout_ms) { outcome = FL_O_TIMEOUT; break; }

        if (phase == MESSAGE_IN)
        {
            uint8_t msg = 0xFF;
            scsiHostRead(&msg, 1);
            FL_STAMP_HS(1);
            if (msg == MSG_COMMAND_COMPLETE)
            {
                complete = true;
                break;
            }
            // Any other message: logged, and the loop carries on.
            logmsg("Forge Link: MESSAGE IN ", msg);
        }
        else if (phase == MESSAGE_OUT)
        {
            scsiHostPhySetATN(false);
            uint8_t msg = identify_sent ? FL_MSG_NO_OPERATION : FL_MSG_IDENTIFY_LUN0_NODISC;
            identify_sent = true;
            scsiHostWrite(&msg, 1);
            FL_STAMP_HS(1);
        }
        else if (phase == COMMAND)
        {
            if (send_identify && !identify_sent)
            {
                // Target went straight to COMMAND; it does not take messages.
                scsiHostPhySetATN(false);
            }
            scsiHostWrite(cdb, cdbLen);
            FL_STAMP_HS(cdbLen);
            FL_STAMP(FL_ST_CMD);   // DSP2 t_cmd_done
        }
        else if (phase == DATA_IN)
        {
            uint32_t remaining = inAlloc - *received;
            if (remaining == 0)
            {
                // Target has more to send than the host allocated. Bail; the
                // bus-free wait below drains the rest as dummy bytes.
                outcome = FL_O_DATA_OVERFLOW;
                break;
            }
            uint32_t n = scsiHostRead(in + *received, remaining);
            FL_STAMP(FL_ST_DATA);   // DSP2 t_data_done (the last data transfer wins)
            FL_STAMP_HS(n);
            *received += n;
            if (n == 0)
            {
                // Zero bytes while the phase said DATA_IN: reset, parity error,
                // or a 10 s REQ timeout inside the read. Not a residual.
                outcome = g_scsiHostPhyReset ? FL_O_BUS_RESET : FL_O_PHASE_ERROR;
                break;
            }
            // n < remaining is a short read: fine, the next phase is read next.
        }
        else if (phase == DATA_OUT)
        {
            uint32_t remaining = outLen - *sent;
            if (remaining == 0)
            {
                // Target wants more than the host supplied.
                outcome = FL_O_PHASE_ERROR;
                break;
            }
            uint32_t n = scsiHostWrite(out + *sent, remaining);
            FL_STAMP(FL_ST_DATA);   // DSP2 t_data_done (the last data transfer wins)
            FL_STAMP_HS(n);
            *sent += n;
            if (n == 0)
            {
                outcome = g_scsiHostPhyReset ? FL_O_BUS_RESET : FL_O_PHASE_ERROR;
                break;
            }
        }
        else if (phase == STATUS)
        {
            uint8_t st = 0xFF;
            scsiHostRead(&st, 1);
            FL_STAMP_HS(1);
            FL_STAMP(FL_ST_STATUS);   // DSP2 t_status
            *status_out = st;
            got_status = true;
        }
        // BUS_BUSY: target has not asserted REQ yet; keep polling.
    }

    scsiHostPhySetATN(false);
    scsiHostWaitBusFree();
    FL_STAMP(FL_ST_FREE);   // DSP2 t_free

    if (complete && got_status) outcome = FL_O_COMPLETED;
    // A quiet EXEC (since L1 every EXEC, unless -DFORGE_LINK_EXEC_TRACE) ran without its line trace;
    // if it failed, say where.
    if (!line_trace && outcome != FL_O_COMPLETED)
        logmsg("Forge Link: quiet EXEC failed at +", (int)(millis() - start), " ms, last phase/REQ ",
               trace_last, " (phase<<1|REQ), outcome ", (int)outcome);
    return outcome;
}

#endif // PLATFORM_HAS_INITIATOR_MODE

/*************************
 * Frame handlers        *
 *************************/

// ⭐ N6 PROBE-BULK SLICE 2 · THE CHAIN LATCH (frame item 11) and the bench's two counters. All of
// it is main-loop state: set and read only by the dispatcher, never from platform_poll.
static bool     g_fl_chain_broken = false;   // the last EXEC failed; a CHAINED one is refused
static uint32_t g_fl_no_select = 0;          // EXECs whose target did not answer selection
static uint32_t g_fl_chain_refused = 0;      // CHAINED EXECs refused because the chain was broken
// ⚠ UNMEASURED, AND THIS IS HOW IT GETS MEASURED: the S1000's inter-select gap at window 2. With
// two EXECs in flight the second is selected as soon as the first reaches bus free, and whether
// the S1000 answers a selection that soon is unknown. Every EXEC's log line says how long the
// bus had been free since the previous one and the running NO_SELECT total, so a bench run at
// window 2 counts them straight off the capture. (A board-side minimum gap comes if they appear.)
static uint32_t g_fl_last_exec_end_ms = 0;
// The lowest free heap the main loop has seen this boot - the probe's port of the injection
// build's low-water sampler (ForgeLink_hwdog_cold.cc, fix 3), so the bench measures the probe's
// real runtime heap need rather than only its static free heap. Sampled in forgeLinkProbeMainLoop.
static uint32_t g_fl_heap_low_water = 0xFFFFFFFFu;

// The bench's counters, on the session boundaries (HELLO and EXIT) rather than on every EXEC.
static void fl_log_counters(const char *when)
{
    logmsg("Forge Link: ", when, " counters: no_select total ", (int)g_fl_no_select,
           ", chain_broken refusals ", (int)g_fl_chain_refused, ", heap low-water ",
           (int)(g_fl_heap_low_water == 0xFFFFFFFFu ? 0 : g_fl_heap_low_water),
           " B, free now ", (int)rp2040.getFreeHeap(), " B");
}

static void fl_handle_hello(uint8_t seq, uint32_t req_crc, uint32_t req_len)
{
    // A HELLO starts a session: it clears the replay cache, the type cache and the chain latch.
    flFramesInvalidateCache();
    fl_forget_types();
    g_fl_chain_broken = false;
#ifdef FORGE_LINK_NOTE_AT
    forgeLinkNoteAtHello();   // TQ2 §10: a new host session flushes the timed queue and disarms
#endif

#ifdef FL_UNIFIED
    uint8_t buf[8 + 128 + 2 + 4 + 4];   // + {features, rx_slots} (N6 probe-bulk) + watchdog x4 + session (r7)
#else
    uint8_t buf[8 + 128 + 2 + 4];   // + {features, rx_slots}, appended after the ident (N6 probe-bulk) + session (r7)
#endif
    uint32_t n = 0;
    buf[n++] = FORGE_LINK_PROTOCOL_VERSION;
    bool initiator = false;
#ifdef PLATFORM_HAS_INITIATOR_MODE
    initiator = platform_is_initiator_mode_enabled();
#endif
    buf[n++] = initiator ? (uint8_t)scsiInitiatorGetOwnID() : 0xFF;
    // roles: bit0 initiator bridge live, bit1 target (disk) role, bit2 role forced by build (DIP
    // ignored) - bit2 is NEVER set since 2026-09-24: the role follows the DIP again.
    buf[n++] = (initiator ? 0x01 : 0x02);
    fl_put32(&buf[n], FORGE_LINK_MAX_DATA); n += 4;
    // ⭐ L4 (2026-10-06) · THREE TOKENS APPENDED AFTER git:, NEVER BEFORE IT. Every parser keys on the
    // "Forge Link" prefix and reads git: up to the next space (forge_link_proto.parse_hello,
    // fl_rear.hello_git, the app's BridgeHello), so old hosts read exactly what they read before.
    // caps:<a,b> what this bridge can promise. safe-reset = 's C2a: a BULK RESET drops the
    //               queued, unstarted frames and suppresses the stale reply ("BULK RESET dropped",
    //               ForgeLink_frames.cpp fl_reply_stale / flFramesNextHead). It lives under
    //               FORGE_LINK_BULK, so the token does too (block-interface §9 TIMEOUTS).
    // trace:on|off FL_IDENT_TRACE above, the per-EXEC TRACE log.
    //   build:<N>   `git rev-list --count HEAD` at build time (inject_build_info.py): monotonic, so a
    //               host can say "or later", which a hash cannot.
    static const char ident[] = FORGE_LINK_NAME " " ZULU_FW_VERSION " git:" ZULU_BUILD_GIT_REV
#ifdef FORGE_LINK_BULK
        " caps:safe-reset" FL_IDENT_NOTE_AT FL_IDENT_DATA_CRC
#endif
        FL_IDENT_TRACE " build:" ZULU_BUILD_NUMBER FL_IDENT_STAMPS FL_IDENT_MARK FL_IDENT_DIAG FL_IDENT_RXSTAMP;
    static_assert(sizeof(ident) - 1 <= 127, "the HELLO ident is length-prefixed to 127 bytes");
    uint32_t sl = strlen(ident);
    if (sl > 127) sl = 127;
    buf[n++] = (uint8_t)sl;
    memcpy(&buf[n], ident, sl); n += sl;
    // ⭐ N6 PROBE-BULK · TWO BYTES AFTER THE IDENT, ADDITIVE (frame item 4). The ident is length-
    // prefixed, so a host that reads only up to its end never sees these, and a host that knows
    // them reads them only when they are present. The feature bits are the SAME namespace as the
    // injection HELLO's (ForgeLink_probe.h: FL_FEAT_QUEUE 0x02, FL_FEAT_BULK 0x08), so one Mac
    // rule reads both: BULK = the vendor-bulk interface is compiled in and claimable; QUEUE = the
    // parser holds rx_slots frames in order, set only when there is more than one.
    uint8_t features = 0;
#ifdef FORGE_LINK_BULK
    features |= FL_FEAT_BULK;
#endif
    if (FL_RX_SLOTS > 1) features |= FL_FEAT_QUEUE;
#ifdef FL_UNIFIED
    features |= FL_FEAT_HWDOG;
#endif
#ifdef FORGE_LINK_BULK
    features |= FL_FEAT_DATA_CRC;   // ARMRETRY r7 · DATACRC: the session follows at the very end
#endif
    buf[n++] = features;
    buf[n++] = (uint8_t)FL_RX_SLOTS;
#ifdef FL_UNIFIED
    // ⭐ THE WATCHDOG BYTES, APPENDED AT THE VERY END, AFTER rx_slots. The same four bytes,
    // same meaning, as the injection HELLO's bytes 9..12 (ForgeLink_hwdog.h: flags, consecutive
    // watchdog boots, K, timeout s), flagged by the same FL_FEAT_HWDOG bit in the same features
    // namespace. A Mac parser that reads the ident and the two-byte tail and stops (every probe
    // parser before) never sees them; one that knows them reads them only when the bit is set.
    forgeLinkHwdogHelloBytes(&buf[n]); n += 4;
#endif
#ifdef FORGE_LINK_BULK
    // ⭐ ARMRETRY r7 · DATACRC · THE SESSION, u32 LE, APPENDED AT THE VERY END: after rx_slots, and
    // after the four watchdog bytes when FL_FEAT_HWDOG is set, flagged by FL_FEAT_DATA_CRC (and the ident's
    // caps:data-crc). Every parser before r7 stops earlier and never sees it.
    fl_put32(&buf[n], forgeLinkBulkSession()); n += 4;
#endif
    flFramesSendReply(FL_T_HELLO, seq, req_crc, req_len, buf, n, nullptr, 0);
    logmsg("Forge Link: HELLO from host, bridge ", initiator ? "live" : "unavailable (target role)");
    fl_log_counters("HELLO");
#ifdef FL_STAMPS_ON
    forgeLinkStampsHelloLine();   // DSP2: says on every HELLO that this is the bench-only stamps build
#endif
#ifdef FL_RXSTAMP_ON
    forgeLinkRxStampHelloLine();  // RX-SLOW: says on every HELLO that this is the bench-only rxstamp build
#endif
}

static void fl_handle_exec(uint8_t seq, uint32_t req_crc, uint32_t req_len, const uint8_t *p, uint32_t len)
{
#ifdef FORGE_LINK_NOTE_AT
    uint8_t rep[FL_REPLY_FIXED_LEN + 8];   // + u64 retry_at_us on a HELD_FOR_TIMED (TQ2, note-at §8)
#else
    uint8_t rep[FL_REPLY_FIXED_LEN];
#endif
    memset(rep, 0, sizeof(rep));
    rep[1] = 0xFF;  // status: none
    uint32_t received = 0, sent = 0;
    // What the chain latch judges the outcome against; zero until the parse has read them.
    uint32_t inAlloc = 0, outLen = 0;
    // The cached reply buffer is about to be reused for this command's data.
    flFramesInvalidateCache();
    // Where DATA_IN lands. Decided ONCE, before the bus, and never changed after it.
    uint8_t *land = flFramesReplyDataBuffer();
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    bool from_slot = false;
#endif

#ifdef FL_STAMPS_ON
    // DSP2: this frame is an EXEC, so the main loop commits its record after the dispatch. A refused
    // or malformed one is recorded too (its outcome says so; its bus slots are absent).
    g_fl_st.open = 1;
    if (len >= 2) g_fl_st.flags = p[1];
    if (len >= 8) g_fl_st.cdb0 = p[7];
#endif
    // Parse. Any malformation is answered without touching the bus.
    uint8_t outcome = FL_O_BAD_REQUEST;
    const char *why = "malformed EXEC";
    do
    {
        if (len < FL_EXEC_FIXED_LEN) break;
        uint8_t target = p[0];
        uint8_t flags = p[1];
        uint32_t timeout_ms = fl_get32(&p[2]);
        uint8_t cdbLen = p[6];
        if (cdbLen != 6 && cdbLen != 10 && cdbLen != 12) break;
        if (len < FL_EXEC_FIXED_LEN + cdbLen) break;
        const uint8_t *cdb = &p[7];
        inAlloc = fl_get32(&p[7 + cdbLen]);
        outLen = fl_get32(&p[11 + cdbLen]);
        const uint8_t *out = &p[15 + cdbLen];
        if (len != FL_EXEC_FIXED_LEN + cdbLen + outLen) break;
#ifdef FL_STAMPS_ON
        if (outLen > 0) g_fl_st.out0 = out[0];   // DSP2: 9x = a note on, so the host can match records to notes
#endif
        if (inAlloc > FORGE_LINK_MAX_DATA || outLen > FORGE_LINK_MAX_DATA) { why = "data length over maximum"; break; }
        if (((flags & FL_F_DATA_OUT) != 0) != (outLen > 0)) { why = "data-out flag disagrees with length"; break; }
        if (((flags & FL_F_DATA_IN) != 0) != (inAlloc > 0)) { why = "data-in flag disagrees with allocation"; break; }
        if (timeout_ms == 0 || timeout_ms > 60000) timeout_ms = 30000;
        // BLIND runs without reading REQ, so it can carry no message, no data phase, and no
        // RST or BSY release of ours: refuse every such combination here, before the bus.
        if ((flags & FL_F_BLIND) && (flags & FL_F_IDENTIFY)) { why = "blind with IDENTIFY"; break; }
        if ((flags & FL_F_BLIND) && (flags & (FL_F_RELEASE_BSY | FL_F_BUS_RESET | FL_F_DATA_OUT | FL_F_DATA_IN)))
        { why = "blind with RELEASE_BSY, BUS_RESET or a data phase"; break; }
        // ⛔ THE CHAIN LATCH, BEFORE ANYTHING THAT COULD TOUCH THE BUS. A CHAINED EXEC behind a
        // failed one is the next chunk of a fill whose previous chunk did not land; sending it
        // would pour it into the S1000's armed residual one chunk early.
        if ((flags & FL_F_CHAINED) && g_fl_chain_broken)
        {
            outcome = FL_O_CHAIN_BROKEN;
            why = "chained behind a failed EXEC";
            g_fl_chain_refused++;
            break;
        }

#ifdef PLATFORM_HAS_INITIATOR_MODE
        if (!platform_is_initiator_mode_enabled())
        {
            outcome = FL_O_NOT_INITIATOR;
            why = "INITIATOR DIP is off";
            break;
        }
        if (target == scsiInitiatorGetOwnID())
        {
            outcome = FL_O_REJECTED;
            why = "target is the bridge's own ID";
            break;
        }
        if (!forgeLinkCdbAllowed(target, cdb, cdbLen, &why))
        {
            outcome = FL_O_REJECTED;
            break;
        }
#ifdef FORGE_LINK_NOTE_AT
        // TQ2 · ADMISSION (note-at §8): armed with notes queued, an EXEC that would cross a due is not
        // started. Answered at once, nothing on the bus, no log line (a due is near). A 09 OFF flushes
        // the queue here first (§10) and is never held.
        if (forgeLinkNoteAtExecHold(p, len, &rep[FL_REPLY_FIXED_LEN]))
        {
            outcome = FL_O_HELD_FOR_TIMED;
            why = nullptr;
            break;
        }
#endif

        // ⛔ A bulk data EXEC is the hot path: its CDB line would cost ~1 ms of blocking UART per EXEC,
        // the same as the done line below. Since even the first (unchained) 32 KiB fill and every
        // fetch READ are quiet, so a bulk run leaves NO CDB line here - the Mac capture carries every
        // CDB. Small control commands still log theirs.
        const bool quiet = fl_exec_quiet(flags, inAlloc, outLen);
        if (!quiet)
            logmsg("Forge Link: EXEC id ", (int)target, " cdb ", bytearray(cdb, cdbLen),
                   " in ", (int)inAlloc, " out ", (int)outLen);
        // ⛔ DATA_IN LANDS IN THE REPLY BUFFER, and on the vendor-bulk link the previous reply may
        // still be leaving from those same bytes (N6 probe-bulk, frame item 8). Wait for the IN
        // endpoint to go idle first - thread mode, bounded, watchdog fed; a no-op without bulk.
        //
        // ⭐ FETCH FROM THE SLOT. On the probe+bulk build a
        // bulk DATA_IN EXEC (>= 4 KiB, no data-out) lands in its OWN rx slot instead, and so skips
        // that wait: READ k+1 selects while READ k's reply is still leaving. ⛔ THE CDB IS COPIED
        // FIRST - it is body[7..7+cdbLen) and the data lands from body[18], so a 12-byte CDB is
        // overwritten by the first bytes the target sends.
        uint8_t cdb_local[12];
        memcpy(cdb_local, cdb, cdbLen);
        cdb = cdb_local;
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
        if (inAlloc >= FL_QUIET_BYTES && outLen == 0)
        {
            uint8_t *slot = flFramesHeadReplyArea();
            if (slot) { land = slot; from_slot = true; }
        }
        if (!from_slot)
#endif
        if (inAlloc > 0) flFramesWaitBulkTxIdle();
        uint8_t status = 0xFF;
        const uint32_t gap_ms = millis() - g_fl_last_exec_end_ms;
#ifdef FORGE_LINK_WRITE_MARK
        const uint32_t rx_ms = millis();
#endif
        outcome = fl_execute(target, flags, timeout_ms, cdb, cdbLen,
                             land, inAlloc, &received, out, outLen, &sent, &status);
        g_fl_last_exec_end_ms = millis();
        if (outcome == FL_O_NO_SELECT) g_fl_no_select++;
#ifdef FORGE_LINK_WRITE_MARK
        // rx = when this EXEC started (the trace's "EXEC id" time); the log prefix is the completion time.
        // sum = ZuluSCSI's trace checksum (ZuluSCSI_log_trace.cpp scsiLogDataOut) over the bytes that went out.
        if (cdb[0] == 0x0C && outcome == FL_O_COMPLETED)
        {
            uint16_t c = 0;
            uint32_t n = sent <= outLen ? sent : outLen;
            for (uint32_t i = 0; i < n; i++) { c = (uint16_t)((c >> 1) + ((c & 1) << 15)); c = (uint16_t)(c + out[i]); }
            logmsg("FL WMARK rx ", (int)rx_ms, " out ", (int)sent, " sum ", (int)c);
        }
#endif
        rep[1] = status;
        why = nullptr;

        // Cache the peripheral type from a successful standard INQUIRY at LUN 0.
        if (cdb[0] == 0x12 && cdb[1] == 0x00 && cdb[2] == 0x00 &&
            outcome == FL_O_COMPLETED && status == 0x00 && received >= 1)
        {
            uint8_t b0 = land[0];
            g_fl_periph_type[target] = ((b0 >> 5) == 0) ? (b0 & 0x1F) : 0xFF;
        }
        // ⛔ NOT ON EVERY EXEC (the review of slice 2, required 1). The Blaster logs to UART0 at
        // 1 Mbaud and uart_puts blocks on a 32-byte FIFO, so this line cost ~1.1 ms per EXEC -
        // BEFORE the reply goes out, i.e. ~1.1% of the pipelined ceiling per fill. It is logged
        // when the outcome is not COMPLETED (a NO_SELECT is always seen, with its gap) or the EXEC
        // is not chained; the running totals are on the HELLO / EXIT line (fl_log_counters).
        // ~~|| g_fl_diag_log~~ (2026-09-28): that flag defaults ON and fl_rear never turns it off,
        // so the gate never closed on the bench; g_fl_exec_trace is the opt-in instead.
        // QX1: on the ship build a small EXEC is quiet too, so a CHECK CONDITION / BUSY (completed,
        // status != 0) would lose the line that small EXECs used to keep - it still logs there.
        if (outcome != FL_O_COMPLETED || !quiet || (g_fl_release_quiet_exec && status != 0x00))
            logmsg("Forge Link: EXEC done outcome ", (int)outcome, " status ", (int)status,
                   " received ", (int)received, " sent ", (int)sent,
                   "; bus free ", (int)gap_ms, " ms before it; no_select total ", (int)g_fl_no_select,
                   ", chain_broken refusals ", (int)g_fl_chain_refused FL_QX1_DONE_WHO(target, cdb[0]));
#else
        outcome = FL_O_NOT_INITIATOR;
        why = "platform has no initiator mode";
#endif
    } while (0);

    if (why) logmsg("Forge Link: EXEC refused (", why, "), nothing sent on the bus");

    // The latch follows EVERY EXEC, refused ones included: a refusal is a failure, and a
    // CHAIN_BROKEN keeps it broken until an unchained EXEC succeeds or a HELLO clears it.
    g_fl_chain_broken = !flExecChainSuccess(outcome, rep[1], sent, outLen, received, inAlloc);
    // the link guard remembers a successful 09 ON / OFF to ID 6 (ForgeLink_linkguard.cc).
    forgeLinkLinkGuardNoteExec(p, len, !g_fl_chain_broken, outcome);
#ifdef FORGE_LINK_NOTE_AT
    // TQ2 §10: the shared note accounting (armed only) for an EXEC that went on the bus.
    if (outcome < FL_O_REJECTED) forgeLinkNoteAtExecDone(p, len, !g_fl_chain_broken);
#endif

    rep[0] = outcome;
    fl_put32(&rep[2], received);
    fl_put32(&rep[6], sent);
#ifdef FL_STAMPS_ON
    g_fl_st.outcome = outcome;
    g_fl_st.status = rep[1];
    g_fl_st.sent = sent;
    g_fl_st.received = received;
#endif
#if defined(FORGE_LINK_PROBE) && defined(FORGE_LINK_BULK)
    if (from_slot)
    {
        // Sent from the slot (or, small / ZLP-prone / carrier gone, copied into the cache and sent
        // by flFramesSendReply as before - the function decides, the handler does not).
        flFramesSendReplyFromHead(FL_T_EXEC, seq, req_crc, req_len, rep, FL_REPLY_FIXED_LEN, received);
        return;
    }
#endif
    (void)land;
#ifdef FORGE_LINK_NOTE_AT
    if (outcome == FL_O_HELD_FOR_TIMED)
    {
        flFramesSendReply(FL_T_EXEC, seq, req_crc, req_len, rep, sizeof(rep), nullptr, 0);   // 18 bytes
        return;
    }
#endif
    flFramesSendReply(FL_T_EXEC, seq, req_crc, req_len, rep, FL_REPLY_FIXED_LEN, flFramesReplyDataBuffer(), received);
}

static void fl_dispatch()
{
    const uint8_t *hdr = flFramesHdr();
    uint8_t type = hdr[0];
    uint8_t seq = hdr[1];
    uint32_t len = fl_get32(&hdr[2]);
    const uint8_t *p = flFramesBody();
    uint32_t req_crc = fl_get32(&flFramesBody()[len]);

    // A block the vendor-bulk driver refused or lost ahead of this frame breaks the chain for it,
    // in arrival order (ForgeLink_frames.h, flFramesNoteChainBreak). Applied before anything else
    // so a HELLO still clears it and a replay still cannot run anything.
    if (flFramesHeadAfterChainBreak()) g_fl_chain_broken = true;
    // ARMRETRY r7 · DATACRC: the payload length the handler reads. == len, except a session-guarded
    // TQ_SYNC / TQ_STATUS, cut back to its old exact length by the session gate below.
    uint32_t plen = len;

    // Same sequence and identical request bytes as the last one: the host did
    // not get our reply. Resend it; never execute twice.
    if (type != FL_T_HELLO && flFramesReplayedDuplicate(seq, req_crc, len)) return;

#ifdef FL_UNIFIED
    // ⛔ SAFE MODE IN THE BRIDGE ROLE, the same rule as the disk role's gate
    // (ForgeLink_inject.cpp) and the shared vendor-bulk gate (ForgeLink_bulk.cpp): after K
    // consecutive hardware-watchdog boots the watchdog is never armed this run, and every verb that
    // could touch the bus is refused with FL_W_SAFE_MODE before a byte of it is parsed. Answered:
    // HELLO (the Mac must be able to SEE safe mode), HWDOG (status, and the clear), EXIT, and
    // REBOOT_UF2 - the bridge role's only exit, which disarms deliberately first. (CARD_MODE is in
    // the disk role's list; it has no meaning here and falls to the unknown-type answer.)
    if (forgeLinkHwdogSafeMode() && type != FL_T_HELLO && type != FL_T_HWDOG &&
        type != FL_T_EXIT && type != FL_T_REBOOT_UF2)
    {
        uint8_t rep = FL_W_SAFE_MODE;
        flFramesSendReply(type, seq, req_crc, len, &rep, 1, nullptr, 0);
        return;
    }
#endif

#if defined(FORGE_LINK_NOTE_AT) && defined(FORGE_LINK_BULK)
    // ⭐ ARMRETRY r7 · DATACRC: TQ_SYNC 16 / TQ_STATUS 5 carry the session after their payload; one
    // from another session is refused with the one-byte 0x18 (note-at §11) before it can arm, flush or
    // clear anything. The old exact lengths (12 / 1) pass unguarded. Either carrier.
    if (type == FL_T_TQ_SYNC || type == FL_T_TQ_STATUS)
    {
        const uint8_t g = forgeLinkBulkSessionGate(type, p, &plen);
        if (g)
        {
            flFramesSendReply(type, seq, req_crc, len, &g, 1, nullptr, 0);
            return;
        }
    }
#endif

    switch (type)
    {
        case FL_T_HELLO:
            fl_handle_hello(seq, req_crc, len);
            break;

#ifdef FL_UNIFIED
        // the watchdog's own frame, answered in the bridge role exactly as in the disk role
        // ('S' status [+ 'R' reset the loop-gap window], 'C'+"CLR!" clear the counter for the NEXT
        // boot). Same handler, same bytes: fl_hwdog.py needs no second code path for the bridge.
        case FL_T_HWDOG:
            forgeLinkHwdogFrame(type, seq, req_crc, len, p, len);
            break;
#endif

        case FL_T_EXEC:
            fl_handle_exec(seq, req_crc, len, p, len);
            break;

#ifdef FORGE_LINK_NOTE_AT
        // TQ2 · the timed-note verbs (docs/note-at.md). SAFE MODE refused them above with 09.
        case FL_T_NOTE_AT:
        case FL_T_TQ_SYNC:
        case FL_T_TQ_STATUS:
        case FL_T_RELEASE:
            forgeLinkNoteAtFrame(type, seq, req_crc, len, p, plen);
            break;
#endif

#ifdef FL_STAMPS_ON
        // DSP2 (bench only): the per-EXEC us ring, read after a run (ForgeLink_stamps.cc).
        case FL_T_STAMPS:
            forgeLinkStampsFrame(seq, req_crc, len, p, len);
            break;
#endif

#ifdef FL_RXSTAMP_ON
        // RX-SLOW (bench only): the receive-path event ring, read after a run (ForgeLink_rxstamp.cc).
        case FL_T_RXSTAMP:
            forgeLinkRxStampFrame(seq, req_crc, len, p, len);
            break;
#endif

        case FL_T_BUS_RESET:
#ifdef PLATFORM_HAS_INITIATOR_MODE
            if (platform_is_initiator_mode_enabled())
            {
                logmsg("Forge Link: BUS RESET requested by host");
                scsiHostPhyReset();
                fl_forget_types();
            }
#endif
            flFramesSendReply(type, seq, req_crc, len, nullptr, 0, nullptr, 0);
            break;

        case FL_T_EXIT:
            // The bridge stays armed in probe builds (the key menu is compiled
            // out of reach); EXIT only ends the host's session.
            flFramesInvalidateCache();
            flFramesSendReply(type, seq, req_crc, len, nullptr, 0, nullptr, 0);
            fl_log_counters("EXIT");
            break;

        case FL_T_ECHO:
            flFramesSendReply(type, seq, req_crc, len, p, len, nullptr, 0);
            break;

        // ---- ONE-WAY MEASUREMENT ----
        // SINK: the payload already arrived (the frame parser read it), so the host->board leg is
        // done by the time we get here. Reply with just the byte count: 4 bytes back for up to
        // 32 KiB in, so the reply leg is negligible and the timing is the INBOUND direction alone.
        case FL_T_DIAGLOG:
            g_fl_diag_log = (len >= 1 && p[0] != 0);
            // byte 1 (optional) drives the FAULT INJECTION, so the two can be separated without
            // a reflash - they were entangled all evening and that is what made the A/B useless.
            if (len >= 2) g_fl_nak_control = (p[1] != 0);
            flFramesSendReply(type, seq, req_crc, len, nullptr, 0, nullptr, 0);
            break;

        case FL_T_SINK:
        {
            uint8_t n4[4] = { (uint8_t)(len), (uint8_t)(len >> 8), (uint8_t)(len >> 16), (uint8_t)(len >> 24) };
            flFramesSendReply(type, seq, req_crc, len, n4, 4, nullptr, 0);
            break;
        }

        // FL_T_DBUF used to arm the double-buffered OUT experiment. The experiment is gone (N6
        // step 1; W49 is its crash log), but the verb number stays answered: it acks with an empty
        // payload, exactly what every build without the experiment has always replied, so an old
        // host tool asking gets the same answer it always got and nothing is ever armed.
        case FL_T_DBUF:
            flFramesSendReply(type, seq, req_crc, len, nullptr, 0, nullptr, 0);
            break;

        // SOURCE: a 4-byte request asks for N bytes; we send N. The request leg is 4 bytes, so the
        // timing is the board->host direction alone. Content is a counting pattern so the host can
        // verify nothing was dropped or reordered without needing the bytes echoed.
        case FL_T_SOURCE:
        {
            uint32_t want = 0;
            if (len >= 4) want = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
            if (want > 32768) want = 32768;
            // ⛔ NO 32 KiB PATTERN BUFFER OF ITS OWN (N6 probe-bulk, frame item 10). The pattern is
            // built straight into the reply cache at the payload offset, where flFramesSendReply
            // does not move it (p2 == &f[8]). That returns 32,768 B of bss to the heap: with the
            // vendor-bulk link the probe's free heap measured 31,624 B at b41e595 (the RAM-placed
            // USB path moved ~11 KiB of code into .data), under the 34,000 B floor.
#ifdef FORGE_LINK_BULK
            // The previous reply may still be leaving from these bytes on the vendor IN endpoint.
            flFramesWaitBulkTxIdle();
#endif
            uint8_t *pat = flFramesReplyPayloadAt(0);
            for (uint32_t i = 0; i < want; i++) pat[i] = (uint8_t)(i & 0xFF);
            flFramesSendReply(type, seq, req_crc, len, nullptr, 0, pat, want);
            break;
        }

        case FL_T_REBOOT_UF2:
            if (len == 4 && memcmp(p, "UF2!", 4) == 0)
            {
                logmsg("Forge Link: rebooting into UF2 bootloader on host request");
                flFramesSendReply(type, seq, req_crc, len, nullptr, 0, nullptr, 0);
                platform_flush_usb_log();
                delay(100);
                // ⛔ C90 (second half): ~~link guard, disarm, then reset_usb_boot(0, 0)
                // straight from here~~ - no wait for the sampler's SCSI write, no image close, no card
                // sync. Now the SAME committal as the vendor BOOTSEL (ForgeLink_uf2_reboot.h): the C44
                // link guard (note releases, LINK OFF) while still armed, the deliberate disarm, the UF2
                // flag and g_rebooting; this pass returns and spin_for_reboot drains (capped, A14.12),
                // closes, syncs, and platform_reset_mcu enters the bootloader.
                forgeLinkUf2RebootCommit("REBOOT_UF2");
            }
            else
            {
                uint8_t err = 1;
                flFramesSendReply(type, seq, req_crc, len, &err, 1, nullptr, 0);
            }
            break;

        default:
        {
            uint8_t err = 0xFF;  // unknown frame type
            flFramesSendReply(type, seq, req_crc, len, &err, 1, nullptr, 0);
            break;
        }
    }
}

// PASSIVE BUS LISTENER (contact-1, 2026-09-18). Reads SEL/BSY/ATN/RST and the data
// lines between commands and logs what it sees. It NEVER drives a line: no output
// enable is touched, so it cannot answer a selection or disturb the bus. Question it
// answers: with MIDI via SCSI ON, does the S1000 SELECT the remote ID (7) itself?
static void fl_bus_listen(void)
{
    static bool was_sel = false, was_bsy = false, was_rst = false;
    static uint32_t last_log_ms = 0, suppressed = 0;
    bool sel = SCSI_IN(SEL), bsy = SCSI_IN(BSY), rst = SCSI_IN(RST);
    uint32_t now = millis();
    bool rate_ok = (now - last_log_ms) >= 20;

    if (rst && !was_rst) logmsg("Forge Link BUS: RST asserted by another device");
    if (sel && !was_sel)
    {
        uint8_t ids = (uint8_t)(SCSI_IN_DATA() & 0xFF);
        if (rate_ok)
        {
            logmsg("Forge Link BUS: SEL asserted, data lines (ID bits) ", ids,
                   bsy ? " BSY on" : " BSY off", SCSI_IN(ATN) ? " ATN on" : "",
                   " [suppressed since last: ", (int)suppressed, "]");
            last_log_ms = now; suppressed = 0;
        }
        else suppressed++;
    }
    if (bsy != was_bsy && !sel && rate_ok)
    {
        logmsg("Forge Link BUS: BSY ", bsy ? "asserted" : "released", " (no SEL)");
        last_log_ms = now;
    }
    was_sel = sel; was_bsy = bsy; was_rst = rst;
}

// C14 (ForgeLink_probe.h): the bridge's SD log save waits for FL_LOGSAVE_QUIET_MS without a host frame,
// and at most FL_LOGSAVE_MAX_DEFER_MS. Main loop only.
static uint32_t g_fl_last_frame_ms = 0;
static uint32_t g_fl_logsave_defer_since_ms = 0;
static bool g_fl_logsave_deferring = false;
extern "C" bool forgeLinkBridgeLogSaveOk(void)
{
    const uint32_t now = millis();
    if ((uint32_t)(now - g_fl_last_frame_ms) >= FL_LOGSAVE_QUIET_MS) { g_fl_logsave_deferring = false; return true; }
    if (!g_fl_logsave_deferring) { g_fl_logsave_deferring = true; g_fl_logsave_defer_since_ms = now; }
    if ((uint32_t)(now - g_fl_logsave_defer_since_ms) >= FL_LOGSAVE_MAX_DEFER_MS) { g_fl_logsave_deferring = false; return true; }
    return false;
}

#ifdef FL_RXSTAMP_ON
// RX-SLOW (bench only): when the last pass of the bridge poll ENDED; the next entry stamps a LOOP_GAP when the
// rest of the main loop (platform_poll, log saves, the SD poll, ...) held it longer than FL_RXSTAMP_GAP_US.
static uint32_t g_fl_rx_pass_end_us;
#endif

extern "C" void forgeLinkProbeMainLoop(void)
{
#ifdef FL_RXSTAMP_ON
    {
        const uint32_t now = FL_RX_NOW();
        const uint32_t gap = now - g_fl_rx_pass_end_us;
        if (g_fl_rx_pass_end_us != 0 && gap > FL_RXSTAMP_GAP_US) FL_RX_AT(g_fl_rx_pass_end_us, FL_RXS_LOOP_GAP, 0, gap);
    }
#endif
    // ⭐ N6 PROBE-BULK · THE VENDOR-BULK DRIVER'S MAIN-LOOP HALF, FIRST (frame item 3): send a
    // stashed reply, retry a refused arm, release idle slots, serve a parked command block - so a
    // block that arrived while the last frame ran becomes the frame dispatched below, this pass.
    // Main loop only, never from platform_poll: the executor calls platform_poll mid-EXEC, and
    // this poll may write the reply buffer an EXEC's DATA_IN is landing in. The driver's Init is
    // NOT called here - the platform calls it (platform_late_init, under the same flag).
    forgeLinkBulkPoll();
    // ⭐ THE HARDWARE WATCHDOG, ARMED IN THE BRIDGE ROLE TOO (2026-09-30, both
    // modes). The same call, mechanism AND ORDER (right after the bulk poll) as the disk role's
    // forgeLinkInjectPoll: arm on the first pass (8 s, FL_HWDOG_TIMEOUT_MS), clear the consecutive
    // count after 60 s armed, sample the loop gap and the heap low-water mark. card_mode is always
    // false: card-reader mode is disk-role only. Only the call below is FL_UNIFIED; in the probe
    // build this comment describes a watchdog that build does not have.
    //
    // WHAT FEEDS IT (unchanged, ForgeLink_hwdog.cpp): (1) forgeLinkHwdogKick from
    // platform_reset_watchdog, thread mode on core 0; (2) forgeLinkHwdogAlarmFeed from upstream's
    // 1 s TIMER-IRQ tick (watchdog_callback) while ARMED - the "truly frozen only" ruling. So
    // any block with interrupts ALIVE is fed at <= 1 s whatever the main loop is doing, and the 8 s
    // watchdog fires only when interrupts are dead for 8 s. A live-interrupt wedge is left to
    // upstream's software ladder (bus reset at 15 s, reboot at 30 s - which disarms and COUNTS).
    //
    // EVERY PLACE THE BRIDGE LOOP CAN BLOCK · worst-case gap between hardware feeds, vs 8,000 ms:
    //   main loop pass           platform_reset_watchdog at the top of zuluscsi_main_loop  kick/pass
    //   5 s diag/NAK block       one log burst per 5 s, not a 5 s block; the NAK positive
    //                            control (default OFF) busy-waits 5 ms, IRQs on            <= 5 ms
    //   selection                scsiHostPhySelect / fl_select_scsi1: 2,500 x 100 us + 5 ms
    //                            of sampling, IRQs on                             ~255 ms (tick 1 s)
    //   EXEC phase loop          platform_reset_watchdog every iteration                  kick/iter
    //   one DATA_IN / DATA_OUT   scsiHostRead: PIO accel, 10 s no-data timeout or the 15 s
    //   (32 KiB, a slow Akai)    bus-reset flag; scsiHostWrite: REQ wait until that flag.
    //                            No kick inside (upstream code), IRQs on                1 s (tick)
    //   bus-free wait            scsiHostWaitBusFree: <= 10 s, then a 252 ms reset, IRQs on  1 s (tick)
    //   RELBSY knock             <= timeout_ms (<= 60 s), kick every iteration             kick/iter
    //   BLIND settle / BSY watch <= 2 s settle kicked every 1 ms; 3 s watch kicked every iteration
    //   bulk TX-idle wait        flFramesWaitBulkTxIdle: <= 2 s, kick every 1 ms       <= 1 ms
    //   CDC reply write          fl_write_all: <= 2 s host stall, kick every iteration     kick/iter
    //   host BUS_RESET           scsiHostPhyReset: 2 + 250 ms of delay(), IRQs on          1 s (tick)
    //   REBOOT_UF2               delay(100), then the UF2 commit (disarm, deliberate); spin_for_reboot drains <= 2 s, resets
    //   log lines / log save     blocking UART ~1 ms a line; save_logfile to SD, IRQs on   1 s (tick)
    // Interrupt-MASKED sections reachable from the bridge are all bounded far under 8 s: the USB
    // re-arm guard (<= 10,000 polls, tinyusb_override/rp2040_usb.c) and the XIP/JEDEC setup at init.
#ifdef FL_UNIFIED
    forgeLinkHwdogPoll(false);   // arm / healthy-clear / loop gap
#endif
    // The heap low-water mark: two reads and a compare, main loop only.
    uint32_t heap_now = (uint32_t)rp2040.getFreeHeap();
    if (heap_now < g_fl_heap_low_water) g_fl_heap_low_water = heap_now;
    // Pull any input here too (platform_poll also does), then run at most one
    // pending frame. Runs from the main loop only, never inside platform_poll,
    // because the executor itself calls platform_poll (C10).
#ifdef FORGE_LINK_NOTE_AT
    // TQ2 (note-at §6): the safety checks (lease, USB, bulk reset), the owed clean-up and any due fire,
    // BEFORE the next host frame is picked up.
    forgeLinkNoteAtPoll();
#endif
    forgeLinkSerialPoll();
    const bool fed = flFramesNextHead();   // C2a: drops a BULK RESET's abandoned frames first
    if (fed)
    {
        g_fl_last_frame_ms = millis();     // C14: MIDI is flowing - the SD log save waits
#ifdef FL_STAMPS_ON
        // DSP2 t_pick FIRST, then the head frame's own arrival stamps (taken by the parser / the bulk
        // commit in micros(), the same TIMER0 count time_us_32 reads). The record is committed after
        // the dispatch, when the reply has already gone out: packing it is outside every interval.
        flStamp(FL_ST_PICK);
        g_fl_st.open = 0;
        g_fl_st.handshakes = 0;
        g_fl_st.flags = g_fl_st.outcome = g_fl_st.cdb0 = g_fl_st.out0 = 0;
        g_fl_st.status = 0xFF;
        g_fl_st.sent = g_fl_st.received = 0;
        g_fl_st.mask = (uint16_t)(1u << FL_ST_PICK);
        flStampAt(FL_ST_RX0, flFramesHeadStartUs());
        flStampAt(FL_ST_USB, flFramesHeadDoneUs());
        g_fl_st.type = flFramesHdr()[0];
        g_fl_st.seq = flFramesHdr()[1];
        g_fl_st.len = fl_get32(&flFramesHdr()[2]);
#endif
#ifdef FL_RXSTAMP_ON
        FL_RX(FL_RXS_PICK, flFramesHdr()[0], flFramesHdr()[1]);   // RX-SLOW (bench only)
#endif
        fl_dispatch();
#ifdef FL_RXSTAMP_ON
        FL_RX(FL_RXS_DONE, flFramesHdr()[0], flFramesHdr()[1]);   // RX-SLOW (bench only)
#endif
        flFramesClearPending();
#ifdef FORGE_LINK_NOTE_AT
        forgeLinkNoteAtServed();   // TQ2 §4: any served frame renews the lease
#endif
#ifdef FL_STAMPS_ON
        forgeLinkStampsCommit();
#endif
    }
    else
    {
        fl_bus_listen();
    }
    // THE LINK GUARD, after the dispatch: a served frame stamps the host clock; a quiet pass
    // may send 09 off if the host left the sampler's link ON and went silent / unplugged.
    forgeLinkLinkGuardPoll(fed);
#ifdef FL_RXSTAMP_ON
    g_fl_rx_pass_end_us = FL_RX_NOW();
#endif
}

#ifdef FL_UNIFIED
// ⛔ C2c · THE UNION'S BANNER TEXT LIVES IN FLASH, not .rodata: a .cpp.o's .rodata is copied to
// RAM on this template (rp23xx-template.ld excludes *.cpp.o from the flash .rodata; .flashdata*
// stays in flash), and C2a left the union 180 B under its free-heap floor. Same text, byte
// for byte; only where it is stored moved. Safe in flash: forgeLinkLogBanner runs once, from
// setup() (ZuluSCSI.cpp), on the main thread - never from an IRQ, never while flash is written
// (the only flash writes are the ROM-drive/firmware-update paths, interrupts off, no logmsg).
// The two DIP-reading literals stay inline (test_the_union_banner_names_the_role_and_the_dip_reading).
#define FL_COLD_STR(name, text) \
    static const char name[] __attribute__((section(".flashdata.fl_c2c_" #name))) = text
FL_COLD_STR(k_flb_rule, "=============================================================");
FL_COLD_STR(k_flb_name, " " FORGE_LINK_NAME " git:" ZULU_BUILD_GIT_REV " · INITIATOR DIP ");
FL_COLD_STR(k_flb_clean, " tree: clean at build time");
FL_COLD_STR(k_flb_unknown, " tree: UNKNOWN at build time (no build-info header)");
FL_COLD_STR(k_flb_dirty, " tree: DIRTY at build time, ");
FL_COLD_STR(k_flb_paths, " path(s): " ZULU_BUILD_DIRTY_FILES);
FL_COLD_STR(k_flb_fork, " A fork of ZuluSCSI-firmware by Rabbit Hole Computing, GPLv3.");
FL_COLD_STR(k_flb_roles, " One binary, two roles: the INITIATOR DIP chooses at boot (flip it only powered down).");
FL_COLD_STR(k_flb_scan, " Imaging scan and USB MSC initiator are compiled out in BOTH roles.");
FL_COLD_STR(k_flb_allow, " EXEC allow-list: 12h, 00h; 0Ah/08h only to a type-03h target.");
FL_COLD_STR(k_flb_frames, " USB serial input is bridge frames only: console keys are off.");
FL_COLD_STR(k_flb_leave, " Leave this role by REBOOT_UF2 (it disarms the hardware watchdog first).");
FL_COLD_STR(k_flb_safe, "   In the BRIDGE role SAFE MODE refuses EXEC and every verb but HELLO, HWDOG, EXIT"
                        " and REBOOT_UF2 (the 'plain disk' line above is the disk role's wording).");
FL_COLD_STR(k_flb_fault_on, " ⛔⛔ FAULT INJECTION ARMED: EP1 OUT is DISABLED 5 ms every second. Frame loss "
                            "measured in this state is NOT the board's behaviour.");
FL_COLD_STR(k_flb_fault_off, " fault injection: off (EP1 OUT is never deliberately disabled).");
FL_COLD_STR(k_flb_fifo, " CDC TX FIFO the LIBRARY actually has: ");
FL_COLD_STR(k_flb_bytes, " bytes");
FL_COLD_STR(k_flb_marker, " build marker: " __DATE__ " " __TIME__);
#undef FL_COLD_STR
#endif // FL_UNIFIED

extern "C" void forgeLinkLogBanner(void)
{
#ifdef FL_UNIFIED
    // ⭐ ONE LINE THAT NAMES THE BUILD, THE ROLE AND THE DIP READING, so the first thing read on
    // the CDC log says which half of this binary is running and why.
    const bool bridge = platform_is_initiator_mode_enabled();
    logmsg(k_flb_rule);
    logmsg(k_flb_name,
           bridge ? "ON -> BRIDGE (initiator)" : "OFF -> DISK + injector");
    if (ZULU_BUILD_DIRTY_COUNT == 0) logmsg(k_flb_clean);
    else if (ZULU_BUILD_DIRTY_COUNT < 0) logmsg(k_flb_unknown);
    else logmsg(k_flb_dirty, (int)ZULU_BUILD_DIRTY_COUNT, k_flb_paths);
    logmsg(k_flb_fork);
    logmsg(k_flb_roles);
    logmsg(k_flb_scan);
    if (bridge)
    {
        logmsg(k_flb_allow);
        logmsg(k_flb_frames);
        logmsg(k_flb_leave);
        // the watchdog is armed in this role too; the shared banner says its state and record.
        forgeLinkHwdogBanner();
        if (forgeLinkHwdogSafeMode())
            logmsg(k_flb_safe);
        if (g_fl_nak_control)
            logmsg(k_flb_fault_on);
        else
            logmsg(k_flb_fault_off);
        logmsg(k_flb_fifo, (int)Serial.availableForWrite(), k_flb_bytes);
    }
    else
    {
        forgeLinkInjectBannerLines();
    }
    logmsg(k_flb_marker);
    logmsg(k_flb_rule);
#else
    logmsg("=============================================================");
    logmsg(" " FORGE_LINK_NAME " git:" ZULU_BUILD_GIT_REV);
    logmsg(" A fork of ZuluSCSI-firmware by Rabbit Hole Computing, GPLv3.");
    logmsg(" Imaging scan and USB MSC initiator are compiled out.");
    logmsg(" EXEC allow-list: 12h, 00h; 0Ah/08h only to a type-03h target.");
    logmsg(" USB serial input is bridge frames only: console keys are off.");
    logmsg(" EXEC flag BLIND (0x40) known: CDB clocked on fixed timing, REQ never read (study W35).");
    // ⛔ A FAULT INJECTION MUST ANNOUNCE ITSELF. This one ran for hours in a build everyone
    // called stable, and its losses were measured as the board's own behaviour. If it is ever
    // armed again, the very first thing anyone reads must say so.
    if (g_fl_nak_control)
        logmsg(" ⛔⛔ FAULT INJECTION ARMED: EP1 OUT is DISABLED 5 ms every second. Frame loss "
               "measured in this state is NOT the board's behaviour.");
    else
        logmsg(" fault injection: off (EP1 OUT is never deliberately disabled).");
    // ⛔ REPORT THE VALUE THE FIRMWARE WAS ACTUALLY COMPILED WITH, not the one platformio.ini
    // asked for. A -D in build_flags does not necessarily reach a framework translation unit,
    // and an A/B against a flag that never took is indistinguishable from a dead hypothesis -
    // which is how a failed flash nearly became a finding earlier today (C82/C77).
    // ⛔ THE RUNTIME READ-BACK, from the layer that OWNS the value. The previous line here
    // printed CFG_TUD_CDC_EP_BUFSIZE, which is only visible in THIS translation unit when a -D
    // puts it here - so it echoed our own build flag back at us while the PRECOMPILED TinyUSB in
    // libpico.a ran its own 256/256/64. That is how two hypotheses (C77, C82) were recorded dead
    // having never been tested (C87). Serial.availableForWrite() on an idle port IS the library's
    // real TX FIFO size, and it cannot lie about which code is running.
    logmsg(" CDC TX FIFO the LIBRARY actually has: ", (int)Serial.availableForWrite(), " bytes");
    // ⛔ A BUILD MARKER, because a git hash is NOT an identity when the tree is dirty: two
    // different firmwares both reported "8ed464f-dirty" tonight and I could not tell which was
    // on the board, so a fix could not be distinguished from an untested build (the same class
    // of error as the flash that silently failed, C82).
    // The "[stable: ...]" tag is kept word for word: it is what every build since the double-buffer
    // experiment was compiled out has printed, and that experiment no longer exists to compile in.
    // ⭐ WHICH FILES WERE DIRTY AT BUILD TIME, so the METHOD RULE ("verify the build on the board
    // by its banner hash") can actually be made: "-dirty" alone never said whether firmware or a
    // doc differed (2026-09-22). The header half landed then; this is the banner half (2026-09-24).
    if (ZULU_BUILD_DIRTY_COUNT == 0) logmsg(" tree: clean at build time");
    else if (ZULU_BUILD_DIRTY_COUNT < 0) logmsg(" tree: UNKNOWN at build time (no build-info header)");
    else logmsg(" tree: DIRTY at build time, ", (int)ZULU_BUILD_DIRTY_COUNT, " path(s): " ZULU_BUILD_DIRTY_FILES);
    logmsg(" build marker: " __DATE__ " " __TIME__
           "  [stable: no USB experiment compiled in]");
    logmsg(" Role: follows the INITIATOR DIP (forcing removed 2026-09-24).");
    logmsg("=============================================================");
#endif // FL_UNIFIED
}

#elif !defined(FORGE_LINK_INJECT)
// Neither the probe nor the injector: the four entry points the platform layer calls
// unconditionally still need bodies. In an INJECTION build they are provided by
// ForgeLink_inject.cpp instead, which is why that arm is excluded here.

extern "C" bool forgeLinkCdbAllowed(int, const uint8_t *, size_t, const char **) { return true; }
extern "C" bool forgeLinkSerialPoll(void) { return false; }
extern "C" bool forgeLinkTxBusy(void) { return false; }
extern "C" void forgeLinkProbeMainLoop(void) {}
extern "C" void forgeLinkLogBanner(void) {}

#endif // FORGE_LINK_PROBE
