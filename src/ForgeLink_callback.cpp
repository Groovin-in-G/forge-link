/**
 * Forge Link - a fork of ZuluSCSI-firmware by Rabbit Hole Computing.
 * Modifications Copyright (c) 2026 the Forge Link contributors.
 * GNU General Public License v3 or later; see LICENSE.
**/

// Forge Link TARGET7 "call-back" experiment (contact-1, 2026-09-18).
//
// Measured: with MIDI via SCSI ON (own 6, remote 7), an EXCL SEND makes the S1000 select
// ID 7 and run TEST UNIT READY + INQUIRY(7), twice, then give up after ~1 s, whatever
// INQUIRY answer it gets. Hypothesis: it then waits for the remote to CALL IT BACK.
// So: ~30 ms after the S1000's last INQUIRY to ID 7, with the bus free, switch the Blaster
// to host role WITHOUT a bus reset, select ID 6 once, send INQUIRY(36) and, only if the
// reply says processor (03h), RECEIVE(6) of up to 1 KB. Log everything, then restore the
// target role. Allow-list: only 12h and 08h are ever sent; nothing that writes.

// ⭐ THE MAIN-LOOP SLOT. zuluscsi_main_loop() already calls forgeLinkCallbackPoll() between
// scsiPoll() and scsiDiskPoll() - exactly the "from the main loop, at bus-free" slot the block
// injector needs - so the injector hangs off this one existing call rather than adding another
// to ZuluSCSI.cpp. In any build that is not the injection build it is an empty inline.
#include "ForgeLink_inject.h"

#ifdef FORGE_LINK_TARGET7

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "ZuluSCSI_log.h"
#include <ZuluSCSI_platform.h>
#include <scsiHostPhy.h>
#include <scsiPhy.h>
extern "C" {
#include <scsi.h>
}
#include "scsi_accel_host.h"
extern "C" void forgeLinkReenterTargetRole(void);

// ⛔ H5 REFUTED BY THE ROM (2026-09-19, second pass). FL_SETTLE_MS was built on the theory that
// our own bus reset leaves RESET CONDITION set in the S1000's INTS, and that its exact-compare
// against 80h then bails. The ROM says that bail-out branch CLEARS INTS AND ISSUES BUS RELEASE -
// so a stale flag would make BSY DROP, which is the opposite of what we see. It also cannot
// explain attempt 1, before any reset had been issued. The settle is kept because it is harmless
// and paces the knocks; it is NOT a fix for anything, and nobody should tune it.
#define FL_SELECT_TRIES 5
#define FL_SETTLE_MS     900   // its main loop must run and clear INTS between knocks   // the blind states self-clear; one try cannot tell dead from mid-window
static volatile uint8_t g_fl_probe_id = 6;   // !T <id> - 6 is the S1000; any other id is the CONTROL
static volatile uint32_t g_fl_last_inq_ms = 0;
static volatile bool g_fl_armed = false;
static volatile uint32_t g_fl_wait_ms = 3000;

static volatile uint32_t g_fl_fire_at = 0;     // 0 = not scheduled
static volatile uint32_t g_fl_after_close_ms = 0;  // 0 = do not chase the session close

// Schedule a call to ID 6 <ms> from now. The window we are hunting is narrow, so the delay
// is a parameter rather than a constant: "just after it closes a session" is the first
// candidate, "during its disk-not-ready loop" the second.
extern "C" void forgeLinkRemoteEnableIn(uint32_t ms)
{
    g_fl_wait_ms = 2000;
    g_fl_fire_at = millis() + ms;
    logmsg("Forge Link CB: call to ID 6 scheduled in ", (int)ms, " ms");
}

// Chase the NEXT session close by <ms>. 0 disables.
extern "C" void forgeLinkChaseClose(uint32_t ms)
{
    g_fl_after_close_ms = ms;
    if (ms) logmsg("Forge Link CB: will call ID 6 ", (int)ms, " ms after the session closes");
    else    logmsg("Forge Link CB: close-chasing off");
}

// ── THE FL-WRITE REPORT ────────────────────────────────────────────────────────────────────
// One line per write, printed when the S1000 closes the session. Counters live in scsi.c
// beside the mailbox they measure.
//
// ⚠ audio_Bps CARRIES AN ASSUMPTION AND SAYS SO: an SDS data packet is 127 bytes of which 120
// are payload, 7-bit packed at 3 wire bytes per 16-bit word, so 40 words = 80 audio bytes per
// packet. That held for W1 (30 packets x 40 = 1200 words exactly). It is arithmetic on the
// packet count, NOT a measurement, and a differently-packed dump would make it wrong. The raw
// counts beside it are the measurement; delete the derived figure before trusting it blindly.
extern "C" uint32_t g_flw_t0, g_flw_tlast, g_flw_wire;
extern "C" uint32_t g_flw_polls, g_flw_acks, g_flw_dpkts, g_flw_hdrs;

extern "C" void forgeLinkWriteReport(void)
{
    if (!g_flw_t0) return;                       // nothing was handed over this session
    uint32_t ms    = g_flw_tlast - g_flw_t0;
    uint32_t audio = g_flw_dpkts * 80;
    // ⛔ A rate from a sub-10 ms window is an invented number, the same rule forgeLinkLogBulk
    // already lives by. Report the counts and withhold the rate rather than printing a fiction.
    if (ms >= 10)
        logmsg("FL-WRITE ms=", (int)ms, " wire=", (int)g_flw_wire, " dpkts=", (int)g_flw_dpkts,
               " hdrs=", (int)g_flw_hdrs, " acks=", (int)g_flw_acks, " polls=", (int)g_flw_polls,
               " audioB=", (int)audio,
               " wire_Bps=", (int)((g_flw_wire * 1000ULL) / ms),
               " audio_Bps=", (int)((audio * 1000ULL) / ms), " (audio assumes SDS 16-bit)");
    else
        logmsg("FL-WRITE ms=", (int)ms, " wire=", (int)g_flw_wire, " dpkts=", (int)g_flw_dpkts,
               " hdrs=", (int)g_flw_hdrs, " acks=", (int)g_flw_acks, " polls=", (int)g_flw_polls,
               " audioB=", (int)audio, " (too short to rate)");
    g_flw_t0 = g_flw_tlast = g_flw_wire = 0;
    g_flw_polls = g_flw_acks = g_flw_dpkts = g_flw_hdrs = 0;
}

// Called from the target path the moment the S1000 closes a session (09 with byte 2 = 0).
extern "C" void forgeLinkSessionClosed(void)
{
    forgeLinkWriteReport();
    if (g_fl_after_close_ms) forgeLinkRemoteEnableIn(g_fl_after_close_ms);
}

// T1b: console 'k' arms one call to ID 6 with a 60 s wait for REQ (E8 folded in).
extern "C" void forgeLinkCallbackArm(void)
{
    g_fl_wait_ms = 60000;
    g_fl_last_inq_ms = millis() - 1000;
    g_fl_armed = true;
    logmsg("Forge Link CB: armed from console (T1b), will wait up to 60 s for REQ");
}

// ⛔ DOES NOT AUTO-ARM ANY MORE. This used to set g_fl_armed on every INQUIRY to ID 7, which
// was right for the contact-1 experiment and catastrophic once the call-back was compiled into
// the BRIDGE build: 20 ms into every session the board stopped being a target, seized the bus as
// an initiator and wrecked the transfer in progress - and left the target side unable to complete
// a command until it was rebooted. Three sessions were lost to it before the log showed the
// call firing 20 ms after a perfect handshake (bench 2026-09-19).
// A call to ID 6 now happens ONLY when something explicitly schedules one: !E <ms>, !X <ms>,
// or the console 'k' key. The timestamp is still recorded so a scheduled call can be placed
// relative to the sampler's own activity.
extern "C" void forgeLinkNoteCommand(uint8_t target_id, uint8_t opcode)
{
#ifdef FORGE_LINK_CALLBACK
    if ((target_id & 7) == 7 && opcode == 0x12)
    {
        g_fl_last_inq_ms = millis();
    }
#else
    (void)target_id; (void)opcode;  // call-back experiment off in the rotor build
#endif
}

extern "C" void forgeLinkLogHex(const char *what, const uint8_t *d, int n)
{
    char line[3 * 32 + 1];
    logmsg(what, " (", n, " bytes)");
    for (int off = 0; off < n; off += 32)
    {
        int k = (n - off) < 32 ? (n - off) : 32, p = 0;
        for (int i = 0; i < k; i++) p += snprintf(line + p, sizeof(line) - p, "%02X ", d[off + i]);
        logmsg(what, " +", off, ": ", line);
    }
}

extern "C" uint8_t g_fl7_inq[36], g_fl7_inq_len, g_fl7_mode[256], g_fl7_din[1024];
extern "C" uint16_t g_fl7_len[256], g_fl7_din_len;

static int fl_parse_hex(const char *p, uint8_t *out, int max)
{
    int n = 0;
    while (*p && n < max)
    {
        while (*p == ' ') p++;
        if (!*p) break;
        char *e; long v = strtol(p, &e, 16);
        if (e == p) break;
        out[n++] = (uint8_t)v; p = e;
    }
    return n;
}

extern "C" uint8_t g_fl_sweep_kind, g_fl_sweep_i;
extern "C" uint8_t g_fl7_out[8192];
extern "C" uint16_t g_fl7_out_len, g_fl7_out_pos;
extern "C" uint8_t g_fl7_mail;
extern "C" uint16_t g_fl7_chunk;
extern "C" void forgeLinkRemoteEnableIn(uint32_t ms);
extern "C" void forgeLinkChaseClose(uint32_t ms);
extern "C" uint16_t g_fl_sweep_tries[64];

// ── L1b SERVE ON REQUEST: staging, arming, the data phase and its log (scsi.c "SERVE ON
// REQUEST"; the decisions are in fl7_serve.h) ─────────────────────────────────────────────────
extern "C" {
#include "fl7_serve.h"
}
extern "C" fl7s_t g_fl7s;

static const char *fl7s_state_name(int s)
{
    static const char *n[] = {"OFF", "ARMED", "HEADER", "FILL", "DONE"};
    return (s >= 0 && s <= 4) ? n[s] : "?";
}

// "!B <offset> <densehex>" / "!H <offset> <densehex>": stage audio / the SDATA header.
// ⛔ CONTIGUOUS ONLY: offset 0 restarts, any other offset must equal the current length, so a
// dropped or repeated line is REFUSED rather than leaving a hole the S1000 would play.
static void fl7s_stage(const char *what, const char *l, uint8_t *buf, uint32_t cap, uint32_t *len)
{
    char *e; unsigned long off = strtoul(l, &e, 0);
    int ok = (e == l) ? 2 : fl7s_stage_ok(&g_fl7s, (uint32_t)off, *len, cap);
    if (ok == 1)
    {
        logmsg("FL7 STAGE ", what, " REFUSED: a serve is armed (!W 0 first)");
        return;
    }
    if (ok)
    {
        logmsg("FL7 STAGE ", what, " REFUSED: offset ", (int)off, " but length is ", (int)*len, " of ", (int)cap);
        return;
    }
    int n = fl7s_parse_dense_hex(e, buf + off, (int)(cap - off));
    if (n <= 0)
    {
        logmsg("FL7 STAGE ", what, " REFUSED: bad hex or past the ", (int)cap, "-byte buffer at ", (int)off);
        return;
    }
    *len = (uint32_t)off + (uint32_t)n;
    g_fl7s.state = FL7S_OFF;           // new material: any finished serve is history
    logmsg("FL7 STAGE ", what, " @", (int)off, " +", n, " = ", (int)*len);
}

static void fl7s_report(void)
{
    uint32_t resid = 0;
    int chk = fl7s_check_sdata(g_fl7s.sdata, g_fl7s.sdata_len, &resid);
    logmsg("FL7 SERVE ", fl7s_state_name(g_fl7s.state), ": header ", (int)g_fl7s.sdata_len,
           " bytes (check ", chk, ", residual ", (int)resid, ", sum ", (int)fl7s_sum(0, g_fl7s.sdata, g_fl7s.sdata_len),
           "), audio ", (int)g_fl7s.audio_len, " of ", (int)g_fl7s.cap, " bytes (sum ",
           (int)fl7s_sum(0, g_fl7s.audio, g_fl7s.audio_len), "), serve total ", (int)fl7s_total(&g_fl7s),
           ", chunk ", (int)g_fl7s.chunk, ", served ", (int)g_fl7s.pos, " in ", (int)g_fl7s.cycles,
           " cycles, padded ", (int)g_fl7s.pad, ", count/0E mismatches ", (int)g_fl7s.warn,
           ", zero polls ", (int)g_fl7s.zero_polls, ", largest count ", (int)g_fl7s.maxcount);
    // The staging buffer's cost, measured on the running board rather than argued from the
    // linker: what the heap has left with the 64 KiB buffer in place.
    logmsg("FL7 SERVE RAM: heap free ", rp2040.getFreeHeap(), " of ", rp2040.getTotalHeap(), " bytes");
}

extern "C" void forgeLinkServeEvent(int ev, uint32_t a, uint32_t b, uint32_t c)
{
    switch (ev)
    {
    case FL7S_EV_SDATA_QUEUED:
        logmsg("FL7 SERVE RSDATA for sample ", (int)a, " -> SDATA queued (", (int)FL7S_SDATA_LEN,
               " bytes, one packet per poll)"); break;
    case FL7S_EV_RSDATA_IGNORED:
        logmsg("FL7 SERVE RSDATA for sample ", (int)a, " NOT answered (state ", fl7s_state_name((int)b), ")"); break;
    case FL7S_EV_END_OF_LIST:
        logmsg("FL7 SERVE RSDATA for sample ", (int)a, " -> lone F7 (end of list: one serve per arm)"); break;
    case FL7S_EV_SDS_REFUSED:
        logmsg("FL7 SERVE SDS DUMP REQUEST seen - NOT answered (struck carrier). Set protocol S1000."); break;
    case FL7S_EV_HEADER_DONE:
        logmsg("FL7 SERVE header handed over whole (", (int)a, " bytes)"); break;
    case FL7S_EV_RSPACK:
        logmsg("FL7 SERVE RSPACK seen (taken, not answered: no ASPACK, by design)"); break;
    case FL7S_EV_ACK:
        break;                          // already logged in full by the FL7 BULK line
    case FL7S_EV_BEGIN:
        logmsg("FL7 SERVE BEGIN bulk fill at ", (int)g_fl7s.t0, " ms, first count ", (int)a); break;
    case FL7S_EV_END:
        if (b >= 10)
            logmsg("FL7 SERVE END ", (int)a, " bytes in ", (int)b, " ms = ", (int)((a * 1000ULL) / (b * 1024ULL)),
                   " KiB/s (board ms, first bulk 0D..last byte, poll gaps included), ", (int)g_fl7s.cycles,
                   " cycles, sum ", (int)c, ", padded ", (int)g_fl7s.pad, ", mismatches ", (int)g_fl7s.warn);
        else
            logmsg("FL7 SERVE END ", (int)a, " bytes in ", (int)b, " ms (too short to rate), ", (int)g_fl7s.cycles,
                   " cycles, sum ", (int)c, ", padded ", (int)g_fl7s.pad, ", mismatches ", (int)g_fl7s.warn);
        break;
    case FL7S_EV_RESET:
        logmsg("FL7 SERVE BUS RESET during a bulk 0E at byte ", (int)a, " - fill abandoned"); break;
    case FL7S_EV_CLOSED:
        logmsg("FL7 SERVE session closed (09 link off) from ", fl7s_state_name((int)a), ": served ", (int)b,
               " of ", (int)c, a == FL7S_DONE ? " - COMPLETE" : (a == FL7S_ARMED ? " - still armed (no RSDATA came)" :
               " - INCOMPLETE, disarmed"));
        fl7s_report();
        break;
    }
}

// ⛔ THE DATA PHASE. One bulk 0E = `real` staged bytes then `pad` zeros, as ONE phase, with:
//  - the debug trace SUSPENDED (g_log_debug off): no per-slice lines and no per-byte checksum,
//    so the number measures the S1000's intake, not our logging (W38's 241 KB/s was taken with
//    a hex line per 1 KiB slice);
//  - the board's MICROSECOND clock on the first and last slice, and every slice timed, so a
//    stall of more than 1 ms inside the phase is named rather than averaged away.
// Slices are 4 KiB and synchronous (scsiWrite), each far inside the accel's 5 s finish timeout.
#define FL7S_SLICE 4096u
#define FL7S_MAXSLICES 256u
static uint8_t fl7s_zero[512];                       // RAM zeros: the pad source
static uint32_t fl7s_slice_us[FL7S_MAXSLICES], fl7s_slice_len[FL7S_MAXSLICES];
extern "C" int forgeLinkServeStream(uint32_t real, uint32_t pad)
{
    bool dbg = g_log_debug;
    g_log_debug = false;
    uint32_t p = g_fl7s.pos, left = real, ns = 0;
    uint32_t t_first = micros(), t_prev = t_first;
    while (left && !scsiDev.resetFlag)
    {
        uint32_t k;
        const uint8_t *src = fl7s_src(&g_fl7s, p, left < FL7S_SLICE ? left : FL7S_SLICE, &k);
        scsiWrite(src, k);
        uint32_t t = micros();
        if (ns < FL7S_MAXSLICES) { fl7s_slice_us[ns] = t - t_prev; fl7s_slice_len[ns] = k; }
        ns++;
        t_prev = t;
        p += k; left -= k;
    }
    uint32_t t_last = t_prev;
    uint32_t z = pad;
    while (z && !scsiDev.resetFlag)
    {
        uint32_t k = z > sizeof(fl7s_zero) ? sizeof(fl7s_zero) : z;
        scsiWrite(fl7s_zero, k);
        z -= k;
    }
    g_log_debug = dbg;
    int reset = scsiDev.resetFlag ? 1 : 0;

    // After the phase (logging is allowed again): the rate, the fastest full slice as the
    // reference pace, and every slice that took more than 1 ms longer than that pace predicts.
    uint32_t us = t_last - t_first, n = ns < FL7S_MAXSLICES ? ns : FL7S_MAXSLICES;
    uint32_t best_i = 0, slow = 0;
    for (uint32_t i = 0; i < n; i++)
        if (fl7s_slice_len[i] == FL7S_SLICE &&
            (fl7s_slice_len[best_i] != FL7S_SLICE || fl7s_slice_us[i] < fl7s_slice_us[best_i])) best_i = i;
    for (uint32_t i = 0; i < n; i++) if (fl7s_slice_us[i] > fl7s_slice_us[slow]) slow = i;
    uint32_t gaps = 0; char where[96]; int wp = 0; where[0] = 0;
    if (n)
    {
        uint64_t ref_ns_per_b = ((uint64_t)fl7s_slice_us[best_i] * 1000ULL) /
                                (fl7s_slice_len[best_i] ? fl7s_slice_len[best_i] : 1);
        uint32_t off = 0;
        for (uint32_t i = 0; i < n; i++)
        {
            uint64_t expect_us = (ref_ns_per_b * fl7s_slice_len[i]) / 1000ULL;
            if (fl7s_slice_us[i] > expect_us + 1000u)
            {
                gaps++;
                if (wp < (int)sizeof(where) - 24)
                    wp += snprintf(where + wp, sizeof(where) - wp, " @%lu+%luus", (unsigned long)off,
                                   (unsigned long)(fl7s_slice_us[i] - expect_us));
            }
            off += fl7s_slice_len[i];
        }
    }
    if (us)
        logmsg("FL7 SERVE 0E ", (int)(real + pad), " bytes (", (int)real, " audio + ", (int)pad, " pad) from byte ",
               (int)g_fl7s.pos, ": ", (int)us, " us first..last slice = ",
               (int)((real * 1000000ULL) / ((uint64_t)us * 1024ULL)), " KiB/s; ", (int)ns, " slices, fastest ",
               (int)fl7s_slice_us[best_i], " us, slowest ", (int)fl7s_slice_us[slow], " us; gaps >1 ms: ", (int)gaps,
               where, reset ? "; BUS RESET" : "", ns > FL7S_MAXSLICES ? "; (slice log truncated)" : "");
    else
        logmsg("FL7 SERVE 0E ", (int)(real + pad), " bytes (", (int)real, " audio + ", (int)pad,
               " pad): too short to time", reset ? "; BUS RESET" : "");
    return reset;
}

// One '!' command line from the Mac (see scsi.c "ID 7 LIVE TARGET").
extern "C" void forgeLinkTarget7Line(const char *l)
{
    if (l[0] == 'I')
    {
        g_fl7_inq_len = (uint8_t)fl_parse_hex(l + 1, g_fl7_inq, 36);
        if (g_fl7_inq_len) forgeLinkLogHex("FL7 INQUIRY set", g_fl7_inq, g_fl7_inq_len);
        else logmsg("FL7 INQUIRY cleared: rotor answers again");
    }
    else if (l[0] == 'O')
    {
        char *e; long op = strtol(l + 1, &e, 16);
        while (*e == ' ') e++;
        char mode = *e ? *e : 'g';
        long n = (*e) ? strtol(e + 1, NULL, 0) : 0;
        if (op >= 0 && op < 256) { g_fl7_mode[op] = (uint8_t)mode; g_fl7_len[op] = (uint16_t)n; }
        logmsg("FL7 opcode ", (uint8_t)op, " -> ", mode, " len ", (int)n);
    }
    else if (l[0] == 'D')
    {
        g_fl7_din_len = (uint16_t)fl_parse_hex(l + 1, g_fl7_din, 1024);
        logmsg("FL7 DATA IN queued: ", (int)g_fl7_din_len, " bytes");
    }
    else if (l[0] == 'R')
    {
        // L1b: a reset also disarms any serve (a no-op when none is armed), and a started one's
        // queued SDATA goes with it (C56).
        fl7s_disarm(&g_fl7s, &g_fl7_out_len, &g_fl7_out_pos);
        g_fl7_inq_len = 0; g_fl7_din_len = 0;
        // Back to the BOOT table, not all-zero: zeroing deleted 0x0C = 'o', so the next SEND
        // skipped its DATA OUT and hung the bus (C63).
        fl7_opcode_defaults(g_fl7_mode, g_fl7_len);
        logmsg("FL7 reset to defaults");
    }
    else if (l[0] == 'E')          // call ID 6 in <ms> and try to enable the link remotely
    {
        long n = strtol(l + 1, NULL, 0);
        forgeLinkRemoteEnableIn((uint32_t)(n > 0 ? n : 50));
    }
    else if (l[0] == 'X')          // chase the NEXT session close by <ms> (0 = off)
    {
        long n = strtol(l + 1, NULL, 0);
        forgeLinkChaseClose((uint32_t)(n >= 0 ? n : 0));
    }
    else if (l[0] == 'C')          // bytes offered per poll (flow control for its 256-byte ring)
    {
        long n = strtol(l + 1, NULL, 0);
        g_fl7_chunk = (uint16_t)((n >= 0 && n <= 8192) ? n : 128);
        logmsg("FL7 chunk = ", (int)g_fl7_chunk, " bytes per poll");
    }
    else if (l[0] == 'A')          // append to the mailbox, for payloads bigger than one line
    {
        if (g_fl7_out_len < sizeof(g_fl7_out))
        {
            uint16_t n = (uint16_t)fl_parse_hex(l + 1, g_fl7_out + g_fl7_out_len,
                                                sizeof(g_fl7_out) - g_fl7_out_len);
            g_fl7_out_len += n; g_fl7_mail = 1;
            logmsg("FL7 MAILBOX +", (int)n, " = ", (int)g_fl7_out_len, " bytes");
        }
    }
    else if (l[0] == 'M')
    {
        uint16_t n = (uint16_t)fl_parse_hex(l + 1, g_fl7_out, sizeof(g_fl7_out));
        g_fl7_out_len = n; g_fl7_out_pos = 0; g_fl7_mail = 1;
        if (n) forgeLinkLogHex("FL7 MAILBOX queued", g_fl7_out, n);
        else logmsg("FL7 MAILBOX armed and empty (STATUS will answer 00 00 00)");
    }
    else if (l[0] == 'T')          // set the probe target id, then knock
    {
        long n = strtol(l + 1, NULL, 0);
        g_fl_probe_id = (uint8_t)((n >= 0 && n <= 7) ? n : 6);
        logmsg("FL7 PROBE target = ID ", (int)g_fl_probe_id);
        forgeLinkRemoteEnableIn(100);
    }
    else if (l[0] == 'S')
    {
        long k = strtol(l + 1, NULL, 0);
        g_fl_sweep_kind = (uint8_t)((k >= 0 && k <= 3) ? k : 0);
        g_fl_sweep_i = 0;
        memset(g_fl_sweep_tries, 0, sizeof(g_fl_sweep_tries));
        static const char *names[] = {"off", "byte 1", "byte 5", "byte 6"};
        logmsg("FL7 SWEEP ", names[g_fl_sweep_kind & 3], " armed, step reset");
    }
    else if (l[0] == 'B')          // L1b: stage audio bytes (dense hex) at an offset
    {
        fl7s_stage("audio", l + 1, g_fl7s.audio, g_fl7s.cap, &g_fl7s.audio_len);
    }
    else if (l[0] == 'H')          // L1b: stage the SDATA header (dense hex) at an offset
    {
        fl7s_stage("header", l + 1, g_fl7s.sdata, FL7S_SDATA_LEN, &g_fl7s.sdata_len);
    }
    else if (l[0] == 'L')          // L1b: bytes to serve (0 = what is staged; more repeats it)
    {
        long n = strtol(l + 1, NULL, 0);
        if (g_fl7s.state != FL7S_OFF && g_fl7s.state != FL7S_DONE)
            logmsg("FL7 SERVE total REFUSED: a serve is armed (!W 0 first)");
        else if (n >= 0 && n <= 0xFFFFFE && !(n & 1)) { g_fl7s.total = (uint32_t)n; g_fl7s.state = FL7S_OFF; }
        logmsg("FL7 SERVE total = ", (int)fl7s_total(&g_fl7s), " bytes");
    }
    else if (l[0] == 'P')          // L1b: bytes offered per BULK 0D (0 = the whole residual)
    {
        long n = strtol(l + 1, NULL, 0);
        if (n >= 0 && n <= 0xFFFFFE) g_fl7s.chunk = (uint32_t)n & ~1u;   // even: whole words
        logmsg("FL7 SERVE chunk = ", (int)g_fl7s.chunk, " bytes per bulk poll (0 = whole residual)");
    }
    else if (l[0] == 'V')          // L1b: what is staged, its sums, and the last serve's tally
    {
        fl7s_report();
    }
    else if (l[0] == 'W')          // L1b: !W 1 arm serve-on-request, !W 0 disarm
    {
        long on = strtol(l + 1, NULL, 0);
        if (!on)
        {
            if (g_fl7s.state == FL7S_HEADER || g_fl7s.state == FL7S_FILL)
                logmsg("FL7 SERVE WARNING: disarmed MID-SERVE at byte ", (int)g_fl7s.pos,
                       " - the S1000 will time out the fill (bounded, W39)");
            // A started serve's queued SDATA (or DONE's lone F7) goes with it (C56).
            fl7s_disarm(&g_fl7s, &g_fl7_out_len, &g_fl7_out_pos);
            logmsg("FL7 SERVE disarmed");
            return;
        }
        int rc = fl7s_arm(&g_fl7s, g_fl7_out_len);
        if (rc)
            logmsg("FL7 SERVE ARM REFUSED: code ", rc, rc <= 6 ? " (header check)" : rc == 7 ?
                   " (header length x2 != serve total)" : rc == 8 ? " (mailbox not empty: !M to empty it)" :
                   " (no audio, or an odd byte count)");
        else
            logmsg("FL7 SERVE ARMED: ", (int)g_fl7s.residual, " bytes from ", (int)g_fl7s.audio_len,
                   " staged, header sum ", (int)fl7s_sum(0, g_fl7s.sdata, g_fl7s.sdata_len), ", audio sum ",
                   (int)fl7s_sum(0, g_fl7s.audio, g_fl7s.audio_len), ", chunk ", (int)g_fl7s.chunk,
                   ". Now SAY NOTHING to the board until the session ends.");
    }
    else logmsg("FL7 unknown command: ", l);
}

extern "C" void forgeLinkLogBulk(uint32_t bytes, uint32_t ms, uint32_t sum, const uint8_t *head, int headn, int parityError)
{
    // Log small transfers IN FULL. A 56-byte sample list truncated at 32 bytes showed one name
    // out of four and sent us back to the bench for bytes we already had (2026-09-19).
    char hex[3 * 256 + 1]; int p = 0;
    for (int i = 0; i < headn && i < 256; i++) p += snprintf(hex + p, sizeof(hex) - p, "%02X ", head[i]);
    // KB/s is reported only when the transfer lasted long enough to mean anything: a
    // sub-10 ms move is all overhead and a rate computed from it is an invented number.
    if (ms >= 10)
        logmsg("FL7 BULK ", (int)bytes, " bytes in ", (int)ms, " ms = ", (int)((bytes * 1000ULL) / (ms * 1024ULL)),
               " KB/s, sum ", (int)sum, parityError ? ", PARITY ERROR" : "", "; head ", hex);
    else
        logmsg("FL7 BULK ", (int)bytes, " bytes in ", (int)ms, " ms (too short to rate), sum ", (int)sum,
               parityError ? ", PARITY ERROR" : "", "; head ", hex);
}

extern "C" void forgeLinkLogSweep(int kind, int i, int val, const uint8_t *b)
{
    char hex[3 * 7 + 1]; int p = 0;
    for (int n = 0; n < 7; n++) p += snprintf(hex + p, sizeof(hex) - p, "%02X ", b[n]);
    static const char *names[] = {"off", "b1", "b5", "b6"};
    logmsg("SWEEP ", names[kind & 3], " step ", i, " = ", (uint8_t)val, " -> ", hex);
}

extern "C" void forgeLinkLogRotor(int k, int tur, int len, const uint8_t *b)
{
    char hex[3 * 8 + 1]; int p = 0;
    for (int i = 0; i < len && i < 8; i++) p += snprintf(hex + p, sizeof(hex) - p, "%02X ", b[i]);
    static const char *turs[] = {"GOOD", "CHECK NOT READY", "CHECK UNIT ATTENTION", "BUSY"};
    logmsg("ROTOR ", k, ": TUR -> ", turs[tur & 3], "; INQUIRY body (", len, " bytes) ", hex);
}

extern "C" uint16_t g_fl_rotor_attempts[64];
extern "C" uint8_t g_fl_prize_op[16], g_fl_prize_step[16], g_fl_prize_n;
extern "C" void forgeLinkRotorReport(void)
{
    char line[160]; int p = 0, total = 0, tried = 0;
    for (int k = 0; k < 64; k++) { total += g_fl_rotor_attempts[k]; if (g_fl_rotor_attempts[k]) tried++; }
    logmsg("ROTOR REPORT: ", tried, " of 64 answers tried, ", total, " attempts; prize commands: ", (int)g_fl_prize_n);
    for (int k = 0; k < 64; k++)
    {
        p += snprintf(line + p, sizeof(line) - p, "%02X:%u ", k, g_fl_rotor_attempts[k]);
        if ((k % 16) == 15) { logmsg("ROTOR REPORT ", line); p = 0; }
    }
    for (int i = 0; i < g_fl_prize_n; i++)
        logmsg("ROTOR PRIZE: opcode ", g_fl_prize_op[i], " at step ", (int)g_fl_prize_step[i]);
}

static const char *fl_phase_name(int p)
{
    switch (p)
    {
        case DATA_OUT: return "DATA_OUT";
        case DATA_IN: return "DATA_IN";
        case COMMAND: return "COMMAND";
        case STATUS: return "STATUS";
        case MESSAGE_OUT: return "MESSAGE_OUT";
        case MESSAGE_IN: return "MESSAGE_IN";
        case BUS_FREE: return "BUS_FREE";
        case BUS_BUSY: return "BUS_BUSY";
        default: return "?";
    }
}

// One host transaction, no ATN, no IDENTIFY. Returns bytes received, or -1 if no select.
static int fl_cb_command(int target, const uint8_t *cdb, int cdbLen, uint8_t *in, int inAlloc, uint8_t *status)
{
    *status = 0xFF;
    if (!scsiHostPhySelect(target, 7))
    {
        scsiHostPhyRelease();
        return -1;
    }
    int received = 0, lastp = 100, n = 0;
    uint32_t start = millis();
    int phase;
    while ((phase = scsiHostPhyGetPhase()) != BUS_FREE)
    {
        platform_reset_watchdog();
        if (phase != lastp && n < 20)
        {
            logmsg("Forge Link CB: +", (int)(millis() - start), " ms phase ", fl_phase_name(phase),
                   " REQ ", (int)SCSI_IN(REQ));
            lastp = phase; n++;
        }
        if ((uint32_t)(millis() - start) > g_fl_wait_ms) { logmsg("Forge Link CB: timeout after ", (int)g_fl_wait_ms, " ms"); break; }
        if (phase == COMMAND) scsiHostWrite(cdb, cdbLen);
        else if (phase == DATA_IN)
        {
            if (received >= inAlloc) { logmsg("Forge Link CB: target sends more than allocated"); break; }
            uint32_t r = scsiHostRead(in + received, inAlloc - received);
            if (r == 0) break;
            received += r;
        }
        else if (phase == STATUS) scsiHostRead(status, 1);
        else if (phase == MESSAGE_IN)
        {
            uint8_t m = 0xFF; scsiHostRead(&m, 1);
            logmsg("Forge Link CB: MESSAGE IN ", m);
            if (m == 0x00) break;
        }
        else if (phase == MESSAGE_OUT)
        {
            uint8_t nop = 0x08; scsiHostWrite(&nop, 1);
        }
        else if (phase == DATA_OUT)
        {
            logmsg("Forge Link CB: target asked for DATA_OUT; not sending anything");
            break;
        }
    }
    scsiHostWaitBusFree();
    return received;
}

static void fl_hexlog(const char *what, const uint8_t *d, int n)
{
    char line[3 * 32 + 1];
    for (int off = 0; off < n; off += 32)
    {
        int k = (n - off) < 32 ? (n - off) : 32, p = 0;
        for (int i = 0; i < k; i++) p += snprintf(line + p, sizeof(line) - p, "%02X ", d[off + i]);
        logmsg("Forge Link CB ", what, " +", off, ": ", line);
    }
}


// ---- REMOTE-ENABLE PROBE (2026-09-19, designed from the ROM read) ----
// ⛔ THE CYCLE THIS BREAKS. A failed selection leaves the S1000's SPC holding BSY, because its
// hardware answers selection but its firmware never services the result. The only way off that
// is a bus RESET - and per the ROM read, a reset sets RESET CONDITION in the SPC's INTS, while
// the service routine at 0F5E3E compares INTS against 80h EXACTLY rather than testing the bit.
// So a reset followed immediately by another selection hands the machine the very flag that
// makes it bail. Every earlier attempt re-selected 2 ms after its own reset.
// The fix is a PAUSE: the S1000 polls its target side from the first instruction of its main
// loop (0E6D21 -> 0F5DEC), and the failing branch CLEARS INTS on its way out. Give that loop
// time to run and the next selection should find INTS clean.
// Returns: 1 = reached COMMAND phase (the door opened), 0 = answered BSY but never serviced,
//         -1 = never got on the bus at all.
static int fl_probe_once(int target, const uint8_t *cdb, int cdbLen, uint8_t *in, int inAlloc,
                         uint8_t *status, uint32_t phase_ms)
{
    *status = 0xFF;
    if (!scsiHostPhySelect(target, 7)) { scsiHostPhyRelease(); return -1; }
    int received = 0, reached = 0;
    uint32_t start = millis();
    int phase;
    while ((phase = scsiHostPhyGetPhase()) != BUS_FREE)
    {
        platform_reset_watchdog();
        if ((uint32_t)(millis() - start) > phase_ms) break;
        if (phase == COMMAND)        { reached = 1; scsiHostWrite(cdb, cdbLen); }
        else if (phase == DATA_IN)   { if (received >= inAlloc) break;
                                       uint32_t r = scsiHostRead(in + received, inAlloc - received);
                                       if (!r) break; received += r; }
        else if (phase == STATUS)    scsiHostRead(status, 1);
        else if (phase == MESSAGE_IN){ uint8_t m = 0xFF; scsiHostRead(&m, 1); if (m == 0) break; }
        else if (phase == MESSAGE_OUT){ uint8_t nop = 0x08; scsiHostWrite(&nop, 1); }
    }
    // Bounded BSY wait - NOT the platform's 10 s one, which is most of what this probe used to cost.
    SCSI_RELEASE_OUTPUTS();
    uint32_t t0 = millis();
    while (SCSI_IN(BSY) && (uint32_t)(millis() - t0) < 600) platform_reset_watchdog();
    if (SCSI_IN(BSY)) { logmsg("Forge Link PROBE: BSY still held - one reset"); scsiHostPhyReset(); }
    scsiHostPhyRelease();
    return reached ? 1 : (received || *status != 0xFF) ? 1 : 0;
}

extern "C" void forgeLinkCallbackPoll(void)
{
    forgeLinkInjectPoll();
    if (g_fl_fire_at && (int32_t)(millis() - g_fl_fire_at) >= 0)
    {
        g_fl_fire_at = 0; g_fl_armed = true; g_fl_last_inq_ms = millis() - 1000;
    }
    if (!g_fl_armed) return;
    if ((uint32_t)(millis() - g_fl_last_inq_ms) < 30) return;
    if (scsiDev.phase != BUS_FREE || SCSI_IN(BSY) || SCSI_IN(SEL)) return;
    g_fl_armed = false;

    logmsg("Forge Link CB: calling ID ", (int)g_fl_probe_id);
    // Target -> host, WITHOUT the RST pulse scsiHostPhyReset() would send.
    gpio_set_irq_enabled(SCSI_IN_BSY, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(SCSI_IN_SEL, GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(SCSI_IN_RST, GPIO_IRQ_EDGE_FALL, false);
    SCSI_RELEASE_OUTPUTS();
    SCSI_ENABLE_INITIATOR();
    scsi_accel_host_init();

    static uint8_t buf[1024];
    uint8_t st;
    // REMOTE ENABLE (2026-09-19). The S1000's ROM answers these commands as a TARGET, and its
    // handler for 09 writes the received byte straight into its own "MIDI via SCSI" setting -
    // so a host can turn the link on without anyone touching the front panel. Measured
    // counter-fact: at idle it answers selection in hardware but never raises REQ, so its
    // target side is only serviced inside particular firmware windows. THIS IS THE SEARCH FOR
    // THAT WINDOW. INQUIRY first, because its answer tells us the door is open at all: the
    // S1000's own template is C7 00 00 00 04 00 48 + a version word.
    // THE TEST: select up to FL_SELECT_TRIES times, with a real pause after each reset so the
    // S1000's main loop can clear INTS before we knock again. TEST UNIT READY needs no session
    // and no "MIDI via SCSI" setting (ROM: 00h/03h/12h are answered unconditionally), so this
    // works with his DIN sockets alive - which is the whole point.
    const uint8_t tur[6] = {0x00, 0, 0, 0, 0, 0};
    const uint8_t inq[6] = {0x12, 0, 0, 0, 9, 0};
    int opened = 0;
    for (int attempt = 1; attempt <= FL_SELECT_TRIES; attempt++)
    {
        uint32_t a0 = millis();
        int r = fl_probe_once(g_fl_probe_id, tur, 6, buf, 0, &st, 250);
        logmsg("Forge Link PROBE: try ", attempt, "/", (int)FL_SELECT_TRIES,
               " TEST UNIT READY -> ",
               (r == 1) ? "SERVICED" : (r == 0) ? "answered BSY, never serviced" : "never got on the bus",
               "  status ", st, "  ", (int)(millis() - a0), " ms");
        if (r == 1) { opened = 1; break; }
        delay(FL_SETTLE_MS);        // let its idle loop run and clear INTS
    }
    int n = -1;
    if (opened)
    {
        logmsg("Forge Link PROBE: *** THE DOOR OPENED *** - asking who it is");
        n = fl_probe_once(g_fl_probe_id, inq, 6, buf, 9, &st, 250);
        if (n > 0) fl_hexlog("INQUIRY", buf, 9);
    }
    else logmsg("Forge Link PROBE: no service in ", (int)FL_SELECT_TRIES,
                " tries with ", (int)FL_SETTLE_MS, " ms settle");

    // Host -> target. ⛔ THIS USED TO BE INCOMPLETE AND IT COST THREE BENCH SESSIONS: releasing
    // the outputs and calling scsiPhyReset() does NOT restore the output enables that
    // SCSI_ENABLE_INITIATOR() cleared on IO/CD/MSG/REQ, so the board could never assert REQ
    // again and every later command hung the bus until reboot. forgeLinkReenterTargetRole()
    // re-runs the target pin configuration for exactly those pins and then resets the PHY.
    SCSI_RELEASE_OUTPUTS();
    sio_hw->gpio_oe_clr = (1 << SCSI_OUT_ACK) | (1 << SCSI_OUT_ATN);
    forgeLinkReenterTargetRole();
    g_fl_wait_ms = 3000;
    logmsg("Forge Link CB: back in target role");
}

#else

extern "C" void forgeLinkNoteCommand(unsigned char, unsigned char) {}
extern "C" void forgeLinkLogRotor(int, int, int, const unsigned char *) {}
extern "C" void forgeLinkLogHex(const char *, const unsigned char *, int) {}
extern "C" void forgeLinkTarget7Line(const char *) {}
extern "C" void forgeLinkCallbackPoll(void) { forgeLinkInjectPoll(); }
extern "C" void forgeLinkCallbackArm(void) {}
extern "C" void forgeLinkRemoteEnableIn(unsigned int) {}
extern "C" void forgeLinkChaseClose(unsigned int) {}
extern "C" void forgeLinkSessionClosed(void) {}
extern "C" void forgeLinkWriteReport(void) {}
extern "C" void forgeLinkRotorReport(void) {}

#endif
