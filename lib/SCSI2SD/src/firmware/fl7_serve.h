/**
 * Forge Link - a fork of ZuluSCSI-firmware by Rabbit Hole Computing.
 * Modifications Copyright (c) 2026 the Forge Link contributors.
 * GNU General Public License v3 or later; see LICENSE.
**/

// ── L1b SERVE-ON-REQUEST: THE STATE MACHINE, WITH NO HARDWARE IN IT ──────────────────────────
// Everything the serve path DECIDES lives here, so the same code runs in the firmware (scsi.c
// fl7_live, TARGET7 build only) and on the Mac, where tools/tests/fl7_serve_sim.c wraps it and
// a scripted S1000 built from the 2026-09-21 captures drives it (tools/fl_serve.py --fake, and
// tools/tests/test_fl7_serve.py - including mutation tests of the guards).
// The firmware keeps only the bus work: phases, scsiWrite, the clock and the log.
//
// The contract:
//   request   OPTION held + SEND; link on 09 00 01 01; RSDATA F0 47 ch 0A 48 lo hi F7 on a
//             NON-bulk 0C; then non-bulk 0D polls every ~8 ms for ~20.5 s (MEASURED, W39)
//   answer    ONE SDATA F0 47 ch 0B 48 lo hi <300 nibbles> F7, 128 bytes per poll
//   then      it may send ACK/RSPACK on non-bulk 0C (taken, never answered - no ASPACK)
//   fill      BULK 0D (CDB[5] bit 7) -> a 24-bit EVEN count <= its residual (length x 2)
//             BULK 0E (CDB[5] bit 7) -> DATA IN of exactly CDB[2..4] bytes of raw LE words
//   end       residual 0 -> commit -> 09 00 00 00 00 00
// ⛔ AUDIO NEVER MOVES AS SysEx (the owner's ruling). SysEx carries the header only; the audio
// goes ONLY through the bulk data phase. An SDS dump request (F0 7E ch 03, protocol STANDARD)
// is logged and NEVER answered - that carrier is struck.
#ifndef FL7_SERVE_H
#define FL7_SERVE_H

#include <stdint.h>
#include <string.h>

#define FL7S_SDATA_LEN     308u   // 7 preamble + 150 header bytes nibbled (300) + F7
#define FL7S_HDR_LEN_OFF    26u   // SLNGTH, u32 LE, in WORDS (W38: 13230 words = 26,460 bytes)
#define FL7S_PACKET        128u   // non-bulk offer cap while armed (bridge-operation §3 rule 1)

enum { FL7S_OFF = 0, FL7S_ARMED, FL7S_HEADER, FL7S_FILL, FL7S_DONE };

// Events handed to the log. The firmware prints them; the simulator records them.
enum {
    FL7S_EV_NONE = 0,
    FL7S_EV_SDATA_QUEUED,     // a = sample index, b = bytes
    FL7S_EV_RSDATA_IGNORED,   // a = sample index, b = state
    FL7S_EV_HEADER_DONE,      // a = bytes handed over
    FL7S_EV_RSPACK,           // a = offset in the payload
    FL7S_EV_BEGIN,            // a = first count
    FL7S_EV_END,              // a = bytes, b = ms, c = sum
    FL7S_EV_RESET,            // a = byte position
    FL7S_EV_CLOSED,           // a = state at close, b = bytes served, c = residual
    FL7S_EV_SDS_REFUSED,      // an SDS dump request - struck carrier, not answered
    FL7S_EV_END_OF_LIST,      // a = sample index answered with a lone F7
    FL7S_EV_ACK,              // a non-bulk 0C that is neither RSDATA nor RSPACK (ACK etc.)
};

typedef struct {
    uint8_t  state;
    uint8_t *audio; uint32_t cap;       // the staging buffer, in board RAM
    uint32_t audio_len;                 // bytes staged
    uint32_t total;                     // bytes to serve; 0 = audio_len; more REPEATS the buffer
    uint8_t  sdata[FL7S_SDATA_LEN]; uint32_t sdata_len;
    uint32_t chunk;                     // bytes offered per bulk 0D; 0 = the whole residual
    uint32_t residual, pos, sum, promised, maxcount;
    uint32_t cycles, pad, warn, zero_polls, t0, t1;
} fl7s_t;

static inline uint32_t fl7s_total(const fl7s_t *s) { return s->total ? s->total : s->audio_len; }

// The integrity sum the Mac checks before arming (same fold as forgeLinkLogBulk's).
static inline uint32_t fl7s_sum(uint32_t sum, const uint8_t *d, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) sum = (sum << 1) + (sum >> 31) + d[i];
    return sum;
}

// ⛔ THE COUNT GUARD. The S1000 checks the count against its residual; a count past it is a
// violation whose handling is unread. So the offer is bounded by what we still hold, by what it
// still expects, by the chunk, by the 24 bits the answer has, and it is EVEN (whole words).
static inline uint32_t fl7s_bulk_count(uint32_t staged_left, uint32_t residual_left, uint32_t chunk)
{
    uint32_t n = staged_left < residual_left ? staged_left : residual_left;
    if (chunk && n > chunk) n = chunk;
    if (n > 0xFFFFFEu) n = 0xFFFFFEu;
    return n & ~1u;
}

// ⛔ THE PHASE GUARD. A bulk 0E is answered with EXACTLY `want` bytes: `real` from staging,
// never past what is left, and `pad` zeros for any shortfall; real + pad == want, always. A
// short data phase hangs the bus until the watchdog (bridge-operation §3 rule 3).
static inline void fl7s_plan_0e(uint32_t want, uint32_t staged_left, uint32_t *real, uint32_t *pad)
{
    *real = want < staged_left ? want : staged_left;
    *pad = want - *real;
}

// The non-bulk mailbox offer (bridge-operation §3 rules 1 and 2): at most `chunk` bytes, and
// trimmed back to the last F7 inside that window so each handover ends on a complete message.
// No F7 in the window (a message longer than a packet, e.g. the 308-byte SDATA) -> offer the
// window as-is; the rest of that message is already queued behind it.
static inline uint32_t fl7s_offer(const uint8_t *mb, uint32_t left, uint32_t chunk)
{
    if (chunk && left > chunk) left = chunk;
    uint32_t end = 0;
    for (uint32_t i = 0; i < left; i++) if (mb[i] == 0xF7) end = i + 1;
    return end ? end : left;
}
// The chunk the non-bulk path uses: !C's value, except that while a serve is armed it is never
// above one packet, whatever !C says.
static inline uint32_t fl7s_nonbulk_chunk(const fl7s_t *s, uint32_t console_chunk)
{
    if (s->state != FL7S_OFF && (console_chunk == 0 || console_chunk > FL7S_PACKET)) return FL7S_PACKET;
    return console_chunk;
}

static inline uint8_t fl7s_hdr_byte(const uint8_t *sd, uint32_t k)
{
    return (uint8_t)((sd[7 + 2 * k] & 0x0F) | ((sd[8 + 2 * k] & 0x0F) << 4));
}

// Validate a staged SDATA and return the residual it will arm, in BYTES. 0 = OK.
//   1 wrong length  2 not F0 47 .. 0B 48  3 no F7 at the end  4 a data byte >= 0x80 inside
//   5 not a sample header (SHIDENT != 3)  6 zero length
static inline int fl7s_check_sdata(const uint8_t *sd, uint32_t n, uint32_t *residual)
{
    *residual = 0;
    if (n != FL7S_SDATA_LEN) return 1;
    if (sd[0] != 0xF0 || sd[1] != 0x47 || sd[3] != 0x0B || sd[4] != 0x48) return 2;
    if (sd[n - 1] != 0xF7) return 3;
    for (uint32_t i = 1; i + 1 < n; i++) if (sd[i] & 0x80) return 4;
    if (fl7s_hdr_byte(sd, 0) != 3) return 5;
    uint32_t words = (uint32_t)fl7s_hdr_byte(sd, FL7S_HDR_LEN_OFF)
                   | ((uint32_t)fl7s_hdr_byte(sd, FL7S_HDR_LEN_OFF + 1) << 8)
                   | ((uint32_t)fl7s_hdr_byte(sd, FL7S_HDR_LEN_OFF + 2) << 16)
                   | ((uint32_t)fl7s_hdr_byte(sd, FL7S_HDR_LEN_OFF + 3) << 24);
    if (!words) return 6;
    *residual = words * 2u;
    return 0;
}

// Arm. 0 = armed; 1..6 = the header check above; 7 = header length != bytes to serve;
// 8 = the mailbox is not empty (the first complete SysEx back MUST be the SDATA);
// 9 = nothing staged, or an odd byte count.
static inline int fl7s_arm(fl7s_t *s, uint32_t mailbox_len)
{
    uint32_t resid = 0;
    int chk = fl7s_check_sdata(s->sdata, s->sdata_len, &resid);
    if (chk) return chk;
    if (!s->audio_len || (s->audio_len & 1) || (fl7s_total(s) & 1)) return 9;
    if (resid != fl7s_total(s)) return 7;
    if (mailbox_len) return 8;
    s->residual = resid;
    s->pos = s->sum = s->promised = s->maxcount = 0;
    s->cycles = s->pad = s->warn = s->zero_polls = s->t0 = s->t1 = 0;
    s->state = FL7S_ARMED;
    return 0;
}

// Find an Akai S1000 SysEx of function `fn` (F0 47 ch fn 48 ...). Offset, or -1.
static inline int fl7s_find_akai(const uint8_t *b, uint32_t n, uint8_t fn)
{
    for (uint32_t i = 0; i + 4 < n; i++)
        if (b[i] == 0xF0 && b[i + 1] == 0x47 && b[i + 3] == fn && b[i + 4] == 0x48) return (int)i;
    return -1;
}

// RSDATA = F0 47 ch 0A 48 idx_lo idx_hi F7 (W39: TWO index bytes). 1 if found.
static inline int fl7s_parse_rsdata(const uint8_t *b, uint32_t n, uint8_t *ch, uint8_t *lo, uint8_t *hi)
{
    int at = fl7s_find_akai(b, n, 0x0A);
    if (at < 0 || (uint32_t)at + 8 > n || b[at + 7] != 0xF7) return 0;
    *ch = b[at + 2]; *lo = b[at + 5]; *hi = b[at + 6];
    return 1;
}

// SDS DUMP REQUEST = F0 7E ch 03 ... (what protocol STANDARD sends instead of RSDATA).
static inline int fl7s_is_sds_request(const uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i + 3 < n; i++)
        if (b[i] == 0xF0 && b[i + 1] == 0x7E && b[i + 3] == 0x03) return 1;
    return 0;
}

// A NON-bulk 0C payload while a serve is on. Mailbox = the firmware's outbound queue.
// Returns an FL7S_EV_*; *arg carries the sample index where there is one.
static inline int fl7s_on_0c(fl7s_t *s, const uint8_t *b, uint32_t n,
                             uint8_t *mbox, uint16_t *mlen, uint16_t *mpos, uint32_t *arg)
{
    uint8_t ch, lo, hi;
    *arg = 0;
    if (s->state == FL7S_OFF) return FL7S_EV_NONE;
    if (fl7s_is_sds_request(b, n)) return FL7S_EV_SDS_REFUSED;       // never answered
    if (fl7s_parse_rsdata(b, n, &ch, &lo, &hi))
    {
        *arg = (uint32_t)lo | ((uint32_t)hi << 7);
        if (s->state == FL7S_ARMED && *mlen == 0)
        {
            // Answer AS the peer: the requested channel and index go back in the reply.
            memcpy(mbox, s->sdata, FL7S_SDATA_LEN);
            mbox[2] = ch; mbox[5] = lo; mbox[6] = hi;
            *mpos = 0; *mlen = (uint16_t)FL7S_SDATA_LEN;
            s->state = FL7S_HEADER;
            return FL7S_EV_SDATA_QUEUED;
        }
        if (s->state == FL7S_DONE && *mlen == 0)
        {
            // One serve per arm. The next request of an ALL SAMPLES run is ended with a lone
            // F7: any complete non-SDATA message ends its loop cleanly (0x0F790-0x0F797).
            mbox[0] = 0xF7; *mpos = 0; *mlen = 1;
            return FL7S_EV_END_OF_LIST;
        }
        return FL7S_EV_RSDATA_IGNORED;
    }
    int at = fl7s_find_akai(b, n, 0x0C);
    if (at >= 0)
    {
        if (s->state == FL7S_HEADER) s->state = FL7S_FILL;
        *arg = (uint32_t)at;
        return FL7S_EV_RSPACK;
    }
    return FL7S_EV_ACK;
}

// After a non-bulk 0E handed over `n` bytes and the mailbox drained. 1 = the SDATA is now whole.
static inline int fl7s_on_mailbox_drained(const fl7s_t *s, uint32_t n)
{
    return s->state == FL7S_HEADER && n != 0;
}

// Audio only once the header has been handed over WHOLE.
static inline int fl7s_ready(const fl7s_t *s, uint32_t mailbox_len)
{
    return (s->state == FL7S_HEADER || s->state == FL7S_FILL) && mailbox_len == 0;
}

// BULK 0D. Returns the count to answer; *begin set when this is the first non-zero offer.
static inline uint32_t fl7s_on_bulk0d(fl7s_t *s, uint32_t mailbox_len, uint32_t now_ms, int *begin)
{
    uint32_t total = fl7s_total(s);
    uint32_t staged_left = total > s->pos ? total - s->pos : 0;
    uint32_t residual_left = s->residual > s->pos ? s->residual - s->pos : 0;
    uint32_t n = fl7s_ready(s, mailbox_len) ? fl7s_bulk_count(staged_left, residual_left, s->chunk) : 0;
    *begin = 0;
    if (n && !s->t0) { s->t0 = now_ms ? now_ms : 1; *begin = 1; }
    if (!n) s->zero_polls++;
    if (n > s->maxcount) s->maxcount = n;
    s->promised = n;
    return n;
}

// BULK 0E: how many real bytes and how many zero-pad bytes make exactly `want`.
static inline void fl7s_plan_bulk0e(fl7s_t *s, uint32_t want, uint32_t mailbox_len, uint32_t *real, uint32_t *pad)
{
    uint32_t total = fl7s_total(s);
    uint32_t left = total > s->pos ? total - s->pos : 0;
    uint32_t resid = s->residual > s->pos ? s->residual - s->pos : 0;
    if (resid < left) left = resid;
    fl7s_plan_0e(want, fl7s_ready(s, mailbox_len) ? left : 0, real, pad);
    if (want != s->promised) s->warn++;
}

// Where the serve byte at `pos` lives in the staging buffer, and how many run on contiguously
// (a total larger than the buffer repeats it - a 64 KiB ramp period served as 256 KiB).
static inline const uint8_t *fl7s_src(const fl7s_t *s, uint32_t pos, uint32_t maxk, uint32_t *k)
{
    uint32_t off = pos % s->audio_len;
    uint32_t run = s->audio_len - off;
    *k = run < maxk ? run : maxk;
    return s->audio + off;
}

// After a bulk 0E's data phase. Returns FL7S_EV_END when the last staged byte has gone,
// FL7S_EV_RESET if the bus was reset mid-phase, else NONE.
static inline int fl7s_after_bulk0e(fl7s_t *s, uint32_t real, uint32_t pad, int reset, uint32_t now_ms)
{
    s->promised = 0;
    s->cycles++;
    s->pad += pad;
    if (reset) return FL7S_EV_RESET;
    uint32_t p = s->pos, left = real;
    while (left)
    {
        uint32_t k;
        const uint8_t *src = fl7s_src(s, p, left, &k);
        s->sum = fl7s_sum(s->sum, src, k);
        p += k; left -= k;
    }
    s->pos += real;
    if (real && s->pos >= fl7s_total(s))
    {
        s->t1 = now_ms;
        s->state = FL7S_DONE;
        return FL7S_EV_END;
    }
    return FL7S_EV_NONE;
}

// The S1000 closed the session (09, CDB[2] == 0). A serve that STARTED (header queued) ends
// here whatever happened - complete, timed out mid-fill, or abandoned - so no board state is
// left armed and nothing of ours is left in the mailbox to become the first SysEx of the next
// session. A serve still waiting for its RSDATA stays armed (e.g. SEND pressed without OPTION,
// which takes the transmit branch and never asks). Returns the state it closed from.
static inline int fl7s_on_close(fl7s_t *s, uint16_t *mlen, uint16_t *mpos)
{
    int was = s->state;
    if (was == FL7S_HEADER || was == FL7S_FILL || was == FL7S_DONE)
    {
        *mlen = 0; *mpos = 0;
        s->state = FL7S_OFF;
    }
    return was;
}

// May a staging line land at `off`? 0 = yes; 1 = a serve is armed or running (never restage
// under it); 2 = not contiguous (offset 0 restarts; any other offset must equal the current
// length, so a dropped or repeated line is REFUSED instead of leaving a hole the S1000 plays);
// 3 = past the buffer.
static inline int fl7s_stage_ok(const fl7s_t *s, uint32_t off, uint32_t cur_len, uint32_t cap)
{
    if (s->state != FL7S_OFF && s->state != FL7S_DONE) return 1;
    if (off != 0 && off != cur_len) return 2;
    if (off >= cap) return 3;
    return 0;
}

// Dense hex ("0a1B..", no separators) -> bytes. Returns the count, or -1 on any non-hex
// character, an odd digit count or overflow: a half-parsed staging line is REFUSED, never
// stored short, or the Mac's offset for the next line would be wrong.
static inline int fl7s_hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}
static inline int fl7s_parse_dense_hex(const char *p, uint8_t *out, int max)
{
    int n = 0;
    while (*p == ' ') p++;
    while (*p && *p != ' ' && *p != '\r' && *p != '\n')
    {
        int a = fl7s_hexval(p[0]);
        if (a < 0) return -1;
        int b = fl7s_hexval(p[1]);
        if (b < 0) return -1;              // also catches an odd count (p[1] == 0)
        if (n >= max) return -1;
        out[n++] = (uint8_t)((a << 4) | b);
        p += 2;
    }
    return n;
}

#endif
