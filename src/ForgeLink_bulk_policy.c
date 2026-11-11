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

#include "ForgeLink_bulk_policy.h"

// ⛔ COMPILED ONLY INTO BULK BUILDS - AND THE HOST TEST. PlatformIO compiles every file in src/
// for every environment, so without this guard the object is linked into the PROBE and TARGET7
// builds too. Its symbols are garbage-collected out, so `size` and the symbol list still match
// exactly - but its presence REORDERS THE LINKER'S VENEERS, and gate A6 measured 4,828 scattered
// differing bytes in a build that is supposed to be untouched by this flag. An unreferenced
// object is not a free object. FL_BULK_HOST is the test's escape hatch, the same shape as
// ForgeLink_digest.c's FL_DIGEST_HOST.
#if defined(FORGE_LINK_BULK) || defined(FL_BULK_HOST)

#include "ForgeLink_digest.h"

#include <string.h>

// The sync bytes are the frame format's, shared with CDC. Spelled out rather than pulled from
// ForgeLink_probe.h so this file stays free of board headers (see the header's note).
#define FL_BULK_SYNC0 0xF5
#define FL_BULK_SYNC1 0xA7

// sync(2) type(1) seq(1) len(4) payload crc(4)
#define FL_BULK_PAYLOAD_OFF 8
#define FL_BULK_MAX_CB_PAYLOAD (FL_BULK_CB_BYTES - FL_BULK_PAYLOAD_OFF - 4)  /* 52 */

static uint32_t fl_bulk_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void fl_bulk_put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

fl_cb_result_t flBulkParseCb(const uint8_t *buf, uint32_t len, fl_bulk_cb_t *out)
{
    if (!buf || !out) return FL_CB_NOT_A_CB;

    // ⛔ A COMMAND BLOCK IS EXACTLY 64 BYTES, NEVER FEWER AND NEVER A ZERO-LENGTH PACKET.
    // That is the whole reason the host zero-pads: it makes a ZLP unambiguous (it can only be
    // the flush marker of a no-stall reset) and it makes a short transfer a torn write rather
    // than a command to be guessed at.
    if (len != FL_BULK_CB_BYTES) return FL_CB_NOT_A_CB;

    if (buf[0] != FL_BULK_SYNC0 || buf[1] != FL_BULK_SYNC1) return FL_CB_BAD_FRAME;

    uint32_t plen = fl_bulk_get32(&buf[4]);
    if (plen > FL_BULK_MAX_CB_PAYLOAD) return FL_CB_BAD_FRAME;

    uint32_t crc_off = FL_BULK_PAYLOAD_OFF + plen;
    uint32_t crc = flDigestCrc32(0, &buf[2], 6 + plen);
    if (crc != fl_bulk_get32(&buf[crc_off])) return FL_CB_BAD_FRAME;

    // ⛔ THE PADDING IS CHECKED, AND IT IS NOT DECORATION. Everything after the CRC must be zero.
    // Non-zero padding means the host built the block from a reused buffer, which is the exact
    // shape of a stale command being replayed into a fresh epoch.
    for (uint32_t i = crc_off + 4; i < FL_BULK_CB_BYTES; i++)
    {
        if (buf[i] != 0) return FL_CB_BAD_FRAME;
    }

    out->type = buf[2];
    out->seq = buf[3];
    out->len = plen;
    out->payload_off = FL_BULK_PAYLOAD_OFF;
    out->data_bytes = flBulkDataBytesFor(out->type, &buf[FL_BULK_PAYLOAD_OFF], plen);
    out->guard = FL_GUARD_NONE;
    out->head_len = plen;
    out->session = 0;
    out->data_crc = 0;
    const uint8_t *pl = &buf[FL_BULK_PAYLOAD_OFF];

    // The two data-carrying verbs have EXACT payload lengths. A block whose length disagrees is
    // a bad frame, not a request to be interpreted: the data phase's size is read out of it, so
    // a wrong length means the board would consume the wrong number of bytes and desynchronise
    // the pipe for every command after it.
    // ⭐ ARMRETRY r7 · DATACRC: WRITE_BULK is 17 (unguarded) or 21 (+ session u32 at +17); nothing else.
    if (out->type == FL_T_WRITE_BULK)
    {
        if (plen == FL_BULK_WRITE_CB_LEN + FL_DC_TRAILER_SESSION)
        {
            out->guard = FL_GUARD_SESSION;
            out->head_len = FL_BULK_WRITE_CB_LEN;
            out->session = fl_bulk_get32(&pl[FL_BULK_WRITE_CB_LEN]);
        }
        else if (plen != FL_BULK_WRITE_CB_LEN) return FL_CB_BAD_FRAME;
    }
    if (out->type == FL_T_SINK_BULK  && plen != FL_BULK_SINK_CB_LEN)  return FL_CB_BAD_FRAME;
    // ⭐ N6 PROBE-BULK · AN EXEC BLOCK IS EXACTLY ITS FIXED HEADER PLUS ITS CDB. Its data-out, when
    // it has one, is the separate phase, so a block carrying any byte past the CDB's end, or one
    // too short to hold its own cdb_len byte, is a bad frame: the out_len field the data phase is
    // sized from would be read from the wrong offset. A CDB-only EXEC satisfies this unchanged
    // (its out_len is 0), which is what keeps a no-data EXEC a plain one-block request.
    if (out->type == FL_T_EXEC)
    {
        if (plen < FL_BULK_EXEC_CDB_OFF) return FL_CB_BAD_FRAME;
        uint32_t cdb_len = buf[FL_BULK_PAYLOAD_OFF + FL_BULK_EXEC_CDB_OFF - 1];
        // ⭐ ARMRETRY r7 · DATACRC: or exactly the head + the 8-byte trailer (session, data_crc32).
        if (plen == FL_BULK_EXEC_FIXED + cdb_len + FL_DC_TRAILER_FULL)
        {
            out->guard = FL_GUARD_FULL;
            out->head_len = FL_BULK_EXEC_FIXED + cdb_len;
        }
        else if (plen != FL_BULK_EXEC_FIXED + cdb_len) return FL_CB_BAD_FRAME;
    }
#ifdef FL_BULK_NOTEAT
    // TQ2: a NOTE_AT block is exactly its 24-byte head, a RELEASE block its 16; a length field above
    // the cap is a bad block (no reply: the host's timeout and the reset), never a phase to swallow.
    // ARMRETRY r7 · DATACRC: or exactly the head + the 8-byte trailer (NOTE_AT 32, RELEASE 24).
    if (out->type == FL_BULK_T_NOTE_AT &&
        ((plen != FL_BULK_NOTE_AT_HEAD && plen != FL_BULK_NOTE_AT_HEAD + FL_DC_TRAILER_FULL) ||
         fl_bulk_get32(&buf[FL_BULK_PAYLOAD_OFF + 12]) > FL_BULK_NOTE_AT_MAX))
        return FL_CB_BAD_FRAME;
    if (out->type == FL_BULK_T_RELEASE &&
        ((plen != FL_BULK_RELEASE_HEAD && plen != FL_BULK_RELEASE_HEAD + FL_DC_TRAILER_FULL) ||
         fl_bulk_get32(&buf[FL_BULK_PAYLOAD_OFF + 12]) > FL_BULK_RELEASE_MAX))
        return FL_CB_BAD_FRAME;
    if ((out->type == FL_BULK_T_NOTE_AT && plen == FL_BULK_NOTE_AT_HEAD + FL_DC_TRAILER_FULL) ||
        (out->type == FL_BULK_T_RELEASE && plen == FL_BULK_RELEASE_HEAD + FL_DC_TRAILER_FULL))
    {
        out->guard = FL_GUARD_FULL;
        out->head_len = plen - FL_DC_TRAILER_FULL;
    }
#endif
    if (out->guard == FL_GUARD_FULL)
    {
        out->session = fl_bulk_get32(&pl[out->head_len]);
        out->data_crc = fl_bulk_get32(&pl[out->head_len + 4]);
    }

    return FL_CB_OK;
}

uint32_t flBulkDataBytesFor(uint8_t type, const uint8_t *payload, uint32_t len)
{
    // ⛔ THE LENGTH COMES FROM THE COMMAND BLOCK, NEVER FROM THE TRANSFER. The board must know how
    // much the data phase carries BEFORE the bytes arrive, because a REFUSED request still has to
    // swallow exactly its data phase (the discard sink) to stay in step with the host. A board
    // that learned the length from the arriving bytes could not refuse anything without
    // desynchronising the pipe.
    if (!payload) return 0;
    if (type == FL_T_WRITE_BULK)
    {
        if (len < FL_BULK_WRITE_CB_LEN) return 0;
        uint32_t count = (uint32_t)payload[5] | ((uint32_t)payload[6] << 8);   // count u16 at +5
        return count * 512u;
    }
    if (type == FL_T_SINK_BULK)
    {
        if (len < FL_BULK_SINK_CB_LEN) return 0;
        return fl_bulk_get32(payload);
    }
    if (type == FL_T_EXEC)
    {
        // {target, flags, timeout u32, cdb_len, cdb[cdb_len], in_alloc u32, out_len u32}. The
        // data phase is out_len ONLY when the flags say data-out: a block whose flag and length
        // disagree carries no phase here and is refused by the EXEC handler's own check ("data-out
        // flag disagrees with length"), before the bus, exactly as the same frame is over CDC.
        if (len < FL_BULK_EXEC_CDB_OFF) return 0;
        uint32_t cdb_len = payload[FL_BULK_EXEC_CDB_OFF - 1];
        if (len < FL_BULK_EXEC_FIXED + cdb_len) return 0;
        if ((payload[1] & FL_F_DATA_OUT) == 0) return 0;
        return fl_bulk_get32(&payload[FL_BULK_EXEC_CDB_OFF + cdb_len + 4]);   // out_len
    }
#ifdef FL_BULK_NOTEAT
    // TQ2: entries_len (NOTE_AT) / keys_len (RELEASE), u32 at +12 of the head.
    if (type == FL_BULK_T_NOTE_AT)
        return len >= FL_BULK_NOTE_AT_HEAD ? fl_bulk_get32(&payload[12]) : 0;
    if (type == FL_BULK_T_RELEASE)
        return len >= FL_BULK_RELEASE_HEAD ? fl_bulk_get32(&payload[12]) : 0;
#endif
    return 0;
}

bool flBulkDataPhaseAllowed(uint8_t type, uint32_t data_bytes)
{
    if (type == FL_T_EXEC) return data_bytes <= FL_BULK_EXEC_MAX_DATA;
#ifdef FL_BULK_NOTEAT
    if (type == FL_BULK_T_NOTE_AT) return data_bytes <= FL_BULK_NOTE_AT_MAX;
    if (type == FL_BULK_T_RELEASE) return data_bytes <= FL_BULK_RELEASE_MAX;
#endif
    return true;
}

uint32_t flBulkDescriptor(uint8_t *dst, uint32_t cap,
                          uint8_t itf, uint8_t ep_out, uint8_t ep_in, uint8_t str_index)
{
    if (!dst || cap < FL_BULK_DESC_BYTES) return 0;

    uint8_t *d = dst;
    // Interface descriptor (9)
    *d++ = 9;                      // bLength
    *d++ = 0x04;                   // bDescriptorType = INTERFACE
    *d++ = itf;                    // bInterfaceNumber
    *d++ = 0;                      // bAlternateSetting
    *d++ = 2;                      // bNumEndpoints
    *d++ = FL_BULK_ITF_CLASS;
    *d++ = FL_BULK_ITF_SUBCLASS;
    *d++ = FL_BULK_ITF_PROTOCOL;
    *d++ = str_index;              // iInterface

    // Endpoint OUT (7)
    *d++ = 7;
    *d++ = 0x05;                   // ENDPOINT
    *d++ = ep_out;                 // direction bit already clear
    *d++ = 0x02;                   // bulk
    fl_bulk_put16(d, FL_BULK_EP_SIZE); d += 2;
    *d++ = 0;                      // bInterval

    // Endpoint IN (7)
    *d++ = 7;
    *d++ = 0x05;
    *d++ = (uint8_t)(ep_in | 0x80);
    *d++ = 0x02;
    fl_bulk_put16(d, FL_BULK_EP_SIZE); d += 2;
    *d++ = 0;

    return (uint32_t)(d - dst);
}

bool flBulkOpenMatches(uint8_t cls, uint8_t sub, uint8_t proto)
{
    return cls == FL_BULK_ITF_CLASS &&
           sub == FL_BULK_ITF_SUBCLASS &&
           proto == FL_BULK_ITF_PROTOCOL;
}

void flBulkRtInit(fl_bulk_rt_t *rt)
{
    if (!rt) return;
    memset(rt, 0, sizeof(*rt));
    rt->state = FL_BULK_IDLE;
    rt->epoch = 1;   // never 0: 0 means "no epoch"
}

bool flBulkRtAcceptCompletion(fl_bulk_rt_t *rt, uint16_t completion_epoch)
{
    if (!rt) return false;
    if (completion_epoch != rt->epoch)
    {
        // ⛔ IGNORED AND COUNTED, NEVER ACTED ON. After a reset the host and board both know the
        // epoch; a completion from the old one is a transfer that was already in the hardware
        // when the stall landed. Acting on it would apply a command the host has abandoned.
        rt->stale_completions++;
        return false;
    }
    return true;
}

uint16_t flBulkRtNewEpoch(fl_bulk_rt_t *rt)
{
    if (!rt) return 0;
    uint16_t e = (uint16_t)(rt->epoch + 1);
    if (e > FL_BULK_EPOCH_MAX || e == 0) e = 1;
    rt->epoch = e;
    return e;
}

uint32_t flBulkRtNoteRefusedArm(fl_bulk_rt_t *rt)
{
    if (!rt) return 0;
    rt->refused_arms++;
    return rt->refused_arms;
}

bool flBulkControlAllowed(uint8_t b_request, uint16_t w_value)
{
    switch (b_request)
    {
        case FL_REQ_BULK_RESET:
        {
            // wValue is the new epoch, 1..0x7FFF, optionally with the no-stall flush bit.
            uint16_t epoch = (uint16_t)(w_value & ~FL_BULK_NOSTALL_BIT);
            return epoch >= 1 && epoch <= FL_BULK_EPOCH_MAX;
        }
        case FL_REQ_STATUS:
            return true;
        case FL_REQ_REBOOT:
        case FL_REQ_BOOTSEL:
            // ⛔ THE MAGIC IS THE WHOLE GUARD ON A DELIBERATE REBOOT OF A BOARD INSIDE A POWERED
            // SAMPLER. A stray control transfer with the right bRequest and no magic is STALLed.
            return w_value == FL_REQ_MAGIC;
        default:
            return false;
    }
}

bool flBulkClaimAllowed(bool cdc_at_sync0, uint32_t queued, bool work_active)
{
    return cdc_at_sync0 && queued == 0 && !work_active;
}

bool flBulkReleaseAllowed(uint32_t queued)
{
    return queued == 0;
}

bool flBulkFrameStale(uint16_t frame_epoch, uint16_t current_epoch)
{
    // 0 is "no epoch": a CDC-delivered frame (or a reply owed to one) is never stale, whatever the
    // vendor-bulk side has done. Any other stamp that is not the current epoch belongs to a session
    // a BULK RESET has abandoned.
    return frame_epoch != 0 && frame_epoch != current_epoch;
}

bool flBulkAtomicMustWait(uint8_t mode, uint32_t queued_other)
{
    // mode 0x01 is FL_WM_ATOMIC. An atomic run publishes the whole payload without yielding, so
    // it may not start behind work that is already queued: the sampler would wait for both.
    return mode == 0x01 && queued_other > 0;
}

bool flBulkDataCrcOk(uint32_t expected, const uint8_t *data, uint32_t len)
{
    if (!data && len) return false;
    return flDigestCrc32(0, data, len) == expected;
}

/*****************************************************************************
 * ⭐ ARMRETRY r7 · DATACRC - see the header *
 *****************************************************************************/

uint32_t flBulkSessionNext(uint32_t *counter, uint32_t salt)
{
    // xor with a fixed salt is a bijection, so a new counter value is a new session; one counter value in
    // 2^32 maps to 0 and is stepped over (at most one extra step).
    uint32_t s;
    do { (*counter)++; s = *counter ^ salt; } while (s == 0);
    return s;
}

static uint16_t fl_bulk_epoch_dist(uint16_t from, uint16_t to)
{
    return (uint16_t)(((uint32_t)to + FL_BULK_EPOCH_MAX - from) % FL_BULK_EPOCH_MAX);
}

bool flBulkFrameOwed(uint16_t frame_epoch, uint16_t owed_from, uint16_t current)
{
    if (owed_from == 0 || frame_epoch == 0 || frame_epoch == current) return false;
    const uint16_t d = fl_bulk_epoch_dist(frame_epoch, current);
    return d != 0 && d <= fl_bulk_epoch_dist(owed_from, current);
}

uint8_t flBulkGuardCheckCb(const fl_bulk_cb_t *cb, uint32_t current_session)
{
    if (!cb || cb->guard == FL_GUARD_NONE) return 0;
    // ⛔ THE SESSION FIRST. A block from before a reset is refused whatever its data, because its
    // data may be intact: that is exactly the stale command the CRC cannot see.
    if (cb->session != current_session) return FL_DC_STALE_SESSION;
    // ⛔ A FULL BLOCK WITH NO DATA PHASE HAS ONE LEGAL data_crc32: the CRC of nothing, 0. Anything else
    // says the host thought a phase was coming that the board will not arm - refused before the commit.
    if (cb->guard == FL_GUARD_FULL && cb->data_bytes == 0 && cb->data_crc != 0) return FL_DC_BAD_DATA;
    return 0;
}

uint8_t flBulkGuardCheckData(const fl_bulk_cb_t *cb, const uint8_t *data, uint32_t n)
{
    if (!cb || cb->guard != FL_GUARD_FULL) return 0;
    return flBulkDataCrcOk(cb->data_crc, data, n) ? 0 : FL_DC_BAD_DATA;
}

uint32_t flSessionTrailerAt(uint8_t type, const uint8_t *p, uint32_t len)
{
    if (!p) return FL_DC_NO_TRAILER;
    uint32_t legacy;
    switch (type)
    {
        case FL_T_WRITE_SECTORS:
        {
            // {id u8, lba u32, count u16, mode u8, flags u8, gen u32} + count x 512, then the session.
            // The two shapes differ by 4 and every legal one by a multiple of 512, so they cannot meet.
            if (len < FL_BULK_WRITE_HDR + FL_DC_TRAILER_SESSION) return FL_DC_NO_TRAILER;
            uint32_t count = (uint32_t)p[5] | ((uint32_t)p[6] << 8);
            legacy = FL_BULK_WRITE_HDR + count * 512u;
            break;
        }
        case FL_T_SYNC:         legacy = 1;  break;   /* {id u8} (0xFF = the whole card) */
        case FL_T_CARD_MODE:    legacy = 4;  break;   /* "CARD" */
        case FL_DC_T_TQ_SYNC:   legacy = 12; break;   /* note-at §4 */
        case FL_DC_T_TQ_STATUS: legacy = 1;  break;   /* {flags u8}; the host guards it when it clears */
        default: return FL_DC_NO_TRAILER;
    }
    return len == legacy + FL_DC_TRAILER_SESSION ? legacy : FL_DC_NO_TRAILER;
}

#endif // FORGE_LINK_BULK || FL_BULK_HOST
