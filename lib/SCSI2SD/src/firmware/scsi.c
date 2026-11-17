//	Copyright (C) 2014 Michael McMaster <michael@codesrc.com>
//	Copyright (c) 2023 joshua stein <jcs@jcs.org>
//	Copyright (c) 2023 Andrea Ottaviani <andrea.ottaviani.69@gmail.com>
//	Copyright (c) 2024-2025 Rabbit Hole Computing™
//	Copyright (c) 2025 Kevin Moonlight <me@yyzkevin.com>
//
//	This file is part of SCSI2SD.
//
//	SCSI2SD is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	SCSI2SD is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with SCSI2SD.  If not, see <http://www.gnu.org/licenses/>.


#include "scsi.h"
#include "scsiPhy.h"
#include "config.h"
#include "diagnostic.h"
#include "disk.h"
#include "inquiry.h"
#include "led.h"
#include "mode.h"
#include "scsi2sd_time.h"
#include "timings.h"
#include "bsp.h"
#include "cdrom.h"
#include "network.h"
#include "tape.h"
#include "mo.h"
#include "vendor.h"
#include <string.h>
#include "toolbox.h"
#include <ZuluSCSI_platform_config.h>

#ifdef ENABLE_AUDIO_STREAM
// Host PCM audio streaming handler (ZuluSCSI_audio_stream.cpp, C++ extern "C").
int scsiAudioCommand(void);
#endif

// Global SCSI device state.
ScsiDevice scsiDev S2S_DMA_ALIGN;

static void enter_SelectionPhase(void);
static void process_SelectionPhase(void);
static void enter_MessageIn(uint8_t message);
static void enter_Status(uint8_t status);
static void enter_DataIn(int len);
static void process_DataIn(void);
static void process_DataOut(void);
static void process_Command(void);

static void doReserveRelease(void);

static struct {
	int active;
	int phase;
	uint8_t cdbLen;
	uint32_t disconnectTime_ms;
} g_scsi_disconnect_state;

void enter_BusFree()
{
	// This delay probably isn't needed for most SCSI hosts, but it won't
	// hurt either. It's possible some of the samplers needed this delay.
	if (scsiDev.compatMode < COMPAT_SCSI2)
	{
		s2s_delay_us(2);
	}

#if 0
	if (scsiDev.status != GOOD)// && isDebugEnabled())
	{
		// We want to capture debug information for failure cases.
		s2s_delay_ms(80);
	}
#endif


	scsiEnterBusFree();

	// Wait for the initiator to cease driving signals
	// Bus settle delay + bus clear delay = 1200ns
	// Just waiting the clear delay is sufficient.
	s2s_delay_ns(800);

	s2s_ledOff();
	scsiDev.phase = BUS_FREE;
	scsiDev.selFlag = 0;
}

static void enter_MessageIn(uint8_t message)
{
	scsiDev.msgIn = message;
	scsiDev.phase = MESSAGE_IN;
}

int process_MessageIn(int releaseBusFree)
{
	scsiEnterPhase(MESSAGE_IN);
	scsiWriteByte(scsiDev.msgIn);

	if (unlikely(scsiDev.atnFlag))
	{
		// If there was a parity error, we go
		// back to MESSAGE_OUT first, get out parity error message, then come
		// back here.
		return 0;
	}
	else if ((scsiDev.msgIn == MSG_LINKED_COMMAND_COMPLETE) ||
		(scsiDev.msgIn == MSG_LINKED_COMMAND_COMPLETE_WITH_FLAG))
	{
		// Go back to the command phase and start again.
		scsiDev.phase = COMMAND;
		scsiDev.dataPtr = 0;
		scsiDev.savedDataPtr = 0;
		scsiDev.dataLen = 0;
		scsiDev.status = GOOD;
		transfer.blocks = 0;
		transfer.currentBlock = 0;
		return 0;
	}
	else if (releaseBusFree) /*if (scsiDev.msgIn == MSG_COMMAND_COMPLETE)*/
	{
		enter_BusFree();
		return 1;
	}
	else
	{
		return 1;
	}
}

static void messageReject()
{
	scsiEnterPhase(MESSAGE_IN);
	scsiWriteByte(MSG_REJECT);
}

static void enter_Status(uint8_t status)
{
	scsiDev.status = status;
	scsiDev.phase = STATUS;

	scsiDev.lastStatus = scsiDev.status;
	scsiDev.lastSense = scsiDev.target->sense.code;
	scsiDev.lastSenseASC = scsiDev.target->sense.asc;
}

void process_Status()
{
	scsiEnterPhase(STATUS);

	if (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_EWSD)
	{
		s2s_delay_ms(1);
	}

	uint8_t message;

	uint8_t control = scsiDev.cdb[scsiDev.cdbLen - 1];

	if (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_OMTI)
	{
		// All commands have a control byte, except 0xC0
		if (scsiDev.cdb[0] == 0xC0)
		{
			control = 0;
		}

		// OMTI non-standard LINK control
		if (control & 0x01)
		{
			scsiDev.phase = COMMAND;
			return;
		}
	}

	if ((scsiDev.status == GOOD) && (control & 0x01) &&
		scsiDev.target->cfg->quirks != S2S_CFG_QUIRKS_XEBEC)
	{
		// Linked command.
		scsiDev.status = INTERMEDIATE;
		if (control & 0x02)
		{
			message = MSG_LINKED_COMMAND_COMPLETE_WITH_FLAG;
		}
		else
		{
			message = MSG_LINKED_COMMAND_COMPLETE;
		}
	}
	else
	{
		message = MSG_COMMAND_COMPLETE;
	}

	if (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_XEBEC)
	{
		// More non-standardness. Expects 2 status bytes (really status + msg)
		// 00 d 000 err 0
		// d == disk number
		// ERR = 1 if error.
		if (scsiDev.status == GOOD)
		{
			scsiWriteByte(scsiDev.cdb[1] & 0x20);
		}
		else
		{
			scsiWriteByte((scsiDev.cdb[1] & 0x20) | 0x2);
		}
		s2s_delay_us(10); // Seems to need a delay before changing phase bits.
	}
	else if (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_OMTI)
	{
		scsiDev.status |= (scsiDev.target->targetId & 0x03) << 5;
		scsiWriteByte(scsiDev.status);
	}
	else
	{
		scsiWriteByte(scsiDev.status);
	}

	scsiDev.lastStatus = scsiDev.status;
	scsiDev.lastSense = scsiDev.target->sense.code;
	scsiDev.lastSenseASC = scsiDev.target->sense.asc;

	// Command Complete occurs AFTER a valid status has been
	// sent. then we go bus-free.
	enter_MessageIn(message);
}

static void enter_DataIn(int len)
{
	scsiDev.dataLen = len;
	scsiDev.phase = DATA_IN;
}

static void process_DataIn()
{
	uint32_t len;

	if (scsiDev.dataLen > sizeof(scsiDev.data))
	{
		scsiDev.dataLen = sizeof(scsiDev.data);
	}

	len = scsiDev.dataLen - scsiDev.dataPtr;
	if (len > 0)
	{
		scsiEnterPhase(DATA_IN);
		scsiWrite(scsiDev.data + scsiDev.dataPtr, len);
		scsiDev.dataPtr += len;
	}

	if ((scsiDev.dataPtr >= scsiDev.dataLen) &&
		(transfer.currentBlock == transfer.blocks))
	{
		enter_Status(scsiDev.status);
	}
}

static void process_DataOut()
{
	uint32_t len;

	if (scsiDev.dataLen > sizeof(scsiDev.data))
	{
		scsiDev.dataLen = sizeof(scsiDev.data);
	}

	len = scsiDev.dataLen - scsiDev.dataPtr;
	if (len > 0)
	{
		scsiEnterPhase(DATA_OUT);

		int parityError = 0;
		scsiRead(scsiDev.data + scsiDev.dataPtr, len, &parityError);
		scsiDev.dataPtr += len;

		if (parityError &&
			(scsiDev.boardCfg.flags & S2S_CFG_ENABLE_PARITY))
		{
			scsiDev.target->sense.code = ABORTED_COMMAND;
			scsiDev.target->sense.asc = SCSI_PARITY_ERROR;
			enter_Status(CHECK_CONDITION);
		}
	}

	if ((scsiDev.dataPtr >= scsiDev.dataLen) &&
		(transfer.currentBlock == transfer.blocks))
	{
		if (scsiDev.postDataOutHook != NULL)
		{
			scsiDev.postDataOutHook();
		}
		else
		{
			enter_Status(GOOD);
		}
	}
}

static const uint8_t CmdGroupBytes[8] = {6, 10, 10, 6, 16, 12, 6, 6};

#ifdef FORGE_LINK_TARGET7
// T3 ROTOR (contact-1, 2026-09-18, investigation-2 T3). ID 7 only. Each EXCL SEND
// press on the S1000 = one TUR,INQ,TUR,INQ burst; a TUR arriving >300 ms after the last
// ID-7 command starts the next rotor step. Each step serves a different 7-byte INQUIRY
// body (all guesses, ranked by the investigation) or a different TUR answer, and logs
// "ROTOR k". Any THIRD command in a burst is the prize: the stock log records its CDB.
typedef struct { uint8_t tur; uint8_t len; uint8_t body[8]; } fl_rotor_t;
// tur: 0 GOOD, 1 CHECK NOT READY, 2 CHECK UNIT ATTENTION, 3 BUSY
static const fl_rotor_t fl_rotor_tab[] __attribute__((unused)) = {
    {0, 7, {0x03,0x00,0x01,0x00,0x02,0x00,0x00}},
    {0, 7, {0x00,0x00,0x01,0x00,0x02,0x00,0x00}},
    {0, 7, {0x03,0x00,0x01,0x00,0x02,0x47,0x48}},
    {0, 7, {0x00,0x00,0x01,0x00,0x02,0x47,0x48}},
    {0, 7, {0x03,0x00,0x00,0x00,0x02,0x00,0x00}},
    {0, 7, {0x03,0x00,0x01,0x00,0x1F,0x47,0x48}},
    {0, 5, {0x03,0x00,0x01,0x00,0x00}},
    {1, 7, {0x03,0x00,0x01,0x00,0x02,0x00,0x00}},
    {2, 7, {0x03,0x00,0x01,0x00,0x02,0x00,0x00}},
    {3, 7, {0x03,0x00,0x01,0x00,0x02,0x00,0x00}},
};
// ROTOR 3 (contact-1): error replies to INQUIRY, then a short list of
// vendor-specific bytes 5-6 (Akai SysEx maker 47h, S1000 model 48h, neighbours, ASCII).
// inq: 0 = data (body, len 7), 1 = CHECK ILLEGAL REQUEST, 2 = CHECK NOT READY, 3 = BUSY,
// 4 = GOOD with no data.
typedef struct { uint8_t inq; uint8_t b0, b5, b6; } fl_r3_t;
static const fl_r3_t fl_r3[] = {
    {1,0,0,0},{2,0,0,0},{3,0,0,0},{4,0,0,0},
    {0,0x03,0x47,0x00},{0,0x03,0x00,0x47},{0,0x03,0x47,0x48},{0,0x03,0x48,0x47},
    {0,0x03,0x47,0x49},{0,0x03,0x47,0x01},{0,0x03,0x01,0x47},{0,0x03,0x48,0x00},
    {0,0x03,0x00,0x48},{0,0x03,0x41,0x4B},{0,0x03,0x53,0x31},{0,0x03,0x47,0x47},
    {0,0x00,0x47,0x00},{0,0x00,0x00,0x47},{0,0x00,0x48,0x00},{0,0x00,0x00,0x48},
    {0,0x00,0x41,0x4B},{0,0x03,0x10,0x00},
};
#define FL_ROTOR_N ((int)(sizeof(fl_r3) / sizeof(fl_r3[0])))
static fl_rotor_t fl_rotor_cur;
static const fl_rotor_t *fl_rotor_entry(int k)
{
    const fl_r3_t *e = &fl_r3[k];
    fl_rotor_cur.tur = 0; fl_rotor_cur.len = 7;
    fl_rotor_cur.body[0] = e->b0; fl_rotor_cur.body[1] = 0x00; fl_rotor_cur.body[2] = 0x01;
    fl_rotor_cur.body[3] = 0x00; fl_rotor_cur.body[4] = 0x02;
    fl_rotor_cur.body[5] = e->b5; fl_rotor_cur.body[6] = e->b6;
    fl_rotor_cur.body[7] = e->inq;   // carried in the spare byte
    return &fl_rotor_cur;
}
extern void forgeLinkLogRotor(int k, int tur, int len, const uint8_t *b);
extern void forgeLinkLogSweep(int kind, int i, int val, const uint8_t *b);
extern void forgeLinkLogBulk(uint32_t bytes, uint32_t ms, uint32_t sum, const uint8_t *head, int headn, int parityError);
extern void forgeLinkSessionClosed(void);
static int fl_rotor_k = -1;
// In-RAM tally (the USB log drops lines under the S1000's rapid checks):
uint16_t g_fl_rotor_attempts[64];  // indexed by rotor step (< 64)
uint8_t g_fl_prize_op[16], g_fl_prize_step[16];
uint8_t g_fl_prize_n = 0;
static uint32_t fl_rotor_last_ms = 0;
// ---- ID 7 LIVE TARGET (contact-1 next step, 2026-09-18) ----
// Set from the Mac over the USB console, no reflash:
//   !I <hex...>          INQUIRY reply bytes for ID 7 (1-36 bytes); !I with no bytes = back to rotor
//   !O <op> <g|o|i> <n>  per-opcode action: g = GOOD no data; o = take n bytes DATA OUT
//                        (n = 0 means "24-bit length from CDB bytes 2-4"); i = serve queued DATA IN
//   !D <hex...>          queue DATA IN bytes (up to 1024) for the next 'i' command
//   !R                   clear everything back to defaults
// Unknown opcodes default to GOOD (a CHECK stalls the S1000 ~15 s). Every non-TUR/INQUIRY CDB
// and every DATA OUT payload is hex-logged to the USB console for the Mac.
uint8_t g_fl7_inq[36]; uint8_t g_fl7_inq_len = 0;
// The Akai vendor opcodes DEFAULT to their protocol meaning. They used to default to
// "GOOD, no data", which meant that after every reflash the board accepted the S1000's
// SEND (0x0C) and never took the data phase - the bus then hung until the watchdog reset
// it (bench 2026-09-19, three times before the cause was seen). A default that matches the
// protocol cannot be forgotten; an arming step can.
// The default lives in fl7_serve.h (FL7_SEND_OP / FL7_SEND_MODE) so '!R' restores the same table
// (fl7_opcode_defaults) rather than zeroing it (C63).
#include "fl7_serve.h"
uint8_t g_fl7_mode[256] = { [FL7_SEND_OP] = FL7_SEND_MODE }; uint16_t g_fl7_len[256];
uint8_t g_fl7_din[1024]; uint16_t g_fl7_din_len = 0;
// ---- OUTBOUND MAILBOX (2026-09-19, the return path) ----
// The S1000 polls STATUS (0x0D) between messages and expects a 24-bit big-endian
// count of the bytes we are holding for it; if that is non-zero it issues RECEIVE
// (0x0E) for exactly that many. Described in the interface spec and proved
// on the bench the same afternoon. The older `din` queue cannot serve this: it is
// cleared after one answer, and this needs to answer twice, consistently.
uint8_t g_fl7_out[8192]; uint16_t g_fl7_out_len = 0, g_fl7_out_pos = 0; uint8_t g_fl7_mail = 0;
uint16_t g_fl7_chunk = 128;   // bytes offered per poll; 0 = no limit. !C <n> to change.
// 8 KiB, not 1 KiB: a 320-word sample dump is 1037 bytes and the old mailbox silently
// dropped the last 13, leaving the S1000 waiting forever for the end of a packet it was
// never sent (bench 2026-09-19 - it sat on "receiving MIDI exclusive data" for minutes).
static uint8_t g_fl7_buf[1024];

// ── THE FL-WRITE INSTRUMENT ────────────────────────────────────────────────────────────────
// Counts one Mac->S1000-RAM write, on the BOARD's clock, and reports at session close.
// ⛔ The Mac's clock cannot measure this: the whole payload is preloaded into the mailbox
// before the S1000 ever polls, so a host-side timer measures the queueing, not the transfer.
// ⚠ And every rate we have quoted so far was taken through a hex-logging build, which makes
// it a FLOOR, not a measurement. This counts with logging OFF and costs six adds per poll.
uint32_t g_flw_t0 = 0, g_flw_tlast = 0, g_flw_wire = 0;
uint32_t g_flw_polls = 0, g_flw_acks = 0, g_flw_dpkts = 0, g_flw_hdrs = 0;
extern void forgeLinkWriteReport(void);
extern void forgeLinkLogHex(const char *what, const uint8_t *d, int n);

// ── L1b · SERVE ON REQUEST (2026-09-21) ─────────────────────────────────────────────────────
// The S1000's REQUEST branch (OPTION held + SEND, W39) asks its MIDI-via-SCSI peer for a
// sample with RSDATA and - if the first complete SysEx back is an SDATA - arms a residual of the
// whole sample and pulls the audio with BULK 0D/0E (W37 request code 4). This makes our ID 7
// that peer. The Mac stages the header and the audio in board RAM BEFORE the session (!H, !B),
// arms with !W 1, and is then silent (bridge-operation §4); the board answers on its own.
// ⛔ AUDIO NEVER MOVES AS SysEx (a project rule): SysEx carries only the header; audio goes
// ONLY through the bulk data phase. NOTHING here runs unless !W 1 armed it: with the serve OFF
// every 0C/0D/0E takes exactly the path it took before.
// The decisions are in fl7_serve.h (shared with the Mac-side simulator and its tests); this is
// only the bus glue.
#include "fl7_serve.h"
#ifndef FL7_STAGE_SIZE
#define FL7_STAGE_SIZE (64u * 1024u)   // 64 KiB: one period of the 16-bit test ramp, and the
#endif                                 // W38 sample (26,460 B). Sized against this build's RAM.
uint8_t g_fl7s_audio[FL7_STAGE_SIZE];
fl7s_t g_fl7s = { FL7S_OFF, g_fl7s_audio, FL7_STAGE_SIZE };   // chunk 0 = the whole residual
extern void forgeLinkServeEvent(int ev, uint32_t a, uint32_t b, uint32_t c);
// Streams `real` staged bytes then `pad` zeros as ONE data phase (already entered), with the
// debug trace suspended and the board's microsecond clock on it. Returns 1 on a bus reset.
extern int forgeLinkServeStream(uint32_t real, uint32_t pad);

static int fl7s_bulk(uint8_t command)
{
    if (command == 0x0D)
    {
        int begin = 0;
        uint32_t n = fl7s_on_bulk0d(&g_fl7s, g_fl7_out_len, s2s_getTime_ms(), &begin);
        if (begin) forgeLinkServeEvent(FL7S_EV_BEGIN, n, 0, 0);
        scsiDev.data[0] = (uint8_t)(n >> 16);
        scsiDev.data[1] = (uint8_t)(n >> 8);
        scsiDev.data[2] = (uint8_t)n;
        scsiDev.dataLen = 3;
        scsiDev.phase = DATA_IN;
        return 1;
    }
    // BULK 0E: exactly CDB[2..4] bytes, streamed straight from staging. scsiDev.data is too
    // small for a bulk block, and the non-bulk path caps it there - i.e. it would SHORT the phase.
    uint32_t want = ((uint32_t)scsiDev.cdb[2] << 16) | ((uint32_t)scsiDev.cdb[3] << 8) | scsiDev.cdb[4];
    uint32_t real = 0, pad = 0;
    fl7s_plan_bulk0e(&g_fl7s, want, g_fl7_out_len, &real, &pad);
    int reset = 0;
    if (want)
    {
        scsiEnterPhase(DATA_IN);
        reset = forgeLinkServeStream(real, pad);
    }
    uint32_t at = g_fl7s.pos;
    int ev = fl7s_after_bulk0e(&g_fl7s, real, pad, reset || scsiDev.resetFlag, s2s_getTime_ms());
    if (ev == FL7S_EV_END) forgeLinkServeEvent(ev, g_fl7s.pos, g_fl7s.t1 - g_fl7s.t0, g_fl7s.sum);
    else if (ev == FL7S_EV_RESET)
    {
        forgeLinkServeEvent(ev, at, 0, 0);
        // The serve ends here, as at a close: no stale position or header for the next session
        // (C59). scsiReset() does the same for a reset outside a bulk 0E.
        fl7s_on_bus_reset(&g_fl7s, &g_fl7_out_len, &g_fl7_out_pos);
    }
    return 1;   // phase still COMMAND -> GOOD via fall-through
}

static int fl7_live(uint8_t command)
{
    if (command == 0x12 && g_fl7_inq_len)
    {
        uint32_t alloc = scsiDev.cdb[4];
        memcpy(scsiDev.data, g_fl7_inq, g_fl7_inq_len);
        scsiDev.dataLen = (g_fl7_inq_len < alloc) ? g_fl7_inq_len : alloc;
        scsiDev.phase = DATA_IN;
        return 1;
    }
    // ⛔ UNCONDITIONAL. 0D and 0E are answered whether or not anything has been queued: an
    // empty mailbox answers "00 00 00", it does NOT fall through to a bare GOOD. Gating this
    // on the mailbox meant that a bridge nobody had armed answered the S1000's poll with no
    // data phase at all, and the bus hung until the watchdog (bench 2026-09-19). A device that
    // asks a question must always get an answer, even when the answer is "nothing".
    // L1b: a BULK 0D/0E (CDB[5] bit 7) is the audio pull, and only while a serve is armed.
    // Unarmed, bulk polls fall through to the mailbox path below exactly as before.
    if ((command == 0x0D || command == 0x0E) && (scsiDev.cdb[5] & 0x80)
        && g_fl7s.state != FL7S_OFF)
        return fl7s_bulk(command);
    if (command == 0x0D || command == 0x0E)
    {
        uint32_t left = (uint32_t)(g_fl7_out_len - g_fl7_out_pos);
        // L1b: while a serve is armed the header goes ONE PACKET PER POLL whatever !C says -
        // rule 1 of bridge-operation §3 is not left to a setting during a serve.
        uint32_t chunk = fl7s_nonbulk_chunk(&g_fl7s, g_fl7_chunk);
        // (fl7s_offer: at most `chunk`, trimmed back to the last F7 in range - the two rules
        // written out here before L1b, moved into fl7_serve.h unchanged so they are tested.)
        // ⛔ PACE IT. The S1000's MIDI input ring is 256 bytes. Answering the poll with the
        // WHOLE mailbox makes it ask for the lot in one RECEIVE, and everything past the ring
        // is lost - it then waits forever for the end of a shredded SysEx message, on a screen
        // with no cancel (bench 2026-09-19, twice). Offer ONE packet at a time and let its own
        // poll cadence do the flow control: that is what the ACKs were for.
        // ⛔ AND END ON A MESSAGE BOUNDARY. Cutting a SysEx mid-flight is what stranded the
        // S1000 twice; so the offer is trimmed back to the last F7 inside the window and every
        // handover is a COMPLETE message (no F7 in range: offered as-is, raw or a long message).
        left = fl7s_offer(g_fl7_out + g_fl7_out_pos, left, chunk);
        if (command == 0x0D)                    // STATUS: how much have you got?
        {
            g_flw_polls++;
            scsiDev.data[0] = (uint8_t)(left >> 16);
            scsiDev.data[1] = (uint8_t)(left >> 8);
            scsiDev.data[2] = (uint8_t)left;
            scsiDev.dataLen = 3;
            scsiDev.phase = DATA_IN;
            if (g_fl7s.state == FL7S_OFF) forgeLinkLogHex("FL7 STATUS", scsiDev.data, 3);
            return 1;
        }
        uint32_t want = ((uint32_t)scsiDev.cdb[2] << 16) | ((uint32_t)scsiDev.cdb[3] << 8) | scsiDev.cdb[4];
        if (want > sizeof(scsiDev.data)) want = sizeof(scsiDev.data);
        uint32_t n = want < left ? want : left;
        // ⛔ NEVER SHORT THE DATA PHASE. Serving fewer bytes than the CDB asked for leaves
        // the transfer unfinished and HANGS THE BUS until the board's watchdog resets it -
        // measured twice on the bench 2026-09-19 ("WATCHDOG TIMEOUT, attempting bus reset").
        // Give exactly what was asked for and pad the remainder with zeros.
        memcpy(scsiDev.data, g_fl7_out + g_fl7_out_pos, n);
        if (want > n) memset(scsiDev.data + n, 0, want - n);
        scsiDev.dataLen = want;
        scsiDev.phase = want ? DATA_IN : scsiDev.phase;
        if (g_fl7s.state == FL7S_OFF) forgeLinkLogHex("FL7 RECEIVE", g_fl7_out + g_fl7_out_pos, (int)n);
        if (n)                                  // FL-WRITE: tally this handover
        {
            uint32_t now = s2s_getTime_ms();
            if (!g_flw_t0) g_flw_t0 = now;      // first byte out is t0, never the session open
            g_flw_tlast = now;
            g_flw_wire += n;
            // Classify by the SysEx type byte, never by size: F0 7E <ch> <type>.
            // 01 = dump header, 02 = data packet. Anything else is not counted as either.
            for (uint32_t i = 0; i + 3 < n; i++)
            {
                const uint8_t *q = g_fl7_out + g_fl7_out_pos + i;
                if (q[0] == 0xF0 && q[1] == 0x7E)
                {
                    if (q[3] == 0x02) g_flw_dpkts++;
                    else if (q[3] == 0x01) g_flw_hdrs++;
                }
            }
        }
        g_fl7_out_pos += (uint16_t)n;
        if (g_fl7_out_pos >= g_fl7_out_len)
        {
            if (fl7s_on_mailbox_drained(&g_fl7s, n)) forgeLinkServeEvent(FL7S_EV_HEADER_DONE, g_fl7_out_len, 0, 0);
            g_fl7_out_len = 0; g_fl7_out_pos = 0;
        }
        return 1;
    }
    if (command == 0x00 || command == 0x12 || command == 0x03) return 0;
    forgeLinkLogHex("FL7 CDB", scsiDev.cdb, scsiDev.cdbLen);
    // The S1000 closes a session with 09 and byte 2 = 0. That instant is the first candidate
    // for the window in which its own target side is alive - see the remote-enable hunt.
    if (command == 0x09 && scsiDev.cdb[2] == 0)
    {
        forgeLinkSessionClosed();
        if (g_fl7s.state != FL7S_OFF)
        {
            uint32_t served = g_fl7s.pos, resid = g_fl7s.residual;
            int was = fl7s_on_close(&g_fl7s, &g_fl7_out_len, &g_fl7_out_pos);
            forgeLinkServeEvent(FL7S_EV_CLOSED, (uint32_t)was, served, resid);
        }
    }
    uint8_t m = g_fl7_mode[command];
    if (m == 'o')
    {
        uint32_t n = g_fl7_len[command];
        if (n == 0) n = ((uint32_t)scsiDev.cdb[2] << 16) | ((uint32_t)scsiDev.cdb[3] << 8) | scsiDev.cdb[4];
        if (n)
        {
            // STREAM the whole transfer, however big. A sample arrives as ONE command:
            // 264,704 bytes measured on the bench 2026-09-19, and capping the read at the
            // 1 KB buffer left the DATA OUT phase unfinished and stopped the conversation
            // dead. Absorb it in buffer-sized bites, keep the head for identification and a
            // checksum for integrity, and TIME it - this is the only honest throughput
            // number we can take, since it excludes the per-command setup.
            int parityError = 0;
            uint8_t head[256]; uint32_t headn = 0, left = n, sum = 0;   // small replies land WHOLE
            uint32_t t0 = s2s_getTime_ms();
            scsiEnterPhase(DATA_OUT);
            // C57: stop on a bus reset rather than parse a stale buffer. C58: scsiRead clears its
            // flag on entry, so each chunk's parity result is ORed in, not assigned.
            while (left && !scsiDev.resetFlag)
            {
                uint32_t k = left > sizeof(g_fl7_buf) ? sizeof(g_fl7_buf) : left;
                int pe = 0;
                scsiRead(g_fl7_buf, k, &pe);
                parityError |= pe;
                for (uint32_t i = 0; i < k; i++)
                {
                    sum = (sum << 1) + (sum >> 31) + g_fl7_buf[i];
                    if (headn < sizeof(head)) head[headn++] = g_fl7_buf[i];
                }
                left -= k;
            }
            forgeLinkLogBulk(n - left, s2s_getTime_ms() - t0, sum, head, (int)headn, parityError);
            // C57: a bus reset mid-transfer leaves a partial, possibly stale head. Parse none of
            // it; scsiPoll performs the reset next.
            if (scsiDev.resetFlag) return 1;
            // FL-WRITE: the S1000 ACKs every packet and that ACK is flow control, so the
            // ack count is how we tell a completed dump from one that stalled half way.
            // Scan the head rather than testing offset 0: one 0C can carry several.
            if (command == 0x0C && g_fl7s.state != FL7S_OFF && !(scsiDev.cdb[5] & 0x80))
            {
                uint32_t arg = 0;
                int ev = fl7s_on_0c(&g_fl7s, head, headn, g_fl7_out, &g_fl7_out_len, &g_fl7_out_pos, &arg);
                if (ev == FL7S_EV_SDATA_QUEUED || ev == FL7S_EV_END_OF_LIST) g_fl7_mail = 1;
                if (ev != FL7S_EV_NONE) forgeLinkServeEvent(ev, arg, g_fl7s.state, 0);
            }
            if (command == 0x0C)
                for (uint32_t i = 0; i + 5 < headn; i++)
                    if (head[i] == 0xF0 && head[i+1] == 0x7E && head[i+3] == 0x7F)
                    { g_flw_acks++; g_flw_tlast = s2s_getTime_ms(); }
        }
        return 1;  // GOOD via fall-through
    }
    if (m == 'i')
    {
        uint32_t alloc = g_fl7_len[command] ? g_fl7_len[command] : 1024;
        uint32_t n = g_fl7_din_len < alloc ? g_fl7_din_len : alloc;
        memcpy(scsiDev.data, g_fl7_din, n);
        scsiDev.dataLen = n;
        scsiDev.phase = n ? DATA_IN : scsiDev.phase;
        forgeLinkLogHex("FL7 DIN", g_fl7_din, (int)n);
        g_fl7_din_len = 0;
        return 1;
    }
    return 1;  // default 'g': GOOD, no data
}

// ---- BYTE SWEEP (2026-09-19). The bench proved the S1000 ignores bytes 0,2,3,4 across
// every legal value (5 candidates incl. a control, all 4-command bursts, no third command).
// The untried data fields are byte 1 (SCSI-1's vendor-specific device-type qualifier) and
// bytes 5-6. A full byte-1 sweep is 128 presses of EXCL SEND, which is too many hands, so
// this is a CURATED list, and the step advances per INQUIRY rather than per burst: the
// S1000 asks twice per press, so one press covers two candidates.
// KNOWN BLIND SPOT, stated rather than hidden: no value is seen twice in one press, so a
// check that compares the two answers of a burst cannot be caught by this sweep.
//   !S 1   byte 1     !S 2   byte 5     !S 3   byte 6     !S 0   off (back to the rotor)
static const uint8_t fl_sweep_b1[] = {
    0x00, 0x80,              // the two already tried, as in-run controls
    0x01, 0x02, 0x03, 0x04,  // low qualifiers
    0x0F, 0x10, 0x20, 0x40,  // bit edges
    0x47, 0x48, 0x49,        // Akai SysEx maker 47h, S1000 model 48h, neighbour
    0x60, 0x7F, 0xFF,        // high edges and all-ones
};
static const uint8_t fl_sweep_b56[] = {
    0x00, 0x47, 0x48, 0x49, 0x01, 0x02, 0x10, 0x20,
    0x41, 0x4B, 0x53, 0x31, 0x7F, 0x80, 0xF0, 0xFF,
};
uint8_t g_fl_sweep_kind = 0;      // 0 off, 1 = byte 1, 2 = byte 5, 3 = byte 6
uint8_t g_fl_sweep_i = 0;
uint16_t g_fl_sweep_tries[64];
// The body everything sweeps around: the best-formed answer the bench could build
// (textbook SCSI-2 processor), since bytes 0/2/3/4 are measured not to matter.
static uint8_t fl_sweep_body[8] = {0x03, 0x00, 0x02, 0x02, 0x1F, 0x00, 0x00, 0x00};
static int fl_sweep_n(void)
{
    if (g_fl_sweep_kind == 1) return (int)(sizeof(fl_sweep_b1));
    if (g_fl_sweep_kind) return (int)(sizeof(fl_sweep_b56));
    return 0;
}
static uint8_t fl_sweep_val(int i)
{
    return (g_fl_sweep_kind == 1) ? fl_sweep_b1[i] : fl_sweep_b56[i];
}
// Answer one INQUIRY from the sweep, then step. Returns 1 if it handled the command.
static int fl_sweep_inquiry(void)
{
    int n = fl_sweep_n();
    if (!n) return 0;
    if (g_fl_sweep_i >= n) g_fl_sweep_i = 0;
    uint8_t v = fl_sweep_val(g_fl_sweep_i);
    fl_sweep_body[g_fl_sweep_kind == 1 ? 1 : (g_fl_sweep_kind == 2 ? 5 : 6)] = v;
    uint32_t alloc = scsiDev.cdb[4];
    memset(scsiDev.data, 0, 8);
    memcpy(scsiDev.data, fl_sweep_body, 7);
    scsiDev.dataLen = (7 < alloc) ? 7 : alloc;
    scsiDev.phase = DATA_IN;
    if (g_fl_sweep_i < 64 && g_fl_sweep_tries[g_fl_sweep_i] < 0xFFFF) g_fl_sweep_tries[g_fl_sweep_i]++;
    forgeLinkLogSweep(g_fl_sweep_kind, g_fl_sweep_i, v, fl_sweep_body);
    g_fl_sweep_i++;
    return 1;
}

static int fl_rotor(uint8_t command)
{
    if ((scsiDev.target->targetId & 7) != 7) return 0;
    if (fl7_live(command)) return 1;
    if (g_fl7_inq_len) return 0;
    if (g_fl_sweep_kind)
    {
        // The sweep owns TUR (GOOD) and INQUIRY; anything else falls through to the
        // prize path below, which is the whole point of running it.
        if (command == 0x00) return 1;
        if (command == 0x12 && fl_sweep_inquiry()) return 1;
        // THE PRIZE: any third command means the S1000 accepted the answer that was in
        // force. Record which step it was BEFORE falling through to stock handling -
        // the rotor's own prize path below never runs while the sweep owns the replies.
        if (g_fl_prize_n < 16)
        {
            g_fl_prize_op[g_fl_prize_n] = command;
            g_fl_prize_step[g_fl_prize_n] = (uint8_t)(g_fl_sweep_i ? g_fl_sweep_i - 1 : 0);
            g_fl_prize_n++;
        }
        return 0;
    }
    uint32_t now = s2s_getTime_ms();
    if (command == 0x00 && (fl_rotor_k < 0 || (uint32_t)(now - fl_rotor_last_ms) > 100))
    {
        fl_rotor_k = (fl_rotor_k + 1) % FL_ROTOR_N;
        if (g_fl_rotor_attempts[fl_rotor_k] < 0xFFFF) g_fl_rotor_attempts[fl_rotor_k]++;
        const fl_rotor_t *r = fl_rotor_entry(fl_rotor_k);
        forgeLinkLogRotor(fl_rotor_k, r->tur, r->len, r->body);
    }
    fl_rotor_last_ms = now;
    if (fl_rotor_k < 0) return 0;
    const fl_rotor_t *r = fl_rotor_entry(fl_rotor_k);
    if (command == 0x00)
    {
        if (r->tur == 0) return 1;  // GOOD via the phase==COMMAND fall-through
        if (r->tur == 3) { enter_Status(BUSY); return 1; }
        scsiDev.target->sense.code = (r->tur == 1) ? NOT_READY : UNIT_ATTENTION;
        scsiDev.target->sense.asc = (r->tur == 1) ? LOGICAL_UNIT_NOT_READY_CAUSE_NOT_REPORTABLE
                                                  : POWER_ON_RESET_OR_BUS_DEVICE_RESET_OCCURRED;
        enter_Status(CHECK_CONDITION);
        return 1;
    }
    if (command == 0x12 && r->body[7] != 0)
    {
        switch (r->body[7])
        {
            case 1: scsiDev.target->sense.code = ILLEGAL_REQUEST; scsiDev.target->sense.asc = INVALID_COMMAND_OPERATION_CODE; enter_Status(CHECK_CONDITION); break;
            case 2: scsiDev.target->sense.code = NOT_READY; scsiDev.target->sense.asc = LOGICAL_UNIT_NOT_READY_CAUSE_NOT_REPORTABLE; enter_Status(CHECK_CONDITION); break;
            case 3: enter_Status(BUSY); break;
            default: break;  // 4: GOOD with no data (phase stays COMMAND -> GOOD)
        }
        return 1;
    }
    if (command == 0x12)
    {
        uint32_t alloc = scsiDev.cdb[4];
        memset(scsiDev.data, 0, 8);
        memcpy(scsiDev.data, r->body, r->len);
        scsiDev.dataLen = (r->len < alloc) ? r->len : alloc;
        scsiDev.phase = DATA_IN;
        return 1;
    }
    if (g_fl_prize_n < 16) { g_fl_prize_op[g_fl_prize_n] = command; g_fl_prize_step[g_fl_prize_n] = (uint8_t)fl_rotor_k; g_fl_prize_n++; }
    return 0;  // REQUEST SENSE and anything else: stock handling, stock logging
}
#endif

static void process_Command()
{
	int group;
	uint8_t command;
	uint8_t control;

	scsiEnterPhase(COMMAND);

	memset(scsiDev.cdb + 6, 0, sizeof(scsiDev.cdb) - 6);
	int parityError = 0;
	scsiRead(scsiDev.cdb, 6, &parityError);
	command = scsiDev.cdb[0];
	{
		extern void forgeLinkNoteCommand(uint8_t target_id, uint8_t opcode);
		forgeLinkNoteCommand(scsiDev.target->targetId, command);
	}

	group = scsiDev.cdb[0] >> 5;
	scsiDev.cdbLen = CmdGroupBytes[group];
	scsiVendorCommandSetLen(scsiDev.cdb[0], &scsiDev.cdbLen);
	
	if (parityError &&
		(scsiDev.boardCfg.flags & S2S_CFG_ENABLE_PARITY))
	{
		// Don't try and read more bytes, as we cannot be sure what group
		// the command should be.
	}
	else if (scsiDev.cdbLen - 6 > 0)
	{
		scsiRead(scsiDev.cdb + 6, scsiDev.cdbLen - 6, &parityError);
	}


	// Prefer LUN's set by IDENTIFY messages for newer hosts.
	if (scsiDev.lun < 0)
	{
		if (command == 0xE0 || command == 0xE4) // XEBEC s1410
		{
			scsiDev.lun = 0;
		}
		else
		{
			scsiDev.lun = scsiDev.cdb[1] >> 5;
		}
	}


	// For Philips P2000C with Xebec S1410 SASI/MFM adapter
	// http://bitsavers.trailing-edge.com/pdf/xebec/104524C_S1410Man_Aug83.pdf
	if ((scsiDev.lun > 0) && (scsiDev.boardCfg.flags & S2S_CFG_MAP_LUNS_TO_IDS))
	{
		int tgtIndex;
		for (tgtIndex = 0; tgtIndex < S2S_MAX_TARGETS; ++tgtIndex)
		{
			if (scsiDev.targets[tgtIndex].targetId == scsiDev.lun)
			{
				scsiDev.target = &scsiDev.targets[tgtIndex];
				scsiDev.lun = 0;
				break;
			}
		}
	}

	control = scsiDev.cdb[scsiDev.cdbLen - 1];

	scsiDev.cmdCount++;
	const S2S_TargetCfg* cfg = scsiDev.target->cfg;

	if (unlikely(scsiDev.resetFlag))
	{
		// Don't log bogus commands
		scsiDev.cmdCount--;
		memset(scsiDev.cdb, 0xff, sizeof(scsiDev.cdb));
		return;
	}
	// X68000 and strange "0x00 0xXX .. .. .. .." command
	else if ((command == 0x00) && likely(scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_X68000))
	{
		if (scsiDev.cdb[1] == 0x28)
		{
			scsiDev.target->sense.code = NO_SENSE;
			scsiDev.target->sense.asc = NO_ADDITIONAL_SENSE_INFORMATION;
			enter_Status(CHECK_CONDITION);
			return;
		} 	else if (scsiDev.cdb[1] == 0x03)
		{
			scsiDev.target->sense.code = NO_SENSE;
			scsiDev.target->sense.asc = NO_ADDITIONAL_SENSE_INFORMATION;
			enter_Status(GOOD);
			return;
		}
	}
	else if (parityError &&
		(scsiDev.boardCfg.flags & S2S_CFG_ENABLE_PARITY))
	{
		scsiDev.target->sense.code = ABORTED_COMMAND;
		scsiDev.target->sense.asc = SCSI_PARITY_ERROR;
		enter_Status(CHECK_CONDITION);
	}
	else if ((control & 0x02) && ((control & 0x01) == 0) &&
		// used for head step options on xebec.
		likely(scsiDev.target->cfg->quirks != S2S_CFG_QUIRKS_XEBEC))
	{
		// FLAG set without LINK flag.
		scsiDev.target->sense.code = ILLEGAL_REQUEST;
		scsiDev.target->sense.asc = INVALID_FIELD_IN_CDB;
		enter_Status(CHECK_CONDITION);
	}
#ifdef FORGE_LINK_TARGET7
	else if (fl_rotor(command))
	{
		// handled by the T3 rotor (ID 7 only)
	}
#endif
	else if (command == 0x12)
	{
		s2s_scsiInquiry();
	}
	else if (command == 0x03)
	{
		// REQUEST SENSE
		uint32_t allocLength = scsiDev.cdb[4];

		if (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_XEBEC)
		{
			// Completely non-standard
			allocLength = 4;

			switch (scsiDev.target->sense.code)
			{
				case NO_SENSE:
					scsiDev.data[0] = 0;
					break;
				case MEDIUM_ERROR:
					switch (scsiDev.target->sense.asc)
					{
						case NO_SEEK_COMPLETE:
							scsiDev.data[0] = 0x15; // Seek Error
							break;
						case WRITE_ERROR_AUTO_REALLOCATION_FAILED:
							scsiDev.data[0] = 0x03; // Write fault
							break;
						default:
						case UNRECOVERED_READ_ERROR:
							scsiDev.data[0] = 0x11; // Uncorrectable read error
							break;
					}
					break;
				case ILLEGAL_REQUEST:
					switch (scsiDev.target->sense.asc)
					{
						case LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE:
							scsiDev.data[0] = 0x14; // Target sector not found
							break;
						case WRITE_PROTECTED:
							scsiDev.data[0] = 0x03; // Write fault
							break;
						default:
							scsiDev.data[0] = 0x20; // Invalid command
							break;
					}
					break;
				case NOT_READY:
					switch (scsiDev.target->sense.asc)
					{
						default:
						case MEDIUM_NOT_PRESENT:
							scsiDev.data[0] = 0x04; // Drive not ready
							break;
						case LOGICAL_UNIT_NOT_READY_INITIALIZING_COMMAND_REQUIRED:
							scsiDev.data[0] = 0x1A; // Format Error
							break;
					}
					break;
				default:
					scsiDev.data[0] = 0x11;  // Uncorrectable data error
					break;
			}

			scsiDev.data[1] = (scsiDev.cdb[1] & 0x20) | ((transfer.lba >> 16) & 0x1F);
			scsiDev.data[2] = transfer.lba >> 8;
			scsiDev.data[3] = transfer.lba;
		}
		else if (cfg->quirks == S2S_CFG_QUIRKS_OMTI)
		{
			// The response is completely non-standard.
			if (likely(allocLength > 12))
				allocLength = 12;
			else if (unlikely(allocLength < 4))
				allocLength = 4;
			if (cfg->deviceType != S2S_CFG_SEQUENTIAL)
				allocLength = 4;
			memset(scsiDev.data, 0, allocLength);
			if (scsiDev.target->sense.code == NO_SENSE)
			{
				// Nothing to report.
			}
			else if (scsiDev.target->sense.code == UNIT_ATTENTION &&
				cfg->deviceType == S2S_CFG_SEQUENTIAL)
			{
				scsiDev.data[0] = 0x10; // Tape exception
			}
			else if (scsiDev.target->sense.code == ILLEGAL_REQUEST)
			{
				if (scsiDev.target->sense.asc == LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE)
				{
					if (cfg->deviceType == S2S_CFG_SEQUENTIAL)
						scsiDev.data[0] = 0x10; // Tape exception
					else
						scsiDev.data[0] = 0x21; // Illegal Parameters
				}
				else if (scsiDev.target->sense.asc == INVALID_COMMAND_OPERATION_CODE)
				{
					scsiDev.data[0] = 0x20; // Invalid Command
				}
			}
			else if (scsiDev.target->sense.code == NOT_READY)
			{
				scsiDev.data[0] = 0x04; // Drive not ready
			}
			else if (scsiDev.target->sense.code == BLANK_CHECK)
			{
				scsiDev.data[0] = 0x10; // Tape exception
			}
			else
			{
				scsiDev.data[0] = 0x11; // Uncorrectable data error
			}
			scsiDev.data[1] = (scsiDev.cdb[1] & 0x60) | ((transfer.lba >> 16) & 0x1F);
			scsiDev.data[2] = transfer.lba >> 8;
			scsiDev.data[3] = transfer.lba;
			if (cfg->deviceType == S2S_CFG_SEQUENTIAL)
			{
				// For the tape drive there are 8 extra sense bytes.
				if (scsiDev.target->sense.code == BLANK_CHECK)
					scsiDev.data[11] = 0x88; // End of data recorded on the tape
				else if (scsiDev.target->sense.code == UNIT_ATTENTION)
					scsiDev.data[5] = 0x81; // Power On Reset occurred
				else if (scsiDev.target->sense.code == ILLEGAL_REQUEST &&
					 scsiDev.target->sense.asc == LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE)
					scsiDev.data[4] = 0x81; // File Mark detected
			}
		}
#ifdef PLATFORM_AS400
		else if (cfg->quirks == S2S_CFG_QUIRKS_AS400 && cfg->deviceType == S2S_CFG_FIXED)
		{
			// As specified by the SASI and SCSI1 standard.
			// Newer initiators won't be specifying 0 anyway.
			if (allocLength == 0) allocLength = 4;

			// If we receive a stand-alone REQUEST SENSE to a bad LUN we still need to respond
			// with LUN not supported. SCSI-2 Spec 7.5.3.
			if (scsiDev.lun && scsiDev.lastStatus != CHECK_CONDITION)
			{
				scsiDev.target->sense.code = ILLEGAL_REQUEST;
				scsiDev.target->sense.asc = LOGICAL_UNIT_NOT_SUPPORTED;
				transfer.lba = 0;
			}
			memset(scsiDev.data, 0, 256); // Max possible alloc length
			scsiDev.data[0] = 0x70;
			scsiDev.data[2] = scsiDev.target->sense.code & 0x0F;

			// LBA is Valid Information for direct access devices.
			scsiDev.data[3] = transfer.lba >> 24;
			scsiDev.data[4] = transfer.lba >> 16;
			scsiDev.data[5] = transfer.lba >> 8;
			scsiDev.data[6] = transfer.lba;


			// Additional bytes if there are errors to report
			scsiDev.data[7] = 0x18; // additional length
			scsiDev.data[12] = scsiDev.target->sense.asc >> 8;
			scsiDev.data[13] = scsiDev.target->sense.asc;

			if(scsiDev.target->sense.code == NOT_READY && scsiDev.target->sense.asc == LOGICAL_UNIT_NOT_READY_INITIALIZING_COMMAND_REQUIRED) {
				scsiDev.data[20]=1;
				scsiDev.data[21]=1;
			}
			if(scsiDev.target->sense.code == UNIT_ATTENTION && scsiDev.target->sense.asc == POWER_ON_RESET_OR_BUS_DEVICE_RESET_OCCURRED) {
				scsiDev.data[20]=1;
				scsiDev.data[21]=0x41;
			}
		}
#endif
		else
		{
			// As specified by the SASI and SCSI1 standard.
			// Newer initiators won't be specifying 0 anyway.
			if (allocLength == 0) allocLength = 4;

			memset(scsiDev.data, 0, 256); // Max possible alloc length
			scsiDev.data[0] = 0x70; // error code, Valid=0 by default
			scsiDev.data[2] = scsiDev.target->sense.code & 0x0F;
			if (cfg->deviceType == S2S_CFG_SEQUENTIAL)
			{
				scsiDev.data[0] = 0xF0; // Valid=1, tape always has meaningful info
				scsiDev.data[2] |= scsiDev.target->sense.filemark ? 1 << 7 : 0;
				scsiDev.data[2] |= scsiDev.target->sense.eom ? 1 << 6 : 0;
				scsiDev.data[2] |= scsiDev.target->sense.ili ? 1 << 5 : 0;
				scsiDev.data[3] = scsiDev.target->sense.info >> 24;
				scsiDev.data[4] = scsiDev.target->sense.info >> 16;
				scsiDev.data[5] = scsiDev.target->sense.info >> 8;
				scsiDev.data[6] = scsiDev.target->sense.info;
				// QIC drive vendor-specific sense bytes (Caliper/Sankyo/Wangtek)
				// Byte 9 bit 3: BOM (Beginning of Medium)
				scsiDev.data[9] = scsiDev.target->tapeBOM ? (1 << 3) : 0;
			}
			else if (scsiDev.target->sense.code == MEDIUM_ERROR
				|| scsiDev.target->sense.code == HARDWARE_ERROR
				|| scsiDev.target->sense.code == ABORTED_COMMAND)
			{
			// Valid=1 + LBA only for block-related errors
				scsiDev.data[0] = 0xF0;
				scsiDev.data[3] = transfer.lba >> 24;
				scsiDev.data[4] = transfer.lba >> 16;
				scsiDev.data[5] = transfer.lba >> 8;
				scsiDev.data[6] = transfer.lba;
			}
			// Additional bytes if there are errors to report
			scsiDev.data[7] = 10; // additional length
			scsiDev.data[12] = scsiDev.target->sense.asc >> 8;
			scsiDev.data[13] = scsiDev.target->sense.asc;
			if ((scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_EWSD))
			{
				/* EWSD seems not to want something behind additional length. (8 + 0x0e = 22) */
				allocLength=22;
				scsiDev.data[7] = 0x0e;
			}
		}

		// Silently truncate results. SCSI-2 spec 8.2.14.
		enter_DataIn(allocLength);

		// This is a good time to clear out old sense information.
		memset(&scsiDev.target->sense, 0, sizeof(ScsiSense));
	}
	else if (command == 0xA0)
	{
		// Report LUNs
		scsiDiskReportLUNs();
	}
	// Some old SCSI drivers do NOT properly support
	// unitAttention. eg. the Mac Plus would trigger a SCSI reset
	// on receiving the unit attention response on boot, thus
	// triggering another unit attention condition.
	else if (scsiDev.target->unitAttention &&
		scsiDev.target->unitAttentionStop == 0 &&
		((scsiDev.boardCfg.flags & S2S_CFG_ENABLE_UNIT_ATTENTION) ||
		(scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_EWSD)))
	{
		/* EWSD requires unitAttention to be sent only once. */
		if (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_EWSD)
		{
			scsiDev.target->unitAttentionStop = 1;
		}
		scsiDev.target->sense.code = UNIT_ATTENTION;
		scsiDev.target->sense.asc = scsiDev.target->unitAttention;

		// If initiator doesn't do REQUEST SENSE for the next command, then
		// data is lost.
		scsiDev.target->unitAttention = 0;

		enter_Status(CHECK_CONDITION);
	}
	else if (scsiDev.lun && (command < 0xD0))
	{
		scsiDev.target->sense.code = ILLEGAL_REQUEST;
		scsiDev.target->sense.asc = LOGICAL_UNIT_NOT_SUPPORTED;
		enter_Status(CHECK_CONDITION);
	}
	else if (command == 0x17 || command == 0x16)
	{
		doReserveRelease();
	}
	else if ((scsiDev.target->reservedId >= 0) &&
		(scsiDev.target->reservedId != scsiDev.initiatorId))
	{
		enter_Status(CONFLICT);
	}
	// Handle Toolbox commands, overriding other vendor commands if enabled
	else if (scsiToolboxEnabled() && scsiToolboxCommand())
	{
		// already handled
	}
	// Handle odd device types first that may override basic read and
	// write commands. Will fall-through to generic disk handling.
	// Some device specific vendor commands are located here instead
	// of in scsiVendorCommand()
	else if (((cfg->deviceType == S2S_CFG_OPTICAL) && scsiCDRomCommand()) ||
		((cfg->deviceType == S2S_CFG_SEQUENTIAL) && scsiTapeCommand()) ||
#ifdef ZULUSCSI_NETWORK
		((cfg->deviceType == S2S_CFG_AMIGAWIFI && amigaWifiCommand())) ||
		((cfg->deviceType == S2S_CFG_NETWORK && scsiNetworkCommand())) ||
#endif // ZULUSCSI_NETWORK
#ifdef ENABLE_AUDIO_STREAM
		((cfg->deviceType == S2S_CFG_AUDIO) && scsiAudioCommand()) ||
#endif // ENABLE_AUDIO_STREAM
		((cfg->deviceType == S2S_CFG_MO) && scsiMOCommand()))
	{
		// Already handled.
	}
	else if (scsiDiskCommand())
	{
		// Already handled.
		// check for the performance-critical read/write
		// commands ASAP.
	}
	else if (command == 0x1C)
	{
		scsiReceiveDiagnostic();
	}
	else if (command == 0x1D)
	{
		scsiSendDiagnostic();
	}
	else if (command == 0x3B)
	{
		scsiWriteBuffer();
	}
	else if (command == 0x3C)
	{
		scsiReadBuffer();
	}
	else if (scsiModeCommand())
	{
		// handled
	}
	else if (scsiVendorCommand())
	{
		// handled
	}
	else if (unlikely(command == 0x00))
    {
        // TEST UNIT READY
        doTestUnitReady();
    }
    else if (unlikely(!doTestUnitReady()))
    {
		// This should be last as it can override other commands 
        // Status and sense codes already set by doTestUnitReady
    }
	else
	{
		scsiDev.target->sense.code = ILLEGAL_REQUEST;
		scsiDev.target->sense.asc = INVALID_COMMAND_OPERATION_CODE;
		enter_Status(CHECK_CONDITION);
	}

	// Successful
	if (scsiDev.phase == COMMAND) // No status set, and not in DATA_IN
	{
		enter_Status(GOOD);
	}

}

static void doReserveRelease()
{
	int extentReservation = scsiDev.cdb[1] & 1;
	int thirdPty = scsiDev.cdb[1] & 0x10;
	int thirdPtyId = (scsiDev.cdb[1] >> 1) & 0x7;
	uint8_t command = scsiDev.cdb[0];

	int canRelease =
		(!thirdPty && (scsiDev.initiatorId == scsiDev.target->reservedId)) ||
			(thirdPty &&
				(scsiDev.target->reserverId == scsiDev.initiatorId) &&
				(scsiDev.target->reservedId == thirdPtyId)
			);

	if (extentReservation)
	{
#ifdef PLATFORM_AS400
		if (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_AS400 && scsiDev.target->cfg->deviceType == S2S_CFG_FIXED)
		{
			enter_Status(GOOD);
		}	
		else
#endif
		{
			// Not supported.
			scsiDev.target->sense.code = ILLEGAL_REQUEST;
			scsiDev.target->sense.asc = INVALID_FIELD_IN_CDB;
			enter_Status(CHECK_CONDITION);
		}
	}
	else if (command == 0x17) // release
	{
		if ((scsiDev.target->reservedId < 0) || canRelease)
		{
			scsiDev.target->reservedId = -1;
			scsiDev.target->reserverId = -1;
		}
		else
		{
			enter_Status(CONFLICT);
		}
	}
	else // assume reserve.
	{
		if ((scsiDev.target->reservedId < 0) || canRelease)
		{
			scsiDev.target->reserverId = scsiDev.initiatorId;
			if (thirdPty)
			{
				scsiDev.target->reservedId = thirdPtyId;
			}
			else
			{
				scsiDev.target->reservedId = scsiDev.initiatorId;
			}
		}
		else
		{
			// Already reserved by someone else!
			enter_Status(CONFLICT);
		}
	}
}

static uint32_t resetUntil = 0;

static void scsiReset()
{
	scsiDev.rstCount++;
	s2s_ledOff();

	scsiPhyReset();

	scsiDev.phase = BUS_FREE;
	scsiDev.atnFlag = 0;
	scsiDev.resetFlag = 0;
	scsiDev.selFlag = 0;
	scsiDev.lun = -1;
	scsiDev.compatMode = COMPAT_UNKNOWN;

	scsiDev.target = NULL;

#ifdef FORGE_LINK_TARGET7
	fl7s_on_bus_reset(&g_fl7s, &g_fl7_out_len, &g_fl7_out_pos);
#endif
	// ^ a started ID 7 serve ends at a bus reset, as at a close: the sampler that asked was
	// reset and will send no 09, and the next session must not get its stale state (C59).

	// Must refresh EVERY target's state here, not just whichever one
	// scsiDev.target happened to point to -- that pointer is NULL
	// whenever a reset arrives on an idle bus (e.g. the initiator's own
	// power-cycle, with Zulu itself staying powered via USB throughout),
	// which is the common case for a bus reset, not the exception. The
	// old single-pointer version silently skipped this whole block in
	// that case, leaving every target's unitAttention/sense/started
	// state exactly as the previous connection left it -- confirmed on
	// real AS/400 hardware: power-cycling the P02 while Zulu stayed up
	// produced a second connection whose first TEST UNIT READY returned
	// Status GOOD immediately (no unit attention, no not-ready, no
	// START STOP UNIT), unlike a fresh boot or a mid-connection reset.
	// Same bug shape as the scsiInit() fix earlier this session.
	for (int i = 0; i < S2S_MAX_TARGETS; ++i)
	{
		if (scsiDev.targets[i].unitAttention != POWER_ON_RESET)
		{
			scsiDev.targets[i].unitAttention = SCSI_BUS_RESET;
		}
		scsiDev.targets[i].reservedId = -1;
		scsiDev.targets[i].reserverId = -1;
#ifdef PLATFORM_AS400
		// Gated on EnableUnitAttention (not a specific quirk) and no
		// longer restricted to deviceType==FIXED, 2026-09-08: originally
		// AS400+disk-only, generalized because (a) this flag, not the
		// AS400 quirk specifically, is what actually means "this host
		// wants unit attention" everywhere else in this file, and (b)
		// doTestUnitReady() (ZuluSCSI_disk.cpp)'s !started gate and its
		// "preserve pending sense" branch apply uniformly to every
		// device type -- restricting this priming to FIXED disks left
		// tape/CD-ROM/etc. under the same quirk with the identical
		// latent bug this whole block exists to fix, just not yet
		// observed on hardware. Still #ifdef PLATFORM_AS400-gated: the
		// flag itself is general-purpose (e.g. the DOS preset also sets
		// it), but this exact priming mechanism has only been verified
		// against AS/400 hardware, not tested for any other quirk.
		if ((scsiDev.boardCfg.flags & S2S_CFG_ENABLE_UNIT_ATTENTION) &&
			(!scsiDev.targets[i].cfg || scsiDev.targets[i].cfg->deviceType != S2S_CFG_OPTICAL))
		{
			// CD-ROM excluded, 2026-09-09: see the matching comment in
			// process_SelectionPhase() -- real AS/400 Model 600 (RISC)
			// regression (SRC B1014507), confirmed by the maintainers.
			scsiDev.targets[i].sense.code = UNIT_ATTENTION;
			scsiDev.targets[i].sense.asc = POWER_ON_RESET_OR_BUS_DEVICE_RESET_OCCURRED;
			// Confirmed necessary on real hardware, 2026-09-08: without
			// this, doTestUnitReady() never revisits the sense/
			// unitAttention state re-armed above at all -- it gates its
			// entire CHECK_CONDITION/NOT_READY branch on !started, and
			// started stays 1 forever after the first successful START
			// STOP UNIT (set generically, not reset anywhere else).
			// Tried dropping this same day, on the theory that
			// correctly-raised unit attention alone (the fix a few
			// lines above / in process_SelectionPhase()) would be
			// enough and the full re-spin-up dance was unnecessary --
			// disproven immediately: TEST UNIT READY went back to
			// returning GOOD immediately on every reset after the
			// first, identical to the original bug, confirmed via a
			// live hardware retest.
			scsiDev.targets[i].started = 0;
		}
		else
#endif
		{
			scsiDev.targets[i].sense.code = NO_SENSE;
			scsiDev.targets[i].sense.asc = NO_ADDITIONAL_SENSE_INFORMATION;
		}

		if (g_force_sync > 0)
		{
			scsiDev.targets[i].syncPeriod = g_force_sync;
			scsiDev.targets[i].syncOffset = g_force_offset;
		}
		else
		{
			scsiDev.targets[i].syncOffset = 0;
			scsiDev.targets[i].syncPeriod = 0;
		}

		scsiDev.targets[i].busWidth = 0;
	}
	scsiDev.minSyncPeriod = 0;
	g_scsi_disconnect_state.active = 0;

	scsiDiskReset();

	scsiDev.postDataOutHook = NULL;

	scsiDev.sdUnderrunCount = 0;

	// Sleep to allow the bus to settle down a bit.
	// We must be ready again within the "Reset to selection time" of
	// 250ms.
	// There is no guarantee that the RST line will be negated by then.
	// NOTE: We could be connected and powered by USB for configuration,
	// in which case TERMPWR cannot be supplied, and reset will ALWAYS
	// be true. Therefore, the sleep here must be slow to avoid slowing
	// USB comms
	resetUntil = s2s_getTime_ms() + 2; // At least 1ms.
}

static void enter_SelectionPhase()
{
	// Ignore stale versions of this flag, but ensure we know the
	// current value if the flag is still set.
	scsiDev.atnFlag = 0;
	scsiDev.dataPtr = 0;
	scsiDev.savedDataPtr = 0;
	scsiDev.dataLen = 0;
	scsiDev.status = GOOD;
	scsiDev.phase = SELECTION;
	scsiDev.lun = -1;
	scsiDev.discPriv = 0;

	scsiDev.initiatorId = -1;
	scsiDev.target = NULL;

	transfer.blocks = 0;
	transfer.currentBlock = 0;

	scsiDev.postDataOutHook = NULL;

	scsiDev.needSyncNegotiationAck = 0;
	g_scsi_disconnect_state.active = 0;
	// C62 (deep review): do NOT reset g_scsi_sts_selection_initiator here. The
	// selection IRQ (pack_selection_status) captured the initiator's ID before scsiPoll got
	// here, and process_SelectionPhase reads it into scsiDev.initiatorId; a 0xFF here made
	// every initiatorId -1 (RESERVE stored -1, disconnect privilege lost). The phy resets it
	// itself on bus free, bus reset and reselection, so no "no selection" marker is lost.
}

static void process_SelectionPhase()
{
	// Selection delays.
	// Many SCSI1 samplers that use a 5380 chip need a delay of at least 1ms.
	// The Mac Plus boot-time (ie. rom code) selection abort time
	// is < 1ms and must have no delay (standard suggests 250ms abort time)
	// Most newer SCSI2 hosts don't care either way.
	// C61 (deep review): scsiDev.target is still NULL here (enter_SelectionPhase
	// cleared it and it is only set once we answer below), so read the quirks of the target
	// being selected, found from the latched selection status the same way as below.
	const TargetState* selTarget = NULL;
	{
		uint8_t selPeek = *SCSI_STS_SELECTED;
		if ((selPeek == 0) && (scsiDev.boardCfg.flags & S2S_CFG_ENABLE_SEL_LATCH))
		{
			selPeek = scsiDev.selFlag;
		}
		for (int i = 0; i < S2S_MAX_TARGETS; ++i)
		{
			if (scsiDev.targets[i].targetId == (selPeek & S2S_CFG_TARGET_ID_BITS))
			{
				selTarget = &scsiDev.targets[i];
				break;
			}
		}
	}
	if ((selTarget != NULL) && (selTarget->cfg->quirks == S2S_CFG_QUIRKS_XEBEC))
	{
		s2s_delay_ms(1); // Simply won't work if set to 0.
	}
	else if (scsiDev.boardCfg.selectionDelay == 255) // auto
	{
		if (scsiDev.compatMode < COMPAT_SCSI2)
		{
			s2s_delay_ms(1);
		}
	}
	else if (scsiDev.boardCfg.selectionDelay != 0)
	{
		s2s_delay_ms(scsiDev.boardCfg.selectionDelay);
	}

	uint8_t selStatus = *SCSI_STS_SELECTED;
	if ((selStatus == 0) && (scsiDev.boardCfg.flags & S2S_CFG_ENABLE_SEL_LATCH))
	{
		selStatus = scsiDev.selFlag;
	}

	int tgtIndex;
	TargetState* target = NULL;
	for (tgtIndex = 0; tgtIndex < S2S_MAX_TARGETS; ++tgtIndex)
	{
		if (scsiDev.targets[tgtIndex].targetId == (selStatus & S2S_CFG_TARGET_ID_BITS))
		{
			target = &scsiDev.targets[tgtIndex];
			break;
		}
	}
	if ((target != NULL) && (selStatus & 0x40))
	{
		// We've been selected!
		// Assert BSY - Selection success!
		// must happen within 200us (Selection abort time) of seeing our
		// ID + SEL.
		// (Note: the initiator will be waiting the "Selection time-out delay"
		// for our BSY response, which is actually a very generous 250ms)
		*SCSI_CTRL_BSY = 1;
		s2s_ledOn();

		scsiDev.target = target;

		// Do we enter MESSAGE OUT immediately ? SCSI 1 and 2 standards says
		// move to MESSAGE OUT if ATN is true before we assert BSY.
		// The initiator should assert ATN with SEL.
		scsiDev.atnFlag = selStatus & 0x80;


		// Unit attention breaks many older SCSI hosts (eg. the Mac Plus --
		// see the comment on the general UA-check below). The historical
		// fix was to disable it completely for any host selecting without
		// ATN (old-style SCSI-1-ish selection), regardless of the
		// configured EnableUnitAttention setting.
		//
		// That default is too broad: EnableUnitAttention=Yes is explicitly
		// set by two presets in ZuluSCSISettings -- AS/400 (deviceInitAS400())
		// and DOS (SYS_PRESET_DOS) -- and both are known to select without
		// ATN despite being SCSI-2-capable hosts that do want unit
		// attention reported, not broken SCSI-1 hosts. A real AS/400 P02's
		// Ancot baseline shows its SP correctly consuming a genuine
		// POWER_ON_RESET unit attention on its very first TEST UNIT READY
		// (sense 06/2900) before a second TUR/REQUEST SENSE cycle reports
		// NOT_READY (02/0402) and START STOP UNIT succeeds -- unconditional
		// suppression here wiped the pending condition on the very first
		// SELECT (for INQUIRY, before any command byte is read), so Zulu's
		// first TUR jumped straight to the "second cycle" NOT_READY state
		// the real disk only reaches after the initiator has already seen
		// and cleared the power-on attention, a sequence this SP's ROM
		// likely never anticipated (confirmed via real-hardware Ancot
		// trace comparison, 2026-09-07). Gating on the same
		// EnableUnitAttention flag the deferred UA-check below already
		// uses -- instead of a platform-specific quirks check -- fixes
		// this for AS/400 and for DOS (same preset, not yet confirmed on
		// real DOS hardware, but the identical shape of bug) without
		// touching the historical default (EnableUnitAttention=No) that
		// still protects genuinely ATN-less-and-broken hosts.
		if (!scsiDev.atnFlag)
		{
			// CD-ROM excluded, 2026-09-09: a real AS/400 Model 600 (RISC)
			// manual D-mode IPL from CD-ROM (SRC B1014507, a generic
			// CD-ROM read error) regressed once this exemption's gate
			// widened from AS400+FIXED-only to EnableUnitAttention-for-
			// any-device-type -- confirmed by the maintainers reverting
			// just this file's #952 changes and reporting it fixes their
			// hardware. Unlike disk (and tape, per the P02 baseline),
			// real optical drives auto-spin-up with no expected START
			// UNIT handshake; this RISC boot ROM's minimal CD-boot path
			// evidently can't tolerate a real unit attention it never
			// saw before. Keep the fix for FIXED/SEQUENTIAL (still
			// needed, still hardware-confirmed); suppress again for
			// OPTICAL specifically rather than reverting to disk-only.
			if (!(scsiDev.boardCfg.flags & S2S_CFG_ENABLE_UNIT_ATTENTION) ||
				target->cfg->deviceType == S2S_CFG_OPTICAL)
			{
				target->unitAttention = 0;
			}
			scsiDev.compatMode = COMPAT_SCSI1;
		}
		else if (!(scsiDev.boardCfg.flags & S2S_CFG_ENABLE_SCSI2))
		{
			scsiDev.compatMode = COMPAT_SCSI2_DISABLED;
		}
		else
		{
			scsiDev.compatMode = COMPAT_SCSI2;
		}

		scsiDev.selCount++;


		// Save our initiator now that we're no longer in a time-critical
		// section.
		// SCSI1/SASI initiators may not set their own ID.
		scsiDev.initiatorId =
			(g_scsi_sts_selection_initiator < S2S_MAX_TARGETS) ? g_scsi_sts_selection_initiator : -1;

		// Wait until the end of the selection phase.
		uint32_t selTimerBegin = s2s_getTime_ms();
		while (likely(!scsiDev.resetFlag))
		{
			if (!scsiStatusSEL())
			{
				break;
			}
			else if (s2s_elapsedTime_ms(selTimerBegin) >= 10 &&
				scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_XEBEC)
			{
				// XEBEC hosts may not bother releasing SEL at all until
				// just before the command ends.
				break;
			}
			else if (s2s_elapsedTime_ms(selTimerBegin) >= 250)
			{
				*SCSI_CTRL_BSY = 0;
				scsiDev.resetFlag = 1;
				break;
			}
		}

		scsiDev.phase = COMMAND;
	}
	else if (!selStatus)
	{
		scsiDev.phase = BUS_BUSY;
	}
	scsiDev.selFlag = 0;
}

static void process_MessageOut()
{
	int wasNeedSyncNegotiationAck = scsiDev.needSyncNegotiationAck;
	scsiDev.needSyncNegotiationAck = 0; // Successful on -most- messages.

	scsiEnterPhase(MESSAGE_OUT);

	scsiDev.atnFlag = 0;
	scsiDev.msgOut = scsiReadByte();
	scsiDev.msgCount++;

	if (scsiParityError() &&
		(scsiDev.boardCfg.flags & S2S_CFG_ENABLE_PARITY))
	{
		// Skip the remaining message bytes, and then start the MESSAGE_OUT
		// phase again from the start. The initiator will re-send the
		// same set of messages.
		while (scsiStatusATN() && !scsiDev.resetFlag)
		{
			scsiReadByte();
		}

		// Go-back and try the message again.
		scsiDev.atnFlag = 1;
	}
	else if (scsiDev.msgOut == 0x00)
	{
		// COMMAND COMPLETE. but why would the target be receiving this ? nfi.
		enter_BusFree();
	}
	else if (scsiDev.msgOut == 0x06)
	{
		// ABORT
		scsiDiskReset();
		enter_BusFree();
	}
	else if (scsiDev.msgOut == 0x0C)
	{
		// BUS DEVICE RESET

		scsiDiskReset();

		scsiDev.target->unitAttention = SCSI_BUS_RESET;

		// ANY initiator can reset the reservation state via this message.
		scsiDev.target->reservedId = -1;
		scsiDev.target->reserverId = -1;

		// Cancel any sync negotiation
		scsiDev.target->syncOffset = 0;
		scsiDev.target->syncPeriod = 0;

		enter_BusFree();
	}
	else if (scsiDev.msgOut == 0x05)
	{
		// Initiator Detected Error
		// Ignore for now
	}
	else if (scsiDev.msgOut == 0x0F)
	{
		// INITIATE RECOVERY
		// Ignore for now
	}
	else if (scsiDev.msgOut == 0x10)
	{
		// RELEASE RECOVERY
		// Ignore for now
		enter_BusFree();
	}
	else if (scsiDev.msgOut == MSG_REJECT)
	{
		// Message Rejected
		// Oh well.

		if (wasNeedSyncNegotiationAck)
		{
			scsiDev.target->syncOffset = 0;
			scsiDev.target->syncPeriod = 0;
		}
	}
	else if (scsiDev.msgOut == 0x08)
	{
		// NOP
	}
	else if (scsiDev.msgOut == 0x09)
	{
		// Message Parity Error
		// Go back and re-send the last message.
		scsiDev.phase = MESSAGE_IN;

		if (wasNeedSyncNegotiationAck)
		{
			scsiDev.target->syncOffset = 0;
			scsiDev.target->syncPeriod = 0;
		}
	}
	else if (scsiDev.msgOut & 0x80) // 0x80 -> 0xFF
	{
		// IDENTIFY
		if ((scsiDev.msgOut & 0x18) || // Reserved bits set.
			(scsiDev.msgOut & 0x20))  // We don't have any target routines!
		{
			messageReject();
		}

		scsiDev.lun = scsiDev.msgOut & 0x7;
		scsiDev.discPriv = 
			((scsiDev.msgOut & 0x40) && (scsiDev.initiatorId >= 0))
				? 1 : 0;
	}
	else if (scsiDev.msgOut >= 0x20 && scsiDev.msgOut <= 0x2F)
	{
		// Two byte message. Read the parameter byte either way so we stay in
		// sync with the initiator regardless of whether we accept the message.
		uint8_t param = scsiReadByte();
		(void)param;

		if (scsiDev.msgOut == 0x23) {
			// Ignore Wide Residue. We're only 8 bit anyway.
		} else if (scsiDev.msgOut == MSG_SIMPLE_QUEUE_TAG ||
		           scsiDev.msgOut == MSG_HEAD_OF_QUEUE_TAG ||
		           scsiDev.msgOut == MSG_ORDERED_QUEUE_TAG) {
			// Tagged command queueing. SCSI2SD is non-disconnecting today
			// (scsiDisconnect() / scsiReconnect() further down in this file
			// are commented out), so the SPC-3 §6.5 tag-echo path -- which
			// the spec only specifies at RESELECTION immediately following a
			// disconnect -- is unreachable. Accepting the tag here without
			// rejecting the message is sufficient for compliant initiators
			// (notably AS/400 9406-class IOAs that issue every command
			// tagged once CmdQue=1 is advertised in std INQUIRY). The tag
			// value is intentionally discarded; if this firmware ever gains
			// disconnect support, store it on the TargetState here and echo
			// `MSG_SIMPLE_QUEUE_TAG + tag` in the MESSAGE_IN that follows
			// the RESELECTION phase.
		} else {
			messageReject();
		}
	}
	else if (scsiDev.msgOut == 0x01)
	{
		int i;

		// Extended message. These include speed negotiation
		int msgLen = scsiReadByte();
		if (msgLen == 0) msgLen = 256;
		uint8_t extmsg[256];
		for (i = 0; i < msgLen && !scsiDev.resetFlag; ++i)
		{
			// Discard bytes.
			extmsg[i] = scsiReadByte();
		}

		if (extmsg[0] == 3 && msgLen == 2) // Wide Data Request
		{
			uint8_t width = extmsg[1]; // Transfer width exponent, 0: 8-bit, 1: 16-bit, 2: 32-bit
			if (width > scsiDev.boardCfg.busWidth)
			{
				width = scsiDev.boardCfg.busWidth;
			}
			scsiDev.target->busWidth = width;

			// Reply with the width we support
			scsiEnterPhase(MESSAGE_IN);
			uint8_t WDTR[] = {0x01, 0x02, 0x03, width};
			scsiWrite(WDTR, sizeof(WDTR));

			// SDTR becomes invalidated.
			scsiDev.target->syncOffset = 0;
			scsiDev.target->syncPeriod = 0;
		}
		else if (extmsg[0] == 1 && msgLen == 3) // Synchronous data request
		{
			int oldPeriod = scsiDev.target->syncPeriod;
			int oldOffset = scsiDev.target->syncOffset;

			int transferPeriod = extmsg[1];	//This is actually 1/4 of the duration of transfer speed in ns
			int offset = extmsg[2];	//Req/Ack offset

			if ((
					(transferPeriod > 0) &&
					(transferPeriod < scsiDev.minSyncPeriod)) ||
				(scsiDev.minSyncPeriod == 0))
			{
				scsiDev.minSyncPeriod = transferPeriod;
			}

			if ((transferPeriod > 80) || // 320ns, 3.125MB/s
				// Amiga A590 (WD33C93 chip) only does 3.5MB/s sync
				// After 80 we start to run out of bits in the fpga timing
				// register.
				(transferPeriod == 0) ||
				(offset == 0) ||
				((scsiDev.boardCfg.scsiSpeed != S2S_CFG_SPEED_NoLimit) &&
					(scsiDev.boardCfg.scsiSpeed <= S2S_CFG_SPEED_ASYNC_50)))
			{
				scsiDev.target->syncOffset = 0;
				scsiDev.target->syncPeriod = 0;
			} else {
				scsiDev.target->syncOffset = offset <= 15 ? offset : 15;
				// FAST20 / 50ns / 20MHz is disabled for now due to
				// data corruption while reading data. We can count the
				// ACK's correctly, but can't save the data to a register
				// before it changes. (ie. transferPeriod == 12)
				//****TODO Try to get higher speeds
				//The Adaptec AHA2940-UW seems to support 10, 13.4, 16 and 20 MHz
				//The speeds above correspond to syncPeriod values of 25, 18, 15 and 12 (maybe, the last 3 are truncated)
				//We will set the syncPeriod and syncOffset to the fastest we 
				//can support if the initiator requests a faster speed
				if (scsiDev.boardCfg.scsiSpeed == S2S_CFG_SPEED_SYNC_20 || scsiDev.boardCfg.scsiSpeed == S2S_CFG_SPEED_NoLimit)
				{
					if (transferPeriod <= g_max_sync_20_period)
						scsiDev.target->syncPeriod = g_max_sync_20_period;
					else 
						scsiDev.target->syncPeriod = transferPeriod;	
				}
				else if (scsiDev.boardCfg.scsiSpeed >= S2S_CFG_SPEED_SYNC_10)
				{
					if (transferPeriod <= g_max_sync_10_period)
						scsiDev.target->syncPeriod = g_max_sync_10_period;
					else
						scsiDev.target->syncPeriod = transferPeriod;
				}
				else if (scsiDev.boardCfg.scsiSpeed == S2S_CFG_SPEED_SYNC_5)
				{
					if (transferPeriod <= g_max_sync_5_period)
						scsiDev.target->syncPeriod = g_max_sync_5_period;
					else
						scsiDev.target->syncPeriod = transferPeriod;
				}
			}

			//Reply back with negotiation speed (if different from previous one)
			//Reply should be the same as request (if we support it) or slower if we don't
			//Slower in this context means a higher syncPeriod or lower syncOffset 
			if (transferPeriod != oldPeriod ||
				scsiDev.target->syncPeriod != oldPeriod ||
				offset != oldOffset ||
				scsiDev.target->syncOffset != oldOffset ||
				!wasNeedSyncNegotiationAck) // Don't get into infinite loops negotiating.
			{
				scsiEnterPhase(MESSAGE_IN);
				uint8_t SDTR[] = {0x01, 0x03, 0x01, scsiDev.target->syncPeriod, scsiDev.target->syncOffset};
				scsiWrite(SDTR, sizeof(SDTR));
				scsiDev.needSyncNegotiationAck = 1; // Check if this message is rejected.
				scsiDev.sdUnderrunCount = 0;  // reset counter, may work now.

				// Set to the theoretical speed, then adjust if we measure lower
				// actual speeds.
				scsiDev.hostSpeedKBs = s2s_getScsiRateKBs();
				scsiDev.hostSpeedMeasured = 0;
			}
		}
		else
		{
			// Not supported
			messageReject();
		}
	}
	else
	{
		messageReject();
	}

	// Re-check the ATN flag in case it stays asserted.
	scsiDev.atnFlag |= scsiStatusATN();

	if (!scsiDev.atnFlag)
	{
		// Message wasn't rejected!
		scsiDev.needSyncNegotiationAck = 0;
	}
}

void scsiPoll(void)
{
	if (resetUntil != 0 && resetUntil > s2s_getTime_ms())
	{
		return;
	}
	resetUntil = 0;

	if (unlikely(scsiDev.resetFlag))
	{
		scsiReset();
		// Still in reset phase for a few ms.
		// Do not try and process any commands.
		return;
	}

	switch (scsiDev.phase)
	{
	case BUS_FREE:
		if (scsiStatusBSY())
		{
			scsiDev.phase = BUS_BUSY;
		}
		// The Arbitration phase is optional for SCSI1/SASI hosts if there is only
		// one initiator in the chain. Support this by moving
		// straight to selection if SEL is asserted.
		// ie. the initiator won't assert BSY and it's own ID before moving to selection.
		else if (scsiDev.selFlag || *SCSI_STS_SELECTED)
		{
			enter_SelectionPhase();
		}
	break;

	case BUS_BUSY:
		// Someone is using the bus. Perhaps they are trying to
		// select us.
		if (scsiDev.selFlag || *SCSI_STS_SELECTED)
		{
			enter_SelectionPhase();
		}
		else if (!scsiStatusBSY())
		{
			scsiDev.phase = BUS_FREE;
		}
	break;

	case ARBITRATION:
		// TODO Support reselection.
		break;

	case SELECTION:
		process_SelectionPhase();
	break;

	case RESELECTION:
		// Command-specific code performs the actual reconnect once it is ready
		// to resume the disconnected command.
	break;

	case COMMAND:
		// Do not check ATN here. SCSI 1 & 2 initiators must set ATN
		// and SEL together upon entering the selection phase if they
		// want to send a message (IDENTIFY) immediately.
		if (scsiDev.atnFlag)
		{
			process_MessageOut();
		}
		else
		{
			process_Command();
		}
	break;

	case DATA_IN:
		scsiDev.atnFlag |= scsiStatusATN();
		if (scsiDev.atnFlag)
		{
			process_MessageOut();
		}
		else
		{
			process_DataIn();
		}
	break;

	case DATA_OUT:
		scsiDev.atnFlag |= scsiStatusATN();
		if (scsiDev.atnFlag)
		{
			process_MessageOut();
		}
		else
		{
			process_DataOut();
		}
	break;

	case STATUS:
		scsiDev.atnFlag |= scsiStatusATN();
		if (scsiDev.atnFlag)
		{
			process_MessageOut();
		}
		else
		{
			process_Status();
		}
	break;

	case MESSAGE_IN:
		scsiDev.atnFlag |= scsiStatusATN();
		if (scsiDev.atnFlag)
		{
			process_MessageOut();
		}
		else
		{
			process_MessageIn(1);
		}

	break;

	case MESSAGE_OUT:
		process_MessageOut();
	break;
	}
}

void scsiInit()
{
	static int firstInit = 1;

	scsiDev.atnFlag = 0;
	scsiDev.resetFlag = 1;
	scsiDev.selFlag = 0;
	scsiDev.phase = BUS_FREE;
	scsiDev.target = NULL;
	scsiDev.compatMode = COMPAT_UNKNOWN;
	scsiDev.hostSpeedKBs = 0;
	scsiDev.hostSpeedMeasured = 0;

	int i;
	for (i = 0; i < S2S_MAX_TARGETS; ++i)
	{
		const S2S_TargetCfg* cfg = s2s_getConfigByIndex(i);
		if (cfg && (cfg->scsiId & S2S_CFG_TARGET_ENABLED))
		{
			scsiDev.targets[i].targetId = cfg->scsiId & S2S_CFG_TARGET_ID_BITS;
			scsiDev.targets[i].cfg = cfg;

			scsiDev.targets[i].liveCfg.bytesPerSector = cfg->bytesPerSector;
			scsiDev.targets[i].liveCfg.tapeDensity = cfg->tapeDensity;
			scsiDev.targets[i].liveCfg.tapeBufferedMode = cfg->tapeBufferedMode;
			scsiDev.targets[i].liveCfg.disconnectTimeLimit = 0;
		}
		else
		{
			scsiDev.targets[i].targetId = 0xff;
			scsiDev.targets[i].cfg = NULL;
		}
		scsiDev.targets[i].reservedId = -1;
		scsiDev.targets[i].reserverId = -1;
		if (firstInit)
		{
			if ((cfg->deviceType == S2S_CFG_MO) && (scsiDev.target->cfg->quirks == S2S_CFG_QUIRKS_EWSD))
			{
				scsiDev.targets[i].unitAttention = POWER_ON_RESET_OR_BUS_DEVICE_RESET_OCCURRED;
			} else
			{
				scsiDev.targets[i].unitAttention = POWER_ON_RESET;
			}
		}
		else
		{
			scsiDev.targets[i].unitAttention = PARAMETERS_CHANGED;
		}
		// reset sense
		memset(&scsiDev.targets[i].sense, 0, sizeof(scsiDev.targets[i].sense));

		if (g_force_sync > 0)
		{
			scsiDev.targets[i].syncPeriod = g_force_sync;
			scsiDev.targets[i].syncOffset = g_force_offset;
		}
		else
		{
			scsiDev.targets[i].syncOffset = 0;
			scsiDev.targets[i].syncPeriod = 0;
		}

#ifdef PLATFORM_AS400
		// Gated on EnableUnitAttention, not a specific quirk, and not
		// restricted to deviceType==FIXED -- see the matching comment in
		// scsiReset() for the full reasoning (same generalization, same
		// investigation, 2026-09-08).
		if ((scsiDev.boardCfg.flags & S2S_CFG_ENABLE_UNIT_ATTENTION) &&
			(!cfg || cfg->deviceType != S2S_CFG_OPTICAL))
		{
			// scsiDev.target (the currently-selected-target pointer) is
			// NULL throughout this whole init loop -- these must prime
			// this target's own (array-indexed) sense state, or the write
			// silently goes nowhere and this priming never actually takes
			// effect. Found alongside the ATN/unitAttention fix above,
			// same investigation.
			// CD-ROM excluded, 2026-09-09: see the matching comment in
			// process_SelectionPhase() -- real AS/400 Model 600 (RISC)
			// regression (SRC B1014507), confirmed by the maintainers.
			scsiDev.targets[i].sense.code = UNIT_ATTENTION;
			scsiDev.targets[i].sense.asc = POWER_ON_RESET_OR_BUS_DEVICE_RESET_OCCURRED;
			scsiDev.targets[i].started = 0;
		}
		else
#endif
		{
			scsiDev.targets[i].sense.code = NO_SENSE;
			scsiDev.targets[i].sense.asc = NO_ADDITIONAL_SENSE_INFORMATION;
			// Always "start" the device. Many systems (eg. Apple System 7)
			// won't respond properly to
			// LOGICAL_UNIT_NOT_READY_INITIALIZING_COMMAND_REQUIRED sense
			// code
			scsiDev.targets[i].started = 1;
		}
	}
	firstInit = 0;
}

void scsiDisconnect()
{
	if (!scsiDev.target || !scsiDev.discPriv || g_scsi_disconnect_state.active)
	{
		return;
	}

	g_scsi_disconnect_state.active = 1;
	g_scsi_disconnect_state.phase = scsiDev.phase;
	g_scsi_disconnect_state.cdbLen = scsiDev.cdbLen;
	scsiDev.savedDataPtr = scsiDev.dataPtr;

	scsiEnterPhase(MESSAGE_IN);
	scsiWriteByte(0x02); // SAVE DATA POINTER
	scsiWriteByte(0x04); // DISCONNECT

	if (scsiDev.resetFlag)
	{
		g_scsi_disconnect_state.active = 0;
		return;
	}

	enter_BusFree();
	scsiDev.phase = RESELECTION;
	scsiDev.cdbLen = g_scsi_disconnect_state.cdbLen;
	g_scsi_disconnect_state.disconnectTime_ms = s2s_getTime_ms();
}

int scsiReconnect()
{
	if (!g_scsi_disconnect_state.active ||
		!scsiDev.target ||
		scsiDev.initiatorId < 0 ||
		scsiDev.resetFlag)
	{
		return 0;
	}

	// SCSI-2 6.6.6: the target shall not participate in another
	// ARBITRATION phase for at least a disconnection delay (200us,
	// table 7) or the disconnect time limit negotiated via the
	// Disconnect-Reconnect mode page (8.3.3.2), whichever is greater.
	// Our timer's granularity is 1ms; rounding up is harmless,
	// arbitrating early is not.
	uint32_t negotiatedLimitUs =
		(uint32_t)scsiDev.target->liveCfg.disconnectTimeLimit * 100;
	uint32_t requiredWaitUs = (negotiatedLimitUs > 200) ? negotiatedLimitUs : 200;
	uint32_t requiredWaitMs = (requiredWaitUs + 999) / 1000;

	uint32_t elapsedSinceDisconnect_ms =
		s2s_elapsedTime_ms(g_scsi_disconnect_state.disconnectTime_ms);
	if (elapsedSinceDisconnect_ms < requiredWaitMs)
	{
		s2s_delay_ms(requiredWaitMs - elapsedSinceDisconnect_ms);
	}

	if (!scsiPhyReselect(scsiDev.target->targetId, scsiDev.initiatorId))
	{
		return 0;
	}

	// Prepare for the initial IDENTIFY message.
	scsiDev.atnFlag = 0;
	scsiDev.selFlag = 0;
	scsiDev.cdbLen = g_scsi_disconnect_state.cdbLen;
	scsiDev.dataPtr = scsiDev.savedDataPtr;
	scsiEnterPhase(MESSAGE_IN);
	scsiWriteByte(0x80 | (scsiDev.lun & 0x7));

	if (scsiDev.resetFlag)
	{
		g_scsi_disconnect_state.active = 0;
		return 0;
	}

	// SCSI-2 6.2.1(f): if ATN is true, the initiator wants to send a
	// message before we resume the disconnected command -- most likely
	// ABORT, ABORT TAG, or BUS DEVICE RESET for this exact nexus right
	// at reconnection (note 29 anticipates exactly this). Service it the
	// same way every other phase already does in scsiPoll(), rather than
	// silently discarding it.
	scsiDev.atnFlag |= scsiStatusATN();
	while (scsiDev.atnFlag && !scsiDev.resetFlag)
	{
		process_MessageOut();
		scsiDev.atnFlag |= scsiStatusATN();
	}

	if (scsiDev.resetFlag || scsiDev.phase == BUS_FREE)
	{
		// A reset, or the message just handled (e.g. ABORT, BUS DEVICE
		// RESET), already tore down this nexus -- do not resume it.
		g_scsi_disconnect_state.active = 0;
		return 0;
	}

	scsiDev.phase = g_scsi_disconnect_state.phase;
	scsiDev.cdbLen = g_scsi_disconnect_state.cdbLen;
	scsiDev.dataPtr = scsiDev.savedDataPtr;
	if (scsiDev.phase >= 0)
	{
		scsiEnterPhase(scsiDev.phase);
	}

	g_scsi_disconnect_state.active = 0;
	return 1;
}
