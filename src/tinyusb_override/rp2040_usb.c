/* ⚠ FORGE LINK OVERRIDE COPY — NOT OUR CODE. Verbatim TinyUSB rp2040_usb.c (MIT, header below), as
 * shipped in framework-arduinopico's pico-sdk/lib/tinyusb, with TWO additions: NDEBUG, just below, and
 * the RE-ARM GUARD (fl_ep_abort_before_rearm and its one call in hw_endpoint_xfer_start; the
 * hardware/sync.h include), added 2026-09-29 for the card-mode panic.
 * Same build-system override as cdc_device.c: the linker prefers this object over libpico.a's.
 *
 * ⛔ THIS FILE IS ON THE USB INTERRUPT PATH. Every function marked __tusb_irq_path_func runs from
 * the USB IRQ and must live in RAM, and must not call into flash beyond what upstream's own copy
 * already does. tools/check-irq-ram.sh checks both before any flash: the symbols, and (via
 * tools/irq_call_targets.py) every call target against tools/irq-flash-edges.baseline.
 *
 * HISTORY (N6 step 1, 2026-09-21). This copy used to carry the re-arm cadence instrumentation
 * (fl_note_arm and the fl_ep_* counters) and the FORGE_LINK_DBUF double-buffered OUT
 * experiment (C95-). Both were removed: the cadence question is answered, and the
 * double-buffer path is banned - W49 is its crash log, a panic in the USB buff-status IRQ 4 ms
 * after arming. Both remain in git history (last present at 5bcb6a3) if they are ever needed.
 *
 * WHY NDEBUG. libpico.a compiles this file with -DNDEBUG (tools/libpico/make-libpico.sh:26 in the
 * framework), so upstream's sync_ep_buffer has no asserts. Our copy was compiled without it, so its
 * two assert()s survived as calls to __assert_func, IN FLASH, from a RAM IRQ function: the one flash
 * edge this build had that upstream's does not (N6 design rev 2 §1.4). Defining it here puts this
 * translation unit back in upstream's shape. assert.h is re-included after the headers so the
 * macro follows NDEBUG even if something force-included it earlier.
 */
#ifndef NDEBUG
#define NDEBUG
#endif
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2021 Ha Thach (tinyusb.org) for Double Buffered
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUSB_MCU == OPT_MCU_RP2040

#include <stdlib.h>
#include "rp2040_usb.h"
#include <assert.h>   /* FORGE LINK: re-arm assert() under the NDEBUG above */
#include "hardware/sync.h"  /* FORGE LINK: save_and_disable_interrupts, the re-arm guard */

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPE
//--------------------------------------------------------------------+
static void _hw_endpoint_xfer_sync(struct hw_endpoint* ep);

#if TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
  static bool e15_is_bulkin_ep(struct hw_endpoint* ep);
  static bool e15_is_critical_frame_period(struct hw_endpoint* ep);
#else
  #define e15_is_bulkin_ep(x)             (false)
  #define e15_is_critical_frame_period(x) (false)
#endif

// if usb hardware is in host mode
TU_ATTR_ALWAYS_INLINE static inline bool is_host_mode(void) {
  return (usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS) ? true : false;
}

//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+
// Provide own byte by byte memcpy as not all copies are aligned
static void unaligned_memcpy(void *dst, const void *src, size_t n) {
  uint8_t *dst_byte = (uint8_t*)dst;
  const uint8_t *src_byte = (const uint8_t*)src;
  while (n--) {
    *dst_byte++ = *src_byte++;
  }
}

void rp2040_usb_init(void) {
  // Reset usb controller
  reset_block(RESETS_RESET_USBCTRL_BITS);
  unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

#ifdef __GNUC__
  // Clear any previous state just in case
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#if __GNUC__ > 6
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
#endif
  memset(usb_dpram, 0, sizeof(*usb_dpram));
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

  // Mux the controller to the onboard usb phy
  usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

  TU_LOG2_INT(sizeof(hw_endpoint_t));
}

void __tusb_irq_path_func(hw_endpoint_reset_transfer)(struct hw_endpoint* ep) {
  ep->active = false;
  ep->remaining_len = 0;
  ep->xferred_len = 0;
  ep->user_buf = 0;
}

void __tusb_irq_path_func(_hw_endpoint_buffer_control_update32)(struct hw_endpoint* ep, uint32_t and_mask,
                                                               uint32_t or_mask) {
  uint32_t value = 0;

  if (and_mask) {
    value = *ep->buffer_control & and_mask;
  }

  if (or_mask) {
    value |= or_mask;
    if (or_mask & USB_BUF_CTRL_AVAIL) {
      if (*ep->buffer_control & USB_BUF_CTRL_AVAIL) {
        panic("ep %02X was already available", ep->ep_addr);
      }
      *ep->buffer_control = value & ~USB_BUF_CTRL_AVAIL;
      // 4.1.2.5.1 Con-current access: 12 cycles (should be good for 48*12Mhz = 576Mhz) after write to buffer control
      // Don't need delay in host mode as host is in charge
      if ( !is_host_mode()) {
        busy_wait_at_least_cycles(12);
      }
    }
  }

  *ep->buffer_control = value;
}

// prepare buffer, return buffer control
static uint32_t __tusb_irq_path_func(prepare_ep_buffer)(struct hw_endpoint* ep, uint8_t buf_id) {
  uint16_t const buflen = tu_min16(ep->remaining_len, ep->wMaxPacketSize);
  ep->remaining_len = (uint16_t) (ep->remaining_len - buflen);

  uint32_t buf_ctrl = buflen | USB_BUF_CTRL_AVAIL;

  // PID
  buf_ctrl |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
  ep->next_pid ^= 1u;

  if (!ep->rx) {
    // Copy data from user buffer to hw buffer
    unaligned_memcpy(ep->hw_data_buf + buf_id * 64, ep->user_buf, buflen);
    ep->user_buf += buflen;

    // Mark as full
    buf_ctrl |= USB_BUF_CTRL_FULL;
  }

  // Is this the last buffer? Only really matters for host mode. Will trigger
  // the trans complete irq but also stop it polling. We only really care about
  // trans complete for setup packets being sent
  if (ep->remaining_len == 0) {
    buf_ctrl |= USB_BUF_CTRL_LAST;
  }

  if (buf_id) buf_ctrl = buf_ctrl << 16;

  return buf_ctrl;
}

// Prepare buffer control register value
void __tusb_irq_path_func(hw_endpoint_start_next_buffer)(struct hw_endpoint* ep) {
  uint32_t ep_ctrl = *ep->endpoint_control;

  // always compute and start with buffer 0
  uint32_t buf_ctrl = prepare_ep_buffer(ep, 0) | USB_BUF_CTRL_SEL;

  // For now: skip double buffered for OUT endpoint in Device mode, since
  // host could send < 64 bytes and cause short packet on buffer0
  // NOTE: this could happen to Host mode IN endpoint
  // Also, Host mode "interrupt" endpoint hardware is only single buffered,
  // NOTE2: Currently Host bulk is implemented using "interrupt" endpoint
  bool const is_host = is_host_mode();
  bool const force_single = (!is_host && !tu_edpt_dir(ep->ep_addr)) ||
                            (is_host && tu_edpt_number(ep->ep_addr) != 0);

  if (ep->remaining_len && !force_single) {
    // Use buffer 1 (double buffered) if there is still data
    // TODO: Isochronous for buffer1 bit-field is different than CBI (control bulk, interrupt)

    buf_ctrl |= prepare_ep_buffer(ep, 1);

    // Set endpoint control double buffered bit if needed
    ep_ctrl &= ~EP_CTRL_INTERRUPT_PER_BUFFER;
    ep_ctrl |= EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER;
  } else {
    // Single buffered since 1 is enough
    ep_ctrl &= ~(EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
    ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;
  }

  *ep->endpoint_control = ep_ctrl;

  TU_LOG(3, "  Prepare BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n", tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));

  // Finally, write to buffer_control which will trigger the transfer
  // the next time the controller polls this dpram address
  _hw_endpoint_buffer_control_set_value32(ep, buf_ctrl);
}

/* ---- FORGE LINK: the re-arm guard ------------
 * A host CLEAR_FEATURE(ENDPOINT_HALT) on a bulk OUT endpoint whose read is still armed makes usbd zero
 * its busy flag (usbd_edpt_clear_stall ignores its own "only clear if currently stalled"), the class
 * re-arms, and upstream arrives here with the buffer still AVAILABLE in hardware: the panic at
 * _hw_endpoint_buffer_control_update32 ("ep %02X was already available") that froze Blaster #1 off USB
 * for 13 h in card mode. It also leaves the old buffer's PID stale while the host restarts at DATA0.
 * So: ABORT the endpoint (the RP2350 recipe reset_ep0 uses), then look at what the hardware holds:
 *   still AVAILABLE      -> nothing landed: drop the buffer, re-arm cleanly with next_pid (DATA0)
 *   a packet landed      -> its completion IRQ is pending: do NOT re-arm; the pending completion
 *                           finishes the old transfer (every re-armer passes the same buffer/length)
 *   neither              -> the ordinary stall -> clear path: upstream's reset + arm, unchanged
 * Not an IRQ-path function (only tud_task reaches xfer_start), so it adds no RAM/flash edge.
 * INVARIANT (a review): the only route to "active with AVAIL set" is a class re-arm after
 * usbd_edpt_clear_stall zeroed busy, and dcd_edpt_clear_stall has then already set next_pid = 0, so
 * the clean re-arm starts at DATA0 as the host does. Do not toggle next_pid here.
 * KNOWN EDGE: a vendor-bulk OUT armed as the SINK (g_sink_buf) when a clear-halt lands completes as a
 * CB arm; fl_bulk_on_cb then parses stale g_cb_buf -> one bad frame, which the host retries. */
volatile uint32_t fl_ep_rearm_aborts;          /* re-arms that found the buffer still armed */
volatile uint32_t fl_ep_rearm_landed;          /* re-arms that found a packet already landed */
/* ARMRETRY r5: ~~the spin's timeout was ignored and buffer control read
 * and rewritten anyway~~ - before ABORT_DONE the RP2350 does not promise that is safe. On a timeout NOTHING
 * is touched and the new arm is SKIPPED (returns true, as for a landed packet): the live transfer finishes
 * the old one. The bulk driver never arms over a transfer it could not cancel (out/in_uncancelled). */
volatile uint32_t fl_ep_rearm_abort_timeouts;  /* re-arms whose abort timed out (arm skipped) */

static bool fl_ep_abort_before_rearm(struct hw_endpoint* ep) {
  if (ep->pending) {                           /* E15 deferred IN: no buffer is owned yet */
    ep->pending = 0;
    return false;
  }
  uint8_t const num = tu_edpt_number(ep->ep_addr);
  bool const out = tu_edpt_dir(ep->ep_addr) == TUSB_DIR_OUT;
  uint32_t const bit = 1u << (2u * num + (out ? 1u : 0u));
  uint32_t const irq = save_and_disable_interrupts();
  if (rp2040_chip_version() >= 2) {
    usb_hw_clear->abort_done = bit;            /* ARMRETRY r4: as fl_ep_abort_xfer - no stale ABORT_DONE */
    usb_hw_set->abort = bit;
    for (uint32_t n = 0; n < 10000u && !(usb_hw->abort_done & bit); n++) {}
    if (!(usb_hw->abort_done & bit)) {         /* ARMRETRY r5: timed out - touch nothing, skip the arm */
      usb_hw_clear->abort = bit;
      fl_ep_rearm_abort_timeouts++;
      restore_interrupts(irq);
      return true;
    }
  }
  uint32_t const bc = *ep->buffer_control;
  bool const owned = (bc & (USB_BUF_CTRL_AVAIL | (USB_BUF_CTRL_AVAIL << 16))) != 0;
  bool const landed = !owned && (usb_hw->buf_status & bit) != 0;
  if (owned) {
    _hw_endpoint_buffer_control_set_value32(ep, 0);
    fl_ep_rearm_aborts++;
  } else if (landed) {
    fl_ep_rearm_landed++;
  }
  if (rp2040_chip_version() >= 2) {
    usb_hw_clear->abort_done = bit;
    usb_hw_clear->abort = bit;
  }
  restore_interrupts(irq);
  return landed;
}
/* --------------------------------------------------------------------------------------------------- */
/* FORGE LINK HUB1 (2026-10-13): ABORT A TRANSFER THE HOST ABANDONED.
 * The vendor-bulk watchdog (src/ForgeLink_bulk.cpp, forgeLinkBulkPoll) halts a data phase that never
 * finished. A stall alone leaves this endpoint `active` with `user_buf` aimed at a frame slot the board
 * then hands to CDC, and a packet the SIE had already taken would still be copied into that slot by its
 * completion IRQ (an outside review). So, with interrupts masked: abort the endpoint in hardware (the same
 * RP2350 recipe as fl_ep_abort_before_rearm), drop whatever buffer it owns, clear its buf_status bit so no
 * completion runs for it (a completion on an inactive endpoint is upstream's panic in
 * hw_endpoint_xfer_continue), and reset the transfer: nothing is left pointing at the slot. ~~Thread mode
 * only~~ (ARMRETRY r2): called from thread mode (the poll's watchdog) AND from tud_task (the
 * bulk driver's link fault and the stall branch of FL_REQ_BULK_RESET); safe in both because it masks
 * interrupts itself and tud_task (usbIRQ) and the USBCTRL IRQ share a priority. Never from the USBCTRL
 * IRQ handler itself. The endpoint pointer comes from fl_ep_seen, recorded at every xfer_start.
 * ~~Returns true when a landed packet was dropped.~~ ARMRETRY r3: returns true when the
 * transfer is CANCELLED (or none was ever started); a dropped landed packet is counted in
 * fl_ep_abort_landed. Returns false when EP_ABORT_DONE never came within the spin: then NOTHING is
 * touched (the abort request is withdrawn, the transfer left alive and active), because resetting a
 * transfer the SIE may still complete is upstream's "continue xfer on inactive ep" panic; the caller
 * must not re-arm over it (ForgeLink_bulk.cpp holds the link HALTED). Counted in fl_ep_abort_timeouts.
 * ARMRETRY r4 (the r3 review's F3): "nothing is touched" holds for THIS function; r3's callers then
 * stalled the endpoint anyway (dcd_edpt_stall writes buffer control). They no longer do: ForgeLink_bulk.cpp
 * stalls only an endpoint whose abort completed (fl_bulk_stall_cancelled). ABORT_DONE is cleared BEFORE
 * the abort is set, so a late ABORT_DONE from a timed-out abort cannot end the next spin early. */
static struct hw_endpoint* fl_ep_seen[32];
static inline uint8_t fl_ep_slot(uint8_t ep_addr) {
  return (uint8_t)((tu_edpt_number(ep_addr) << 1) | (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT ? 1u : 0u));
}
volatile uint32_t fl_ep_watchdog_aborts;       /* transfers the bulk watchdog aborted */
volatile uint32_t fl_ep_abort_timeouts;        /* ARMRETRY r3: aborts whose EP_ABORT_DONE never came */
volatile uint32_t fl_ep_abort_landed;          /* ARMRETRY r3: landed packets an abort dropped */
bool fl_ep_abort_xfer(uint8_t ep_addr) {
  struct hw_endpoint* ep = fl_ep_seen[fl_ep_slot(ep_addr) & 31u];
  if (!ep) return true;                        /* ARMRETRY r3: never started, nothing to cancel */
  uint8_t const num = tu_edpt_number(ep->ep_addr);
  bool const out = tu_edpt_dir(ep->ep_addr) == TUSB_DIR_OUT;
  uint32_t const bit = 1u << (2u * num + (out ? 1u : 0u));
  uint32_t const irq = save_and_disable_interrupts();
  if (rp2040_chip_version() >= 2) {
    usb_hw_clear->abort_done = bit;            /* ARMRETRY r4: a stale ABORT_DONE (a timed-out abort's
                                                * late one) must not end this spin before THIS abort */
    usb_hw_set->abort = bit;
    for (uint32_t n = 0; n < 10000u && !(usb_hw->abort_done & bit); n++) {}
    if (!(usb_hw->abort_done & bit)) {         /* ARMRETRY r3: timed out - touch nothing */
      usb_hw_clear->abort = bit;
      fl_ep_abort_timeouts++;
      restore_interrupts(irq);
      return false;
    }
  }
  bool const landed = (usb_hw->buf_status & bit) != 0;
  if (landed) fl_ep_abort_landed++;
  _hw_endpoint_buffer_control_set_value32(ep, 0);
  usb_hw_clear->buf_status = bit;
  ep->pending = 0;
  hw_endpoint_reset_transfer(ep);
  if (rp2040_chip_version() >= 2) {
    usb_hw_clear->abort_done = bit;
    usb_hw_clear->abort = bit;
  }
  fl_ep_watchdog_aborts++;
  restore_interrupts(irq);
  return true;                                 /* ARMRETRY r3: was `return landed;` */
}
/* ARMRETRY r6: clear ONE endpoint's latched BUFF_STATUS bit (WC, via the
 * clear alias as hw_handle_buff_status does). Called by the bulk driver, masked, at its bus reset and at
 * open (after hw_endpoint_init zeroed the buffer control, before any arm): the bit then belongs to a
 * transfer that no longer exists. Not the IRQ path. */
volatile uint32_t fl_ep_pending_cleared;       /* stale BUFF_STATUS bits cleared at open / reset */
volatile uint32_t fl_ep_stale_status_drops;    /* buffer completions on an inactive ep, dropped (was a panic) */
void fl_ep_clear_pending(uint8_t ep_addr) {
  uint32_t const bit = 1u << (fl_ep_slot(ep_addr) & 31u);
  if (usb_hw->buf_status & bit) {
    usb_hw_clear->buf_status = bit;
    fl_ep_pending_cleared++;
  }
}
/* --------------------------------------------------------------------------------------------------- */

void hw_endpoint_xfer_start(struct hw_endpoint* ep, uint8_t* buffer, uint16_t total_len) {
  hw_endpoint_lock_update(ep, 1);
  fl_ep_seen[fl_ep_slot(ep->ep_addr) & 31u] = ep;   /* FORGE LINK HUB1: for fl_ep_abort_xfer */

  if (ep->active) {
    // TODO: Is this acceptable for interrupt packets?
    TU_LOG(1, "WARN: starting new transfer on already active ep %02X\r\n", ep->ep_addr);
    if (fl_ep_abort_before_rearm(ep)) {        /* FORGE LINK: a packet landed; its completion finishes */
      hw_endpoint_lock_update(ep, -1);
      return;
    }
    hw_endpoint_reset_transfer(ep);
  }

  // Fill in info now that we're kicking off the hw
  ep->remaining_len = total_len;
  ep->xferred_len = 0;
  ep->active = true;
  ep->user_buf = buffer;

  if (e15_is_bulkin_ep(ep)) {
    usb_hw_set->inte = USB_INTS_DEV_SOF_BITS;
  }

  if (e15_is_critical_frame_period(ep)) {
    ep->pending = 1;
  } else {
    hw_endpoint_start_next_buffer(ep);
  }

  hw_endpoint_lock_update(ep, -1);
}

// sync endpoint buffer and return transferred bytes
static uint16_t __tusb_irq_path_func(sync_ep_buffer)(struct hw_endpoint* ep, uint8_t buf_id) {
  uint32_t buf_ctrl = _hw_endpoint_buffer_control_get_value32(ep);
  if (buf_id) buf_ctrl = buf_ctrl >> 16;

  uint16_t xferred_bytes = buf_ctrl & USB_BUF_CTRL_LEN_MASK;

  if (!ep->rx) {
    // We are continuing a transfer here. If we are TX, we have successfully
    // sent some data can increase the length we have sent
    assert(!(buf_ctrl & USB_BUF_CTRL_FULL));

    ep->xferred_len = (uint16_t) (ep->xferred_len + xferred_bytes);
  } else {
    // If we have received some data, so can increase the length
    // we have received AFTER we have copied it to the user buffer at the appropriate offset
    assert(buf_ctrl & USB_BUF_CTRL_FULL);

    unaligned_memcpy(ep->user_buf, ep->hw_data_buf + buf_id * 64, xferred_bytes);
    ep->xferred_len = (uint16_t) (ep->xferred_len + xferred_bytes);
    ep->user_buf += xferred_bytes;
  }

  // Short packet
  if (xferred_bytes < ep->wMaxPacketSize) {
    pico_trace("  Short packet on buffer %d with %u bytes\r\n", buf_id, xferred_bytes);
    // Reduce total length as this is last packet
    ep->remaining_len = 0;
  }

  return xferred_bytes;
}

static void __tusb_irq_path_func(_hw_endpoint_xfer_sync)(struct hw_endpoint* ep) {
  // Update hw endpoint struct with info from hardware
  // after a buff status interrupt

  uint32_t __unused buf_ctrl = _hw_endpoint_buffer_control_get_value32(ep);
  TU_LOG(3, "  Sync BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n", tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));

  // always sync buffer 0
  uint16_t buf0_bytes = sync_ep_buffer(ep, 0);

  // sync buffer 1 if double buffered
  if ((*ep->endpoint_control) & EP_CTRL_DOUBLE_BUFFERED_BITS) {
    if (buf0_bytes == ep->wMaxPacketSize) {
      // sync buffer 1 if not short packet
      sync_ep_buffer(ep, 1);
    } else {
      // short packet on buffer 0
      // TODO couldn't figure out how to handle this case which happen with net_lwip_webserver example
      // At this time (currently trigger per 2 buffer), the buffer1 is probably filled with data from
      // the next transfer (not current one). For now we disable double buffered for device OUT
      // NOTE this could happen to Host IN
#if 0
      uint8_t const ep_num = tu_edpt_number(ep->ep_addr);
      uint8_t const dir =  (uint8_t) tu_edpt_dir(ep->ep_addr);
      uint8_t const ep_id = 2*ep_num + (dir ? 0 : 1);

      // abort queued transfer on buffer 1
      usb_hw->abort |= TU_BIT(ep_id);

      while ( !(usb_hw->abort_done & TU_BIT(ep_id)) ) {}

      uint32_t ep_ctrl = *ep->endpoint_control;
      ep_ctrl &= ~(EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
      ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;

      _hw_endpoint_buffer_control_set_value32(ep, 0);

      usb_hw->abort &= ~TU_BIT(ep_id);

      TU_LOG(3, "----SHORT PACKET buffer0 on EP %02X:\r\n", ep->ep_addr);
      TU_LOG(3, "  BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n", tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));
#endif
    }
  }
}

// Returns true if transfer is complete
bool __tusb_irq_path_func(hw_endpoint_xfer_continue)(struct hw_endpoint* ep) {
  hw_endpoint_lock_update(ep, 1);

  // Part way through a transfer
  if (!ep->active) {
    // ~~panic("Can't continue xfer on inactive ep %02X", ep->ep_addr);~~ FORGE LINK ARMRETRY r6 (study
    // the r5 review's H1): a BUFF_STATUS bit that outlived its transfer (an endpoint closed or reset
    // with a packet latched) froze the board here. hw_handle_buff_status has already cleared the bit;
    // the controller has handed the buffer back, so its buffer control is ours to zero. DROPPED, counted,
    // no completion (false = "more to do", so the caller posts nothing).
    fl_ep_stale_status_drops++;
    if (ep->buffer_control) _hw_endpoint_buffer_control_set_value32(ep, 0);   /* NULL after close_all's memclr */
    hw_endpoint_lock_update(ep, -1);
    return false;
  }

  // Update EP struct from hardware state
  _hw_endpoint_xfer_sync(ep);

  // Now we have synced our state with the hardware. Is there more data to transfer?
  // If we are done then notify tinyusb
  if (ep->remaining_len == 0) {
    pico_trace("Completed transfer of %d bytes on ep %02X\r\n", ep->xferred_len, ep->ep_addr);
    // Notify caller we are done so it can notify the tinyusb stack
    hw_endpoint_lock_update(ep, -1);
    return true;
  } else {
    if (e15_is_critical_frame_period(ep)) {
      ep->pending = 1;
    } else {
      hw_endpoint_start_next_buffer(ep);
    }
  }

  hw_endpoint_lock_update(ep, -1);
  // More work to do
  return false;
}

//--------------------------------------------------------------------+
// Errata 15
//--------------------------------------------------------------------+

#if TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX

/* Don't mark IN buffers as available during the last 200us of a full-speed
   frame. This avoids a situation seen with the USB2.0 hub on a Raspberry
   Pi 4 where a late IN token before the next full-speed SOF can cause port
   babble and a corrupt ACK packet. The nature of the data corruption has a
   chance to cause device lockup.

   Use the next SOF to mark delayed buffers as available. This reduces
   available Bulk IN bandwidth by approximately 20%, and requires that the
   SOF interrupt is enabled while these transfers are ongoing.

   Inherit the top-level enable from the corresponding Pico-SDK flag.
   Applications that will not use the device in a situation where it could
   be plugged into a Pi 4 or Pi 400 (for example, when directly connected
   to a commodity hub or other host) can turn off the flag in the SDK.
*/

volatile uint32_t e15_last_sof = 0;

// check if Errata 15 is needed for this endpoint i.e device bulk-in
static bool __tusb_irq_path_func(e15_is_bulkin_ep)(struct hw_endpoint* ep) {
  return (!is_host_mode() && tu_edpt_dir(ep->ep_addr) == TUSB_DIR_IN &&
          ep->transfer_type == TUSB_XFER_BULK);
}

// check if we need to apply Errata 15 workaround : i.e
// Endpoint is BULK IN and is currently in critical frame period i.e 20% of last usb frame
static bool __tusb_irq_path_func(e15_is_critical_frame_period)(struct hw_endpoint* ep) {
  TU_VERIFY(e15_is_bulkin_ep(ep));

  /* Avoid the last 200us (uframe 6.5-7) of a frame, up to the EOF2 point.
   * The device state machine cannot recover from receiving an incorrect PID
   * when it is expecting an ACK.
   */
  uint32_t delta = time_us_32() - e15_last_sof;
  if (delta < 800 || delta > 998) {
    return false;
  }
  TU_LOG(3, "Avoiding sof %lu now %lu last %lu\r\n", (usb_hw->sof_rd + 1) & USB_SOF_RD_BITS, time_us_32(),
         e15_last_sof);
  return true;
}

#endif // TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
#endif
