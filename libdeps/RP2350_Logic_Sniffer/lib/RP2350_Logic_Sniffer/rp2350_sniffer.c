/** 
 * RP2350_Logic_Sniffer - Copyright (c) 2025-2026 Rabbit Hole Computing™
 * 
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <hardware/pio.h>
#include <hardware/gpio.h>
#include <hardware/dma.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/clocks.h>
#include "rp2350_sniffer.h"
#include "rp2350_sniffer.pio.h"

static_assert(SNIFFER_BLOCKSIZE % 4 == 0, "Buffer size must be divisible by 16");
static_assert((SNIFFER_BLOCKCOUNT & (SNIFFER_BLOCKCOUNT - 1)) == 0, "Block count must be power of 2");

// DMA transfers captured transitions to this buffer
#define SNIFFER_BLOCKSIZE_WORDS (SNIFFER_BLOCKSIZE / 4)
static sniffer_block_t g_sniffer_buf[SNIFFER_BLOCKCOUNT];

static struct {
    bool channels_claimed;
    uint32_t offset_sniffer;
    uint32_t offset_trigger;
    
    uint32_t total_blocks_fetched;
    uint32_t total_blocks_released;
    uint32_t total_bytes;
    uint32_t overruns;
} g_sniffer;

// These buffer pointers are used to retrigger DMA from
// the start when it reaches the end.
// Half of the entries are nullptr, which stops DMA from overwriting
// pending data.
#define DMA_BLOCKPTR_COUNT (SNIFFER_BLOCKCOUNT * 2)
__attribute__((aligned(sizeof(sniffer_block_t*) * DMA_BLOCKPTR_COUNT)))
sniffer_block_t *g_sniffer_dma_dest_blocks[DMA_BLOCKPTR_COUNT];

// Setup trigger state machines based on pin mask.
// Up to three contiguous pin ranges are supported.
void rp2350_sniffer_setup_triggers(uint32_t trigpins)
{
    trigpins &= SNIFFER_PINS_ALL;
    SNIFFER_LOG("Sniffer trigger configuration: 0x%08x", (unsigned)trigpins);

    // Find continuous pin ranges and configure state machines
    for (int sm = SNIFFER_PIO_SM_TRIGGER_MIN; sm <= SNIFFER_PIO_SM_TRIGGER_MAX; sm++)
    {
        if (!trigpins) break; // No more pins

        int lowpin = -1;
        for (int i = 0; i < SNIFFER_PINCOUNT; i++)
        {
            if (trigpins & (1 << i))
            {
                lowpin = i;
                break;
            }
        }

        int highpin = lowpin;
        for (int i = lowpin; i < SNIFFER_PINCOUNT; i++)
        {
            if (!(trigpins & (1 << i)))
            {
                break;
            }
            else
            {
                // Clear the pins are part of this range
                highpin = i;
                trigpins &= ~(1 << i);
            }
        }

        SNIFFER_LOG("Sniffer SM %d triggers on bit range %d-%d", sm, lowpin, highpin);

        pio_sm_config cfg = rp2350_sniffer_trigger_program_get_default_config(g_sniffer.offset_trigger);
        sm_config_set_in_pins(&cfg, SNIFFER_FIRSTPIN + lowpin);
        sm_config_set_in_pin_count(&cfg, highpin - lowpin + 1);
        int rc = pio_sm_init(SNIFFER_PIO, sm, g_sniffer.offset_trigger, &cfg);
        assert(rc == PICO_OK);
        pio_sm_set_enabled(SNIFFER_PIO, sm, true);
    }

    // Check if we were able to setup all pins
    if (trigpins)
    {
        SNIFFER_LOG("Sniffer trigger has too many pin ranges, remaining pins ignored: 0x%08x", (unsigned)trigpins);
    }
}

// Try to reserve up to "length" bytes of space in the current block.
// Returns pointer where data can be written.
// If less than "length" is available, modifies the value.
__attribute__((optimize("O3")))
static uint32_t *__time_critical_func(reserve_aux_space)(size_t *length)
{
    // Round up to whole words
    uint32_t words = (*length + sizeof(uint32_t) - 1) / sizeof(uint32_t);

    // Stop the DMA for a moment, get the pointer and resume
    // This has to be done fast enough that the PIO RX FIFO doesn't overflow
    uint32_t saved_interrupts = save_and_disable_interrupts();

    // Note: RP2350-E5 workaround: clear enable bits before aborting channel
    dma_hw->ch[SNIFFER_DMA_CH_A].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
    dma_hw->ch[SNIFFER_DMA_CH_B].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
    __sync_synchronize();
    dma_channel_abort(SNIFFER_DMA_CH_A);

    // Get the current write position and the start index of the block
    uint32_t dma_nextblock = (dma_hw->ch[SNIFFER_DMA_CH_B].al1_read_addr - (uint32_t)g_sniffer_dma_dest_blocks) / sizeof(uint32_t*);
    uint32_t *pos_start = g_sniffer_dma_dest_blocks[(dma_nextblock - 1) % (SNIFFER_BLOCKCOUNT * 2)]->data;
    uint32_t *pos = (uint32_t*)dma_hw->ch[SNIFFER_DMA_CH_A].al1_write_addr;
    uint32_t remain = SNIFFER_BLOCKSIZE_WORDS - (pos - pos_start);
    if (words > remain) words = remain;
    remain -= words;

    dma_hw->ch[SNIFFER_DMA_CH_A].al1_ctrl |= DMA_CH0_CTRL_TRIG_EN_BITS;
    dma_hw->ch[SNIFFER_DMA_CH_B].al1_ctrl |= DMA_CH0_CTRL_TRIG_EN_BITS;

    if (remain > 0)
    {
        // Resume writing to same block
        dma_hw->ch[SNIFFER_DMA_CH_A].al2_transfer_count = remain;
        dma_hw->ch[SNIFFER_DMA_CH_A].al2_write_addr_trig = (uint32_t)(pos + words);
        dma_hw->ch[SNIFFER_DMA_CH_A].al2_transfer_count = SNIFFER_BLOCKSIZE_WORDS;
    }
    else
    {
        // Get next block
        dma_channel_start(SNIFFER_DMA_CH_B);
    }
    restore_interrupts(saved_interrupts);

    *length = words * sizeof(uint32_t);
    return pos;
}

size_t rp2350_sniffer_write_aux_data(sniffer_aux_data_channel_t data_id, const void *data, size_t length)
{
    size_t bytes_done = 0;
    while (bytes_done < length)
    {
        size_t remain = length - bytes_done;
        size_t blocklen = remain + sizeof(uint32_t);
        uint32_t *dest = reserve_aux_space(&blocklen);

        if (!dest) break;

        size_t payload_len = blocklen - sizeof(uint32_t);
        if (payload_len > remain) payload_len = remain;
        dest[0] = 0xFF800000 | (data_id << 16) | payload_len;

        memcpy(&dest[1], data + bytes_done, payload_len);
        bytes_done += payload_len;
    }
    return bytes_done;
}

static uint32_t format_timestamp()
{
    return 0xFC000000 | ((time_us_32() / 1000) & 0xFFFFFF);
}

static void write_header()
{
    size_t hdrwords = 8;
    size_t blocklen = sizeof(uint32_t) * hdrwords;
    uint32_t *dest = reserve_aux_space(&blocklen);
    assert(blocklen == sizeof(uint32_t) * hdrwords);
    memcpy(dest, "RP2350_Logic_Sniffer", 20);
    dest[5] = 0xFF000003;                                                   // Sniffer format version
    dest[6] = 0xFF010000 | ((clock_get_hz(clk_sys) + 500000) / 1000000);    // CPU clock speed
    dest[7] = format_timestamp();              // System timestamp
}

bool rp2350_sniffer_start(uint32_t pin_mask)
{
    // Claim channels and load programs on first call
    if (!g_sniffer.channels_claimed)
    {
        pio_sm_claim(SNIFFER_PIO, SNIFFER_PIO_SM);

        for (int i = SNIFFER_PIO_SM_TRIGGER_MIN; i <= SNIFFER_PIO_SM_TRIGGER_MAX; i++)
        {
            pio_sm_claim(SNIFFER_PIO, i);
        }

        g_sniffer.offset_sniffer = pio_add_program(SNIFFER_PIO, &rp2350_sniffer_program);
        g_sniffer.offset_trigger = pio_add_program(SNIFFER_PIO, &rp2350_sniffer_trigger_program);

        dma_channel_claim(SNIFFER_DMA_CH_A);
        dma_channel_claim(SNIFFER_DMA_CH_B);
        g_sniffer.channels_claimed = true;
    }

    // Disable state machines and reset state
    rp2350_sniffer_stop();
    g_sniffer.total_bytes = 0;
    g_sniffer.overruns = 0;
    g_sniffer.total_blocks_fetched = 0;
    g_sniffer.total_blocks_released = 0;
    
    // Setup the main state machine
    {
        pio_sm_config cfg = rp2350_sniffer_program_get_default_config(g_sniffer.offset_sniffer);
        sm_config_set_in_pins(&cfg, SNIFFER_FIRSTPIN);
        sm_config_set_in_pin_count(&cfg, SNIFFER_PINCOUNT);
        sm_config_set_mov_status(&cfg, STATUS_IRQ_SET, SNIFFER_TRIGGER_IRQ);
        int rc = pio_sm_init(SNIFFER_PIO, SNIFFER_PIO_SM, g_sniffer.offset_sniffer + rp2350_sniffer_offset_init, &cfg);
        assert(rc == PICO_OK);
    }

    // Setup trigger state machines
    rp2350_sniffer_setup_triggers(pin_mask);

    // First DMA channel for data transfer from PIO to RAM buffers.
    // The first DMA channel is started by the second DMA channel.
    {
        dma_channel_config cfg = dma_channel_get_default_config(SNIFFER_DMA_CH_A);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg, false);
        channel_config_set_write_increment(&cfg, true);
        channel_config_set_chain_to(&cfg, SNIFFER_DMA_CH_B);
        channel_config_set_dreq(&cfg, pio_get_dreq(SNIFFER_PIO, SNIFFER_PIO_SM, false));
        dma_channel_configure(SNIFFER_DMA_CH_A, &cfg,
            NULL,
            &SNIFFER_PIO->rxf[SNIFFER_PIO_SM],
            SNIFFER_BLOCKSIZE_WORDS,
            false);
    }

    // Second DMA channel to retrigger first one from the start of the buffer
    {
        // Fill in pointers for the first set of blocks, and null for blocks after that.
        for (int i = 0; i < SNIFFER_BLOCKCOUNT; i++)
        {
            g_sniffer_dma_dest_blocks[i] = &g_sniffer_buf[i];
            g_sniffer_dma_dest_blocks[i + SNIFFER_BLOCKCOUNT] = NULL;
        }

        dma_channel_config cfg = dma_channel_get_default_config(SNIFFER_DMA_CH_B);
        channel_config_set_read_increment(&cfg, true);
        channel_config_set_write_increment(&cfg, false);
        channel_config_set_ring(&cfg, false, __builtin_ctz(sizeof(g_sniffer_dma_dest_blocks)));
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
        dma_channel_configure(SNIFFER_DMA_CH_B, &cfg,
            &dma_hw->ch[SNIFFER_DMA_CH_A].al2_write_addr_trig,
            &g_sniffer_dma_dest_blocks,
            1,
            true);
    }

    // Write version and system info
    write_header();

    // Write pinout information
    rp2350_sniffer_write_aux_data(SNIFFER_DATAID_IO_PINS, SNIFFER_PINNAMES, strlen(SNIFFER_PINNAMES));

    // Start encoding with the initial states of the pins
    SNIFFER_PIO->irq_force = (1 << SNIFFER_TRIGGER_IRQ);
    pio_sm_set_enabled(SNIFFER_PIO, SNIFFER_PIO_SM, true);

    return true;
}

// Fetch a filled DMA block
sniffer_block_t *rp2350_sniffer_fetch(bool force_sync)
{
    // Do we have any full blocks?
    uint32_t dma_wrpos = (dma_hw->ch[SNIFFER_DMA_CH_B].al1_read_addr - (uint32_t)g_sniffer_dma_dest_blocks) / sizeof(uint32_t*);
    uint32_t cpu_rdpos = (g_sniffer.total_blocks_fetched % DMA_BLOCKPTR_COUNT);
    uint32_t available = (dma_wrpos - cpu_rdpos - 1) % DMA_BLOCKPTR_COUNT;

    if (force_sync && available == 0)
    {
        // Block is not full, so force it to fill up.
        size_t blocklen = SNIFFER_BLOCKSIZE;
        uint32_t *dest = reserve_aux_space(&blocklen);

        if (blocklen >= sizeof(uint32_t))
        {
            // Update the system timestamp
            dest[0] = format_timestamp();

            if (blocklen >= 2 * sizeof(uint32_t))
            {
                // Fill with zero bytes to data_id 0 (they are ignored by decoder)
                size_t payload_len = blocklen - 2 * sizeof(uint32_t);
                dest[1] = 0xFF800000 | payload_len;
                memset(&dest[2], 0, payload_len);
            }
        }

        available = 1;
    }

    if (available > 0)
    {
        // Remove the block from DMA queue and return
        sniffer_block_t *block = g_sniffer_dma_dest_blocks[cpu_rdpos];
        g_sniffer_dma_dest_blocks[cpu_rdpos] = NULL;
        g_sniffer.total_blocks_fetched++;
        g_sniffer.total_bytes += SNIFFER_BLOCKSIZE;
        return block;
    }

    // No filled blocks available
    return NULL;
}

// Release a block received from rp2350_sniffer_fetch().
void rp2350_sniffer_release(sniffer_block_t *block)
{
    if (!block) return;

    // Add the block to DMA queue
    uint32_t idx = (g_sniffer.total_blocks_released + SNIFFER_BLOCKCOUNT) % DMA_BLOCKPTR_COUNT;
    g_sniffer_dma_dest_blocks[idx] = block;
    g_sniffer.total_blocks_released++;

    // Check if the DMA has ran out of free blocks and paused (causes data loss)
    if (dma_hw->ch[SNIFFER_DMA_CH_A].al2_write_addr_trig == 0)
    {
        g_sniffer.overruns++;

        // There was dropped data.
        // Encode a "glitch" that will visually indicate lost data
        const uint32_t glitch[6] = {
            0xF0000000, // All signals low, 1 cycle
            0xFBFF8ACF, // 1 ms pause
            0xF7FFFFFF, // All signals high, 1 cycle
            0xF0000000, // All signals low, 1 cycle
            0xFBFF8ACF, // 1 ms pause
            format_timestamp(), // Timestamp
        };

        memcpy(block, glitch, sizeof(glitch));

        // Resume writing to the block but with less words
        dma_hw->ch[SNIFFER_DMA_CH_A].al2_transfer_count = (SNIFFER_BLOCKSIZE - sizeof(glitch)) / sizeof(uint32_t);
        dma_hw->ch[SNIFFER_DMA_CH_A].al2_write_addr_trig = (uint32_t)block + sizeof(glitch);

        // Force encoding of latest pin states
        SNIFFER_PIO->irq_force = (1 << SNIFFER_TRIGGER_IRQ);

        // Restore block size for next transfer
        dma_hw->ch[SNIFFER_DMA_CH_A].al2_transfer_count = SNIFFER_BLOCKSIZE_WORDS;
    }
}

void rp2350_sniffer_get_status(rp2350_sniffer_stats_t *stats)
{
    stats->partial_block_len = (SNIFFER_BLOCKSIZE_WORDS - dma_hw->ch[SNIFFER_DMA_CH_A].al2_transfer_count) * sizeof(uint32_t);
    stats->total_bytes = g_sniffer.total_bytes;
    stats->overruns = g_sniffer.overruns;

    uint32_t dma_wrpos = (dma_hw->ch[SNIFFER_DMA_CH_B].al1_read_addr - (uint32_t)g_sniffer_dma_dest_blocks) / sizeof(uint32_t*);
    stats->free_blocks = (SNIFFER_BLOCKCOUNT + g_sniffer.total_blocks_released - dma_wrpos) % DMA_BLOCKPTR_COUNT;
}

void rp2350_sniffer_stop()
{
    pio_sm_set_enabled(SNIFFER_PIO, SNIFFER_PIO_SM, false);
    for (int i = SNIFFER_PIO_SM_TRIGGER_MIN; i <= SNIFFER_PIO_SM_TRIGGER_MAX; i++)
    {
        pio_sm_set_enabled(SNIFFER_PIO, i, false);
    }
    
    // Note: RP2350-E5 workaround: clear enable bits before aborting channels
    dma_hw->ch[SNIFFER_DMA_CH_A].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
    dma_hw->ch[SNIFFER_DMA_CH_B].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
    __sync_synchronize();
    dma_channel_abort(SNIFFER_DMA_CH_A);
    dma_channel_abort(SNIFFER_DMA_CH_B);
}
