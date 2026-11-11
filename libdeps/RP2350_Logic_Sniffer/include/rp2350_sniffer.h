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


#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if defined __has_include
# if __has_include (<rp2350_sniffer_config.h>)
#  include <rp2350_sniffer_config.h>
#else
#  include <rp2350_sniffer_config_example.h>
# endif
#else
# include <rp2350_sniffer_config.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SNIFFER_DATAID_IO_PINS = 0x0,
    SNIFFER_DATAID_SYSLOG  = 0x1,
    SNIFFER_DATAID_USER8   = 0x8,
    SNIFFER_DATAID_USER9   = 0x9,
    SNIFFER_DATAID_USERA   = 0xA,
    SNIFFER_DATAID_USERB   = 0xB,
    SNIFFER_DATAID_USERC   = 0xC,
    SNIFFER_DATAID_USERD   = 0xD,
    SNIFFER_DATAID_USERE   = 0xE,
    SNIFFER_DATAID_USERF   = 0xF,
} sniffer_aux_data_channel_t;

typedef struct {
    uint32_t data[SNIFFER_BLOCKSIZE / sizeof(uint32_t)];
} sniffer_block_t;

#define SNIFFER_PINS_ALL ((~(uint32_t)0) >> (32 - SNIFFER_PINCOUNT))

// Initialize sniffer operation and start capturing data.
// pin_mask can optionally limit which IO pins are monitored,
// or set to SNIFFER_PINS_ALL to monitor all pins.
bool rp2350_sniffer_start(uint32_t pin_mask);

// Write custom auxiliary data to the next output data block.
// The data_id identifies data purpose:
// 0: IO pin names, separated by space
// 1: System log data
// 2-7: Reserved for future
// 8-15: Custom user data
//
// Returns number of bytes successfully written.
size_t rp2350_sniffer_write_aux_data(sniffer_aux_data_channel_t data_id,
    const void *data, size_t length);

// Call this function periodically when capture is running to fetch
// the captured data. Returns a pointer to a SNIFFER_BLOCKSIZE sized
// data block.
//
// If force_sync is true, forces the flush of any partially
// filled block. Otherwise returns NULL if there are no fully filled
// blocks.
//
// You can fetch multiple block pointers and write them out asynchronously.
// After you have written out the block, call rp2350_sniffer_release().
sniffer_block_t *rp2350_sniffer_fetch(bool force_sync);

// Release a block received from rp2350_sniffer_fetch().
void rp2350_sniffer_release(sniffer_block_t *block);

typedef struct {
    uint32_t partial_block_len; // Amount of bytes waiting in an unsynced partial block
    uint32_t total_bytes; // Total bytes written to output
    uint32_t overruns; // Number of buffer overruns
    uint32_t free_blocks; // Number of blocks that are free to be used
} rp2350_sniffer_stats_t;

// Query sniffer status
void rp2350_sniffer_get_status(rp2350_sniffer_stats_t *stats);

// Stop sniffer operation
void rp2350_sniffer_stop();

#ifdef __cplusplus
}
#endif
