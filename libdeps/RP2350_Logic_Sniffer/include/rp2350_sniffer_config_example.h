/* Copy this file to rp2350_sniffer_config.h in your own project */

#pragma once

#include <stdio.h>

// Optional informational message logging
#ifdef SNIFFER_DISABLE_LOG
#   ifdef SNIFFER_LOG
#       undef SNIFFER_LOG
#   endif
#   define SNIFFER_LOG(x, ...) (void)x
#else
#   ifndef SNIFFER_LOG
#       define SNIFFER_LOG(x, ...) printf(x, __VA_ARGS__)
#   endif
#endif

// Size of capture data blocks, must be a multiple of 4 bytes.
#ifndef SNIFFER_BLOCKSIZE
#define SNIFFER_BLOCKSIZE 512
#endif

// Number of capture data blocks to allocate
// Must be a power of two
#ifndef SNIFFER_BLOCKCOUNT
#define SNIFFER_BLOCKCOUNT 16
#endif

// DMA channel for transfer of data from PIO to RAM
#ifndef SNIFFER_DMA_CH_A
#define SNIFFER_DMA_CH_A 5
#endif

// DMA channel for reconfiguring first DMA channel
#ifndef SNIFFER_DMA_CH_B
#define SNIFFER_DMA_CH_B 6
#endif

// PIO block used for capture
#ifndef SNIFFER_PIO
#define SNIFFER_PIO pio2
#endif

// PIO state machine used for main sniffer code
#ifndef SNIFFER_PIO_SM
#define SNIFFER_PIO_SM 3
#endif

// PIO state machines for triggering
// If you have other code on the PIO, you can reduce the number of these
#ifndef SNIFFER_PIO_SM_TRIGGER_MIN
#define SNIFFER_PIO_SM_TRIGGER_MIN 0
#endif
#ifndef SNIFFER_PIO_SM_TRIGGER_MAX
#define SNIFFER_PIO_SM_TRIGGER_MAX 2
#endif

// Number of IO pins to read
#ifndef SNIFFER_PINCOUNT
#define SNIFFER_PINCOUNT            27
#endif

// Index of the lowest IO pin
#ifndef SNIFFER_FIRSTPIN
#define SNIFFER_FIRSTPIN            0
#endif

// Pin names to use in the decoded .vcd file
#ifndef SNIFFER_PINNAMES
#define SNIFFER_PINNAMES "D0 D1 D2 D3 D4 D5 D6 D7 D8 D9 D10 D11 D12 D13 D14 D15 D16 D17 D18 D19 D20 D21 D22 D23 D24 D25 D26"
#endif
