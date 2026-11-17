/**
 * Forge Link - a fork of ZuluSCSI-firmware by Rabbit Hole Computing.
 * Modifications Copyright (c) 2026 the Forge Link contributors.
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

// THE TWO DIGESTS THE BOARD COMPUTES, IN PLAIN C WITH NO ARDUINO IN IT - ON PURPOSE.
//
// This file compiles on the Mac as well as on the RP2350, and tools/tests/test_fl_pipeline.py
// does exactly that: it builds THIS source with the host compiler and checks every output
// against Python's zlib and hashlib, byte for byte, over lengths that straddle every block
// boundary. So "the board's SHA-256 is SHA-256" is a test that runs, not a belief - the only
// part of the verify path that is proven without the board.
//
//   flDigestCrc32 - IEEE CRC-32 (zlib), table-driven. The frame transport's CRC and the WRITE
//                   reply's read-back CRC. Same numbers as the old bit-at-a-time loop, ~8x
//                   fewer cycles: on a 32 KiB frame that is the difference between a ~9 ms
//                   lump in the main loop and ~1.5 ms (INFERRED from cycle counts, not timed).
//   flSha256*     - SHA-256 (FIPS 180-4), the HASH_SECTORS verb's fingerprint. Software, so
//                   the host test covers the exact code the board runs. The RP2350 has a SHA-256
//                   accelerator; it is deliberately NOT used yet, because its padding and byte
//                   order could only be proven on the board, and a verify that is wrong on the
//                   board is a push that stops for no reason at the bench.

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// zlib-compatible: flDigestCrc32(0, p, n) == zlib.crc32(p), and it chains like zlib's.
uint32_t flDigestCrc32(uint32_t crc, const uint8_t *data, size_t len);

#define FL_SHA256_BYTES 32

typedef struct {
    uint32_t h[8];
    uint64_t total;       // bytes fed so far
    uint32_t buflen;      // bytes waiting in buf
    uint8_t  buf[64];
} fl_sha256_t;

void flSha256Init(fl_sha256_t *s);
void flSha256Update(fl_sha256_t *s, const uint8_t *data, size_t len);
void flSha256Final(fl_sha256_t *s, uint8_t out[FL_SHA256_BYTES]);

#ifdef __cplusplus
}
#endif
