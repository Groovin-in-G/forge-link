#!/bin/env python3
#
# RP2350_Logic_Sniffer - Copyright (c) 2025-2026 Rabbit Hole Computing™
# 
# MIT License
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

'''This program converts the custom format produced by RP2350_Logic_Sniffer
to standard VCD (Value Change Dump) format.
The resulting file can be opened using e.g. PulseView.

See rp2350_sniffer.pio for definition of the encoding format.
'''

import sys
import os.path
import struct
import time

class SniffDecoder:
    magic = b'RP2350_Logic_Sniffer'
    supported_versions = [3]

    def __init__(self):
        self.reset()

    def reset(self):
        '''Initialize state at start of a file'''
        self.timestamp = 0
        self.systime = 0
        self.systime_ref = 0
        self.last_vcd_timestamp = 0
        self.pin_states = 0
        self.version = 1
        self.pinmap = []
        self.excess = b''
        self.aux_data = [b''] * 16
        self.cpu_freq = 150e6
        self.cpu_freq_MHz = 150
        self.divider = 3
        self.timescale = "20ns"
        self.got_magic = False
        self.got_header = False
        self.auxdataid = 0
        self.auxdatalen = 0

    def get_systime(self):
        '''Interpolate system time from sample timestamps.'''
        return self.systime + int((self.timestamp - self.systime_ref) * 1000 // self.cpu_freq)

    def decode(self, data: bytes):
        '''Decode binary data and return a list of tuples:
        [(timedelta, pin_states, systime), ...]
        '''

        # If we had any left-over bytes from previous iteration, add them
        if self.excess:
            data = self.excess + data
            self.excess = b''

        # Discard garbage before start of magic bytes
        if not self.got_magic:
            pos = data.find(self.magic)
            if pos >= 0:
                # Ok, magic bytes found
                self.got_magic = True
                data = data[pos + len(self.magic):]
                pass
            elif not self.magic in data:
                # Data does not start yet
                self.excess = data[-len(self.magic):]
                return []

        # Always process data in blocks of 4 bytes, and store any
        # excess bytes for next iteration.
        leftover = len(data) % 4
        if leftover != 0:
            self.excess = data[-leftover:]
            data = data[:-leftover]

        result = []

        for word, in struct.iter_unpack("<I", data):
            d = word >> 27
            p = word & 0x07FFFFFF
            max_timedelta = 0x03FFFFFF

            if self.auxdatalen > 0:
                # Process auxiliary data words from a previously started chunk
                # Chunk is always padded to full word even if payload is shorter
                raw = struct.pack("<I", word)[:self.auxdatalen] # Back to raw bytes and limit length
                if self.auxdataid == 0: raw = raw.replace(b'\x00', b'') # Data ID 0 with null bytes is used for padding
                self.aux_data[self.auxdataid] += raw
                self.auxdatalen -= 4

            elif (word >> 16) == 0xFF00:
                # Sniffer version header
                self.version = word & 0xFFFF
                self.got_header = True

                if self.version not in self.supported_versions:
                    print("Warning: unsupported sniffer format version %d" % self.version)

            elif (word >> 16) == 0xFF01:
                # CPU frequency header
                self.cpu_freq_MHz = word & 0xFFFF
                self.cpu_freq = self.cpu_freq_MHz * 1e6

                print("Sniffer version %d, CPU frequency %d MHz" % (self.version, self.cpu_freq_MHz))

                self.divider = 3
                self.timescale = '%dps' % (1e12 * self.divider / self.cpu_freq)

            elif (word >> 24) == 0xFC:
                # System millisecond timestamp
                self.systime = word & 0xFFFFFF
                self.systime_ref = self.timestamp

            elif (word >> 20) == 0xFF8:
                # Auxiliary data block
                self.auxdataid = (word >> 16) & 0x0F
                self.auxdatalen = (word & 0xFFFF)

            else:
                # Pin state information
                if d != 31:
                    self.timestamp += 3 * (32 - d)
                    self.pin_states = p
                elif p <= max_timedelta:
                    self.timestamp += 3 * (max_timedelta - p + 3)
                elif word == 0xFFFFFFFF:
                    self.timestamp += 3 * (max_timedelta + 2)
                else:
                    print("Unknown word 0x%08x" % word)

                result.append((self.timestamp, self.pin_states, self.get_systime()))

        return result

    def format_vcd(self, transitions):
        '''Format tuple of changes into VCD format'''
        result = []
        for timestamp, pin_states, systime in transitions:
            line = '#%d r%0.3f t' % (timestamp // self.divider, systime / 1000.0)
            for bit, bitcount, name, symbol in self.pinmap:
                if bitcount == 1:
                    line += " %d%s" % ((pin_states >> bit) & 1, symbol)
                else:
                    bitstring = bin(pin_states >> bit)[2:]
                    bitstring = bitstring.zfill(bitcount)[-bitcount:]
                    line += " b%s %s" % (bitstring, symbol)
            result.append(line)
            self.last_vcd_timestamp = (timestamp // self.divider)
        result.append('')
        return '\n'.join(result)

    def format_vcd_header(self):
        '''Format header for a VCD file with the signal map.'''
        result = []
        result.append('$version RP2350 Logic Sniffer %d $end' % self.version)
        result.append('$timescale %s $end' % self.timescale)
        result.append('$scope module sniff $end')

        # Get IO pin names from aux data
        pinnames = self.aux_data[0].decode('ascii', errors = 'replace').split()
        if not pinnames:
            print("No IO pin names found in data, using defaults")
            pinnames = ['D%d' for i in range(27)]

        # Generate pin map based on the pin names indicated by file
        self.pinmap = []
        for i, pinname in enumerate(pinnames):
            self.pinmap.append((i, 1, pinname, chr(ord('A') + i)))
        
        # And write out the pin map to VCD file
        for bit, bitcount, name, symbol in self.pinmap:
            result.append('$var wire %d %s %s $end' % (bitcount, symbol, name))

        result.append('$var real 24 t systime $end')
        result.append('$upscope $end')
        result.append('$enddefinitions $end')
        return '\n'.join(result) + '\n\n'

    def convert_file(self, infile, outfile, blocksize = 65536):
        '''Convert a complete dump file to VCD format.
        Yields total number of transitions after each block
        '''
        self.reset()

        tcount = 0
        while True:
            data = infile.read(blocksize)
            if not data: break

            trans = self.decode(data)

            if len(trans) and tcount == 0:
                # Write header after we have decoded the first block, which
                # includes format version information.
                outfile.write('\n'.join(self.format_vcd_header()) + '\n\n')

            lines = self.format_vcd(trans)
            outfile.write('\n'.join(lines) + '\n')

            tcount += len(trans)
            yield tcount

if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.stderr.write("Usage:\n")
        sys.stderr.write("    python3 decode_sniff.py sniff.dat        Decode already captured data file\n")
        sys.stderr.write("    python3 decode_sniff.py /dev/ttyACM0     Capture from USB serial port and decode\n")
        sys.exit(1)

    infilename = sys.argv[1]

    if os.path.isfile(infilename):
        infile = open(infilename, 'rb')
        outfilebase = os.path.splitext(infilename)[0]
        total_size = os.path.getsize(infilename)
        print("Decoding from capture file %s to %s.vcd\n" % (infilename, outfilebase))
    else:
        import serial
        infile = serial.Serial(infilename, baudrate=1000000, timeout=5.0)
        total_size = 0
        outfilebase = "sniff"
        print("Capturing from serial port %s to %s.vcd\n" % (infilename, outfilebase))

    outfile = open(outfilebase + '.vcd', 'w')
    decoder = SniffDecoder()
    processed_size = 0
    total_transitions = 0

    try:
        last_status_time = 0
        while True:
            block = infile.read(65536)
            if not block: break

            transitions = decoder.decode(block)

            if processed_size == 0:
                # Write header after we have decoded the first block, which
                # includes format version information.
                outfile.write(decoder.format_vcd_header())
            
            outfile.write(decoder.format_vcd(transitions))
            processed_size += len(block)
            total_transitions += len(transitions)

            if time.time() - last_status_time > 0.5:
                last_status_time = time.time()
                if total_size:
                    sys.stdout.write("Progress: %d/%d kB (%d %%)          \r" % (processed_size // 1024, total_size // 1024, 100 * processed_size // total_size))
                else:
                    sys.stdout.write("Captured %d kB, %d transitions      \r" % (processed_size // 1024, total_transitions))
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("Ending capture")
        infile.close()
        pass

    print("Done, total %d transitions, length %0.3f s,                        " %
          (total_transitions, decoder.last_vcd_timestamp * decoder.divider / decoder.cpu_freq))

    for i, aux_data in enumerate(decoder.aux_data):
        if i == 0: continue

        if aux_data:
            aux_filename = outfilebase + ".aux%x" % i
            open(aux_filename, "wb").write(aux_data)
            print("Saved %d bytes of auxiliary data from channel %x to %s" % (len(aux_data), i, aux_filename))
    
    if decoder.last_vcd_timestamp > 1e9:
        print("WARNING: Result file has a large time range, PulseView may take a lot of memory when loading")
        print("         Consider using 'Compress idle periods' in PulseView VCD Import")
