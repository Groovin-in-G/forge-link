/**
 * ZuluContainerFS - Copyright (c) 2024 Rabbit Hole Computing™
 *
 * Designed for ZuluIDE and ZuluSCSI
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


// All calls like open and close must be made on ZCFsFile class's object
// as there is no way to override FsFile calls

#pragma once
#include <cstdint>

namespace ZuluContainerFs 
{
    /**
     * Swaps the Endianess of integer type like uint16 and int64
     * behavior is undefined for non integer type
     * \param integer the integer to be swapped
     * \returns the swapped integer value
     */
    template <typename T> T swapIntEndian(const T &integer)
    {
        if (sizeof(T) == 1)
            return integer;

        T swappedEndian = 0;
        for (unsigned int i = 0; i < sizeof(T); ++i)
        {
            (reinterpret_cast<uint8_t*>(&swappedEndian))[i] = (reinterpret_cast<const uint8_t*>(&integer))[sizeof(T) - 1 - i]; 
        }
        return swappedEndian;
    }
}