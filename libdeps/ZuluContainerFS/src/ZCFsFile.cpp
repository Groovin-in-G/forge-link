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

#include <cstddef>
#include <algorithm>
#include "ZCUtil.h"
#include "ZCFsFile.h"

using namespace ZuluContainerFs;
// The Microsoft VHD standard's footer had two different length depending
// on which version was being used
inline constexpr uint16_t VHD_FOOTER_LEN_POST_2004 {512};
inline constexpr uint16_t VHD_FOOTER_LEN_PRE_2004 {511};
inline constexpr char VHD_COOKIE[] {"conectix"};


ZCFsFile::ZCFsFile()
    : FsFile()
    , m_container_format(Container::None)
    , m_chs({})
    , m_image_size_bytes(0)
{
}

bool ZCFsFile::open(const char *path, oflag_t oflag)
{
    FsFile::open(path, oflag);
    return openCheck();
}
bool ZCFsFile::open(FsBaseFile *dir, const char *path, oflag_t oflag)
{
    FsFile::open(dir, path, oflag);
    return openCheck();

}
bool ZCFsFile::open(FsBaseFile *dir, uint32_t index, oflag_t oflag)
{
    FsFile::open(dir, index, oflag);
    return openCheck();

}
bool ZCFsFile::open(FsVolume *vol, const char *path, oflag_t oflag)
{
    FsFile::open(vol, path, oflag);
    return openCheck();
}

bool ZCFsFile::close()
{
    m_container_format = Container::None;
    m_chs = {};
    m_image_size_bytes = 0;
    return FsFile::close();
}

uint64_t ZCFsFile::size() const
{
    switch (m_container_format)
    {
    case Container::Vhd:
        return m_image_size_bytes;
    default:
        return FsBaseFile::size();
    }
}

ZCFsFile &ZCFsFile::operator=(FsFile && from)
{
    move(&from);
    verifyAndInit();
    return *this;
}

Container ZCFsFile::getContainerFormat() const
{
    return m_container_format;
}


const char* ZCFsFile::getContainerNameCstr() const
{
    switch (m_container_format)
    {
    case Container::None:
        return "none";
    case Container::Vhd:
        return "vhd";
    }
    return "unknown";
}

bool ZCFsFile::isUnsupportedContainerType() const
{
    return !isOpen() && (m_container_format != Container::None);
}

bool ZCFsFile::setCHS(uint16_t &cylinders, uint8_t &heads, uint8_t &sectors)
{
    if (m_container_format != Container::Vhd)
        return false;
    if(m_chs.cylinders == 0 && m_chs.heads == 0 && m_chs.sectors == 0)
        return false;

    cylinders = m_chs.cylinders;
    heads = m_chs.heads;
    sectors = m_chs.sectors;
    return true;

}

bool ZCFsFile::openCheck()
{
    verifyAndInit();
    return isOpen();
}

bool ZCFsFile::verifyAndInit()
{
    reset();
    if (isOpen() && isFile())
    {
        if (verifyAndInitVhd(VHD_FOOTER_LEN_POST_2004))
            return true;
        if ( verifyAndInitVhd(VHD_FOOTER_LEN_PRE_2004))
            return true;
    }
    // Container was found, but container format type is unsupported
    // Reset everything but the format for isUnsupportedType checking
    if (m_container_format != Container::None)
    {
        m_image_size_bytes = 0;
        m_chs = {};
        FsFile::close();
    }
    return false;
}

bool ZCFsFile::verifyAndInitVhd(size_t footer_len)
{
    if(!seekEnd(-static_cast<int64_t>(footer_len)))
        return false;
 
    VhdFooter footer;

    int len = read(static_cast<void*>(&footer), sizeof(footer));

    if (len != sizeof(footer))
    {
        return false;
    }
    // Verify cookie
    bool valid_cookie = true;
    for (size_t i = 0; i < sizeof(footer.cookie); ++i)
    {
        if(VHD_COOKIE[i] != footer.cookie[i])
        {
            valid_cookie = false;
            break;
        }
    }
    if (!valid_cookie)
        return false;

    auto calced_checksum = vhdFooterChecksum(static_cast<void*>(&footer), sizeof(footer));
    if (calced_checksum != swapIntEndian(footer.checksum))
    {
        return false;
    }

    m_container_format = Container::Vhd;

    if (swapIntEndian(footer.disk_type) != 2)
    {
        return false;
    }
 
    m_chs.cylinders = swapIntEndian(footer.cylinders);
    m_chs.heads = swapIntEndian(footer.heads);
    m_chs.sectors = swapIntEndian(footer.sectors_per_track);

    // use image size is smallest, data payload or the current size in the footer
    // they should be equal
    auto file_size = FsBaseFile::size();
    auto adjusted_file_size = file_size > footer_len ? (file_size - footer_len) : 0;
    m_image_size_bytes = std::min(swapIntEndian(footer.current_size), adjusted_file_size);
    return true;
}


uint32_t ZCFsFile::vhdFooterChecksum(const void* footer, size_t len)
{
    auto *data = static_cast<const uint8_t*>(footer);
    uint32_t checksum{0};
    for (size_t i = 0; i < len; i++)
    {
        // skip checksum data
        if (i >= offsetof(VhdFooter, checksum) && i < offsetof(VhdFooter, checksum) + sizeof(uint32_t))
            continue;

        checksum += data[i];
    }
    return ~checksum;
}

void ZCFsFile::reset()
{
    m_chs = {};
    m_container_format = Container::None;
    m_image_size_bytes = 0;
}