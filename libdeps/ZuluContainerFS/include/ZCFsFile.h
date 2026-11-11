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

#include <FsLib/FsFile.h>

namespace ZuluContainerFs 
{
    enum class Container 
    {
        None,
        Vhd
    };

    struct __attribute__((packed)) VhdFooter {
        char cookie[8];
        uint32_t features;
        uint32_t file_format_version;
        uint64_t data_offset;
        uint32_t time_stamp;
        uint32_t creator_app;
        uint32_t creator_version;
        uint32_t creator_host_os;
        uint64_t original_size;
        uint64_t current_size;
        uint16_t cylinders;
        uint8_t  heads;
        uint8_t  sectors_per_track;
        uint32_t disk_type;
        uint32_t checksum;
        char unique_id[16];
        uint8_t save_state;
    };

    struct CHS
    {
        uint16_t cylinders;
        uint8_t heads;
        uint8_t sectors;
    };

    class ZCFsFile : public FsFile 
    {
    public:
        ZCFsFile();

        // Replacing FsFile/ FsFileBase calls
        bool open(const char *path, oflag_t oflag=0X00);
        bool open(FsBaseFile *dir, const char *path, oflag_t oflag=0X00);
        bool open(FsBaseFile *dir, uint32_t index, oflag_t oflag=0X00);
        bool open(FsVolume *vol, const char *path, oflag_t oflag=0X00);
        bool close();
        uint64_t size() const;
        ZCFsFile &operator=(FsFile &&);

        // Calls unique to this class
        /**
         * Get the current container format
         * \returns the format of the container as a class enum Container value
         */

        virtual Container getContainerFormat() const;
        /**
         * Get the C string name for the container type
         * \returns a C string name of the container type 
         */
        virtual const char* getContainerNameCstr() const;

        /**
         * Determine if the Container iamge type is not supported. Should generally be checked when an open fails
         * Currently only full image allocation is supported
         * \return true if the image is a container, but the type of the container format is unsupported
         *         false if the image isn't a container or the container is supported
         */
        virtual bool isUnsupportedContainerType() const;

        /**
         * Set the parameters with CHS from the metadata from the container
         * \param cylinders set the cylinders
         * \param heads set the heads
         * \param sectors set the sectors per track
         * \returns true if CHS was set, false if the metadata does not contain CHS values
         */
        virtual bool setCHS(uint16_t &cylinders, uint8_t &heads, uint8_t &sectors);

    protected:
        /**
         * Check if the file is open
         * \returns true if the file is open whether it is a container or not, false if the file is closed
         */
        virtual bool openCheck();
        /**
         *  Check all formats, verify that metadata is good, and init the image
         * \returns true if a format is found, false if initialized as a plain iamge
         *  */
        virtual bool verifyAndInit();
        /**
         * Verify Microsoft's vhd file format and initialize the image
         * \param footer_len pass length of the footer from the specific version of vhd
         * \returns true if found a vhd format was initialized, false if format was invalid
         */
        virtual bool verifyAndInitVhd(size_t footer_len);
        /**
         * Calculate the VHD format footer's checksum
         * \param footer a void pointer to the VhdFooter struct
         * \param len the length of the VhdFooter struct
         * \return the VHD footer's checksum
         */
        virtual uint32_t vhdFooterChecksum(const void* footer, size_t len);
        /**
         * Reset private values of the ZCFsFile class
         */
        virtual void reset();

    private:
        Container m_container_format {Container::None};
        CHS m_chs {};
        uint64_t m_image_size_bytes {};
    };
}