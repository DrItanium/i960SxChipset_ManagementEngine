/*
i960SxChipset
Copyright (c) 2020-2022, Joshua Scoggins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by jwscoggins on 7/13/22.
//

#ifndef SXCHIPSET_MANAGEMENTENGINE_CONFIGURATIONSPACE_H
#define SXCHIPSET_MANAGEMENTENGINE_CONFIGURATIONSPACE_H
#include <cstdint>
namespace ConfigurationSpace {
    /**
     * @brief A single 256 byte page that is mapped into configuration space which describes a given device
     */
   class Page {
   public:
       virtual ~Page() noexcept = default;
       virtual uint8_t readByte(uint8_t offset) const noexcept {
            switch (offset) {
                case 0: return getType();
                case 1: return getType() >> 8;
                case 2: return getType() >> 16;
                case 3: return getType() >> 24;
                case 4: return getFlags();
                case 5: return getFlags() >> 8;
                case 6: return getFlags() >> 16;
                case 7: return getFlags() >> 24;
            }
           return readByte0(offset);
       }
       virtual uint32_t readWord(uint8_t offset) const noexcept { return readWord0(makeWordAddress(offset)); }
       virtual uint16_t readHalfWord(uint8_t offset) const noexcept { return readHalfWord0(makeHalfWordAddress(offset)); }
       virtual void writeByte(uint8_t offset, uint8_t value) noexcept { writeByte0(offset, value); }
       virtual void writeWord(uint8_t offset, uint32_t value) noexcept { writeWord0(makeWordAddress(offset), value); }
       virtual void writeHalfWord(uint8_t offset, uint16_t value) noexcept { writeHalfWord0(makeHalfWordAddress(offset), value); }
       virtual uint32_t getType() const noexcept {
           return readWord(0);
       }
       virtual uint32_t getFlags() const noexcept {
           return readWord(1);
       }
       virtual uint32_t getBaseAddress() const noexcept {
           // the format is type, flags, baseAddress with type and flags being a 64-bit value
           return readWord(2);
       }
       virtual uint32_t getSize() const noexcept {
           return readWord(3);
       }
       virtual void setBaseAddress(uint32_t baseAddress) noexcept {
           writeWord(2, baseAddress);
       }
   protected:
       static constexpr uint8_t makeHalfWordAddress(uint8_t offset) noexcept { return offset << 1; }
       static constexpr uint8_t makeWordAddress(uint8_t offset) noexcept { return offset << 2; }
   protected:
       virtual uint8_t readByte0(uint8_t offset) const noexcept { return 0; }
       virtual uint16_t readHalfWord0(uint8_t offset) const noexcept { return 0; }
       virtual uint32_t readWord0(uint8_t offset) const noexcept { return 0; }
       virtual void writeByte0(uint8_t offset, uint8_t value) noexcept { }
       virtual void writeWord0(uint8_t offset, uint32_t value) noexcept { }
       virtual void writeHalfWord0(uint8_t offset, uint16_t value) noexcept { }
   };
   class StandardPage : public Page {
   public:
       StandardPage(uint64_t type, uint64_t flags, uint32_t defaultBar, uint32_t size) noexcept : type_(type), flags_(flags), bar_(defaultBar), size_(size) { }
       ~StandardPage() override = default;
       uint64_t getType() const override { return type_; }
       uint64_t getFlags() const override { return flags_; }
   private:
       uint64_t type_;
       uint64_t flags_;
       uint32_t bar_;
       uint32_t size_;
   };
} // end namespace ConfigurationSpace

#endif //SXCHIPSET_MANAGEMENTENGINE_CONFIGURATIONSPACE_H
