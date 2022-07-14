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
       virtual uint8_t readByte(uint8_t offset) noexcept { return 0; }
       virtual uint32_t readWord(uint8_t offset) noexcept { return 0; }
       virtual uint16_t readHalfWord(uint8_t offset) noexcept { return 0; }
       virtual uint64_t readLongWord(uint8_t offset) noexcept { return 0; }
       virtual void writeByte(uint8_t offset, uint8_t value) noexcept { }
       virtual void writeWord(uint8_t offset, uint32_t value) noexcept { }
       virtual void writeHalfWord(uint8_t offset, uint16_t value) noexcept { }
       virtual void writeLongWord(uint8_t offset,uint64_t value) noexcept { }
   };
} // end namespace ConfigurationSpace

#endif //SXCHIPSET_MANAGEMENTENGINE_CONFIGURATIONSPACE_H
