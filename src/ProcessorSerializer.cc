/*
SxChipset_ManagementEngine
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
#include "ProcessorSerializer.h"

SplitWord16
ProcessorInterface::getHalfAddress() noexcept {
    return extractAddress(DigitalPin<i960Pinout::MUXADR0>::readInPort());
}
void
ProcessorInterface::setupDataLinesForWrite() noexcept {
    static constexpr uint32_t PortDirectionMask = 0x0003FCFF;
    static constexpr uint32_t InvertPortDirectionMask = ~PortDirectionMask;
    auto portDirection = DigitalPin<i960Pinout::Data0>::readPortDir();
    portDirection &= InvertPortDirectionMask;
    DigitalPin<i960Pinout::Data0>::writePortDir(portDirection);
}
void
ProcessorInterface::setupDataLinesForRead() noexcept {
    static constexpr uint32_t PortDirectionMask = 0x0003FCFF;
    static constexpr uint32_t InvertPortDirectionMask = ~PortDirectionMask;
    auto portDirection = DigitalPin<i960Pinout::Data0>::readPortDir();
    portDirection &= InvertPortDirectionMask;
    DigitalPin<i960Pinout::Data0>::writePortDir(portDirection | PortDirectionMask);
}
bool
ProcessorInterface::isReadOperation() noexcept {
    return DigitalPin<i960Pinout::W_R_>::isAsserted();
}
SplitWord16
ProcessorInterface::getDataBits() noexcept {
    auto portContents = DigitalPin<i960Pinout::Data0>::readInPort();
    SplitWord16 result{0};
    result.bytes[0] = static_cast<byte>(portContents);
    result.bytes[1] = static_cast<byte>(portContents >> 10);
    //delayMicroseconds(10);
    return result;
}
void
ProcessorInterface::setDataBits(uint16_t value) noexcept {
    // the latch is preserved in between data line changes
    // okay we are still pointing as output values
    // check the latch and see if the output value is the same as what is latched
    constexpr uint32_t normalMask = 0x0003FCFF;
    constexpr uint32_t invertMask = ~normalMask;
    SplitWord16 data(value);
    auto contents = (static_cast<uint32_t>(data.bytes[0]) | (static_cast<uint32_t>(data.bytes[1]) << 10)) & normalMask;
    auto portContents = DigitalPin<i960Pinout::Data0>::readOutPort() & invertMask;
    auto output = contents | portContents;
    DigitalPin<i960Pinout::Data0>::writeOutPort(output);
    //delayMicroseconds(10);
};
LoadStoreStyle
ProcessorInterface::getStyle() noexcept {
    auto lower = static_cast<byte>(DigitalPin<i960Pinout::BE0>::read());
    auto upper = static_cast<byte>(DigitalPin<i960Pinout::BE1>::read()) << 1;
    return static_cast<LoadStoreStyle>(lower | upper);
}
void
ProcessorInterface::begin() noexcept {
    updateTargetFunctions<true>();
    updateTargetFunctions<false>();
}
