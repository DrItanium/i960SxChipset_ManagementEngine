/*
i960SxChipset
Copyright (c) 2020-2021, Joshua Scoggins
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
#include <Arduino.h>
#include "Pinout.h"

namespace {
    constexpr byte MuxTranslationTable[4] {
            0,
            0b0100'0000,
            0b1000'0000,
            0b1100'0000,
    };
    constexpr byte SPIIDTranslationTable[8] {
            0b0000'0000,
            0b0000'1000,
            0b0001'0000,
            0b0001'1000,
            0b0010'0000,
            0b0010'1000,
            0b0011'0000,
            0b0011'1000,
    };
}
void
connectMuxPinsToId(byte id) noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type2()) {
        auto oldValue = PORTD & ~(0b1100'0000); // mask out all bits but the ones we care about
        PORTD = (oldValue | MuxTranslationTable[id & 0b11]);
    }
}

void
setSPIBusId(byte id) noexcept {
    if constexpr (TargetBoard::onAtmega1284p_Type2()) {
        auto oldValue = PORTD & ~(0b0011'1000);
        PORTD = (oldValue | SPIIDTranslationTable[id & 0b111]);
    }
}