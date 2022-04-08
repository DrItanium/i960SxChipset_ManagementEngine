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
//
// Created by jwscoggins on 11/19/21.
//

#ifndef SXCHIPSET_TWELVEWAY_RAND_PLRU_ENTRY_H
#define SXCHIPSET_TWELVEWAY_RAND_PLRU_ENTRY_H
#include "MCUPlatform.h"
#include "Pinout.h"
#include "TaggedCacheAddress.h"
#include "CacheEntry.h"

template<byte numTagBits, byte totalBitCount, byte numLowestBits, typename T, bool debugMode = false>
class TwelveWayRandPLRUCacheWay {
public:
    static constexpr auto NumberOfWays = 12;
    static constexpr auto NumberOfGroups = NumberOfWays / 2;
    static constexpr auto WayMask = NumberOfWays - 1;
    using CacheEntry = ::CacheEntry<numTagBits, totalBitCount, numLowestBits, T, debugMode>;
    using TaggedAddress = typename CacheEntry::TaggedAddress;
    static constexpr auto NumBytesCached = CacheEntry::NumBytesCached;
public:
    CacheEntry& getLine(const TaggedAddress& theAddress) noexcept {
        byte firstInvalid = NumberOfWays;
        for (byte i = 0; i < NumberOfWays; ++i) {
            if (ways_[i].matches(theAddress)) {
                updateFlags(i);
                return ways_[i];
            } else if (firstInvalid == NumberOfWays && !ways_[i].isValid()) {
                firstInvalid = i;
            }
        }
        auto index = firstInvalid != NumberOfWays ? firstInvalid : getLeastRecentlyUsed();
        updateFlags(index);
        ways_[index].reset(theAddress);
        return ways_[index];

    }
    void clear() noexcept {
        for (auto way : ways_) {
            way.clear();
        }
        bits_ = 0;
    }
private:
    void updateFlags(byte index) noexcept {
        constexpr byte masks[NumberOfGroups*2] {
                0b1111110, 0b000001,
                0b1111101, 0b000010,
                0b1111011, 0b000100,
                0b1110111, 0b001000,
                0b1101111, 0b010000,
                0b1011111, 0b100000,
        };
        // taken from the 8way rand plru implementation
        // Take the index provided and see if the least significant bit is zero or not
        // if it is zero then and the tracking bits with the value stored in the masks table
        // if we just used 0, then we do bits_ = bits_ & (0b1110) which will clear the least significant bit
        // if the index is 1 then we do bits_ = bits | (0b0001) which will set the least significant bit
        // when 2 => bits_ &= 0b1101 -> which will clear the next least significant bit
        // and so on.
        if (auto rIndex = index & 0b1111; (rIndex & 0b1) == 0) {
            bits_ &= masks[rIndex];
        } else {
            bits_ |= masks[rIndex];
        }
    }
    [[nodiscard]] byte getLeastRecentlyUsed() const noexcept {
        static constexpr auto NumberOfRandomTableEntries = 256;
        static bool initialized = false;
        static byte counter = 0;
        static byte randomTable[NumberOfRandomTableEntries] = { 0 };
        static constexpr byte secondLookupTable[NumberOfGroups][2] {
                { 1, 0 },
                {3, 2},
                {5, 4},
                {7, 6},
                {9, 8},
                {11, 10},
                //{13, 12},
                //{15, 14},
        };
        static constexpr byte maskLookup[NumberOfGroups] {
                0b00000001,
                0b00000010,
                0b00000100,
                0b00001000,
                0b00010000,
                0b00100000,
                //0b01000000,
                //0b10000000,
        };
        if (!initialized) {
            initialized = true;
            counter = 0;
            for (uint16_t i = 0; i < NumberOfRandomTableEntries; ++i) {
                randomTable[i] = random(0, NumberOfGroups);
            }
        }
        auto theIndex = randomTable[counter++];
        return secondLookupTable[theIndex][(bits_ & maskLookup[theIndex]) ? 1 : 0];
    }
public:
    [[nodiscard]] constexpr size_t size() const noexcept { return NumberOfWays; }
private:
    CacheEntry ways_[NumberOfWays];
    byte bits_ = 0;
};
#endif //SXCHIPSET_TWELVEWAY_RAND_PLRU_ENTRY_H
