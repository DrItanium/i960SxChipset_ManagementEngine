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

#ifndef ARDUINO_IOEXPANDERS_H
#define ARDUINO_IOEXPANDERS_H
#include <Arduino.h>
#include <SPI.h>
#include "Pinout.h"
#include "i960SxChipset.h"

class ProcessorInterface {
public:
    ProcessorInterface() = delete;
    ~ProcessorInterface() = delete;
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
public:
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    [[nodiscard]] static SplitWord16 getDataBits() noexcept {
        auto portContents = DigitalPin<i960Pinout::Data0>::readInPort();
        SplitWord16 result{0};
        result.bytes[0] = static_cast<byte>(portContents);
        result.bytes[1] = static_cast<byte>(portContents >> 10);
        //delayMicroseconds(10);
        return result;
    }
    static void setDataBits(uint16_t value) noexcept {
        // the latch is preserved in between data line changes
        // okay we are still pointing as output values
        // check the latch and see if the output value is the same as what is latched
        constexpr uint32_t normalMask = 0x0003FCFF;
        constexpr uint32_t invertMask = ~normalMask;
        if (latchedDataOutput.getWholeValue() != value) {
            latchedDataOutput.wholeValue_ = value;
            latchedPortContents = (static_cast<uint32_t>(latchedDataOutput.bytes[0]) | (static_cast<uint32_t>(latchedDataOutput.bytes[1]) << 10)) & normalMask;
        }
        auto portContents = DigitalPin<i960Pinout::Data0>::readOutPort() & invertMask;
        auto output = latchedPortContents | portContents;
        DigitalPin<i960Pinout::Data0>::writeOutPort(output);
        //delayMicroseconds(10);
    };
    [[nodiscard]] static LoadStoreStyle getStyle() noexcept {
        auto lower = static_cast<byte>(DigitalPin<i960Pinout::BE0>::read());
        auto upper = static_cast<byte>(DigitalPin<i960Pinout::BE1>::read()) << 1;
        return static_cast<LoadStoreStyle>(lower | upper);
    }
    [[nodiscard]] static bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
    template<byte offsetMask>
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return (address_.bytes[0] >> 1) & offsetMask; }
    static void setupDataLinesForWrite() noexcept {
        static constexpr uint32_t PortDirectionMask = 0x0003FCFF;
        static constexpr uint32_t InvertPortDirectionMask = ~PortDirectionMask;
        auto portDirection = DigitalPin<i960Pinout::Data0>::readPortDir();
        portDirection &= InvertPortDirectionMask;
        DigitalPin<i960Pinout::Data0>::writePortDir(portDirection);
    }
    static void setupDataLinesForRead() noexcept {
        static constexpr uint32_t PortDirectionMask = 0x0003FCFF;
        static constexpr uint32_t InvertPortDirectionMask = ~PortDirectionMask;
        auto portDirection = DigitalPin<i960Pinout::Data0>::readPortDir();
        portDirection &= InvertPortDirectionMask;
        DigitalPin<i960Pinout::Data0>::writePortDir(portDirection | PortDirectionMask);
    }
private:
    template<bool inDebugMode>
    static void updateTargetFunctions() noexcept {
        if constexpr (auto [a, b] = getSplitBody<inDebugMode>(address_.bytes[3]); inDebugMode) {
            lastReadDebug_ = a;
            lastWriteDebug_ = b;
        } else {
            lastRead_ = a;
            lastWrite_ = b;
        }
    }
public:
    static constexpr SplitWord16 extractAddress(uint32_t value) noexcept {
        // okay first step is to get the part of the value that we actually care about
        constexpr uint32_t LowerPortion =  0b0000000000000000'0000001111111111;
        constexpr uint32_t UpperPortion =  0b0000000000000011'1111000000000000;
        auto lowerPart = LowerPortion & value;
        auto upperPart = (UpperPortion & value) >> 2;
        // The AHD158 inverts the output
        return SplitWord16(~static_cast<uint16_t>(lowerPart | upperPart));
    }

private:
    static SplitWord16 getHalfAddress() noexcept {
        return extractAddress(DigitalPin<i960Pinout::MUXADR0>::readInPort());
    }
public:
    template<bool inDebugMode>
    static void full32BitUpdate() noexcept {
        // The multiplexed address lines are not mapped 01/23
        // when high: 0-7, 16-23
        // when low: 8-15, 24-31
        // They are laid out 13/02 so it is necessary to unpack them and assign them as needed
        digitalWrite<i960Pinout::MUXSel0, LOW>();
        auto addressA = getHalfAddress();
        digitalWrite<i960Pinout::MUXSel0, HIGH>();
        auto addressB = getHalfAddress();
        auto lower = addressA.getLowerHalf();
        auto highest = addressA.getUpperHalf();
        auto lowest = addressB.getLowerHalf();
        auto higher = addressB.getUpperHalf();
        address_.bytes[0] = lowest;
        address_.bytes[1] = lower;
        address_.bytes[2] = higher;
        address_.bytes[3] = highest;
        updateTargetFunctions<inDebugMode>();
    }

    template<bool inDebugMode>
    static void newDataCycle() noexcept {
        full32BitUpdate<inDebugMode>();
        //delayMicroseconds(10); // this gets rid of a chipset halt problem that I'm not sure where is coming from
        //Serial.print(F("Address 0x")); Serial.println(address_.getWholeValue(), HEX);
        if (ProcessorInterface::isReadOperation()) {
            setupDataLinesForRead();
            if constexpr (inDebugMode) {
                lastReadDebug_();
            } else {
                lastRead_();
            }
        } else {
            setupDataLinesForWrite();
            if constexpr (inDebugMode) {
                lastWriteDebug_();
            } else {
                lastWrite_();
            }
        }
    }
    template<bool advanceAddress = true>
    static void burstNext() noexcept {
        if constexpr (advanceAddress) {
            address_.wholeValue_ += 2;
        }
        // a test to see if we are running too fast and not allowing enough charge time between transaction points
        //delayMicroseconds(10);
    }
    /**
     * @brief Return the least significant byte of the address, useful for CoreChipsetFeatures
     * @return The LSB of the address
     */
    [[nodiscard]] static auto getPageOffset() noexcept { return address_.bytes[0]; }
    [[nodiscard]] static auto getPageIndex() noexcept { return address_.bytes[1]; }
    static void begin() noexcept {
        updateTargetFunctions<true>();
        updateTargetFunctions<false>();
    }
private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline uint32_t latchedPortContents = 0;
    static inline BodyFunction lastRead_ = nullptr;
    static inline BodyFunction lastWrite_ = nullptr;
    static inline BodyFunction lastReadDebug_ = nullptr;
    static inline BodyFunction lastWriteDebug_ = nullptr;
};

#endif //ARDUINO_IOEXPANDERS_H
