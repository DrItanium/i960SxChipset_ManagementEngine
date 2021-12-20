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
    enum class MCP23x17Registers : byte {
        IODIRA = 0,
        IODIRB,
        IPOLA,
        IPOLB,
        GPINTENA,
        GPINTENB,
        DEFVALA,
        DEFVALB,
        INTCONA,
        INTCONB,
        _IOCONA,
        _IOCONB,
        GPPUA,
        GPPUB,
        INTFA,
        INTFB,
        INTCAPA,
        INTCAPB,
        GPIOA,
        GPIOB,
        OLATA,
        OLATB,
        OLAT = OLATA,
        GPIO = GPIOA,
        IOCON = _IOCONA,
        IODIR = IODIRA,
        INTCAP = INTCAPA,
        INTF = INTFA,
        GPPU = GPPUA,
        INTCON = INTCONA,
        DEFVAL = DEFVALA,
        GPINTEN = GPINTENA,
        IPOL = IPOLA,
    };
    enum class IOExpanderAddress : byte {
        DataLines = 0b0000,
        Lower16Lines = 0b0010,
        Upper16Lines = 0b0100,
        OtherDevice0 = 0b0110,
        OtherDevice1 = 0b1000,
        OtherDevice2 = 0b1010,
        OtherDevice3 = 0b1100,
        OtherDevice4 = 0b1110,
    };
    static constexpr byte generateReadOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0001 | static_cast<uint8_t>(address);
    }
    static constexpr byte generateWriteOpcode(ProcessorInterface::IOExpanderAddress address) noexcept {
        return 0b0100'0000 | static_cast<uint8_t>(address);
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static inline SplitWord16 read16() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        SplitWord16 output(0);
        DigitalPin<i960Pinout::GPIOSelect>::assertPin();
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        output.bytes[0] = SPI.transfer(0);
        output.bytes[1] = SPI.transfer(0);
        DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
        return output;
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static inline uint8_t read8() noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        DigitalPin<i960Pinout::GPIOSelect>::assertPin();
        SPI.transfer(generateReadOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        auto outcome = SPI.transfer(0);
        DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
      return outcome;
    }

    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static inline void write16(uint16_t value) noexcept {
        SplitWord16 valueDiv(value);
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        DigitalPin<i960Pinout::GPIOSelect>::assertPin();
        SPI.transfer(generateWriteOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(valueDiv.bytes[0]);
        SPI.transfer(valueDiv.bytes[1]);
        DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, MCP23x17Registers opcode, bool standalone = true>
    static inline void write8(uint8_t value) noexcept {
        if constexpr (standalone) {
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
        }
        DigitalPin<i960Pinout::GPIOSelect>::assertPin();
        SPI.transfer(generateWriteOpcode(addr));
        SPI.transfer(static_cast<byte>(opcode));
        SPI.transfer(value);
        DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
        if constexpr (standalone) {
            SPI.endTransaction();
        }
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline SplitWord16 readGPIO16() noexcept {
        return read16<addr, MCP23x17Registers::GPIO, standalone>();
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeGPIO16(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::GPIO, standalone>(value);
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeDirection(uint16_t value) noexcept {
        write16<addr, MCP23x17Registers::IODIR, standalone>(value);
    }
public:
    ProcessorInterface() = delete;
    ~ProcessorInterface() = delete;
    ProcessorInterface(const ProcessorInterface&) = delete;
    ProcessorInterface(ProcessorInterface&&) = delete;
    ProcessorInterface& operator=(const ProcessorInterface&) = delete;
    ProcessorInterface& operator=(ProcessorInterface&&) = delete;
public:
    static void begin() noexcept;
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    [[nodiscard]] static SplitWord16 getDataBits() noexcept {
        return readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
    }
    static void setDataBits(uint16_t value) noexcept {
        // the latch is preserved in between data line changes
        // okay we are still pointing as output values
        // check the latch and see if the output value is the same as what is latched
        if (latchedDataOutput.getWholeValue() != value) {
            latchedDataOutput.wholeValue_ = value;
            writeGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>(latchedDataOutput.getWholeValue());
        }
    }
    [[nodiscard]] static auto getStyle() noexcept {
        auto lower = static_cast<byte>(DigitalPin<i960Pinout::BE0>::read());
        auto upper = static_cast<byte>(DigitalPin<i960Pinout::BE1>::read()) << 1;
        return static_cast<LoadStoreStyle>(lower | upper);
    }
    [[nodiscard]] static bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return cacheOffsetEntry_; }
    inline static void setupDataLinesForWrite() noexcept {
        if (!dataLinesDirection_) {
            dataLinesDirection_ = ~dataLinesDirection_;
            writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0xFFFF);
        }
    }
    inline static void setupDataLinesForRead() noexcept {
            if (dataLinesDirection_) {
                dataLinesDirection_ = ~dataLinesDirection_;
                writeDirection<ProcessorInterface::IOExpanderAddress::DataLines>(0);
            }
    }
private:
    template<bool useInterrupts = true>
    static byte getUpdateKind() noexcept {
        if constexpr (!useInterrupts) {
            return 0;
        } else {
            auto a = static_cast<byte>(DigitalPin<i960Pinout::INT_EN0>::read());
            auto b = static_cast<byte>(DigitalPin<i960Pinout::INT_EN1>::read()) << 1;
            auto c = static_cast<byte>(DigitalPin<i960Pinout::INT_EN2>::read()) << 2;
            auto d = static_cast<byte>(DigitalPin<i960Pinout::INT_EN3>::read()) << 3;
            return a | b | c | d;
        }
    }
private:
    template<byte offsetMask>
    inline static void updateCacheOffsetEntry() noexcept {
        cacheOffsetEntry_ = (address_.bytes[0] >> 1) & offsetMask; // we want to make this quick to increment
    }
public:
    template<byte offsetMask>
    inline static void full32BitUpdate() noexcept {
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        address_.setLowerHalf(readGPIO16<IOExpanderAddress::Lower16Lines>());
        address_.setUpperHalf(readGPIO16<IOExpanderAddress::Upper16Lines>());
        updateCacheOffsetEntry<offsetMask>();
    }
    template<byte offsetMask>
    static void lower16Update() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        address_.setLowerHalf(readGPIO16<IOExpanderAddress::Lower16Lines>());
        updateCacheOffsetEntry<offsetMask>();
    }
    static void upper16Update() noexcept {
        // only read the upper 16-bits
        address_.setUpperHalf(readGPIO16<IOExpanderAddress::Upper16Lines>());
    }
    static void updateHighest8() noexcept {
        // only read the upper 8 bits
        address_.bytes[3] = read8<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPIOB>();
    }
    static void updateHigher8() noexcept {
        // only read the upper 8 bits
        address_.bytes[2] = read8<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPIOA>();
    }
    template<byte offsetMask>
    static void updateLowest8() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        address_.bytes[0] = read8<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPIOA>();
        updateCacheOffsetEntry<offsetMask>();
    }
    static void updateLower8() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        address_.bytes[1] = read8<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPIOB>();
    }
private:
    template<bool inDebugMode>
    inline static void updateTargetFunctions() noexcept {
        if constexpr (auto a = getBody<inDebugMode>(address_.bytes[3]); inDebugMode) {
            lastDebug_ = a;
        } else {
            last_ = a;
        }
    }
public:
    template<bool inDebugMode, byte offsetMask, bool useInterrupts = true, int debugLevel = 0>
    static void newDataCycle() noexcept {
        switch (getUpdateKind<useInterrupts>() & 0b1111) {
            case 0b0001:
                updateLower8();
                upper16Update();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0010:
                updateLowest8<offsetMask>();
                upper16Update();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0011:
                upper16Update();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0100:
                lower16Update<offsetMask>();
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0101:
                updateLower8();
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0110:
                updateLowest8<offsetMask>();
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b0111:
                updateHighest8();
                updateTargetFunctions<inDebugMode>();
                break;
            case 0b1000:
                lower16Update<offsetMask>();
                updateHigher8();
                break;
            case 0b1001:
                updateHigher8();
                updateLower8();
                break;
            case 0b1010:
                updateHigher8();
                updateLowest8<offsetMask>();
                break;
            case 0b1011:
                updateHigher8();
                break;
            case 0b1100:
                lower16Update<offsetMask>();
                break;
            case 0b1101:
                updateLower8();
                break;
            case 0b1110:
                updateLowest8<offsetMask>();
                break;
            case 0b1111: break;
            default:
                full32BitUpdate<offsetMask>();
                updateTargetFunctions<inDebugMode>();
                break;
        }
        if constexpr (inDebugMode) {
            lastDebug_();
        } else {
            last_();
        }
    }
    template<bool advanceAddress = true>
    static void burstNext() noexcept {
        if constexpr (advanceAddress) {
            address_.wholeValue_ += 2;
        }
    }
    /**
     * @brief Return the least significant byte of the address, useful for CoreChipsetFeatures
     * @return The LSB of the address
     */
    [[nodiscard]] static auto getPageOffset() noexcept { return address_.bytes[0]; }
    [[nodiscard]] static auto getPageIndex() noexcept { return address_.bytes[1]; }
private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline byte dataLinesDirection_ = 0xFF;
    static inline byte cacheOffsetEntry_ = 0;
    static inline bool initialized_ = false;
    static inline BodyFunction last_ = nullptr;
    static inline BodyFunction lastDebug_ = nullptr;
};

#endif //ARDUINO_IOEXPANDERS_H
