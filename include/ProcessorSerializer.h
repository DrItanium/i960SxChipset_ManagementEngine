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
    template<IOExpanderAddress addr, MCP23x17Registers reg, bool standalone = true>
    static inline void writeGPIO8(uint8_t value) noexcept {
        static_assert(reg == MCP23x17Registers::GPIOA ||
                      reg == MCP23x17Registers::GPIOB ||
                      reg == MCP23x17Registers::GPIO, "Register must be one of the GPIO addresses for the MCP23S17");
        write8<addr, reg, standalone>(value);
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeGPIOUpper(uint8_t value) noexcept {
        writeGPIO8<addr, MCP23x17Registers::GPIOB, standalone>(value);
    }
    template<IOExpanderAddress addr, bool standalone = true>
    static inline void writeGPIOLower(uint8_t value) noexcept {
        writeGPIO8<addr, MCP23x17Registers::GPIOA, standalone>(value);
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
    [[nodiscard]] static constexpr Address getAddress() noexcept { return address_.getWholeValue(); }
    [[nodiscard]] static SplitWord16 getDataBits() noexcept {
        auto portContents = DigitalPin<i960Pinout::Data0>::readInPort();
        SplitWord16 result{0};
        result.bytes[0] = static_cast<byte>(portContents);
        result.bytes[1] = static_cast<byte>(portContents >> 10);
#if 0
        Serial.println("{");
        Serial.print("\tP00 Result: 0x");
        Serial.println(result.getWholeValue(), HEX);
        auto spiResult = readGPIO16<ProcessorInterface::IOExpanderAddress::DataLines>();
        Serial.print("\tSPI Result: 0x");
        Serial.println(spiResult.getWholeValue(), HEX);
        Serial.println("}");
        return spiResult;
#endif
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
    };
    union PortAInput {
        uint32_t raw = 0;
        struct {
            // 8 bits
            Address : 4;
            Address successfulBoot : 1;
            Address writeRead : 1;
            Address byteEnable : 2;
            // 11 bits
            Address : 4;
            Address doCycle : 1;
            Address inTransaction : 1;
            Address burstNext : 1;
            Address lineInterrupt : 4;
            Address : 13;
        };
    } ;
    [[nodiscard]] static LoadStoreStyle getStyle() noexcept {
        if constexpr (TargetBoard::usePortReads()) {
            PortAInput tmp;
            // assume for now that BE0 and BE1 are the same port
            // in this case it is PA14 and PA15 so just shift right by 14 and mask
            tmp.raw = DigitalPin<i960Pinout::BE0>::readInPort();
            return static_cast<LoadStoreStyle>(tmp.byteEnable);
        } else {
            auto lower = static_cast<byte>(DigitalPin<i960Pinout::BE0>::read());
            auto upper = static_cast<byte>(DigitalPin<i960Pinout::BE1>::read()) << 1;
            return static_cast<LoadStoreStyle>(lower | upper);
        }
    }
    [[nodiscard]] static bool isReadOperation() noexcept { return DigitalPin<i960Pinout::W_R_>::isAsserted(); }
    template<byte offsetMask>
    [[nodiscard]] static auto getCacheOffsetEntry() noexcept { return (address_.bytes[0] >> 1) & offsetMask; }
    inline static void setupDataLinesForWrite() noexcept {
        static constexpr uint32_t PortDirectionMask = 0x0003FCFF;
        static constexpr uint32_t InvertPortDirectionMask = ~PortDirectionMask;
        auto portDirection = DigitalPin<i960Pinout::Data0>::readPortDir();
        portDirection &= InvertPortDirectionMask;
        DigitalPin<i960Pinout::Data0>::writePortDir(portDirection);
    }
    inline static void setupDataLinesForRead() noexcept {
        static constexpr uint32_t PortDirectionMask = 0x0003FCFF;
        static constexpr uint32_t InvertPortDirectionMask = ~PortDirectionMask;
        auto portDirection = DigitalPin<i960Pinout::Data0>::readPortDir();
        portDirection &= InvertPortDirectionMask;
        DigitalPin<i960Pinout::Data0>::writePortDir(portDirection | PortDirectionMask);
    }
private:
    static byte getUpdateKind() noexcept {
        if constexpr (TargetBoard::addressViaSPI()) {
            if constexpr (TargetBoard::useIOExpanderAddressLineInterrupts()) {
                if constexpr (TargetBoard::usePortReads()) {
                    // in this case I know that INT_EN[0,3] are lined up correctly so this is a very cheap operation
                    PortAInput tmp;
                    tmp.raw = DigitalPin<i960Pinout::INT_EN0>::readInPort();
                    // force the upper two bits low in all cases
                    return static_cast<byte>(tmp.lineInterrupt);
                } else {
                    auto a = static_cast<byte>(DigitalPin<i960Pinout::INT_EN0>::read());
                    auto b = static_cast<byte>(DigitalPin<i960Pinout::INT_EN1>::read()) << 1;
                    auto c = static_cast<byte>(DigitalPin<i960Pinout::INT_EN2>::read()) << 2;
                    auto d = static_cast<byte>(DigitalPin<i960Pinout::INT_EN3>::read()) << 3;
                    return a | b | c | d;
                }
            } else {
                return 0;
            }
        } else {
           // okay access it via parallel so just return read all
           return 0;
        }
    }
private:
    template<bool inDebugMode>
    inline static void updateTargetFunctions() noexcept {
        if constexpr (TargetBoard::separateReadWriteFunctionPointers()) {
            if constexpr (auto [a, b] = getSplitBody<inDebugMode>(address_.bytes[3]); inDebugMode) {
                lastReadDebug_ = a;
                lastWriteDebug_ = b;
            } else {
                lastRead_ = a;
                lastWrite_ = b;
            }
        } else {
            if constexpr (auto a = getBody<inDebugMode>(address_.bytes[3]); inDebugMode) {
                lastDebug_ = a;
            } else {
                last_ = a;
            }
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
        auto channelA = DigitalPin<i960Pinout::MUXADR0>::readInPort();
        return extractAddress(channelA);
    }
public:
    template<bool inDebugMode>
    static void full32BitUpdate() noexcept {
        if constexpr (TargetBoard::addressViaSPI()) {
            // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
            // where we can insert operations to take place that would otherwise be waiting
            address_.setLowerHalf(readGPIO16<IOExpanderAddress::Lower16Lines>());
            address_.setUpperHalf(readGPIO16<IOExpanderAddress::Upper16Lines>());
        } else {
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
        }
        updateTargetFunctions<inDebugMode>();
    }
    static inline void lower16Update() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        address_.setLowerHalf(readGPIO16<IOExpanderAddress::Lower16Lines>());
    }
    template<bool inDebugMode>
    static inline void upper16Update() noexcept {
        // only read the upper 16-bits
        address_.setUpperHalf(readGPIO16<IOExpanderAddress::Upper16Lines>());
        updateTargetFunctions<inDebugMode>();
    }
    template<bool inDebugMode>
    static inline void updateHighest8() noexcept {
        // only read the upper 8 bits
        address_.bytes[3] = read8<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPIOB>();
        updateTargetFunctions<inDebugMode>();
    }
    static inline void updateHigher8() noexcept {
        // only read the upper 8 bits
        address_.bytes[2] = read8<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPIOA>();
    }
    static inline void updateLowest8() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        address_.bytes[0] = read8<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPIOA>();
    }
    static inline void updateLower8() noexcept {
        // read only the lower half
        // we want to overlay actions as much as possible during spi transfers, there are blocks of waiting for a transfer to take place
        // where we can insert operations to take place that would otherwise be waiting
        address_.bytes[1] = read8<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPIOB>();
    }

    union PortDecomposition {
        uint32_t raw;
        struct {
            uint32_t lowerHalf : 8;
            uint32_t upperHalf_Lower2 : 2;
            uint32_t upper: 8; // lowest two bits will be ignored
        };
        inline constexpr SplitWord16 extractAddress() const noexcept {
            SplitWord16 temp{0};
            temp.bytes[0] = lowerHalf;
            uint8_t fractional = upper & 0b1111'1100;
            fractional |= upperHalf_Lower2;
            temp.bytes[1] = fractional;
            return temp;
        }
    };
    template<bool inDebugMode>
    static inline void newDataCycle() noexcept {
        switch (getUpdateKind()) {
            case 0b0001:
                upper16Update<inDebugMode>();
                updateLower8();
                break;
            case 0b0010:
                upper16Update<inDebugMode>();
                updateLowest8();
                break;
            case 0b0011:
                upper16Update<inDebugMode>();
                break;
            case 0b0100:
                updateHighest8<inDebugMode>();
                lower16Update();
                break;
            case 0b0101:
                updateHighest8<inDebugMode>();
                updateLower8();
                break;
            case 0b0110:
                updateHighest8<inDebugMode>();
                updateLowest8();
                break;
            case 0b0111:
                updateHighest8<inDebugMode>();
                break;
            case 0b1000:
                updateHigher8();
                lower16Update();
                break;
            case 0b1001:
                updateHigher8();
                updateLower8();
                break;
            case 0b1010:
                updateHigher8();
                updateLowest8();
                break;
            case 0b1011:
                updateHigher8();
                break;
            case 0b1100:
                lower16Update();
                break;
            case 0b1101:
                updateLower8();
                break;
            case 0b1110:
                updateLowest8();
                break;
            case 0b1111: break;
            default:
                full32BitUpdate<inDebugMode>();
                break;
        }
        Serial.print(F("Address: 0x"));
        Serial.println(address_.getWholeValue(), HEX);
        /// @todo implement parallel read support
        if constexpr (TargetBoard::separateReadWriteFunctionPointers()) {
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
        } else {
            if constexpr (inDebugMode) {
                lastDebug_();
            } else {
                last_();
            }
        }
    }
    template<bool advanceAddress = true>
    [[gnu::always_inline]]
    static inline void burstNext() noexcept {
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
    static void begin() noexcept {
        if (!initialized_) {
            initialized_ = true;
            SPI.beginTransaction(SPISettings(TargetBoard::runIOExpanderSPIInterfaceAt(), MSBFIRST, SPI_MODE0));
            DigitalPin<i960Pinout::GPIOSelect>::configure();
            DigitalPin<i960Pinout::GPIOSelect>::deassertPin();
            // at bootup, the IOExpanders all respond to 0b000 because IOCON.HAEN is
            // disabled. We can send out a single IOCON.HAEN enable message and all
            // should receive it.
            // so do a begin operation on all chips (0b000)
            // set IOCON.HAEN on all chips
            write8<ProcessorInterface::IOExpanderAddress::DataLines, MCP23x17Registers::IOCON, false>(0b0000'1000);
            write8<ProcessorInterface::IOExpanderAddress::Upper16Lines, MCP23x17Registers::IOCON, false>(0b0000'1000);
            write8<ProcessorInterface::IOExpanderAddress::Lower16Lines, MCP23x17Registers::IOCON, false>(0b0000'1000);
            writeDirection<IOExpanderAddress::Lower16Lines, false>(0xFFFF);
            writeDirection<IOExpanderAddress::Upper16Lines, false>(0xFFFF);
            writeDirection<IOExpanderAddress::DataLines, false>(0xFFFF);
            write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF);
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::GPINTEN, false>(0xFFFF);
            write16<IOExpanderAddress::Lower16Lines, MCP23x17Registers::INTCON, false>(0x0000);
            write16<IOExpanderAddress::Upper16Lines, MCP23x17Registers::INTCON, false>(0x0000);
            write16<IOExpanderAddress::DataLines, MCP23x17Registers::OLAT, false>(latchedDataOutput.getWholeValue());
            updateTargetFunctions<true>();
            updateTargetFunctions<false>();
            SPI.endTransaction();
        }
    }
private:
    static inline SplitWord32 address_{0};
    static inline SplitWord16 latchedDataOutput {0};
    static inline uint32_t latchedPortContents = 0;
    static inline byte dataLinesDirection_ = 0xFF;
    static inline bool initialized_ = false;
    static inline BodyFunction last_ = nullptr;
    static inline BodyFunction lastDebug_ = nullptr;
    static inline BodyFunction lastRead_ = nullptr;
    static inline BodyFunction lastWrite_ = nullptr;
    static inline BodyFunction lastReadDebug_ = nullptr;
    static inline BodyFunction lastWriteDebug_ = nullptr;
};

#endif //ARDUINO_IOEXPANDERS_H
