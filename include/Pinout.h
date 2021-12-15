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

#ifndef ARDUINO_PINOUT_H
#define ARDUINO_PINOUT_H
#include <Arduino.h>
#include "MCUPlatform.h"
using Address = uint32_t;
/**
 * @brief Sx Load/Store styles that the processor will request
 */
enum class LoadStoreStyle : uint8_t {
    // based off of BE0,BE1 pins
#if 0
    Full16 = 0b00,
    Upper8 = 0b01,
    Lower8 = 0b10,
    None = 0b11,
#else
// no need to shift
    Full16 = 0b0000'0000,
    Upper8 = 0b0001'0000,
    Lower8 = 0b0010'0000,
    None = 0b0011'0000,
#endif
};
/// @todo fix this pinout for different targets
enum class i960Pinout : int {
#ifdef CHIPSET_TYPE1
#include "Type1Pinout.def"
#elif defined(CHIPSET_TYPE2)
#include "Type2Pinout.def"
#elif defined(CHIPSET_TYPE1_4)
#include "Type1_4Pinout.def"
#elif defined(CHIPSET_TYPE3)
#include "Type3Pinout.def"
#else
#error "Target Chipset Hardware has no pinout defined"
#endif
};
[[gnu::always_inline]]
inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

[[gnu::always_inline]]
inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}
[[gnu::always_inline]]
inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
}
template<i960Pinout pin>
constexpr auto isValidPin960_v = static_cast<int>(pin) < static_cast<int>(i960Pinout::Count);
#include "Atmega1284p_Pinout.h"
#include "GrandCentralM4_Pinout.h"
template<i960Pinout pin>
[[gnu::always_inline]] inline void digitalWrite(bool level) noexcept {
    digitalWrite<pin>(level ? HIGH : LOW);
}

template<i960Pinout pin, decltype(HIGH) switchTo = LOW>
[[gnu::always_inline]]
inline void pulse() noexcept {
    // use the switch to value to compute what to revert to
    digitalWrite<pin, switchTo>();
    digitalWrite<pin, ((switchTo == LOW) ? HIGH : LOW)>();
}


template<i960Pinout pin>
struct DigitalPin {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr bool isInputPin() noexcept { return false; }
    static constexpr bool isOutputPin() noexcept { return false; }
    static constexpr bool getDirection() noexcept { return false; }
    static constexpr auto getPin() noexcept { return pin; }
    static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }
};

#define DefOutputPin(pin, asserted, deasserted) \
    template<> \
    struct DigitalPin< pin > { \
    static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
        DigitalPin() = delete; \
        ~DigitalPin() = delete; \
        DigitalPin(const DigitalPin&) = delete; \
        DigitalPin(DigitalPin&&) = delete; \
        DigitalPin& operator=(const DigitalPin&) = delete; \
        DigitalPin& operator=(DigitalPin&&) = delete; \
        static constexpr auto isInputPin() noexcept { return false; } \
        static constexpr auto isOutputPin() noexcept { return true; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return OUTPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; }      \
        [[gnu::always_inline]] inline static void assertPin() noexcept { digitalWrite<pin,getAssertionState()>(); } \
        [[gnu::always_inline]] inline static void deassertPin() noexcept { digitalWrite<pin,getDeassertionState()>(); } \
        [[gnu::always_inline]] inline static void write(decltype(LOW) value) noexcept { digitalWrite<pin>(value); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }          \
        template<decltype(LOW) switchTo = LOW>  \
        [[gnu::always_inline]] inline static void pulse() noexcept { ::pulse<pin, switchTo>(); }  \
        [[gnu::always_inline]] static void configure() noexcept { pinMode(pin, OUTPUT); } \
    }
#define DefInputPin(pin, asserted, deasserted) \
    template<> \
    struct DigitalPin< pin > { \
        static_assert(asserted != deasserted, "Asserted and deasserted must not be equal!"); \
        DigitalPin() = delete; \
        ~DigitalPin() = delete; \
        DigitalPin(const DigitalPin&) = delete; \
        DigitalPin(DigitalPin&&) = delete; \
        DigitalPin& operator=(const DigitalPin&) = delete; \
        DigitalPin& operator=(DigitalPin&&) = delete; \
        static constexpr auto isInputPin() noexcept { return true; } \
        static constexpr auto isOutputPin() noexcept { return false; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return INPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        [[gnu::always_inline]] inline static auto read() noexcept { return digitalRead<pin>(); } \
        [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
        static constexpr auto valid() noexcept { return isValidPin960_v<pin>; }              \
        [[gnu::always_inline]] static void configure() noexcept { pinMode(pin, INPUT); } \
    }
#define DefSPICSPin(pin) DefOutputPin(pin, LOW, HIGH)

DefSPICSPin(i960Pinout::GPIOSelect);
DefSPICSPin(i960Pinout::SD_EN);
DefSPICSPin(i960Pinout::PSRAM_EN);
#ifndef CHIPSET_TYPE1
DefSPICSPin(i960Pinout::PSRAM_EN1);
#ifdef CHIPSET_TYPE3
DefSPICSPin(i960Pinout::PSRAM_EN2);
DefSPICSPin(i960Pinout::PSRAM_EN3);
#endif
#endif

DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
#ifdef CHIPSET_TYPE3
DefOutputPin(i960Pinout::Reset4809, LOW, HIGH);
#endif
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
DefInputPin(i960Pinout::BE0, LOW, HIGH);
DefInputPin(i960Pinout::BE1, LOW, HIGH);
DefInputPin(i960Pinout::INT_EN0, LOW, HIGH);
DefInputPin(i960Pinout::INT_EN1, LOW, HIGH);
#ifndef CHIPSET_TYPE1
DefInputPin(i960Pinout::INT_EN2, LOW, HIGH);
DefInputPin(i960Pinout::INT_EN3, LOW, HIGH);
#endif
#undef DefSPICSPin
#undef DefInputPin
#undef DefOutputPin

template<typename ... Pins>
[[gnu::always_inline]]
inline void setupPins(decltype(OUTPUT) direction, Pins ... pins) noexcept {
    (pinMode(pins, direction), ...);
}

template<typename ... Pins>
[[gnu::always_inline]]
inline void digitalWriteBlock(decltype(HIGH) value, Pins ... pins) noexcept {
    (digitalWrite(pins, value), ...);
}

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};

#endif //ARDUINO_PINOUT_H
