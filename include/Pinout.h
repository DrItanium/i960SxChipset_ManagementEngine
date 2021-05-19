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
#include <libbonuspin.h>
using Address = uint32_t;
/// @todo fix this pinout for different targets
enum class i960Pinout : decltype(A0) {
#ifdef ARDUINO_AVR_ATmega1284
        // PORT B
        Led = 0,      // output
        CLOCK_OUT, // output, unusable
        AS_,     // input, AVR Int2
        PWM4, // unused
        GPIOSelect,        // output
        MOSI,          // reserved
        MISO,          // reserved
        SCK,          // reserved
// PORT D
        RX0,          // reserved
        TX0,          // reserved
        DEN_,      // AVR Interrupt INT0
        AVR_INT1,        // AVR Interrupt INT1
        SPI_BUS_EN, // output
        DC,     // output
        DISPLAY_EN, // output
        SD_EN,      // output
// PORT C
        SCL,          // reserved
        SDA,          // reserved
        Ready,      // output
        Int0_,          // output
        W_R_,          // input
        Reset960,          // output
        BLAST_,     // input
        FAIL,         // input
// PORT A, used to select the spi bus address (not directly used)
        SPI_BUS_A0,
        SPI_BUS_A1,
        SPI_BUS_A2,
        SPI_BUS_A3,
        SPI_BUS_A4,
        SPI_BUS_A5,
        SPI_BUS_A6,
        SPI_BUS_A7,
        Count,          // special
#elif defined(ARDUINO_NRF52_ADAFRUIT) // end defined(ARDUINO_AVR_ATmega1284)
    RedLed = LED_RED,
    BlueLed = LED_BLUE,
    Led = RedLed,
    MOSI = ::MOSI,          // reserved
    MISO = ::MISO,          // reserved
    SCK = ::SCK,          // reserved
    SDA = PIN_WIRE_SDA,          // reserved
    SCL = PIN_WIRE_SCL,          // reserved
    Reset960 = ::A1,          // output
    SPI_BUS_EN = ::A2, // output
    Int0_ = ::A3,          // output
    DEN_ = ::A4,
    W_R_ = ::A5,          // input
    GPIOSelect = 27,        // output
    Ready = 30,      // output
    FAIL = 7,         // input
    SD_EN = 11,      // output
    AS_ = 15,
    BLAST_ = 16,     // input
    DISPLAY_EN = SDA, // done over i2c
    DC = SDA, // done over i2c
    Count,
#elif defined(ARDUINO_GRAND_CENTRAL_M4)// end defined(NRF52832_XXAA)
        Led = LED_BUILTIN,      // output
        GPIOSelect = ::SS,        // output
        MOSI = ::MOSI,          // reserved
        MISO = ::MISO,          // reserved
        SCK = ::SCK,          // reserved
        DC = 8,     // output
        DISPLAY_EN = 10, // output
        AS_ = 22,     // input, AVR Int2
        Int0_ = 23,          // output
        DEN_ = 24,      // AVR Interrupt INT0
        BLAST_ = 25,     // input
        W_R_ = 26,          // input
        Reset960 = 27,          // output
        Ready = 28,      // output
        FAIL = 29,         // input
        SD_EN = SDCARD_SS_PIN,
        SCL = ::SCL,          // reserved
        SDA = ::SDA,          // reserved
        // for now, it is [30, 38]
        SPI_BUS_EN = 30, // output
        Count,          // special
#elif defined(ADAFRUIT_FEATHER_M0_BASIC)
#elif defined(ADAFRUIT_FEATHER_M0_ADALOGGER)
    RedLed = LED_RED,
    BlueLed = LED_BLUE,
    Led = RedLed,
    MOSI = ::MOSI,          // reserved
    MISO = ::MISO,          // reserved
    SCK = ::SCK,          // reserved
    SDA = PIN_WIRE_SDA,          // reserved
    SCL = PIN_WIRE_SCL,          // reserved
    Reset960 = ::A1,          // output
    SPI_BUS_EN = ::A2, // output
    Int0_ = ::A3,          // output
    DEN_ = ::A4,
    W_R_ = ::A5,          // input
    GPIOSelect = 27,        // output
    Ready = 30,      // output
    FAIL = 7,         // input
    SD_EN = 11,      // output
    AS_ = 15,
    BLAST_ = 16,     // input
    DISPLAY_EN = SDA, // done over i2c
    DC = SDA, // done over i2c
    Count,

#else
#endif
};

/**
 * @brief If both the display en and dc pins are bound to sda or scl then communication is taking place over i2c
 */
constexpr auto talksToDisplayOverI2C() noexcept {
    return (i960Pinout::DISPLAY_EN == i960Pinout::SDA || i960Pinout::DISPLAY_EN == i960Pinout::SCL) &&
            (i960Pinout::DC == i960Pinout::SDA || i960Pinout::DC == i960Pinout::SCL );
}

/**
 * @brief If both the display and dc pins are bound to non i2c pins then assume it is talking over spi
 */
constexpr auto talksToDisplayOverSPI() noexcept {
    return (i960Pinout::DISPLAY_EN != i960Pinout::SDA) &&
           (i960Pinout::DISPLAY_EN != i960Pinout::SCL) &&
           (i960Pinout::DC != i960Pinout::SDA) &&
           (i960Pinout::DC != i960Pinout::SCL);
}

inline void digitalWrite(i960Pinout ip, decltype(HIGH) value) {
    digitalWrite(static_cast<int>(ip), value);
}

inline void pinMode(i960Pinout ip, decltype(INPUT) value) {
    pinMode(static_cast<int>(ip), value);
}

inline auto digitalRead(i960Pinout ip) {
    return digitalRead(static_cast<int>(ip));
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
		static constexpr auto getDeassertionState() noexcept { return deasserted; } \
		inline static void assertPin() noexcept { digitalWrite(pin, getAssertionState()); } \
		inline static void deassertPin() noexcept { digitalWrite(pin, getDeassertionState()); } \
		inline static void write(decltype(LOW) value) noexcept { digitalWrite(pin, value); } \
		inline static void pulse() noexcept { \
			assertPin(); \
			deassertPin(); \
		} \
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
		inline static auto read() noexcept { return digitalRead(pin); } \
		inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
		inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
	}
#define DefInputPullupPin(pin, asserted, deasserted) \
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
		static constexpr auto getDirection() noexcept { return INPUT_PULLUP; } \
		static constexpr auto getAssertionState() noexcept { return asserted; } \
		static constexpr auto getDeassertionState() noexcept { return deasserted; } \
		inline static bool isAsserted() noexcept { return digitalRead(pin) == getAssertionState(); } \
		inline static bool isDeasserted() noexcept { return digitalRead(pin) == getDeassertionState(); } \
		inline static auto read() noexcept { return digitalRead(pin); } \
	}
DefOutputPin(i960Pinout::GPIOSelect, LOW, HIGH);
DefOutputPin(i960Pinout::Reset960, LOW, HIGH);
DefOutputPin(i960Pinout::Ready, LOW, HIGH);
DefOutputPin(i960Pinout::SPI_BUS_EN, LOW, HIGH);
DefOutputPin(i960Pinout::DISPLAY_EN, LOW, HIGH);
DefOutputPin(i960Pinout::SD_EN, LOW, HIGH);
DefInputPin(i960Pinout::FAIL, HIGH, LOW);
DefInputPin(i960Pinout::DEN_, LOW, HIGH);
DefInputPin(i960Pinout::AS_, LOW, HIGH);
DefInputPin(i960Pinout::BLAST_, LOW, HIGH);
DefInputPin(i960Pinout::W_R_, LOW, HIGH);
#undef DefInputPin
#undef DefOutputPin
#undef DefInputPullupPin

template<typename ... Pins>
void setupPins(decltype(OUTPUT) direction, Pins ... pins) {
    (pinMode(pins, direction), ...);
}

template<typename ... Pins>
void digitalWriteBlock(decltype(HIGH) value, Pins ... pins) {
    (digitalWrite(pins, value), ...);
}

template<i960Pinout pinId, decltype(HIGH) onConstruction, decltype(LOW) onDestruction>
class PinToggler {
public:
    PinToggler() { digitalWrite(pinId, onConstruction); }
    ~PinToggler() { digitalWrite(pinId, onDestruction); }
};

template<i960Pinout pinId>
using HoldPinLow = PinToggler<pinId, LOW, HIGH>;

template<i960Pinout pinId>
using HoldPinHigh = PinToggler<pinId, HIGH, LOW>;

template<i960Pinout pinId>
class PinAsserter {
public:
    static_assert(DigitalPin<pinId>::isOutputPin());
    PinAsserter() { DigitalPin<pinId>::assertPin(); }
    ~PinAsserter() { DigitalPin<pinId>::deassertPin(); }
};

#endif //ARDUINO_PINOUT_H
