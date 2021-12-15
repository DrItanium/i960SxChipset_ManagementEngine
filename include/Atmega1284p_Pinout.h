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

#ifndef SXCHIPSET_ATMEGA1284P_PINOUT_H
#define SXCHIPSET_ATMEGA1284P_PINOUT_H
#if defined(ARDUINO_AVR_ATmega1284)
#include "Pinout.h"
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] inline volatile unsigned char& getAssociatedOutputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return PORTA;
        Y(C): return PORTC;
        Y(D): return PORTD;
        Y(B): return PORTB;
#undef Y
#undef X

        default:
            return PORTA;
    }
}

template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] inline volatile unsigned char& getAssociatedDirectionPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return DDRA;
        Y(C): return DDRC;
        Y(D): return DDRD;
        Y(B): return DDRB;
#undef Y
#undef X

        default:
            return DDRA;
    }
}

template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] inline volatile unsigned char& getAssociatedInputPort() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number
#define Y(id) \
    X(id, 0): \
    X(id, 1): \
    X(id, 2): \
    X(id, 3): \
    X(id, 4): \
    X(id, 5): \
    X(id, 6): \
    X(id, 7)
        Y(A): return PINA;
        Y(C): return PINC;
        Y(D): return PIND;
        Y(B): return PINB;
#undef Y
#undef X
        default:
            return PINA;
    }
}
template<i960Pinout pin>
[[gnu::always_inline]]
[[nodiscard]] constexpr decltype(auto) getPinMask() noexcept {
    static_assert(isValidPin960_v<pin>, "INVALID PIN PROVIDED");
    switch (pin) {
#define X(id, number) case i960Pinout:: PORT_ ## id ## number : return _BV ( P ## id ## number )
#define Y(id) \
    X(id, 0); \
    X(id, 1); \
    X(id, 2); \
    X(id, 3); \
    X(id, 4); \
    X(id, 5); \
    X(id, 6); \
    X(id, 7)
        Y(A);
        Y(C);
        Y(D);
        Y(B);
#undef Y
#undef X
        default:
            return 0xFF;
    }
}

class InterruptDisabler final {
public:
    InterruptDisabler() noexcept {
        storage_ = SREG;
        cli();
    }
    ~InterruptDisabler() noexcept {
        SREG = storage_;
    }
private:
    uint8_t storage_ = 0;
};


template<i960Pinout pin, decltype(HIGH) value>
[[gnu::always_inline]]
inline void digitalWrite() noexcept {
    if constexpr (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}
template<i960Pinout pin>
[[gnu::always_inline]]
inline void digitalWrite(decltype(HIGH) value) noexcept {
    // don't disable interrupts, that should be done outside this method
    if (auto& thePort = getAssociatedOutputPort<pin>(); value == LOW) {
        thePort &= ~getPinMask<pin>();
    } else {
        thePort |= getPinMask<pin>();
    }
}
#endif
#endif //SXCHIPSET_ATMEGA1284P_PINOUT_H
