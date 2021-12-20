// Target Chip: Atmega 4809 (DIP40)
// Target Platform: MegaCoreX
// Clockspeed: 20MHz internal oscillator
// PF6 is reset
// No bootloader
// UPDI is connected to Serial1's alternate pinout
// this sketch just holds the i960 in reset, sets up the aux pins and then releases the reset pin 2 seconds after finishing everything else.
// the serial console is totally optional and could be easily removed if needed. 
//
// The objective of this program is to eliminate pins
constexpr auto EnableExternalClockSource = true;
constexpr auto EnableClockOutputPort = false;
constexpr auto Reset960 = PIN_PF0;
constexpr auto LOCK960 = PIN_PF1;
constexpr auto HLDA960 = PIN_PF2;
constexpr auto HOLD960 = PIN_PF3;
constexpr auto WAITBOOT960 = PIN_PF4;
constexpr auto FAIL960 = PIN_PF5;

constexpr auto DO_CYCLE = PIN_PC0;
constexpr auto TRANSACTION_START = PIN_PC1;
constexpr auto BURST_NEXT = PIN_PC2;
constexpr auto TRANSACTION_END = PIN_PC3;

constexpr auto MCU_READY = PIN_PD2;
constexpr auto READY960 = PIN_PD3;
constexpr auto BLAST = PIN_PD4;
constexpr auto AS = PIN_PD5;
constexpr auto DEN = PIN_PD6;
constexpr auto SYSTEMBOOT = PIN_PD7;
// this pin is responsible for allowing the 
constexpr auto INT0_960 = PIN_PE0;
constexpr auto INT1_960 = PIN_PE1;
constexpr auto INT2_960 = PIN_PE2;
constexpr auto INT3_960 = PIN_PE3;


template<decltype(WAITBOOT960) pin>
struct DigitalPin {
    DigitalPin() = delete;
    ~DigitalPin() = delete;
    DigitalPin(const DigitalPin&) = delete;
    DigitalPin(DigitalPin&&) = delete;
    DigitalPin& operator=(const DigitalPin&) = delete;
    DigitalPin& operator=(DigitalPin&&) = delete;
    static constexpr bool isInputPin() noexcept { return false; }
    static constexpr bool isOutputPin() noexcept { return false; }
    static constexpr auto isInputPullupPin() noexcept { return false; }
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
        static constexpr auto isInputPullupPin() noexcept { return false; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return OUTPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; }      \
        [[gnu::always_inline]] inline static void assertPin() noexcept { digitalWriteFast(pin, getAssertionState()); } \
        [[gnu::always_inline]] inline static void deassertPin() noexcept { digitalWriteFast(pin,getDeassertionState()); } \
        [[gnu::always_inline]] inline static void write(decltype(LOW) value) noexcept { digitalWriteFast(pin,value); } \
        [[gnu::always_inline]] inline static void pulse() noexcept {  assertPin(); __builtin_avr_nops(2); deassertPin(); } \
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
        static constexpr auto isInputPullupPin() noexcept { return false; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return INPUT; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        [[gnu::always_inline]] inline static auto read() noexcept { return digitalReadFast(pin); } \
        [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
        [[gnu::always_inline]] static void configure() noexcept { pinMode(pin, INPUT); }     \
        [[gnu::always_inline]] inline static auto isLow() noexcept { return read() == LOW; } \
        [[gnu::always_inline]] inline static auto isHigh() noexcept { return read() == HIGH; } \
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
        static constexpr auto isOutputPin() noexcept { return false; }  \
        static constexpr auto isInputPullupPin() noexcept { return true; } \
        static constexpr auto getPin() noexcept { return pin; } \
        static constexpr auto getDirection() noexcept { return INPUT_PULLUP; } \
        static constexpr auto getAssertionState() noexcept { return asserted; } \
        static constexpr auto getDeassertionState() noexcept { return deasserted; } \
        [[gnu::always_inline]] inline static auto read() noexcept { return digitalReadFast(pin); } \
        [[gnu::always_inline]] inline static bool isAsserted() noexcept { return read() == getAssertionState(); } \
        [[gnu::always_inline]] inline static bool isDeasserted() noexcept { return read() == getDeassertionState(); } \
        [[gnu::always_inline]] static void configure() noexcept { pinMode(pin, INPUT_PULLUP); } \
        [[gnu::always_inline]] inline static auto isLow() noexcept { return read() == LOW; } \
        [[gnu::always_inline]] inline static auto isHigh() noexcept { return read() == HIGH; } \
    }

DefInputPin(DEN, LOW, HIGH);
DefInputPin(AS, LOW, HIGH);
DefInputPin(BLAST, LOW, HIGH);
DefInputPin(MCU_READY, LOW, HIGH);
DefInputPin(FAIL960, HIGH, LOW);
DefInputPin(HLDA960, HIGH, LOW);

DefOutputPin(Reset960, LOW, HIGH);
DefOutputPin(READY960, LOW, HIGH);
DefOutputPin(LOCK960, LOW, HIGH);
DefOutputPin(HOLD960, HIGH, LOW);

DefOutputPin(INT0_960, LOW, HIGH);
DefOutputPin(INT1_960, HIGH, LOW);
DefOutputPin(INT2_960, HIGH, LOW);
DefOutputPin(INT3_960, LOW, HIGH);
DefOutputPin(SYSTEMBOOT, HIGH, LOW);
DefOutputPin(TRANSACTION_START, LOW, HIGH);
DefOutputPin(TRANSACTION_END, LOW, HIGH);
DefOutputPin(DO_CYCLE, LOW, HIGH);
DefOutputPin(BURST_NEXT, LOW, HIGH);

DefInputPullupPin(WAITBOOT960, LOW, HIGH);
#undef DefInputPin
#undef DefOutputPin
#undef DefInputPullupPin

template<decltype(WAITBOOT960) pin>
struct PinAsserter final {
    static_assert(DigitalPin<pin>::isOutputPin(), "Pin asserting only works with output pins!");
    PinAsserter() noexcept { DigitalPin<pin>::assertPin(); }
    ~PinAsserter() noexcept { DigitalPin<pin>::deassertPin(); }
};

[[noreturn]] void
handleChecksumFail() noexcept {
    // keep an eye on the FAIL960 pin, if we run into an issue then tell the chipset this
    DigitalPin<SYSTEMBOOT>::deassertPin();
    while (true) {
        delay(1000);
    }
}

inline void
triggerInt0() noexcept {
    DigitalPin<INT0_960>::pulse();
}
inline void
triggerInt1() noexcept {
    DigitalPin<INT1_960>::pulse();
}
inline void
triggerInt2() noexcept {
    DigitalPin<INT2_960>::pulse();
}

inline void
triggerInt3() noexcept {
    DigitalPin<INT3_960>::pulse();
}

void configureClockSource() noexcept {
    byte clkBits = 0;
    if constexpr (EnableExternalClockSource) {
        clkBits |= 0b0000'0011;
    }
    if constexpr (EnableClockOutputPort) {
        clkBits |= 0b1000'0000;
    }
    // setup all of the pins that can affect the i960 itself
    if constexpr (EnableExternalClockSource || EnableClockOutputPort) {
        CCP = 0xD8;
        CLKCTRL.MCLKCTRLA = clkBits; // enable the external 20mhz clock source just to be on the safe side
        CCP = 0xD8;
    }
    if constexpr (EnableClockOutputPort) {
            CCP = 0xD8;
            CLKCTRL.OSC20MCTRLA |= 0b0000'0010;
            CCP = 0xD8;
    }
}
template<int ... pins>
void configurePins() noexcept {
    (DigitalPin<pins>::configure() , ...);
}
template<int ... pins>
void deassertPins() noexcept {
    (DigitalPin<pins>::deassertPin(), ...);
}
void setup() {
    configureClockSource();
    // always pull this low to start
    DigitalPin<Reset960>::configure();
    {
        PinAsserter<Reset960> holdI960InReset;
        DigitalPin<WAITBOOT960>::configure();
        delay(2000);
        //Serial1.swap(1);
        //Serial1.begin(9600);
        //Serial1.println("Bringing everything up!");
        configurePins<DEN,
                HLDA960,
                FAIL960,
                AS,
                BLAST,
                MCU_READY,
                READY960,
                LOCK960,
                HOLD960,
                INT0_960,
                INT1_960,
                INT2_960,
                INT3_960,
                SYSTEMBOOT,
                TRANSACTION_START,
                TRANSACTION_END,
                DO_CYCLE,
                BURST_NEXT>();

        deassertPins<BURST_NEXT,
                     DO_CYCLE,
                     TRANSACTION_START,
                     TRANSACTION_END,
                     LOCK960,
                     HOLD960,
                     INT0_960,
                     INT1_960,
                     INT2_960,
                     INT3_960,
                     SYSTEMBOOT,
                     READY960>();
        // do not attach an interrupt to MCU_READY, it adds way too much latency on the 4809
        while (DigitalPin<WAITBOOT960>::isAsserted());
    }

    while (DigitalPin<FAIL960>::isLow()) {
        if (DigitalPin<DEN>::isAsserted()) {
            break;
        }
    }
    while (DigitalPin<FAIL960>::isHigh()) {
        if (DigitalPin<DEN>::isAsserted()) {
            break;
        }
    }
    DigitalPin<SYSTEMBOOT>::assertPin();
    //Serial1.println("SYSTEM BOOTED!");
    // after this point, if FAIL960 ever goes from LOW to HIGH again, then we have checksum failed!
    attachInterrupt(digitalPinToInterrupt(FAIL960), handleChecksumFail, RISING);
}

inline bool
informCPU() noexcept {
    // sample blast at this point, I can guarantee it accurate for this cycle
    bool isBurstLast = DigitalPin<BLAST>::isAsserted();
    // pulse ready since it will automatically be synchronized by the external hardware
    DigitalPin<READY960>::pulse();
    return isBurstLast;
}
[[gnu::always_inline]]
inline void
transactionBody() noexcept {
    // okay so we need to wait for AS and DEN to go low
    while (DigitalPin<DEN>::isDeasserted());
    /// @todo look into triggering transaction start before checking to see if den was triggered, does it improve responsiveness?
    DigitalPin<TRANSACTION_START>::pulse(); // tell the chipset that it can safely pull down the base address of the transaction
    // okay now we need to emulate the wait loop
    do {
        DigitalPin<DO_CYCLE>::pulse(); // tell the chipset that it is safe to process this data cycle (regardless of where we are)
        // now wait for the chipset to tell us it has satisified the current part of the transaction
        while (DigitalPin<MCU_READY>::isDeasserted());
        if (informCPU()) {
            DigitalPin<TRANSACTION_END>::pulse(); // let the chipset know this is the end of the transaction
            break;
        } else {
            // if we got here then it is a burst transaction and as such
            // let the chipset know this is the next word of the burst transaction
            // this will act as a gate action
            DigitalPin<BURST_NEXT>::pulse();
        }
    } while (true);
    // now we just loop back around and wait for the next
}


void loop() {
    while (true) {
        transactionBody();
    }
}
