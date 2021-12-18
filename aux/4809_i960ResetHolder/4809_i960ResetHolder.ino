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
volatile bool denTriggered = false;
volatile bool asTriggered = false;
volatile bool readyTriggered = false;
void
handleASTrigger() noexcept {
    asTriggered = true;
}
void
handleDENTrigger() noexcept {
    denTriggered = true;
}

void
handleREADYTrigger() noexcept {
    readyTriggered = true;
}
[[noreturn]] void
handleChecksumFail() noexcept {
    // keep an eye on the FAIL960 pin, if we run into an issue then tell the chipset this
    digitalWriteFast(SYSTEMBOOT, LOW);
    while (true) {
        delay(1000);
    }
}
template<decltype(READY960) pin, decltype(LOW) to = LOW, decltype(HIGH) from = HIGH>
inline void
pulse() noexcept {
   digitalWriteFast(pin, to);
   digitalWriteFast(pin, from);
}
inline void
triggerInt0() noexcept {
    pulse<INT0_960, LOW, HIGH>();
}
inline void
triggerInt1() noexcept {
    pulse<INT1_960, HIGH, LOW>();
}
inline void
triggerInt2() noexcept {
    pulse<INT2_960, HIGH, LOW>();
}

inline void
triggerInt3() noexcept {
    pulse<INT3_960, LOW, HIGH>();
}

bool
informCPU() noexcept {
    // sample blast at this point, I can guarantee it accurate for this cycle
    auto isBurstLast = digitalReadFast(BLAST) == LOW;
    // pulse ready since it will automatically be synchronized by the external hardware
    pulse<READY960, LOW, HIGH>();
    return isBurstLast;
}
void setup() {
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
    // always pull this low to start
    pinMode(Reset960, OUTPUT);
    digitalWriteFast(Reset960, LOW);
    pinMode(WAITBOOT960, INPUT_PULLUP);
    delay(2000);
    Serial1.swap(1);
    Serial1.begin(9600);
    Serial1.println("Bringing everything up!");
    pinMode(HLDA960, INPUT);
    pinMode(DEN, INPUT);
    pinMode(FAIL960, INPUT);
    pinMode(AS, INPUT);
    pinMode(BLAST, INPUT);
    pinMode(MCU_READY, INPUT);

    pinMode(READY960, OUTPUT);
    pinMode(LOCK960, OUTPUT);
    pinMode(HOLD960, OUTPUT);
    pinMode(INT0_960, OUTPUT);
    pinMode(INT1_960, OUTPUT);
    pinMode(INT2_960, OUTPUT);
    pinMode(INT3_960, OUTPUT);
    pinMode(SYSTEMBOOT, OUTPUT);
    pinMode(TRANSACTION_START, OUTPUT);
    pinMode(DO_CYCLE, OUTPUT);
    pinMode(BURST_NEXT, OUTPUT);
    pinMode(TRANSACTION_END, OUTPUT);

    digitalWriteFast(BURST_NEXT, HIGH);
    digitalWriteFast(DO_CYCLE, HIGH);
    digitalWriteFast(TRANSACTION_START, HIGH);
    digitalWriteFast(TRANSACTION_END, HIGH);
    digitalWriteFast(LOCK960, HIGH);
    digitalWriteFast(HOLD960, LOW);
    digitalWriteFast(INT0_960, HIGH);
    digitalWriteFast(INT1_960, LOW);
    digitalWriteFast(INT2_960, LOW);
    digitalWriteFast(INT3_960, HIGH);
    digitalWriteFast(SYSTEMBOOT, LOW);
    digitalWriteFast(READY960, HIGH);
    // these interrupts are used by the boot process and such
    Serial1.println("Setting up Interrupts");
    attachInterrupt(digitalPinToInterrupt(DEN), handleDENTrigger, FALLING);
    attachInterrupt(digitalPinToInterrupt(AS), handleASTrigger, FALLING);
    attachInterrupt(digitalPinToInterrupt(MCU_READY), handleREADYTrigger, FALLING);
    Serial1.println("Waiting for the chipset to be ready");
    while (digitalReadFast(WAITBOOT960) == LOW);
    digitalWriteFast(Reset960, HIGH);

    while (digitalReadFast(FAIL960) == LOW) {
        if (asTriggered && denTriggered) {
            break;
        }
    }
    while (digitalReadFast(FAIL960) == HIGH) {
        if (asTriggered && denTriggered) {
            break;
        }
    }
    digitalWriteFast(SYSTEMBOOT, HIGH);
    Serial1.println("SYSTEM BOOTED!");
    // after this point, if FAIL960 ever goes from LOW to HIGH again, then we have checksum failed!
    attachInterrupt(digitalPinToInterrupt(FAIL960), handleChecksumFail, RISING);
}


void loop() {
    // okay so we need to wait for AS and DEN to go low
    while (!asTriggered);
    asTriggered = false;
    while (!denTriggered);
    denTriggered = false;
    /// @todo look into triggering transaction start before checking to see if den was triggered, does it improve responsiveness?
    pulse<TRANSACTION_START, LOW, HIGH>(); // tell the chipset that it can safely pull down the base address of the transaction
    // okay now we need to emulate the wait loop
    do {
        pulse<DO_CYCLE, LOW, HIGH>(); // tell the chipset that it is safe to process this data cycle (regardless of where we are)
        // now wait for the chipset to tell us it has satisified the current part of the transaction
        while (!readyTriggered);
        readyTriggered = false;
        if (informCPU()) {
            pulse<TRANSACTION_END, LOW, HIGH>(); // let the chipset know this is the end of the transaction
            break; // leave the loop!
        } else {
            // if we got here then it is a burst transaction and as such
            // let the chipset know this is the next word of the burst transaction
            // this will act as a gate action
            pulse<BURST_NEXT, LOW, HIGH>(); // tell the chipset that we are continuing this burst transaction
        }
        // then we start this again until we hit burst last
    } while (true);
    // now we just loop back around and wait for the next
}
