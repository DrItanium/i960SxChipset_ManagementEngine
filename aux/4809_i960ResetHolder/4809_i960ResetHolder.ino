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
volatile bool blastTriggered = false;
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
handleBLASTTrigger() noexcept {
    blastTriggered = true;
}
void
handleReadyTrigger() noexcept {
    readyTriggered = true;
}
[[noreturn]] void
handleChecksumFail() noexcept {
    // keep an eye on the FAIL960 pin, if we run into an issue then tell the chipset this
    digitalWrite(SYSTEMBOOT, LOW);
    while (true) {
        delay(1000);
    }
}
template<decltype(READY960) pin, decltype(LOW) to = LOW, decltype(HIGH) from = HIGH>
inline void
pulse() noexcept {
   digitalWrite(pin, to);
   digitalWrite(pin, from);
}
void
informCPU() noexcept {
    // pulse it since it will automatically be synchronized and detected
    pulse<READY960, LOW, HIGH>();
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
    digitalWrite(Reset960, LOW);
    pinMode(WAITBOOT960, INPUT_PULLUP);
    delay(2000);
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

    digitalWrite(LOCK960, HIGH);
    digitalWrite(HOLD960, LOW);
    digitalWrite(INT0_960, HIGH);
    digitalWrite(INT1_960, LOW);
    digitalWrite(INT2_960, LOW);
    digitalWrite(INT3_960, HIGH);
    digitalWrite(SYSTEMBOOT, LOW);
    digitalWrite(READY960, HIGH);
    // these interrupts are used by the boot process and such
    attachInterrupt(digitalPinToInterrupt(DEN), handleDENTrigger, FALLING);
    attachInterrupt(digitalPinToInterrupt(AS), handleASTrigger, FALLING);
    attachInterrupt(digitalPinToInterrupt(BLAST), handleBLASTTrigger, FALLING);
    attachInterrupt(digitalPinToInterrupt(MCU_READY), handleREADYTrigger, FALLING);
    while (digitalRead(WAITBOOT960) == LOW);
    digitalWrite(Reset960, HIGH);

    while (digitalRead(FAIL960) == LOW) {
        if (asTriggered && denTriggered) {
            break;
        }
    }
    while (digitalRead(FAIL960) == HIGH) {
        if (asTriggered && denTriggered) {
            break;
        }
    }
    digitalWrite(SYSTEMBOOT, HIGH);
    // after this point, if FAIL960 ever goes from LOW to HIGH again, then we have checksum failed!
    attachInterrupt(digitalPinToInterrupt(FAIL960), handleChecksumFail, RISING)
}


void loop() {
    // okay so we need to wait for AS and DEN to go low
    while (!asTriggered);
    asTriggered = false;
    while (!denTriggered);
    denTriggered = false;
    // this would tell the chipset we have a new transaction, do a pulse for this
    // okay now we need to emulate the wait loop
    do {

    } while (true);
}
