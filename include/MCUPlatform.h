// contains routines for handling microcontroller platform detection
// Created by jwscoggins on 5/18/21.
//

#ifndef I960SXCHIPSET_MCUPLATFORM_H
#define I960SXCHIPSET_MCUPLATFORM_H
#ifdef ARDUINO_SAMD_FEATHER_M0
#define ADAFRUIT_FEATHER_M0
#ifdef HAS_BUILTIN_SDCARD
#define ADAFRUIT_FEATHER_M0_ADALOGGER
#else /* !defined(HAS_BUILTIN_SDCARD) */
#define ADAFRUIT_FEATHER_M0_BASIC
#endif
#endif
enum class TargetMCU {
    ATmega1284p,
    GrandCentralM4,
    FeatherNRF52830,
    FeatherM0Basic,
    FeatherM0Adalogger,
    Unknown,
};
/**
 * @brief Is there an onboard sdcard slot?
 * @return True if defined via the command line
 */
[[nodiscard]] constexpr auto hasBuiltinSDCard() noexcept {
#ifdef HAS_BUILTIN_SDCARD
    return true;
#else
    return false;
#endif
}
[[nodiscard]] constexpr auto getMCUTarget() noexcept {
#ifdef ARDUINO_AVR_ATmega1284
    return TargetMCU::ATmega1284p;
#elif defined(ARDUINO_GRAND_CENTRAL_M4)
    return TargetMCU::GrandCentralM4;
#elif defined(ARDUINO_NRF52_ADAFRUIT)
    return TargetMCU::FeatherNRF52830;
    // @todo add more targets here
#elif defined(ADAFRUIT_FEATHER_M0_ADALOGGER)
    return TargetMCU::FeatherM0Adalogger;
#elif defined(ADAFRUIT_FEATHER_M0_BASIC)
    return TargetMCU::FeatherM0Basic;
#else
    return TargetMCU::Unknown;
#endif
}
[[nodiscard]] constexpr auto onAtmega1284p() noexcept {
    return getMCUTarget() == TargetMCU::ATmega1284p;
}
[[nodiscard]] constexpr auto onGrandCentralM4() noexcept {
    return getMCUTarget() == TargetMCU::GrandCentralM4;
}

[[nodiscard]] constexpr auto onFeatherNRF52830() noexcept {
    return getMCUTarget() == TargetMCU::FeatherNRF52830;
}

[[nodiscard]] constexpr auto onUnknownTarget() noexcept {
    return getMCUTarget() == TargetMCU::Unknown;
}

[[nodiscard]] constexpr auto onFeatherM0Basic() noexcept {
    return getMCUTarget() == TargetMCU::FeatherM0Basic;
}

[[nodiscard]] constexpr auto onFeatherM0Adalogger() noexcept {
    return getMCUTarget() == TargetMCU::FeatherM0Adalogger;
}

[[nodiscard]] constexpr auto onFeatherM0() noexcept {
    return onFeatherM0Basic() ||
           onFeatherM0Adalogger();
}

[[nodiscard]] constexpr auto getCPUFrequency() noexcept {
    return F_CPU;
}

[[nodiscard]] constexpr auto onFeatherBoard() noexcept {
    return onFeatherM0() ||
           onFeatherNRF52830();
}

#endif //I960SXCHIPSET_MCUPLATFORM_H