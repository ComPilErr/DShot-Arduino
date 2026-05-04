#include "Arduino.h"

#ifndef DShot_h
#define DShot_h

#if !defined(STM32F1xx) && !defined(ARDUINO_ARCH_STM32)
  #error "This library targets STM32 (BluePill / STM32F103). Use DShot-Arduino for AVR boards."
#endif

/*
 * Supported motor pins (must use one of these):
 *   PA0  — TIM2_CH1
 *   PA6  — TIM3_CH1
 *   PB6  — TIM4_CH1
 *
 * Each motor must be on a different pin from the list above.
 * Multiple motors on the same timer are not supported.
 */

class DShot {
  public:
    enum Mode {
      DSHOT600,
      DSHOT300,
      DSHOT150
    };

    DShot(const enum Mode mode);
    void attach(uint8_t pin);
    uint16_t setThrottle(uint16_t throttle);

  private:
    uint16_t _packet   = 0;
    uint16_t _throttle = 0;
    uint8_t  _motorIdx = 0;
};

#endif
