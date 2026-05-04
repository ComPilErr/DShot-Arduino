#include <DShot.h>

/*
 * DShotLibraryTest for STM32 BluePill (STM32F103C8T6)
 *
 * Send throttle commands to a DShot ESC over Serial.
 * The ESC signal pin is PA0 (Arduino pin 0 on generic_stm32f103c8 variant).
 * All pins used for DShot must share the same GPIO port (e.g. all on GPIOA).
 *
 * Wiring:
 *   BluePill PA0  -->  ESC signal wire
 *   BluePill GND  -->  ESC GND   (common ground required)
 *
 * Serial monitor: 115200 baud
 *   Type a number (48–2047) and press Enter to set the target throttle.
 *   The sketch ramps the throttle toward the target at 1 step per 10 ms.
 */

DShot esc1(DShot::Mode::DSHOT600);

uint16_t throttle = 0;
uint16_t target   = 0;

void setup() {
    Serial.begin(115200);

    // PA0 on BluePill — all DShot pins must be on the same GPIO port.
    esc1.attach(PA0);
    esc1.setThrottle(throttle);
}

void loop() {
    if (Serial.available() > 0) {
        target = Serial.parseInt();
        if (target > 2047)
            target = 2047;
        Serial.print(target, HEX);
        Serial.print("\t");
    }

    if (throttle < 48) {
        throttle = 48;
    }

    if (target <= 48) {
        esc1.setThrottle(target);
    } else {
        if (target > throttle) {
            throttle++;
            esc1.setThrottle(throttle);
        } else if (target < throttle) {
            throttle--;
            esc1.setThrottle(throttle);
        }
    }
    delay(10);
}
