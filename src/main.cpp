#include <Arduino.h>
#include <Servo.h>

/*

redefine DSHOT_PORT if you want to change the default PORT

Defaults
UNO: PORTD, available pins 0-7 (D0-D7)
Leonardo: PORTB, available pins 4-7 (D8-D11)

e.g.
#define DSHOT_PORT PORTD
*/
Servo myESC;
volatile uint16_t target;

void setup() {
  // Notice, all pins must be connected to same PORT
  target = 0;
  Serial.begin(115200);
  myESC.attach(PD7); 
  myESC.writeMicroseconds(1000);
}

void loop() {
  if (Serial.available()>0){
    target = Serial.parseInt();
    if (target>2020) target = 2020;
    Serial.print(target, DEC); Serial.print("\n");
    myESC.writeMicroseconds(target);
  }
  delay(10);
}
