#include <Arduino.h>
#include <DShot.h>
#include <RH_ASK.h>

#define ESC_PIN PD7
#define RADIO_RX_PIN PD2
#define RADIO_TX_PIN PD3
#define RADIO_PTT_PIN PD4
#define SPEED_LIMIT 666

/*

redefine DSHOT_PORT if you want to change the default PORT

Defaults
UNO: PORTD, available pins 0-7 (D0-D7)
Leonardo: PORTB, available pins 4-7 (D8-D11)

e.g.
#define DSHOT_PORT PORTD
*/
DShot esc1(DShot::Mode::DSHOT300);
RH_ASK driver(2000, RADIO_RX_PIN, RADIO_TX_PIN, RADIO_PTT_PIN);

struct __attribute__((__packed__)) ControlPacket {
 uint32_t remote_id;
 uint16_t action;
 int8_t steering;
};
const uint32_t ALLOWED_ID = 0xDEADBEEF;

volatile uint16_t target = 0;
volatile uint16_t prev_target = 0;

volatile uint32_t timer = 0;
volatile uint32_t current_timer = 0;

void setup() {
  Serial.begin(115200);
  timer = millis();
  current_timer = timer;

  if (!driver.init())
  { Serial.println("Oops.. RadioHead initialization error :O"); }
  else
  { Serial.println("RadioHead initialization has been complete successfully"); }

  // Notice, all pins must be connected to same PORT
  esc1.attach(ESC_PIN);  
  esc1.setThrottle(0);
}

void loop() {
  ControlPacket incomingPacket;
  uint8_t buflen = sizeof(incomingPacket);
  current_timer = millis();

  if ((abs(timer - current_timer) > 200) && (prev_target != 0))
  {esc1.setThrottle(0);
    prev_target = 0; 
  Serial.print("Motors has been Disabled");}

  if (driver.recv((uint8_t *)&incomingPacket, &buflen))
  {
    if (incomingPacket.remote_id == ALLOWED_ID){
      timer = millis();
      Serial.print("Command received, action: ");
      Serial.print(incomingPacket.action);
      Serial.print(" | steering: ");
      Serial.println(incomingPacket.steering);
      
      target = (uint16_t)incomingPacket.action;

      //if (target >= SPEED_LIMIT)  target = SPEED_LIMIT;
      Serial.print("Target: ");
      Serial.println(target, DEC);
      if (prev_target != target){ esc1.setThrottle(target); prev_target = target;}
      //esc1.setThrottle(target);

    }
    else 
    { Serial.print("Someone else on the line, ID has not been recognized: ");
           Serial.println(incomingPacket.remote_id); }
  }
  
}
