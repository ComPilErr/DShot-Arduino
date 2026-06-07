#include <Arduino.h>
#include <DShot.h>
#include <RH_ASK.h>

#define ESC_PIN PD7
#define LED_PIN 13
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

volatile bool dark = true;
volatile uint8_t update_made = 0;

ISR ( TIMER3_COMPA_vect )
{
  dark = !dark;
  digitalWrite(LED, dark);
  update_made++;
  if ((update_made >= 3) && (prev_target !=0))
  { esc1.setThrottle(0);
    prev_target = 0;
    Serial.print("Motors have been Disabled");}
}

void setup() {
  Serial.begin(115200);

  if (!driver.init())
  { Serial.println("Oops.. RadioHead initialization error :O"); }
  else
  { Serial.println("RadioHead initialization has been complete successfully"); }

  // Notice, all pins must be connected to same PORT
  esc1.attach(ESC_PIN);  
  esc1.setThrottle(0);
  pinMode(LED_PIN, OUTPUT);

  cli();
  TCCR3A=0; // нормальный режим работы таймера
  TCCR3B=0;
  OCR3A=0x1869; // it's 100 ms measure
  TCCR3B = 1<<CS32|0<<CS31|0<<CS30|0<<WGM33|1<<WGM32; // режим сравнения, делитель 256
  TIMSK3 = 0<<ICIE3|0<<OCIE3B|1<<OCIE3A|0<<TOIE3; // разрешение прерываний по сравнению
  TCNT3=0;
  //TIMSK3 &= ~(1<<OCIE3A); // turn off the timer
  TIMSK3 |= (1<<OCIE3A); // // turn on the timer
  sei();
}

void loop() {
  ControlPacket incomingPacket;
  uint8_t buflen = sizeof(incomingPacket);

  if (driver.recv((uint8_t *)&incomingPacket, &buflen))
  {
    if (incomingPacket.remote_id == ALLOWED_ID){
      update_made = 0;
      Serial.print("Command received, action: ");
      Serial.print(incomingPacket.action);
      Serial.print(" | steering: ");
      Serial.println(incomingPacket.steering);
      
      target = (uint16_t)incomingPacket.action;

      //if (target >= SPEED_LIMIT)  target = SPEED_LIMIT;
      Serial.print("Target: ");
      Serial.println(target, DEC);
      if (prev_target != target){ esc1.setThrottle(target); prev_target = target;}
    }
    else 
    { Serial.print("Someone else on the line, ID has not been recognized: ");
           Serial.println(incomingPacket.remote_id); }
  }
  
}
