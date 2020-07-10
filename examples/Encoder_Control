#include <DShot.h>

#define ENC_PIN_A 2
#define ENC_PIN_B 3

#define MAX_SPEED 2000
#define MIN_SPEED 48
#define STEP 25

unsigned long previous_time = 0;   
unsigned long current_time = 0;
unsigned int encoder = 0;
const long interval = 1000; 

u8 val = 0;
u8 pre = 0;

DShot esc;

void action()
{
  val = (PIND & 12) >>2; 
  if (val!=pre){
  int out = val*10+pre; 
  if (encoder < (MIN_SPEED+STEP)) encoder = MIN_SPEED+STEP;
  if (encoder > (MAX_SPEED-STEP)) encoder = MAX_SPEED-STEP;  
  switch(out){
    case 31:    case 10:    case  2:    case 23:encoder+=STEP; break;
    case 32:    case 20:    case  1:    case 13:encoder-=STEP; break;
    }
  Serial.print(encoder);
  Serial.print("\n");
  pre = val;
  }
  }

void setup() {
esc.attach(7);  
Serial.begin(115200);
pinMode(ENC_PIN_B, INPUT);
pinMode(ENC_PIN_A, INPUT);
esc.setThrottle(0);
delay(1000*3);
}

void loop() {
    current_time = micros();
    
   if (abs(current_time - previous_time) >= interval) {
        previous_time = current_time;
        action();
        esc.setThrottle(encoder);
    }
}
