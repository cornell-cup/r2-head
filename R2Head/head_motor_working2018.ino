#include <stdint.h>
#include <Servo.h>
//#include <Encoder.h>
//#include "math.h"

#define PIN_HEAD 5
Servo head;

void setup() {
  head.attach(PIN_HEAD);
  Serial.begin(9600);
}

void loop() {

  int pos = 0;
  for(pos=0; pos<180; pos +=1){
    head.write(pos);
    delay(15); // controls how fast
  }
  for(pos=180; pos>=1; pos -=1){
    head.write(pos);
    delay(15);
  }


}
