#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(2, 3);


void setup() {
  Serial.begin(115200);
  
}

void loop() {
  short newLeft;
  newLeft = knobLeft.read();
  uint8_t *data = (uint8_t *) malloc(128);
  if (newLeft < 0)
  { 
    data[0]= 0;
    newLeft = abs(newLeft);
  } 
  else  
    data[0]= 1;
  data[1]= newLeft & 0b11111111;
  data[2]= newLeft >>8;
 // Serial.println(newLeft);
  if(newLeft >= 2000 || newLeft <= -2000)
    knobLeft.write(0);
  if (Serial.available()) {
    if(Serial.read())
        Serial.write(data,3);
 }
  
  free(data);
}
