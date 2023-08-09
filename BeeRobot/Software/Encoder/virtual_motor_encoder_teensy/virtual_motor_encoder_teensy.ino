#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(22, 23);

uint8_t *data2  = (uint8_t *) malloc(128);

void setup() {
  Serial3.begin(115200);
  
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
  if (Serial3.available()) {
    Serial3.readBytes(data2,1);
    if(data2[0] == 1)
        Serial3.write(data,3);
    if(data2[0] == 3) 
        knobLeft.write(0);   
 }
  
  free(data);
}
