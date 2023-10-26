//ptit code cute fait par CAD pour tester les capteurs de distance, fonctionne avec butché

#include <Arduino.h>
#include <LibRobus.h>

void setup() {
  BoardInit();
}

void loop() {
  uint16_t distance0 = ROBUS_ReadIR(0);
  uint16_t distance1= ROBUS_ReadIR(1);
  
  uint16_t past_distance0 = 0;
  uint16_t past_distance1 = 0;
  
  for (int x = 0; x < 10000; x++) {
    distance0 = ROBUS_ReadIR(0);
    distance1 = ROBUS_ReadIR(1);

    if (distance0 != past_distance0) {
      Serial.print("Sensor 0: ");
      Serial.println(distance0);
      past_distance0 = distance0;
    }

    if (distance1 != past_distance1) {
      Serial.print("Sensor 1: ");
      Serial.println(distance1);
      past_distance1 = distance1;
    }
    
    delay(1000);
  }
}