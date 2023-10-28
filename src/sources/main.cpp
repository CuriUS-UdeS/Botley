#include <Arduino.h>
#include <librobus.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

uint16_t r, g, b, c;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
    BoardInit();
    Serial.begin(9600);
    Serial.println("Color View Test!");

    if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
    }
}


void loop(void) {

    r = tcs.read16(TCS34725_RDATAL);
    g = tcs.read16(TCS34725_GDATAL);
    b = tcs.read16(TCS34725_BDATAL);
    c = tcs.read16(TCS34725_CDATAL);

    tcs.getRawData(&r, &g, &b, &c);

    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    Serial.println(" ");
    
  
       // Add a delay to control the update rate (in milliseconds)
    delay(100);

 //These values work when strapped directly to the motor'
  
  if (r > 90 && r < 155 && g > 60 && g < 100 && b > 50 && b < 100 && c > 250 && c < 360){
    Serial.println("Red");
  }
  else if (r > 200 && r < 330 && g > 200 && g < 300 && b > 100 && b < 200 && c > 500 && c < 900)
  {
    Serial.println("Yellow");
  }
   
  else if (r > 45 && r < 70 && g > 90 && g < 120 && b > 110 && b < 170 && c > 250 && c < 350)
  {
    Serial.println("Blue");
  }
  else if (r > 50 && r < 69 && g > 95 && g < 150 && b > 80 && b < 130 && c > 250 && c < 400)
  {
    Serial.println("Green");
  }
 else if (r > 70 && r < 95 && g > 95 && g < 200 && b > 100 && b < 200 && c > 275 && c < 350)
  {
    Serial.println("Carpet");
  }

  else if (r > 50 && r < 110 && g > 105 && g < 120 && b > 70 && b < 90 && c > 300 && c < 350)
  {
    Serial.println("Black");
  } 

  else if (r > 350 && r < 400 && g > 400 && g < 450 && b > 350 && b < 500 && c > 1000 && c < 1500)
  {
    Serial.println("White");
  }
  /*else{
    Serial.println("IDK");
  }*/

}
//good one