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
/*
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    Serial.println(" ");
    
  
       // Add a delay to control the update rate (in milliseconds)
    delay(100);
  */
  
  if (r > 70 && r < 90 && g > 65 && g < 75 && b > 65 && b < 80 && c > 230 && c < 250){
    Serial.println("Red");
  }
  else if (r > 145 && r < 160 && g > 140 && g < 150 && b > 85 && b < 105 && c > 400 && c < 445)
  {
    Serial.println("Yellow");
  }
  else if (r > 40 && r < 50 && g > 70 && g < 80 && b > 70 && b < 90 && c > 200 && c < 240)
  {
    Serial.println("Blue");
  }
  
  else if (r > 45 && r < 60 && g > 75 && g < 90 && b > 65 && b < 90 && c > 220 && c < 245)
  {
    Serial.println("Green");
  }
  
  else if (r > 175 && r < 190 && g > 185 && g < 200 && b > 165 && b < 180 && c > 540 && c < 590)
  {
    Serial.println("White");
  }

  else if (r > 55 && r < 70 && g > 75 && g < 85 && b > 75 && b < 85 && c > 225 && c < 255)
  {
    Serial.println("Carpet");
  }
  

}