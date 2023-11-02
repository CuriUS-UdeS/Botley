#include <Arduino.h>
#include <librobus.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

//color 
uint16_t r, g, b, c;

struct readings{
  int R = 0;
  int G = 0;
  int B = 0;
  int C = 0;
};

struct values {
  int Max = 0;
  int Min = 32000;
};

struct calibrationColor{
  values R;
  values G;
  values B;
  values C;
};

struct sensorColor {
  readings          Values;
  calibrationColor  Red;
  calibrationColor  Yellow;
  calibrationColor  Green;
  calibrationColor  Blue;
  calibrationColor  White;
};

sensorColor I2C;

const int calRed    = 1;
const int calYellow = 2;
const int calGreen  = 3;
const int calBlue   = 4;
const int calWhite  = 5;

const int precisionColor = 25;

//uint16_t r, g, b, c; //ancien programme

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
void initCalibrationValues(){
// Initialize the Red calibration values
    I2C.Red.R.Min = 32000;
    I2C.Red.R.Max = 0;
    I2C.Red.G.Min = 32000;
    I2C.Red.G.Max = 0;
    I2C.Red.B.Min = 32000;
    I2C.Red.B.Max = 0;
    I2C.Red.C.Min = 32000;
    I2C.Red.C.Max = 0;

    // Initialize the Yellow calibration values
    I2C.Yellow.R.Min = 32000;
    I2C.Yellow.R.Max = 0;
    I2C.Yellow.G.Min = 32000;
    I2C.Yellow.G.Max = 0;
    I2C.Yellow.B.Min = 32000;
    I2C.Yellow.B.Max = 0;
    I2C.Yellow.C.Min = 32000;
    I2C.Yellow.C.Max = 0;

    // Initialize the Green calibration values
    I2C.Green.R.Min = 32000;
    I2C.Green.R.Max = 0;
    I2C.Green.G.Min = 32000;
    I2C.Green.G.Max = 0;
    I2C.Green.B.Min = 32000;
    I2C.Green.B.Max = 0;
    I2C.Green.C.Min = 32000;
    I2C.Green.C.Max = 0;

    // Initialize the Blue calibration values
    I2C.Blue.R.Min = 32000;
    I2C.Blue.R.Max = 0;
    I2C.Blue.G.Min = 32000;
    I2C.Blue.G.Max = 0;
    I2C.Blue.B.Min = 32000;
    I2C.Blue.B.Max = 0;
    I2C.Blue.C.Min = 32000;
    I2C.Blue.C.Max = 0;

    // Initialize the White calibration values
    I2C.White.R.Min = 32000;
    I2C.White.R.Max = 0;
    I2C.White.G.Min = 32000;
    I2C.White.G.Max = 0;
    I2C.White.B.Min = 32000;
    I2C.White.B.Max = 0;
    I2C.White.C.Min = 32000;
    I2C.White.C.Max = 0;
}

//@brief  readI2C: lit les valeurs R,G,B,C du capteur et les ecrit dans I2C.Values."R,G,B et C"
void readI2C(){
  //read values
  Serial.println("reading values");
  I2C.Values.R = tcs.read16(TCS34725_RDATAL);
  I2C.Values.G = tcs.read16(TCS34725_GDATAL);
  I2C.Values.B = tcs.read16(TCS34725_BDATAL);
  I2C.Values.C = tcs.read16(TCS34725_CDATAL);
}

void printMaxMin(int calibrationColor){  //@param calibrationColor : Red=1, Yellow=2, Green=3, Blue=4, White=5)
  if (calibrationColor == 1) {
    Serial.print("Red:");
    Serial.println("R: ");
    Serial.print("Min: ");
    Serial.println(I2C.Red.R.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Red.R.Max);
    
    Serial.print("G:");
    Serial.println("Min: ");
    Serial.print(I2C.Red.G.Min);
    Serial.println(", Max: ");
    Serial.print(I2C.Red.G.Max);
    
    Serial.print("B:");
    Serial.println("Min: ");
    Serial.print(I2C.Red.B.Min);
    Serial.println(", Max: ");
    Serial.print(I2C.Red.B.Max);
    
    Serial.print("C:");
    Serial.println("Min: ");
    Serial.print(I2C.Red.C.Min);
    Serial.println(", Max: ");
    Serial.print(I2C.Red.C.Max);
  }
  
  if (calibrationColor == 2) {
    Serial.println("Yellow:");
    Serial.print("R: ");
    Serial.print("Min: ");
    Serial.print(I2C.Yellow.R.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Yellow.R.Max);
    
    Serial.println("G:");
    Serial.print("Min: ");
    Serial.print(I2C.Yellow.G.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Yellow.G.Max);
    
    Serial.println("B:");
    Serial.print("Min: ");
    Serial.print(I2C.Yellow.B.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Yellow.B.Max);
    
    Serial.println("C:");
    Serial.print("Min: ");
    Serial.print(I2C.Yellow.C.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Yellow.C.Max);
  }

  if (calibrationColor == 3) {
    Serial.println("Green:");
    Serial.print("R: ");
    Serial.print("Min: ");
    Serial.print(I2C.Green.R.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Green.R.Max);
    
    Serial.println("G:");
    Serial.print("Min: ");
    Serial.print(I2C.Green.G.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Green.G.Max);
    
    Serial.println("B:");
    Serial.print("Min: ");
    Serial.print(I2C.Green.B.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Green.B.Max);
    
    Serial.println("C:");
    Serial.print("Min: ");
    Serial.print(I2C.Green.C.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Green.C.Max);
  }

  if (calibrationColor == 4) {
    Serial.println("Blue:");
    Serial.print("R: ");
    Serial.print("Min: ");
    Serial.print(I2C.Blue.R.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Blue.R.Max);
    
    Serial.println("G:");
    Serial.print("Min: ");
    Serial.print(I2C.Blue.G.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Blue.G.Max);
    
    Serial.println("B:");
    Serial.print("Min: ");
    Serial.print(I2C.Blue.B.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Blue.B.Max);
    
    Serial.println("C:");
    Serial.print("Min: ");
    Serial.print(I2C.Blue.C.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.Blue.C.Max);
  }
  
  if (calibrationColor == 5) {
    Serial.println("White:");
    Serial.print("R: ");
    Serial.print("Min: ");
    Serial.print(I2C.White.R.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.White.R.Max);
    
    Serial.println("G:");
    Serial.print("Min: ");
    Serial.print(I2C.White.G.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.White.G.Max);
    
    Serial.println("B:");
    Serial.print("Min: ");
    Serial.print(I2C.White.B.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.White.B.Max);
    
    Serial.println("C:");
    Serial.print("Min: ");
    Serial.print(I2C.White.C.Min);
    Serial.print(", Max: ");
    Serial.println(I2C.White.C.Max);
  }

}


void AutomaticCalibrationColor (int calibrationColor){ //@param calibrationColor : Red=1, Yellow=2, Green=3, Blue=4, White=5)
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // indicateur pour mode claibration
  if (calibrationColor >= 1 && calibrationColor <= 5){
    
    initCalibrationValues(); //Reset Min et Max

    for (int i = 0; i < 200; i++){
        Serial.println("je fait une boucle");
      readI2C();
      //Trouver Min et Max
      if (calibrationColor == 1){
      
        if (I2C.Values.R < I2C.Red.R.Min) {
          I2C.Red.R.Min = I2C.Values.R;
        }
        if (I2C.Values.R > I2C.Red.R.Max) {
          I2C.Red.R.Max = I2C.Values.R;
        }
        if (I2C.Values.G < I2C.Red.G.Min) {
          I2C.Red.G.Min = I2C.Values.G;
        }

        if (I2C.Values.G > I2C.Red.G.Max) {
          I2C.Red.G.Max = I2C.Values.G;
        }

        if (I2C.Values.B < I2C.Red.B.Min) {
          I2C.Red.B.Min = I2C.Values.B;
        }

        if (I2C.Values.B > I2C.Red.B.Max) {
         I2C.Red.B.Max = I2C.Values.B;
        }

        if (I2C.Values.C < I2C.Red.C.Min) {
          I2C.Red.C.Min = I2C.Values.C;
        }

        if (I2C.Values.C > I2C.Red.C.Max) {
          I2C.Red.C.Max = I2C.Values.C;
        }

      }
      if (calibrationColor == 2){
    if (I2C.Values.R < I2C.Yellow.R.Min) {
        I2C.Yellow.R.Min = I2C.Values.R;
    }
    if (I2C.Values.R > I2C.Yellow.R.Max) {
        I2C.Yellow.R.Max = I2C.Values.R;
    }
    if (I2C.Values.G < I2C.Yellow.G.Min) {
        I2C.Yellow.G.Min = I2C.Values.G;
    }

    if (I2C.Values.G > I2C.Yellow.G.Max) {
        I2C.Yellow.G.Max = I2C.Values.G;
    }

    if (I2C.Values.B < I2C.Yellow.B.Min) {
        I2C.Yellow.B.Min = I2C.Values.B;
    }

    if (I2C.Values.B > I2C.Yellow.B.Max) {
        I2C.Yellow.B.Max = I2C.Values.B;
    }

    if (I2C.Values.C < I2C.Yellow.C.Min) {
        I2C.Yellow.C.Min = I2C.Values.C;
    }

    if (I2C.Values.C > I2C.Yellow.C.Max) {
        I2C.Yellow.C.Max = I2C.Values.C;
    }
  }
      if (calibrationColor == 3){
    if (I2C.Values.R < I2C.Green.R.Min) {
        I2C.Green.R.Min = I2C.Values.R;
    }
    if (I2C.Values.R > I2C.Green.R.Max) {
        I2C.Green.R.Max = I2C.Values.R;
    }
    if (I2C.Values.G < I2C.Green.G.Min) {
        I2C.Green.G.Min = I2C.Values.G;
    }

    if (I2C.Values.G > I2C.Green.G.Max) {
        I2C.Green.G.Max = I2C.Values.G;
    }

    if (I2C.Values.B < I2C.Green.B.Min) {
        I2C.Green.B.Min = I2C.Values.B;
    }

    if (I2C.Values.B > I2C.Green.B.Max) {
        I2C.Green.B.Max = I2C.Values.B;
    }

    if (I2C.Values.C < I2C.Green.C.Min) {
        I2C.Green.C.Min = I2C.Values.C;
    }

    if (I2C.Values.C > I2C.Green.C.Max) {
        I2C.Green.C.Max = I2C.Values.C;
    }
  }
      if (calibrationColor == 4){
    if (I2C.Values.R < I2C.Blue.R.Min) {
        I2C.Blue.R.Min = I2C.Values.R;
    }
    if (I2C.Values.R > I2C.Blue.R.Max) {
        I2C.Blue.R.Max = I2C.Values.R;
    }
    if (I2C.Values.G < I2C.Blue.G.Min) {
        I2C.Blue.G.Min = I2C.Values.G;
    }

    if (I2C.Values.G > I2C.Blue.G.Max) {
        I2C.Blue.G.Max = I2C.Values.G;
    }

    if (I2C.Values.B < I2C.Blue.B.Min) {
        I2C.Blue.B.Min = I2C.Values.B;
    }

    if (I2C.Values.B > I2C.Blue.B.Max) {
        I2C.Blue.B.Max = I2C.Values.B;
    }

    if (I2C.Values.C < I2C.Blue.C.Min) {
        I2C.Blue.C.Min = I2C.Values.C;
    }

    if (I2C.Values.C > I2C.Blue.C.Max) {
        I2C.Blue.C.Max = I2C.Values.C;
    }
  }
      if (calibrationColor == 5){
    if (I2C.Values.R < I2C.White.R.Min) {
        I2C.White.R.Min = I2C.Values.R;
    }
    if (I2C.Values.R > I2C.White.R.Max) {
        I2C.White.R.Max = I2C.Values.R;
    }
    if (I2C.Values.G < I2C.White.G.Min) {
        I2C.White.G.Min = I2C.Values.G;
    }

    if (I2C.Values.G > I2C.White.G.Max) {
        I2C.White.G.Max = I2C.Values.G;
    }

    if (I2C.Values.B < I2C.White.B.Min) {
        I2C.White.B.Min = I2C.Values.B;
    }

    if (I2C.Values.B > I2C.White.B.Max) {
        I2C.White.B.Max = I2C.Values.B;
    }

    if (I2C.Values.C < I2C.White.C.Min) {
        I2C.White.C.Min = I2C.Values.C;
    }

    if (I2C.Values.C > I2C.White.C.Max) {
        I2C.White.C.Max = I2C.Values.C;
    }
  }
    delay(100);
    }
  }
    Serial.println("setting pin13 low");
  digitalWrite(13, LOW); //Fin de Calibration



}

void ManualCalibration (){

I2C.Red.R.Min = 100;  I2C.Red.R.Max = 100;
I2C.Red.G.Min = 100;  I2C.Red.G.Max = 100;
I2C.Red.B.Min = 100;  I2C.Red.B.Max = 100;
I2C.Red.C.Min = 100;  I2C.Red.C.Max = 100;

I2C.Yellow.R.Min = 100; I2C.Yellow.R.Max = 100;
I2C.Yellow.G.Min = 100; I2C.Yellow.G.Max = 100;
I2C.Yellow.B.Min = 100; I2C.Yellow.B.Max = 100;
I2C.Yellow.C.Min = 100; I2C.Yellow.C.Max = 100;

I2C.Green.R.Min = 100;  I2C.Green.R.Max = 100;
I2C.Green.G.Min = 100;  I2C.Green.G.Max = 100;
I2C.Green.B.Min = 100;  I2C.Green.B.Max = 100;
I2C.Green.C.Min = 100;  I2C.Green.C.Max = 100;

I2C.Blue.R.Min = 100; I2C.Blue.R.Max = 100;
I2C.Blue.G.Min = 100; I2C.Blue.G.Max = 100;
I2C.Blue.B.Min = 100; I2C.Blue.B.Max = 100;
I2C.Blue.C.Min = 100; I2C.Blue.C.Max = 100;

I2C.White.R.Min = 100;  I2C.White.R.Max = 100;
I2C.White.G.Min = 100;  I2C.White.G.Max = 100;
I2C.White.B.Min = 100;  I2C.White.B.Max = 100;
I2C.White.C.Min = 100;  I2C.White.C.Max = 100;

}

void findColor (){
  readI2C();
 
  if (I2C.Red.R.Min - precisionColor > I2C.Values.R && I2C.Values.R < I2C.Red.R.Max + precisionColor &&
      I2C.Red.G.Min - precisionColor > I2C.Values.G && I2C.Values.G < I2C.Red.G.Max + precisionColor &&
      I2C.Red.B.Min - precisionColor > I2C.Values.B && I2C.Values.B < I2C.Red.B.Max + precisionColor &&
      I2C.Red.C.Min - precisionColor > I2C.Values.C && I2C.Values.C < I2C.Red.C.Max + precisionColor) {
    
    Serial.println("Red");
  }

  if (I2C.Yellow.R.Min - precisionColor > I2C.Values.R && I2C.Values.R < I2C.Yellow.R.Max + precisionColor &&
      I2C.Yellow.G.Min - precisionColor > I2C.Values.G && I2C.Values.G < I2C.Yellow.G.Max + precisionColor &&
      I2C.Yellow.B.Min - precisionColor > I2C.Values.B && I2C.Values.B < I2C.Yellow.B.Max + precisionColor &&
      I2C.Yellow.C.Min - precisionColor > I2C.Values.C && I2C.Values.C < I2C.Yellow.C.Max + precisionColor) {
    
    Serial.println("Yellow");
  }

  if (I2C.Green.R.Min - precisionColor > I2C.Values.R && I2C.Values.R < I2C.Green.R.Max + precisionColor &&
      I2C.Green.G.Min - precisionColor > I2C.Values.G && I2C.Values.G < I2C.Green.G.Max + precisionColor &&
      I2C.Green.B.Min - precisionColor > I2C.Values.B && I2C.Values.B < I2C.Green.B.Max + precisionColor &&
      I2C.Green.C.Min - precisionColor > I2C.Values.C && I2C.Values.C < I2C.Green.C.Max + precisionColor) {
      
      Serial.println("Green");
  }

  if (I2C.Blue.R.Min - precisionColor > I2C.Values.R && I2C.Values.R < I2C.Blue.R.Max + precisionColor &&
      I2C.Blue.G.Min - precisionColor > I2C.Values.G && I2C.Values.G < I2C.Blue.G.Max + precisionColor &&
      I2C.Blue.B.Min - precisionColor > I2C.Values.B && I2C.Values.B < I2C.Blue.B.Max + precisionColor &&
      I2C.Blue.C.Min - precisionColor > I2C.Values.C && I2C.Values.C < I2C.Blue.C.Max + precisionColor) {
      
      Serial.println("Blue");
  }

  if (I2C.White.R.Min - precisionColor > I2C.Values.R && I2C.Values.R < I2C.White.R.Max + precisionColor &&
      I2C.White.G.Min - precisionColor > I2C.Values.G && I2C.Values.G < I2C.White.G.Max + precisionColor &&
      I2C.White.B.Min - precisionColor > I2C.Values.B && I2C.Values.B < I2C.White.B.Max + precisionColor &&
      I2C.White.C.Min - precisionColor > I2C.Values.C && I2C.Values.C < I2C.White.C.Max + precisionColor) {
      
      Serial.println("White");
  }
}
//@brief superCrazyCalibration : Finds and prints all the Min and Max RGBC for Red, Green, Blue and White, Made by CAD:P (yes had lil fun with this fonction oops)
void superCrazyCalibration (){
  Serial.print("Starting Calibration...");
  delay(2000);
  Serial.println("each calibration take approx 10 seconds + 5 seconds in between each colours, be ready"); 
  delay(3000);

  Serial.println("Calibrating Red in 5 seconds");
  delay(1000);
  Serial.print("..3");
  delay(1000);
  Serial.print("..2");
  delay(1000);
  Serial.print("..1");
  delay(1000);
  

  AutomaticCalibrationColor(calRed);
  Serial.println("AutomaticCalib Red finit");
  printMaxMin(calRed);

  Serial.println("Calibrating Yellow in 5 seconds");
  delay(2000);
  Serial.print("..3");
  delay(1000);
  Serial.print("..2");
  delay(1000);
  Serial.print("..1");
  delay(1000);

  AutomaticCalibrationColor(calYellow);
  printMaxMin(calYellow);
  
  Serial.println("Calibrating Green in 5 seconds");
  delay(2000);
  Serial.print("..3");
  delay(1000);
  Serial.print("..2");
  delay(1000);
  Serial.print("..1");
  delay(1000);

  AutomaticCalibrationColor(calGreen);
  printMaxMin(calGreen);

  Serial.println("Calibrating Blue in 5 seconds");
  delay(2000);
  Serial.print("..3");
  delay(1000);
  Serial.print("..2");
  delay(1000);
  Serial.print("..1");
  delay(1000);

  AutomaticCalibrationColor(calBlue);
  printMaxMin(calBlue);

  Serial.println("Calibrating White in 5 seconds");
  delay(2000);
  Serial.print("..3");
  delay(1000);
  Serial.print("..2");
  delay(1000);
  Serial.print("..1");
  delay(1000);

  AutomaticCalibrationColor(calWhite);
  printMaxMin(calWhite);

}



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

  
  //superCrazyCalibration(); //Finds all the values
}


void loop(void) {
  
  //ManualCalibration ();


//findColor();



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
    
/*
 //These values work when strapped directly to the motor'
  
  if (r > 90 && r < 155 && g > 60 && g < 100 && b > 50 && b < 100 && c > 250 && c < 360){
    Serial.println("Red");
  }
  else if (r > 200 && r < 330 && g > 200 && g < 300 && b > 100 && b < 200 && c > 500 && c < 900)
  {
    Serial.println("Yellow");
  }
   
  else if (r > 50 && r < 69 && g > 95 && g < 150 && b > 80 && b < 130 && c > 250 && c < 400)
  {
    Serial.println("Green");
  }

  else if (r > 45 && r < 70 && g > 90 && g < 120 && b > 110 && b < 170 && c > 250 && c < 350)
  {
    Serial.println("Blue");
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

  */
delay(400);
}
//good one