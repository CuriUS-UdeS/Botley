#include <Arduino.h>
#include <librobus.h>

const int kHzWhistlePin=A0; 


//tout pour fonction verre 
struct StructPourCup{
  const int Yellow  = 1;
  const int Green   = 0;
  float posInitialCupKiller = 25; 
  bool cupDetected = false;
};
StructPourCup cup;


bool DetectWhistle()
{
  bool isRunning = false;
  int A = analogRead(kHzWhistlePin); 
  if (A > 590)
    isRunning = true;
  return isRunning;
}

void RetractArm(){
  SERVO_Enable(0);
  SERVO_SetAngle(0,110);
}
void ExtendArm(bool RightorLeft){
  
  if(RightorLeft == 0){
    SERVO_Enable(0);
    SERVO_SetAngle(0, 0);
  }

  
  if(RightorLeft == 1){
    SERVO_Enable(0);
   SERVO_SetAngle(0, 180);
  }
  
}
void CupKiller (int color){//Color: green = 0 yellow = 1
  //faire distance pis cehck quand 50cm du verre 
// position relative du robot : xPosition

  int A = 1;
  //Check pour verre
  if (color == cup.Green){
    A = digitalRead(42);
  //sort le bras s'il y a un verre
  }
  if(color == cup.Yellow){
    A = digitalRead(40);
  }
  if (A == 0){
    cup.cupDetected = 1;
  }
  
  if (cup.cupDetected == 1){
    if (color == cup.Green){
      ExtendArm(cup.Green);//positions on ete guess alors pt a changer
      }
    if (color == cup.Yellow){
    ExtendArm(cup.Yellow);
    }
  }

return;
}

void setup() {
  BoardInit();
  RetractArm();
}
void loop(){
  CupKiller (cup.Yellow);
  if (DetectWhistle()){
    Serial.print("AaAaaaAaaAAA");

  }
}
