#include <Arduino.h>
#include <librobus.h>
bool IsEnable = true;
void setup(){
  BoardInit();
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(1,120);
  SERVO_SetAngle(0,110);
  delay(500);
  SERVO_Disable(0);
  SERVO_Disable(1);
  }
void CupKiller(){
  SERVO_Enable(0);
  SERVO_SetAngle(0,180);
  delay(5000);
  SERVO_SetAngle(0,95);
  delay(500);
  SERVO_Disable(0);
}

void CaptureBall(){
  SERVO_Enable(1);
  SERVO_SetAngle(1,0);
  delay(1000);
  SERVO_Disable(1);
}

 void loop(){ 
  delay(1000);
  if (IsEnable == true){
    CupKiller();
    delay(3000);
    CaptureBall();
  IsEnable = false;
  }

 }
