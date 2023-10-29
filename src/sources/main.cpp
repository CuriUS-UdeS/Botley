#include <Arduino.h>
#include <LibRobus.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <math.h>

#define WHEEL_DIAMETER  3
#define PULSE_PER_ROTATION 3200

#define GREEN_DISTANCE 200
#define YELLOW_DISTANCE 90
#define BLUE_DISTANCE 380

uint16_t r, g, b, c;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


const float KP = 0.02;
const float KI = 0.0;
const float KD = 0.002;
const float MAX_SPEED_IN_INCHES = 34.5;
const float MAX_SPEED = 0.5;

bool isRunning = true;

int actualStep = 1;
String actualColor;

float lastErrorLineFollowing = 0;
uint16_t lastPv = -1;

float pulseInDistance = M_PI*WHEEL_DIAMETER/PULSE_PER_ROTATION; // La distance en pouce fait avec un pulse

struct PIDValues {
    float kp;
    float ki;
    float kd;
    float dt;
    float ti;
    float error;
    float sp;
    float pv;
    float output;
    float integral;
};

PIDValues rightPIDValues = {0.6, 0.1};
PIDValues leftPIDValues = {0.6, 0.2};
PIDValues mainPIDValues;

void displayPIDValues(PIDValues values)
{
   /*
    Serial.print("Kp : ");
    Serial.print(values.kp, 5);
    Serial.print("\t Ki : ");
    Serial.print(values.ki, 5);
    Serial.print("\t Integral : ");
    Serial.print(values.integral, 5);
    Serial.print("\t Kd : ");
    Serial.print(values.kd, 5);
    Serial.print("\t Ti : ");
    Serial.print(values.ti, 5);
    Serial.print("\t dt : ");
    Serial.print(values.dt, 5);
    Serial.print("\t Sp : ");
    Serial.print(values.sp, 5);
    Serial.print("\t Pv : ");
   

    Serial.print(values.pv, 5);

    Serial.print("\t Error : ");
    Serial.print(values.error, 5);
    Serial.print("\t Output : ");
    Serial.print(values.output, 5);
    Serial.print("\t New speed : ");
    Serial.print(constrain((values.pv/MAX_SPEED_IN_INCHES + values.output/MAX_SPEED_IN_INCHES),0.1,0.6), 5);
    Serial.println();
    */
    
}

void turn360()
{
    uint8_t gauche = 0; // Moteur2
    uint8_t droite = 1; // Moteur1
    int rotation = 7740;
    bool turnCompleted = false;
 
    ENCODER_ReadReset(gauche);
    ENCODER_ReadReset(droite);
    
    while (!turnCompleted) {
        // Read current encoder positions
        float pCurrentd = ENCODER_Read(droite);
        float pCurrentg = ENCODER_Read(gauche);
        // Apply corrections to motor speeds
        MOTOR_SetSpeed(droite, -0.3);
        MOTOR_SetSpeed(gauche, 0.3);
        // Check if the robot has turned the desired angle
        if (fabs(pCurrentg) >= rotation && fabs(pCurrentd) >= rotation) {
            // Set the flag to indicate that the turn has been completed
            turnCompleted = true;
            
            // Stop the motors
            MOTOR_SetSpeed(droite, 0);
            MOTOR_SetSpeed(gauche, 0);
            
            Serial.println("Turn completed!");
        }
    }
  delay(10);
}

void turn60(uint8_t direction) {
  uint8_t gauche = 0; // Moteur2
  uint8_t droite = 1; // Moteur1
  int rotation = 1260;
  bool turnCompleted = false;

  ENCODER_ReadReset(gauche);
  ENCODER_ReadReset(droite);

  Serial.println("Turning right !");
  
  while (!turnCompleted) {
      // Read current encoder positions
      float pCurrentd = ENCODER_Read(droite);
      float pCurrentg = ENCODER_Read(gauche);

      // Apply corrections to motor speeds

      if(direction == 0) {
        MOTOR_SetSpeed(droite, 0.3);
        MOTOR_SetSpeed(gauche, -0.3);
      } else {
        MOTOR_SetSpeed(droite, -0.3);
        MOTOR_SetSpeed(gauche, 0.3);
      }
      
      // Check if the robot has turned the desired angle
      if (fabs(pCurrentg) >= rotation && fabs(pCurrentd) >= rotation) {
          // Set the flag to indicate that the turn has been completed
          turnCompleted = true;
          
          // Stop the motors
          MOTOR_SetSpeed(droite, 0);
          MOTOR_SetSpeed(gauche, 0);
          
          Serial.println("Turn completed!");
      }
  }

  delay(10);
}

String detectColor() {
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
    
  
    if (r > 70 && r < 90 && g > 65 && g < 75 && b > 65 && b < 80 && c > 230 && c < 250){
        Serial.println("red");
        return "red";
    }
    else if (r > 145 && r < 160 && g > 140 && g < 150 && b > 85 && b < 105 && c > 400 && c < 445)
    {
        Serial.println("yellow");
        return "yellow";
    }
    else if (r > 40 && r < 50 && g > 70 && g < 80 && b > 70 && b < 90 && c > 200 && c < 240)
    {
        Serial.println("blue");
        return "blue";
    }
    
    else if (r > 45 && r < 60 && g > 75 && g < 90 && b > 65 && b < 90 && c > 220 && c < 245)
    {
        Serial.println("green");
        return "green";
    }
    
    else if (r > 175 && r < 190 && g > 185 && g < 200 && b > 165 && b < 180 && c > 540 && c < 590)
    {
        Serial.println("white");
        return "white";
    }

    else if (r > 55 && r < 75 && g > 75 && g < 95 && b > 75 && b < 90 && c > 225 && c < 280)
    {
        Serial.println("carpet");
        return "carpet";
    }
    else {
        Serial.println("wth");
        return "wth";
    }
}

void turnArcGreen() {
    MOTOR_SetSpeed(1, MAX_SPEED - 0.17);
    MOTOR_SetSpeed(0, MAX_SPEED);

    if(detectColor() == "carpet" && actualStep == 2)
    {
        actualStep++;
    }
}

void turnArcYellow() {
    MOTOR_SetSpeed(1, MAX_SPEED - 0.1);
    MOTOR_SetSpeed(0, MAX_SPEED-0.01);

    if(detectColor() == "carpet" && actualStep == 2)
    {
        actualStep++;
    }
}

void followWall(float sp, float speed) {
    uint16_t pv = ROBUS_ReadIR(3);
    float error = sp - pv;
    float output = KP*error + KD*(error - lastErrorLineFollowing);

    Serial.print("Distance : ");
    Serial.println(pv);

    if (pv <= 25) {
        actualStep++;
    } else {
        MOTOR_SetSpeed(0, speed + output);
        MOTOR_SetSpeed(1, speed - output);
    }
    lastErrorLineFollowing = error;
}

float getLeftSpeed() {
    static float past = 0.0;
    float speed = 0.0;
    static float oldPulse = 0.0;
    float present = micros();
    float actualPulse = ENCODER_Read(0);
    speed = 1000000.0 * pulseInDistance*float(actualPulse-oldPulse)/float(present - past);
    past = present;
    oldPulse = actualPulse;
    return speed;
}

float getRightSpeed() {
    static float past = 0.0;
    float speed = 0.0;
    static float oldPulse = 0.0;
    float present = micros();
    float actualPulse = ENCODER_Read(1);
    speed = 1000000.0 * pulseInDistance*float(actualPulse-oldPulse)/float(present - past);
    past = present;
    oldPulse = actualPulse;
    return speed;
}

void calculPID(PIDValues &values)
{
    values.dt = (millis() - values.ti)/1000;
    values.error = values.sp - values.pv;
    if(abs(values.error) < 1)
        values.integral += values.error * values.dt;
    if(abs(values.error) > 1)
        values.integral = 1;
    values.integral = constrain(values.integral, -10,10);
    values.output = values.kp*values.error + values.ki*values.integral;
    values.ti = millis();
}

void updatePIDMain(float speed) {
    float leftSpeed = getLeftSpeed();
    float rightSpeed = getRightSpeed();
}

void leftPID(float sp) {
    leftPIDValues.sp = sp;
    leftPIDValues.pv = getLeftSpeed();
    calculPID(leftPIDValues);
    MOTOR_SetSpeed(0, constrain((leftPIDValues.pv/MAX_SPEED_IN_INCHES + leftPIDValues.output/MAX_SPEED_IN_INCHES),0.1,0.6));
    //displayPIDValues(leftPIDValues);
}

void rightPID(float sp) {
    rightPIDValues.sp = sp;
    rightPIDValues.pv = getRightSpeed();
    calculPID(rightPIDValues);
    MOTOR_SetSpeed(1, constrain((rightPIDValues.pv/MAX_SPEED_IN_INCHES + rightPIDValues.output/MAX_SPEED_IN_INCHES),0.1,0.6));

}

float getDistanceTraveled(int encoder_channel) {
    return ENCODER_Read(encoder_channel) * pulseInDistance;
}

void goForward(float speed, float distance) {
    // Reset encoders
    ENCODER_Reset(0);
    ENCODER_Reset(1);

    float distanceTraveledLeft = 0;
    float distanceTraveledRight = 0;
    
    while ((distanceTraveledLeft + distanceTraveledRight) / 2 < distance) {
        distanceTraveledLeft = getDistanceTraveled(0);
        distanceTraveledRight = getDistanceTraveled(1);

        leftPID(speed);
        rightPID(speed);
    }

    // Stop the motors once the target distance is reached
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}

void initializeServos() {
    SERVO_Enable(0);
    SERVO_Enable(1);
    SERVO_SetAngle(1,160);
    SERVO_SetAngle(0,110);
    delay(500);
    SERVO_Disable(0);
    SERVO_Disable(1);
}

void deployArm(){
    SERVO_Enable(0);
    SERVO_SetAngle(0,180);
}

void retractArm(){
    SERVO_SetAngle(0,95);
    delay(500);
    SERVO_Disable(0);
}

void cupKiller() {
    deployArm();
    turn360();
    retractArm();
}

void captureBall(){
    SERVO_Enable(1);
    SERVO_SetAngle(1,0);
    delay(1000);
    SERVO_Disable(1);
}

void setup() {
    BoardInit();
    initializeServos();
    /*
    if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1); // halt!
    }
    */
}

void loop() {
    if(isRunning)
    {
        int Maxspeed = 17;
        float DistanceWanted = 80; 
        float acc = 0.1;
        float decel = 0.1;

        for (int i = 0; i <= (Maxspeed); i++)
        {
            goForward(i, acc);
   
        }

    // 1 boucle -- DistanceWanted - (Maxspeed*distanceWanted/16)

        goForward(Maxspeed,  DistanceWanted - ((acc + decel)*Maxspeed));


        for (int j = Maxspeed; j > -1; j--)
        {
            goForward(j, decel);
   
        }
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    
    isRunning = false;

    }
    /*
    MOTOR_SetSpeed(0,0.5);
    Serial.println(getLeftSpeed());
    */  
    /*
    actualColor = "yellow";

    //detectColor();
   switch (actualStep)
   {
    case 1:
        // Ici, on vérifie sur quelle couleur on se situe pour définir la bonne distance du mur
        if(actualColor == "green")
            followWall(GREEN_DISTANCE, MAX_SPEED);
        if(actualColor == "yellow")
            followWall(YELLOW_DISTANCE, MAX_SPEED);
    break;
    case 2:
        if(actualColor == "green")
            turnArcGreen();
        if(actualColor == "yellow")
            turnArcYellow();
    break;
    case 3:
        if(actualColor == "green")
            followWall(GREEN_DISTANCE, MAX_SPEED);
        if(actualColor == "yellow")
            followWall(YELLOW_DISTANCE, MAX_SPEED);
    break;
    case 4:
        if(actualColor == "green")
            turnArcGreen();
        if(actualColor == "yellow")
            turnArcYellow();
    break;
    case 5:
        if(actualColor == "green")
            followWall(GREEN_DISTANCE, MAX_SPEED);
        if(actualColor == "yellow")
            followWall(YELLOW_DISTANCE, MAX_SPEED);
    break;
    case 6:
        cupKiller();
    break;
    default:
    break;
   }
   */
   
    delay(100);
}
